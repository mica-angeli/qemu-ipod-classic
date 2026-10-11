/*
 * Apple iPod classic (6th generation), Samsung S5L8702 SoC.
 *
 * Milestone H1: CPU, RAM and the interrupt controllers, plus a -kernel style
 * boot path so a raw image (a Rockbox build, for now) can be dropped straight
 * into SDRAM without going through a bootrom we do not have. Every peripheral
 * that is not modelled yet is registered as an unimplemented device so that
 * -d unimp tells us exactly what the guest reaches for next.
 *
 * See wiki/02-s5l8702-hardware.md for where the addresses come from and
 * wiki/04-roadmap.md for what comes next.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/arm/boot.h"
#include "hw/loader.h"
#include "elf.h"
#include "exec/address-spaces.h"
#include "hw/misc/unimp.h"
#include "hw/qdev-properties.h"
#include "sysemu/blockdev.h"
#include "hw/irq.h"
#include "sysemu/sysemu.h"
#include "sysemu/reset.h"
#include "hw/qdev-clock.h"
#include "hw/arm/exynos4210.h"
#include "hw/dma/pl080.h"
#include "hw/arm/ipod_classic/ipod_classic.h"
#include "hw/arm/ipod_classic/ipod_classic_cs42l55.h"
#include "hw/arm/ipod_classic/ipod_classic_hle_audio.h"
#include "hw/arm/ipod_classic/ipod_classic_hle_np.h"
#include "hw/arm/ipod_classic/ipod_classic_np_overlay.h"
#include "hw/arm/ipod_classic/ipod_classic_timer.h"
/* The S5L8720's SHA-1 block; the S5L8702 drives the same register map. */
#include "hw/arm/ipod_touch_sha1.h"
#include "hw/sysbus.h"
#include "target/arm/cpregs.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "semihosting/semihost.h"

static MemoryRegion *allocate_ram(MemoryRegion *top, const char *name,
                                  uint32_t addr, uint32_t size)
{
    MemoryRegion *sec = g_new(MemoryRegion, 1);
    memory_region_init_ram(sec, NULL, name, size, &error_fatal);
    memory_region_add_subregion(top, addr, sec);
    return sec;
}

static void ipod_classic_cpu_setup(MachineState *machine, MemoryRegion **sysmem,
                                   ARMCPU **cpu, AddressSpace **nsas)
{
    Object *cpuobj = object_new(machine->cpu_type);
    *cpu = ARM_CPU(cpuobj);
    CPUState *cs = CPU(*cpu);

    *sysmem = get_system_memory();

    object_property_set_link(cpuobj, "memory", OBJECT(*sysmem), &error_abort);
    object_property_set_bool(cpuobj, "realized", true, &error_fatal);

    *nsas = cpu_get_address_space(cs, ARMASIdx_NS);

    object_unref(cpuobj);
}

/*
 * Reproduce what the bootrom leaves in the top of IRAM1 before it jumps into
 * the OS image. See the S5L8702_SYSINFO_* comments in the machine header for
 * the disassembly each field comes from.
 *
 * Written on every reset, not just at startup, because it lives in RAM the
 * guest is free to overwrite and because a guest-requested reboot (SWRCON)
 * has to come back to a machine that looks freshly powered on.
 */
static void ipod_classic_write_bootinfo(IPodClassicMachineState *nms)
{
    uint32_t sysinfo[S5L8702_SYSINFO_SIZE / 4] = {
        [S5L8702_SYSINFO_HW_ID_OFF / 4]     = cpu_to_le32(S5L8702_HW_ID),
        [S5L8702_SYSINFO_DRAM_SIZE_OFF / 4] = cpu_to_le32(SDRAM_MEM_SIZE),
        [S5L8702_SYSINFO_DRAM_BASE_OFF / 4] = cpu_to_le32(SDRAM_MEM_BASE),
    };
    uint32_t tag = cpu_to_le32(S5L8702_SYSINFO_MAGIC);
    uint32_t ptr = cpu_to_le32(S5L8702_SYSINFO_BASE);
    uint8_t zero[0x100] = { 0 };

    cpu_physical_memory_write(S5L8702_BOOTINFO_BASE, zero, sizeof(zero));
    if (nms->sysinfo_path[0] != '\0') {
        /* A block taken from a real device replaces the synthesised one. */
        uint8_t *data = NULL;
        gsize size;

        if (!g_file_get_contents(nms->sysinfo_path, (char **)&data, &size,
                                 NULL) || size != S5L8702_SYSINFO_SIZE) {
            error_report("sysinfo \"%s\" must be readable and exactly 0x%x "
                         "bytes", nms->sysinfo_path, S5L8702_SYSINFO_SIZE);
            exit(1);
        }
        cpu_physical_memory_write(S5L8702_SYSINFO_BASE, data, size);
        g_free(data);
    } else {
        cpu_physical_memory_write(S5L8702_SYSINFO_BASE, sysinfo,
                                  sizeof(sysinfo));
    }
    cpu_physical_memory_write(S5L8702_BOOTINFO_TAG, &tag, sizeof(tag));
    cpu_physical_memory_write(S5L8702_BOOTINFO_PTR, &ptr, sizeof(ptr));
}

static void ipod_classic_cpu_reset(void *opaque)
{
    IPodClassicMachineState *nms = IPOD_CLASSIC_MACHINE((MachineState *)opaque);
    CPUState *cs = CPU(nms->cpu);

    cpu_reset(cs);
    cpu_set_pc(cs, nms->reset_pc);
    ipod_classic_write_bootinfo(nms);
}

/*
 * The scratch stack the -kernel hand-off seeds: the top of IRAM0. crt0.S's
 * memory_init() runs on it for a handful of pushes and the app switches to its
 * own stacks the moment it returns, so nothing lives here for long. IRAM0's top
 * keeps it clear of the SYSINFO block, which sits at the top of IRAM1.
 */
#define S5L8702_HANDOFF_SP   (IRAM0_MEM_BASE + IRAM0_MEM_SIZE)

static void ipod_classic_poke32(hwaddr addr, uint32_t value)
{
    uint32_t le = cpu_to_le32(value);
    cpu_physical_memory_write(addr, &le, sizeof(le));
}

/*
 * Reproduce the state the bootrom + Rockbox/emCORE bootloader leave behind when
 * they hand control to a raw OS image, for the -kernel path that skips them.
 * Two things the firmware relies on that a cold ARM reset does not provide:
 *
 *  - A valid stack pointer. Rockbox's crt0.S (non-BOOTLOADER) calls
 *    memory_init() -- whose first instruction is "push {r4,lr}" -- BEFORE it
 *    sets up its own stacks (firmware/target/arm/s5l8702/crt0.S:143 vs :184), so
 *    it runs on whatever SP the bootloader left in SVC mode. With SP=0 from
 *    reset that push writes to 0xfffffff8 and takes an unhandled data abort into
 *    a still-zero vector table -- MEASURED: pc then walks 0x10,0x14,... The
 *    retailOS is unaffected because it sets its own SP before pushing anything
 *    (it boots today with SP=0, which would fault the same way if it did not).
 *
 *  - A running microsecond timer. USEC_TIMER == TECNT of timer E, and udelay()
 *    spins on it with no timeout (system-target.h). The only thing that starts
 *    timer E is usec_timer_init(), which is #ifdef BOOTLOADER
 *    (clocking-s5l8702.c:381), so the app never starts it and the first I2C
 *    transfer in pmu_preinit() -- udelay(5) -- would hang. We program timer E
 *    exactly as usec_timer_init() does: source ECLK/1, prescaler 11 so
 *    12 MHz / 12 = 1 MHz, DATA0 = ~0, free-running, CLR|EN. The retailOS drives
 *    timer E at 1 MHz itself and re-programs it (with CLR) before use, so this
 *    is transparent to it -- confirmed by navtest.sh.
 *
 * Skipped when a real bootrom is supplied: the chain would then do this itself.
 * Done from the machine reset AFTER qemu_devices_reset() so the timer device's
 * own reset does not undo the timer program -- see ipod_classic_machine_reset().
 */
static void ipod_classic_handoff(IPodClassicMachineState *nms)
{
    if (nms->bootrom_path[0] != '\0') {
        return;
    }

    /* The SVC-mode stack the bootloader would have left in r13. */
    nms->cpu->env.regs[13] = S5L8702_HANDOFF_SP;

    /* Timer E at 1 MHz, the register-for-register image of usec_timer_init(). */
    ipod_classic_poke32(TIMER_MEM_BASE + 0xA0, (4u << 8) | (1u << 6)); /* TECON  */
    ipod_classic_poke32(TIMER_MEM_BASE + 0xB0, (12000000u / 1000000u) - 1u);
                                                                      /* TEPRE  */
    ipod_classic_poke32(TIMER_MEM_BASE + 0xA8, 0xffffffffu);          /* TEDATA0 */
    ipod_classic_poke32(TIMER_MEM_BASE + 0xA4, (1u << 1) | (1u << 0));/* TECMD  */
}

static void ipod_classic_machine_reset(MachineState *machine,
                                       ShutdownCause reason)
{
    /*
     * Reset every device the normal way first -- that includes the timer's own
     * reset -- and only THEN reproduce the bootloader hand-off, so the timer we
     * program does not get wiped a moment later. The sysbus cold-reset handler
     * is registered by qdev_machine_creation_done(), i.e. after this machine's
     * init has registered ipod_classic_cpu_reset(), so it would otherwise run
     * last. This override is the one place guaranteed to run after all of them.
     */
    qemu_devices_reset(reason);
    ipod_classic_handoff(IPOD_CLASSIC_MACHINE(machine));
}

static void ipod_classic_memory_setup(MachineState *machine,
                                      MemoryRegion *sysmem, AddressSpace *nsas)
{
    IPodClassicMachineState *nms = IPOD_CLASSIC_MACHINE(machine);

    MemoryRegion *iram0;
    MemoryRegion *sdram;
    MemoryRegion *sdram_uncached;

    sdram = allocate_ram(sysmem, "sdram", SDRAM_MEM_BASE, SDRAM_MEM_SIZE);

    /*
     * DRAM ALSO ANSWERS AT 0x88000000, and this is not decoration.
     *
     * The SoC exposes the same DRAM twice, cached low and uncached with bit 31
     * set, and the firmware uses both at once: the note on DMAC0 below already
     * says the guest programs a channel with the cached alias (0x0be55320)
     * while the CPU runs on the uncached one (0x8be55320).
     *
     * MEASURED with SHA1TRACE=1: the SHA-1 engine is handed 0x8ba718c0 and
     * 0x8ba7192c, which are 0x88000000 + 0x03a718c0 and + 0x03a7192c -- offsets
     * comfortably inside 64 MB, i.e. the uncached view of 0x0ba718c0. With
     * nothing mapped there, cpu_physical_memory_read returned ZEROS and the
     * engine hashed nothing at all. Any digest the firmware compared against
     * was therefore guaranteed to mismatch, which is what emptied the music
     * library the moment the accelerator was wired up.
     */
    sdram_uncached = g_new(MemoryRegion, 1);
    memory_region_init_alias(sdram_uncached, NULL, "sdram-uncached", sdram, 0,
                             SDRAM_MEM_SIZE);
    memory_region_add_subregion(sysmem, SDRAM_UNCACHED_MEM_BASE,
                                sdram_uncached);

    iram0 = allocate_ram(sysmem, "iram0", IRAM0_MEM_BASE, IRAM0_MEM_SIZE);
    allocate_ram(sysmem, "iram1", IRAM1_MEM_BASE, IRAM1_MEM_SIZE);

    /*
     * IRAMSPILL=N: RAM immediately above IRAM1, for MEASUREMENT ONLY.
     *
     * The render's halfword copy at 0x08016f08 walks its destination straight
     * off the top of IRAM1 -- caught at the data-abort vector with
     * r2 = 0x22040a16, i.e. 0xa16 past the end -- and that single abort is what
     * becomes CPU_exception_handler and the software reset that cuts playback
     * short. Whether the ring it is filling is bigger than 128 KB on real
     * hardware, or the pointer should have wrapped and did not, is UNDETERMINED.
     * Backing the region lets the run continue so the overrun can be measured:
     * where the writes actually stop says how big the firmware believes the ring
     * is. It is not a fix and must not be left on.
     */
    if (getenv("IRAMSPILL")) {
        unsigned long n = strtoul(getenv("IRAMSPILL"), NULL, 0);
        if (n) {
            allocate_ram(sysmem, "iram-spill",
                         IRAM1_MEM_BASE + IRAM1_MEM_SIZE, (uint32_t)n);
            fprintf(stderr, "IRAMSPILL: %lu bytes mapped above IRAM1 "
                            "(measurement aid, not a fix)\n", n);
        }
    }

    allocate_ram(sysmem, "edgeic", EDGEIC_MEM_BASE, EDGEIC_MEM_SIZE);

    nms->reset_pc = SDRAM_MEM_BASE;

    /*
     * IRAM0 also appears at address 0, which is where the ARM926 takes its
     * exceptions from. Rockbox's crt0.S is explicit about it -- "copy vector
     * section to start of IRAM0 (it will be mapped to 0x0)" and "we do not
     * want to execute from 0x0 as iram will be mapped there".
     *
     * What we do not know yet is whether the bootrom sits there first and the
     * alias only appears after a remap, the way the S5L8720 works. Until a
     * bootrom dump settles it, a supplied bootrom takes the address instead.
     */
    if (nms->bootrom_path[0] == '\0') {
        MemoryRegion *alias = g_new(MemoryRegion, 1);
        memory_region_init_alias(alias, NULL, "iram0-alias", iram0, 0,
                                 IRAM0_MEM_SIZE);
        memory_region_add_subregion(sysmem, 0, alias);
    }

    if (nms->bootrom_path[0] != '\0') {
        uint8_t *data = NULL;
        gsize size;

        if (!g_file_get_contents(nms->bootrom_path, (char **)&data, &size,
                                 NULL)) {
            error_report("could not read bootrom \"%s\"", nms->bootrom_path);
            exit(1);
        }
        if (size > BOOTROM_MEM_SIZE) {
            error_report("bootrom is 0x%zx bytes, the window at 0x%x is only "
                         "0x%x", (size_t)size, (unsigned)BOOTROM_MEM_BASE,
                         (unsigned)BOOTROM_MEM_SIZE);
            exit(1);
        }
        allocate_ram(sysmem, "bootrom", BOOTROM_MEM_BASE, BOOTROM_MEM_SIZE);
        address_space_rw(nsas, BOOTROM_MEM_BASE, MEMTXATTRS_UNSPECIFIED, data,
                         size, 1);
        g_free(data);
        nms->reset_pc = BOOTROM_MEM_BASE;
    }

    /*
     * -kernel takes either an ELF or a raw binary, and wins over the bootrom
     * so an image can be booted directly while a bootrom is still supplied for
     * whatever it may map.
     *
     * An ELF carries its own load addresses and entry point, which matters
     * because the two things we want to boot land in different places: a
     * Rockbox bootloader runs from IRAM1 at 0x22020800, while the osos image
     * of a retail firmware loads at 0x08000000 (addr in the firmware
     * directory, see wiki/01-firmware-ipsw.md). A raw binary has no such
     * information, so it goes to SDRAM.
     */
    if (machine->kernel_filename) {
        uint64_t entry = 0, lowaddr = 0, highaddr = 0;
        ssize_t size;

        size = load_elf(machine->kernel_filename, NULL, NULL, NULL, &entry,
                        &lowaddr, &highaddr, NULL, 0, EM_ARM, 1, 0);
        if (size > 0) {
            info_report("loaded ELF \"%s\": 0x%" PRIx64 "-0x%" PRIx64
                        ", entry 0x%" PRIx64, machine->kernel_filename,
                        lowaddr, highaddr, entry);
            nms->reset_pc = entry;
        } else {
            size = load_image_targphys(machine->kernel_filename,
                                       SDRAM_MEM_BASE, SDRAM_MEM_SIZE);
            if (size < 0) {
                error_report("could not load \"%s\" at 0x%x",
                             machine->kernel_filename,
                             (unsigned)SDRAM_MEM_BASE);
                exit(1);
            }
            info_report("loaded 0x%zx raw bytes at 0x%x", (size_t)size,
                        (unsigned)SDRAM_MEM_BASE);
            nms->reset_pc = SDRAM_MEM_BASE;
        }
    }
}

static char *ipod_classic_get_bootrom_path(Object *obj, Error **errp)
{
    IPodClassicMachineState *nms = IPOD_CLASSIC_MACHINE(obj);
    return g_strdup(nms->bootrom_path);
}

static void ipod_classic_set_bootrom_path(Object *obj, const char *value,
                                          Error **errp)
{
    IPodClassicMachineState *nms = IPOD_CLASSIC_MACHINE(obj);

    if (!g_file_test(value, G_FILE_TEST_EXISTS)) {
        error_report("bootrom at path \"%s\" must exist", value);
        exit(1);
    }
    g_strlcpy(nms->bootrom_path, value, sizeof(nms->bootrom_path));
}

static char *ipod_classic_get_sysinfo_path(Object *obj, Error **errp)
{
    IPodClassicMachineState *nms = IPOD_CLASSIC_MACHINE(obj);
    return g_strdup(nms->sysinfo_path);
}

static void ipod_classic_set_sysinfo_path(Object *obj, const char *value,
                                          Error **errp)
{
    IPodClassicMachineState *nms = IPOD_CLASSIC_MACHINE(obj);

    if (!g_file_test(value, G_FILE_TEST_EXISTS)) {
        error_report("sysinfo at path \"%s\" must exist", value);
        exit(1);
    }
    g_strlcpy(nms->sysinfo_path, value, sizeof(nms->sysinfo_path));
}

static bool ipod_classic_get_semihosting(Object *obj, Error **errp)
{
    return IPOD_CLASSIC_MACHINE(obj)->semihosting;
}

static void ipod_classic_set_semihosting(Object *obj, bool value, Error **errp)
{
    IPOD_CLASSIC_MACHINE(obj)->semihosting = value;
}

static void ipod_classic_instance_init(Object *obj)
{
    object_property_add_str(obj, "bootrom", ipod_classic_get_bootrom_path,
                            ipod_classic_set_bootrom_path);
    object_property_set_description(obj, "bootrom",
                                    "Path to the S5L8702 bootrom binary");
    object_property_add_str(obj, "sysinfo", ipod_classic_get_sysinfo_path,
                            ipod_classic_set_sysinfo_path);
    object_property_set_description(obj, "sysinfo",
        "Path to a 0x120-byte SysInfo block from a real device, handed to "
        "the OS instead of the synthesised 6G one. Firmware 2.0.x needs a "
        "hardware id of 0x00130300 or later");

    /*
     * Apple's retailOS is built against ARM's own C library with the
     * semihosting retarget left in: 131 "svc 0x123456" sites in the image,
     * 115 of them preceded by "mov r0, #4", which is SYS_WRITE0. It also ships
     * RTXCbug, the RTXC kernel's interactive debugger, whose prompt strings
     * ("** RTXCbug - ", "RTXCbug> ", "Bad command") sit at 0x0827bd79 and are
     * printed the same way. With semihosting off those SVCs go to a vector the
     * firmware may not have installed and the machine dies quietly; with it on
     * the firmware tells us what it thinks is wrong. Costs nothing when
     * nothing is printed, so it is on by default.
     */
    IPOD_CLASSIC_MACHINE(obj)->semihosting = true;
    object_property_add_bool(obj, "semihosting", ipod_classic_get_semihosting,
                             ipod_classic_set_semihosting);
    object_property_set_description(obj, "semihosting",
        "Answer ARM semihosting calls (SVC 0x123456). Apple's retailOS prints "
        "its diagnostics through them. Turn it off to see the firmware take "
        "the SVC vector instead.");
}

static inline qemu_irq s5l8702_get_irq(IPodClassicMachineState *s, int n)
{
    return s->irq[n / S5L8702_VIC_SIZE][n % S5L8702_VIC_SIZE];
}

/*
 * Peripherals we know the address of but have not modelled. Registering them
 * keeps stray accesses out of the "unassigned memory" path and makes -d unimp
 * report which block the guest is talking to.
 */
static const struct { const char *name; hwaddr base; hwaddr size; }
ipod_classic_unimplemented[] = {
    /* s5l8702.sha1 (0x38000000) is a real device now; see the machine init. */
    { "s5l8702.miu",      MIU_MEM_BASE,      0x100000 },
    /* DMAC0 and DMAC1 are real PL080s now; see the machine init. */
    { "s5l8702.usb-otg",  USBOTG_MEM_BASE,   0x100000 },
    /* s5l8702.sm1 (0x38500000) is a real (minimal) device now; see below. */
    { "s5l8702.usb-hc",   USB_HC_MEM_BASE,   0x100000 },
    { "s5l8702.usb-fc",   USB_FC_MEM_BASE,   0x100000 },
    { "s5l8702.ata-unk",  ATA_UNK_MEM_BASE,  0x100000 },
    { "s5l8702.sdci",     SDCI_MEM_BASE,     0x100000 },
    { "s5l8702.aes",      AES_MEM_BASE,      0x100000 },
    { "s5l8702.adm-sfr",  ADM_SFR_MEM_BASE,  0x100000 },
    { "s5l8702.lcd-win",  LCD_WIN_MEM_BASE,  0x100000 },
    { "s5l8702.fmc",      FMC_MEM_BASE,      0x100000 },
    { "s5l8702.eic",      EIC_MEM_BASE,      0x100000 },
    { "s5l8702.int",      INT_MEM_BASE,      0x100000 },
    { "s5l8702.ecc",      ECC_MEM_BASE,      0x100000 },
    /* The click wheel is a real device now; see the machine init. */
    { "s5l8702.usb-phy",  USBPHYS_MEM_BASE,  0x100000 },
    { "s5l8702.wdt",      WDT_MEM_BASE,      0x100000 },
    /* s5l8702.i2s is a real device now; see the machine init. */
    { "s5l8702.spdif",    SPDIF_MEM_BASE,    0x100000 },
    { "s5l8702.spi0",     SPI0_MEM_BASE,     0x100000 },
    { "s5l8702.adc",      ADC_MEM_BASE,      0x100000 },
    { "s5l8702.chipid",   CHIPID_MEM_BASE,   0x100000 },
    { "s5l8702.rtc",      RTC_MEM_BASE,      0x100000 },
    { "s5l8702.adm",      ADM_MEM_BASE,      0x100000 },
};

/*
 * Everything in the peripheral window that is not one of the blocks above.
 * Same behaviour as create_unimplemented_device(), but one region for the
 * whole window and at a lower priority so the named blocks stay on top.
 */
static uint64_t ipod_classic_io_floor_read(void *opaque, hwaddr offset,
                                           unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "s5l8702.io: read  (size %d, addr 0x%08" PRIx64
                  ")\n", size, (uint64_t)(IO_WINDOW_BASE + offset));
    return 0;
}

static void ipod_classic_io_floor_write(void *opaque, hwaddr offset,
                                        uint64_t value, unsigned size)
{
    qemu_log_mask(LOG_UNIMP, "s5l8702.io: write (size %d, addr 0x%08" PRIx64
                  ", value 0x%" PRIx64 ")\n", size,
                  (uint64_t)(IO_WINDOW_BASE + offset), value);
}

static const MemoryRegionOps ipod_classic_io_floor_ops = {
    .read = ipod_classic_io_floor_read,
    .write = ipod_classic_io_floor_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 8,
};

/*
 * The block at 0x3C100000, which no source we have names.
 *
 * Rockbox does not know it exists on this chip: the one 0x3C100000 in
 * s5l87xx.h is LCD_BASE, and it sits under "#if CONFIG_CPU==S5L8700", so it
 * describes a different SoC. freemyipod's model has nothing there either.
 * Everything below is from Apple's own code.
 *
 * Exactly three instructions in the whole 9.9 MB image reference this base --
 * found by scanning for the literal word 0x3C100000 and then for every
 * pc-relative LDR that resolves to one of those words, so the list is complete
 * rather than merely what turned up:
 *
 *   0x08031d9c  str r0,[r1,#8]     seed  = sum of seven bytes from 0x08295294
 *                                        + TECNT + the word at 0x3C700008
 *   0x08327738  str #0,[r1]        writes ZERO to +0
 *   0x08327760  "while (!([+0] & 7)) ; return [+4]"
 *
 * The third addend is NOT a previous seed read back from this block, which is
 * what an earlier version of this comment said: the literal at 0x08031de0 is
 * 0x3C700000, i.e. TIMER_BASE, and +0x08 is TADATA0. The seed is stirred with
 * two independent timer reads, and nothing ever reads +4 or +8 of this block
 * back -- so the seed register here is write-only as far as the guest is
 * concerned.
 *
 * Read in context: 0x08327714 takes a critical section and then writes that
 * zero; 0x08031d30 seeds +8, draws twenty words through 0x08327760
 * discarding nineteen, and releases the section via 0x08327750. Drawing and
 * throwing away the first outputs of a freshly seeded generator is an RNG
 * idiom, and the seed is stirred with a free-running timer count (0x08015748
 * is a veneer to 0x22001EC4, which is "return TECNT"). Nothing else fits: no
 * fresh input is written between the twenty reads, so a hash or checksum unit
 * would return the same word each time and the loop would have no purpose.
 *
 * The identity of the block is therefore INFERRED, and its name and any other
 * registers are UNDETERMINED. The three-register contract above is VERIFIED.
 *
 * One consequence of that contract is worth spelling out, because getting it
 * wrong is a silent hang. There is no enable bit. No instruction anywhere
 * writes a non-zero value to +0; the only write is the zero at 0x08327738, and
 * it happens BEFORE the twenty reads, not after them. So "+0 reads ready only
 * once the guest has enabled the block" deadlocks on the very first draw --
 * 0x08327760 spins with no timeout and no escape. The ready bits have to be
 * set on their own, which is what the model does.
 *
 * The generator is a plain 32-bit LCG seeded from what the guest wrote, so a
 * given boot produces the same sequence twice. Reproducibility is worth more
 * here than unpredictability; the firmware only wants variety.
 *
 * Measured, so that nobody credits this block with more than it does: with a
 * disk attached the retailOS never touches 0x3C100000 at all -- 0x08327760
 * does not appear in a -d in_asm log of that boot. The block is only reached
 * on the diskless path, where it is called once per boot attempt, and the
 * seeding site 0x08031d9c never runs in either, so s->seed stays zero.
 */
#define S5L8702_RNG_MEM_BASE   0x3c100000
#define S5L8702_RNG_MEM_SIZE   0x10
#define S5L8702_RNG_STATUS     0x00   /* read: ready in [2:0]. Written with 0. */
#define S5L8702_RNG_DATA       0x04
#define S5L8702_RNG_SEED       0x08

typedef struct {
    uint32_t seed;
    uint32_t state;
} S5L8702RNGState;

static uint64_t ipod_classic_rng_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    S5L8702RNGState *s = opaque;

    switch (offset) {
    case S5L8702_RNG_STATUS:
        /*
         * 0x08327760 spins on "& 7" with no timeout, and nothing ever enables
         * the block, so a word is always available. Which of the three bits
         * means what is UNDETERMINED: the firmware only tests them together.
         */
        return 7;
    case S5L8702_RNG_DATA:
        s->state = s->state * 1103515245u + 12345u;
        return s->state;
    case S5L8702_RNG_SEED:
        return s->seed;
    default:
        qemu_log_mask(LOG_UNIMP, "s5l8702.rng: read of +0x%" HWADDR_PRIx "\n",
                      offset);
        return 0;
    }
}

static void ipod_classic_rng_write(void *opaque, hwaddr offset, uint64_t value,
                                   unsigned size)
{
    S5L8702RNGState *s = opaque;

    switch (offset) {
    case S5L8702_RNG_STATUS:
        /* Only ever written with zero, and readiness does not depend on it. */
        break;
    case S5L8702_RNG_SEED:
        s->seed = value;
        s->state = value;
        break;
    default:
        qemu_log_mask(LOG_UNIMP, "s5l8702.rng: write of 0x%" PRIx64
                      " to +0x%" HWADDR_PRIx "\n", value, offset);
        break;
    }
}

static const MemoryRegionOps ipod_classic_rng_ops = {
    .read = ipod_classic_rng_read,
    .write = ipod_classic_rng_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void ipod_classic_machine_init(MachineState *machine)
{
    IPodClassicMachineState *nms = IPOD_CLASSIC_MACHINE(machine);
    MemoryRegion *sysmem;
    AddressSpace *nsas;
    ARMCPU *cpu;
    DeviceState *dev;

    if (nms->semihosting) {
        qemu_semihosting_enable();
    }

    ipod_classic_cpu_setup(machine, &sysmem, &cpu, &nsas);
    nms->cpu = cpu;
    nms->nsas = nsas;

    nms->sysclk = clock_new(OBJECT(machine), "SYSCLK");
    clock_set_hz(nms->sysclk, 12000000ULL);

    ipod_classic_memory_setup(machine, sysmem, nsas);

    /* VIC0 drives the CPU, VIC1 daisy-chains into it. */
    nms->irq = g_malloc0(sizeof(qemu_irq *) * S5L8702_VIC_N);

    dev = pl192_manual_init("vic0",
                            qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_IRQ),
                            qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_FIQ), NULL);
    nms->vic0 = PL192(dev);
    memory_region_add_subregion(sysmem, VIC0_MEM_BASE, &nms->vic0->iomem);
    nms->irq[0] = g_malloc0(sizeof(qemu_irq) * S5L8702_VIC_SIZE);
    for (int i = 0; i < S5L8702_VIC_SIZE; i++) {
        nms->irq[0][i] = qdev_get_gpio_in(dev, i);
    }

    dev = pl192_manual_init("vic1", NULL);
    nms->vic1 = PL192(dev);
    memory_region_add_subregion(sysmem, VIC1_MEM_BASE, &nms->vic1->iomem);
    nms->irq[1] = g_malloc0(sizeof(qemu_irq) * S5L8702_VIC_SIZE);
    for (int i = 0; i < S5L8702_VIC_SIZE; i++) {
        nms->irq[1][i] = qdev_get_gpio_in(dev, i);
    }

    nms->vic1->daisy = nms->vic0;

    /* Clock and PLL block. */
    dev = qdev_new(TYPE_IPOD_CLASSIC_CLK);
    nms->clock0 = IPOD_CLASSIC_CLK(dev);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, CLOCK0_MEM_BASE);

    /*
     * Timers. Channels A..D share IRQ 8 and E..H share IRQ 7; Rockbox drives
     * B for the 10 kHz kernel tick and F for the user timer.
     */
    dev = qdev_new(TYPE_IPOD_CLASSIC_TIMER);
    nms->timer = IPOD_CLASSIC_TIMER(dev);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, TIMER_MEM_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       s5l8702_get_irq(nms, S5L8702_TIMER_IRQ));
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 1,
                       s5l8702_get_irq(nms, S5L8702_TIMER32_IRQ));

    /*
     * I2C0, and the PCF50635 power management unit on it. Apple's retailOS
     * talks to the PMU before it does anything else, and 0x08325a20 waits on
     * the result with a retry loop and no way out, so this is what the boot
     * hangs on when it is missing.
     */
    dev = qdev_new(TYPE_IPOD_CLASSIC_I2C);
    nms->i2c0 = IPOD_CLASSIC_I2C(dev);
    qdev_prop_set_uint32(dev, "reset-con", S5L8702_I2C_BOOTROM_CON);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, I2C0_MEM_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       s5l8702_get_irq(nms, S5L8702_I2C0_IRQ));
    i2c_slave_create_simple(nms->i2c0->bus, TYPE_IPOD_CLASSIC_PMU,
                            S5L8702_PMU_I2C_ADDR);

    /*
     * The CS42L55 audio codec, also on I2C0, at 7-bit 0x4A. The retailOS talks
     * to it when it brings up or starts the audio engine (osos cscodec_write at
     * 0x0808e1ac, and set_freq's cscodec_write(5, code)); while it did not exist
     * every write was NACKed and every read returned bus garbage, which is a
     * candidate for why selecting a song never engages playback. See
     * ipod_classic_cs42l55.c and wiki/14.
     */
    i2c_slave_create_simple(nms->i2c0->bus, TYPE_IPOD_CLASSIC_CS42L55,
                            S5L8702_CS42L55_I2C_ADDR);

    /*
     * The disk. -drive if=ide,file=... attaches one; with none the controller
     * still answers, which is what lets the retailOS find out there is no
     * disk instead of spinning on ATA_PIO_READY for ever.
     */
    dev = qdev_new(TYPE_IPOD_CLASSIC_ATA);
    nms->ata = IPOD_CLASSIC_ATA(dev);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, ATA_MEM_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       s5l8702_get_irq(nms, S5L8702_ATA_IRQ));
    ipod_classic_ata_init_drive(dev, drive_get(IF_IDE, 0, 0));

    /*
     * The LCD command port. Both drivers we have spin on its status bits with
     * no timeout, so an unimplemented window here is not a zero read: it is a
     * hang. See ipod_classic_lcd.c.
     */
    dev = qdev_new(TYPE_IPOD_CLASSIC_LCD);
    nms->lcd = IPOD_CLASSIC_LCD(dev);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, LCD_MEM_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       s5l8702_get_irq(nms, S5L8702_LCD_IRQ));

    /*
     * The two DMA controllers, and this is what unblocks a boot with a disk.
     *
     * They are PrimeCell PL080s. Two independent sources say so and they agree:
     * Rockbox names them ("2 PL080 DMA controllers" at 0x38200000 and
     * 0x39900000, firmware/target/arm/s5l8702/dma-s5l8702.h:33-36), and the
     * register offsets the retailOS actually drives match pl080.h to the byte
     * -- +0x04 INTTCSTS, +0x08 INTTCCLR, +0x30 CONFIG, channels at
     * base + 0x100 + (ch << 5).
     *
     * WHY IT MATTERS, measured: after the ATA driver's two DMA reads complete
     * -- and they do complete, the ATA interrupt fires and its RTXC semaphore
     * is signalled -- the caller does a memcpy through DMAC0 channel 0
     * (0x2200844c, 0x1000 bytes) and then sleeps on the channel's semaphore in
     * 0x22008600 with KS_wait and NO timeout. Only DMAC0's terminal-count ISR
     * (registered at 0x080357f4 on IRQ 16, entry 0x22002810) can wake it. With
     * the block unimplemented that interrupt never came, so every boot with a
     * disk parked the storage task for ever and the volume was never mounted.
     *
     * Two details that are easy to get wrong:
     *  - the region is 0x1000, not 0x100000, so the rest of the 1 MB block
     *    still falls to the io floor and -d unimp will say so if the guest
     *    reaches for it;
     *  - the master port has to be system memory, because the guest programs
     *    the channel with the CACHED alias (0x0be55320) while the CPU runs on
     *    the uncached one (0x8be55320). Both have to land on the same DRAM.
     */
    dev = qdev_new(TYPE_PL080);
    object_property_set_link(OBJECT(dev), "downstream", OBJECT(sysmem),
                             &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, DMAC0_MEM_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       s5l8702_get_irq(nms, S5L8702_DMAC0_IRQ));
    PL080State *dmac0 = PL080(dev);

    dev = qdev_new(TYPE_PL080);
    object_property_set_link(OBJECT(dev), "downstream", OBJECT(sysmem),
                             &error_fatal);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, DMAC1_MEM_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       s5l8702_get_irq(nms, S5L8702_DMAC1_IRQ));

    /*
     * The display pipe at 0x38900000. No external source names this block; the
     * whole register map in ipod_classic_disp.h comes from Apple's own code.
     * It holds the panel geometry (+0x214 = 0x013f00ef, i.e. 319 and 239), the
     * three layer descriptors, nine source-address slots and three 256-entry
     * gamma tables, and the retailOS programs 793 distinct addresses in it on
     * every boot attempt.
     *
     * It was create_unimplemented_device() until now, and that was not a
     * neutral choice: six functions in the osos read-modify-write +0x008 and
     * one reads +0x200 back, so a zero readback silently threw away every
     * field written before it. Honest readback is the first thing this device
     * buys, before any question of composing a frame.
     */
    dev = qdev_new(TYPE_IPOD_CLASSIC_DISP);
    nms->disp = IPOD_CLASSIC_DISP(dev);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, DISP_MEM_BASE);
    /*
     * And this is the wire that finally puts a picture on the screen. The
     * panel behind the LCD port holds the image, but the retailOS never sends
     * it a pixel: it composes the frame in this block from layers in DRAM. So
     * the panel asks the pipe for the frame when it is about to be shown. See
     * the note in ipod_classic_lcd_gfx_update().
     */
    nms->lcd->disp = nms->disp;

    /*
     * GPIO. Only one pin is modelled and it is the reason this device exists:
     * 0x080aeca4 reads pin 86 and the storage task turns it straight into
     * "mount the volume" or "there is no media" (see ipod_classic_gpio.h). With
     * the window unimplemented it read zero, the volume was never mounted, and
     * every boot WITH a disk parked the user-interface task in
     * "while (!ready) wait();" at 0x081228c4 -- which is why those boots
     * reached the RTXC idle loop and never drew anything.
     *
     * The pin follows whether a drive is attached, so a boot with no -drive is
     * unchanged.
     */
    dev = qdev_new(TYPE_IPOD_CLASSIC_GPIO);
    qdev_prop_set_bit(dev, "media-present", drive_get(IF_IDE, 0, 0) != NULL);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, GPIO_MEM_BASE);

    /*
     * The click wheel at 0x3C200000, and with it the keyboard. This is what
     * makes the user interface usable rather than merely visible.
     *
     * It is interrupt driven, not polled, and that is a measurement rather
     * than a preference: in a 60 s boot the guest writes WHEELTX exactly five
     * times -- one polled transaction with its five retries -- and then never
     * touches the block again. What it does instead is register 0x080479a8 on
     * IRQ 23 (0x08047a68: mov r1,#23) and park its wheel task on kernel event
     * 5 for ever. With no interrupt there is no input, however honestly the
     * registers read back.
     *
     * The line itself was checked end to end before a byte of this was
     * written, because the 5G taught us that raising an IRQ proves nothing
     * (hw/arm/ipod_video/ipod_video_intc.c, the HI_IRQ gate): VIC0 INTENABLE
     * reads 0x01A3010F with the machine running, bit 23 set, and the IRAM
     * dispatcher at 0x22004DA0 reads both banks with no cross-condition.
     *
     * Keys, the same as the iPod-Video machine: q/e turn the wheel, enter is
     * SELECT, w is MENU, a/d are previous/next, s is play. See the header.
     */
    dev = qdev_new(TYPE_IPOD_CLASSIC_WHEEL);
    nms->wheel = IPOD_CLASSIC_WHEEL(dev);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, WHEEL_MEM_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       s5l8702_get_irq(nms, S5L8702_WHEEL_IRQ));

    /*
     * The I2S transmitter at 0x3CA00000, the audio data path. It was
     * create_unimplemented_device() until now, and that threw ~6144 PCM samples
     * per boot on the floor (the DMA target FIFO is +0x10) and made every
     * I2STXCOM readback zero, so the firmware could never confirm the
     * transmitter started. See ipod_classic_i2s.c and wiki/14.
     *
     * The window is 0x1000; the rest of the 1 MB block still falls to the io
     * floor. It is given DMAC0 so it can pace the audio DMA (hito 2).
     */
    dev = qdev_new(TYPE_IPOD_CLASSIC_I2S);
    nms->i2s = IPOD_CLASSIC_I2S(dev);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, I2S_MEM_BASE);
    ipod_classic_i2s_set_dmac0(nms->i2s, dmac0);

    /*
     * The random number generator at 0x3C100000. See the long note above it:
     * the retailOS draws from this before it finishes bringing up storage, and
     * an unmapped block there is not a zero read but a spin with no timeout.
     */
    MemoryRegion *rng_mr = g_new(MemoryRegion, 1);
    memory_region_init_io(rng_mr, NULL, &ipod_classic_rng_ops,
                          g_new0(S5L8702RNGState, 1), "s5l8702.rng",
                          S5L8702_RNG_MEM_SIZE);
    memory_region_add_subregion(sysmem, S5L8702_RNG_MEM_BASE, rng_mr);

    /*
     * SM1 (0x38500000): the audio clock/stream engine. A stateful device that
     * models the run/power handshake the retailOS audio bring-up drives (block
     * identity, clock-ready, per-channel state machine, run latch). It was an
     * unimplemented zero window until the play path was found to hang on it; see
     * ipod_classic_sm1.c and wiki/19 section 11. Full 1 MB window so every
     * undecoded offset still reads 0 like the device it replaces.
     */
    /*
     * The SHA-1 accelerator (0x38000000).
     *
     * It was an unimplemented zero window, and the retailOS uses it: measured
     * live it writes the config at +0x00, arms memory mode at +0x80, sets the
     * input size at +0x8c and then reads the digest words back from +0x20..+0x34
     * -- and got zeros for the digest every time. A hash that never matches is
     * how the firmware ends up in its own panic path (0x08021bec, which prints
     * "CPU_exception_handler" through semihosting and then software-resets at
     * 0x08322b10), which is the ~46 s reboot that has made every measurement in
     * this project noisy and stops playback before it can start.
     *
     * The S5L8720 block modelled in hw/arm/ipod_touch_sha1.c has exactly the
     * register map this guest drives (SHA_CONFIG 0x0, SHA_MEMORY_MODE 0x80,
     * SHA_INSIZE 0x8c, digest at 0x20..0x34), so the same device serves the
     * S5L8702. Reused rather than re-written.
     */
    /*
     * SHA1_OFF=1 leaves the block as the zero window it used to be. Kept as a
     * switch because turning the engine on changes what the firmware can
     * verify: with it the panic-reset stops, but a disk image whose iTunesDB
     * hash the device cannot reproduce is then rejected outright ("0 songs")
     * instead of being accepted by a check that could never fail. Being able to
     * A/B the two on the same image is what tells those apart.
     */
    /*
     * The 1 MB window is backed EITHER WAY, and the engine sits on top of it.
     *
     * It used to be one or the other: with SHA1_OFF the whole megabyte was a
     * stub, and with the engine wired only its 0x100 bytes of registers were
     * mapped, leaving 0x38000100..0x380fffff unassigned. So the switch changed
     * far more than whether digests were real -- every access to the rest of
     * the block behaved differently -- and that is not a difference any A/B
     * comparison between the two should be carrying.
     */
    create_unimplemented_device("s5l8702.sha1", SHA1_MEM_BASE, 0x100000);
    if (!getenv("SHA1_OFF")) {
        dev = qdev_new("ipodtouch.sha1");
        sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
        memory_region_add_subregion_overlap(sysmem, SHA1_MEM_BASE,
                                            &IPOD_TOUCH_SHA1(dev)->iomem, 1);
    }

    dev = qdev_new(TYPE_IPOD_CLASSIC_SM1);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, SM1_MEM_BASE);
    /* The engine's playback position tracks what the I2S clocks out. */
    ipod_classic_sm1_set_i2s(IPOD_CLASSIC_SM1(dev), nms->i2s);
    /*
     * The engine's interrupt, VIC0 line 18. Our IRQ map came from Rockbox's
     * s5l87xx.h, which has a GAP at 18 -- Rockbox decodes on the ARM and never
     * drives this engine, so it never needed the name. Apple's retailOS does:
     * it installs a handler at 0x08017378 for index 18, and that handler is the
     * only thing that signals the event its native render pump waits on. The
     * line is wired unconditionally; SM1 only raises it under SM1_IRQ=1.
     * See wiki/19 section 11.25.
     */
    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0,
                       s5l8702_get_irq(nms, S5L8702_SM1_IRQ));

    /*
     * The audio data-path HLE. A no-op unless HLE_AUDIO_DATAPATH is set; when it
     * is, the host decodes the guest's own song (from the same disk image) and
     * feeds the PCM into DMAC0 channel 1 -- the channel that drains the audio
     * ping-pong to the I2S FIFO -- so the retailOS plays its real song through
     * its own I2S despite the MeCCA decode deadlock. See ipod_classic_hle_audio.c
     * and wiki/19 section 11.19. Default build is byte-identical.
     */
    ipod_classic_hle_audio_init(cpu, dmac0, nms->i2s);

    /* HLE "Now Playing" bootstrap (wiki/19 s.11.20): break the MeCCA decode
     * deadlock so the guest's own engine renders. No-op unless HLE_NP is set. */
    ipod_classic_hle_np_init(cpu);

    /* HLE "Now Playing" host overlay (wiki/19 s.11.23): a visible, controllable
     * player drawn over the framebuffer, driven by the data-path audio. No-op
     * unless HLE_NP_OVERLAY is set. */
    ipod_classic_np_overlay_init();

    /* Serial console. */
    exynos4210_uart_create(UART0_MEM_BASE, 256, 0, serial_hd(0),
                           s5l8702_get_irq(nms, S5L8702_UART0_IRQ));

    for (size_t i = 0; i < ARRAY_SIZE(ipod_classic_unimplemented); i++) {
        create_unimplemented_device(ipod_classic_unimplemented[i].name,
                                    ipod_classic_unimplemented[i].base,
                                    ipod_classic_unimplemented[i].size);
    }

    /*
     * A floor under the whole peripheral window. The retailOS maps
     * 0x38000000..0x3fffffff as one 768 MB uncached identity range in the page
     * table it builds at 0x22004534, so a register we have not listed above is
     * not a harmless zero read: it is a data abort, taken through a handler
     * that lives in DRAM and will cascade. Priority -2000 puts it below the
     * per-block windows, which are themselves at -1000.
     */
    MemoryRegion *io_floor = g_new(MemoryRegion, 1);
    memory_region_init_io(io_floor, NULL, &ipod_classic_io_floor_ops, NULL,
                          "s5l8702.io", IO_WINDOW_SIZE);
    memory_region_add_subregion_overlap(sysmem, IO_WINDOW_BASE, io_floor,
                                        -2000);

    qemu_register_reset(ipod_classic_cpu_reset, nms);
}

static void ipod_classic_machine_class_init(ObjectClass *klass, void *data)
{
    MachineClass *mc = MACHINE_CLASS(klass);

    mc->desc = "Apple iPod classic (6th generation), S5L8702";
    mc->init = ipod_classic_machine_init;
    mc->reset = ipod_classic_machine_reset;
    mc->max_cpus = 1;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm926");
}

static const TypeInfo ipod_classic_machine_info = {
    .name          = TYPE_IPOD_CLASSIC_MACHINE,
    .parent        = TYPE_MACHINE,
    .instance_size = sizeof(IPodClassicMachineState),
    .class_size    = sizeof(IPodClassicMachineClass),
    .class_init    = ipod_classic_machine_class_init,
    .instance_init = ipod_classic_instance_init,
};

static void ipod_classic_machine_types(void)
{
    type_register_static(&ipod_classic_machine_info);
}

type_init(ipod_classic_machine_types)
