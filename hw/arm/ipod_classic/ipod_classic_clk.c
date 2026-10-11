/*
 * S5L8702 clock and PLL block.
 *
 * A register file, plus the two pieces of behaviour the guest can observe:
 * PLLLOCK has to report the PLLs locked or a driver that waits for them never
 * comes back, and a write to SWRCON reboots the machine.
 *
 * See include/hw/arm/ipod_classic/ipod_classic_clk.h for where the register
 * map comes from.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "exec/cpu-common.h"
#include "exec/exec-all.h"
#include "hw/core/cpu.h"
#include "target/arm/cpu.h"
#include "migration/vmstate.h"
#include "sysemu/runstate.h"
#include "qemu/error-report.h"
#include "hw/arm/ipod_classic/ipod_classic_clk.h"
#include "hw/arm/ipod_classic/ipod_classic_hle_audio.h"

/*
 * The 256 bytes at the top of IRAM1 that the bootrom and the OS use to talk to
 * each other. The first eight bytes are the name of the image that should run
 * next, which is exactly what we want to see when the guest asks for a reboot:
 * 0x0827a8d4 copies one of "--------", "diskmode", "diskscan", "softupdt" or
 * "retailOS" there from the table at 0x0837be25, and 0x08322b10 then writes
 * SWRCON. See wiki/11-classic6g-retailos.md.
 */
#define S5L8702_BOOT_TAG_ADDR   0x2203ff00

static uint64_t ipod_classic_clk_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    IPodClassicClkState *s = opaque;

    if (offset + size > S5L8702_CLK_MEM_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: read past the block at 0x%"
                      HWADDR_PRIx "\n", __func__, offset);
        return 0;
    }

    switch (offset) {
    case S5L8702_CLK_PLLLOCK:
        /*
         * "Start status: 0 -> in progress, 1 -> locked" for bits 0..2, and the
         * divisor-mode lock in bits 4..6. Our PLLs settle instantly, so report
         * every PLL the guest enabled in PLLMODE as both started and locked.
         * Reporting a PLL locked that was never enabled would be a lie the
         * guest could catch; reporting zero would hang whoever waits.
         */
        {
            uint32_t mode = s->reg[S5L8702_CLK_PLLMODE / 4];
            uint32_t lock = 0;
            for (int n = 0; n < 3; n++) {
                if (mode & (1u << n)) {
                    lock |= S5L8702_PLLLOCK_LCK(n) | S5L8702_PLLLOCK_DMLCK(n);
                }
            }
            return lock;
        }
    default:
        return s->reg[offset / 4];
    }
}

static void ipod_classic_clk_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    IPodClassicClkState *s = opaque;

    if (offset + size > S5L8702_CLK_MEM_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: write past the block at 0x%"
                      HWADDR_PRIx " (value 0x%" PRIx64 ")\n", __func__, offset,
                      value);
        return;
    }

    switch (offset) {
    case S5L8702_CLK_SWRCON:
        if (value != 0) {
            char tag[9] = { 0 };

            cpu_physical_memory_read(S5L8702_BOOT_TAG_ADDR, tag, 8);
            for (int i = 0; i < 8; i++) {
                if (tag[i] < 0x20 || tag[i] > 0x7e) {
                    tag[i] = '.';
                }
            }
            CPUState *cs = current_cpu;
            uint32_t pc = cs ? (uint32_t)CPU_GET_CLASS(cs)->get_pc(cs) : 0;

            /*
             * HLE audio: while the shim is actively feeding the guest's own I2S
             * with the decoded song, the retailOS MeCCA pipeline is still in its
             * symmetric bootstrap deadlock (wiki/19 11.12-11.18), so its play
             * task stalls and its watchdog reboots the device (the blank "----"
             * image tag) ~15 s in -- which would cut the song off and stop the
             * Now Playing screen from ever settling. Suppress ONLY that
             * watchdog reboot (blank tag) and ONLY while the HLE is playing, so
             * the song can run to the end through the guest's own DMA/I2S. Real
             * reboots (diskmode/softupdt/retailOS tags, and any reboot when the
             * HLE is not active) are untouched. Gated entirely by
             * HLE_AUDIO_DATAPATH; default build unaffected.
             */
            if (ipod_classic_hle_audio_sustain_on() && tag[0] == '-') {
                /*
                 * Suppress the deadlock-watchdog reboot AND redirect the CPU out
                 * of the dead reset spin. wiki/19 s11.20 found the old suppress
                 * (just skip the reset) left the CPU spinning forever at the nop
                 * loop 0x08322b28, freezing the UI/compositor so the (natively
                 * running) Now Playing controller could never present its frame.
                 * The reset routine 0x08322b10 is a LEAF called via `bl` from the
                 * reboot executor (LR = 0x0811325c), so returning (PC = LR) resumes
                 * normal execution: the executor returns to the service thread and
                 * the guest UI keeps running while the data-path HLE plays audio.
                 * Gated exactly as before (HLE_AUDIO_SUSTAIN + blank "----" tag);
                 * real reboots (diskmode/retailOS tags) are untouched.
                 */
                s->reg[offset / 4] = value;
                if (cs) {
                    CPUARMState *env = &ARM_CPU(cs)->env;
                    warn_report("S5L8702: SUPPRESSED deadlock-watchdog reboot "
                                "(SWRCON=0x%" PRIx64 ", pc~0x%08x) -- redirecting "
                                "CPU to lr=0x%08x so the UI keeps running",
                                value, pc, env->regs[14]);
                    env->regs[15] = env->regs[14];   /* return from reset routine */
                    cpu_loop_exit(cs);               /* does not return */
                }
                warn_report("S5L8702: SUPPRESSED deadlock-watchdog reboot "
                            "(SWRCON=0x%" PRIx64 ", pc~0x%08x)", value, pc);
                break;
            }

            /*
             * pc is only TB-accurate from an MMIO callback; lr and sp are
             * exact, and lr is what identifies who asked for the reset.
             */
            warn_report("S5L8702: guest asked for a software reset "
                        "(SWRCON = 0x%" PRIx64 "), next image \"%s\" pc~0x%08x"
                        " lr=0x%08x sp=0x%08x",
                        value, tag, pc,
                        cs ? ARM_CPU(cs)->env.regs[14] : 0,
                        cs ? ARM_CPU(cs)->env.regs[13] : 0);
            s->reg[S5L8702_CLK_RSTSR / 4] |= S5L8702_RSTSR_SWR;
            qemu_system_reset_request(SHUTDOWN_CAUSE_GUEST_RESET);
        }
        s->reg[offset / 4] = value;
        break;
    case S5L8702_CLK_RSTSR:
        /* Write-one-to-clear, the usual Samsung convention for this one. */
        s->reg[offset / 4] &= ~(uint32_t)value;
        break;
    case S5L8702_CLK_PLLLOCK:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: PLLLOCK is read-only\n", __func__);
        break;
    default:
        s->reg[offset / 4] = value;
        break;
    }
}

static const MemoryRegionOps ipod_classic_clk_ops = {
    .read = ipod_classic_clk_read,
    .write = ipod_classic_clk_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};

static void ipod_classic_clk_reset(DeviceState *dev)
{
    IPodClassicClkState *s = IPOD_CLASSIC_CLK(dev);

    memset(s->reg, 0, sizeof(s->reg));
    /*
     * Tell the guest the last reset was a power-on one. Apple's retailOS reads
     * RSTSR; leaving it at zero would mean "no reset happened", which is not a
     * state real silicon can be in.
     */
    s->reg[S5L8702_CLK_RSTSR / 4] = S5L8702_RSTSR_HWR;
}

static void ipod_classic_clk_init(Object *obj)
{
    IPodClassicClkState *s = IPOD_CLASSIC_CLK(obj);

    memory_region_init_io(&s->iomem, obj, &ipod_classic_clk_ops, s,
                          TYPE_IPOD_CLASSIC_CLK, S5L8702_CLK_MEM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
}

static const VMStateDescription vmstate_ipod_classic_clk = {
    .name = TYPE_IPOD_CLASSIC_CLK,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(reg, IPodClassicClkState,
                             S5L8702_CLK_MEM_SIZE / 4),
        VMSTATE_END_OF_LIST()
    },
};

static void ipod_classic_clk_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "S5L8702 clock and PLL block";
    dc->reset = ipod_classic_clk_reset;
    dc->vmsd = &vmstate_ipod_classic_clk;
}

static const TypeInfo ipod_classic_clk_info = {
    .name          = TYPE_IPOD_CLASSIC_CLK,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IPodClassicClkState),
    .instance_init = ipod_classic_clk_init,
    .class_init    = ipod_classic_clk_class_init,
};

static void ipod_classic_clk_register_types(void)
{
    type_register_static(&ipod_classic_clk_info);
}

type_init(ipod_classic_clk_register_types)
