/*
 * Apple iPod classic (6th generation) — Samsung S5L8702 SoC.
 *
 * The memory map below is derived from Rockbox's register header
 * firmware/export/s5l87xx.h, resolved for CONFIG_CPU == S5L8702. See
 * wiki/02-s5l8702-hardware.md for the derivation and the per-block notes.
 */

#ifndef HW_ARM_IPOD_CLASSIC_H
#define HW_ARM_IPOD_CLASSIC_H

#include "exec/hwaddr.h"
#include "hw/boards.h"
#include "hw/intc/pl192.h"
#include "hw/arm/boot.h"
#include "ui/console.h"
#include "cpu.h"
#include "hw/arm/ipod_classic/ipod_classic_timer.h"
#include "hw/arm/ipod_classic/ipod_classic_clk.h"
#include "hw/arm/ipod_classic/ipod_classic_i2c.h"
#include "hw/arm/ipod_classic/ipod_classic_pmu.h"
#include "hw/arm/ipod_classic/ipod_classic_ata.h"
#include "hw/arm/ipod_classic/ipod_classic_lcd.h"
#include "hw/arm/ipod_classic/ipod_classic_gpio.h"
#include "hw/arm/ipod_classic/ipod_classic_disp.h"
#include "hw/arm/ipod_classic/ipod_classic_wheel.h"
#include "hw/arm/ipod_classic/ipod_classic_i2s.h"
#include "hw/arm/ipod_classic/ipod_classic_sm1.h"

#define TYPE_IPOD_CLASSIC "iPod-Classic"

#define TYPE_IPOD_CLASSIC_MACHINE   MACHINE_TYPE_NAME(TYPE_IPOD_CLASSIC)
#define IPOD_CLASSIC_MACHINE(obj) \
    OBJECT_CHECK(IPodClassicMachineState, (obj), TYPE_IPOD_CLASSIC_MACHINE)

#define S5L8702_VIC_N     2
#define S5L8702_VIC_SIZE  32

/* RAM. DRAM_ORIG / IRAM_ORIG / IRAM0_SIZE / IRAM1_SIZE in s5l87xx.h. */
#define SDRAM_MEM_BASE    0x08000000
#define SDRAM_MEM_SIZE    0x04000000   /* 64 MB */

/*
 * The same DRAM again, uncached, with bit 31 set. The firmware uses both views
 * in the same breath -- it programs DMAC0 with a cached address while the CPU
 * runs on the uncached one -- and it hands the SHA-1 engine uncached addresses
 * (measured: 0x8ba718c0, i.e. this base + 0x03a718c0). See the note where the
 * alias is created.
 */
#define SDRAM_UNCACHED_MEM_BASE  0x88000000
#define IRAM0_MEM_BASE    0x22000000
#define IRAM0_MEM_SIZE    0x00020000   /* 128 KB */
#define IRAM1_MEM_BASE    0x22020000
#define IRAM1_MEM_SIZE    0x00020000   /* 128 KB, only 64 KB on the S5L8720 */

/*
 * The bootrom base is not documented by Rockbox (it never runs there). The
 * S5L8720 maps it at 0, and the S5L8702 reset vector is assumed to be the
 * same until we can check against a dump. See wiki/02, "pendiente de confirmar".
 */
#define BOOTROM_MEM_BASE  0x00000000
#define BOOTROM_MEM_SIZE  0x00020000

/*
 * The peripheral window. The retailOS maps all 768 MB of it as one uncached
 * identity section in its page table (0x22004534), so every address in here
 * has to answer: an unmapped one is a data abort, not a zero read.
 */
#define IO_WINDOW_BASE    0x38000000
#define IO_WINDOW_SIZE    0x08000000

/* Peripherals, IO_BASE 0x38000000. */
#define SHA1_MEM_BASE     0x38000000   /* SHA1_BASE */
#define MIU_MEM_BASE      0x38100000   /* MIU_BASE, memory interface unit */
#define DMAC0_MEM_BASE    0x38200000   /* DMA0_BASE, PL080 */
#define LCD_MEM_BASE      0x38300000   /* LCD_BASE */
#define USBOTG_MEM_BASE   0x38400000   /* OTGBASE, Synopsys DWC */
#define SM1_MEM_BASE      0x38500000   /* SM1_BASE, clock divider */
#define USB_HC_MEM_BASE   0x38600000   /* USB_HC_BASE */
#define ATA_MEM_BASE      0x38700000   /* ATA_BASE, the hard disk */
#define USB_FC_MEM_BASE   0x38800000   /* USB_FC_BASE */
/*
 * Not in s5l87xx.h and not in freemyipod's model: the display pipe. Named from
 * Apple's own code -- see ipod_classic_disp.h, which carries the whole register
 * map and how each line of it was found.
 */
#define DISP_MEM_BASE     0x38900000
#define ATA_UNK_MEM_BASE  0x38A00000   /* ATA_UNKNOWN_BASE */
#define SDCI_MEM_BASE     0x38B00000   /* SDCI_BASE, CE-ATA */
#define AES_MEM_BASE      0x38C00000   /* AES_BASE */
#define VIC0_MEM_BASE     0x38E00000   /* VIC_BASE */
#define VIC1_MEM_BASE     0x38E01000
/*
 * Sits right behind the two VICs. The iPod Touch machines call it "edgeic"
 * and back it with plain RAM rather than modelling it; Rockbox's bootloader
 * writes to +0x08 early on, so we need at least that much.
 */
#define EDGEIC_MEM_BASE   0x38E02000
#define EDGEIC_MEM_SIZE   0x1000
#define ADM_SFR_MEM_BASE  0x39000000   /* ADM_SFR_BASE, CalmADM2E audio DSP */
#define LCD_WIN_MEM_BASE  0x39200000   /* LCDBASE, framebuffer windows */
#define FMC_MEM_BASE      0x39400000   /* FMC_BASE, NAND controller */
#define DMAC1_MEM_BASE    0x39900000   /* DMA1_BASE, PL080 */
#define EIC_MEM_BASE      0x39A00000   /* EIC_BASE / SYSALV_BASE */
#define INT_MEM_BASE      0x39C00000   /* INT_BASE, legacy interrupt ctrl */
#define ECC_MEM_BASE      0x39E00000   /* ECC_BASE */
#define WHEEL_MEM_BASE    0x3C200000   /* WHEEL_BASE, click wheel */
#define USBPHYS_MEM_BASE  0x3C400000   /* USB_PHY_BASE */
#define CLOCK0_MEM_BASE   0x3C500000   /* CLK_BASE */
#define I2C0_MEM_BASE     0x3C600000   /* IIC_BASE */
#define TIMER_MEM_BASE    0x3C700000   /* TIMER_BASE */
#define WDT_MEM_BASE      0x3C800000   /* WDT_BASE */
#define I2S_MEM_BASE      0x3CA00000   /* I2S_BASE */
#define SPDIF_MEM_BASE    0x3CB00000   /* SPD_BASE */
#define UART0_MEM_BASE    0x3CC00000   /* UARTC_BASE_ADDR */
#define SPI0_MEM_BASE     0x3CD00000   /* SPI_BASE */
#define ADC_MEM_BASE      0x3CE00000   /* ADC_BASE */
#define GPIO_MEM_BASE     0x3CF00000   /* GPIO_BASE */
#define CHIPID_MEM_BASE   0x3D100000   /* CHIPID_BASE */
#define RTC_MEM_BASE      0x3D200000   /* RTC_BASE, also SPIBASE(2) */
#define ADM_MEM_BASE      0x3F000000   /* ADM_BASE */

/* IRQ numbers, from the IRQ_* macros in s5l87xx.h. */
#define S5L8702_TIMER32_IRQ  7
#define S5L8702_TIMER_IRQ    8
#define S5L8702_SPI0_IRQ     9
#define S5L8702_SPI1_IRQ     10
#define S5L8702_SPI2_IRQ     11
#define S5L8702_LCD_IRQ      14
#define S5L8702_DMAC0_IRQ    16
#define S5L8702_DMAC1_IRQ    17
/*
 * 18 is absent from Rockbox's s5l87xx.h IRQ list (our source for this map)
 * because Rockbox decodes audio on the ARM and never drives the SM1 engine.
 * Apple's retailOS installs a VIC handler at index 18 (0x08017378) which reads
 * an SM1 register and signals the event its native render pump waits on, so on
 * this SoC line 18 is the audio engine's. INFERIDO from the handler's own
 * register access; see wiki/19 section 11.25.
 */
#define S5L8702_SM1_IRQ      18
#define S5L8702_USB_FUNC_IRQ 19
#define S5L8702_NAND_IRQ     20
#define S5L8702_I2C0_IRQ     21
#define S5L8702_I2C1_IRQ     22
#define S5L8702_WHEEL_IRQ    23
#define S5L8702_UART0_IRQ    24
#define S5L8702_UART1_IRQ    25
#define S5L8702_UART2_IRQ    26
#define S5L8702_UART3_IRQ    27
#define S5L8702_UART4_IRQ    28
#define S5L8702_ATA_IRQ      29
#define S5L8702_SECBOOT_IRQ  36
#define S5L8702_AES_IRQ      39
#define S5L8702_SHA_IRQ      40
#define S5L8702_NANDECC_IRQ  43
#define S5L8702_MMC_IRQ      44

/*
 * The parameter block the bootrom leaves behind, in the last 256 bytes of
 * IRAM1. Skipping the bootrom means we owe the firmware everything that block
 * would have contained; see wiki/11-classic6g-retailos.md for how each field
 * below was found.
 *
 *   +0x00  eight bytes naming the image that should run: "retailOS",
 *          "diskmode", "diskscan", "softupdt" or "--------". The retailOS
 *          writes it (0x0827a8d4) before asking for a reboot, and the bootrom
 *          is what reads it back. VERIFIED as a writer; nothing in the osos
 *          reads it, so what the bootrom expects at power-on is UNDETERMINED
 *          and we leave it zero.
 *   +0x18  'IsyS' if what follows is valid
 *   +0x1c  pointer to a 288-byte SysInfo struct
 *
 * 0x0827a904 is the whole accessor: "return 0x2203ff00 + N". 0x08323aec is the
 * only caller, asking for N = 24, comparing 'IsyS' and memcpy()ing 0x120 bytes
 * from the pointer into its own bss at 0x0896d6fc.
 */
#define S5L8702_BOOTINFO_BASE     0x2203ff00
#define S5L8702_BOOTINFO_TAG      (S5L8702_BOOTINFO_BASE + 0x18)
#define S5L8702_BOOTINFO_PTR      (S5L8702_BOOTINFO_BASE + 0x1c)
#define S5L8702_SYSINFO_MAGIC     0x53797349   /* 'IsyS' */
#define S5L8702_SYSINFO_SIZE      0x120

/*
 * Where we put the struct itself. ASSUMED: the real bootrom's address for it
 * is unknown, and nothing in the osos cares -- it only ever follows the
 * pointer. Parking it immediately below the parameter block keeps both in the
 * same 512 bytes at the top of IRAM1, away from anything the firmware uses.
 */
#define S5L8702_SYSINFO_BASE      (S5L8702_BOOTINFO_BASE - S5L8702_SYSINFO_SIZE)

/*
 * The three fields of that struct the boot path actually reads.
 *
 *   +0x84  hardware id. 0x0827a188 demands (id >> 16) == 19 and 0x083286d4
 *          reboots into "softupdt" when it is anything else, so this one is
 *          not optional and its value is not a guess. 19 == 0x13 also matches
 *          the HwVr entry of the SysCfg block in this device's own NOR
 *          (assets/classic6g/bootflash_nor_1MB.bin, offset 0x5c).
 *   +0xe0  DRAM size, +0xe4 DRAM base. 0x22004534 maps
 *          [+0xe4 .. +0xe4 + +0xe0) as cached identity in the page table it
 *          builds, and 0x080634c4 hands +0xe4 + +0xe0 to the heap as the top
 *          of memory. Zeros there produce an identity map over all 4 GB that
 *          wipes out the VA 0 -> IRAM0 entry, and a heap whose top is below
 *          its base.
 *
 * Everything else stays zero on purpose. 0x08323aa4's callers also read +0x18,
 * +0x38, +0x3c, +0x48, +0x4c, +0x94, +0xe8 and +0x104; what those mean is
 * UNDETERMINED, and filling them from a plausible-looking guess would be
 * inventing with the appearance of rigour.
 */
#define S5L8702_SYSINFO_HW_ID_OFF     0x84
#define S5L8702_SYSINFO_DRAM_SIZE_OFF 0xe0
#define S5L8702_SYSINFO_DRAM_BASE_OFF 0xe4
#define S5L8702_HW_ID                 0x00130000

typedef struct {
    MachineClass parent;
} IPodClassicMachineClass;

typedef struct {
    MachineState parent;
    AddressSpace *nsas;
    qemu_irq **irq;
    ARMCPU *cpu;
    PL192State *vic0;
    PL192State *vic1;
    IPodClassicClkState *clock0;
    IPodClassicTimerState *timer;
    IPodClassicI2CState *i2c0;
    IPodClassicATAState *ata;
    IPodClassicLCDState *lcd;
    IPodClassicDispState *disp;
    IPodClassicWheelState *wheel;
    IPodClassicI2SState *i2s;
    Clock *sysclk;
    bool semihosting;
    /*
     * Where the CPU starts executing after reset. Either BOOTROM_MEM_BASE when
     * a bootrom is supplied, or SDRAM_MEM_BASE when a raw image is loaded with
     * -kernel.
     */
    hwaddr reset_pc;
    char bootrom_path[1024];
    /*
     * Optional 0x120-byte SysInfo block to hand the OS instead of the three
     * fields we synthesise. Firmware 2.0.x needs it: 0x082be27c (2.0.5) wants
     * (hw_id >> 16) == 0x13 AND (hw_id & 0xffff) >= 0x300, so the 6G's
     * 0x00130000 reboots it into "softupdt".
     */
    char sysinfo_path[1024];
} IPodClassicMachineState;

#endif
