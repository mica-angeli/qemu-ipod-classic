/*
 * S5L8702 I2C master, 0x3C600000 (bus 0) and 0x3C900000 (bus 1).
 *
 * A Samsung S3C-style controller with one S5L8702-specific twist, and the
 * twist is the whole design of this model.
 *
 * Rockbox's firmware/export/s5l87xx.h states it:
 *
 *     - IICCON[13:8] are used to enable interrupts.
 *       IICSTA2[13:8] are used to read the status and write-clear interrupts.
 *       Known interrupts:
 *        [13] STOP on bus (TBC)
 *        [12] START on bus (TBC)
 *        [8]  byte transmited or received in Master mode
 *     - IICCON[4] does not clear interrupts, it is enabled when a byte is
 *       transmited or received, in Master mode the tx/rx of the next byte
 *       starts when it is written as "1".
 *
 * Apple's driver agrees on every point. 0x08325908 configures the block with
 * IICCON = 0x184, which is CK_REG 4 | ACK_GEN | bit 8 -- the byte-transferred
 * interrupt enable. Its interrupt handler at 0x0832566c acknowledges with
 * "IICSTAT2 |= 0x100", and 0x08325634 clears the lot with "IICSTAT2 |= 0x3f00"
 * followed by "IICCON = IICCON", which is the same resume idiom Rockbox writes
 * as "IICCON(bus) = IICCON(bus)" in i2c_wr_internal().
 *
 * So: bit 4 of IICCON is owned by the hardware on read and is the go button on
 * write, and the interrupt lives in a different register from the flow
 * control. Getting those two the wrong way round is what makes an I2C model
 * that looks right hang the guest.
 */

#ifndef HW_ARM_IPOD_CLASSIC_I2C_H
#define HW_ARM_IPOD_CLASSIC_I2C_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/i2c/i2c.h"
#include "qom/object.h"

#define TYPE_IPOD_CLASSIC_I2C "s5l8702.i2c"
OBJECT_DECLARE_SIMPLE_TYPE(IPodClassicI2CState, IPOD_CLASSIC_I2C)

#define S5L8702_I2C_MEM_SIZE   0x24

#define S5L8702_IICCON     0x00
#define S5L8702_IICSTAT    0x04
#define S5L8702_IICADD     0x08
#define S5L8702_IICDS      0x0c
#define S5L8702_IICUNK10   0x10   /* reads non-zero while the block is busy */
#define S5L8702_IICUNK14   0x14
#define S5L8702_IICUNK18   0x18
#define S5L8702_IICSTAT2   0x20

/* IICCON */
#define S5L8702_IICCON_CK_REG_MASK  0x0f
#define S5L8702_IICCON_GO           (1u << 4)   /* read: byte done, write: go */
#define S5L8702_IICCON_CKSEL        (1u << 6)
#define S5L8702_IICCON_ACK_GEN      (1u << 7)
#define S5L8702_IICCON_INT_MASK     0x3f00      /* [13:8] interrupt enables */

/* IICSTAT: [7:6] mode, [5] busy, [4] serial output enable, [0] last-received-bit */
#define S5L8702_IICSTAT_LRB         (1u << 0)   /* 1 == the slave did not ACK */
#define S5L8702_IICSTAT_SOE         (1u << 4)
#define S5L8702_IICSTAT_BUSY        (1u << 5)
#define S5L8702_IICSTAT_MODE_MASK   0xf0
#define S5L8702_IICSTAT_START_TX    0xf0
#define S5L8702_IICSTAT_START_RX    0xb0
#define S5L8702_IICSTAT_STOP_TX     0xd0
#define S5L8702_IICSTAT_STOP_RX     0x90

/* IICSTAT2, write-one-to-clear, same bit numbers as the IICCON enables. */
#define S5L8702_IICSTAT2_BYTE       (1u << 8)
#define S5L8702_IICSTAT2_START      (1u << 12)
#define S5L8702_IICSTAT2_STOP       (1u << 13)

/*
 * How long a byte takes on the wire. Not cosmetic: both drivers write IICDS,
 * then write IICCON to start the byte, and only then clear the previous
 * interrupt in IICSTAT2. If the byte finished the instant IICCON was written,
 * that clear would wipe out the interrupt it just raised and the transfer
 * would stop for ever. 9 bits at 100 kHz is 90 us on real silicon; 25 is
 * enough to be after the driver's own tail and far inside every timeout it
 * uses.
 */
#define S5L8702_I2C_BYTE_US   25

/*
 * What the bootrom leaves in IICCON, and why this is a property rather than a
 * reset value.
 *
 * The retailOS never configures the block on the path it actually takes.
 * 0x08325908 is the only code that writes IICCON = 0x184 -- CK_REG 4, ACK_GEN,
 * and bit 8, the byte-transferred interrupt enable -- and its only caller,
 * 0x083259c8, reaches it exclusively when 0x220003b8 reports the bus still
 * busy after 100 ms, i.e. as bus recovery. Every other write is a
 * read-modify-write that preserves whatever is already there: "IICCON |= 0x80"
 * in the two transfer starters, "IICCON = IICCON" in 0x08325634, and the
 * ACK_GEN flip in the interrupt handler.
 *
 * So the driver inherits its own interrupt enable. Something before it must
 * have set it, and the only thing that runs before it is the bootrom, which
 * has to reach the PMU on this very bus to find out why the machine woke up.
 *
 * INFERRED. We have no bootrom dump -- wiki/10 records that this SoC will not
 * give one up -- so this is reasoning from the driver's own shape, not a
 * reading of the code that does it. What is VERIFIED is the consequence: with
 * IICCON left at zero the enable mask is zero, the handler never runs, and the
 * retailOS blocks on RTXC event 61 for ever.
 */
#define S5L8702_I2C_BOOTROM_CON   0x184

struct IPodClassicI2CState {
    SysBusDevice parent_obj;

    /* IICCON as of reset; see S5L8702_I2C_BOOTROM_CON. */
    uint32_t reset_con;

    MemoryRegion iomem;
    I2CBus *bus;
    qemu_irq irq;
    QEMUTimer *byte_timer;

    uint32_t iiccon;
    uint32_t iicstat;
    uint32_t iicadd;
    uint32_t iicds;
    uint32_t iicunk14;
    uint32_t iicunk18;
    uint32_t iicstat2;

    /* IICCON bit 4 as the hardware sees it: a byte has finished, go is armed. */
    bool byte_done;
    /* Set between IICSTAT = 0xf0/0xb0 and the address byte completing. */
    bool in_address;

    /*
     * Whether a transfer is open on the QEMU bus, and to whom. NOT the BUSY
     * bit of IICSTAT: the guest writes that bit itself, as part of the mode
     * field, so by the time a STOP is being handled it already reads 0.
     */
    bool xfer_open;
    uint8_t xfer_addr;
};

#endif
