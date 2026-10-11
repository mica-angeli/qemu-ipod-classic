/*
 * Cirrus Logic CS42L55 audio codec of the iPod classic 6G, on I2C0 at 7-bit
 * address 0x4A (8-bit 0x94).
 *
 * Both drivers agree on the address and protocol. Rockbox: cscodec-6g.c:39
 * does i2c_write(0, 0x94, reg, 1, &data) -- bus 0, 8-bit slave 0x94, a
 * sub-address (register number) then one data byte. Apple's retailOS: the
 * cscodec_write veneer at osos VA 0x0808e1ac loads r0=0x4a and calls the same
 * I2C transfer routine the PMU uses (0x08325a20). See wiki/14.
 *
 * To a driver this chip is a 128-register file. Every access is either a plain
 * write or a read-modify-write (Rockbox cscodec_setbits, cs42l55.c:49; Apple's
 * init is the same shape), so a register file with honest readback is all the
 * firmware needs. Nothing in either driver reads CHIPVERSION (0x01), but it is
 * seeded with a plausible id in case a future path checks it.
 */

#ifndef HW_ARM_IPOD_CLASSIC_CS42L55_H
#define HW_ARM_IPOD_CLASSIC_CS42L55_H

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "qom/object.h"

#define TYPE_IPOD_CLASSIC_CS42L55 "cs42l55"
OBJECT_DECLARE_SIMPLE_TYPE(IPodClassicCS42L55State, IPOD_CLASSIC_CS42L55)

#define S5L8702_CS42L55_I2C_ADDR   0x4a   /* 0x94 >> 1 */
#define S5L8702_CS42L55_NUM_REGS   0x80

#define CS42L55_REG_CHIPVERSION    0x01
/*
 * CONVENIENCE. The CS42L55 reports its device id in the high bits of register
 * 0x01. No dump of a real part is in our sources and no driver we have reads
 * it; 0x5A is the value the freemyipod research notes for this codec. If a
 * firmware path is found that compares it, this is where it gets a real value.
 */
#define S5L8702_CS42L55_CHIP_ID    0x5a

/*
 * The headphone amplifier, one register per channel. Rockbox's driver names
 * them (firmware/export/cs42l55.h) and Apple's uses them the same way --
 * MEASURED with CODECTRACE=1 on firmware 2.0.5, one wheel step at a time:
 *
 *     bottom of the bar   0x44   -60 dB
 *     factory default     0x60   -32 dB
 *     each step           +3     +3 dB
 *     top of the bar      0x02    +2 dB
 *
 * Bits 6:0 are the gain in dB as a seven-bit two's complement number, bit 7
 * mutes the channel. This is where an iPod's volume lives: the PCM that
 * reaches the I2S is not scaled by the volume setting, so a model that
 * ignores these registers plays everything at full scale whatever the screen
 * says.
 */
#define CS42L55_REG_HPACTL         0x1a
#define CS42L55_REG_HPBCTL         0x1b
#define CS42L55_HPCTL_MUTE         0x80
#define CS42L55_HPCTL_VOL_MASK     0x7f

/* The register-address byte: bit 7 asks for auto-increment. */
#define CS42L55_MAP_INCR           0x80

struct IPodClassicCS42L55State {
    I2CSlave parent_obj;

    uint8_t reg[S5L8702_CS42L55_NUM_REGS];
    uint8_t addr;         /* register pointer, survives a STOP like the PMU */
    bool addr_pending;    /* next byte written is the register number */
    bool incr;            /* the pointer advances after each byte */
};

/*
 * The headphone gain the codec is set to, per channel (0 = A/left), as a Q15
 * multiplier for whoever turns the I2S stream into sound. Unity until the
 * guest programs the amplifier; never above unity, which would clip.
 */
uint32_t ipod_classic_cs42l55_hp_gain_q15(int ch);

#endif
