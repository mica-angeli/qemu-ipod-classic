/*
 * Cirrus Logic CS42L55 audio codec, I2C0 slave 0x4A.
 *
 * See include/hw/arm/ipod_classic/ipod_classic_cs42l55.h for the address and
 * protocol, and wiki/14-classic6g-audio.md for the audio path.
 *
 * The chip is a register-pointer I2C device, exactly like the PMU next to it on
 * the same bus: a write starts with the register number and the pointer
 * advances after every byte; a read carries on from wherever the pointer was
 * left. That is what makes the firmware's read-modify-write of a codec register
 * (read reg, change some bits, write it back) come out right. While this slave
 * did not exist the retailOS's writes to 0x4A were NACKed and every cscodec_read
 * returned bus garbage.
 */

#include "qemu/osdep.h"
#include <math.h>
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "hw/arm/ipod_classic/ipod_classic_cs42l55.h"

/*
 * THE VOLUME. One codec per machine, so the gain the I2S model asks for lives
 * at file scope like the rest of the audio path's shared state.
 *
 * AUDIOGAIN adjusts how it is applied, because a faithful headphone level is
 * not always what a desk wants: the factory default is -32 dB.
 *     AUDIOGAIN=<dB>   make-up gain added to the codec's setting
 *     AUDIOGAIN=line   ignore the codec: the fixed level of the dock line-out,
 *                      which is what this model played before it had a volume
 * Whatever the sum, it is capped at unity -- there is no headroom above full
 * scale to amplify into.
 */
static uint32_t cs42l55_gain_q15[2] = { 32768, 32768 };

static void cs42l55_update_gain(IPodClassicCS42L55State *s)
{
    static int mode = -1;       /* 0 = follow the codec, 1 = line level */
    static double makeup;

    if (mode < 0) {
        const char *e = getenv("AUDIOGAIN");

        mode = e && !strcmp(e, "line");
        makeup = (e && !mode) ? atof(e) : 0.0;
    }
    for (int ch = 0; ch < 2; ch++) {
        uint8_t r = s->reg[CS42L55_REG_HPACTL + ch];
        int db = r & CS42L55_HPCTL_VOL_MASK;
        double g;

        if (mode) {
            cs42l55_gain_q15[ch] = 32768;
            continue;
        }
        if (db & 0x40) {
            db -= 0x80;                 /* seven-bit two's complement */
        }
        db = MAX(-60, MIN(12, db));
        g = (r & CS42L55_HPCTL_MUTE) ? 0.0
                                     : pow(10.0, MIN(0.0, db + makeup) / 20.0);
        cs42l55_gain_q15[ch] = (uint32_t)(g * 32768.0 + 0.5);
    }
}

uint32_t ipod_classic_cs42l55_hp_gain_q15(int ch)
{
    return cs42l55_gain_q15[ch & 1];
}

static int ipod_classic_cs42l55_event(I2CSlave *i2c, enum i2c_event event)
{
    IPodClassicCS42L55State *s = IPOD_CLASSIC_CS42L55(i2c);

    switch (event) {
    case I2C_START_SEND:
        /* The next byte is the register number, not data. */
        s->addr_pending = true;
        break;
    case I2C_START_RECV:
    case I2C_FINISH:
    case I2C_NACK:
        break;
    default:
        break;
    }
    return 0;
}

static uint8_t ipod_classic_cs42l55_recv(I2CSlave *i2c)
{
    IPodClassicCS42L55State *s = IPOD_CLASSIC_CS42L55(i2c);
    uint8_t val = s->reg[s->addr & (S5L8702_CS42L55_NUM_REGS - 1)];

    if (s->incr) {
        s->addr++;
    }
    return val;
}

static int ipod_classic_cs42l55_send(I2CSlave *i2c, uint8_t data)
{
    IPodClassicCS42L55State *s = IPOD_CLASSIC_CS42L55(i2c);

    if (s->addr_pending) {
        /*
         * The register-address byte. Bit 7 is not part of the address: it
         * says whether the pointer moves on after each byte. The firmware
         * sets it for the two-register writes (0x9a then both headphone
         * gains) and leaves it clear for single ones.
         */
        s->addr = data & ~CS42L55_MAP_INCR;
        s->incr = data & CS42L55_MAP_INCR;
        s->addr_pending = false;
    } else {
        /*
         * CODECTRACE=1: every register write. The volume the listener hears
         * is set here, not in the PCM, so this is how to see what the
         * firmware does with it.
         */
        static int trace = -1;
        if (trace < 0) {
            trace = getenv("CODECTRACE") != NULL;
        }
        if (trace) {
            fprintf(stderr, "CODEC wr reg 0x%02x = 0x%02x\n",
                    s->addr & (S5L8702_CS42L55_NUM_REGS - 1), data);
        }
        s->reg[s->addr & (S5L8702_CS42L55_NUM_REGS - 1)] = data;
        if (s->incr) {
            s->addr++;
        }
        cs42l55_update_gain(s);
    }
    return 0;
}

static void ipod_classic_cs42l55_reset(DeviceState *dev)
{
    IPodClassicCS42L55State *s = IPOD_CLASSIC_CS42L55(dev);

    memset(s->reg, 0, sizeof(s->reg));
    s->addr = 0;
    s->addr_pending = false;
    s->incr = false;
    cs42l55_update_gain(s);
    s->reg[CS42L55_REG_CHIPVERSION] = S5L8702_CS42L55_CHIP_ID;
}

static const VMStateDescription vmstate_ipod_classic_cs42l55 = {
    .name = TYPE_IPOD_CLASSIC_CS42L55,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_I2C_SLAVE(parent_obj, IPodClassicCS42L55State),
        VMSTATE_UINT8_ARRAY(reg, IPodClassicCS42L55State,
                            S5L8702_CS42L55_NUM_REGS),
        VMSTATE_UINT8(addr, IPodClassicCS42L55State),
        VMSTATE_BOOL(addr_pending, IPodClassicCS42L55State),
        VMSTATE_END_OF_LIST()
    },
};

static void ipod_classic_cs42l55_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->desc = "Cirrus Logic CS42L55 audio codec";
    dc->reset = ipod_classic_cs42l55_reset;
    dc->vmsd = &vmstate_ipod_classic_cs42l55;
    k->event = ipod_classic_cs42l55_event;
    k->recv = ipod_classic_cs42l55_recv;
    k->send = ipod_classic_cs42l55_send;
}

static const TypeInfo ipod_classic_cs42l55_info = {
    .name          = TYPE_IPOD_CLASSIC_CS42L55,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(IPodClassicCS42L55State),
    .class_init    = ipod_classic_cs42l55_class_init,
};

static void ipod_classic_cs42l55_register_types(void)
{
    type_register_static(&ipod_classic_cs42l55_info);
}

type_init(ipod_classic_cs42l55_register_types)
