/*
 * S5L8702 ATA controller.
 *
 * PIO and DMA. The task file goes through QEMU's IDE core; the DMA engine is
 * the SoC's own, a flat buffer rather than a scatter list, and it is driven
 * from the go bit in ATA_COMMAND. Apple's retailOS identifies the disk over
 * PIO and then reads every sector with CMD_READ_DMA, so both paths are needed
 * before the boot gets past storage.
 *
 * See include/hw/arm/ipod_classic/ipod_classic_ata.h for the bridge protocol,
 * which is the part of this block that is not obvious.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/cpu.h"
#include "target/arm/cpu.h"
#include "qemu/module.h"
#include "exec/address-spaces.h"
#include "sysemu/dma.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "sysemu/blockdev.h"
#include "hw/arm/ipod_classic/ipod_classic_ata.h"
#include "hw/arm/ipod_classic/ipod_classic_hle_audio.h"

/* Which IDE core port a device-register offset corresponds to. */
static int ata_ioport(hwaddr offset)
{
    return (offset - S5L8702_ATA_PIO_DTR) / 4;
}

/*
 * Set ATATRACE=1 to get every register access on stderr. This block is driven
 * by a guest we cannot single-step and whose source we do not have, so reading
 * the order it touches things in is how its protocol was worked out in the
 * first place -- ATA_PIO_READY is left out because it is polled in a tight
 * loop and would bury everything else. Checked once: getenv on every MMIO
 * access is not free.
 */
static bool ata_trace(void)
{
    static int on = -1;

    if (on < 0) {
        on = getenv("ATATRACE") != NULL;
    }
    return on;
}

/*
 * ATATRACE=2 stamps each line with the virtual time. The order of the accesses
 * says what the guest's protocol is; the gaps between them say where it is
 * WAITING, which is the question when a track takes 23 s to start.
 */
static bool ata_trace_stamp(void)
{
    static int on = -1;

    if (on < 0) {
        const char *e = getenv("ATATRACE");
        on = e && atoi(e) >= 2;
    }
    return on;
}

static void ipod_classic_ata_update_irq(IPodClassicATAState *s)
{
    uint32_t active = s->reg[S5L8702_ATA_IRQSTAT / 4] &
                      s->reg[S5L8702_ATA_IRQMASK / 4];

    qemu_set_irq(s->irq, active != 0);
}

static void ipod_classic_ata_set_irq(void *opaque, int n, int level)
{
    IPodClassicATAState *s = opaque;

    s->intrq = level;
    if (level) {
        s->reg[S5L8702_ATA_IRQSTAT / 4] |= S5L8702_ATA_INTRQ_BIT;
    }
    ipod_classic_ata_update_irq(s);
}

/*
 * The go bit. By the time this runs the guest has programmed the buffer and
 * the IDE core has already handed us the callback it wants run when the
 * transfer is set up, so this is where the cursor is loaded and the core is
 * let go. Doing it any earlier would read a buffer address the guest has not
 * written yet -- see the sequence in the header.
 */
static void ipod_classic_ata_dma_go(IPodClassicATAState *s)
{
    bool is_write = s->reg[S5L8702_ATA_CFG / 4] & S5L8702_ATA_CFG_WRITE;

    s->dma_cur = is_write ? s->reg[S5L8702_ATA_SBUF_START / 4]
                          : s->reg[S5L8702_ATA_TBUF_START / 4];
    /* ATA_XFR_NUM holds the byte count minus one. */
    s->dma_left = s->reg[S5L8702_ATA_XFR_NUM / 4] + 1;
    s->reg[S5L8702_ATA_XFR_CNT / 4] = 0;

    if (s->dma_cb) {
        BlockCompletionFunc *cb = s->dma_cb;

        s->dma_cb = NULL;
        cb(ide_bus_active_if(&s->bus), 0);
    }
}

static void ipod_classic_ata_dma_start(const IDEDMA *dma, IDEState *is,
                                       BlockCompletionFunc *cb)
{
    IPodClassicATAState *s = container_of(dma, IPodClassicATAState, dma);

    /*
     * Latch only. The ATA command is written before the buffer address and
     * before the go bit, so there is nothing to transfer yet.
     */
    s->dma_cb = cb;
}

static int32_t ipod_classic_ata_dma_prepare_buf(const IDEDMA *dma,
                                                int32_t limit)
{
    IPodClassicATAState *s = container_of(dma, IPodClassicATAState, dma);
    IDEState *is = ide_bus_active_if(&s->bus);
    uint32_t len = MIN((uint32_t)limit, s->dma_left);

    /*
     * One flat buffer, so one entry. If the guest asked for fewer bytes than
     * the ATA command will produce, the short count goes back to the core,
     * which treats it as the error it is instead of writing past the end.
     */
    qemu_sglist_init(&is->sg, DEVICE(s), 1, &address_space_memory);
    if (len) {
        qemu_sglist_add(&is->sg, s->dma_cur, len);
        s->dma_cur += len;
        s->dma_left -= len;
        s->reg[S5L8702_ATA_XFR_CNT / 4] += len;

        /*
         * ATALBA=1: which disk bytes are actually read, and how many.
         *
         * The audio investigation reached a point where the decoder appears
         * never to produce, and "produces nothing" and "is never given
         * anything" look identical from downstream. This answers that
         * directly: during playback the guest should be pulling the song off
         * the disk in a steady trickle. If reads stop once the database is
         * loaded, the input side is what is broken and nothing further down
         * matters. ATATRACE only ever printed register offsets, so this could
         * not be seen before. Doubling schedule plus a running total.
         */
        if (getenv("ATALBA") &&
            !(s->reg[S5L8702_ATA_CFG / 4] & S5L8702_ATA_CFG_WRITE)) {
            static uint64_t rd_ops, rd_bytes;
            /*
             * Comprobar que los bytes ATERRIZAN. Saber que se lee el sector
             * correcto no dice que el dato llegue a la memoria del invitado: si
             * el destino esta mal, el parser ve basura y el reproductor aborta
             * -- que es exactamente el sintoma. Leer el destino de la
             * transferencia ANTERIOR (esta ya ha terminado cuando llega la
             * siguiente) y publicar sus primeras palabras junto al sector del
             * que venian. Un volcado por el monitor no sirve: los buferes se
             * reutilizan en segundos.
             */
            static uint32_t prev_dst;
            static int64_t prev_sec = -1;
            if (prev_sec >= 0) {
                uint8_t head[8] = { 0 };
                address_space_read(&address_space_memory, prev_dst,
                                   MEMTXATTRS_UNSPECIFIED, head, sizeof(head));
                fprintf(stderr, "ATALND lba=%" PRId64 " dst=0x%08x -> "
                        "%02x %02x %02x %02x %02x %02x %02x %02x\n",
                        prev_sec, prev_dst, head[0], head[1], head[2], head[3],
                        head[4], head[5], head[6], head[7]);
            }
            /*
             * OJO: `s->dma_cur` ya se ha adelantado `len` bytes unas lineas mas
             * arriba, asi que el principio de ESTA transferencia es
             * `dma_cur - len`. Leer sin restarlo mira el final del bloque y
             * devuelve ceros, que se lee igual que "el dato no llega" -- un
             * falso positivo listo para colarse en una conclusion.
             */
            /*
             * ANCLA DE TIEMPO DE REPRODUCCION. Todos los sitios con nombre del
             * mapa heredado disparan tambien en el arranque, asi que no sirven
             * para abrir una ventana de traza en el momento de reproducir. Una
             * lectura de un sector DENTRO del fichero de la cancion si es
             * exclusiva de la reproduccion: ATALBA_LO/HI acotan ese rango (en
             * bytes) y aqui se publica el PC del invitado que la pidio, que es
             * codigo del reproductor y sirve de punto de entrada fiable.
             */
            {
                const char *lo_s = getenv("ATALBA_LO");
                const char *hi_s = getenv("ATALBA_HI");
                if (lo_s && hi_s) {
                    uint64_t byte = (uint64_t)ide_get_sector(is) * 512;
                    uint64_t lo = strtoull(lo_s, NULL, 0);
                    uint64_t hi = strtoull(hi_s, NULL, 0);
                    if (byte >= lo && byte <= hi) {
                        static uint64_t in_song;
                        uint64_t k = ++in_song;
                        if ((k & (k - 1)) == 0) {
                            CPUState *cs = current_cpu;
                            uint32_t pc = 0, lr = 0, sp = 0;
                            if (cs) {
                                CPUARMState *e = &ARM_CPU(cs)->env;
                                pc = (uint32_t)CPU_GET_CLASS(cs)->get_pc(cs);
                                /*
                                 * LR y SP del invitado en el momento de pedir el
                                 * sector: el PC solo da la rutina del driver, que
                                 * es comun a todo el disco. El LR nombra a QUIEN
                                 * la llama, y eso ya es codigo del reproductor.
                                 */
                                lr = (uint32_t)e->regs[14];
                                sp = (uint32_t)e->regs[13];
                            }
                            fprintf(stderr, "ATASONG #%" PRIu64 " byte=%" PRIu64
                                    " pc~0x%08x lr=0x%08x sp=0x%08x\n",
                                    k, byte, pc, lr, sp);
                            /*
                             * LA CADENA DE LLAMADAS, que es lo que hace falta.
                             * El LR solo da un nivel. Barriendo la pila y
                             * quedandose con los valores que caen en el rango de
                             * codigo del firmware salen las direcciones de
                             * retorno de los marcos de arriba -- o sea el camino
                             * desde el reproductor hasta esta lectura. Con la
                             * pila fija (mismo SP en todas las lecturas) esto es
                             * estable y repetible. Hay falsos positivos (datos
                             * que parecen punteros), asi que se publican en
                             * crudo y se filtran leyendo el desensamblado.
                             */
                            if (cs && k == 1) {
                                for (int w = 0; w < 96; w++) {
                                    uint32_t v = 0;
                                    address_space_read(&address_space_memory,
                                                       sp + w * 4,
                                                       MEMTXATTRS_UNSPECIFIED,
                                                       &v, 4);
                                    if (v >= 0x08000000 && v < 0x08400000
                                        && (v & 3) == 0) {
                                        fprintf(stderr, "  ATASTACK +0x%03x = "
                                                "0x%08x\n", w * 4, v);
                                    }
                                }
                            }
                        }
                    }
                }
            }
            prev_dst = (uint32_t)(s->dma_cur - len);
            prev_sec = ide_get_sector(is);
            int64_t sec = prev_sec;
            uint64_t n = ++rd_ops;
            /*
             * ATALBA=2 logs EVERY read with a virtual timestamp. The doubling
             * schedule answers "is the disk being read at all"; it cannot
             * answer "is the song being read WHILE it plays", which is the
             * question that separates a starved decoder from a broken one.
             */
            bool all = getenv("ATALBA")[0] == '2';

            rd_bytes += len;
            if (all || (n & (n - 1)) == 0 || n % 2000 == 0) {
                fprintf(stderr, "ATALBA +%09" PRId64 "us rd #%" PRIu64
                        " lba=%" PRId64 " (byte %" PRId64 ") len=%u dst=0x%08x"
                        "  total=%" PRIu64 " KB\n",
                        qemu_clock_get_us(QEMU_CLOCK_VIRTUAL), n, sec,
                        sec * 512, (unsigned)len, (uint32_t)(s->dma_cur - len),
                        rd_bytes / 1024);
            }
        }

        /*
         * DATA-PATH HLE track identification: tell the audio shim which disk
         * bytes the guest is reading, so it can tell which audio file is being
         * streamed and decode that same song host-side. Reads only; a no-op
         * unless HLE_AUDIO_DATAPATH is set. See ipod_classic_hle_audio.c.
         */
        if (ipod_classic_hle_audio_enabled() &&
            !(s->reg[S5L8702_ATA_CFG / 4] & S5L8702_ATA_CFG_WRITE)) {
            int64_t sec = ide_get_sector(is);
            if (sec >= 0) {
                ipod_classic_hle_audio_note_disk_read((uint64_t)sec * 512, len);
            }
        }
    }

    return is->sg.size;
}

static int ipod_classic_ata_dma_rw_buf(const IDEDMA *dma, bool is_write)
{
    /* Only the PIO fallback paths use this, and they do not come through. */
    return 0;
}

static void ipod_classic_ata_dma_set_inactive(const IDEDMA *dma, bool more)
{
    IPodClassicATAState *s = container_of(dma, IPodClassicATAState, dma);

    s->dma_cb = NULL;
}

static void ipod_classic_ata_dma_reset(const IDEDMA *dma)
{
    IPodClassicATAState *s = container_of(dma, IPodClassicATAState, dma);

    s->dma_cb = NULL;
    s->dma_cur = 0;
    s->dma_left = 0;
}

static const IDEDMAOps ipod_classic_ata_dma_ops = {
    .start_dma    = ipod_classic_ata_dma_start,
    .prepare_buf  = ipod_classic_ata_dma_prepare_buf,
    .rw_buf       = ipod_classic_ata_dma_rw_buf,
    .set_inactive = ipod_classic_ata_dma_set_inactive,
    .reset        = ipod_classic_ata_dma_reset,
};

static uint64_t ipod_classic_ata_read(void *opaque, hwaddr offset,
                                      unsigned size)
{
    IPodClassicATAState *s = opaque;

    if (ata_trace() && offset != S5L8702_ATA_PIO_READY) {
        if (ata_trace_stamp()) {
            fprintf(stderr, "t=%" PRId64 "ms ",
                    qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL));
        }
        fprintf(stderr, "ATA RD +0x%02x\n", (unsigned)offset);
    }
    switch (offset) {
    case S5L8702_ATA_CONTROL:
        /*
         * Bit 1 is the clock-down status: Rockbox's ata_power_down() writes
         * zero and then spins on it ("ATA_CONTROL = 0; while (!(ATA_CONTROL &
         * BIT(1)))"), so it has to come up as soon as the block is disabled.
         */
        if (s->reg[offset / 4] & S5L8702_ATA_CONTROL_ENABLE) {
            return s->reg[offset / 4] & ~S5L8702_ATA_CONTROL_CLOCKDOWN;
        }
        return s->reg[offset / 4] | S5L8702_ATA_CONTROL_CLOCKDOWN;

    case S5L8702_ATA_PIO_READY:
        /*
         * Bit 1 is hardwired set: a cycle may always be started, because ours
         * completes inside the store or load that begins it and there is no
         * window in which the bridge could be busy. That is a simplification
         * of real hardware, not something read off a driver -- but reporting
         * busy here has no upside and both drivers spin on it without a
         * timeout. Bit 0 is the real state: it goes up when a read cycle has
         * latched a result and down when ATA_PIO_RDATA takes it.
         */
        return S5L8702_ATA_READY_CYCLE |
               (s->rdata_valid ? S5L8702_ATA_READY_RDATA : 0);

    case S5L8702_ATA_PIO_RDATA:
        s->rdata_valid = false;
        return s->rdata;

    case S5L8702_ATA_PIO_DTR:
    case S5L8702_ATA_PIO_FED:
    case S5L8702_ATA_PIO_SCR:
    case S5L8702_ATA_PIO_LLR:
    case S5L8702_ATA_PIO_LMR:
    case S5L8702_ATA_PIO_LHR:
    case S5L8702_ATA_PIO_DVR:
    case S5L8702_ATA_PIO_CSD:
    case S5L8702_ATA_PIO_DAD:
        /*
         * The dummy read that starts the cycle. Its own value is thrown away
         * by both drivers; what matters is that the result is latched where
         * ATA_PIO_RDATA will find it.
         */
        if (offset == S5L8702_ATA_PIO_DTR) {
            s->rdata = ide_data_readw(&s->bus, 0);
        } else if (offset == S5L8702_ATA_PIO_DAD) {
            /* Alternate status: same value as the status register, no side
             * effect on the device's interrupt. */
            s->rdata = ide_status_read(&s->bus, 0);
        } else {
            s->rdata = ide_ioport_read(&s->bus, ata_ioport(offset));
        }
        s->rdata_valid = true;
        return s->rdata;

    default:
        if (offset < S5L8702_ATA_PIO_DTR) {
            return s->reg[offset / 4];
        }
        qemu_log_mask(LOG_UNIMP, "%s: read of 0x%" HWADDR_PRIx "\n", __func__,
                      offset);
        return 0;
    }
}

static void ipod_classic_ata_write(void *opaque, hwaddr offset,
                                   uint64_t value, unsigned size)
{
    IPodClassicATAState *s = opaque;

    if (ata_trace()) {
        if (ata_trace_stamp()) {
            fprintf(stderr, "t=%" PRId64 "ms ",
                    qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL));
        }
        fprintf(stderr, "ATA WR +0x%02x = 0x%08x\n", (unsigned)offset,
                (unsigned)value);
    }
    switch (offset) {
    case S5L8702_ATA_SWRST:
        s->reg[offset / 4] = value;
        if (value & 1) {
            ide_bus_reset(&s->bus);
            s->rdata_valid = false;
        }
        break;

    case S5L8702_ATA_IRQSTAT:
        s->reg[offset / 4] &= ~(uint32_t)value;
        ipod_classic_ata_update_irq(s);
        break;

    case S5L8702_ATA_IRQMASK:
        s->reg[offset / 4] = value;
        ipod_classic_ata_update_irq(s);
        break;

    case S5L8702_ATA_PIO_DTR:
        ide_data_writew(&s->bus, 0, value);
        break;
    case S5L8702_ATA_PIO_DAD:
        ide_ctrl_write(&s->bus, 0, value);
        break;
    case S5L8702_ATA_PIO_FED:
    case S5L8702_ATA_PIO_SCR:
    case S5L8702_ATA_PIO_LLR:
    case S5L8702_ATA_PIO_LMR:
    case S5L8702_ATA_PIO_LHR:
    case S5L8702_ATA_PIO_DVR:
    case S5L8702_ATA_PIO_CSD:
        /* Port 7 is the command register; the IDE core executes on write. */
        ide_ioport_write(&s->bus, ata_ioport(offset), value);
        break;

    case S5L8702_ATA_COMMAND:
        s->reg[offset / 4] = value;
        if (value & S5L8702_ATA_COMMAND_START) {
            ipod_classic_ata_dma_go(s);
        }
        if (value & S5L8702_ATA_COMMAND_STOP) {
            /*
             * Written after every transfer, successful or not, so this is a
             * "the engine is idle now" statement rather than an abort. The
             * cursor is already spent by then; clearing it is what makes a
             * stop after a timeout safe.
             */
            s->dma_left = 0;
        }
        break;

    case S5L8702_ATA_PIO_READY:
    case S5L8702_ATA_PIO_RDATA:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: 0x%" HWADDR_PRIx " is read-only\n",
                      __func__, offset);
        break;

    default:
        if (offset < S5L8702_ATA_PIO_DTR) {
            s->reg[offset / 4] = value;
        } else {
            qemu_log_mask(LOG_UNIMP, "%s: write of 0x%" PRIx64 " to 0x%"
                          HWADDR_PRIx "\n", __func__, value, offset);
        }
        break;
    }
}

static const MemoryRegionOps ipod_classic_ata_ops = {
    .read = ipod_classic_ata_read,
    .write = ipod_classic_ata_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
};

static void ipod_classic_ata_reset(DeviceState *dev)
{
    IPodClassicATAState *s = IPOD_CLASSIC_ATA(dev);

    ide_bus_reset(&s->bus);
    memset(s->reg, 0, sizeof(s->reg));
    s->rdata = 0;
    s->rdata_valid = false;
    s->intrq = false;
    s->dma_cb = NULL;
    s->dma_cur = 0;
    s->dma_left = 0;
}

static void ipod_classic_ata_realize(DeviceState *dev, Error **errp)
{
    IPodClassicATAState *s = IPOD_CLASSIC_ATA(dev);

    ide_bus_init_output_irq(&s->bus,
                            qemu_allocate_irq(ipod_classic_ata_set_irq, s, 0));

    /*
     * After ide_bus_init_output_irq, not before: the last thing it does is
     * bus->dma = &ide_dma_nop, so claiming the bus any earlier is silently
     * undone and every DMA command is accepted and then ignored.
     */
    s->dma.ops = &ipod_classic_ata_dma_ops;
    s->bus.dma = &s->dma;

    memory_region_init_io(&s->iomem, OBJECT(s), &ipod_classic_ata_ops, s,
                          TYPE_IPOD_CLASSIC_ATA, S5L8702_ATA_MEM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void ipod_classic_ata_init(Object *obj)
{
    IPodClassicATAState *s = IPOD_CLASSIC_ATA(obj);

    /* One channel, one device: the iPod has a single disk and no slave. */
    ide_bus_init(&s->bus, sizeof(s->bus), DEVICE(obj), 0, 1);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
}

void ipod_classic_ata_init_drive(DeviceState *dev, DriveInfo *hd)
{
    IPodClassicATAState *s = IPOD_CLASSIC_ATA(dev);

    if (hd) {
        ide_bus_create_drive(&s->bus, 0, hd);
    }
}

static void ipod_classic_ata_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->desc = "S5L8702 ATA controller";
    dc->realize = ipod_classic_ata_realize;
    dc->reset = ipod_classic_ata_reset;
    /* Not migratable: the latched read result is not saved. */
    dc->user_creatable = false;
}

static const TypeInfo ipod_classic_ata_info = {
    .name          = TYPE_IPOD_CLASSIC_ATA,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IPodClassicATAState),
    .instance_init = ipod_classic_ata_init,
    .class_init    = ipod_classic_ata_class_init,
};

static void ipod_classic_ata_register_types(void)
{
    type_register_static(&ipod_classic_ata_info);
}

type_init(ipod_classic_ata_register_types)
