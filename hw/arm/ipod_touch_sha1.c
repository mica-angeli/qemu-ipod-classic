#include "hw/arm/ipod_touch_sha1.h"
#include "hw/qdev-properties.h"
#include "qemu/bitops.h"
#include "qemu/bswap.h"

static uint64_t swapLong(uint64_t x) {
    x = (x & 0x00000000FFFFFFFF) << 32 | (x & 0xFFFFFFFF00000000) >> 32;
    x = (x & 0x0000FFFF0000FFFF) << 16 | (x & 0xFFFF0000FFFF0000) >> 16;
    x = (x & 0x00FF00FF00FF00FF) << 8  | (x & 0xFF00FF00FF00FF00) >> 8;
    return x;
}

/*
 * THE BLOCK ENGINE (property "block-engine", set by the iPod Classic machine).
 *
 * Everything else in this file models an accelerator that is handed a message
 * and answers with its digest. Apple's S5L8702 driver does not use it that
 * way. Read in the 2.0.5 image:
 *
 *   Crypto_SHA1InitializeHardware  0x08072994   pulse +0x04; next CONFIG is 2
 *   Crypto_SHA1FeedHardware        0x0808171c   +0x80 = 1, +0x84 = address,
 *                                               +0x8c = whole blocks only,
 *                                               +0x00 = CONFIG; the odd bytes
 *                                               stay in the driver's context
 *   Crypto_SHA1SubmitHardwareBlock 0x080cdea8   one 64-byte block the same way
 *   Crypto_SHA1FinalizeHardware    0x0807b33c   builds the 0x80 / zeros /
 *                                               bit-length padding ITSELF and
 *                                               submits it as ordinary blocks
 *   Crypto_SHA1ReadHardwareState   0x08087528   five words from +0x20
 *   Crypto_SHA1RestoreHardwareState 0x08091c60  five words TO +0x20, and the
 *                                               next CONFIG is 0xa
 *
 * So the hardware is the compression function and nothing more: it folds whole
 * blocks into a chaining state, starting from the initial vector when CONFIG
 * is 2 and from whatever the state registers hold when it is 0xa, and the
 * state can be saved and put back, which is how the driver keeps several
 * hashes going at once. Padding is the driver's business.
 *
 * The digest model pads what it is given, so every answer was the SHA-1 of an
 * already-padded message, and it ignored the restore altogether. Nothing the
 * firmware checked against a real digest could pass: the iTunesDB signature
 * (hash58) is why SHA1_ZERO and a blanked hash exist.
 *
 * The state registers hold the digest bytes in memory order -- ReadHardwareState
 * stores each word low byte first -- so each is the byte-swap of the big-endian
 * word SHA-1 is defined on.
 */
static void sha1_iv(uint32_t *h)
{
    h[0] = 0x67452301;
    h[1] = 0xefcdab89;
    h[2] = 0x98badcfe;
    h[3] = 0x10325476;
    h[4] = 0xc3d2e1f0;
}

static void sha1_block(uint32_t *h, const uint8_t *p)
{
    uint32_t w[80], a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];

    for (int i = 0; i < 16; i++) {
        w[i] = (uint32_t)p[i * 4] << 24 | p[i * 4 + 1] << 16 |
               p[i * 4 + 2] << 8 | p[i * 4 + 3];
    }
    for (int i = 16; i < 80; i++) {
        w[i] = rol32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    }
    for (int i = 0; i < 80; i++) {
        uint32_t f, k, t;

        if (i < 20) {
            f = (b & c) | (~b & d);
            k = 0x5a827999;
        } else if (i < 40) {
            f = b ^ c ^ d;
            k = 0x6ed9eba1;
        } else if (i < 60) {
            f = (b & c) | (b & d) | (c & d);
            k = 0x8f1bbcdc;
        } else {
            f = b ^ c ^ d;
            k = 0xca62c1d6;
        }
        t = rol32(a, 5) + f + e + k + w[i];
        e = d;
        d = c;
        c = rol32(b, 30);
        b = a;
        a = t;
    }
    h[0] += a;
    h[1] += b;
    h[2] += c;
    h[3] += d;
    h[4] += e;
}

/* CONFIG written with the go bit: fold in whatever the guest pointed at. */
static void sha1_block_go(IPodTouchSHA1State *s, uint32_t config)
{
    uint8_t block[0x40];

    if (!(config & 0x8)) {
        sha1_iv(s->h);                  /* 2: a new message. 0xa: carry on */
    }
    if (s->hw_buffer_dirty) {
        memcpy(block, s->hw_buffer, sizeof(block));
        memset(s->hw_buffer, 0, sizeof(s->hw_buffer));
        s->hw_buffer_dirty = false;
        sha1_block(s->h, block);
    }
    if (s->memory_mode) {
        for (uint32_t i = 0; i + 0x40 <= s->insize; i += 0x40) {
            cpu_physical_memory_read(s->memory_start + i, block, sizeof(block));
            sha1_block(s->h, block);
        }
    }
    if (getenv("SHA1TRACE")) {
        static int n;
        if (n++ < 24) {
            fprintf(stderr, "SHA1TRACE block #%d: config=0x%x %u bytes at 0x%08x"
                    " -> state %08x %08x %08x %08x %08x\n", n, config,
                    s->memory_mode ? s->insize : 0x40, s->memory_start,
                    s->h[0], s->h[1], s->h[2], s->h[3], s->h[4]);
        }
    }
}

static void flush_hw_buffer(IPodTouchSHA1State *s) {
    // Flush the hardware buffer to the state buffer and clear the buffer.
    memcpy(s->buffer + s->buffer_ind, (uint8_t *)s->hw_buffer, 0x40);
    memset(s->hw_buffer, 0, 0x40);
    s->hw_buffer_dirty = false;
    s->buffer_ind += 0x40;
}

static void sha1_reset(IPodTouchSHA1State *s)
{
	s->config = 0;
	s->memory_start = 0;
	s->memory_mode = 0;
	s->insize = 0;
	memset(&s->buffer, 0, SHA1_BUFFER_SIZE);
	memset(&s->hw_buffer, 0, 0x10 * sizeof(uint32_t));
	s->buffer_ind = 0;
	memset(&s->hashout, 0, 0x14);
	s->hw_buffer_dirty = false;
	s->hash_computed = false;
	s->memory_fed = false;
	sha1_iv(s->h);
}

static uint64_t ipod_touch_sha1_read(void *opaque, hwaddr offset, unsigned size)
{
	IPodTouchSHA1State *s = (IPodTouchSHA1State *)opaque;

    //fprintf(stderr, "%s: offset 0x%08x\n", __FUNCTION__, offset);

	switch(offset) {
		case SHA_CONFIG:
			return s->config;
		case SHA_RESET:
			return 0;
		case SHA_MEMORY_START:
			return s->memory_start;
		case SHA_MEMORY_MODE:
			return s->memory_mode;
		case SHA_INSIZE:
			return s->insize;
		/* Hash result ouput */
		case 0x20 ... 0x34:
			if (s->block_engine) {
				/* The chaining state, as it stands. SHA1_ZERO as below. */
				if (getenv("SHA1_ZERO") || offset > 0x30) {
					return 0;
				}
				return bswap32(s->h[(offset - 0x20) / 4]);
			}
			//fprintf(stderr, "Hash out %08x\n",  *(uint32_t *)&s->hashout[offset - 0x20]);
            if(!s->hash_computed) {
                /*
                 * How many of the buffered bytes are the message?
                 *
                 * Two firmwares drive this block differently and the model has
                 * to cope with both:
                 *
                 *  - The iPod Touch code this was ported from pads the message
                 *    ITSELF, so the buffer ends with the standard SHA-1 trailer
                 *    and the real length is in its last eight bytes, in bits.
                 *    Hashing everything would then pad an already-padded
                 *    message and give the wrong digest.
                 *
                 *  - The Classic's retailOS does NOT pad. MEASURED with
                 *    SHA1TRACE=1: it hands over 64 and then 4096 bytes in memory
                 *    mode with no trailer at all, so the last eight bytes read
                 *    as zero. Trusting them made data_length 0, and the engine
                 *    answered da39a3ee5e6b4b0d3255bfef95601890afd80709 -- the
                 *    SHA-1 of the EMPTY STRING -- for every request. No database
                 *    could ever pass a check against that, which is exactly why
                 *    wiring this accelerator turned the music library empty.
                 *
                 * The two cases are told apart by HOW the data arrived, not by
                 * what the last bytes look like. A first attempt trusted the
                 * trailer whenever it was a plausible length, and immediately
                 * misread 34 out of 64 bytes of ordinary data as a length --
                 * these messages look random, so any 8 bytes of them can pass
                 * for one. Memory mode means the guest already said how much to
                 * hash, via SHA_INSIZE; only the block-at-a-time register path
                 * carries a message the guest padded itself.
                 */
                uint64_t data_length;
                if (s->memory_fed) {
                    data_length = s->buffer_ind;
                } else {
                    uint64_t declared = s->buffer_ind >= 8
                        ? swapLong(((uint64_t *)s->buffer)[s->buffer_ind / 8 - 1]) / 8
                        : 0;
                    data_length = (declared > 0 && declared <= s->buffer_ind)
                        ? declared : s->buffer_ind;
                }

                SHA_CTX ctx;
                SHA1_Init(&ctx);
                SHA1_Update(&ctx, s->buffer, data_length);
                SHA1_Final(s->hashout, &ctx);
                s->hash_computed = true;

                /*
                 * SHA1TRACE=1: what is the firmware actually hashing?
                 *
                 * This matters because with this engine wired the firmware
                 * rejects the iTunesDB outright (0 songs), and there are two
                 * very different reasons that could happen: the database really
                 * fails a signature check, or THIS MODEL returns the wrong
                 * digest and nothing could ever match. The lazy length above is
                 * an assumption -- that the guest writes its own SHA-1 padding
                 * and the "length in bits" trailer -- and a real accelerator
                 * usually pads itself. Printing the declared length, the length
                 * actually fed in, the head of the data and the digest says
                 * which of the two is happening, and whether the input is even
                 * the database (an mhbd magic would be unmistakable).
                 */
                if (getenv("SHA1TRACE")) {
                    static int n;
                    if (n++ < 12) {
                        fprintf(stderr,
                                "SHA1TRACE #%d: buffered=%u bytes, trailer says %"
                                PRIu64 ", memory_mode=%u start=0x%08x insize=%u\n",
                                n, s->buffer_ind, data_length,
                                s->memory_mode, s->memory_start, s->insize);
                        fprintf(stderr, "  head: %02x %02x %02x %02x %02x %02x %02x %02x"
                                "   '%c%c%c%c'\n",
                                s->buffer[0], s->buffer[1], s->buffer[2], s->buffer[3],
                                s->buffer[4], s->buffer[5], s->buffer[6], s->buffer[7],
                                isprint(s->buffer[0]) ? s->buffer[0] : '.',
                                isprint(s->buffer[1]) ? s->buffer[1] : '.',
                                isprint(s->buffer[2]) ? s->buffer[2] : '.',
                                isprint(s->buffer[3]) ? s->buffer[3] : '.');
                        fprintf(stderr, "  digest:");
                        for (int i = 0; i < 20; i++) {
                            fprintf(stderr, "%02x", s->hashout[i]);
                        }
                        fprintf(stderr, "\n");
                    }
                }
            }

			/*
			 * SHA1_ZERO=1: keep the device, but answer the digest as the
			 * unimplemented stub used to -- all zeros.
			 *
			 * This is the discriminator for the one difference left between the
			 * two configurations. With SHA1_OFF the library loads and playback
			 * starts; with the engine wired the boot is clean but the library is
			 * empty. Both now back the whole 1 MB window, so the only thing that
			 * still differs is what these registers answer. If zeroing just the
			 * digest brings the library back, the firmware is acting on the
			 * VALUE -- it compares it against something -- and the empty library
			 * is a verdict, not a side effect. If it does not, the difference is
			 * in another register and the digest is a red herring.
			 */
			if (getenv("SHA1_ZERO")) {
				return 0;
			}
			return *(uint32_t *)&s->hashout[offset - 0x20];
	}

    return 0;
}

static void ipod_touch_sha1_write(void *opaque, hwaddr offset, uint64_t value, unsigned size)
{
    IPodTouchSHA1State *s = (IPodTouchSHA1State *)opaque;

    //fprintf(stderr, "%s: offset 0x%08x value 0x%08x\n", __FUNCTION__, offset, value);

	if (s->block_engine) {
		switch (offset) {
		case SHA_CONFIG:
			if (value & 0x2) {
				sha1_block_go(s, value);
			}
			/* Never busy: the driver polls bit 0 of this after every go. */
			s->config = 0;
			return;
		case 0x20 ... 0x30:
			s->h[(offset - 0x20) / 4] = bswap32(value);
			return;
		default:
			break;                      /* the plain registers, as below */
		}
	}

	switch(offset) {
		case SHA_CONFIG:
			if(value == 0x2 || value == 0xa)
			{
                if(s->hw_buffer_dirty) {
                    flush_hw_buffer(s);
                }

				if(s->memory_mode)
				{
					// we are in memory mode - gradually add the memory to the buffer
					for(int i = 0; i < s->insize / 0x40; i++) {
						cpu_physical_memory_read(s->memory_start + i * 0x40, s->buffer + s->buffer_ind, 0x40);
						s->buffer_ind += 0x40;
					}
					/*
					 * Remember that the guest sized this message itself. The
					 * digest read needs to know: a message that arrived this
					 * way is NOT padded, so its last eight bytes are data, not
					 * a length.
					 */
					s->memory_fed = true;
				}
			} else {
				s->config = value;
			}
			break;
		case SHA_RESET:
			sha1_reset(s);
			break;
		case SHA_MEMORY_START:
			s->memory_start = value;
			break;
		case SHA_MEMORY_MODE:
			s->memory_mode = value;
			break;
		case SHA_INSIZE:
            assert(value <= SHA1_BUFFER_SIZE);
			s->insize = value;
			break;
		case 0x40 ... 0x7c:
            // write to the hardware buffer
            s->hw_buffer[(offset - 0x40) / 4] |= value;
            s->hw_buffer_dirty = true;
			break;
	}
}

static const MemoryRegionOps sha1_ops = {
    .read = ipod_touch_sha1_read,
    .write = ipod_touch_sha1_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void ipod_touch_sha1_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    DeviceState *dev = DEVICE(sbd);
    IPodTouchSHA1State *s = IPOD_TOUCH_SHA1(dev);

    memory_region_init_io(&s->iomem, obj, &sha1_ops, s, "sha1", 0x100);
    sysbus_init_mmio(sbd, &s->iomem);

    sha1_reset(s);
}

static Property ipod_touch_sha1_properties[] = {
    DEFINE_PROP_BOOL("block-engine", IPodTouchSHA1State, block_engine, false),
    DEFINE_PROP_END_OF_LIST(),
};

static void ipod_touch_sha1_class_init(ObjectClass *klass, void *data)
{
    device_class_set_props(DEVICE_CLASS(klass), ipod_touch_sha1_properties);
}

static const TypeInfo ipod_touch_sha1_info = {
    .name          = TYPE_IPOD_TOUCH_SHA1,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(IPodTouchSHA1State),
    .instance_init = ipod_touch_sha1_init,
    .class_init    = ipod_touch_sha1_class_init,
};

static void ipod_touch_machine_types(void)
{
    type_register_static(&ipod_touch_sha1_info);
}

type_init(ipod_touch_machine_types)
