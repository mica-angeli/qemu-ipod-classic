/*
 * HLE audio DATA-PATH shim -- iPod classic 6G (S5L8702) retailOS.
 *
 * WHAT THIS IS (and is not)
 * -------------------------
 * A High-Level Emulation of the audio DATA PATH, in the same spirit as the 5G
 * VideoCore HLE (hw/arm/ipod_video/ipod_video_vc.c). Twelve investigation
 * sessions (wiki/19 section 11) proved by measurement that the retailOS MeCCA
 * audio pipeline is a SYMMETRIC bootstrap deadlock that does not self-prime
 * under a faithful emulation: the guest AAC decoder (task t2a) and the render
 * engine (task t25) each gate on the other half's in-regime state (a production
 * credit the decoder needs, and a render descriptor the engine needs), and
 * neither is primed to start without the other. copy32 (0x08016eb4) -- the
 * store that would put the first real PCM sample into the audio buffer -- never
 * executes; the I2S clocks silence from end to end.
 *
 * The output half of the pipeline is HEALTHY and identical to Rockbox's: the
 * firmware brings up the CS42L55 codec over I2C, starts the I2S transmitter,
 * and programs DMAC0 channel 1 to stream a ping-pong buffer (IRAM ~0x22008c5c/
 * 0x22009c5c) to the I2S TX FIFO (0x3ca00010) on a self-relinking LLI chain,
 * paced by the sample clock. That DMA runs continuously; all it ever carries is
 * the silence the deadlocked decoder left in the buffer.
 *
 * So this shim supplies what the deadlock withholds -- and ONLY that. It:
 *   1. parses the guest's OWN FAT32 partition and identifies the audio file
 *      (MP3/M4A) the guest is streaming, from the guest's OWN disk reads (the
 *      ATA DMA read that lands in a file's data), so we decode the SAME song the
 *      guest chose. The guest's play is non-deterministic (wiki/19 11.11 regime
 *      A/B); when it never streams the file for us to key on, a fallback decodes
 *      the disc's single unambiguous song at the play-start edge;
 *   2. decodes that file host-side with ffmpeg to 16-bit stereo 44100 PCM --
 *      the ONLY host-side step, and it reads the SAME raw disk image the guest
 *      has on -drive, so the bytes are the real song;
 *   3. writes that PCM into the audio DMA channel's current source segment,
 *      one burst ahead of each paced drain, so the guest's OWN DMA transports
 *      the real song to the guest's OWN I2S FIFO -> CS42L55 -> speakers.
 *
 * It is NOT a bypass of the output: the audio leaves through the guest's DMA and
 * I2S exactly as Rockbox's does. It does NOT inject a canned or foreign file:
 * the PCM is the guest's own song, verifiable by correlating the captured WAV
 * with the same file decoded independently (measured Pearson ~1.0, contiguous,
 * in tune). It does NOT patch firmware. What it replaces is the deadlocked
 * guest-side software decode, decoding host-side instead -- honestly HLE.
 *
 * LIMIT (wiki/19 11.19): the guest's own render engine stays deadlocked
 * (copy32=0), so the Now Playing screen does not advance and, without
 * HLE_AUDIO_SUSTAIN, the guest's deadlock-watchdog reboots ~15 s in. This shim
 * breaks the SILENCE (the real song plays out the guest's I2S); it does not
 * break the engine deadlock (that is the wall of wiki/19 11.12-11.18).
 *
 * Gated entirely by HLE_AUDIO_DATAPATH. Unset: this file installs nothing and
 * the machine is byte-for-byte the baseline (Rockbox and navtest untouched).
 * Environment:
 *   HLE_AUDIO_DATAPATH=1   enable the shim
 *   HLE_AUDIO_DISK=/path   the raw disk image to decode from (the -drive file)
 *   HLE_AUDIO_TRACK=N      force track index N (skip auto-identification)
 *   HLE_AUDIO_SUSTAIN=1    suppress the guest's deadlock-watchdog reboot so the
 *                          whole song plays (guest CPU then idles in its reset
 *                          spin; audio continues via the DMA + host fill)
 *   HLE_AUDIO_TRACE=1      progress on stderr
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/thread.h"
#include "qemu/atomic.h"
#include "qemu/timer.h"
#include "hw/core/cpu.h"
#include "exec/cpu-common.h"
#include "hw/dma/pl080.h"
#include "hw/arm/ipod_classic/ipod_classic_hle_audio.h"
#include "hw/arm/ipod_classic/ipod_classic_i2s.h"

#include <fcntl.h>
#include <unistd.h>

/*
 * I2S transmit-sample count past which a read of the FAT data region is taken to
 * be the user opening a file to PLAY (not the early boot filesystem scan). At
 * ~88200 samples/s this is ~34 s -- comfortably after the boot scan (~20 s) and
 * before playback navigation completes. Only used for the single-song fallback.
 */
#define HLE_BOOT_TX_SAMPLES  (3u * 1000u * 1000u)

/*
 * How much of one file the guest must have read before the disk-read fallback
 * takes it for the track being played. It was 8192, the size of one readahead,
 * and MEASURED on a five-song disk that is how the mount-time scan -- a 32 KiB
 * read that runs from the directory clusters into the head of the first file --
 * got the first file on the disk "playing" from boot -- and since it repeats
 * that same read, a bigger total alone does not stop it, so the head of each
 * file is not counted at all. A track being streamed passes both in its first
 * seconds; a scan never does.
 */
#define HLE_DISK_PICK_BYTES  (256u * 1024u)
#define HLE_DISK_PICK_SKIP   (64u * 1024u)

/* A stream position further than this from the PCM cursor is a seek. */
#define HLE_STREAM_SEEK_BYTES  (5u * 44100u * 4u)

/* No frame handed to the engine for this long means the guest is not playing. */
#define HLE_STREAM_IDLE_NS  (400 * 1000 * 1000LL)

/* The DMAC0 request line the I2S TX FIFO drives (mem->peri, flow 1). */
#define HLE_IIS0_TX            0xA

/* PL080 per-channel config/control bit decode (see hw/dma/pl080.c). */
#define HLE_CCONF_E            0x00001u
#define HLE_CCONF_SRCID_SH     1
#define HLE_CCONF_DSTID_SH     6
#define HLE_CCONF_ID_MASK      0x1f
#define HLE_CCTRL_SWIDTH_SH    18
#define HLE_CCTRL_SWIDTH_MASK  0x7
#define HLE_CCTRL_XSIZE_MASK   0xfffu

/* Sanity cap on one segment fill. A ping-pong half is ~4 KiB; 64 KiB is slack. */
#define HLE_FILL_MAX_BYTES     (64u * 1024u)

#define HLE_MAX_TRACKS         64

typedef struct {
    uint64_t file_off;      /* byte offset of the file's data on the image    */
    uint64_t size;          /* file size in bytes                             */
    char     name[16];      /* 8.3 short name (diagnostic)                    */
} HLETrack;

typedef struct {
    bool        enabled;
    bool        trace;
    bool        trace_disk;         /* HLE_AUDIO_TRACE=2: log every disk read  */
    ARMCPU     *cpu;
    PL080State *dmac0;
    IPodClassicI2SState *i2s;    /* for tx_samples (boot-vs-play gate)         */
    uint64_t    data_start;      /* FAT data area byte offset                  */
    char        disk_path[1024];
    int         force_track;        /* HLE_AUDIO_TRACK, or -1                  */

    /* Track table, filled by the worker's disk scan. */
    HLETrack    tracks[HLE_MAX_TRACKS];
    int         ntracks;
    bool        scan_done;          /* worker finished scanning                */

    /* Per-track whole-file bytes read by the guest (for auto-identification). */
    uint64_t    file_read[HLE_MAX_TRACKS];

    /* Chosen track and its decoded PCM (16-bit stereo interleaved 44100). */
    int         chosen;             /* -1 until identified                     */
    uint8_t    *pcm;                /* malloc'd, owned by the shim             */
    size_t      pcm_len;            /* bytes                                   */
    size_t      pcm_cursor;         /* bytes already fed                       */
    double      start_frac;         /* where in the track a decode starts, 0..1 */
    bool        stream_seen;        /* the engine has been handed a stream     */
    int64_t     stream_ns;          /* virtual time of the last such hand-over */
    int         pcm_ready;          /* qatomic: 1 once pcm is decoded          */

    bool        filled;             /* fill_dma staged a burst; commit pending  */
    uint32_t    fill_swidth;        /* source element width of the staged burst */
    bool        unique_song;        /* all audio files are the same song        */
    bool        play_seen;          /* real-playback clock re-config seen        */
    bool        sustain;            /* HLE_AUDIO_SUSTAIN: suppress deadlock reset*/

    /* Player controls (driven by the wheel buttons via the public API). */
    bool        paused;             /* feed+drain frozen; UI position stops     */
    int         req_track;          /* worker: track index requested for change */
    int         decode_gen;         /* bumped on each decode request           */
    int         decoded_gen;        /* last generation the worker installed     */

    /* Current track tags (from ffprobe), for the Now Playing overlay. */
    char        cur_title[96];
    char        cur_artist[96];
    char        cur_album[96];

    QemuThread  worker;
    QemuMutex   lock;
    QemuCond    cond;               /* worker waits for a decode request        */
    bool        started;            /* play-start edge seen                    */
} HLEAudioState;

static HLEAudioState hle;

/*
 * Decode for the engine rather than for the host feed: the file's own rate
 * and channels, frame-aligned. Set once by the SM1 model before any track is
 * decoded (SM1_SYNC + SM1_DECODE).
 */
static bool hle_native;
void ipod_classic_hle_audio_set_native(bool on) { hle_native = on; }

static bool hle_env_on(const char *name)
{
    const char *e = getenv(name);
    return e && *e && *e != '0';
}

/* ------------------------------------------------------------ FAT scan ----- */
/*
 * The guest plays music from a FAT filesystem: an MBR partition holding an iPod
 * FAT32 volume with music under /iPod_Control/Music/. The song data lives at the
 * files' cluster offsets, NOT wherever a raw container header happens to sit on
 * the image. So we parse the FAT exactly as the guest's own filesystem does,
 * enumerate the audio files, and record each one's byte offset and size. Plain
 * pread of the same raw image the guest has on -drive; no guest state touched.
 */

static uint16_t hle_le16(const uint8_t *p) { return p[0] | (p[1] << 8); }
static uint32_t hle_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

typedef struct {
    int      fd;
    uint32_t bps;           /* bytes per sector                               */
    uint32_t cbytes;        /* bytes per cluster                              */
    uint64_t data_start;    /* byte offset of cluster 2                       */
    uint8_t *fat;           /* the FAT table, in memory                       */
    uint32_t fat_bytes;
} HLEFat;

static uint32_t hle_fat_next(HLEFat *fs, uint32_t c)
{
    if ((uint64_t)c * 4 + 4 > fs->fat_bytes) {
        return 0x0FFFFFFF;
    }
    return hle_le32(fs->fat + c * 4) & 0x0FFFFFFF;
}

static uint64_t hle_clus_off(HLEFat *fs, uint32_t c)
{
    return fs->data_start + (uint64_t)(c - 2) * fs->cbytes;
}

/* Is the file's cluster chain a single contiguous run? (Ours are.) */
static bool hle_chain_contig(HLEFat *fs, uint32_t clus, uint32_t *nclus)
{
    uint32_t c = clus, n = 0;
    bool contig = true;
    while (c >= 2 && c < 0x0FFFFFF8 && n < 4000000) {
        uint32_t nc = hle_fat_next(fs, c);
        n++;
        if (nc >= 2 && nc < 0x0FFFFFF8 && nc != c + 1) {
            contig = false;
        }
        if (nc == c) {
            break;
        }
        c = nc;
    }
    *nclus = n;
    return contig;
}

/* Read a whole directory (following its cluster chain) into a malloc'd buffer. */
static uint8_t *hle_read_dir(HLEFat *fs, uint32_t clus, size_t *out_len)
{
    uint8_t *buf = NULL;
    size_t len = 0;
    uint32_t c = clus, guard = 0;

    while (c >= 2 && c < 0x0FFFFFF8 && guard++ < 100000) {
        buf = g_realloc(buf, len + fs->cbytes);
        if (pread(fs->fd, buf + len, fs->cbytes, hle_clus_off(fs, c))
                != (ssize_t)fs->cbytes) {
            break;
        }
        len += fs->cbytes;
        c = hle_fat_next(fs, c);
    }
    *out_len = len;
    return buf;
}

static bool hle_is_audio_ext(const uint8_t *e83)
{
    /* 8.3 extension is bytes 8..10, space-padded uppercase. */
    return (memcmp(e83 + 8, "MP3", 3) == 0) || (memcmp(e83 + 8, "M4A", 3) == 0) ||
           (memcmp(e83 + 8, "AAC", 3) == 0) || (memcmp(e83 + 8, "M4P", 3) == 0);
}

static void hle_walk_dir(HLEFat *fs, uint32_t clus, int depth)
{
    size_t len = 0;
    uint8_t *dir;

    if (depth > 8) {
        return;
    }
    dir = hle_read_dir(fs, clus, &len);
    if (!dir) {
        return;
    }
    for (size_t i = 0; i + 32 <= len && hle.ntracks < HLE_MAX_TRACKS; i += 32) {
        uint8_t *e = dir + i;
        uint8_t attr = e[11];
        uint32_t fc, sz;

        if (e[0] == 0x00) {
            break;              /* end of directory */
        }
        if (e[0] == 0xE5 || attr == 0x0F || e[0] == 0x2E) {
            continue;           /* free / LFN / "." ".." */
        }
        fc = ((uint32_t)hle_le16(e + 20) << 16) | hle_le16(e + 26);
        sz = hle_le32(e + 28);
        if (attr & 0x10) {
            if (fc >= 2) {
                hle_walk_dir(fs, fc, depth + 1);   /* subdirectory */
            }
        } else if (hle_is_audio_ext(e) && fc >= 2 && sz > 0) {
            uint32_t nclus = 0;
            bool contig = hle_chain_contig(fs, fc, &nclus);
            HLETrack *t = &hle.tracks[hle.ntracks];
            t->file_off = hle_clus_off(fs, fc);
            t->size = sz;
            memcpy(t->name, e, 11);
            t->name[11] = '\0';
            hle.ntracks++;
            if (hle.trace) {
                fprintf(stderr, "HLE_AUDIO: file%d '%.11s' off=%llu size=%u "
                        "clus=%u nclus=%u%s\n", hle.ntracks - 1, e,
                        (unsigned long long)t->file_off, sz, fc, nclus,
                        contig ? "" : " NON-CONTIGUOUS!");
            }
        }
    }
    g_free(dir);
}

/*
 * Parse the MBR + FAT32 and enumerate the audio files into hle.tracks.
 */
static void hle_scan_disk(void)
{
    int fd = open(hle.disk_path, O_RDONLY);
    uint8_t mbr[512], bpb[512];
    HLEFat fs = { 0 };
    uint64_t part_off = 0;
    uint32_t rsvd, nfat, fatsz, rootclus, spc;

    if (fd < 0) {
        fprintf(stderr, "HLE_AUDIO: cannot open disk '%s'\n", hle.disk_path);
        return;
    }
    fs.fd = fd;

    /*
     * Find the FAT boot sector. The MBR partition LBAs are in the disk's LOGICAL
     * sector size, which on this iPod image is 4096, not 512 (a "4Kn" disk:
     * part LBA 22696 * 4096 = 92962816 = the FAT32 boot sector). So for each
     * partition entry try both LBA*512 and LBA*4096, plus 0 (superfloppy), and
     * use the first candidate that has a valid FAT boot signature (jump opcode +
     * 0x55AA). Disk-layout-agnostic; no hard-coded offset.
     */
    if (pread(fd, mbr, 512, 0) != 512) {
        close(fd);
        return;
    }
    {
        uint64_t cands[9];
        int nc = 0;
        cands[nc++] = 0;                         /* superfloppy */
        for (int p = 0; p < 4; p++) {
            uint32_t lba = hle_le32(mbr + 446 + p * 16 + 8);
            if (lba != 0) {
                cands[nc++] = (uint64_t)lba * 512;
                cands[nc++] = (uint64_t)lba * 4096;
            }
        }
        for (int i = 0; i < nc; i++) {
            if (pread(fd, bpb, 512, cands[i]) == 512 &&
                (bpb[0] == 0xEB || bpb[0] == 0xE9) &&
                bpb[510] == 0x55 && bpb[511] == 0xAA &&
                hle_le16(bpb + 11) != 0 && bpb[13] != 0) {
                part_off = cands[i];
                break;
            }
        }
    }
    if (part_off == 0 && !((bpb[0] == 0xEB || bpb[0] == 0xE9) &&
                           bpb[510] == 0x55 && bpb[511] == 0xAA)) {
        fprintf(stderr, "HLE_AUDIO: no FAT boot sector found\n");
        close(fd);
        return;
    }
    fs.bps  = hle_le16(bpb + 11);
    spc     = bpb[13];
    rsvd    = hle_le16(bpb + 14);
    nfat    = bpb[16];
    fatsz   = hle_le32(bpb + 36);           /* FAT32 32-bit FAT size          */
    rootclus = hle_le32(bpb + 44);
    if (fs.bps == 0 || spc == 0 || fatsz == 0) {
        fprintf(stderr, "HLE_AUDIO: unsupported FAT (bps=%u spc=%u fatsz=%u)\n",
                fs.bps, spc, fatsz);
        close(fd);
        return;
    }
    fs.cbytes = fs.bps * spc;
    fs.fat_bytes = fatsz * fs.bps;
    uint64_t fat_start = part_off + (uint64_t)rsvd * fs.bps;
    fs.data_start = fat_start + (uint64_t)nfat * fatsz * fs.bps;
    hle.data_start = fs.data_start;
    fs.fat = g_malloc(fs.fat_bytes);
    if (pread(fd, fs.fat, fs.fat_bytes, fat_start) != (ssize_t)fs.fat_bytes) {
        fprintf(stderr, "HLE_AUDIO: cannot read FAT\n");
        g_free(fs.fat);
        close(fd);
        return;
    }

    if (hle.trace) {
        fprintf(stderr, "HLE_AUDIO: FAT32 part@%llu bps=%u spc=%u data@%llu "
                "rootclus=%u\n", (unsigned long long)part_off, fs.bps, spc,
                (unsigned long long)fs.data_start, rootclus);
    }

    hle_walk_dir(&fs, rootclus, 0);

    /*
     * Are all the audio files the same song? (Compare the first 64 KB.) On this
     * iPod image libgpod wrote four byte-identical copies of one track, so the
     * "played song" is unambiguous even when the guest's non-deterministic play
     * (wiki/19 11.11 regime A/B) never streams the file's data for us to key on.
     * Only then does the play-time fallback (below) decode a file we did not see
     * the guest read -- and it is provably the only song present, so it is still
     * the guest's real song.
     */
    if (hle.ntracks == 1) {
        hle.unique_song = true;
    } else if (hle.ntracks > 1) {
        uint8_t *a = g_malloc(65536), *b = g_malloc(65536);
        hle.unique_song = (pread(fd, a, 65536, hle.tracks[0].file_off) == 65536);
        for (int k = 1; k < hle.ntracks && hle.unique_song; k++) {
            if (pread(fd, b, 65536, hle.tracks[k].file_off) != 65536 ||
                memcmp(a, b, 65536) != 0) {
                hle.unique_song = false;
            }
        }
        g_free(a);
        g_free(b);
    }

    g_free(fs.fat);
    close(fd);

    if (hle.trace) {
        fprintf(stderr, "HLE_AUDIO: FAT scan found %d audio file(s)%s\n",
                hle.ntracks, hle.unique_song ? " (all the same song)" : "");
    }
}

/* ---------------------------------------------------------- host decode ---- */

/*
 * Extract the chosen track from the raw disk image and decode it host-side with
 * ffmpeg to 16-bit stereo 44100 PCM. Runs on the worker thread so the ~1 s
 * decode never stalls the vCPU or the drain timer.
 */
/* Read one format/stream tag from a media file via ffprobe into out[0..n).
 * Leaves out[0]=0 on any failure. */
static void hle_probe_tag(const char *file, const char *key, char *out, size_t n)
{
    char cmd[1400];
    FILE *pp;
    size_t got = 0;

    out[0] = '\0';
    snprintf(cmd, sizeof(cmd),
             "ffprobe -v quiet -show_entries format_tags=%s:stream_tags=%s "
             "-of default=nw=1:nk=1 '%s' 2>/dev/null", key, key, file);
    pp = popen(cmd, "r");
    if (!pp) {
        return;
    }
    got = fread(out, 1, n - 1, pp);
    pclose(pp);
    out[got] = '\0';
    /* Trim trailing newline(s)/CR and any extra lines (keep the first). */
    for (size_t i = 0; i < got; i++) {
        if (out[i] == '\n' || out[i] == '\r') {
            out[i] = '\0';
            break;
        }
    }
}

static uint8_t *hle_decode_track(int k, size_t *out_used)
{
    HLETrack *t;
    char tmp[] = "/tmp/hle_track_XXXXXX.m4a";
    char cmd[1200];
    int tfd, dfd;
    uint64_t off, len;
    uint8_t *fbuf;
    FILE *pp;
    uint8_t *pcm = NULL;
    size_t cap = 0, used = 0;

    *out_used = 0;
    if (k < 0 || k >= hle.ntracks) {
        return NULL;
    }
    t = &hle.tracks[k];
    off = t->file_off;
    len = t->size;

    /* Extract the whole audio file (contiguous on this image) to a temp. */
    tfd = mkstemps(tmp, 4);
    if (tfd < 0) {
        fprintf(stderr, "HLE_AUDIO: mkstemps failed\n");
        return NULL;
    }
    dfd = open(hle.disk_path, O_RDONLY);
    if (dfd < 0) {
        close(tfd);
        return NULL;
    }
    fbuf = g_malloc(1u << 20);
    for (uint64_t done = 0; done < len; ) {
        size_t want = MIN((uint64_t)(1u << 20), len - done);
        ssize_t got = pread(dfd, fbuf, want, off + done);
        if (got <= 0) {
            break;
        }
        if (write(tfd, fbuf, got) != got) {
            break;
        }
        done += got;
    }
    g_free(fbuf);
    close(dfd);
    close(tfd);

    /* Decode to raw s16le stereo 44100 on stdout. */
    if (hle_native) {
        /*
         * For the engine (ipod_classic_hle_audio_frame_pcm): the decoder's
         * own output, in the file's own rate and channel count, with nothing
         * trimmed. skip_manual stops ffmpeg dropping the encoder/decoder
         * priming samples, so audio frame N is exactly samples
         * [N * spf, (N + 1) * spf) -- MEASURED on a LAME file of 9355 frames:
         * 10775808 samples = 9354 * 1152, against 10773924 by default (1105
         * trimmed). The firmware trims the priming itself.
         */
        snprintf(cmd, sizeof(cmd),
                 "ffmpeg -v error -flags2 +skip_manual -i '%s' -f s16le - "
                 "2>/dev/null", tmp);
    } else {
        snprintf(cmd, sizeof(cmd),
                 "ffmpeg -v error -i '%s' -f s16le -ar 44100 -ac 2 - "
                 "2>/dev/null", tmp);
    }
    pp = popen(cmd, "r");
    if (!pp) {
        fprintf(stderr, "HLE_AUDIO: popen(ffmpeg) failed\n");
        unlink(tmp);
        return NULL;
    }
    for (;;) {
        if (used + (1u << 16) > cap) {
            cap = cap ? cap * 2 : (1u << 20);
            pcm = g_realloc(pcm, cap);
        }
        size_t got = fread(pcm + used, 1, 1u << 16, pp);
        used += got;
        if (got == 0) {
            break;
        }
    }
    pclose(pp);

    /* Real title/artist/album for the Now Playing overlay (ffprobe; no new
     * dependency). Fall back to the FAT 8.3 name when a tag is absent. */
    hle_probe_tag(tmp, "title",  hle.cur_title,  sizeof(hle.cur_title));
    hle_probe_tag(tmp, "artist", hle.cur_artist, sizeof(hle.cur_artist));
    hle_probe_tag(tmp, "album",  hle.cur_album,  sizeof(hle.cur_album));
    if (!hle.cur_title[0]) {
        g_strlcpy(hle.cur_title, t->name, sizeof(hle.cur_title));
    }

    unlink(tmp);

    if (used == 0) {
        fprintf(stderr, "HLE_AUDIO: ffmpeg produced no PCM for track %d\n", k);
        g_free(pcm);
        return NULL;
    }

    *out_used = used;
    fprintf(stderr, "HLE_AUDIO: decoded track %d -> %zu PCM bytes "
            "(%.1f s @ 44100 stereo)\n", k, used, used / 4.0 / 44100.0);
    return pcm;
}

/*
 * Request the worker to (re)decode track k. Clears the current PCM so feeding
 * stops (clean silence) during the ~1s decode; a generation counter coalesces
 * rapid next/next presses so only the latest wins. Called for the initial pick
 * and for NEXT/PREV track changes.
 */
static void hle_request_decode(int k)
{
    qemu_mutex_lock(&hle.lock);
    hle.chosen = k;
    hle.req_track = k;
    hle.decode_gen++;
    hle.pcm_cursor = 0;
    hle.start_frac = 0.0;
    hle.paused = false;
    qatomic_set(&hle.pcm_ready, 0);
    qemu_cond_signal(&hle.cond);
    qemu_mutex_unlock(&hle.lock);
}

static void *hle_worker(void *arg)
{
    hle_scan_disk();
    qatomic_set(&hle.scan_done, true);

    if (hle.force_track >= 0 && hle.force_track < hle.ntracks) {
        hle_request_decode(hle.force_track);
        if (hle.trace) {
            fprintf(stderr, "HLE_AUDIO: forced track %d\n", hle.force_track);
        }
    }

    /* Decode-request loop: wait for a request, decode the latest-requested
     * track into a fresh buffer (no lock held during the slow ffmpeg), then
     * install it -- unless a newer request arrived meanwhile (generation). */
    for (;;) {
        int k, gen;
        uint8_t *pcm;
        size_t used = 0;

        qemu_mutex_lock(&hle.lock);
        while (hle.decode_gen == hle.decoded_gen) {
            qemu_cond_wait(&hle.cond, &hle.lock);
        }
        k = hle.req_track;
        gen = hle.decode_gen;
        qemu_mutex_unlock(&hle.lock);

        pcm = hle_decode_track(k, &used);

        qemu_mutex_lock(&hle.lock);
        if (gen == hle.decode_gen && pcm) {
            g_free(hle.pcm);
            hle.pcm = pcm;
            hle.pcm_len = used;
            hle.pcm_cursor = (size_t)(hle.start_frac * (double)used) & ~(size_t)3;
            hle.decoded_gen = gen;
            qatomic_set(&hle.pcm_ready, 1);
            qemu_mutex_unlock(&hle.lock);
            fprintf(stderr, "HLE_AUDIO: now feeding track %d (gen %d)\n", k, gen);
        } else {
            /* superseded by a newer request, or decode failed */
            hle.decoded_gen = gen;
            qemu_mutex_unlock(&hle.lock);
            g_free(pcm);
        }
    }
    return NULL;
}

/* ------------------------------------------------------------ callbacks ---- */

static void hle_pick_track(int k, uint64_t byte_off, const char *why)
{
    bool first;

    qemu_mutex_lock(&hle.lock);
    first = (hle.chosen < 0);
    qemu_mutex_unlock(&hle.lock);
    if (first) {
        if (hle.trace) {
            fprintf(stderr, "HLE_AUDIO: identified guest track %d (%s, disk "
                    "offset %llu)\n", k, why, (unsigned long long)byte_off);
        }
        hle_request_decode(k);
    }
}

void ipod_classic_hle_audio_note_disk_read(uint64_t byte_off, uint32_t len)
{
    if (!hle.enabled || !qatomic_read(&hle.scan_done)) {
        return;
    }
    /*
     * HLE_AUDIO_TRACE=2: every guest disk read, with the virtual time and the
     * file it falls in. "Is the guest reading, and how fast" is the first
     * question whenever a track is slow to start.
     */
    if (hle.trace_disk) {
        int in = -1;
        for (int k = 0; k < hle.ntracks; k++) {
            if (byte_off + len > hle.tracks[k].file_off &&
                byte_off < hle.tracks[k].file_off + hle.tracks[k].size) {
                in = k;
                break;
            }
        }
        fprintf(stderr, "HLE_AUDIO: disk read t=%" PRId64 "ms off=%llu len=%u "
                "file=%d (+%lld)\n",
                qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL),
                (unsigned long long)byte_off, len, in,
                in >= 0 ? (long long)(byte_off - hle.tracks[in].file_off) : 0LL);
    }
    if (qatomic_read(&hle.pcm_ready) || hle.chosen >= 0 || hle.stream_seen) {
        return;
    }

    /*
     * Credit the read to the file it overlaps MOST, not to the one its first
     * byte lands in. Files are packed back to back, so a readahead that opens
     * a track starts in the tail cluster of the previous file: MEASURED on a
     * five-song disk, the 32 KiB read that opened file1 began 2048 bytes
     * before it, and keying on the start address played file0 instead.
     */
    int best = -1;
    uint64_t best_ov = 0;
    for (int k = 0; k < hle.ntracks; k++) {
        HLETrack *t = &hle.tracks[k];
        /* Past the head only: a scan re-reads the first clusters for ever. */
        uint64_t lo = MAX(byte_off, t->file_off + HLE_DISK_PICK_SKIP);
        uint64_t hi = MIN(byte_off + len, t->file_off + t->size);
        if (hi > lo && hi - lo > best_ov) {
            best_ov = hi - lo;
            best = k;
        }
    }
    for (int k = best; k >= 0 && k == best; k++) {
        HLETrack *t = &hle.tracks[k];
        {
            hle.file_read[k] += best_ov;
            if (hle.trace) {
                fprintf(stderr, "HLE_AUDIO: read file%d '%s' off=%llu len=%u "
                        "(tot=%llu)\n", k, t->name, (unsigned long long)byte_off,
                        len, (unsigned long long)hle.file_read[k]);
            }
            /* The guest is streaming this file's data -- the played track.
             * Low threshold: the MeCCA deadlock can cut the stream short. */
            if (hle.file_read[k] >= HLE_DISK_PICK_BYTES) {
                hle_pick_track(k, byte_off, "file stream");
            }
            return;
        }
    }

    /*
     * Fallback for the guest's non-deterministic play (wiki/19 11.11 regime B),
     * where it never streams the file's data for us to key on: a read of the FAT
     * data area AFTER boot (I2S has clocked > ~34 s, so this is the user opening
     * a file to play, not the early boot filesystem scan) means playback -- so
     * decode the disc's single unambiguous song. Only when the library is one
     * song (all files identical), so the decoded track is provably the guest's.
     */
    if (hle.unique_song && hle.chosen < 0 && hle.data_start &&
        byte_off >= hle.data_start && hle.i2s &&
        hle.i2s->tx_samples > HLE_BOOT_TX_SAMPLES) {
        hle_pick_track(0, byte_off, "play (FAT access past boot)");
    }
}

bool ipod_classic_hle_audio_enabled(void)
{
    return hle.enabled;
}

bool ipod_classic_hle_audio_active(void)
{
    /* Reset suppression is opt-in (HLE_AUDIO_SUSTAIN): only then do we hold off
     * the guest's deadlock-watchdog reboot so the whole song can play.
     *
     * Consider the HLE "active" from the moment a track is CHOSEN (hle.chosen >=
     * 0), not only once the host decode has finished (pcm_ready). The decode
     * runs async on a worker thread and can take longer than the guest's ~15 s
     * deadlock watchdog: measured order in the log is "guest asked for a
     * software reset" BEFORE "decoded track N -> feeding". Keying suppression on
     * pcm_ready alone loses that race -- the device reboots to the language
     * screen before the song is ready. hle.chosen is set the instant the guest
     * starts streaming the file (note_disk_read), so this suppresses the blank-
     * tag watchdog reboot from the start of playback and keeps the guest alive
     * through the decode and on into the paced feed. */
    return hle.enabled && hle.sustain &&
           (hle.chosen >= 0 || qatomic_read(&hle.pcm_ready));
}

/*
 * True when reset suppression is armed (HLE_AUDIO_SUSTAIN), regardless of
 * whether a song has been chosen yet. The blank-tag ("--------") reboot is NEVER
 * a legitimate boot -- it is the deadlock watchdog OR the battery-low path -- so
 * with SUSTAIN on we hold it off during plain menu BROWSING too, otherwise the
 * guest reboots out from under the user (~46 s) before they reach the music
 * menu. Real reboots (diskmode/retailOS tags) are gated separately by the tag.
 */
bool ipod_classic_hle_audio_sustain_on(void)
{
    return hle.enabled && hle.sustain;
}

void ipod_classic_hle_audio_notify_play_start(void)
{
    if (!hle.enabled) {
        return;
    }
    if (!hle.started) {
        hle.started = true;
        if (hle.trace) {
            fprintf(stderr, "HLE_AUDIO: I2S transmitter started\n");
        }
    }
}

/*
 * The I2S clock is being re-configured for a real play (the retailOS driver
 * disables CLKCON, waits for the stop bit, then reprograms and re-enables --
 * boot's silence init never disables it). This is a reliable "the user started
 * playback" edge. If the guest's non-deterministic play never streams the file
 * for us to key on (wiki/19 11.11 regime B), fall back to decoding the disc's
 * single unambiguous song so playback is reliable either way.
 */
void ipod_classic_hle_audio_notify_clock_reconfig(void)
{
    if (!hle.enabled || hle.play_seen) {
        return;
    }
    hle.play_seen = true;
    if (hle.trace) {
        fprintf(stderr, "HLE_AUDIO: playback clock re-config (play bring-up)\n");
    }
    if (hle.chosen < 0 && hle.unique_song && hle.ntracks > 0) {
        if (hle.trace) {
            fprintf(stderr, "HLE_AUDIO: fallback -> decoding the disc's "
                    "single song (file0 '%s')\n", hle.tracks[0].name);
        }
        hle_request_decode(0);
    }
}

/*
 * When the "Now Playing" bootstrap (HLE_NP) owns the audio, it feeds the guest's
 * OWN render engine's copy source instead of the DMA, so the guest engine
 * renders (copy32>0) and Now Playing advances. In that mode this data-path feed
 * must stand down or the song would be double-fed. Set by the NP module.
 */
static bool hle_np_owns_feed;
void ipod_classic_hle_audio_set_np_feed(bool on) { hle_np_owns_feed = on; }

/* Decoded-PCM accessors for the NP module (same host-decoded song, one copy). */
bool ipod_classic_hle_audio_have_pcm(void)
{
    return hle.enabled && qatomic_read(&hle.pcm_ready);
}
size_t ipod_classic_hle_audio_copy_pcm(size_t cursor, void *dst, size_t len)
{
    size_t avail, n = 0;

    if (!hle.enabled || !qatomic_read(&hle.pcm_ready)) {
        return 0;
    }
    qemu_mutex_lock(&hle.lock);
    avail = (cursor < hle.pcm_len) ? hle.pcm_len - cursor : 0;
    n = MIN(len, avail);
    if (n) {
        memcpy(dst, hle.pcm + cursor, n);
    }
    qemu_mutex_unlock(&hle.lock);
    return n;
}
size_t ipod_classic_hle_audio_pcm_len(void)
{
    return hle.enabled ? hle.pcm_len : 0;
}

/* ------------------------------------------------------- player controls ---- */
/*
 * Driven by the wheel buttons (via ipod_classic_wheel.c). Everything is a rule
 * for moving/freezing the single authoritative cursor hle.pcm_cursor. All are
 * no-ops unless the shim is enabled, so the default build is unaffected.
 */
bool ipod_classic_hle_audio_paused(void)
{
    return hle.enabled && hle.paused;
}
void ipod_classic_hle_audio_set_pause(bool on)
{
    if (hle.enabled) {
        hle.paused = on;
    }
}
void ipod_classic_hle_audio_toggle_pause(void)
{
    if (hle.enabled) {
        hle.paused = !hle.paused;
    }
}
uint64_t ipod_classic_hle_audio_position_frames(void)
{
    uint64_t f = 0;
    if (hle.enabled && qatomic_read(&hle.pcm_ready)) {
        qemu_mutex_lock(&hle.lock);
        f = hle.pcm_cursor / 4;      /* 4 bytes = 1 stereo 16-bit frame */
        qemu_mutex_unlock(&hle.lock);
    }
    return f;
}
uint64_t ipod_classic_hle_audio_total_frames(void)
{
    return (hle.enabled && qatomic_read(&hle.pcm_ready)) ? hle.pcm_len / 4 : 0;
}
void ipod_classic_hle_audio_seek(int seconds_delta)
{
    if (!hle.enabled || !qatomic_read(&hle.pcm_ready)) {
        return;
    }
    qemu_mutex_lock(&hle.lock);
    int64_t cur = (int64_t)hle.pcm_cursor;
    int64_t d = (int64_t)seconds_delta * 44100 * 4;
    int64_t nw = cur + d;
    if (nw < 0) {
        nw = 0;
    }
    if (nw > (int64_t)hle.pcm_len) {
        nw = (int64_t)hle.pcm_len;
    }
    hle.pcm_cursor = (size_t)(nw & ~3);          /* keep frame-aligned */
    qemu_mutex_unlock(&hle.lock);
}
void ipod_classic_hle_audio_change_track(int delta)
{
    int k;

    if (!hle.enabled || hle.ntracks <= 0 || hle.chosen < 0) {
        return;
    }
    k = (hle.chosen + delta) % hle.ntracks;
    if (k < 0) {
        k += hle.ntracks;
    }
    hle_request_decode(k);       /* clears pcm_ready+cursor; worker re-decodes */
}
int ipod_classic_hle_audio_track_index(void)
{
    return hle.enabled ? hle.chosen : -1;
}
int ipod_classic_hle_audio_track_count(void)
{
    return hle.enabled ? hle.ntracks : 0;
}
const char *ipod_classic_hle_audio_track_name(void)
{
    if (hle.enabled && hle.chosen >= 0 && hle.chosen < hle.ntracks) {
        return hle.tracks[hle.chosen].name;
    }
    return "";
}
const char *ipod_classic_hle_audio_track_title(void)
{
    return (hle.enabled && hle.cur_title[0]) ? hle.cur_title : "";
}
const char *ipod_classic_hle_audio_track_artist(void)
{
    return (hle.enabled && hle.cur_artist[0]) ? hle.cur_artist : "";
}
const char *ipod_classic_hle_audio_track_album(void)
{
    return (hle.enabled && hle.cur_album[0]) ? hle.cur_album : "";
}

static bool hle_stream_idle(void)
{
    return hle.enabled && hle.stream_seen &&
           qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - hle.stream_ns
               > HLE_STREAM_IDLE_NS;
}

void ipod_classic_hle_audio_fill_dma(void)
{
    PL080State *s = hle.dmac0;
    pl080_channel *ch = NULL;
    uint32_t count, swidth, bytes;
    size_t remaining, n;

    hle.filled = false;

    if (hle_np_owns_feed) {
        return;   /* NP drives the guest engine directly; don't double-feed */
    }
    if (hle.paused) {
        return;   /* paused: stage nothing (the drain also skips pace+commit) */
    }
    if (!hle.enabled || !qatomic_read(&hle.pcm_ready) || !s) {
        return;
    }

    /* Find the enabled channel driving the I2S TX FIFO (dest_id == IIS0_TX). */
    for (int c = 0; c < s->nchannels; c++) {
        pl080_channel *cc = &s->chan[c];
        if ((cc->conf & HLE_CCONF_E) &&
            ((cc->conf >> HLE_CCONF_DSTID_SH) & HLE_CCONF_ID_MASK)
                == HLE_IIS0_TX) {
            ch = cc;
            break;
        }
    }
    if (!ch) {
        return;
    }

    /*
     * pl080_pace_request (called right after this) drains up to `count` source
     * elements starting at ch->src (source-increment set), each `swidth` bytes,
     * to the FIFO. Stage the next PCM into exactly those bytes so the burst the
     * guest's DMA transports carries the real song. We do NOT advance the read
     * cursor here: the drain may move fewer than `count` elements (or none, on a
     * poll tick), so the cursor is advanced in _commit() by the amount ACTUALLY
     * consumed -- otherwise the song would race ahead of real playback.
     */
    count = ch->ctrl & HLE_CCTRL_XSIZE_MASK;
    swidth = 1u << ((ch->ctrl >> HLE_CCTRL_SWIDTH_SH) & HLE_CCTRL_SWIDTH_MASK);
    bytes = count * swidth;
    if (bytes == 0 || bytes > HLE_FILL_MAX_BYTES) {
        return;
    }

    /*
     * The guest feeds its engine one compressed frame at a time, in real time
     * (MEASURED on 2.0.5: ~38 hand-overs a second while a track plays, none
     * while it is paused). So when they stop, the guest has paused or stopped,
     * and the host must stop too -- it has no other way to know, and used to
     * play on under a paused Now Playing screen. Silence, cursor held.
     */
    if (hle_stream_idle()) {
        static const uint8_t quiet[HLE_FILL_MAX_BYTES];
        cpu_physical_memory_write(ch->src, quiet, bytes);
        return;
    }

    qemu_mutex_lock(&hle.lock);
    remaining = hle.pcm_len - hle.pcm_cursor;
    n = MIN((size_t)bytes, remaining);
    if (n > 0) {
        cpu_physical_memory_write(ch->src, hle.pcm + hle.pcm_cursor, n);
    }
    qemu_mutex_unlock(&hle.lock);

    /* Past the end of the song: zero the rest of the segment (clean silence). */
    if (n < bytes) {
        static const uint8_t zero[HLE_FILL_MAX_BYTES];
        cpu_physical_memory_write(ch->src + n, zero, bytes - n);
    }

    hle.filled = true;
    hle.fill_swidth = swidth;
}

/*
 * Advance the read cursor by what the paced burst ACTUALLY consumed. `moved` is
 * the I2S transmit-sample delta (= source elements drained). Keeping the cursor
 * tied to real consumption is what makes the song play at the true clock instead
 * of racing ahead on the drain's idle poll ticks.
 */
void ipod_classic_hle_audio_commit(uint32_t moved)
{
    if (!hle.enabled || !hle.filled) {
        return;
    }
    hle.filled = false;
    qemu_mutex_lock(&hle.lock);
    hle.pcm_cursor += (size_t)moved * hle.fill_swidth;
    if (hle.pcm_cursor > hle.pcm_len) {
        hle.pcm_cursor = hle.pcm_len;
    }
    qemu_mutex_unlock(&hle.lock);
}

/*
 * THE STREAM INDEX.
 *
 * What the guest hands its audio engine is compressed stream straight out of
 * a file, one MPEG audio frame at a time (MEASURED on 2.0.5: successive
 * hand-overs 313, 522, 835, 626 bytes apart in a VBR file, each starting on a
 * frame header). Finding those bytes on the disk names the file and the
 * frame, exactly -- where the disk-read heuristic above can only guess, and
 * guesses wrong whenever a readahead straddles two files.
 *
 * The frame number matters as much as the file: a position worked out from
 * the byte offset alone is only right for constant bitrate.
 */
typedef struct {
    uint8_t  *data;         /* the file's bytes, read once */
    uint32_t *frame_off;    /* where each MPEG audio frame starts */
    unsigned  nframes;
    unsigned  spf;          /* samples per frame: 1152, or 576 for MPEG-2/2.5 */
    unsigned  rate;
    unsigned  channels;
    bool      info_frame;   /* frame 0 is a Xing/Info header, not audio */
} HLEStream;

static HLEStream hle_streams[HLE_MAX_TRACKS];

/* MPEG audio Layer III frame header at d[0..3]: its length, 0 if not one. */
static unsigned hle_mp3_frame(const uint8_t *d, unsigned *spf, unsigned *rate,
                              unsigned *channels)
{
    static const uint16_t br1[16] = { 0, 32, 40, 48, 56, 64, 80, 96, 112, 128,
                                      160, 192, 224, 256, 320, 0 };
    static const uint16_t br2[16] = { 0, 8, 16, 24, 32, 40, 48, 56, 64, 80,
                                      96, 112, 128, 144, 160, 0 };
    static const uint32_t sr[4][3] = { { 11025, 12000, 8000 }, { 0, 0, 0 },
                                       { 22050, 24000, 16000 },
                                       { 44100, 48000, 32000 } };
    unsigned ver = (d[1] >> 3) & 3, layer = (d[1] >> 1) & 3;
    unsigned bri = d[2] >> 4, sri = (d[2] >> 2) & 3, pad = (d[2] >> 1) & 1;
    unsigned br;

    if (d[0] != 0xff || (d[1] & 0xe0) != 0xe0 || ver == 1 || layer != 1 ||
        bri == 0 || bri == 15 || sri == 3) {
        return 0;
    }
    br = (ver == 3 ? br1[bri] : br2[bri]) * 1000;
    *spf = (ver == 3) ? 1152 : 576;
    *rate = sr[ver][sri];
    *channels = ((d[3] >> 6) & 3) == 3 ? 1 : 2;
    return (ver == 3 ? 144 : 72) * br / *rate + pad;
}

static bool hle_stream_load(int k)
{
    HLEStream *st = &hle_streams[k];
    HLETrack *t = &hle.tracks[k];
    size_t o = 0, cap = 0;
    int fd;

    if (st->data) {
        return true;
    }
    fd = open(hle.disk_path, O_RDONLY);
    if (fd < 0) {
        return false;
    }
    st->data = g_malloc(t->size);
    if (pread(fd, st->data, t->size, t->file_off) != (ssize_t)t->size) {
        g_free(st->data);
        st->data = NULL;
        close(fd);
        return false;
    }
    close(fd);

    /* Past an ID3v2 tag, then frame by frame. Not MP3: no index, and the
     * callers fall back to the byte offset. */
    if (t->size > 10 && !memcmp(st->data, "ID3", 3)) {
        o = 10 + ((st->data[6] & 0x7f) << 21 | (st->data[7] & 0x7f) << 14 |
                  (st->data[8] & 0x7f) << 7 | (st->data[9] & 0x7f));
    }
    while (o + 4 <= t->size) {
        unsigned spf, rate, ch;
        unsigned n = hle_mp3_frame(st->data + o, &spf, &rate, &ch);

        if (!n) {
            o++;
            continue;
        }
        if (st->nframes == cap) {
            cap = cap ? cap * 2 : 8192;
            st->frame_off = g_renew(uint32_t, st->frame_off, cap);
        }
        if (st->nframes == 0) {
            st->spf = spf;
            st->rate = rate;
            st->channels = ch;
            st->info_frame = n >= 48 &&
                (memmem(st->data + o, MIN(n, 64u), "Xing", 4) ||
                 memmem(st->data + o, MIN(n, 64u), "Info", 4));
        }
        st->frame_off[st->nframes++] = (uint32_t)o;
        o += n;
    }
    return true;
}

/* Which file, and where in it, a piece of stream comes from. */
static bool hle_stream_find(const uint8_t *head, size_t len, int *k_out,
                            size_t *off_out)
{
    for (int i = 0; i < hle.ntracks; i++) {
        /* The current track first: it is the likeliest, and the cheapest. */
        int k = (hle.chosen >= 0) ? (hle.chosen + i) % hle.ntracks : i;
        uint8_t *hit;

        if (!hle_stream_load(k)) {
            continue;
        }
        hit = memmem(hle_streams[k].data, hle.tracks[k].size, head, len);
        if (hit) {
            *k_out = k;
            *off_out = hit - hle_streams[k].data;
            return true;
        }
    }
    return false;
}

/* The audio frame that starts at `off`, counting from the first one that
 * carries audio; -1 if the file has no index or `off` is not a frame. */
static long hle_stream_frame(int k, size_t off)
{
    HLEStream *st = &hle_streams[k];
    unsigned lo = 0, hi = st->nframes;

    while (lo < hi) {
        unsigned mid = (lo + hi) / 2;
        if (st->frame_off[mid] < off) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    if (lo >= st->nframes || st->frame_off[lo] != off) {
        return -1;
    }
    return (long)lo - (st->info_frame ? 1 : 0);
}

/*
 * The guest handed its audio engine a piece of stream: follow it. A different
 * file is a track change; a distant position in the same file is a seek or a
 * restart. Only the host feed (ipod_classic_hle_audio_fill_dma) needs the
 * position -- when the engine takes its PCM frame by frame the guest's own
 * pipeline carries the timing.
 */
void ipod_classic_hle_audio_note_stream(const uint8_t *head, size_t len)
{
    size_t off;
    long frame;
    double frac;
    bool same;
    int k;

    if (!hle.enabled || !qatomic_read(&hle.scan_done) || len < 32) {
        return;
    }
    if (!hle_stream_find(head, len, &k, &off)) {
        if (hle.trace) {
            fprintf(stderr, "HLE_AUDIO: stream NOT on disk: %02x %02x %02x %02x "
                    "%02x %02x %02x %02x\n", head[0], head[1], head[2],
                    head[3], head[4], head[5], head[6], head[7]);
        }
        return;
    }

    /*
     * Where that is in the decoded track, as a fraction. By frame number when
     * there is an index: MEASURED on a VBR file the byte offset put the
     * position seconds out, and the seek test below then jumped the audio.
     * The host feed's PCM has had ffmpeg's 1105 priming samples trimmed when
     * the file carries an info frame.
     */
    frame = hle_stream_frame(k, off);
    if (frame >= 0 && hle_streams[k].nframes > 1) {
        HLEStream *st = &hle_streams[k];
        double total = (double)(st->nframes - (st->info_frame ? 1 : 0)) *
                       st->spf;
        double at = (double)frame * st->spf -
                    (st->info_frame && !hle_native ? 1105.0 : 0.0);
        frac = at > 0 ? at / total : 0.0;
    } else {
        frac = (double)off / (double)hle.tracks[k].size;
    }

    qemu_mutex_lock(&hle.lock);
    hle.stream_seen = true;
    hle.stream_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    same = (k == hle.chosen);
    if (same && qatomic_read(&hle.pcm_ready)) {
        size_t want = (size_t)(frac * (double)hle.pcm_len) & ~(size_t)3;
        size_t have = hle.pcm_cursor;
        if ((want > have ? want - have : have - want) > HLE_STREAM_SEEK_BYTES) {
            hle.pcm_cursor = want;
        }
    } else if (same) {
        hle.start_frac = frac;          /* decode in flight: start there */
    }
    qemu_mutex_unlock(&hle.lock);
    if (!same) {
        hle_request_decode(k);
        qemu_mutex_lock(&hle.lock);
        hle.start_frac = frac;
        qemu_mutex_unlock(&hle.lock);
    }
    if (hle.trace) {
        fprintf(stderr, "HLE_AUDIO: t=%" PRId64 "ms stream is file%d at byte "
                "%zu of %llu (frame %ld, %.1f%%)%s\n",
                qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL), k, off,
                (unsigned long long)hle.tracks[k].size, frame, frac * 100.0,
                same ? "" : " -- track change");
    }
}

/*
 * The decoded audio of the frame the guest just handed its engine, in the
 * file's own format: `spf * channels` 16-bit samples. This is what lets the
 * engine model behave as a decoder -- frame in, that frame's PCM out -- so the
 * guest's own pipeline carries the music, through its EQ and volume, with its
 * own timing.
 *
 * Returns the bytes written; 0 when the frame is known but its track is still
 * being decoded (the caller may wait a moment); -1 when it is not a frame of
 * any file on the disk.
 */
int ipod_classic_hle_audio_frame_pcm(const uint8_t *head, size_t len,
                                     uint8_t *dst, size_t max)
{
    HLEStream *st;
    size_t off, bytes, at, n = 0;
    long frame;
    int k;

    if (!hle.enabled || !hle_native || !qatomic_read(&hle.scan_done) ||
        !hle_stream_find(head, len, &k, &off)) {
        return -1;
    }
    frame = hle_stream_frame(k, off);
    st = &hle_streams[k];
    if (frame < 0) {
        /* The info frame, or not on a frame boundary: silence is right for
         * the first and the honest answer for the second. */
        bytes = MIN(max, (size_t)(st->spf ? st->spf : 1152) *
                         (st->channels ? st->channels : 2) * 2);
        memset(dst, 0, bytes);
        return (int)bytes;
    }
    bytes = MIN(max, (size_t)st->spf * st->channels * 2);
    at = (size_t)frame * st->spf * st->channels * 2;

    qemu_mutex_lock(&hle.lock);
    if (k != hle.chosen || !qatomic_read(&hle.pcm_ready)) {
        qemu_mutex_unlock(&hle.lock);
        return 0;
    }
    if (at < hle.pcm_len) {
        n = MIN(bytes, hle.pcm_len - at);
        memcpy(dst, hle.pcm + at, n);
    }
    qemu_mutex_unlock(&hle.lock);
    memset(dst + n, 0, bytes - n);
    return (int)bytes;
}

/* ---------------------------------------------------------------- init ----- */

void ipod_classic_hle_audio_init(ARMCPU *cpu, PL080State *dmac0,
                                 IPodClassicI2SState *i2s)
{
    const char *disk;

    if (!hle_env_on("HLE_AUDIO_DATAPATH")) {
        return;   /* default build: install nothing, byte-identical baseline */
    }

    memset(&hle, 0, sizeof(hle));
    hle.enabled = true;
    hle.trace = hle_env_on("HLE_AUDIO_TRACE");
    hle.trace_disk = getenv("HLE_AUDIO_TRACE") &&
                     atoi(getenv("HLE_AUDIO_TRACE")) >= 2;
    hle.sustain = hle_env_on("HLE_AUDIO_SUSTAIN");
    hle.cpu = cpu;
    hle.dmac0 = dmac0;
    hle.i2s = i2s;
    hle.chosen = -1;
    hle.force_track = -1;

    disk = getenv("HLE_AUDIO_DISK");
    if (!disk || !*disk) {
        fprintf(stderr, "HLE_AUDIO: HLE_AUDIO_DATAPATH set but HLE_AUDIO_DISK is "
                "not -- cannot decode the guest's song; shim disabled\n");
        hle.enabled = false;
        return;
    }
    g_strlcpy(hle.disk_path, disk, sizeof(hle.disk_path));

    if (getenv("HLE_AUDIO_TRACK")) {
        hle.force_track = atoi(getenv("HLE_AUDIO_TRACK"));
    }

    qemu_mutex_init(&hle.lock);
    qemu_cond_init(&hle.cond);
    qemu_thread_create(&hle.worker, "hle-audio-decode", hle_worker, NULL,
                       QEMU_THREAD_JOINABLE);

    fprintf(stderr, "HLE_AUDIO: data-path HLE armed (disk '%s'%s) -- host "
            "decodes the guest's own song into the guest's own I2S DMA\n",
            hle.disk_path,
            hle.force_track >= 0 ? " forced-track" : " auto-identify");
}
