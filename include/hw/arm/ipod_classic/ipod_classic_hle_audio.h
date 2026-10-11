/*
 * HLE audio DATA-PATH shim for the iPod classic 6G (S5L8702) retailOS.
 *
 * See hw/arm/ipod_classic/ipod_classic_hle_audio.c and wiki/19 section 11
 * (11.19) for the full rationale. In one line: twelve investigation sessions
 * proved the retailOS MeCCA audio pipeline is a SYMMETRIC bootstrap deadlock
 * that never self-primes under faithful emulation (the guest AAC decoder and
 * the render engine each gate on the other's in-regime state). This shim is a
 * High-Level Emulation of the DATA PATH, exactly analogous to the 5G VideoCore
 * HLE (hw/arm/ipod_video/ipod_video_vc.c): the host decodes the guest's OWN
 * song -- the same M4A the guest is reading off its OWN disk -- and feeds the
 * resulting PCM into the buffer the guest's OWN audio DMA (DMAC0 -> I2S FIFO ->
 * CS42L55) transports to the speakers. The only thing done host-side is WHERE
 * the AAC is decoded; the bytes are the real song and they leave through the
 * guest's own I2S. Everything is gated by HLE_AUDIO_DATAPATH; unset, this file
 * installs nothing and the machine is byte-for-byte the baseline (Rockbox and
 * navtest untouched).
 */
#ifndef IPOD_CLASSIC_HLE_AUDIO_H
#define IPOD_CLASSIC_HLE_AUDIO_H

#include "cpu.h"
#include "hw/dma/pl080.h"

/*
 * Install the shim (no-op unless HLE_AUDIO_DATAPATH is set). Given the CPU (for
 * an optional engine nudge) and DMAC0 (whose channel 1 drains the audio
 * ping-pong to the I2S FIFO). The disk image to decode from is taken from the
 * HLE_AUDIO_DISK environment variable -- the SAME raw image the guest has on
 * -drive, so the bytes decoded are exactly the bytes the guest reads.
 */
struct IPodClassicI2SState;
void ipod_classic_hle_audio_init(ARMCPU *cpu, PL080State *dmac0,
                                 struct IPodClassicI2SState *i2s);

/*
 * Called by the I2S model when the transmitter's run bit goes 1 -- the reliable
 * "audio clocking has started" edge. A no-op unless the shim is enabled.
 */
void ipod_classic_hle_audio_notify_play_start(void);

/*
 * Called by the I2S model when the transmit clock is DISABLED (CLKCON bit0
 * 1->0) -- the retailOS's per-play clock re-config, which boot's silence init
 * never does. The shim treats it as the "playback started" edge and, if the
 * disc's song is unambiguous, decodes it even when the guest never streamed the
 * file for us to identify. A no-op unless HLE_AUDIO_DATAPATH is set.
 */
void ipod_classic_hle_audio_notify_clock_reconfig(void);

/*
 * Called by the I2S model from its paced drain, immediately BEFORE
 * pl080_pace_request hands one burst to the FIFO. Fills the audio DMA channel's
 * current source segment with the next chunk of decoded PCM, so the burst the
 * guest's DMA then transports carries the real song. A no-op until a track has
 * been identified and decoded.
 */
void ipod_classic_hle_audio_fill_dma(void);

/*
 * Called by the I2S model right AFTER pl080_pace_request, with the number of
 * transmit samples the burst actually moved. Advances the decoded-PCM read
 * cursor by exactly that much, so the song tracks the real playback clock
 * instead of racing ahead on the drain's idle poll ticks.
 */
void ipod_classic_hle_audio_commit(uint32_t moved);

/* True iff the shim is installed (HLE_AUDIO_DATAPATH set). Lets callers skip
 * even the cheap work (e.g. an IDE sector lookup) so the default build path is
 * byte-for-byte unchanged. */
bool ipod_classic_hle_audio_enabled(void);

/*
 * Called by the ATA model on each disk DMA read, with the disk byte offset and
 * length. Used to identify which audio file the guest is streaming (the read
 * lands in that file's data), so the host decodes the SAME song the guest chose.
 * A no-op unless the shim is enabled.
 */
void ipod_classic_hle_audio_note_disk_read(uint64_t byte_off, uint32_t len);

/*
 * The guest handed its audio engine `len` bytes of compressed stream: identify
 * the file and the position from them. A no-op unless the shim is enabled.
 */
void ipod_classic_hle_audio_note_stream(const uint8_t *head, size_t len);

/*
 * Decode for the engine: native format, frame-aligned. Call before any track
 * is decoded.
 */
void ipod_classic_hle_audio_set_native(bool on);

/*
 * The decoded PCM of the frame whose first bytes are `head`. Bytes written;
 * 0 if that track is still being decoded; -1 if it is not a known frame.
 */
int ipod_classic_hle_audio_frame_pcm(const uint8_t *head, size_t len,
                                     uint8_t *dst, size_t max);

/*
 * True when the shim is enabled and actively feeding decoded PCM (a song has
 * been identified and decoded). The clock model uses this to suppress the
 * retailOS's deadlock-watchdog reboot while the HLE is playing the song --
 * otherwise the guest reboots ~15 s into a stalled play and the audio (and any
 * Now Playing UI) never lasts. A no-op influence unless HLE_AUDIO_DATAPATH is
 * set. See wiki/19 section 11.19.
 */
bool ipod_classic_hle_audio_active(void);

/* True when HLE_AUDIO_SUSTAIN is armed (independent of whether a song is chosen).
 * Used to hold off the blank-tag deadlock/battery reboot during menu browsing so
 * the guest stays alive long enough to reach the music menu. */
bool ipod_classic_hle_audio_sustain_on(void);

/* --- Bridge to the Now-Playing bootstrap (ipod_classic_hle_np.c) ----------- */

/* When true, the NP module owns audio: it feeds the guest's own render engine's
 * copy source (so the engine renders and Now Playing advances) and this
 * data-path DMA feed stands down to avoid double-feeding. */
void ipod_classic_hle_audio_set_np_feed(bool on);

/* The same host-decoded song the data-path shim holds, shared with the NP
 * module (one decode, one buffer). have_pcm() is true once decoded; copy_pcm()
 * copies up to len bytes from the given byte cursor and returns the count;
 * pcm_len() is the total decoded size in bytes. */
bool   ipod_classic_hle_audio_have_pcm(void);
size_t ipod_classic_hle_audio_copy_pcm(size_t cursor, void *dst, size_t len);
size_t ipod_classic_hle_audio_pcm_len(void);

/* --- Player controls (driven by the wheel buttons; all no-op unless enabled) - */
bool     ipod_classic_hle_audio_paused(void);
void     ipod_classic_hle_audio_set_pause(bool on);
void     ipod_classic_hle_audio_toggle_pause(void);
void     ipod_classic_hle_audio_seek(int seconds_delta);          /* relative */
void     ipod_classic_hle_audio_change_track(int delta);          /* +1 / -1  */
uint64_t ipod_classic_hle_audio_position_frames(void);            /* elapsed  */
uint64_t ipod_classic_hle_audio_total_frames(void);              /* duration */
int      ipod_classic_hle_audio_track_index(void);
int      ipod_classic_hle_audio_track_count(void);
const char *ipod_classic_hle_audio_track_name(void);
const char *ipod_classic_hle_audio_track_title(void);
const char *ipod_classic_hle_audio_track_artist(void);
const char *ipod_classic_hle_audio_track_album(void);

#endif /* IPOD_CLASSIC_HLE_AUDIO_H */
