#!/bin/sh
# Boot RepriseOS (Apple firmware 2.0.5) on the iPod-Classic machine, macOS or Linux.
#
#   tools/ipod_classic/run7g.sh [extra qemu args]
#
# Expects, under assets/ (kept out of git):
#   classic7g/reprise-body.bin     }  reprise_split.py <osos-cfw.bin>
#   classic7g/reprise-payload.bin  }      <cfw-loader.bin> assets/classic7g
#   classic7g/sysinfo.bin          }  (both files are in the iPod's root)
#   disk/E-2.0.5.img               mkdisk6g4k.py <out> Firmware-38.9.0.5 <fat32.img>
#
# DISK=<image> picks another disk, e.g. one with music from mkmusic.sh.
# AUDIO_BACKEND=<qemu audio driver> overrides coreaudio (macOS) / pa (Linux).
#
# The payload goes to 0x08b3c000 (CFW_PAYLOAD_BASE in osos-lab for 2.0.5): on the
# device the companion loader copies it there before entering the OS.
# sysinfo= is not optional on 2.0.x: it reboots to "softupdt" unless the
# hardware id is 0x0013xxxx with a low half of 0x300 or more.
#
# Keys: q/e wheel, Enter SELECT, w MENU, a/d prev/next, s PLAY.
cd "$(dirname "$0")/../.." || exit 1
A=assets/classic7g
DISK=${DISK:-assets/disk/E-2.0.5.img}

# The music player. Harmless on a disk with no music.
#
#  SM1_IRQ SM1_SYNC   the audio engine completes each frame the guest hands it,
#                     paced by the guest's own output queue. This is what makes
#                     elapsed time, pause/resume and track changes work.
#  SM1_DECODE         each completed frame carries its real audio, so the
#                     firmware's own pipeline plays the music. The MP3 decode
#                     itself is ffmpeg on the host; everything around it is
#                     the firmware.
#  HLE_AUDIO_*        where that decode comes from: the same file, off the same
#                     disk image the guest is reading.
#  AUDIOPACE          drain the audio DMA at the sample rate. Unpaced it runs
#                     ahead of the firmware's link descriptor and stalls.
#  AUDIOLIVE          play what the I2S clocks out (AUDIOWAV=<file> records it).
#  AUDIOGAIN          the iPod's volume is its headphone amplifier, and the
#                     factory default is -32 dB: right for earbuds, very quiet
#                     for a desk. 20 dB of make-up gain brings the default to a
#                     normal level; the top third of the volume bar then makes
#                     no further difference. AUDIOGAIN=0 is the true level,
#                     AUDIOGAIN=line the fixed level of the dock line-out.
#  DISP_ALPHA         composite the display layers front to back with alpha.
#
# No longer needed on 2.0.5, each one having been a workaround for a model bug
# since fixed: IRAMSPILL (the engine's window register), SM1_SLOTS/SM1_NSLOTS
# (superseded by SM1_SYNC), HLE_AUDIO_SUSTAIN, PMUCHARGER, and SHA1_ZERO.
#
# About that last one: the iTunesDB signature is now checked for real, against
# the FireWire GUID in sysinfo.bin. A disk from mkmusic.sh is signed for it. A
# disk whose database had its hash blanked (mkdisk6g4k.py on its own, or an
# older mkmusic.sh) shows "No Music" unless you run with SHA1_ZERO=1.
export DISP_ALPHA=1 \
       SM1_IRQ=1 SM1_SYNC=1 SM1_DECODE=1 \
       HLE_AUDIO_DATAPATH=1 HLE_AUDIO_DISK="$DISK" \
       AUDIOPACE=1 AUDIOLIVE=1 AUDIOGAIN="${AUDIOGAIN-20}"
case $(uname) in Darwin) audio=coreaudio;; *) audio=pa;; esac

exec ./build/qemu-system-arm \
    -M "iPod-Classic,sysinfo=$A/sysinfo.bin" \
    -kernel $A/reprise-body.bin \
    -device loader,file=$A/reprise-payload.bin,addr=0x08b3c000,force-raw=on \
    -drive if=ide,format=raw,file="$DISK" -snapshot \
    -audio ${AUDIO_BACKEND:-$audio} \
    -serial null -display ${DISPLAY_BACKEND:-sdl} "$@"
