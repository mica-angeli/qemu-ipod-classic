#!/bin/sh
# Boot RepriseOS (Apple firmware 2.0.5) on the iPod-Classic machine, macOS or Linux.
#
#   tools/ipod_classic/run7g.sh [extra qemu args]
#
# Expects, under assets/ (kept out of git):
#   classic7g/reprise-body.bin     /osos-cfw.bin from the iPod, bytes 0x800..0xa24400
#   classic7g/reprise-payload.bin  the 960 KiB CFW payload appended after it
#   classic7g/sysinfo.bin          0x120 bytes at 0x25100 of /cfw-loader.bin
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

# The music player. Harmless on a disk with no music; see wiki/20 for what each
# one does. Two differences from the 1.1.2 recipe, both measured on 2.0.5:
#  - AUDIOPACE=1 is required. Unpaced, the audio DMA finishes its first buffer
#    before the firmware has written the link descriptor that follows it, loads
#    a zero-length one, and stalls for good after three bursts.
#  - SM1_DECODE / SM1_FEED_OFF stay off. The PCM they put in the engine slots
#    comes out of the I2S garbled; the host feed is byte-exact.
export IRAMSPILL=0x10000 SHA1_ZERO=1 SM1_IRQ=1 SM1_SLOTS=1 SM1_NSLOTS=2 \
       DISP_ALPHA=1 PMUCHARGER=adapter AUDIOPACE=1 AUDIOLIVE=1 \
       HLE_AUDIO_DATAPATH=1 HLE_AUDIO_SUSTAIN=1 HLE_AUDIO_DISK="$DISK"
case $(uname) in Darwin) audio=coreaudio;; *) audio=pa;; esac

exec ./build/qemu-system-arm \
    -M "iPod-Classic,sysinfo=$A/sysinfo.bin" \
    -kernel $A/reprise-body.bin \
    -device loader,file=$A/reprise-payload.bin,addr=0x08b3c000,force-raw=on \
    -drive if=ide,format=raw,file="$DISK" -snapshot \
    -audio ${AUDIO_BACKEND:-$audio} \
    -serial null -display ${DISPLAY_BACKEND:-sdl} "$@"
