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
# The payload goes to 0x08b3c000 (CFW_PAYLOAD_BASE in osos-lab for 2.0.5): on the
# device the companion loader copies it there before entering the OS.
# sysinfo= is not optional on 2.0.x: it reboots to "softupdt" unless the
# hardware id is 0x0013xxxx with a low half of 0x300 or more.
#
# Keys: q/e wheel, Enter SELECT, w MENU, a/d prev/next, s PLAY.
cd "$(dirname "$0")/../.." || exit 1
A=assets/classic7g
exec ./build/qemu-system-arm \
    -M "iPod-Classic,sysinfo=$A/sysinfo.bin" \
    -kernel $A/reprise-body.bin \
    -device loader,file=$A/reprise-payload.bin,addr=0x08b3c000,force-raw=on \
    -drive if=ide,format=raw,file=${DISK:-assets/disk/E-2.0.5.img} -snapshot \
    -serial null -display ${DISPLAY_BACKEND:-sdl} "$@"
