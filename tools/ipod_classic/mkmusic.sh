#!/bin/sh
# Build a disk image with a music library on it, on macOS or Linux, without
# mounting anything: libgpod writes iPod_Control into a plain directory and
# mtools copies that into a fresh FAT32 image.
#
#   mkmusic.sh <out.img> <Firmware-XX> <sysinfo.bin> <file> [<file> ...]
#
# <sysinfo.bin> is the 0x120-byte SysInfo block of the device being emulated
# (the one the machine takes as sysinfo=): its FireWire GUID and model number
# go into iPod_Control/Device/SysInfo, because the iTunesDB hash is keyed off
# that GUID and the firmware recomputes it from its own copy.
#
# Needs: ffprobe, mkfs.vfat (dosfstools), mcopy (mtools), libplist, and libgpod.
#   LIBGPOD=<source tree with src/.libs built>   default ~/Developer/libgpod2
#   FAT_MB=<size of the data volume in MB>       default 256
set -e
[ $# -ge 4 ] || { sed -n 2,16p "$0"; exit 2; }
here=$(cd "$(dirname "$0")" && pwd)
out=$1 fw=$2 sysinfo=$3; shift 3
L=${LIBGPOD:-$HOME/Developer/libgpod2}
work=$(mktemp -d); trap 'rm -rf "$work"' EXIT

# libgpod's headers expect to be installed as <gpod/itdb.h>.
mkdir "$work/inc" && ln -s "$L/src" "$work/inc/gpod"
# Linked statically: an uninstalled libgpod.dylib names an install path that
# does not exist, and pins whatever libplist it was built against.
cc -o "$work/addtrack" "$here/addtrack.c" -I"$work/inc" "$L/src/.libs/libgpod.a" \
    $(pkg-config --cflags --libs glib-2.0 gobject-2.0 gmodule-2.0 libplist-2.0) \
    -lsqlite3 -lxml2 -lz

guid=$(dd if="$sysinfo" bs=1 skip=56 count=8 2>/dev/null | xxd -p | tr a-f A-F)
model=$(dd if="$sysinfo" bs=1 skip=152 count=16 2>/dev/null | tr -d '\0')
root=$work/root
mkdir -p "$root/iPod_Control/Device"
printf 'ModelNumStr: %s\nFirewireGuid: %s\n' "$model" "$guid" \
    > "$root/iPod_Control/Device/SysInfo"

tag() { ffprobe -v error -show_entries "format_tags=$2" -of default=nw=1:nk=1 "$1" | head -1 | tr -d '|'; }
set -- "$@"
n=0
for f in "$@"; do
    n=$((n + 1))
    ms=$(ffprobe -v error -show_entries format=duration -of default=nw=1:nk=1 "$f" \
         | awk '{printf "%d", $1 * 1000}')
    title=$(tag "$f" title); [ -n "$title" ] || title=$(basename "${f%.*}")
    trk=$(tag "$f" track | cut -d/ -f1); [ -n "$trk" ] || trk=$n
    printf '%s|%s|%s|%s|%s|%s\0' "$f" "$title" "$(tag "$f" artist)" \
        "$(tag "$f" album)" "$trk" "$ms"
done | xargs -0 "$work/addtrack" "$root" "$model"

fat=$work/fat32.img
truncate -s "${FAT_MB:-256}M" "$fat"
mkfs.vfat -F 32 -n IPOD "$fat" > /dev/null
mcopy -s -i "$fat" "$root/iPod_Control" ::
python3 "$here/mkdisk6g4k.py" "$out" "$fw" "$fat"
