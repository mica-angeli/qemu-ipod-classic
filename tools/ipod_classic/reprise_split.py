#!/usr/bin/env python3
"""Turn the boot files of a RepriseOS iPod into what run7g.sh loads.

    reprise_split.py <osos-cfw.bin> <cfw-loader.bin> <outdir>

Both inputs sit in the root of the iPod's disk when RepriseOS is installed
(https://github.com/lgreitis/osos-lab). They are already decrypted, so this
needs neither DFU nor the device's key -- just the iPod mounted as a disk.

Writes three files into <outdir>:

    reprise-body.bin      Apple's osos with RepriseOS's patches applied: the
                          image body, without its 0x800-byte "87021.0" header.
                          Loaded at 0x08000000 with -kernel.
    reprise-payload.bin   RepriseOS's own code and data, the last 960 KiB of
                          the file. On the device the companion loader copies
                          it to 0x08b3c000 before entering the OS; in QEMU a
                          "-device loader" does.
    sysinfo.bin           the 0x120-byte SysInfo block the companion loader
                          was personalised with: this iPod's hardware id,
                          serial, model and FireWire GUID. It is what the
                          machine's sysinfo= option takes, and firmware 2.0.x
                          will not boot without one.

sysinfo.bin identifies the device. Keep it out of anything you publish.
"""
import os
import sys

HEADER = 0x800                  # the IMG1 header in front of the body
PAYLOAD_BYTES = 0xf0000         # CFW_PAYLOAD_BYTES in osos-lab payload/native/layout.h
SYSINFO_OFFSET = 0x25100        # SYSINFO_OFFSET in reprise-bundle/src/assembly.rs
SYSINFO_BYTES = 0x120
LOADER_BYTES = 0x2b800


def main():
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    osos = open(sys.argv[1], "rb").read()
    loader = open(sys.argv[2], "rb").read()
    out = sys.argv[3]

    if osos[:7] != b"87021.0":
        sys.exit("%s: no 87021.0 header -- not an osos image" % sys.argv[1])
    if len(osos) <= HEADER + PAYLOAD_BYTES:
        sys.exit("%s: too short to carry a payload" % sys.argv[1])
    if len(loader) != LOADER_BYTES:
        sys.exit("%s: %d bytes, expected %d -- a different companion layout"
                 % (sys.argv[2], len(loader), LOADER_BYTES))
    sysinfo = loader[SYSINFO_OFFSET:SYSINFO_OFFSET + SYSINFO_BYTES]
    if sysinfo[:4] != b"IsyS":
        sys.exit("%s: no SysInfo block at 0x%x" % (sys.argv[2], SYSINFO_OFFSET))

    os.makedirs(out, exist_ok=True)
    parts = {
        "reprise-body.bin": osos[HEADER:len(osos) - PAYLOAD_BYTES],
        "reprise-payload.bin": osos[len(osos) - PAYLOAD_BYTES:],
        "sysinfo.bin": sysinfo,
    }
    for name, data in parts.items():
        with open(os.path.join(out, name), "wb") as f:
            f.write(data)
        print("%-20s %9d bytes" % (name, len(data)))
    hw = int.from_bytes(sysinfo[0x84:0x88], "little")
    print("hardware id 0x%08x, model %s"
          % (hw, sysinfo[0x98:0xa8].split(b"\0")[0].decode("ascii", "replace")))


if __name__ == "__main__":
    main()
