#!/usr/bin/env python3
"""Ender Smart (F15-01): build fingerprint to compare a local build with the CI build.

Prints one line per item: <sha256[:16]> <size> <name>, for
  - each allocated section of build/<project>.elf,
  - generated files that commonly differ between hosts (sdkconfig.h, certificate bundle),
  - the final binaries.

Run from the project directory after `idf.py build` (it uses pyelftools, which comes
with the ESP-IDF Python environment):
    python tools/ci/fingerprint.py [build_dir]
"""
import hashlib
import os
import sys

from elftools.elf.constants import SH_FLAGS
from elftools.elf.elffile import ELFFile


def h(data):
    return hashlib.sha256(data).hexdigest()[:16]


def linha(nome, data):
    print(f"{h(data)} {len(data):8d} {nome}")


def main():
    build = sys.argv[1] if len(sys.argv) > 1 else "build"
    elfs = [f for f in os.listdir(build) if f.endswith(".elf")]
    if not elfs:
        sys.exit(f"no .elf in {build}")
    with open(os.path.join(build, elfs[0]), "rb") as f:
        elf = ELFFile(f)
        for sec in elf.iter_sections():
            if sec["sh_flags"] & SH_FLAGS.SHF_ALLOC and sec["sh_type"] != "SHT_NOBITS":
                linha(f"section {sec.name}", sec.data())
    for rel in ("config/sdkconfig.h", "esp-idf/mbedtls/x509_crt_bundle", "project_description.json"):
        p = os.path.join(build, rel)
        if os.path.exists(p) and not rel.endswith(".json"):
            with open(p, "rb") as f:
                linha(f"file {rel}", f.read())
    for rel in (f"{elfs[0][:-4]}.bin", "bootloader/bootloader.bin"):
        p = os.path.join(build, rel)
        if os.path.exists(p):
            with open(p, "rb") as f:
                linha(f"bin {rel}", f.read())

    # The app image stores the SHA-256 of the whole .elf (debug info included) at
    # offset 0xB0 (esp_app_desc_t.app_elf_sha256). Debug info contains host paths
    # (e.g. the toolchain directory), so this field differs between hosts even when
    # every loaded byte is the same. The normalized hash zeroes that field and drops
    # the image checksum byte + appended SHA-256 (last 33 bytes), which depend on it.
    p = os.path.join(build, f"{elfs[0][:-4]}.bin")
    if os.path.exists(p):
        with open(p, "rb") as f:
            b = bytearray(f.read())
        print(f"{bytes(b[0xB0:0xD0]).hex()[:16]} {32:8d} field app_elf_sha256")
        b[0xB0:0xD0] = bytes(32)
        linha("bin normalized (no elf sha, no checksum/hash)", bytes(b[:-33]))


if __name__ == "__main__":
    main()
