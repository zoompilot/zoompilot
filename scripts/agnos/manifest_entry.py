#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Build the AGNOS manifest entry for a non-sparse partition image, and verify that
the .img.xz we publish decompresses back to exactly what the entry claims.

The field semantics are those of commaai/agnos-builder scripts/package_ota.py,
which is what produces comma's own agnos.json, and of the flasher that consumes
them, openpilot/common/hardware/comma/agnos.py:

  size          bytes written to the partition. agnos.py fails the flash if the
                decompressed stream is not exactly this long.
  hash          sha256 of the decompressed stream. For a non-sparse image that is
                the image itself, so hash == hash_raw. openpilot's own manifest
                test asserts that equality for every non-sparse entry.
  hash_raw      sha256 of the bytes written to the partition. This is the one
                that is re-read and compared on every boot when full_check is
                true, and the one the .img.xz file name carries.
  ondevice_hash sha256 of the image zero padded up to a 4096 byte sector. Nothing
                in openpilot reads it; it is provenance for comma's own tooling.
                Emitted anyway so our entry has the same shape as comma's.
  sparse        false. boot is a flat Android boot image, not an Android sparse
                image, so agnos.py streams it straight to the partition.
  full_check    true. agnos.py re-hashes the first `size` bytes of the partition
                on every check instead of trusting a hash string written past the
                end of the image. boot is small enough to afford it.
  has_ab        true. boot exists as boot_a and boot_b and the updater flashes
                the inactive slot.

Usage:
  manifest_entry.py boot.img --url https://.../boot-<hash_raw>.img.xz
  manifest_entry.py boot.img --xz-name          # just print the file name to publish
  manifest_entry.py boot.img --verify-xz boot-<hash_raw>.img.xz
"""

import argparse
import hashlib
import json
import lzma
import struct
import sys
from pathlib import Path

SECTOR_SIZE = 4096
SPARSE_MAGIC = 0xED26FF3A
BOOT_MAGIC = b"ANDROID!"

# boot_a and boot_b are 67108864 bytes each, read out of comma's gpt_main_4.
# Anything larger cannot be flashed.
MAX_BOOT_SIZE = 64 * 1024 * 1024


def build_entry(path: Path, name: str, url: str) -> dict:
  data = path.read_bytes()
  size = len(data)

  if size >= 4 and struct.unpack("<I", data[:4])[0] == SPARSE_MAGIC:
    raise SystemExit(f"{path} is an Android sparse image; this script only handles flat images")

  # Every other defect here (wrong hash, wrong size, wrong url) aborts the flash and
  # leaves the device on its current slot. An internally consistent image the
  # bootloader refuses is the one input that survives verification and still reaches
  # abctl --set_active, so refuse anything that is not a signed Android boot image.
  # build_kernel.sh leaves the unsigned boot.img.nonsecure next to the real one.
  if data[:8] != BOOT_MAGIC:
    raise SystemExit(f"{path} is not an Android boot image (no ANDROID! magic); refusing to publish it")

  digest = hashlib.sha256(data).hexdigest()

  padded = hashlib.sha256(data)
  padded.update(b"\x00" * ((SECTOR_SIZE - (size % SECTOR_SIZE)) % SECTOR_SIZE))

  return {
    "name": name,
    "url": url,
    "hash": digest,
    "hash_raw": digest,
    "size": size,
    "sparse": False,
    "full_check": True,
    "has_ab": True,
    "ondevice_hash": padded.hexdigest(),
  }


def verify_xz(xz_path: Path, entry: dict) -> None:
  """Decompress the way agnos.py does and check the entry against the result.

  agnos.py uses a single LZMADecompressor and stops at the first stream EOF, so a
  .img.xz built as several concatenated xz streams would silently truncate on
  device. Streaming it back through the same decompressor catches that here.
  """
  decompressor = lzma.LZMADecompressor(format=lzma.FORMAT_AUTO)
  sha256 = hashlib.sha256()
  total = 0

  with open(xz_path, "rb") as f:
    while chunk := f.read(1024 * 1024):
      out = decompressor.decompress(chunk)
      sha256.update(out)
      total += len(out)
      if decompressor.eof:
        break
    trailing = len(decompressor.unused_data) + len(f.read())

  if not decompressor.eof:
    raise SystemExit(f"{xz_path}: decompressor never hit end of stream")
  if trailing:
    raise SystemExit(f"{xz_path}: trailing data after the first xz stream; rebuild single threaded so agnos.py reads the whole image")
  if total != entry["size"]:
    raise SystemExit(f"{xz_path}: decompressed to {total} bytes, entry says {entry['size']}")
  if sha256.hexdigest() != entry["hash"]:
    raise SystemExit(f"{xz_path}: decompressed hash {sha256.hexdigest()}, entry says {entry['hash']}")

  print(f"verified {xz_path}: {total} bytes, sha256 {sha256.hexdigest()}", file=sys.stderr)


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("image", type=Path, help="the partition image, e.g. output/boot.img")
  parser.add_argument("--name", default="boot", help="partition name")
  parser.add_argument("--url", default="", help="URL the .img.xz will be published at")
  parser.add_argument("--xz-name", action="store_true", help="print the file name to publish and exit")
  parser.add_argument("--verify-xz", type=Path, help="check this .img.xz decompresses to the image")
  parser.add_argument("--max-size", type=int, default=MAX_BOOT_SIZE, help="refuse an image larger than this")
  parser.add_argument("-o", "--output", type=Path, help="write the entry JSON here as well as to stdout")
  args = parser.parse_args()

  entry = build_entry(args.image, args.name, args.url)

  if args.max_size and entry["size"] > args.max_size:
    raise SystemExit(f"{args.image} is {entry['size']} bytes, over the {args.max_size} byte partition bound")

  if args.xz_name:
    print(f"{args.name}-{entry['hash_raw']}.img.xz")
    return

  if args.verify_xz is not None:
    verify_xz(args.verify_xz, entry)

  out = json.dumps(entry, indent=2)
  print(out)
  if args.output:
    args.output.write_text(out + "\n")


if __name__ == "__main__":
  main()
