#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Replace one partition entry in an AGNOS manifest with a freshly built one.

Only the named entry is touched, so the comma three keeps comma's official xbl,
xbl_config, abl, aop, devcfg and system images and takes only boot from us.

Usage:
  update_manifest.py openpilot/common/hardware/comma/tici_agnos.json entry.json \\
      --comment "built by <run url> from kernel <sha>"
"""

import argparse
import json
from pathlib import Path

REQUIRED = ("name", "url", "hash", "hash_raw", "size", "sparse", "full_check", "has_ab")


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument("manifest", type=Path)
  parser.add_argument("entry", type=Path, help="JSON object produced by manifest_entry.py")
  parser.add_argument("--comment", default="", help="provenance string stored as _comment")
  args = parser.parse_args()

  entry = json.loads(args.entry.read_text())
  missing = [k for k in REQUIRED if k not in entry]
  if missing:
    raise SystemExit(f"entry is missing {missing}")
  # agnos.py reads size as an int and hashes exactly that many bytes off the block
  # device. A string size leaves a stub in the inactive boot partition and breaks
  # updates permanently, so type check before this can reach a device.
  if not isinstance(entry["size"], int) or entry["size"] <= 0:
    raise SystemExit(f"size must be a positive int, got {entry['size']!r}")
  for k in ("hash", "hash_raw"):
    v = entry[k]
    if not isinstance(v, str) or len(v) != 64 or any(c not in "0123456789abcdef" for c in v):
      raise SystemExit(f"{k} must be a 64 character lowercase sha256, got {v!r}")
  if entry["sparse"] is not False:
    raise SystemExit("this script only writes flat images, sparse must be false")
  if not entry["sparse"] and entry["hash"] != entry["hash_raw"]:
    raise SystemExit("non-sparse entry must have hash == hash_raw")
  if not entry["url"].startswith("https://"):
    raise SystemExit(f"refusing a non-https url: {entry['url']}")
  # the asset is named after its own hash, so a mismatch means the url and the
  # hashes came from different builds
  if entry["hash_raw"] not in entry["url"]:
    raise SystemExit("url does not contain hash_raw; url and hashes are from different builds")

  if args.comment:
    entry["_comment"] = args.comment

  manifest = json.loads(args.manifest.read_text())
  matches = [i for i, p in enumerate(manifest) if p["name"] == entry["name"]]
  if len(matches) != 1:
    raise SystemExit(f"expected exactly one '{entry['name']}' entry in {args.manifest}, found {len(matches)}")

  manifest[matches[0]] = entry
  args.manifest.write_text(json.dumps(manifest, indent=2) + "\n")
  print(f"updated {entry['name']} in {args.manifest}")


if __name__ == "__main__":
  main()
