#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Stock CRZ_INFO ACCEL_CMD while MRCC armed-idle vs main-off vs engaged.
CRZ_INFO 0x21b: ACCEL_CMD 17|13@0+ (0.001,-4.096) -> raw = ((b2)<<5)|(b3>>3) with start bit 17 BE.
CRZ_CTRL 0x21c: CRZ_ACTIVE bit3 byte0, CRZ_AVAILABLE bit17 -> byte2 bit1.
"""
import sys
sys.path.insert(0, "/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot")
from openpilot.tools.lib.logreader import LogReader
from collections import Counter

def be_extract(data, start_bit, size):
  # DBC big-endian: start_bit is MSB position
  byte_i, bit_i = divmod(start_bit, 8)
  bits = []
  pos = start_bit
  for _ in range(size):
    b, i = divmod(pos, 8)
    bits.append((data[b] >> i) & 1)
    # big-endian bit walk: decrement within byte, wrap to bit7 of next byte
    if i == 0:
      pos = (b + 1) * 8 + 7
    else:
      pos -= 1
  v = 0
  for bit in bits:
    v = (v << 1) | bit
  return v

dist = {"engaged": Counter(), "armed_idle": Counter(), "main_off": Counter()}
for path in sys.argv[1:]:
  try:
    lr = LogReader(path)
  except Exception:
    continue
  ctrl = None
  for m in lr:
    if m.which() != "can":
      continue
    for c in m.can:
      if c.src != 0:
        continue
      if c.address == 0x21c and len(c.dat) == 8:
        active = (c.dat[0] >> 3) & 1
        avail = be_extract(bytes(c.dat), 17, 1)
        ctrl = (active, avail)
      elif c.address == 0x21b and len(c.dat) == 8 and ctrl is not None:
        raw = be_extract(bytes(c.dat), 17, 13)
        set_allowed = be_extract(bytes(c.dat), 34, 1)
        key = "engaged" if ctrl[0] else ("armed_idle" if ctrl[1] else "main_off")
        dist[key][(raw, set_allowed)] += 1
for k, cnt in dist.items():
  tot = sum(cnt.values())
  print(f"{k}: n={tot}")
  for (raw, sa), n in cnt.most_common(6):
    print(f"  raw={raw} ({raw*0.001-4.096:+.3f} m/s2) set_allowed={sa}: {n} ({100*n/max(tot,1):.1f}%)")
