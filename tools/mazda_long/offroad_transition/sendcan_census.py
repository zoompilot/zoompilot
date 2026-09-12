#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Count what openpilot transmitted (sendcan) per segment by bus and address, and print every
transmitted CRZ_BTNS (0x9d) frame with its time. Used to tell the wheel's cancel presses
from openpilot's own button commands.

Usage: PYTHONPATH=/data/openpilot python sendcan_census.py RLOG [RLOG ...]
"""
import sys
from collections import Counter

from openpilot.tools.lib.logreader import LogReader

for path in sys.argv[1:]:
  counts = Counter()
  btns = []
  t0 = None
  for msg in LogReader(path):
    if t0 is None:
      t0 = msg.logMonoTime
    if msg.which() != "sendcan":
      continue
    for f in msg.sendcan:
      counts[(f.src, f.address)] += 1
      if f.address == 0x9d:
        btns.append(((msg.logMonoTime - t0) / 1e9, f.src, bytes(f.dat).hex()))
  print(path.split("/")[-2])
  for (src, addr), n in sorted(counts.items()):
    print(f"  bus{src} 0x{addr:x}: {n}")
  for t, src, hx in btns[:40]:
    print(f"  CRZ_BTNS tx {t:8.3f} bus{src} {hx}")
  if len(btns) > 40:
    print(f"  ... {len(btns)} CRZ_BTNS tx total")
