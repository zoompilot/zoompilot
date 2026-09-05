#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

3a: message-rate census on stock segments: freq + max gap per (addr, src).
"""
import sys
sys.path.insert(0, "/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot")
from openpilot.tools.lib.logreader import LogReader

ADDRS = {0x9d: "CRZ_BTNS", 0x165: "PEDALS", 0x21b: "CRZ_INFO", 0x21c: "CRZ_CTRL",
         0x21d: "CAM_EMPTY", 0x25d: "CAM_PEDESTRIAN", 0x440: "CAM_LANEINFO"}

for path in sys.argv[1:]:
  last = {}
  stats = {}  # (addr, src) -> [count, maxgap, t_first, t_last, gap_at_t]
  for m in LogReader(path):
    if m.which() != "can":
      continue
    t = m.logMonoTime * 1e-9
    for c in m.can:
      if c.address in ADDRS:
        k = (c.address, c.src)
        if k in last:
          gap = t - last[k]
          s = stats[k]
          s[0] += 1
          if gap > s[1]:
            s[1] = gap; s[4] = t - s[2]
          s[3] = t
        else:
          stats[k] = [1, 0., t, t, 0.]
        last[k] = t
  print(f"== {path.split('/')[-1] if '/' not in path else path.rsplit('/',1)[-1]} ({path})")
  for (addr, src), (n, mg, tf, tl, gt) in sorted(stats.items()):
    dur = tl - tf
    freq = (n - 1) / dur if dur > 0 else 0
    print(f"  0x{addr:03x} {ADDRS[addr]:16s} src={src} n={n:7d} freq={freq:7.2f} Hz  maxgap={mg*1000:8.1f} ms (at t+{gt:.1f})")
