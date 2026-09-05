#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Distribution of CAM_LANEINFO (0x440 src2) inter-frame gaps across segments.
"""
import sys, glob
sys.path.insert(0, "/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot")
from openpilot.tools.lib.logreader import LogReader

tot = 0; over500 = 0; over550 = 0; mx = 0.; nseg = 0
per_seg = []
for path in sys.argv[1:]:
  last = None; gaps = []
  try:
    lr = LogReader(path)
  except Exception:
    continue
  for m in lr:
    if m.which() != "can":
      continue
    t = m.logMonoTime * 1e-9
    for c in m.can:
      if c.address == 0x440 and c.src == 2:
        if last is not None:
          gaps.append(t - last)
        last = t
  if not gaps:
    continue
  nseg += 1
  o5 = sum(1 for g in gaps if g > 0.5)
  o55 = sum(1 for g in gaps if g > 0.55)
  m_ = max(gaps)
  tot += len(gaps); over500 += o5; over550 += o55; mx = max(mx, m_)
  per_seg.append((path.split('/')[-2] if path.endswith('rlog.zst') else path.split('/')[-1], len(gaps), o5, o55, round(m_*1000,1)))
for r in per_seg:
  print(r)
print(f"TOTAL segs={nseg} gaps={tot} >500ms={over500} >550ms={over550} max={mx*1000:.1f}ms")
