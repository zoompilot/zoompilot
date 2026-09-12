#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Simulate cam_laneinfo_fresh / fsc_settled against real can batches.
Counts can-log-events (approx carstate updates) between 0x440 src2 arrivals,
plus NO_ERR_BIT/ERR_BIT decode (byte0? check DBC) to see if settled can hold 1000 frames.
"""
import sys
sys.path.insert(0, "/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot")
from openpilot.tools.lib.logreader import LogReader

for path in sys.argv[1:]:
  silent = 0
  seen = False
  settled_run = 0
  max_run = 0
  runs_ge_1000 = 0
  n_events = 0
  breaks = 0
  gaps_ev = []
  first_1000_t = None
  t0 = None
  for m in LogReader(path):
    if m.which() != "can":
      continue
    t = m.logMonoTime * 1e-9
    if t0 is None: t0 = t
    n_events += 1
    got = any(c.address == 0x440 and c.src == 2 for c in m.can)
    if got:
      if seen and silent > 0:
        gaps_ev.append(silent)
      seen = True
      silent = 0
    else:
      silent += 1
    fresh = seen and silent < 50
    if fresh:
      settled_run += 1
      if settled_run == 1000 and first_1000_t is None:
        first_1000_t = t - t0
    else:
      if settled_run > 0:
        breaks += 1
      max_run = max(max_run, settled_run)
      settled_run = 0
  max_run = max(max_run, settled_run)
  import collections
  h = collections.Counter()
  for g in gaps_ev:
    h[min(g // 10 * 10, 80)] += 1
  print(f"{path.rsplit('/',2)[-2]}: events={n_events} laneinfo_gap_events hist(10-event bins)={dict(sorted(h.items()))}")
  print(f"   max_fresh_run={max_run} frames, fresh_breaks={breaks}, first_1000_run_at={first_1000_t}")
