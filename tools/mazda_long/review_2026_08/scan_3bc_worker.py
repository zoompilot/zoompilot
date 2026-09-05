#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Per-file worker: emits raw records.
CENSUS file b0 count | VIOL file t b0 nframes | ENG file t gapf gapt btns | NOBTN file t
"""
import sys, collections
sys.path.insert(0, "/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot")
from openpilot.tools.lib.logreader import LogReader

for p in sys.argv[1:]:
  try:
    lr = LogReader(p)
  except Exception as e:
    print(f"SKIP {p} {e}", flush=True)
    continue
  t0 = None; acc = None; btn_frame = 0; last_press = None; cur_viol = None
  census = collections.Counter()
  out = []
  try:
    for m in lr:
      if m.which() != "can":
        continue
      t = m.logMonoTime * 1e-9
      if t0 is None: t0 = t
      for c in m.can:
        d = bytes(c.dat)
        if c.src == 2 and c.address == 0x21d and len(d) == 8:
          census[d[0]] += 1
          if d[0] != 0x7F:
            if cur_viol is None: cur_viol = [t - t0, d[0], 1]
            else: cur_viol[2] += 1
          elif cur_viol is not None:
            out.append(f"VIOL {p} {cur_viol[0]:.1f} 0x{cur_viol[1]:02x} {cur_viol[2]}")
            cur_viol = None
        elif c.src == 0 and c.address == 0x9d and len(d) == 8:
          btn_frame += 1
          if (d[0] >> 4 & 1) or (d[0] >> 5 & 1) or (d[0] >> 2 & 1):
            which = "+".join(n for n, b in (("SET_P",4),("SET_M",5),("RES",2)) if (d[0]>>b)&1)
            last_press = (btn_frame, t, which)
        elif c.src == 0 and c.address == 0x165 and len(d) == 8:
          a = (d[0] >> 3) & 1
          if acc is not None and a == 1 and acc == 0:
            if last_press is not None:
              out.append(f"ENG {p} {t-t0:.1f} {btn_frame-last_press[0]} {t-last_press[1]:.3f} {last_press[2]}")
            else:
              out.append(f"NOBTN {p} {t-t0:.1f}")
          acc = a
  except Exception as e:
    print(f"ERR {p} {e}", flush=True)
  if cur_viol is not None:
    out.append(f"VIOL {p} {cur_viol[0]:.1f} 0x{cur_viol[1]:02x} {cur_viol[2]}")
  for v, n in census.items():
    out.append(f"CENSUS {p} 0x{v:02x} {n}")
  print("\n".join(out) if out else f"EMPTY {p}", flush=True)
