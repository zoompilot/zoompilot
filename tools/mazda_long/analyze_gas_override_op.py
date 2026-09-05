#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

What alpha long does when the driver taps the gas, from our own engaged drives.

Companion to analyze_gas_override.py (which measures stock MRCC). Reports the commanded
accel, longActive, and cruise engagement across each gas press so the two can be compared
directly.

Usage: .venv/bin/python3 tools/mazda_long/analyze_gas_override_op.py [glob...]
"""
import glob
import os
import sys
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import numpy as np

from openpilot.tools.lib.logreader import LogReader

DT = 0.01


def extract(path):
  try:
    lr = LogReader(path)
  except Exception:
    return None

  rows = []
  cur = dict(vEgo=np.nan, aEgo=np.nan, gas=0, brake=0, enabled=0, standstill=0,
             longActive=0, accel=np.nan, aTarget=np.nan, opLong=0, rpm=np.nan, pedal=0)
  for m in lr:
    w = m.which()
    if w == "carState":
      cs = m.carState
      cur.update(vEgo=cs.vEgo, aEgo=cs.aEgo, gas=int(cs.gasPressed), brake=int(cs.brakePressed),
                 enabled=int(cs.cruiseState.enabled), standstill=int(cs.standstill))
      rows.append((m.logMonoTime * 1e-9, cur["vEgo"], cur["aEgo"], cur["gas"], cur["brake"],
                   cur["enabled"], cur["standstill"], cur["longActive"], cur["accel"],
                   cur["aTarget"], cur["pedal"]))
    elif w == "carControl":
      cur.update(longActive=int(m.carControl.longActive), accel=m.carControl.actuators.accel)
    elif w == "longitudinalPlan":
      cur.update(aTarget=m.longitudinalPlan.aTarget)
    elif w == "can":
      for c in m.can:
        if c.address == 0x202 and c.src == 0 and len(c.dat) == 8:
          cur["pedal"] = (c.dat[4] << 4) | (c.dat[5] >> 4)
          cur["rpm"] = ((c.dat[0] << 8) | c.dat[1]) * 0.25

  if len(rows) < 100:
    return None
  return np.array(rows, dtype=float)


def episodes(path):
  d = extract(path)
  if d is None:
    return []
  t, v, a, gas, brake, enabled, standstill, longActive, accel, aTarget, pedal = d.T
  # only drives where openpilot actually had longitudinal
  if not np.nanmax(longActive):
    return []

  out = []
  pressed = gas > 0
  rise = np.flatnonzero(pressed[1:] & ~pressed[:-1]) + 1
  for i in rise:
    pre = slice(max(0, i - 50), i)
    if pre.stop - pre.start < 25 or not longActive[pre].all() or v[i] < 0.3:
      continue
    if brake[pre].any():
      continue
    j = i
    while j < len(t) - 1 and pressed[j]:
      j += 1
    if t[j] - t[i] < 0.15:
      continue
    dur_i = slice(i, j)
    post = slice(j, min(len(t), j + 100))
    out.append(dict(
      path=path, t=t[i], dur=t[j] - t[i], v=v[i] * 3.6, pedal_max=pedal[dur_i].max(),
      accel_pre=accel[pre].mean(), accel_first=accel[i], accel_during=accel[dur_i].mean(),
      a_pre=a[pre].mean(), a_max=a[dur_i].max(), a_post_min=a[post].min() if post.stop > post.start else np.nan,
      accel_post=accel[post].mean() if post.stop > post.start else np.nan,
      long_dropped=int((longActive[dur_i] == 0).any()),
      enabled_dropped=int((enabled[dur_i] == 0).any()),
      enabled_after=int(enabled[post].all()) if post.stop > post.start else -1,
      long_after=int(longActive[post].all()) if post.stop > post.start else -1,
    ))
  return out


def main():
  pats = sys.argv[1:] or ["tools/mazda_long/test_data/alpha_long_logs/*/rlog.zst",
                          "tools/mazda_long/test_data/alpha_long_logs/*/rlog*.zst"]
  # a bare route identifier (dongle/route[/segs]) is handed to LogReader as-is
  paths = sorted({p for pat in pats for p in (glob.glob(pat) or ([pat] if "*" not in pat else []))})
  print(f"{len(paths)} segments")

  eps = []
  with Pool(8) as pool:
    for r in pool.imap_unordered(episodes, paths):
      eps.extend(r)

  if not eps:
    print("no engaged gas-override episodes found in alpha long logs")
    return

  a = lambda k: np.array([e[k] for e in eps], dtype=float)
  print(f"\n{len(eps)} gas presses while alpha long was active\n")
  print(f"  longActive dropped during press : {int(a('long_dropped').sum())} / {len(eps)}")
  print(f"  cruise enabled dropped          : {int(a('enabled_dropped').sum())} / {len(eps)}")
  print(f"  still enabled 1 s after release  : {int((a('enabled_after') == 1).sum())} / {len(eps)}")
  print(f"  longActive back 1 s after release: {int((a('long_after') == 1).sum())} / {len(eps)}")
  print("\ncommanded accel (m/s2)")
  for k, label in (("accel_pre", "0.5 s before"), ("accel_first", "first frame"),
                   ("accel_during", "mean during"), ("accel_post", "1 s after release")):
    x = a(k)
    x = x[np.isfinite(x)]
    if len(x):
      print(f"  {label:18s} mean {x.mean():+6.3f}  p50 {np.percentile(x, 50):+6.3f}")

  print("\npresses that started while openpilot was braking (accel_pre < -0.3):")
  for e in sorted([e for e in eps if e["accel_pre"] < -0.3], key=lambda e: e["accel_pre"])[:15]:
    print(f"  v={e['v']:5.1f} kph pedal={e['pedal_max']:4.0f} dur={e['dur']:4.1f}s  "
          f"cmd {e['accel_pre']:+.2f} -> {e['accel_first']:+.2f} -> mean {e['accel_during']:+.2f} "
          f"| aEgo {e['a_pre']:+.2f} -> max {e['a_max']:+.2f} | after: enabled={e['enabled_after']} "
          f"long={e['long_after']} accel={e['accel_post']:+.2f} aEgo_min={e['a_post_min']:+.2f}")


if __name__ == "__main__":
  main()
