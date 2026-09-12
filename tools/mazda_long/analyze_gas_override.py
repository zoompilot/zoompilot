#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

What does stock MRCC do while the driver overrides with the gas pedal?

Alpha long drops CC.longActive the moment PEDAL_GAS leaves zero (gasPressedOverride),
which zeroes ACCEL_CMD and clears ACC_ACTIVE/CRZ_ACTIVE. This measures what the stock
radar does in the same situation so the port can match it.

For every gas press that starts while MRCC is engaged, report the command before the
press, during it, and after release, plus whether stock ever drops its engaged bits.

Usage: .venv/bin/python3 tools/mazda_long/analyze_gas_override.py [glob...]
"""
import glob
import os
import sys
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import numpy as np

from openpilot.tools.lib.logreader import LogReader

CRZ_INFO, CRZ_CTRL, PEDALS, ENGINE_DATA = 0x21B, 0x21C, 0x165, 0x202


def accel_cmd(dat):
  # CRZ_INFO.ACCEL_CMD 17|13@0+ (0.001,-4.096)
  return ((((dat[2] & 0x3) << 11) | (dat[3] << 3) | (dat[4] >> 5)) - 4096) * 0.001


def extract(path):
  try:
    lr = LogReader(path)
  except Exception:
    return None

  t, cmd, radar_acc, crz_acc, ped_acc, gas, rpm, speed, brake, stopping = ([] for _ in range(10))
  cur = dict(cmd=np.nan, radar_acc=0, crz_acc=0, ped_acc=0, gas=0, rpm=np.nan,
             speed=np.nan, brake=0, stopping=0)

  for m in lr:
    if m.which() != "can":
      continue
    for c in m.can:
      if c.src != 0 or len(c.dat) != 8:
        continue
      d = c.dat
      if c.address == CRZ_INFO:
        cur["cmd"] = accel_cmd(d)
        cur["radar_acc"] = (d[4] >> 1) & 1     # ACC_ACTIVE 33|1@0+
        cur["stopping"] = (d[5] >> 2) & 1      # STOPPING 42|1@0+
      elif c.address == CRZ_CTRL:
        cur["crz_acc"] = (d[0] >> 3) & 1       # CRZ_ACTIVE 3|1@0+
      elif c.address == PEDALS:
        cur["ped_acc"] = (d[0] >> 3) & 1       # ACC_ACTIVE 3|1@0+
        cur["brake"] = (d[0] >> 4) & 1         # BRAKE_ON 4|1@0+
      elif c.address == ENGINE_DATA:
        cur["gas"] = (d[4] << 4) | (d[5] >> 4)  # PEDAL_GAS 39|12@0+
        cur["rpm"] = ((d[0] << 8) | d[1]) * 0.25
        cur["speed"] = ((d[2] << 8) | d[3]) * 0.01
        # ENGINE_DATA is the 100 Hz anchor; sample everything on it
        t.append(m.logMonoTime * 1e-9)
        cmd.append(cur["cmd"]); radar_acc.append(cur["radar_acc"])
        crz_acc.append(cur["crz_acc"]); ped_acc.append(cur["ped_acc"])
        gas.append(cur["gas"]); rpm.append(cur["rpm"]); speed.append(cur["speed"])
        brake.append(cur["brake"]); stopping.append(cur["stopping"])

  if len(t) < 100:
    return None
  return tuple(np.array(a, dtype=float) for a in
               (t, cmd, radar_acc, crz_acc, ped_acc, gas, rpm, speed, brake, stopping))


def episodes(path):
  d = extract(path)
  if d is None:
    return []
  t, cmd, radar_acc, crz_acc, ped_acc, gas, rpm, speed, brake, stopping = d

  out = []
  pressed = gas > 0
  rise = np.flatnonzero(pressed[1:] & ~pressed[:-1]) + 1
  for i in rise:
    # engaged and not already stopped/held for the half second before the press
    pre = slice(max(0, i - 50), i)
    if pre.stop - pre.start < 25:
      continue
    if not (ped_acc[pre].all() and radar_acc[pre].all()):
      continue
    if speed[i] < 1.0 or brake[pre].any():
      continue
    # the press itself
    j = i
    while j < len(t) - 1 and pressed[j]:
      j += 1
    dur = t[j] - t[i]
    if dur < 0.15:
      continue
    dur_i = slice(i, j)
    post = slice(j, min(len(t), j + 50))
    # measured accel from the 100 Hz speed channel, smoothed over 0.2 s
    a_meas = np.gradient(np.convolve(speed / 3.6, np.ones(20) / 20, mode="same"), t)
    out.append(dict(
      a_pre=a_meas[pre].mean(), a_max=a_meas[dur_i].max(), a_mean=a_meas[dur_i].mean(),
      path=path, t=t[i], dur=dur, v=speed[i], gas_max=gas[dur_i].max(),
      cmd_pre=cmd[pre].mean(), cmd_min_pre=cmd[pre].min(),
      cmd_first=cmd[i], cmd_during_mean=cmd[dur_i].mean(), cmd_during_max=cmd[dur_i].max(),
      cmd_during_min=cmd[dur_i].min(), cmd_post=cmd[post].mean() if post.stop > post.start else np.nan,
      radar_acc_drop=int((radar_acc[dur_i] == 0).any()), crz_acc_drop=int((crz_acc[dur_i] == 0).any()),
      ped_acc_drop=int((ped_acc[dur_i] == 0).any()),
      stopping_pre=int(stopping[pre].any()), rpm_pre=rpm[pre].mean(), rpm_max=rpm[dur_i].max(),
      # 2 s of command from the press onset, for the release-rate fit
      profile=np.pad(cmd[i:i + 200], (0, max(0, 200 - len(cmd[i:i + 200]))), constant_values=np.nan),
      t_profile=np.pad(t[i:i + 200] - t[i], (0, max(0, 200 - len(t[i:i + 200]))), constant_values=np.nan),
    ))
  return out


def main():
  pats = sys.argv[1:] or ["tools/mazda_long/test_data/drive_*/rlog_*.zst"]
  paths = sorted({p for pat in pats for p in glob.glob(pat)})
  print(f"{len(paths)} segments")

  eps = []
  with Pool(8) as pool:
    for r in pool.imap_unordered(episodes, paths):
      eps.extend(r)

  if not eps:
    print("no gas-override episodes found")
    return

  print(f"\n{len(eps)} gas presses while MRCC engaged\n")
  arr = lambda k: np.array([e[k] for e in eps], dtype=float)

  print("does stock ever drop its engaged bits during the override?")
  for k, label in (("radar_acc_drop", "CRZ_INFO.ACC_ACTIVE"), ("crz_acc_drop", "CRZ_CTRL.CRZ_ACTIVE"),
                   ("ped_acc_drop", "PEDALS.ACC_ACTIVE")):
    n = int(arr(k).sum())
    print(f"  {label:24s} dropped in {n:5d} / {len(eps)} presses ({100 * n / len(eps):.1f}%)")

  print("\nACCEL_CMD around the press (m/s2)")
  for k, label in (("cmd_pre", "0.5 s before"), ("cmd_first", "first frame of press"),
                   ("cmd_during_mean", "mean during"), ("cmd_during_min", "min during"),
                   ("cmd_during_max", "max during"), ("cmd_post", "0.5 s after release")):
    a = arr(k)
    a = a[np.isfinite(a)]
    print(f"  {label:22s} mean {a.mean():+6.3f}  p10 {np.percentile(a, 10):+6.3f}  "
          f"p50 {np.percentile(a, 50):+6.3f}  p90 {np.percentile(a, 90):+6.3f}")

  # the interesting case: pressing gas while stock is commanding a real decel
  dec = [e for e in eps if e["cmd_pre"] < -0.3]
  print(f"\npresses that started while stock was decelerating (cmd_pre < -0.3): {len(dec)}")
  if dec:
    a = lambda k: np.array([e[k] for e in dec], dtype=float)
    print(f"  cmd 0.5 s before   mean {a('cmd_pre').mean():+.3f}")
    print(f"  cmd first frame    mean {a('cmd_first').mean():+.3f}")
    print(f"  cmd mean during    mean {a('cmd_during_mean').mean():+.3f}")
    print(f"  cmd max during     mean {a('cmd_during_max').mean():+.3f}")
    print(f"  engaged bits held  {100 * (1 - a('radar_acc_drop').mean()):.1f}% of presses")
    # how fast does the command come off the brake once gas is applied?
    print(f"  measured accel     pre {a('a_pre').mean():+.3f} -> max during {a('a_max').mean():+.3f}")
    print(f"  rpm                pre {a('rpm_pre').mean():5.0f} -> max during {a('rpm_max').mean():5.0f}")
    print("\n  sample of the 10 deepest decels overridden by gas:")
    for e in sorted(dec, key=lambda e: e["cmd_pre"])[:10]:
      print(f"    v={e['v']:5.1f} kph gas={e['gas_max']:4.0f} dur={e['dur']:4.1f}s  "
            f"cmd {e['cmd_pre']:+.3f} -> first {e['cmd_first']:+.3f} -> "
            f"mean {e['cmd_during_mean']:+.3f} | aEgo {e['a_pre']:+.2f} -> {e['a_max']:+.2f} "
            f"| rpm {e['rpm_pre']:4.0f} -> {e['rpm_max']:4.0f} | held={not e['radar_acc_drop']}")

    held = [e for e in dec if not e["radar_acc_drop"]]
    if held:
      print(f"\n  how fast stock lets the command off the brake ({len(held)} held episodes)")
      prof = np.vstack([e["profile"] for e in held])
      base = np.array([e["cmd_pre"] for e in held])[:, None]
      rel = prof - base
      for k in (0, 10, 25, 50, 100, 150, 199):
        col = rel[:, k]
        col = col[np.isfinite(col)]
        if len(col):
          print(f"    t=+{k / 100:4.2f}s  cmd - cmd_pre: mean {col.mean():+6.3f} m/s2  "
                f"(rate {col.mean() / max(k / 100, 0.01):+5.2f} m/s3)")


if __name__ == "__main__":
  main()
