#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Compare how alpha long and stock MRCC accelerate.

Extracts a 50 Hz table per segment (cached as .npz next to the rlog), stitches segments into
routes, then cuts "acceleration episodes": engaged, no pedal, positive command held for at
least a second. Per episode it reports the peak command, the peak achieved accel, the command
jerk on the way up and the speed span, and prints op-vs-stock distributions by speed bin and
lead state.

Usage:
  .venv/bin/python3 tools/mazda_long/accel_profile.py [--dump episodes.csv] [glob...]
Default globs: device_data/0000017*, device_data/0000018*, test_data/drive_*.
"""
import argparse
import glob
import os
import re
import sys
from collections import defaultdict
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CRZ_INFO, CRZ_CTRL, PEDALS = 0x21B, 0x21C, 0x165

COLS = ["t", "vEgo", "aEgo", "vCruise", "gas", "brake", "enabled", "standstill", "longActive",
        "accel", "aTarget", "stockCmd", "stockAcc", "leadStatus", "dRel", "vRel", "opLong",
        "personality", "gitDirty", "src"]
IDX = {c: i for i, c in enumerate(COLS)}


def accel_cmd(dat):
  # CRZ_INFO.ACCEL_CMD 17|13@0+ (0.001,-4.096)
  return ((((dat[2] & 0x3) << 11) | (dat[3] << 3) | (dat[4] >> 5)) - 4096) * 0.001


def extract(path):
  cache = path + ".accel.npz"
  if os.path.exists(cache) and os.path.getmtime(cache) > os.path.getmtime(path):
    try:
      z = np.load(cache, allow_pickle=True)
      return path, z["rows"], str(z["commit"])
    except (EOFError, OSError, ValueError, KeyError):
      pass  # partial write from a concurrent run; re-extract
  from openpilot.tools.lib.logreader import LogReader
  try:
    lr = LogReader(path)
  except Exception:
    return path, None, ""

  rows = []
  cur = dict.fromkeys(COLS, np.nan)
  cur.update(gas=0, brake=0, enabled=0, standstill=0, longActive=0, stockAcc=0, leadStatus=0,
             opLong=0, personality=1, gitDirty=0, src=0)
  commit = ""
  for m in lr:
    w = m.which()
    if w == "carState":
      cs = m.carState
      cur.update(t=m.logMonoTime * 1e-9, vEgo=cs.vEgo, aEgo=cs.aEgo, vCruise=cs.vCruise,
                 gas=int(cs.gasPressed), brake=int(cs.brakePressed),
                 enabled=int(cs.cruiseState.enabled), standstill=int(cs.standstill))
      rows.append([cur[c] for c in COLS])
    elif w == "carControl":
      cur.update(longActive=int(m.carControl.longActive), accel=m.carControl.actuators.accel)
    elif w == "longitudinalPlan":
      cur.update(aTarget=m.longitudinalPlan.aTarget)
    elif w == "longitudinalPlanSP":
      cur.update(src=int(m.longitudinalPlanSP.longitudinalPlanSource.raw))
    elif w == "radarState":
      l = m.radarState.leadOne
      cur.update(leadStatus=int(l.present), dRel=l.dRel, vRel=l.vRel)
    elif w == "selfdriveState":
      cur.update(personality=int(m.selfdriveState.personality.raw))
    elif w == "carParams":
      cur.update(opLong=int(m.carParams.openpilotLongitudinalControl))
    elif w == "initData":
      commit = m.initData.gitCommit[:10]
      cur.update(gitDirty=int(m.initData.dirty))
    elif w == "can":
      for c in m.can:
        if c.src != 0 or len(c.dat) != 8:
          continue
        d = c.dat
        if c.address == CRZ_INFO:
          cur["stockCmd"] = accel_cmd(d)
          cur["stockAcc"] = (d[4] >> 1) & 1

  rows = np.array(rows, dtype=float) if len(rows) > 100 else None
  tmp = f"{cache}.{os.getpid()}.tmp"
  with open(tmp, "wb") as f:
    np.savez_compressed(f, rows=rows if rows is not None else np.zeros((0, len(COLS))), commit=commit)
  os.replace(tmp, cache)
  return path, rows, commit


def seg_key(path):
  d = os.path.basename(os.path.dirname(path))
  m = re.match(r"(.*)--(\d+)$", d)
  if m:
    return m.group(1), int(m.group(2))
  m = re.match(r"rlog_(\w+)_seg(\d+)", os.path.basename(path))
  if m:
    return os.path.basename(os.path.dirname(path)), int(m.group(2))
  return d, 0


def smooth(x, n):
  if n <= 1:
    return x
  k = np.ones(n) / n
  return np.convolve(np.nan_to_num(x), k, mode="same")


def episodes(route, d, commit):
  """Cut positive-command episodes from a stitched route table."""
  t = d[:, IDX["t"]]
  v = d[:, IDX["vEgo"]]
  a = smooth(d[:, IDX["aEgo"]], 15)  # 0.3 s box on 50 Hz
  gas, brake = d[:, IDX["gas"]] > 0, d[:, IDX["brake"]] > 0
  op_long = np.nanmax(d[:, IDX["opLong"]]) > 0
  if op_long:
    cmd = d[:, IDX["accel"]]
    engaged = d[:, IDX["longActive"]] > 0
  else:
    cmd = d[:, IDX["stockCmd"]]
    engaged = d[:, IDX["stockAcc"]] > 0
  cmd = np.nan_to_num(cmd)
  dt = np.median(np.diff(t))
  ok = engaged & ~gas & ~brake & (cmd > 0.15) & (v > 0.3)
  # break on gaps in time (segment boundaries with lost frames)
  ok[1:] &= np.diff(t) < 0.5
  out = []
  i = 0
  n = len(ok)
  while i < n:
    if not ok[i]:
      i += 1
      continue
    j = i
    while j < n and ok[j]:
      j += 1
    if t[j - 1] - t[i] >= 1.0:
      s = slice(i, j)
      c = cmd[s]
      pk = int(np.argmax(c))
      # command jerk: max slope over a 0.2 s window on the rising side
      win = max(1, int(round(0.2 / dt)))
      jerk = (c[win:] - c[:-win]) / (win * dt) if len(c) > win else np.array([0.])
      # time from episode start to 90 % of peak command
      thr = 0.9 * c[pk]
      t90 = t[i + int(np.argmax(c >= thr))] - t[i]
      lead = d[s, IDX["leadStatus"]].mean() > 0.5
      drel = float(np.nanmedian(d[s, IDX["dRel"]])) if lead else np.nan
      out.append({"route": route, "op": int(op_long), "commit": commit, "t0": float(t[i] - t[0]),
                  "dur": float(t[j - 1] - t[i]), "v0": float(v[i]), "v1": float(v[j - 1]),
                  "vCruise": float(np.nanmedian(d[s, IDX["vCruise"]])),
                  "cmd_peak": float(c[pk]), "cmd_mean": float(c.mean()),
                  "a_peak": float(np.nanmax(a[s])), "a_mean": float(np.nanmean(a[s])),
                  "jerk_up": float(np.nanmax(jerk)), "t90": float(t90),
                  "lead": int(lead), "dRel": drel,
                  "personality": int(np.nanmedian(d[s, IDX["personality"]])),
                  "src": int(np.nanmedian(d[s, IDX["src"]]))})
    i = j
  return out


def q(x, p):
  return float(np.percentile(x, p)) if len(x) else np.nan


SPEED_BINS = [(0, 5), (5, 10), (10, 15), (15, 20), (20, 30)]


def report(eps):
  print(f"{len(eps)} episodes: op={sum(e['op'] for e in eps)} stock={sum(1 - e['op'] for e in eps)}")
  for lead in (0, 1):
    print(f"\n=== {'LEAD' if lead else 'NO LEAD'} ===")
    print(f"{'v0 m/s':>8} {'sys':>5} {'n':>4} | {'cmd_peak p50':>12} {'p90':>6} {'max':>6} | {'a_peak p50':>10} {'p90':>6} {'max':>6} | "
          + f"{'a_mean p50':>10} | {'jerk p50':>8} {'p90':>6} | {'t90 p50':>7} | {'dur p50':>7} {'dv p50':>7}")
    for lo, hi in SPEED_BINS:
      for op in (1, 0):
        sel = [e for e in eps if e["lead"] == lead and e["op"] == op and lo <= e["v0"] < hi]
        if not sel:
          continue
        g = {k: np.array([e[k] for e in sel]) for k in ("cmd_peak", "a_peak", "a_mean", "jerk_up", "t90", "dur", "v0", "v1")}
        cols = [f"{lo:>3}-{hi:<4} {'op' if op else 'stock':>5} {len(sel):>4}",
                f"{q(g['cmd_peak'], 50):>12.2f} {q(g['cmd_peak'], 90):>6.2f} {g['cmd_peak'].max():>6.2f}",
                f"{q(g['a_peak'], 50):>10.2f} {q(g['a_peak'], 90):>6.2f} {g['a_peak'].max():>6.2f}",
                f"{q(g['a_mean'], 50):>10.2f}",
                f"{q(g['jerk_up'], 50):>8.2f} {q(g['jerk_up'], 90):>6.2f}",
                f"{q(g['t90'], 50):>7.2f}",
                f"{q(g['dur'], 50):>7.1f} {q(g['v1'] - g['v0'], 50):>7.1f}"]
        print(" | ".join(cols))


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("globs", nargs="*")
  ap.add_argument("--dump")
  ap.add_argument("--jobs", type=int, default=os.cpu_count())
  args = ap.parse_args()
  globs = args.globs or [os.path.join(HERE, "device_data", "0000017*"), os.path.join(HERE, "device_data", "0000018*"),
                         os.path.join(HERE, "test_data", "drive_*")]
  files = []
  for g in globs:
    for p in glob.glob(g):
      if os.path.isdir(p):
        files += glob.glob(os.path.join(p, "**", "*rlog*.zst"), recursive=True)
      elif p.endswith(".zst"):
        files.append(p)
  files = sorted(set(files))
  print(f"{len(files)} segments", file=sys.stderr)

  with Pool(args.jobs) as pool:
    results = pool.map(extract, files, chunksize=1)

  routes = defaultdict(list)
  commits = {}
  for path, rows, commit in results:
    if rows is None or len(rows) == 0:
      continue
    r, s = seg_key(path)
    routes[r].append((s, rows))
    if commit:
      commits[r] = commit

  eps = []
  for r, segs in sorted(routes.items()):
    d = np.concatenate([rows for _, rows in sorted(segs, key=lambda x: x[0])])
    e = episodes(r, d, commits.get(r, ""))
    op = int(np.nanmax(d[:, IDX["opLong"]]) > 0)
    eng = (d[:, IDX["longActive"]] > 0).mean() if op else (d[:, IDX["stockAcc"]] > 0).mean()
    print(f"{r:<24} {'op' if op else 'stock':>5} {commits.get(r, ''):<10} {len(d) / 50 / 60:6.1f} min"
          + f"  engaged {100 * eng:4.0f}%  episodes {len(e):3d}", file=sys.stderr)
    eps += e

  report(eps)
  if args.dump:
    import csv
    with open(args.dump, "w", newline="") as f:
      w = csv.DictWriter(f, fieldnames=list(eps[0].keys()))
      w.writeheader()
      w.writerows(eps)


if __name__ == "__main__":
  main()
