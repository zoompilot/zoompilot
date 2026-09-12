#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

On-car tune comparison: sweep every rlog on disk, split by the logged controller version
(ControlsState.LateralTorqueState.version), and score each tune on plant-agnostic metrics.
Unlike the open-loop replay harnesses, every frame here is closed-loop ground truth from
whichever tune actually drove the car; the trade is that roads/conditions differ between
groups, so judge trends, not third decimals.

Metrics (active frames; tracking/weave/saturation additionally require vEgo > 5 and not
steeringPressed):
  track_rms   RMS(actualLateralAccel - lag-aligned desired lat accel). Scored against the
              plan request (controlsState.desiredCurvature * v^2) shifted by lagd's
              lateralDelay, NOT the tune's own setpoint -- a lagged setpoint (v1) is easy
              to track precisely because it is the wrong target.
  weave       band RMS (0.3-3 Hz) of the torque command: sustained oscillation / hunting.
  interventions  steeringPressed rising edges while active, per 100 km.
  handback    per release (pressed falling edge, active after): output range over the
              next 1 s -- the on-car analogue of the replay release-window metric.
  sat_duty    fraction of frames with |output| at the measured EPS rail (CX-5 schedule),
              not the logged saturated flag, whose semantics differ between tunes.

Results are cached per segment file (mtime-keyed) so reruns only decode new logs.

Usage:
  .venv/bin/python tools/mazda_long/tune_version_metrics.py [roots...] [--jobs N] [--refresh]
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

DEFAULT_ROOTS = [
  REPO_ROOT / "tools/mazda_long/device_data",
  REPO_ROOT / "tools/mazda_long/test_data",
]
CACHE_PATH = REPO_ROOT / "tools/mazda_long/test_data/tune_metrics_cache.jsonl"

DT = 0.01
V_MIN = 5.0                      # metrics below this are parking noise
HANDBACK_WINDOW = 100            # frames (1 s), matches the replay release metric
WEAVE_BAND = (0.3, 3.0)          # Hz
WEAVE_NFFT = 1024                # 10.24 s windows
SPEED_BINS = [(5.0, 10.0), (10.0, 15.0), (15.0, 40.0)]

# CX-5 2022 measured EPS rail (opendbc mazda values): ceiling(v) / STEER_MAX(v), clipped 1.
SM_BP, SM_V = [0.0, 14.2, 14.5], [1200.0, 1200.0, 800.0]
CEIL_BP = [8.0, 8.5, 9.4, 10.3, 11.2, 12.1, 13.0, 13.9, 14.5]
CEIL_V = [1148.0, 1132.0, 1092.0, 1048.0, 1012.0, 920.0, 808.0, 676.0, 620.0]


def rail_frac(v):
  return np.minimum(1.0, np.interp(v, CEIL_BP, CEIL_V) / np.interp(v, SM_BP, SM_V))


def analyze_segment(path: str) -> dict:
  from openpilot.tools.lib.logreader import LogReader

  rec: dict = {"path": path}
  version_counts: dict[int, int] = defaultdict(int)
  wall = None
  fp = None
  lat_delay = None
  cs_last = None

  active_l, out_l, actual_l, des_curv_l, v_l, pressed_l, ver_l = [], [], [], [], [], [], []
  try:
    for m in LogReader(path):
      w = m.which()
      if w == "initData" and wall is None:
        wall = m.initData.wallTimeNanos
      elif w == "carParams" and fp is None:
        fp = m.carParams.carFingerprint
      elif w == "lateralDelay":
        lat_delay = float(m.lateralDelay.lateralDelay)
      elif w == "carState":
        cs_last = m.carState
      elif w == "controlsState":
        st = m.controlsState.lateralControlState
        if st.which() != "torqueState" or cs_last is None:
          continue
        ts = st.torqueState
        version_counts[ts.version] += 1
        ver_l.append(ts.version)
        active_l.append(ts.active)
        out_l.append(ts.output)
        actual_l.append(ts.actualLateralAccel)
        des_curv_l.append(m.controlsState.desiredCurvature)
        v_l.append(cs_last.vEgo)
        pressed_l.append(cs_last.steeringPressed)
  except Exception as e:  # truncated/corrupt logs
    rec["error"] = f"{type(e).__name__}: {e}"
    return rec

  n = len(out_l)
  rec.update(fp=fp, wall_ns=wall, n_frames=n,
             versions={str(k): v for k, v in version_counts.items()})
  if n == 0:
    return rec

  active = np.array(active_l, dtype=bool)
  out = np.array(out_l, dtype=np.float64)
  actual = np.array(actual_l, dtype=np.float64)
  v = np.array(v_l, dtype=np.float64)
  pressed = np.array(pressed_l, dtype=bool)
  desired = np.array(des_curv_l, dtype=np.float64) * v ** 2
  ver = np.array(ver_l, dtype=np.int64)

  rec["version"] = int(max(version_counts, key=version_counts.get))
  rec["mixed_versions"] = len(version_counts) > 1
  rec["lat_delay"] = lat_delay
  rec["n_active"] = int(active.sum())
  rec["km_active"] = float((v * active).sum() * DT / 1000.0)
  if rec["n_active"] == 0:
    return rec

  # lag-aligned tracking error, scored against the plan request
  lag = int(round((lat_delay if lat_delay is not None else 0.3) / DT))
  desired_lagged = np.full(n, np.nan)
  if lag < n:
    desired_lagged[lag:] = desired[: n - lag]
  clean = active & ~pressed & (v > V_MIN) & ~np.isnan(desired_lagged)
  err = actual - desired_lagged
  rec["track"] = {"n": int(clean.sum()), "sum_sq": float(np.nansum(err[clean] ** 2))}
  rec["track_bins"] = {}
  for lo, hi in SPEED_BINS:
    m_ = clean & (v >= lo) & (v < hi)
    rec["track_bins"][f"{lo:g}-{hi:g}"] = {"n": int(m_.sum()), "sum_sq": float(np.nansum(err[m_] ** 2))}

  # weave: band RMS of the torque command over contiguous clean runs
  weave_sum_sq, weave_n = 0.0, 0
  idx = np.flatnonzero(np.diff(np.concatenate(([0], clean.view(np.int8), [0]))))
  for s, e in idx.reshape(-1, 2):
    for w0 in range(s, e - WEAVE_NFFT + 1, WEAVE_NFFT // 2):
      seg = out[w0:w0 + WEAVE_NFFT] * np.hanning(WEAVE_NFFT)
      spec = np.fft.rfft(seg)
      freqs = np.fft.rfftfreq(WEAVE_NFFT, DT)
      band = (freqs >= WEAVE_BAND[0]) & (freqs <= WEAVE_BAND[1])
      # band RMS with hann coherent-power correction
      weave_sum_sq += 2.0 * float(np.sum(np.abs(spec[band]) ** 2)) / (WEAVE_NFFT * np.sum(np.hanning(WEAVE_NFFT) ** 2))
      weave_n += 1
  rec["weave"] = {"n_windows": weave_n, "sum_sq": weave_sum_sq}

  # override interventions + hand-back transients
  press_rise = np.flatnonzero(~pressed[:-1] & pressed[1:] & active[1:])
  rec["interventions"] = int(len(press_rise))
  handbacks = []
  press_fall = np.flatnonzero(pressed[:-1] & ~pressed[1:])
  for i in press_fall:
    w0, w1 = i + 1, i + 1 + HANDBACK_WINDOW
    if w1 > n or not active[w0:w1].all() or pressed[w0:w1].any():
      continue
    window = out[w0:w1]
    handbacks.append({"range": float(window.max() - window.min()),
                      "slew": float(np.abs(np.diff(window)).max() / DT),
                      "v": float(v[w0])})
  rec["handbacks"] = handbacks

  # saturation duty against the measured rail (CX-5 only)
  if fp == "MAZDA_CX5_2022":
    at_rail = clean & (np.abs(out) >= rail_frac(v) - 1e-3)
    rec["sat"] = {"n": int(clean.sum()), "at_rail": int(at_rail.sum())}

  # guard against mid-log version flips contaminating the split
  if rec["mixed_versions"]:
    rec["version_frames"] = {str(k): int((ver == k).sum()) for k in version_counts}
  return rec


def discover(roots) -> list[str]:
  seen, files = set(), []
  for root in roots:
    root = Path(root)
    if not root.exists():
      continue
    for p in sorted(root.rglob("*rlog*.zst")):
      rp = os.path.realpath(p)
      if rp not in seen:
        seen.add(rp)
        files.append(rp)
  return files


def load_cache() -> dict:
  cache = {}
  if CACHE_PATH.exists():
    with open(CACHE_PATH) as f:
      for line in f:
        try:
          rec = json.loads(line)
          cache[(rec["path"], rec["mtime"])] = rec
        except (json.JSONDecodeError, KeyError):
          continue
  return cache


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("roots", nargs="*", default=None)
  ap.add_argument("--jobs", type=int, default=8)
  ap.add_argument("--refresh", action="store_true", help="ignore the cache")
  args = ap.parse_args()

  roots = args.roots or DEFAULT_ROOTS
  files = discover(roots)
  cache = {} if args.refresh else load_cache()
  todo = []
  results = []
  for p in files:
    mtime = os.path.getmtime(p)
    hit = cache.get((p, mtime))
    if hit is not None:
      results.append(hit)
    else:
      todo.append((p, mtime))

  print(f"{len(files)} rlogs discovered, {len(results)} cached, {len(todo)} to decode", flush=True)

  if todo:
    CACHE_PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(CACHE_PATH, "a") as cache_f, ProcessPoolExecutor(max_workers=args.jobs) as ex:
      futs = {ex.submit(analyze_segment, p): (p, mtime) for p, mtime in todo}
      done = 0
      for fut in as_completed(futs):
        p, mtime = futs[fut]
        try:
          rec = fut.result()
        except Exception as e:
          rec = {"path": p, "error": f"worker: {type(e).__name__}: {e}"}
        rec["mtime"] = mtime
        cache_f.write(json.dumps(rec) + "\n")
        cache_f.flush()
        results.append(rec)
        done += 1
        if done % 50 == 0:
          print(f"  {done}/{len(todo)} decoded", flush=True)

  report(results)


def agg_rms(pairs):
  n = sum(p["n"] for p in pairs)
  return (sum(p["sum_sq"] for p in pairs) / n) ** 0.5 if n else float("nan")


def report(results):
  import datetime

  errors = [r for r in results if "error" in r]
  usable = [r for r in results if r.get("n_active", 0) > 0 and r.get("fp") == "MAZDA_CX5_2022"]
  other_fp = {r.get("fp") for r in results if r.get("fp") not in (None, "MAZDA_CX5_2022")}
  print(f"\n{len(results)} rlogs: {len(usable)} CX-5 with active lateral frames, {len(errors)} decode errors, "
        + f"other cars skipped: {sorted(other_fp) or 'none'}")

  groups = defaultdict(list)
  for r in usable:
    groups[r["version"]].append(r)

  for ver in sorted(groups):
    segs = groups[ver]
    n_active = sum(r["n_active"] for r in segs)
    km = sum(r["km_active"] for r in segs)
    dates = [r["wall_ns"] for r in segs if r.get("wall_ns")]
    d0 = datetime.datetime.fromtimestamp(min(dates) / 1e9).strftime("%Y-%m-%d") if dates else "?"
    d1 = datetime.datetime.fromtimestamp(max(dates) / 1e9).strftime("%Y-%m-%d") if dates else "?"
    mixed = sum(r.get("mixed_versions", False) for r in segs)

    print(f"\n=== v{ver}  ({len(segs)} segments, {n_active / 100 / 3600:.1f} h active, {km:.0f} km, {d0} .. {d1}"
          + (f", {mixed} mixed" if mixed else "") + ") ===")
    print(f"  track err RMS (v>5, !pressed):  {agg_rms([r['track'] for r in segs]):.4f} m/s^2")
    for b in [f"{lo:g}-{hi:g}" for lo, hi in SPEED_BINS]:
      pairs = [r["track_bins"][b] for r in segs if b in r.get("track_bins", {})]
      nb = sum(p["n"] for p in pairs)
      print(f"    {b:>7} m/s: {agg_rms(pairs):.4f}  ({nb / 100 / 60:.0f} min)")
    weave_pairs = [{"n": r["weave"]["n_windows"], "sum_sq": r["weave"]["sum_sq"]} for r in segs]
    nw = sum(p["n"] for p in weave_pairs)
    weave = (sum(p["sum_sq"] for p in weave_pairs) / nw) ** 0.5 if nw else float("nan")
    print(f"  weave band RMS 0.3-3 Hz (torque): {weave:.4f}  ({nw} windows)")
    n_int = sum(r["interventions"] for r in segs)
    print(f"  interventions: {n_int} ({n_int / km * 100:.1f} / 100 km)" if km else "  interventions: n/a")
    hb = [h for r in segs for h in r.get("handbacks", [])]
    if hb:
      rng = np.array([h["range"] for h in hb])
      slew = np.array([h["slew"] for h in hb])
      print(f"  handback (n={len(hb)}): output range 1s  p50 {np.percentile(rng, 50):.3f}  p90 {np.percentile(rng, 90):.3f}  "
            + f"max {rng.max():.3f} | slew p90 {np.percentile(slew, 90):.2f}/s")
    sat_pairs = [r["sat"] for r in segs if "sat" in r]
    n_sat = sum(p["n"] for p in sat_pairs)
    if n_sat:
      print(f"  at-rail duty: {sum(p['at_rail'] for p in sat_pairs) / n_sat * 100:.2f}%")


if __name__ == "__main__":
  main()
