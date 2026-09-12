#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Inventory slow signaled turns and lane changes across the rlog corpus.

Two episode types, matching the two lateral-assist mechanisms under evaluation:

  slow_turn    one blinker on while below ~11 mph and not lane changing. Captures the
               model command collapse at intersections (the go-wide failure) plus the
               plan-probe signals the turn hold would have seen.
  lane_change  modelV2.meta.laneChangeState in starting/finishing. Captures the applied
               curvature rate to quantify the lateral jerk we command today, split into
               entry and arrest (post-peak unwind) phases.

Each episode is written as an npz window (carState at 100 Hz + modelV2 at 20 Hz,
including the 33-point plan) so a replay harness can drive the ported controllers with
the real inputs. A summary table prints per episode and aggregated.

Usage:
  python tools/mazda_long/scan_lateral_events.py 'tools/mazda_long/test_data/*' --out /tmp/lat_events
"""
from __future__ import annotations

import argparse
import glob
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

SLOW_TURN_SPEED = 5.0        # m/s, just above the hold release speed (4.47)
SLOW_TURN_MIN_T = 1.0        # s of sustained blinker+slow before it counts
WINDOW_PAD = 5.0             # s of context either side of an episode


def scan_segment(rlog: Path):
  from openpilot.tools.lib.logreader import LogReader
  cs_rows, mdl_rows = [], []
  mdl_plans = []
  try:
    lr = LogReader(str(rlog))
  except Exception as e:
    print(f"  ! {rlog}: {e}", file=sys.stderr)
    return None
  cs_last = None
  for m in lr:
    w = m.which()
    if w == 'carState':
      cs_last = m.carState
    elif w == 'modelV2':
      md = m.modelV2
      lc = str(md.meta.laneChangeState)
      mdl_rows.append((m.logMonoTime * 1e-9, md.action.desiredCurvature,
                       0 if lc == 'off' else (1 if lc == 'preLaneChange' else (2 if lc == 'laneChangeStarting' else 3))))
      mdl_plans.append((np.array(md.position.x, dtype=np.float32), np.array(md.position.y, dtype=np.float32)))
    elif w == 'controlsState' and cs_last is not None:
      st = m.controlsState.lateralControlState
      active = float(st.torqueState.active) if st.which() == 'torqueState' else 0.0
      cs_rows.append((m.logMonoTime * 1e-9, cs_last.vEgo, cs_last.aEgo,
                      float(cs_last.leftBlinker), float(cs_last.rightBlinker),
                      cs_last.steeringAngleDeg, float(cs_last.steeringPressed), cs_last.steeringTorque,
                      m.controlsState.desiredCurvature, m.controlsState.curvature, active))
  if not cs_rows or not mdl_rows:
    return None
  cs = np.array(cs_rows)
  mdl = np.array(mdl_rows)
  return cs, mdl, mdl_plans


def find_runs(mask, t, min_t=0.0, merge_gap=1.0):
  """[(i0, i1)] index runs where mask holds, merging gaps < merge_gap seconds."""
  idx = np.flatnonzero(mask)
  if not len(idx):
    return []
  runs = []
  start = prev = idx[0]
  for i in idx[1:]:
    if t[i] - t[prev] > merge_gap:
      runs.append((start, prev))
      start = i
    prev = i
  runs.append((start, prev))
  return [(a, b) for a, b in runs if t[b] - t[a] >= min_t]


def slice_window(cs, mdl, mdl_plans, t0, t1):
  ci = (cs[:, 0] >= t0) & (cs[:, 0] <= t1)
  mi = (mdl[:, 0] >= t0) & (mdl[:, 0] <= t1)
  midx = np.flatnonzero(mi)
  px = np.stack([mdl_plans[i][0] for i in midx]) if len(midx) else np.zeros((0, 33), np.float32)
  py = np.stack([mdl_plans[i][1] for i in midx]) if len(midx) else np.zeros((0, 33), np.float32)
  return cs[ci], mdl[mi], px, py


def lane_change_metrics(csw):
  """Applied lateral jerk (d desired_curvature/dt * v^2) over a lane-change window."""
  t, v, dk = csw[:, 0], csw[:, 1], csw[:, 8]
  if len(t) < 10:
    return None
  dt = np.diff(t)
  ok = dt > 1e-4
  jerk = np.abs(np.diff(dk)[ok] / dt[ok]) * np.maximum(v[1:][ok], 1.0) ** 2
  # split at the |curvature| peak: entry = wind-in, arrest = everything after
  peak = int(np.argmax(np.abs(dk)))
  jerk_entry = jerk[:max(peak - 1, 1)]
  jerk_arrest = jerk[max(peak - 1, 1):]
  return {'dur': t[-1] - t[0], 'v_mean': float(v.mean()),
          'jerk_p99': float(np.percentile(jerk, 99)) if len(jerk) else 0.0,
          'jerk_entry_p99': float(np.percentile(jerk_entry, 99)) if len(jerk_entry) else 0.0,
          'jerk_arrest_p99': float(np.percentile(jerk_arrest, 99)) if len(jerk_arrest) else 0.0}


def slow_turn_metrics(csw):
  """Model-collapse signature: wheel wound (measured curvature) while the applied
  command reads ~zero, and driver torque during."""
  v, ang, pressed, torque = csw[:, 1], csw[:, 5], csw[:, 6], csw[:, 7]
  dk, k_meas, active = csw[:, 8], csw[:, 9], csw[:, 10]
  slow = v < SLOW_TURN_SPEED
  wound = np.abs(k_meas) > 0.01
  collapse = slow & wound & (np.abs(dk) < 0.33 * np.abs(k_meas))
  drv = np.abs(torque[pressed > 0.5]) if (pressed > 0.5).any() else np.array([0.0])
  return {'dur': csw[-1, 0] - csw[0, 0], 'v_min': float(v.min()),
          'swept_deg': float(np.abs(np.diff(ang)).sum()) if len(ang) > 1 else 0.0,
          'max_meas_curv': float(np.abs(k_meas).max()),
          'collapse_frac': float(collapse[slow & wound].mean()) if (slow & wound).any() else 0.0,
          'pressed_frac': float((pressed > 0.5).mean()),
          'drv_torque_p95': float(np.percentile(drv, 95)),
          'active_frac': float(active.mean())}


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('paths', nargs='+')
  ap.add_argument('--out', default=None, help='directory for per-episode npz windows')
  args = ap.parse_args()

  rlogs = []
  for p in args.paths:
    for d in sorted(glob.glob(p)):
      d = Path(d)
      if d.is_file():
        rlogs.append(d)
      else:
        rlogs.extend(sorted(x for x in d.rglob('rlog*.zst')))
  print(f"{len(rlogs)} rlogs", flush=True)
  out_dir = Path(args.out) if args.out else None
  if out_dir:
    out_dir.mkdir(parents=True, exist_ok=True)

  lc_all, st_all = [], []
  for seg in rlogs:
    res = scan_segment(seg)
    if res is None:
      continue
    cs, mdl, mdl_plans = res
    t = cs[:, 0]
    one_blinker = (cs[:, 3] + cs[:, 4]) == 1.0
    # lane change state forward-filled onto the cs timeline
    lc_state = np.interp(t, mdl[:, 0], mdl[:, 2])
    in_lc = lc_state >= 1.5  # starting/finishing

    seg_name = f"{seg.parent.name}--{seg.stem}"
    for a, b in find_runs(in_lc, t, min_t=0.5):
      t0, t1 = t[a] - WINDOW_PAD, t[b] + WINDOW_PAD
      csw, mdlw, px, py = slice_window(cs, mdl, mdl_plans, t0, t1)
      m = lane_change_metrics(csw[(csw[:, 0] >= t[a]) & (csw[:, 0] <= t[b])])
      if m is None:
        continue
      lc_all.append((seg_name, t[a] - t[0], m))
      print(f"LC  {seg_name} +{t[a]-t[0]:5.1f}s dur {m['dur']:4.1f}s v {m['v_mean']*2.237:4.1f}mph " +
            f"jerk p99 {m['jerk_p99']:.2f} entry {m['jerk_entry_p99']:.2f} arrest {m['jerk_arrest_p99']:.2f} m/s^3")
      if out_dir:
        np.savez_compressed(out_dir / f"lc_{seg_name}_{t[a]-t[0]:.0f}.npz", cs=csw, mdl=mdlw, px=px, py=py)

    slow_turn = one_blinker & (cs[:, 1] < SLOW_TURN_SPEED) & ~in_lc
    for a, b in find_runs(slow_turn, t, min_t=SLOW_TURN_MIN_T, merge_gap=3.0):
      t0, t1 = t[a] - WINDOW_PAD, t[b] + WINDOW_PAD
      csw, mdlw, px, py = slice_window(cs, mdl, mdl_plans, t0, t1)
      m = slow_turn_metrics(csw[(csw[:, 0] >= t[a]) & (csw[:, 0] <= t[b])])
      if m['max_meas_curv'] < 0.02:  # never actually turned (waiting at a light, straight creep)
        continue
      st_all.append((seg_name, t[a] - t[0], m))
      print(f"ST  {seg_name} +{t[a]-t[0]:5.1f}s dur {m['dur']:4.1f}s vmin {m['v_min']:3.1f} " +
            f"curv {m['max_meas_curv']:.3f} collapse {m['collapse_frac']:.2f} " +
            f"pressed {m['pressed_frac']:.2f} drv p95 {m['drv_torque_p95']:.0f} active {m['active_frac']:.2f}")
      if out_dir:
        np.savez_compressed(out_dir / f"st_{seg_name}_{t[a]-t[0]:.0f}.npz", cs=csw, mdl=mdlw, px=px, py=py)

  print(f"\n==== {len(lc_all)} lane changes, {len(st_all)} slow turns ====")
  if lc_all:
    for k in ('jerk_p99', 'jerk_entry_p99', 'jerk_arrest_p99', 'dur'):
      vals = np.array([m[k] for _, _, m in lc_all])
      print(f"LC {k:16s} median {np.median(vals):5.2f}  p90 {np.percentile(vals, 90):5.2f}  max {vals.max():5.2f}")
  if st_all:
    for k in ('collapse_frac', 'pressed_frac', 'drv_torque_p95', 'max_meas_curv'):
      vals = np.array([m[k] for _, _, m in st_all])
      print(f"ST {k:16s} median {np.median(vals):5.2f}  p90 {np.percentile(vals, 90):5.2f}  max {vals.max():5.2f}")


if __name__ == '__main__':
  main()
