#!/usr/bin/env python3
# ruff: noqa: ISC002  (report printer: implicit f-string concatenation reads best here)
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Find the local rlogs with the best speed-bin torque learning, and propose seed updates.

torqued_ext learns latAccelFactor/friction independently per speed bin. Two things to know
when mining logs for seed values:

  - the bins ride on torqued_ext's own liveTorqueParametersSP message (customReserved19),
    published beside every lateralTorqueParameters; pre-2026-09 logs carried them on the
    upstream message and are read through speed_bin_log's legacy layout.
  - `speedBinPoints` is only attached to the cache write (every 60 s), so point counts
    have to be read off the last message that carries them, not the last message.
  - learning persists across drives through the LiveTorqueParameters param cache, so point
    counts accumulate and later segments of a route contain earlier segments' data. Snapshots
    are therefore NOT independent samples - prefer "the snapshot with the most points for this
    bin" over averaging across snapshots.
  - SVD output is clipped to +/-FACTOR_SANITY (30%) of the *current* TOML seed for laf and
    +/-FRICTION_SANITY (50%) for friction. A learned value sitting on a bound means the seed
    is holding it back and is the strongest signal that the seed needs to move.

Usage:
  python tools/mazda_long/scan_speed_bin_coverage.py                     # scan the usual dirs
  python tools/mazda_long/scan_speed_bin_coverage.py <dir|rlog> [...] [--jobs N] [--top N]
                                                     [--car MAZDA_CX5_2022] [--json out.json]
"""
from __future__ import annotations

import argparse
import json
import sys
import tomllib
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from openpilot.tools.lib.logreader import LogReader
from speed_bin_log import SpeedBinTracker

TOML_PATH = REPO_ROOT / "opendbc_repo/opendbc/car/torque_data/speed_dependent.toml"
FACTOR_SANITY = 0.3
FRICTION_SANITY = 0.5
MS_TO_MPH = 2.2369

DEFAULT_TARGETS = [
  REPO_ROOT / "tools/mazda_long/device_data",
  REPO_ROOT / "tools/mazda_long/device_data_speedmax",
  REPO_ROOT / "tools/mazda_long/test_data",
]


def load_seeds(car):
  with open(TOML_PATH, "rb") as f:
    cfg = tomllib.load(f)
  c = cfg.get(car, {})
  return list(c.get("speed_bp", [])), list(c.get("laf_bp", [])), list(c.get("friction_bp", []))


def scan_one(path):
  """Return the last values snapshot and the last snapshot carrying point counts."""
  fingerprint = None
  last_vals = None
  last_points = None
  n_msgs = 0
  bins = SpeedBinTracker()
  try:
    for ev in LogReader(str(path)):
      w = ev.which()
      if bins.feed(w, ev):
        continue
      if w == "carParams" and fingerprint is None:
        fingerprint = ev.carParams.carFingerprint
      elif w == "lateralTorqueParameters":
        tp = ev.lateralTorqueParameters
        sp = bins.bins_for(tp)
        centers = list(sp.speedBinCenters) if sp is not None else []
        if not centers:
          continue
        n_msgs += 1
        last_vals = {
          "centers": centers,
          "laf": list(sp.speedBinLatAccelFactors),
          "friction": list(sp.speedBinFrictions),
          "valid": list(sp.speedBinValid),
          "global_laf": float(tp.latAccelFactorFiltered),
          "global_friction": float(tp.frictionCoefficientFiltered),
          "live_valid": bool(tp.valid),
        }
        pts = list(sp.speedBinPoints)
        if pts and any(len(p) for p in pts):
          last_points = [len(p) for p in pts]
  except Exception as e:
    return {"path": str(path), "error": f"{type(e).__name__}: {e}"}

  if last_vals is None:
    return {"path": str(path), "error": "no speed-binned lateralTorqueParameters"}
  out = {"path": str(path), "fingerprint": fingerprint, "msgs": n_msgs, **last_vals}
  out["points"] = last_points if last_points else [0] * len(last_vals["centers"])
  out["has_points"] = last_points is not None
  return out


def depth_one(path, centers):
  """One pass: count the learner's own qualifying samples per bin, how long each bin stayed
  valid, and the largest LKAS torque command (which identifies the STEER_MAX era).

  laf is lateral accel per *normalized* torque, so laf scales with STEER_MAX. Values learned
  under a different STEER_MAX cannot be pasted into the seeds without rescaling.
  """
  bounds = [0.0] + [(centers[i] + centers[i + 1]) / 2 for i in range(len(centers) - 1)] + [99.0]
  n = len(centers)
  samples = [0] * n
  valid_msgs = [0] * n
  msgs = 0
  max_can = 0

  co_t, co_tq, cc_t, cc_act, cs_t, cs_v, cs_ovr = [], [], [], [], [], [], []
  pose_t, pose_roll, llk_t, llk_yaw = [], [], [], []
  lag = 0.34
  bins = SpeedBinTracker()
  try:
    for ev in LogReader(str(path)):
      w = ev.which()
      if bins.feed(w, ev):
        continue
      t = ev.logMonoTime * 1e-9
      if w == "carOutput":
        co_t.append(t)
        co_tq.append(-ev.carOutput.actuatorsOutput.torque)
      elif w == "carControl":
        cc_t.append(t)
        cc_act.append(1.0 if ev.carControl.latActive else 0.0)
      elif w == "carState":
        cs_t.append(t)
        cs_v.append(ev.carState.vEgo)
        cs_ovr.append(1.0 if ev.carState.steeringPressed else 0.0)
      elif w == "deviceMotion":
        pose_t.append(t)
        pose_roll.append(ev.deviceMotion.orientationNED.x)
      elif w == "liveLocationKalman":
        llk_t.append(t)
        llk_yaw.append(ev.liveLocationKalman.angularVelocityCalibrated.value[2])
      elif w == "lateralDelay":
        lag = ev.lateralDelay.lateralDelay
      elif w == "lateralTorqueParameters":
        sp = bins.bins_for(ev.lateralTorqueParameters)
        if sp is not None and list(sp.speedBinCenters):
          msgs += 1
          for i, v in enumerate(list(sp.speedBinValid)[:n]):
            valid_msgs[i] += int(bool(v))
      elif w == "sendcan":
        for m in ev.sendcan:
          if m.address == 0x243 and m.src == 0 and len(m.dat) >= 2:
            raw = (((m.dat[0] & 0x0F) << 8) | m.dat[1]) - 2048
            max_can = max(max_can, abs(raw))
  except Exception:
    pass

  if len(llk_t) > 20 and len(co_t) > 100 and len(pose_t) > 20:
    llk_t_a, llk_yaw_a = np.array(llk_t), np.array(llk_yaw)
    roll = np.interp(llk_t_a, pose_t, pose_roll)
    ts = llk_t_a - lag
    steer = np.interp(ts, co_t, co_tq)
    vego = np.interp(ts, cs_t, cs_v)
    act = np.interp(ts, cc_t, cc_act)
    ovr = np.interp(ts, cs_t, cs_ovr)
    lat_accel = vego * llk_yaw_a - np.sin(roll) * G_ACCEL
    # torqued's own admission filters, incl. LAT_ACC_THRESHOLD
    keep = ((act > 0.999) & (ovr < 0.001) & (np.abs(steer) > 0.02)
            & (np.abs(lat_accel) <= 1.0) & (vego > 3.0))
    vk = vego[keep]
    for i in range(n):
      samples[i] = int(((vk >= bounds[i]) & (vk < bounds[i + 1])).sum())

  return {"path": str(path), "samples": samples, "valid_msgs": valid_msgs,
          "msgs": msgs, "max_can": int(max_can)}


G_ACCEL = 9.81


def find_rlogs(targets):
  out = []
  for t in targets:
    p = Path(t)
    if p.is_file():
      out.append(p)
    elif p.is_dir():
      out.extend(sorted(p.rglob("rlog*.zst")))
  return out


def route_of(path):
  p = Path(path)
  # device_data/<route>--<seg>/rlog.zst  or  <dir>/rlog_<tag>_seg<N>.zst
  if p.name.startswith("rlog") and p.parent.name.count("--") >= 1:
    return p.parent.name.rsplit("--", 1)[0]
  return f"{p.parent.name}/{p.name.split('_seg')[0]}"


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("targets", nargs="*", default=None)
  ap.add_argument("--jobs", type=int, default=8)
  ap.add_argument("--top", type=int, default=15)
  ap.add_argument("--car", default="MAZDA_CX5_2022")
  ap.add_argument("--json", type=Path, default=None)
  args = ap.parse_args()

  targets = args.targets or DEFAULT_TARGETS
  rlogs = find_rlogs(targets)
  print(f"scanning {len(rlogs)} rlogs with {args.jobs} workers")

  seed_bp, seed_laf, seed_fric = load_seeds(args.car)
  print(f"current {args.car} seeds:")
  print(f"  speed_bp    = {seed_bp}")
  print(f"  laf_bp      = {seed_laf}")
  print(f"  friction_bp = {seed_fric}")

  snaps, errors, wrong_car, stale_cfg = [], [], 0, 0
  with ProcessPoolExecutor(max_workers=args.jobs) as ex:
    for i, r in enumerate(ex.map(scan_one, [str(p) for p in rlogs], chunksize=4), 1):
      if i % 200 == 0:
        print(f"  {i}/{len(rlogs)}", flush=True)
      if "error" in r:
        errors.append(r)
        continue
      if r["fingerprint"] and args.car and r["fingerprint"] != args.car:
        wrong_car += 1
        continue
      if seed_bp and (len(r["centers"]) != len(seed_bp)
                      or not np.allclose(r["centers"], seed_bp, atol=0.01)):
        stale_cfg += 1
        continue
      snaps.append(r)

  skip_note = f"(skipped: {len(errors)} unreadable/no-bins, {wrong_car} other car, {stale_cfg} different bin config)"
  print(f"\nusable snapshots: {len(snaps)}   {skip_note}")
  if not snaps:
    if stale_cfg:
      print("all snapshots use different bin centers than the current TOML - the cache is")
      print("rejected on config change, so those logs predate the current speed_bp.")
    return

  # no TOML entry yet (seeding a new car): work off the bin layout the logs learned on,
  # and compare against the offline global values, which is what the learner seeds from
  if not seed_bp:
    seed_bp = list(snaps[0]["centers"])
    with open(TOML_PATH.parent / "params.toml", "rb") as f:
      offline = tomllib.load(f).get(args.car)
    seed_laf = [offline[0] if offline else 0.0] * len(seed_bp)
    seed_fric = [offline[2] if offline else 0.0] * len(seed_bp)
    print(f"no TOML seeds for {args.car}; bin centers from the logs: {seed_bp}")
    print(f"reference = offline globals (the unconfigured learner's seed): laf {seed_laf[0]:.2f}, friction {seed_fric[0]:.3f}")
  n_bins = len(seed_bp)
  for s in snaps:
    s["n_valid"] = sum(1 for x in s["valid"] if x)
    s["total_points"] = sum(s["points"])
    s["route"] = route_of(s["path"])


  # measure learning depth (rlogs don't carry speedBinPoints)
  # torqued_ext only attaches speedBinPoints to the param-cache write, never to the published
  # message, so logged point counts are always zero.
  # Instead: count the samples each segment contributes under torqued's own admission
  # filters, and count how long each bin held valid.
  print(f"\nmeasuring learning depth on {len(snaps)} usable segments (rlogs carry no point counts, so counting admissible samples directly)")
  depth = {}
  with ProcessPoolExecutor(max_workers=args.jobs) as ex:
    for r in ex.map(depth_one, [s["path"] for s in snaps], [seed_bp] * len(snaps), chunksize=1):
      depth[r["path"]] = r
  for s in snaps:
    d = depth.get(s["path"], {})
    s["samples"] = d.get("samples", [0] * n_bins)
    s["valid_msgs"] = d.get("valid_msgs", [0] * n_bins)
    s["ltp_msgs"] = d.get("msgs", 0)
    s["max_can"] = d.get("max_can", 0)

  # aggregate per route
  def seg_index(path):
    stem = Path(path).parent.name
    try:
      return int(stem.rsplit("--", 1)[1])
    except (IndexError, ValueError):
      return 0

  routes = {}
  for s in snaps:
    r = routes.setdefault(s["route"], {"segs": [], "samples": [0] * n_bins,
                                       "valid_msgs": [0] * n_bins, "ltp_msgs": 0, "max_can": 0})
    r["segs"].append(s)
    for i in range(n_bins):
      r["samples"][i] += s["samples"][i]
      r["valid_msgs"][i] += s["valid_msgs"][i]
    r["ltp_msgs"] += s["ltp_msgs"]
    r["max_can"] = max(r["max_can"], s["max_can"])
  for r in routes.values():
    # the last segment holds the most accumulated learning for that route
    r["last"] = max(r["segs"], key=lambda s: seg_index(s["path"]))
    r["n_valid"] = r["last"]["n_valid"]
    # only a >800 command proves the 1200 era; never exceeding 800 proves nothing, since a
    # short or gentle route may simply never have asked for that much
    r["era"] = "1200" if r["max_can"] > 805 else "unconfirmed"

  print(f"\n{'=' * 112}")
  print("ROUTES RANKED BY BINS LEARNED, THEN ADMISSIBLE SAMPLES")
  print("=" * 112)
  print(f"{'route':>22} {'segs':>5} {'valid':>6} {'samples':>8} {'STEER_MAX':>10}  "
        + " ".join(f"{c:>7.1f}" for c in seed_bp))
  ranked_routes = sorted(routes.items(), key=lambda kv: (kv[1]["n_valid"], sum(kv[1]["samples"])),
                         reverse=True)
  for name, r in ranked_routes[:args.top]:
    cells = " ".join(f"{r['samples'][i]:>7,}" if r["last"]["valid"][i] else f"{'-':>7}"
                     for i in range(n_bins))
    print(f"{name:>22} {len(r['segs']):>5} {r['n_valid']:>4}/{n_bins} {sum(r['samples']):>8,} "
          f"{r['era']:>10}  {cells}")
  print("  cells = admissible samples that route contributed to each bin ('-' = bin not valid at")
  print("  the end of the route). STEER_MAX is inferred from the largest LKAS command seen:")
  print("  laf scales with STEER_MAX, so a mismatched era cannot be pasted in without rescaling.")

  eras = {r["era"] for r in routes.values() if r["era"] != "unconfirmed"}
  print(f"\nSTEER_MAX eras confirmed: {sorted(eras) or ['none']}"
        + ("   (consistent - no rescaling needed)" if len(eras) <= 1
           else "   MIXED - low-speed laf must be rescaled by SM_current/SM_logged"))

  # per-bin best source
  print(f"\n{'=' * 112}")
  print("PER-BIN BEST: route with the most admissible samples for that bin, value from its last segment")
  print("=" * 112)
  print(f"{'bin':>7} {'mph':>5} {'samples':>8} {'laf':>7} {'seed':>7} {'delta':>7} {'pin':>5} "
        f"{'fric':>7} {'seed':>7} {'delta':>7} {'pin':>5}  source")
  best_laf, best_fric, best_src, notes = [], [], [], []
  for i in range(n_bins):
    cands = [(nm, r) for nm, r in routes.items() if r["last"]["valid"][i]]
    if not cands:
      print(f"{seed_bp[i]:>7.1f} {seed_bp[i] * MS_TO_MPH:>5.0f} {'-':>8}   never valid in any "
            f"local rlog - keeping seed {seed_laf[i]:.2f} / {seed_fric[i]:.3f}")
      best_laf.append(seed_laf[i])
      best_fric.append(seed_fric[i])
      best_src.append("seed (never learned)")
      notes.append("never learned")
      continue
    nm, r = max(cands, key=lambda kv: kv[1]["samples"][i])
    laf, fric = r["last"]["laf"][i], r["last"]["friction"][i]
    lo_f, hi_f = (1 - FACTOR_SANITY) * seed_laf[i], (1 + FACTOR_SANITY) * seed_laf[i]
    lo_r, hi_r = (1 - FRICTION_SANITY) * seed_fric[i], (1 + FRICTION_SANITY) * seed_fric[i]
    pin_f = "LOW" if laf <= lo_f * 1.01 else ("HIGH" if laf >= hi_f * 0.99 else "")
    pin_r = "LOW" if fric <= lo_r * 1.01 else ("HIGH" if fric >= hi_r * 0.99 else "")
    print(f"{seed_bp[i]:>7.1f} {seed_bp[i] * MS_TO_MPH:>5.0f} {r['samples'][i]:>8,} "
          f"{laf:>7.3f} {seed_laf[i]:>7.2f} {100 * (laf - seed_laf[i]) / seed_laf[i]:>+6.0f}% {pin_f:>5} "
          f"{fric:>7.3f} {seed_fric[i]:>7.3f} {100 * (fric - seed_fric[i]) / seed_fric[i]:>+6.0f}% {pin_r:>5}  "
          f"{nm} ({r['era']})")
    best_laf.append(laf)
    best_fric.append(fric)
    best_src.append(f"{nm} seg{seg_index(r['last']['path'])}")
    notes.append(f"{r['samples'][i]} samples" + (f", laf pinned {pin_f}" if pin_f else ""))
  print("  'pin' = the learned value is sitting on the +/-30% (laf) / +/-50% (friction) sanity")
  print("  bound around the current seed, so the true value is probably outside it.")

  # cross-route agreement
  print(f"\n{'=' * 112}")
  print("CROSS-ROUTE AGREEMENT (one value per route, from its last segment)")
  print("=" * 112)
  print(f"{'bin':>7} {'routes':>7} {'laf min':>8} {'laf med':>8} {'laf max':>8} {'spread':>7} "
        f"{'fric med':>9}")
  for i in range(n_bins):
    vals = [r["last"]["laf"][i] for r in routes.values() if r["last"]["valid"][i]]
    fr = [r["last"]["friction"][i] for r in routes.values() if r["last"]["valid"][i]]
    if not vals:
      print(f"{seed_bp[i]:>7.1f} {0:>7}   (never valid)")
      continue
    v = np.array(vals)
    print(f"{seed_bp[i]:>7.1f} {len(v):>7} {v.min():>8.3f} {float(np.median(v)):>8.3f} "
          f"{v.max():>8.3f} {100 * (v.max() - v.min()) / float(np.median(v)):>6.0f}% "
          f"{float(np.median(fr)):>9.3f}")

  # proposed TOML
  print(f"\n{'=' * 112}")
  print("PROPOSED SEEDS")
  print("=" * 112)
  print(f"[{args.car}]")
  print(f"speed_bp    = {[round(x, 1) for x in seed_bp]}")
  print(f"laf_bp      = {[round(float(x), 2) for x in best_laf]}")
  print(f"friction_bp = {[round(float(x), 3) for x in best_fric]}")
  print()
  for i in range(n_bins):
    print(f"  {seed_bp[i]:>5.1f} m/s: {best_src[i]:<34} {notes[i]}")
  print("\nCaveats before pasting:")
  print("  - seeds define the +/-30%/50% learning window, so any bin flagged 'pin' will keep")
  print("    migrating after the update and needs a second pass to settle.")
  print("  - bins that were never valid locally keep their existing seed; a drive that covers")
  print("    those speeds is the only way to improve them.")
  print("  - the device's own LiveTorqueParameters param is the authoritative accumulated state")
  print("    (it is the only place with real point counts): analyze_speed_dep_torque.py <ip>")

  if args.json:
    args.json.write_text(json.dumps({
      "car": args.car, "speed_bp": seed_bp,
      "current_laf": seed_laf, "current_friction": seed_fric,
      "proposed_laf": [float(x) for x in best_laf],
      "proposed_friction": [float(x) for x in best_fric],
      "sources": best_src, "notes": notes,
      "routes": {nm: {"n_valid": r["n_valid"], "segs": len(r["segs"]), "era": r["era"],
                      "samples": r["samples"], "laf": r["last"]["laf"],
                      "friction": r["last"]["friction"], "valid": [bool(x) for x in r["last"]["valid"]],
                      "last": r["last"]["path"]}
                 for nm, r in ranked_routes},
    }, indent=2))
    print(f"\nwrote {args.json}")


if __name__ == "__main__":
  main()
