#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Replay the 2022-EPS non-delivery alert over every captured drive and count what arms it.

The latch and its banner live in opendbc mazda/carstate.py update_steer_undelivered and are
fed only by STEER_RATE (0x241): LKAS_REQUEST, LKAS_EFFECTIVE, LKAS_BLOCK, LKAS_TRACK_STATE, plus
vEgoRaw. Every rlog carries those, so any candidate threshold can be scored against the whole
corpus before it is driven. This tool decodes each segment once into a cache, then replays the
state machine under several candidate parameter sets and prints, per candidate, every arming
with the context that says whether it was a real loss of steering (a camera ERR_BIT_1 that
followed, rejected 0x243 frames beforehand, how the block began) or a launch standby.

The default candidate mirrors the shipped constants; --verify replays the shipped CarState
itself on a sample of segments and asserts the mirror arms on the same frames.

Usage:
  replay_undelivered_alert.py                 # decode (cached) + score the built-in candidates
  replay_undelivered_alert.py --verify 40     # fidelity check of the mirror against carstate.py
  replay_undelivered_alert.py --car "CX-5 2022" --workers 8
"""
import argparse
import glob
import hashlib
import json
import os
import re
import sys
from multiprocessing import Pool

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)
HERE = os.path.dirname(os.path.abspath(__file__))
CACHE = os.path.join(HERE, ".undelivered_cache")
CACHE_VER = 1

DATASETS = ("device_data", "lkas_block_c4", "test_data", "device_routes", "drive_logs", "device_data_speedmax")

STEER_RATE = 0x241
CAM_LKAS = 0x243
TX_REJECTED = 192  # CAN_REJECTED_BUS_OFFSET | bus 0
MPH = 0.44704


def decode_steer_rate(dat):
  """LKAS_REQUEST 3|12@0+, LKAS_EFFECTIVE 39|12@0+, LKAS_BLOCK 50|1@1+, LKAS_TRACK_STATE 52|1@0+."""
  req = (((dat[0] & 0xf) << 8) | dat[1]) - 2048
  eff = ((dat[4] << 4) | (dat[5] >> 4)) - 2048
  blocked = (dat[6] >> 2) & 1
  track = (dat[6] >> 4) & 1
  return req, eff, blocked, track


def find_rlogs():
  seen = set()
  out = []
  for ds in DATASETS:
    root = os.path.join(HERE, ds)
    for p in sorted(glob.glob(os.path.join(root, "**", "rlog*"), recursive=True)):
      if not p.endswith((".zst", ".bz2", "rlog")):
        continue
      key = os.path.relpath(p, root)
      if key in seen:  # lkas_block_c4 overlaps device_data on a few segments
        continue
      seen.add(key)
      out.append(p)
  return out


def cache_path(p):
  return os.path.join(CACHE, hashlib.md5(p.encode()).hexdigest() + f".v{CACHE_VER}.npz")


def decode(path):
  cp = cache_path(path)
  if os.path.exists(cp):
    return cp
  from openpilot.tools.lib.logreader import LogReader

  t, req, eff, blocked, track, v, lat, pressed = [], [], [], [], [], [], [], []
  rejected_t, err_t = [], []
  meta = {"path": path, "car": "", "commit": "", "branch": "", "device_fault_frames": 0}
  v_ego, lat_active, press = 0.0, False, False
  err_last = None
  t0 = None
  try:
    for m in LogReader(path):
      w = m.which()
      if w == "carState":
        v_ego = m.carState.vEgoRaw
        press = m.carState.steeringPressed
        meta["device_fault_frames"] += int(m.carState.steerFaultTemporary)
        continue
      if w == "carControl":
        lat_active = m.carControl.latActive
        continue
      if w == "carParams":
        meta["car"] = m.carParams.carFingerprint
        continue
      if w == "initData":
        meta["commit"] = m.initData.gitCommit[:10]
        meta["branch"] = m.initData.gitBranch
        continue
      if w != "can":
        continue
      ts = m.logMonoTime * 1e-9
      if t0 is None:
        t0 = ts
      for c in m.can:
        if c.address == STEER_RATE and c.src == 0:
          r, e, b, k = decode_steer_rate(c.dat)
          t.append(ts - t0)
          req.append(r)
          eff.append(e)
          blocked.append(b)
          track.append(k)
          v.append(v_ego)
          lat.append(lat_active)
          pressed.append(press)
        elif c.address == CAM_LKAS:
          if c.src == 2:
            err = c.dat[2] & 1
            if err_last == 0 and err == 1:
              err_t.append(ts - t0)
            err_last = err
          elif c.src == TX_REJECTED:
            rejected_t.append(ts - t0)
  except Exception as e:  # a truncated rlog is still a segment; keep what decoded
    meta["error"] = repr(e)
  os.makedirs(CACHE, exist_ok=True)
  np.savez(cp, t=np.array(t), req=np.array(req, dtype=np.int16), eff=np.array(eff, dtype=np.int16),
           blocked=np.array(blocked, dtype=np.int8), track=np.array(track, dtype=np.int8),
           v=np.array(v, dtype=np.float32), lat=np.array(lat, dtype=np.int8), pressed=np.array(pressed, dtype=np.int8),
           rejected_t=np.array(rejected_t), err_t=np.array(err_t), meta=json.dumps(meta))
  return cp


def load(cp):
  z = np.load(cp)
  d = {k: z[k] for k in z.files if k != "meta"}
  d["meta"] = json.loads(str(z["meta"]))
  return d


# ---------------------------------------------------------------------------------------------
# the state machine, mirrored from opendbc mazda/carstate.py update_steer_undelivered

DEFAULT = {"req_min": 200, "latch_frames": 20, "alert_frames": 80, "min_speed": 12. * MPH,
           "track_gate": True, "origin_max_speed": None, "origin_min_speed": 1.0}


def fresh_state():
  return {"frames": 0, "und": False, "alert": False, "in_block": False, "v_origin": None, "b0": None}


def replay(seg, p, state=None):
  """Return the frame indices where the alert arms, each with the block's start index in this
  segment (None if it began in an earlier one) and the speed the block began at. Pass the
  returned state into the next segment of the same route so a block spanning a boundary is
  one block, as it is on the car."""
  st = state or fresh_state()
  frames, und, alert = st["frames"], st["und"], st["alert"]
  in_block, v_origin, b0 = st["in_block"], st["v_origin"], None if st["in_block"] else None
  arms = []
  t, req, eff, blocked, track, v = seg["t"], seg["req"], seg["eff"], seg["blocked"], seg["track"], seg["v"]
  for i in range(len(t)):
    b = bool(blocked[i])
    if b and not in_block:
      in_block = True
      v_origin = float(v[i])
      b0 = i
    if not b:
      frames = 0
      und = False
      alert = False
      in_block = False
      v_origin = None
      b0 = None
    elif not und:
      if eff[i] == 0 and abs(int(req[i])) > p["req_min"]:
        frames += 1
        und = frames >= p["latch_frames"]
      else:
        frames = 0
    if und:
      frames += 1
      if (not alert and (not p["track_gate"] or not track[i]) and
          frames >= p["latch_frames"] + p["alert_frames"] and v[i] >= p["min_speed"] and
          (p["origin_min_speed"] is None or v_origin >= p["origin_min_speed"]) and
          (p["origin_max_speed"] is None or v_origin <= p["origin_max_speed"])):
        alert = True
        arms.append((i, b0, v_origin))
  return arms, {"frames": frames, "und": und, "alert": alert, "in_block": in_block, "v_origin": v_origin, "b0": None}


def route_key(path):
  """(route id, segment number) so a route's segments replay in order with carried state."""
  d = os.path.dirname(path)
  m = re.search(r"(.+)--(\d+)$", os.path.basename(d))
  if m:
    return os.path.join(os.path.dirname(d), m.group(1)), int(m.group(2))
  m = re.search(r"rlog(?:_([0-9a-f]+))?_seg(\d+)\.", os.path.basename(path))
  if m:
    return os.path.join(d, m.group(1) or ""), int(m.group(2))
  return d, 0


def by_route(segs):
  routes = {}
  for s in segs:
    r, n = route_key(s["meta"]["path"])
    routes.setdefault(r, []).append((n, s))
  return {r: [s for _, s in sorted(v, key=lambda x: x[0])] for r, v in routes.items()}


def census(routes, p):
  """Where do LKAS_BLOCK episodes begin, and which of them could arm the alert at all?"""
  bins = [(0, 0.1), (0.1, 0.5), (0.5, 1), (1, 2), (2, 3), (3, 5), (5, 10), (10, 99)]
  rows = []  # (v_origin, reached_speed, track_cleared, latched, fault_or_starved, dur)
  for segs in routes.values():
    cur = None
    for seg in segs:
      blocked, track, v, req, eff, t = seg["blocked"], seg["track"], seg["v"], seg["req"], seg["eff"], seg["t"]
      rej = seg["rejected_t"]
      for i in range(len(t)):
        if blocked[i]:
          if cur is None:
            cur = {"v0": float(v[i]), "vmax": 0.0, "trk_clear": False, "run": 0, "latched": False, "bad": False, "dur": 0}
          cur["vmax"] = max(cur["vmax"], float(v[i]))
          cur["trk_clear"] |= not track[i]
          cur["dur"] += 1
          if not cur["latched"]:
            cur["run"] = cur["run"] + 1 if (eff[i] == 0 and abs(int(req[i])) > p["req_min"]) else 0
            cur["latched"] = cur["run"] >= p["latch_frames"]
          if len(rej) and ((rej >= t[i] - 10.0) & (rej <= t[i])).any():
            cur["bad"] = True
          if len(seg["err_t"]) and ((seg["err_t"] >= t[i]) & (seg["err_t"] <= t[i] + 10.0)).any():
            cur["bad"] = True
        elif cur is not None:
          rows.append(cur)
          cur = None
    if cur is not None:
      rows.append(cur)
  print(f"\n=== block origin census: {len(rows)} LKAS_BLOCK episodes ===")
  print(f"{'v at block start':18s} {'all':>6s} {'reach ' + str(round(p['min_speed'], 2)):>10s} {'+trk clr':>9s} {'+latched':>9s} {'fault/starved':>14s}")
  for lo, hi in bins:
    sel = [r for r in rows if lo <= r["v0"] < hi]
    reach = [r for r in sel if r["vmax"] >= p["min_speed"]]
    trk = [r for r in reach if r["trk_clear"]]
    lat = [r for r in trk if r["latched"]]
    bad = [r for r in lat if r["bad"]]
    print(f"{lo:5.1f} - {hi:5.1f} m/s   {len(sel):6d} {len(reach):10d} {len(trk):9d} {len(lat):9d} {len(bad):14d}")


def verify(paths, n):
  """Replay the shipped CarState on n segments and demand the same arming frames."""
  from opendbc.car import gen_empty_fingerprint
  from opendbc.car.mazda.carstate import CarState
  from opendbc.car.mazda.interface import CarInterface
  from opendbc.car.mazda.values import CAR
  CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, gen_empty_fingerprint(), [], alpha_long=False, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, gen_empty_fingerprint(), [], alpha_long=False, is_release_sp=False, docs=False)
  params = CarState(CP, CP_SP).params
  shipped = {"req_min": params.STEER_UNDELIVERED_MIN, "latch_frames": params.STEER_UNDELIVERED_FRAMES,
             "alert_frames": params.STEER_UNDELIVERED_ALERT_FRAMES, "min_speed": params.STEER_UNDELIVERED_ALERT_MIN_SPEED,
             "track_gate": True, "origin_max_speed": None,
             "origin_min_speed": params.STEER_UNDELIVERED_ALERT_ORIGIN_SPEED}
  checked = armed = 0
  # bias the sample towards segments with blocks at speed, otherwise verification is trivial
  segs = [load(decode(p)) for p in paths]
  segs.sort(key=lambda s: -int(((s["blocked"] == 1) & (s["v"] > shipped["min_speed"])).sum()))
  for seg in segs[:n]:
    cs = CarState(CP, CP_SP)
    ours = [i for i, _, _ in replay(seg, shipped)[0]]
    theirs = []
    prev = False
    for i in range(len(seg["t"])):
      cs.lkas_effective = int(seg["eff"][i])
      cs.update_steer_undelivered(float(seg["v"][i]), int(seg["req"][i]), bool(seg["blocked"][i]), bool(seg["track"][i]))
      if cs.steer_undelivered_alert and not prev:
        theirs.append(i)
      prev = cs.steer_undelivered_alert
    assert ours == theirs, (seg["meta"]["path"], ours, theirs)
    checked += 1
    armed += len(theirs)
  print(f"verify: {checked} segments, {armed} armings, mirror == carstate.py on every frame")


CANDIDATES = {
  "shipped 09-05: 1.0 s / 12 mph / TRACK_STATE / began above 1 m/s": dict(DEFAULT),
  "09-02: no origin gate": dict(DEFAULT, origin_min_speed=None),
  "08-31: no origin gate, no TRACK_STATE gate": dict(DEFAULT, origin_min_speed=None, track_gate=False),
  "no origin gate, hold 2.0 s": dict(DEFAULT, origin_min_speed=None, alert_frames=180),
  "no origin gate, hold 3.0 s": dict(DEFAULT, origin_min_speed=None, alert_frames=280),
  "no origin gate, 15 mph": dict(DEFAULT, origin_min_speed=None, min_speed=15. * MPH),
  "no origin gate, 20 mph": dict(DEFAULT, origin_min_speed=None, min_speed=20. * MPH),
  "began above 0.5 m/s": dict(DEFAULT, origin_min_speed=0.5),
  "began above 3 m/s": dict(DEFAULT, origin_min_speed=3.0),
  "began above 1 m/s, no TRACK_STATE gate": dict(DEFAULT, track_gate=False),
}


def describe(seg, i, b0, v_origin):
  t, v, blocked, track = seg["t"], seg["v"], seg["blocked"], seg["track"]
  b0 = 0 if b0 is None else b0
  end = b0
  while end + 1 < len(blocked) and blocked[end + 1]:
    end += 1
  t_arm, t0, t1 = t[i], t[b0], t[end]
  err_after = [e for e in seg["err_t"] if t0 <= e <= t1 + 10.0]
  rej_before = int(((seg["rejected_t"] >= t0 - 10.0) & (seg["rejected_t"] <= t1)).sum()) if len(seg["rejected_t"]) else 0
  track_all = bool(track[b0:end + 1].all())
  track_none = not bool(track[b0:end + 1].any())
  m = seg["meta"]
  name = os.path.relpath(m["path"], HERE)
  kind = "FAULT" if err_after else ("starved" if rej_before else "")
  track_s = "all" if track_all else "none" if track_none else "mixed"
  head = f"{name:60s} arm@{t_arm:6.1f}s v={v[i]:4.1f}  block {t0:6.1f}-{t1:6.1f}s ({t1 - t0:4.1f}s)"
  body = f"v_origin={v_origin:4.1f} v_max={v[b0:end + 1].max():4.1f} track={track_s} rej={rej_before:3d}"
  tail = f"err={'yes' if err_after else 'no '} {kind} [{m['branch']}@{m['commit']}]"
  return " ".join((head, body, tail))


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--car", default="CX5_2022", help="substring of carFingerprint to keep (segments with no carParams are kept too)")
  ap.add_argument("--workers", type=int, default=8)
  ap.add_argument("--verify", type=int, default=0)
  ap.add_argument("--decode-only", action="store_true")
  args = ap.parse_args()

  paths = find_rlogs()
  print(f"{len(paths)} rlogs under {', '.join(DATASETS)}")
  with Pool(args.workers) as pool:
    cps = []
    for n, cp in enumerate(pool.imap_unordered(decode, paths, chunksize=4), 1):
      cps.append(cp)
      if n % 200 == 0:
        print(f"  decoded {n}/{len(paths)}", flush=True)
  if args.decode_only:
    return
  if args.verify:
    verify(paths, args.verify)

  segs = [load(cp) for cp in cps]
  cars = {}
  for s in segs:
    cars[s["meta"]["car"]] = cars.get(s["meta"]["car"], 0) + 1
  print("cars:", cars)
  segs = [s for s in segs if (args.car in s["meta"]["car"] or not s["meta"]["car"]) and len(s["t"])]
  hours = sum(float(s["t"][-1]) for s in segs if len(s["t"])) / 3600
  print(f"{len(segs)} segments ({hours:.1f} h) match --car '{args.car}'")
  dev = sum(s["meta"]["device_fault_frames"] > 0 for s in segs)
  print(f"segments where the device itself raised steerFaultTemporary: {dev}")

  routes = by_route(segs)
  print(f"{len(routes)} routes")
  for name, p in CANDIDATES.items():
    lines = []
    for segs_r in routes.values():
      st = None
      for s in segs_r:
        arms, st = replay(s, p, st)
        for i, b0, v_origin in arms:
          lines.append(describe(s, i, b0, v_origin))
    real = sum(1 for ln in lines if "FAULT" in ln or "starved" in ln)
    print(f"\n=== {name}: {len(lines)} armings, {real} with a fault or starvation in the record ===")
    for ln in lines:
      print("  " + ln)
  census(routes, DEFAULT)


if __name__ == "__main__":
  main()
