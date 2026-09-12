#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Scan rlogs for the EPS's LKAS_FAULT bit (STEER_RATE byte 6 bit 5, message bit 53) and the camera's
ERR_BIT_1 that follows it.

Per segment: every run of the bit with what preceded it (the longest gap in delivered 0x243 over the
two seconds before, rejected frames, the request the EPS was echoing) and how long after it the camera
faulted. Delivered 0x243 is src 128 (an accepted openpilot frame or, on builds that log them, a
forwarded camera frame); src 192 is a panda rejection. The EPS echo (LKAS_REQUEST in STEER_RATE) is
the ground truth for what it received.

Usage:
  lkas_fault_scan.py RLOG [RLOG ...]          # print runs and faults per segment
  lkas_fault_scan.py --all [--json OUT]       # every rlog under tools/mazda_long, 8 workers
"""
import argparse
import bisect
import json
import os
import sys
from multiprocessing import Pool

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

STEER_RATE = 0x241
CAM_LKAS = 0x243
TX_ACCEPTED = 128
TX_REJECTED = 192


def scan(path):
  from openpilot.tools.lib.logreader import LogReader

  t0 = None
  v = 0.0
  meta = {"path": path}
  eps = []      # (t, byte6, request, effective, v)
  deliv = []    # t of every src 128 CAM_LKAS
  rej = []      # t of every src 192 CAM_LKAS
  err = []      # t of every camera ERR_BIT_1 rise
  err_last = None
  try:
    for m in LogReader(path):
      w = m.which()
      if w == "carState":
        v = m.carState.vEgoRaw
        continue
      if w == "initData":
        meta["commit"] = m.initData.gitCommit[:10]
        continue
      if w == "carParams":
        meta["car"] = m.carParams.carFingerprint
        meta["long"] = m.carParams.openpilotLongitudinalControl
        continue
      if w != "can":
        continue
      t = m.logMonoTime * 1e-9
      if t0 is None:
        t0 = t
      rt = t - t0
      for c in m.can:
        d = bytes(c.dat)
        if c.address == STEER_RATE and c.src == 0:
          eps.append((rt, d[6], (((d[0] & 0xf) << 8) | d[1]) - 2048, ((d[4] << 4) | (d[5] >> 4)) - 2048, v))
        elif c.address == CAM_LKAS:
          if c.src == 2:
            e = d[2] & 1
            if err_last == 0 and e == 1:
              err.append(rt)
            err_last = e
          elif c.src == TX_ACCEPTED:
            deliv.append(rt)
          elif c.src == TX_REJECTED:
            rej.append(rt)
  except Exception as e:  # a truncated rlog is still a segment; keep what decoded
    meta["error"] = repr(e)

  runs = []
  i = 0
  while i < len(eps):
    j = i
    while j + 1 < len(eps) and (eps[j + 1][1] >> 5) & 1 == (eps[i][1] >> 5) & 1:
      j += 1
    if (eps[i][1] >> 5) & 1:
      ta, tb = eps[i][0], eps[j][0]
      k0, k1 = bisect.bisect_left(deliv, ta - 2.0), bisect.bisect_left(deliv, ta)
      times = deliv[k0:k1] + [ta]
      gaps = [b - a for a, b in zip(times, times[1:])]
      faults = [e for e in err if ta - 0.2 <= e <= tb + 1.0]
      runs.append({
        "t0": round(ta, 3), "dur": round(tb - ta, 3), "v0": round(eps[i][4], 2),
        "prev_byte6": f"{eps[i - 1][1]:02x}" if i > 0 else None,
        "echo_before": eps[i - 1][2] if i > 0 else None,
        "gap_before": round(max(gaps), 3) if len(gaps) > 1 else None,
        "rejected_before": sum(1 for r in rej if ta - 2.0 <= r <= ta),
        "fault_dt": round(faults[0] - ta, 3) if faults else None,
        "censored": i == 0 or j == len(eps) - 1,
      })
    i = j + 1
  meta["runs"] = runs
  meta["faults"] = [round(e, 3) for e in err]
  meta["byte6"] = {}
  for e in eps:
    meta["byte6"][f"{e[1]:02x}"] = meta["byte6"].get(f"{e[1]:02x}", 0) + 1
  return meta


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("rlogs", nargs="*")
  ap.add_argument("--all", action="store_true", help="every rlog replay_undelivered_alert.py knows")
  ap.add_argument("--json", help="write every segment's result here")
  ap.add_argument("--workers", type=int, default=8)
  args = ap.parse_args()
  paths = args.rlogs
  if args.all:
    from replay_undelivered_alert import find_rlogs
    paths = find_rlogs()
  results = []
  with Pool(args.workers) as pool:
    for n, r in enumerate(pool.imap_unordered(scan, paths, chunksize=4), 1):
      results.append(r)
      if r["runs"] or r["faults"]:
        print(f"{r['path']} {r.get('commit', '?')} long={r.get('long')} faults={r['faults']}")
        for x in r["runs"]:
          print(f"  LKAS_FAULT t0={x['t0']:7.2f} dur={x['dur']:6.2f} v0={x['v0']:5.2f} prev={x['prev_byte6']} "
                f"echo={x['echo_before']} gap_before={x['gap_before']} rejected_before={x['rejected_before']} "
                f"camera_fault_dt={x['fault_dt']} censored={x['censored']}")
      if args.all and n % 200 == 0:
        print(f"PROGRESS {n}/{len(paths)}", file=sys.stderr, flush=True)
  if args.json:
    with open(args.json, "w") as f:
      json.dump(results, f)


if __name__ == "__main__":
  main()
