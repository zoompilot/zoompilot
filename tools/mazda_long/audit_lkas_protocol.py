#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Raw-CAN LKAS protocol audit. Compare crawl request onsets, including successful controls.
Checksum predictions use the existing bit-level rule; agreement with independent stock
transmissions tests that rule without assuming injected frames prove their own validity.
Bus-2 provenance starts only after a forwarding echo proves the harness is separated.
Output is observational evidence, not a counterfactual EPS simulation.

PYTHONPATH=. python tools/mazda_long/audit_lkas_protocol.py --all --json /tmp/protocol.json
"""
import argparse
from bisect import bisect_left
from collections import Counter
import hashlib
import json
from multiprocessing import Pool
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
CACHE = ROOT / 'tools/mazda_long/.protocol_cache'
VERSION = 1


def checksum(d):
  angle_hi = d[4] & 3
  angle_mid = (d[5] >> 4) | ((d[5] & 15) << 4)
  return (249 - (d[0] >> 4) - (d[0] & 15) - d[1] - (d[2] & 0x89)
          - ((d[3] >> 6) & 1) * 16 - (d[3] & 32) - angle_hi - angle_mid
          - (d[6] >> 6) * 4 - ((d[6] >> 4) & 1) + (15 if angle_hi == 1 else 0)) & 255


def stream_summary(rows):
  ts = [r[0] for r in rows]
  return {'frames': len(rows), 'checksum_bad': sum(checksum(d) != d[7] for _, d in rows),
          'counter_steps': dict(Counter(str(((b[1][0] >> 4) - (a[1][0] >> 4)) % 16)
                                        for a, b in zip(rows, rows[1:]))),
          'max_gap_ms': round(max((b - a for a, b in zip(ts, ts[1:])), default=0) * 1000, 3),
          'status_bytes': dict(Counter(d[2:7].hex() for _, d in rows))}


def scan(path):
  from openpilot.tools.lib.logreader import LogReader
  p = Path(path)
  stat = p.stat()
  key = hashlib.sha256(f'{VERSION}:{p.resolve()}:{stat.st_size}:{stat.st_mtime_ns}'.encode()).hexdigest()
  cache = CACHE / (key + '.json')
  if cache.exists():
    return json.loads(cache.read_text())
  meta = {'path': str(p), 'bytes': stat.st_size}
  eps, accepted, camera, rejected, hud, uds, ownership = [], [], [], [], [], [], []
  samples = Counter()
  residuals = {'camera': Counter(), 'accepted': Counter()}
  t0 = None
  v = driver = 0.
  separated = False
  lat = long_active = False
  panda_lat = None
  angle = 0.
  last_send = {}
  origins = Counter()
  try:
    for m in LogReader(str(p)):
      w = m.which()
      if t0 is None:
        t0 = m.logMonoTime * 1e-9
      t = m.logMonoTime * 1e-9 - t0
      if w == 'initData':
        meta.update(commit=m.initData.gitCommit, wall=m.initData.wallTimeNanos)
      elif w == 'carParams':
        cp = m.carParams
        meta.update(car=cp.carFingerprint, long=cp.openpilotLongitudinalControl, flags=cp.flags,
                    firmware=[{'ecu': str(f.ecu), 'version': bytes(f.fwVersion).hex()} for f in cp.carFw])
      elif w == 'carState':
        v, driver, angle = m.carState.vEgoRaw, m.carState.steeringTorque, m.carState.steeringAngleDeg
      elif w == 'carControl':
        lat, long_active = m.carControl.latActive, m.carControl.longActive
      elif w == 'pandaStates' and len(m.pandaStates):
        value = m.pandaStates[0].controlsAllowedLateral
        if value != panda_lat:
          ownership.append((t, value))
        panda_lat = value
      elif w == 'sendcan':
        for c in m.sendcan:
          if c.address == 0x243 and c.src == 0:
            last_send[bytes(c.dat)] = t
      elif w == 'can':
        for c in m.can:
          d = bytes(c.dat)
          if c.src == 130:
            separated = True
          if len(d) != 8:
            continue
          if c.address == 0x241 and c.src == 0:
            eps.append((t, d[6], ((d[0] & 15) * 256 + d[1]) - 2048,
                        d[4] * 16 + (d[5] >> 4) - 2048, v, driver, lat, long_active, angle))
          elif c.address == 0x243:
            kind = None
            if c.src == 128:
              accepted.append((t, d)); kind = 'accepted'
              origins['recent_sendcan_match' if t - last_send.get(d, -1000) < .1 else 'unattributed'] += 1
            elif c.src == 192:
              rejected.append(t)
            elif c.src == 2 and separated:
              camera.append((t, d)); kind = 'camera'
            if kind:
              samples[kind] += 1
              residuals[kind][str((d[7] - checksum(d)) % 256)] += 1
          elif c.address == 0x440 and c.src == 2 and separated:
            hud.append((t, d.hex(), (d[4] >> 4) & 7))
          elif c.address == 0x764 and c.src == 128:
            uds.append((t, d.hex()))
  except Exception as e:
    meta['error'] = repr(e)

  meta.update(duration=eps[-1][0] - eps[0][0] if eps else 0,
              samples=dict(samples), checksum_residuals={k: dict(v) for k, v in residuals.items()},
              accepted_origins=dict(origins), initial_eps_fault=bool(eps and eps[0][1] & 32))
  faults = [r[0] for i, r in enumerate(eps) if i and r[1] & 32 and not eps[i - 1][1] & 32]
  meta['eps_fault_onsets'] = faults
  at, ct, ht = [r[0] for r in accepted], [r[0] for r in camera], [r[0] for r in hud]
  exposures = []
  zero_since = None
  for i, row in enumerate(eps):
    t, state, req, eff, speed, torque, active, la, ang = row
    if i and t - eps[i - 1][0] > .1:
      zero_since = None
    if req == 0:
      if zero_since is None:
        zero_since = t
      continue
    zero_duration = t - zero_since if zero_since is not None else 0
    zero_since = None
    if zero_duration < .15 or state != 0x14 or speed > .6 or eff != 0:
      continue
    future = [r for r in eps[i:] if r[0] <= t + .6]
    if not future or future[-1][0] < t + .5:
      continue
    win_a = accepted[bisect_left(at, t - .5):bisect_left(at, t + .3)]
    win_c = camera[bisect_left(ct, t - .5):bisect_left(ct, t + .3)]
    prev_hud = hud[max(0, bisect_left(ht, t) - 1):bisect_left(ht, t + .3)]
    onset = next((f for f in faults if t <= f <= t + .6), None)
    exposures.append({'time': t, 'fault_delay': None if onset is None else onset - t,
                      'speed': speed, 'driver_torque': torque, 'angle': ang, 'lat_active': active,
                      'long_active': la, 'zero_duration': zero_duration,
                      'max_request_300ms': max(abs(r[2]) for r in future if r[0] < t + .3),
                      'max_effective_300ms': max(abs(r[3]) for r in future if r[0] < t + .3),
                      'opposing_frames_300ms': sum(r[2] * r[5] < 0 for r in future if r[0] < t + .3),
                      'accepted': stream_summary(win_a), 'camera': stream_summary(win_c),
                      'camera_hud': prev_hud,
                      'rejected': sum(t - .5 <= r < t + .3 for r in rejected),
                      'last_panda_lat_edge': next((e for e in reversed(ownership) if e[0] <= t), None),
                      'last_uds': next((e for e in reversed(uds) if e[0] <= t), None),
                      'last_session_entry': next((e for e in reversed(uds) if e[0] <= t and e[1].startswith('021002')), None)})
  meta['crawl_onsets'] = exposures
  CACHE.mkdir(exist_ok=True)
  cache.write_text(json.dumps(meta))
  return meta


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('paths', nargs='*')
  ap.add_argument('--all', action='store_true')
  ap.add_argument('--paths-json', type=Path, help='JSON array of selected raw-log paths')
  ap.add_argument('--workers', type=int, default=4)
  ap.add_argument('--json', type=Path, required=True)
  args = ap.parse_args()
  paths = args.paths
  if args.paths_json:
    paths += json.loads(args.paths_json.read_text())
  if args.all:
    from tools.mazda_long.replay_undelivered_alert import find_rlogs
    paths += find_rlogs()
  # Byte-identical duplicates must not inflate the outcome counts. Hash input files once.
  seen = set()
  unique = []
  for p in paths:
    digest = hashlib.sha256(Path(p).read_bytes()).hexdigest()
    if digest not in seen:
      seen.add(digest); unique.append(p)
  print(f'{len(unique)} byte-distinct logs from {len(paths)} paths', flush=True)
  results = []
  with Pool(args.workers) as pool:
    for n, result in enumerate(pool.imap_unordered(scan, unique), 1):
      results.append(result)
      if result['eps_fault_onsets']:
        print('FAULT', result['path'], result['eps_fault_onsets'], flush=True)
      if n % 100 == 0:
        print(f'{n}/{len(unique)} complete', flush=True)
  args.json.write_text(json.dumps(results, indent=2))
  print('saved', args.json, flush=True)


if __name__ == '__main__':
  main()
