#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Evaluate an offline startup guard against cached EPS observations and raw fault captures.
This is an exposure analysis, not an EPS simulation: recorded readiness and fault bits cannot
predict their counterfactual values after changing a command. No vehicle code is modified.

Candidate: remember a block containing TRACK_STATE below 0.6 m/s; withhold nonzero requests
until BLOCK clears. The speed threshold selects the observed crawl cases, not a proven limit.
Use actual timestamps for the zero-request tail; EPS messages are not necessarily 100 Hz.
Only complete, uninterrupted episodes are scored. Duplicate paths in the old cache are removed.

Usage:
  python tools/mazda_long/replay_lkas_startup.py --fault RLOG --fault RLOG --json /tmp/startup.json
"""
import argparse
from collections import Counter
import glob
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def episodes(t, blocked, track, speed):
  """Complete BLOCK runs with observed crawl standby; split at logging gaps over 100 ms."""
  active = None
  entered = False
  for i in range(len(t)):
    if i == 0 or t[i] - t[i - 1] > .1:
      active = None
      entered = False
      continue
    if blocked[i] and not blocked[i - 1]:
      entered = True
    if entered and active is None and blocked[i] and track[i] and speed[i] <= .6:
      active = i
    if not blocked[i]:
      if active is not None:
        yield active, i
      active = None
      entered = False


def corpus(cache):
  counts = Counter()
  seen = set()
  samples = []
  matches = []
  for path in sorted(glob.glob(str(cache / '*.npz'))):
    with np.load(path) as z:
      meta = json.loads(str(z['meta']))
      source = str(Path(meta['path']).resolve())
      if source in seen or meta['car'] != 'MAZDA_CX5_2022':
        continue
      seen.add(source)
      counts['segments'] += 1
      if meta.get('error'):
        counts['segments_with_decode_error_excluded'] += 1
        continue
      t, req, eff = z['t'], z['req'], z['eff']
      # Successful counterexamples to a 300 ms zero-delivery crawl trigger. A segment with
      # no camera onset is only a candidate control until its raw EPS fault bits are checked.
      mask = (req != 0) & (eff == 0) & (z['blocked'] == 1) & (z['track'] == 1)
      edges = np.diff(np.r_[False, mask, False].astype(int))
      for a, b in zip(np.where(edges == 1)[0], np.where(edges == -1)[0]):
        if (a == 0 or b == len(t) or z['v'][a] > .6 or t[b - 1] - t[a] < .3 or
            len(z['err_t']) or np.max(np.diff(t[a:b])) > .1):
          continue
        matches.append({'path': source, 'start': float(t[a]), 'end': float(t[b - 1]),
                        'speed': float(z['v'][a]), 'max_request': int(np.max(np.abs(req[a:b])))})
      for a, b in episodes(t, z['blocked'], z['track'], z['v']):
        if t[b] - t[a] < .25:
          continue
        tail = max(a, int(np.searchsorted(t, t[b] - .25)))
        zero_tail = bool(np.all(req[tail:b] == 0))
        changed = req[a:b] != 0
        counts['complete_startup_blocks'] += 1
        counts['ready_after_zero_tail' if zero_tail else 'ready_after_nonzero_tail'] += 1
        counts['blocks_with_nonzero_request'] += int(np.any(changed))
        counts['nonzero_request_frames_withheld'] += int(np.sum(changed))
        counts['withheld_frames_with_effective_torque'] += int(np.sum(changed & (eff[a:b] != 0)))
        if eff[a] == 0:
          counts['blocks_starting_with_zero_effective'] += 1
          counts['zero_entry_blocks_later_delivering'] += int(np.any(eff[a:b] != 0))
          counts['zero_entry_blocks_later_delivering_over_50'] += int(np.max(np.abs(eff[a:b])) > 50)
        samples.append({'path': source, 'start': float(t[a]), 'ready': float(t[b]),
                        'ready_speed': float(z['v'][b]), 'zero_tail': zero_tail,
                        'max_request': int(np.max(np.abs(req[a:b]))),
                        'max_effective': int(np.max(np.abs(eff[a:b])))})
  counts['candidate_zero_delivery_controls'] = len(matches)
  return {'counts': dict(counts), 'episodes': samples, 'candidate_controls': matches}


def fault_capture(path):
  from openpilot.tools.lib.logreader import LogReader
  from tools.mazda_long.lkas_fault_scan import scan

  rows = []
  t0 = None
  v = driver = 0.
  for m in LogReader(path):
    if m.which() == 'carState':
      v, driver = m.carState.vEgoRaw, m.carState.steeringTorque
    if m.which() != 'can':
      continue
    t = m.logMonoTime * 1e-9
    if t0 is None:
      t0 = t
    for c in m.can:
      if c.src == 0 and c.address == 0x241:
        d = c.dat
        rows.append((t - t0, d[6], ((d[0] & 15) * 256 + d[1]) - 2048,
                     d[4] * 16 + (d[5] >> 4) - 2048, v, driver))
  onsets = []
  for i in range(1, len(rows)):
    if not rows[i][1] & 32 or rows[i - 1][1] & 32:
      continue
    # Trace backward through the immediately preceding TRACK_STATE standby.
    a = i - 1
    while a > 0 and rows[a - 1][1] == 0x14 and rows[a][0] - rows[a - 1][0] <= .1:
      a -= 1
    nonzero = [r for r in rows[a:i] if r[2] != 0]
    onsets.append({'time': rows[i][0], 'speed': rows[i][4], 'standby_start': rows[a][0],
                   'standby_start_censored': a == 0,
                   'first_nonzero': nonzero[0][0] if nonzero else None,
                   'nonzero_to_fault': rows[i][0] - nonzero[0][0] if nonzero else None,
                   'max_request_before': max((abs(r[2]) for r in nonzero), default=0),
                   'max_effective_before': max((abs(r[3]) for r in nonzero), default=0),
                   'opposing_frames': sum(r[2] * r[5] < 0 for r in nonzero),
                   'nonzero_frames': len(nonzero),
                   'candidate_covers_exposure': bool(nonzero and rows[a][4] <= .6)})
  return {'scan': scan(path), 'onsets': onsets}


def verify_control(candidate):
  """Check fault absence, driver opposition, and effective torque in raw CAN, not the cache."""
  from openpilot.tools.lib.logreader import LogReader

  result = dict(candidate)
  t0 = None
  driver = 0.
  rows = []
  fault = False
  for m in LogReader(candidate['path']):
    if m.which() == 'carParams':
      result['flags'] = m.carParams.flags
    if m.which() == 'carState':
      driver = m.carState.steeringTorque
    if m.which() != 'can':
      continue
    t = m.logMonoTime * 1e-9
    if t0 is None:
      t0 = t
    for c in m.can:
      if c.src != 0 or c.address != 0x241:
        continue
      d = c.dat
      fault |= bool(d[6] & 32)
      if candidate['start'] - .001 <= t - t0 <= candidate['end'] + .001:
        rows.append(((d[0] & 15) * 256 + d[1] - 2048, driver, d[6],
                     d[4] * 16 + (d[5] >> 4) - 2048))
  if not rows:
    raise ValueError(f"Cached window has no raw EPS frames: {candidate}")
  result.update(eps_fault_anywhere=fault, frames=len(rows),
                opposing_frames=sum(r[0] * r[1] < 0 for r in rows),
                max_effective=max(abs(r[3]) for r in rows), byte6=sorted({r[2] for r in rows}))
  return result


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--cache', type=Path, default=HERE / '.undelivered_cache')
  ap.add_argument('--fault', action='append', default=[])
  ap.add_argument('--verify-controls', type=int, default=8,
                  help='raw-check controls nearest route 1e8\'s 95-count maximum request')
  ap.add_argument('--json', type=Path, required=True)
  args = ap.parse_args()
  result = {'limitation': 'Recorded-state exposure analysis; not proof that a guard prevents faults or preserves readiness.',
            'corpus': corpus(args.cache), 'faults': [fault_capture(p) for p in args.fault]}
  controls = sorted(result['corpus']['candidate_controls'], key=lambda c: abs(c['max_request'] - 95))
  result['verified_controls'] = [verify_control(c) for c in controls[:args.verify_controls]]
  args.json.write_text(json.dumps(result, indent=2) + '\n')
  print(json.dumps(result['corpus']['counts'], indent=2))
  for f in result['faults']:
    print(f['scan']['path'], json.dumps(f['onsets']))


if __name__ == '__main__':
  main()
