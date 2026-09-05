#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Real-world ICBM servo review over a drive's rlogs.

Segments the drive into servo episodes (contiguous increasing/decreasing state) and
measures what the ECU actually did with our emissions:

  - convergence: did the dash land within the deadband of the target, and how fast
  - hold integration: per hold episode, the dash's step pattern (5-grid snaps vs paced
    1 mph steps vs nothing) tells us how the ECU integrated the synthesized hold
  - efficiency: dash steps landed per command-second, direction reversals (churn)
  - restore behavior: gap between a limiter releasing and the first up-move (quiet time)

Run from repo root: python tools/mazda_long/icbm_sla/icbm_episode_review.py [route_glob]
"""
import glob
import sys
import warnings

warnings.filterwarnings("ignore")

from openpilot.tools.lib.logreader import LogReader
from openpilot.common.constants import CV

MS_MPH = CV.MS_TO_MPH
DEFAULT_GLOB = "tools/mazda_long/test_data/sla_drive_logs/0000000b--b039e84091--*/rlog.zst"


def main(route_glob):
  paths = sorted(glob.glob(route_glob), key=lambda p: int(p.split('--')[-1].split('/')[0]))
  rows = []  # (t, dash, state, btn, vT_mph, lps)
  t0 = None
  state = btn = lps = None
  vt = 0.
  for path in paths:
    for msg in LogReader(path):
      w = msg.which()
      if t0 is None:
        t0 = msg.logMonoTime
      t = (msg.logMonoTime - t0) / 1e9
      if w == 'carControlSP':
        i = msg.carControlSP.intelligentCruiseButtonManagement
        state, btn = str(i.state), str(i.sendButton)
      elif w == 'longitudinalPlanSP':
        vt = msg.longitudinalPlanSP.vTarget * MS_MPH
        lps = str(msg.longitudinalPlanSP.longitudinalPlanSource)
      elif w == 'carState' and state is not None:
        rows.append((t, round(msg.carState.cruiseState.speedCluster * MS_MPH), state, btn, vt, lps))

  # episodes
  episodes = []
  cur = None
  for t, dash, st, b, vt_, lps_ in rows:
    moving = st in ('increasing', 'decreasing')
    if moving and cur is None:
      cur = {'t0': t, 'dir': st, 'dash0': dash, 'targets': {round(vt_)}, 'lps': {lps_},
             'frames': [], 'hold_frames': 0, 'tap_frames': 0}
    if cur is not None:
      if moving:
        cur['frames'].append((t, dash, b))
        cur['targets'].add(round(vt_))
        cur['lps'].add(lps_)
        if 'Hold' in b:
          cur['hold_frames'] += 1
        elif b in ('increase', 'decrease'):
          cur['tap_frames'] += 1
      else:
        cur['t1'], cur['dash1'] = t, dash
        episodes.append(cur)
        cur = None
  if cur is not None:
    cur['t1'], cur['dash1'] = rows[-1][0], rows[-1][1]
    episodes.append(cur)

  print(f"{len(episodes)} servo episodes\n")
  hdr = f"{'t0':>7} {'dur_s':>6} {'dir':<4} {'dash':>9} {'tgt(final)':>10} {'resid':>6}"
  hdr += f" {'steps':>5} {'holds':>6} {'taps':>5} {'rate mph/s':>10} {'pattern'}"
  print(hdr)

  hold_patterns = []
  churn_pairs = 0
  last_ep = None
  for ep in episodes:
    dur = ep['t1'] - ep['t0']
    delta = ep['dash1'] - ep['dash0']
    tgt = sorted(ep['targets'])[-1] if ep['dir'] == 'increasing' else sorted(ep['targets'])[0]
    resid = ep['dash1'] - tgt
    # dash step timing within the episode
    steps = [(t, d) for (t, d, _), (pt, pd, _) in zip(ep['frames'][1:], ep['frames'], strict=False) if d != pd for t, d in [(t, d - pd)]]
    sizes = [d for _, d in steps]
    if ep['hold_frames'] > 5:
      if any(abs(s) >= 2 for s in sizes):
        pat = 'hold:SNAP(' + ','.join(str(s) for s in sizes) + ')'
      elif sizes:
        gaps = [round(b - a, 2) for (a, _), (b, _) in zip(steps, steps[1:], strict=False)]
        pat = f'hold:paced x{len(sizes)}' + (f' gaps~{sorted(gaps)[len(gaps) // 2]}s' if gaps else '')
      else:
        pat = 'hold:no-movement'
      hold_patterns.append(pat.split('(')[0].split(' ')[0])
    else:
      pat = f'taps x{len(sizes)}'
    rate = abs(delta) / dur if dur > 0.2 else 0.
    line = f"{ep['t0']:7.1f} {dur:6.2f} {ep['dir'][:3]:<4} {ep['dash0']:>4}->{ep['dash1']:<4} {tgt:>10} {resid:>6}"
    line += f" {len(sizes):>5} {ep['hold_frames']:>6} {ep['tap_frames']:>5} {rate:>10.1f} {pat}"
    print(line + ('   [' + '/'.join(sorted(ep['lps'])) + ']' if len(ep['lps']) > 1 or 'cruise' not in ep['lps'] else ''))

    if last_ep is not None and ep['t0'] - last_ep['t1'] < 2.0 and ep['dir'] != last_ep['dir']:
      churn_pairs += 1
    last_ep = ep

  n_conv = sum(1 for ep in episodes
               if abs(ep['dash1'] - (sorted(ep['targets'])[-1] if ep['dir'] == 'increasing' else sorted(ep['targets'])[0])) <= 2)
  from collections import Counter
  print(f"\nconverged within deadband(2): {n_conv}/{len(episodes)}")
  print(f"direction reversals within 2s of the previous episode: {churn_pairs}")
  print(f"hold integration outcomes: {dict(Counter(hold_patterns))}")


if __name__ == '__main__':
  main(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_GLOB)
