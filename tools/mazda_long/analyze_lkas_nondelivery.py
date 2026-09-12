#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

LKAS non-delivery analysis: how well LKAS_EFFECTIVE == 0 separates a total LKAS_BLOCK
from normal operation, and how much request the non-delivery latch would have withheld.

The camera latches CAM_LKAS.ERR_BIT_1 ("LKAS Fault: Restart the Car") when it watches a
request go nowhere for long enough; the exact predicate is still unknown, but the two
captured faults rank 1 and 2 of 92 segments by non-delivery budget. A blocked EPS often
still delivers a third to a half of the request, which is why the latch keys off delivery
rather than off LKAS_BLOCK -- gating on the bit would throw away real steering.

What actually produced those two runs was the EPS being starved of 0x243 outright, once by
the panda rejecting every frame while our command rode the driver-torque ceiling and once by
lateral never arming panda-side. The latch bounds what the camera sees; it does not address
the starvation. Rejected frames appear as src == 192 in the rlog can topic, and
carState.pandaStates controlsAllowedLateral separates the two causes.

Usage:
  analyze_lkas_nondelivery.py <dir-under-test_data> [more dirs ...]
  analyze_lkas_nondelivery.py --device <route-prefix>     # device_data/ instead
"""

import collections
import glob
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, REPO)

from openpilot.tools.lib.logreader import LogReader

STEER_RATE_ADDR = 577
CAM_LKAS_ADDR = 579

# mirrors CarControllerParams for the 2022 EPS
REQ_MIN = 200
LATCH_FRAMES = 20


def decode_steer_rate(dat):
  """LKAS_REQUEST 3|12@0+, LKAS_EFFECTIVE 39|12@0+, LKAS_BLOCK 50|1@1+ (mazda_2017.dbc)."""
  req = (((dat[0] & 0xf) << 8) | dat[1]) - 2048
  eff = ((dat[4] << 4) | (dat[5] >> 4)) - 2048
  blocked = (dat[6] >> 2) & 1
  return req, eff, blocked


def scan(files):
  runs = collections.defaultdict(list)
  delivery = collections.defaultdict(lambda: [0, 0])
  # what the latch would have withheld, and what it would have cost
  withheld = 0
  withheld_effective = 0
  cam_errors = []

  for path in files:
    v_ego, pressed, lat_active = 0.0, False, False
    run = {0: 0, 1: 0}
    latched = False
    latch_frames = 0
    cam_err_last = None

    for msg in LogReader(path):
      which = msg.which()
      if which == 'carState':
        v_ego = msg.carState.vEgo
        pressed = msg.carState.steeringPressed
      elif which == 'carControl':
        lat_active = msg.carControl.latActive
      elif which != 'can':
        continue
      else:
        for can in msg.can:
          if can.address == CAM_LKAS_ADDR and can.src == 2:
            err = can.dat[2] & 1
            if cam_err_last == 0 and err == 1:
              cam_errors.append((path, msg.logMonoTime))
            cam_err_last = err
            continue
          if can.address != STEER_RATE_ADDR or can.src != 0:
            continue

          req, eff, blocked = decode_steer_rate(can.dat)
          real_request = lat_active and not pressed and abs(req) > REQ_MIN

          if real_request:
            counts = delivery[(blocked, min(int(v_ego // 2) * 2, 10))]
            counts[0] += 1
            if eff == 0:
              counts[1] += 1

          # zero-delivery run lengths, split by whether the EPS said it was blocking
          if real_request and eff == 0:
            run[blocked] += 1
          else:
            for b in (0, 1):
              if run[b]:
                runs[b].append(run[b])
                run[b] = 0

          # replay the latch against what actually went out
          if not blocked:
            latched, latch_frames = False, 0
          elif not latched:
            if real_request and eff == 0:
              latch_frames += 1
              latched = latch_frames >= LATCH_FRAMES
            else:
              latch_frames = 0
          if latched:
            withheld += 1
            withheld_effective += abs(eff)

    for b in (0, 1):
      if run[b]:
        runs[b].append(run[b])

  return runs, delivery, withheld, withheld_effective, cam_errors


def main():
  args = sys.argv[1:]
  if not args:
    print(__doc__)
    return 1

  root = 'tools/mazda_long/test_data'
  if args[0] == '--device':
    root, args = 'tools/mazda_long/device_data', args[1:]

  files = []
  for arg in args:
    files += sorted(glob.glob(f'{root}/{arg}*/**/rlog*', recursive=True))
  if not files:
    print(f'no rlogs under {root} matching {args}')
    return 1
  print(f'{len(files)} rlogs')

  runs, delivery, withheld, withheld_effective, cam_errors = scan(files)

  print(f'\n=== zero-delivery run lengths (frames @100Hz), latActive & !pressed & |req|>{REQ_MIN} ===')
  for blocked in (0, 1):
    lengths = sorted(runs[blocked], reverse=True)
    print(f'LKAS_BLOCK={blocked}: {len(lengths)} runs; longest: {lengths[:10]}')
    for n in (5, 10, LATCH_FRAMES, 30, 50):
      print(f'    >= {n:3d} frames: {sum(1 for x in lengths if x >= n)}')

  print('\n=== frac(LKAS_EFFECTIVE == 0) by block / speed ===')
  for key in sorted(delivery):
    total, zero = delivery[key]
    if total < 200:
      continue
    print(f'  blocked={key[0]} vEgo>={key[1]:2d}  n={total:7d}  eff==0: {100 * zero / total:5.1f}%')

  print(f'\n=== latch replay (>= {LATCH_FRAMES} frames) ===')
  print(f'  frames withheld:            {withheld} ({withheld * 0.01:.1f} s)')
  if withheld:
    print(f'  EPS torque given up:        {withheld_effective} counts total ({withheld_effective / withheld:.1f}/frame)')
  else:
    print('  nothing withheld')
  print(f'  CAM_LKAS ERR_BIT_1 latches: {len(cam_errors)}')
  for path, mono in cam_errors:
    print(f'    {os.path.basename(os.path.dirname(path))} @ {mono}')
  return 0


if __name__ == '__main__':
  sys.exit(main())
