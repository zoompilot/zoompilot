#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Turn the corpus ceiling measurement into a STEER_MAX_LOOKUP.

The modal-|eff| rail detector in eps_ceiling_curve.py degrades above ~60 mph, where
torque demand is low: the mode lands on a cruising torque instead of a rail. Here the
population is conditioned on genuinely high demand (|req| >= DEMAND) so every bin is
scored on frames where the EPS was actually being pushed.
"""
import os
import numpy as np

DEMAND = 700          # only frames asking for real torque
SAT_MARGIN = 40
CACHE = os.environ.get('CACHE', '/tmp/eps_ceiling_corpus.npz')

z = np.load(CACHE)
mph = z['v'].astype(np.float64) / 10.0 * 2.237
req, eff = np.abs(z['r'].astype(np.int32)), np.abs(z['e'].astype(np.int32))
pushed = (req >= DEMAND) & ((req - eff) >= SAT_MARGIN)
print(f'{len(req):,} clean frames; {pushed.sum():,} with |req|>={DEMAND} and clipping\n')

print(f'{"mph":>7} {"nPushed":>9} {"maxEFF":>7} {"p999":>6} {"p99":>6} {"p95":>6} '
      f'{">620":>7}  ceiling')
print('-' * 72)
pts = []
for lo in range(2, 84, 2):
    m = (mph >= lo) & (mph < lo + 2) & pushed
    if m.sum() < 200:
        continue
    e = eff[m]
    ceil = int(np.percentile(e, 99.5))
    over = 100.0 * np.mean(e > 620)
    pts.append(((lo + 1) / 2.237, ceil, e.max(), m.sum()))
    print(f'{lo:>3}-{lo+2:<3} {m.sum():>9,} {e.max():>7} {np.percentile(e,99.9):>6.0f} '
          f'{np.percentile(e,99):>6.0f} {np.percentile(e,95):>6.0f} {over:>6.1f}%  {ceil}')

# global check: is 620 an absolute rail above 32 mph?
hs = (mph >= 32.5)
print(f'\nabove 32.5 mph: {hs.sum():,} clean frames, max|eff|={eff[hs].max()}, '
      f'frames >620: {np.sum(eff[hs] > 620)} ({100*np.mean(eff[hs]>620):.4f}%)')
ls = (mph < 18)
print(f'below 18 mph:   {ls.sum():,} clean frames, max|eff|={eff[ls].max()}, '
      f'frames >1148: {np.sum(eff[ls] > 1148)}')

# piecewise-linear candidates, scored against the measured ceiling points
V = np.array([p[0] for p in pts]); C = np.array([p[1] for p in pts])
keep = V <= 14.6 + 6      # fit over the rolloff plus the flat top
cands = {
  'current ([0,14.2,14.5] -> [1200,1200,800])': ([0, 14.2, 14.5, 40], [1200, 1200, 800, 800]),
  '2-point  ([0,8.0,14.5] -> [1144,1144,620])': ([0, 8.0, 14.5, 40], [1144, 1144, 620, 620]),
  '3-point  (+11.2 -> 992)':                    ([0, 8.0, 11.2, 14.5, 40], [1144, 1144, 992, 620, 620]),
  '4-point  (+10.7/12.5)':                      ([0, 8.0, 10.7, 12.5, 14.5, 40], [1144, 1144, 1012, 830, 620, 620]),
}
print(f'\n{"candidate":<46} {"max err":>8} {"mean|err|":>10} {"over-command":>13}')
print('-' * 82)
for name, (bx, by) in cands.items():
    pred = np.interp(V[keep], bx, by)
    err = pred - C[keep]
    print(f'{name:<46} {np.abs(err).max():>8.0f} {np.abs(err).mean():>10.1f} '
          f'{err[err>0].sum()/max(len(err),1):>12.0f}')
print('\n(over-command = mean counts commanded above what the EPS delivers; '
      'positive is the blind spot we are trying to remove)')
