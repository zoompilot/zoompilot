#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The model's lead forecast in the long MPC (sunnypilot/selfdrive/controls/lib/lead_forecast) against
upstream's lead extrapolation, over every alpha-long frame in the lead_event_index cache.

Two MPCs run side by side, each seeded every frame with the state the car's planner started from
(the logged plan's first point), so until the first frame they disagree both see exactly what the
car saw: the first-crossing times are exact, later frames show what each would ask from where the
car actually was, not where it would have been. Drives today's upstream MPC does not reproduce
(builds from before cruise left the MPC) are left out.

Reports, per event type from lead_event_index: when the MPC candidate first crosses -0.3 and -1.0
m/s^2 against the lead's decel onset, its minimum, and FCW frames everywhere.

Usage: lead_forecast_replay.py [--limit N] [--out report.md]
"""
import argparse
import os
import sys
from multiprocessing import Pool
from types import SimpleNamespace as NS

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import N, LongitudinalMpc, T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.lead_forecast.forecast import LeadForecast

import lead_event_index as L

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ACTION_T = 0.36 + DT_MDL  # Mazda longitudinalActuatorDelay + DT_MDL, as the planner
NO_LEAD = NS(present=False)
SQP_ITERATIONS = 1


class AlwaysOn:
  def get_bool(self, key):
    return True


def _lead(d, i, suffix):
  if d['leadPresent' if not suffix else 'lead2Present'][i] < 0.5:
    return NO_LEAD
  return NS(present=True, dRel=float(d['dRel' + suffix][i]), vLead=float(d['vLead' + suffix][i]),
            aLeadK=float(d['aLeadK' + suffix][i]), aLeadTau=float(d['aLeadTau' + suffix][i]),
            modelProb=float(d['modelProb' + suffix][i]), radar=bool(d['leadRadar' if not suffix else 'lead2Radar'][i] > 0.5))


def radar_state(d, i):
  return NS(leadOne=_lead(d, i, ''), leadTwo=_lead(d, i, '2'))


def model_v2(d, i):
  return NS(leadsV3=[NS(x=d['lead_x'][i, k], v=d['lead_v'][i, k], prob=float(d['lead_prob'][i, k])) for k in range(2)])


class ConvergedSolver:
  """The acados solver with solve() iterated to convergence; everything else passes through."""
  def __init__(self, solver):
    self._solver = solver

  def __getattr__(self, name):
    return getattr(self._solver, name)

  def solve(self):
    status = 0
    for _ in range(SQP_ITERATIONS):
      status = self._solver.solve()
      if status != 0:
        break
    return status


class Arm:
  """One MPC seeded every frame from the car's planner: its start state (the logged plan's first
  point) and the previous plan the change cost pulls toward. Each frame is then a one-step
  counterfactual with no memory of the arm's own earlier answers."""
  def __init__(self, forecast: bool):
    self.mpc = LongitudinalMpc(dt=DT_MDL)
    # One real-time iteration a frame converges on the car, where each frame starts from the last
    # one's answer. Seeded from the log it does not, and with a distant lead the problem is flat
    # enough that two arms fed identical inputs stay apart. Iterate to convergence instead.
    self.mpc.solver = ConvergedSolver(self.mpc.solver)
    self.forecast = None
    if forecast:
      self.forecast = LeadForecast(params=AlwaysOn())
      self.forecast.install(self.mpc)

  def step(self, d, i, rs, md, personality):
    active = d['longActive'][i] > 0.5
    self.mpc.set_weights(active and d['vEgo'][i] > 0.1, personality=personality)
    v0, a0 = float(d['planSpeeds'][i, 0]), float(d['planAccels'][i, 0])
    if not (np.isfinite(v0) and np.isfinite(a0)):
      v0, a0 = float(d['vEgo'][i]), float(d['aEgo'][i])
    self.mpc.set_cur_state(v0, a0)
    # the previous plan the change cost pulls toward, also the car's (the cost is zero past 2 s,
    # inside the logged 2.5 s)
    if i > 0 and np.isfinite(d['planAccels'][i - 1]).all():
      self.mpc.a_prev = np.interp(T_IDXS_MPC + DT_MDL, CONTROL_N_T_IDX, d['planAccels'][i - 1])
    if self.forecast is not None:
      self.forecast.update({'radarState': rs, 'modelV2': md})
    self.mpc.update(rs, personality=personality)
    v_traj = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    a_traj = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    a_mpc = float(get_accel_from_plan(v_traj, a_traj, CONTROL_N_T_IDX, action_t=ACTION_T))
    fcw = self.mpc.crash_cnt > 2 and d['vEgo'][i] > 0.1
    return a_mpc, fcw, a_traj


def personality_at(d, i):
  p = int(d['personality'][i])
  return p if p < 3 else 1


def plan_fit(d):
  """How well today's upstream MPC reproduces the logged plan, with the logged personality. Builds
  from before cruise left the MPC (it was a third obstacle until mid-2026) do not reproduce."""
  m = (d['longActive'] > 0.5) & (d['leadPresent'] > 0.5) & (d['planSource'] == 1)
  if m.sum() < 40:
    return float('nan')
  arm = Arm(False)
  a = np.array([arm.step(d, i, radar_state(d, i), None, personality_at(d, i))[2] for i in range(len(d['t']))])
  return float(np.median(np.abs(a[m] - d['planAccels'][m])))


ITERATE_FIELDS = (('x', N + 1), ('u', N), ('pi', N), ('lam', N + 1))


def get_iterate(solver):
  return [(f, k, solver.get(k, f)) for f, n in ITERATE_FIELDS for k in range(n)]


def set_iterate(solver, iterate):
  for f, k, value in iterate:
    solver.set(k, f, value)


def replay_drive(d):
  """Both arms start every frame from the same solver iterate, the upstream arm's last one (which
  tracks the car's plan). The MPC has more than one local solution with a distant lead, and an
  arm whose warm start drifted onto another keeps it while being re-seeded from the log: on a
  radar frame (forecast off, identical inputs) one sat 0.9 m/s^2 above the car's plan, where a
  cold solve agrees with it."""
  n = len(d['t'])
  up, fc = Arm(False), Arm(True)
  a = np.zeros((2, n), np.float32)
  fcw = np.zeros((2, n), bool)
  crash = np.zeros((2, n), bool)  # crash_cnt > 0: what DEC reads to go blended
  for i in range(n):
    rs, md = radar_state(d, i), model_v2(d, i)
    iterate = get_iterate(up.mpc.solver)
    a[0, i], fcw[0, i], _ = up.step(d, i, rs, md, personality_at(d, i))
    crash[0, i] = up.mpc.crash_cnt > 0
    set_iterate(fc.mpc.solver, iterate)
    a[1, i], fcw[1, i], _ = fc.step(d, i, rs, md, personality_at(d, i))
    crash[1, i] = fc.mpc.crash_cnt > 0
  return a, fcw, crash


def first_below(x, thr, lo, hi):
  idx = np.flatnonzero(x[lo:hi] < thr)
  return None if len(idx) == 0 else lo + int(idx[0])


FIT_MAX = 0.1  # m/s^2: drives upstream's MPC does not reproduce this well are left out


def analyse(d):
  if not plan_fit(d) < FIT_MAX:
    return {'skipped_s': float(((d['longActive'] > 0.5) & (d['leadPresent'] > 0.5)).sum() * DT_MDL)}
  a, fcw, crash = replay_drive(d)
  n = len(d['t'])
  active = d['longActive'] > 0.5
  # radar leads keep upstream's extrapolation; only camera leads (alpha long with the radar
  # silenced) can differ
  lead = active & (d['leadPresent'] > 0.5) & (d['leadRadar'] < 0.5)
  res = {'slow': [], 'tap': [], 'cut': [], 'lead_s': float(lead.sum() * DT_MDL),
         'fcw_frames': [int((fcw[k] & active).sum()) for k in range(2)],
         'fcw_onsets': [int(np.sum(np.diff(np.r_[0, (fcw[k] & active).astype(np.int8)]) == 1)) for k in range(2)],
         'crash_frames': [int((crash[k] & lead).sum()) for k in range(2)],
         'crash_onsets': [int(np.sum(np.diff(np.r_[0, (crash[k] & lead).astype(np.int8)]) == 1)) for k in range(2)],
         # where the two candidates differ while following a lead
         'diff': (a[1] - a[0])[lead].astype(np.float32), 'exp': d['experimental'][lead] > 0.5}
  for ev in L.slowing_leads(d):
    if ev['radar']:
      continue
    i0 = ev['i']
    pre, end = max(i0 - int(3.0 / DT_MDL), 0), min(ev['iMin'] + int(2.0 / DT_MDL), n)
    row = dict(ev)
    for k, name in enumerate(('old', 'new')):
      for thr in (-0.3, -1.0):
        j = first_below(a[k], thr, pre, end)
        row[f'{name}{thr}'] = None if j is None else float(d['t'][j] - d['t'][i0])
      row[f'{name}Min'] = float(np.min(a[k, i0:end]))
      row[f'{name}Fcw'] = bool(np.any(fcw[k, pre:end]))
    res['slow'].append(row)
  for key, fn in (('tap', L.brake_taps), ('cut', L.cut_ins)):
    for ev in fn(d):
      i0 = ev['i']
      if d['leadRadar'][i0] > 0.5:
        continue
      end = min(i0 + int(3.0 / DT_MDL), n)
      res[key].append(dict(ev, oldMin=float(np.min(a[0, i0:end])), newMin=float(np.min(a[1, i0:end])),
                           oldMean=float(np.mean(a[0, i0:end])), newMean=float(np.mean(a[1, i0:end]))))
  return res


def run_key_group(keys):
  out = []
  for d in L.load_routes(keys):
    if len(d['t']) > 20 and (d['longActive'] > 0.5).any():
      out.append(analyse(d))
  return out


def pct(x, q):
  x = [v for v in x if v is not None]
  return float(np.percentile(x, q)) if x else float('nan')


def report(all_results):
  skipped_h = sum(r.get('skipped_s', 0.) for r in all_results) / 3600
  results = [r for r in all_results if 'skipped_s' not in r]
  slow = [r for res in results for r in res['slow']]
  taps = [r for res in results for r in res['tap']]
  cuts = [r for res in results for r in res['cut']]
  lines = ['# Lead forecast replay', '']
  lead_h = sum(res['lead_s'] for res in results) / 3600
  fcw_f = [sum(res['fcw_frames'][k] for res in results) for k in range(2)]
  fcw_o = [sum(res['fcw_onsets'][k] for res in results) for k in range(2)]
  lines += [f'Drives left out (upstream MPC does not reproduce the logged plan to {FIT_MAX} m/s^2): {skipped_h:.2f} h with a lead.', '']
  cr_f = [sum(res['crash_frames'][k] for res in results) for k in range(2)]
  cr_o = [sum(res['crash_onsets'][k] for res in results) for k in range(2)]
  lines += [f'DEC crash input (crash_cnt > 0) behind a camera lead: upstream {cr_o[0]} onsets / {cr_f[0]} frames, ' +
            f'forecast {cr_o[1]} onsets / {cr_f[1]} frames.', '']
  lines += [f'Following a camera lead: {lead_h:.2f} h. FCW (crash_cnt > 2): upstream {fcw_o[0]} onsets / {fcw_f[0]} frames, ' +
            f'forecast {fcw_o[1]} onsets / {fcw_f[1]} frames.', '']
  diff = np.concatenate([res['diff'] for res in results]) if results else np.zeros(0)
  exp = np.concatenate([res['exp'] for res in results]) if results else np.zeros(0, bool)
  lines += ['MPC candidate, forecast minus upstream, every frame following a camera lead (m/s^2):', '',
            '| | p1 | p10 | p50 | p90 | p99 | abs > 0.3 |', '|---|---|---|---|---|---|---|']
  for name, m in (('all', np.ones(len(diff), bool)), ('exp', exp), ('chill', ~exp)):
    x = diff[m]
    if len(x):
      lines.append(f'| {name} | ' + ' | '.join(f'{np.percentile(x, q):+.2f}' for q in (1, 10, 50, 90, 99)) +
                   f' | {np.mean(np.abs(x) > 0.3) * 100:.1f}% |')
  lines += ['', f'## Slowing lead ({len(slow)} events)', '',
            'Times from the lead\'s decel onset (s); never = not crossed by 2 s after the lead\'s minimum speed.', '',
            '| | -0.3 p50 | p90 | never | -1.0 p50 | p90 | never | min p50 | min p10 | FCW |', '|---|---|---|---|---|---|---|---|---|---|']
  for name in ('old', 'new'):
    t3 = [r[f'{name}-0.3'] for r in slow]
    t10 = [r[f'{name}-1.0'] for r in slow]
    mins = [r[f'{name}Min'] for r in slow]
    lines.append(f'| {"upstream" if name == "old" else "forecast"} | {pct(t3, 50):+.2f} | {pct(t3, 90):+.2f} | ' +
                 f'{sum(x is None for x in t3)} | {pct(t10, 50):+.2f} | {pct(t10, 90):+.2f} | {sum(x is None for x in t10)} | ' +
                 f'{pct(mins, 50):+.2f} | {pct(mins, 10):+.2f} | {sum(r[f"{name}Fcw"] for r in slow)} |')
  paired = [(r['old-0.3'], r['new-0.3']) for r in slow if r['old-0.3'] is not None and r['new-0.3'] is not None]
  if paired:
    dt = np.array([n - o for o, n in paired])
    lines += ['', f'Paired -0.3 crossing, forecast minus upstream (n={len(dt)}): p10 {np.percentile(dt, 10):+.2f}, ' +
              f'p50 {np.percentile(dt, 50):+.2f}, p90 {np.percentile(dt, 90):+.2f} s; earlier in {np.mean(dt < -0.05) * 100:.0f}%, ' +
              f'later in {np.mean(dt > 0.05) * 100:.0f}%.']
  paired = [(r['old-1.0'], r['new-1.0']) for r in slow if r['old-1.0'] is not None and r['new-1.0'] is not None]
  if paired:
    dt = np.array([n - o for o, n in paired])
    lines += [f'Paired -1.0 crossing (n={len(dt)}): p10 {np.percentile(dt, 10):+.2f}, p50 {np.percentile(dt, 50):+.2f}, ' +
              f'p90 {np.percentile(dt, 90):+.2f} s.']
  dmin = np.array([r['newMin'] - r['oldMin'] for r in slow])
  if len(dmin):
    lines += [f'Peak decel, forecast minus upstream: p10 {np.percentile(dmin, 10):+.2f}, p50 {np.percentile(dmin, 50):+.2f}, ' +
              f'p90 {np.percentile(dmin, 90):+.2f} m/s^2 (negative = forecast brakes harder at its peak).']
  for name, rows in (('Lead brake tap', taps), ('Cut-in, lead then accelerates', cuts)):
    lines += ['', f'## {name} ({len(rows)} events), first 3 s', '']
    if rows:
      for f in ('Min', 'Mean'):
        o = np.array([r['old' + f] for r in rows])
        nw = np.array([r['new' + f] for r in rows])
        lines.append(f'- {f.lower()} candidate: upstream p50 {np.median(o):+.2f}, forecast p50 {np.median(nw):+.2f}, ' +
                     f'paired diff p50 {np.median(nw - o):+.2f} (p10 {np.percentile(nw - o, 10):+.2f}, p90 {np.percentile(nw - o, 90):+.2f})')
  worst = sorted(slow, key=lambda r: r['newMin'] - r['oldMin'], reverse=True)[:8]
  lines += ['', '## Slowing leads where the forecast brakes least relative to upstream', '',
            '| route | seg | t | vEgo | lead | gap | upstream min | forecast min | -0.3 upstream / forecast |', '|---|---|---|---|---|---|---|---|---|']
  for r in worst:
    lines.append(f"| {r['route']} | {r['seg']} | {r['t_seg']:.1f} | {r['vEgo']:.1f} | {r['vLead0']:.1f}->{r['vLead1']:.1f} | " +
                 f"{r['dRel0']:.0f}->{r['minGap']:.0f} | {r['oldMin']:+.2f} | {r['newMin']:+.2f} | {r['old-0.3']} / {r['new-0.3']} |")
  return '\n'.join(lines) + '\n'


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--limit', type=int, default=0, help='only the first N cached segments')
  ap.add_argument('--out', default=None)
  args = ap.parse_args()
  keys = L.engaged_keys()
  if args.limit:
    keys = keys[:args.limit]
  # whole routes per worker so drives stitch across segments
  by_route = {}
  for k in keys:
    with np.load(os.path.join(L.CACHE_DIR, k + '.npz')) as z:
      by_route.setdefault(str(z['route']), []).append(k)
  with Pool() as pool:
    results = [r for rs in pool.imap_unordered(run_key_group, list(by_route.values())) for r in rs]
  text = report(results)
  if args.out:
    with open(args.out, 'w') as f:
      f.write(text)
  print(text)


if __name__ == '__main__':
  main()
