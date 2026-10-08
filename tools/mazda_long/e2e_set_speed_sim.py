#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Closed-loop scorecard for experimental mode's set-speed boost (sunnypilot/selfdrive/controls/lib/e2e_set_speed).

Every active stretch of a logged segment is re-driven by a simulated car with each candidate
controller, and scored against the same sim with the boost off.

- Road features are synced by distance: at sim position s the controller sees the frame the log
  recorded at s (plan, curvature, target speed), shifted to the sim's speed. A stop is waited
  out in time, as the log waited.
- The model's acceleration is its logged value plus its measured pull back to its own pace. It
  does not shed speed it did not choose before a slowdown (measured on drives with the boost on),
  so speed a controller adds is carried into the next bend or stop unless the controller removes
  it.
- Where the logged planner drove another candidate (the MPC on a lead, cruise), the sim follows
  that choice corrected toward the logged speed. The rest of the planner as it actually drove.
- The car turns the planner's choice into motion through a 0.5 s lag, plus what the real car did
  beyond its lagged command (grade, controller error), replayed by distance.

Scores: tracking on road that hindsight shows was clear (no lead, curve, stop or slowdown in the
next 10 s); speed carried into curve apexes, stops and slowdowns; time gap to leads; jerk.

Usage:
  e2e_set_speed_sim.py cache <rlog> [<rlog> ...]      extract the segments once
  e2e_set_speed_sim.py score [--split dev|holdout|all] label=path.py:Class[?CONST=json&...] ...
"""
import argparse
import importlib.util
import json
import os
import sys
from multiprocessing import Pool
from types import SimpleNamespace as NS

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from numpy.lib.stride_tricks import sliding_window_view

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_planner import get_cruise_accel
from openpilot.selfdrive.modeld.constants import ModelConstants

CACHE = os.environ.get('E2E_SIM_CACHE', os.path.join(os.path.dirname(os.path.abspath(__file__)), '.e2e_set_speed_cache'))
T_IDXS = np.asarray(ModelConstants.T_IDXS)

K_PACE = 0.0196  # 1/s, the model's pull toward its own pace in cruise
K_TRACK = 0.8  # 1/s, toward the logged speed where another candidate drove
ACT_TAU = 0.5  # s
CLEAR_T = 10.0  # s of hindsight for clear road


# --- cache ---

def cache_segment(path):
  from e2e_set_speed_replay import frames, planner_inputs
  parts = os.path.normpath(path).split(os.sep)
  name = parts[-2] if parts[-2].count('--') == 2 else '_'.join(parts[-3:-1])
  out = os.path.join(CACHE, name + '.npz')
  if os.path.exists(out) or os.path.exists(out + '.skip'):
    return
  S = {}
  def put(k, v):
    S.setdefault(k, []).append(v)
  try:
    for sm, t in frames(path):
      md, CS, rs = sm['modelV2'], sm['carState'], sm['radarState']
      if len(md.velocity.x) != ModelConstants.IDX_N or len(md.position.x) != ModelConstants.IDX_N:
        continue
      kw = planner_inputs(sm)
      put('t', t)
      put('v_ego', CS.vEgo)
      put('a_ego', CS.aEgo)
      put('gas', CS.gasPressed)
      put('brake', CS.brakePressed)
      for k in ('v_cruise', 'is_e2e', 'reset_state', 'dec_active', 'allow_throttle', 'fcw', 'accel_coast'):
        put(k, kw[k])
      put('long_active', sm['carControl'].longActive)
      put('force_decel', sm['controlsState'].forceDecel)
      put('a_model', md.action.desiredAcceleration)
      put('curvature', md.action.desiredCurvature)
      put('should_stop', md.action.shouldStop)
      put('hard_brake', md.meta.hardBrakePredicted)
      put('lane_change', md.meta.laneChangeState.raw != 0)
      put('a_target_logged', sm['longitudinalPlan'].aTarget)
      put('source_e2e', sm['longitudinalPlan'].longitudinalPlanSource == 'e2e')
      for n, ld in enumerate((rs.leadOne, rs.leadTwo)):
        put(f'lead{n}_present', ld.present)
        put(f'lead{n}_d', ld.dRel)
        put(f'lead{n}_v', ld.vLead)
        put(f'lead{n}_a', ld.aLeadK)
      put('vel', np.asarray(md.velocity.x, dtype=np.float32))
      put('yaw', np.asarray(md.orientationRate.z, dtype=np.float32))
      put('pos', np.asarray(md.position.x, dtype=np.float32))
  except Exception as e:
    print(f"{path}: {e!r}", file=sys.stderr)
    return
  if not S or (np.asarray(S['is_e2e']) & np.asarray(S['long_active'])).sum() < 200:
    open(out + '.skip', 'w').close()
    return
  np.savez_compressed(out, **{k: np.asarray(v) for k, v in S.items()})


# --- sim ---

def load(spec):
  """'path.py:Class', optionally '?CONST=json&...' to override module constants."""
  path, _, rest = spec.partition(':')
  cls, _, overrides = rest.partition('?')
  s = importlib.util.spec_from_file_location(f"cand_{abs(hash(spec))}", path)
  m = importlib.util.module_from_spec(s)
  s.loader.exec_module(m)
  for kv in filter(None, overrides.split('&')):
    k, v = kv.split('=', 1)
    setattr(m, k, json.loads(v))
  return getattr(m, cls)


def make_sm(D, j, v_sim, lead_shift):
  """What the controller reads, from log frame j shifted to the sim's speed."""
  dv = v_sim - D['v_ego'][j]
  vel = D['vel'][j].astype(float) + dv
  curvature = D['yaw'][j].astype(float) / np.maximum(D['vel'][j].astype(float), 1.)
  leads = [NS(present=bool(D[f'lead{n}_present'][j]), dRel=float(D[f'lead{n}_d'][j]) + lead_shift,
              vLead=float(D[f'lead{n}_v'][j]), aLeadK=float(D[f'lead{n}_a'][j])) for n in (0, 1)]
  md = NS(velocity=NS(x=vel), orientationRate=NS(z=curvature * np.maximum(vel, 0.)),
          position=NS(x=D['pos'][j].astype(float) + dv * T_IDXS),
          action=NS(desiredCurvature=float(D['curvature'][j]), shouldStop=bool(D['should_stop'][j])),
          meta=NS(hardBrakePredicted=bool(D['hard_brake'][j]), laneChangeState=int(D['lane_change'][j])))
  return {'modelV2': md, 'carState': NS(vEgo=v_sim, gasPressed=False, brakePressed=False),
          'radarState': NS(leadOne=leads[0], leadTwo=leads[1]), 'controlsState': NS(forceDecel=bool(D['force_decel'][j]))}


def stretches(D):
  act = D['is_e2e'] & D['long_active'] & ~D['reset_state'] & ~D['dec_active'] & ~D['gas'] & ~D['brake']
  act &= np.r_[True, np.diff(D['t']) < 2 * DT_MDL]
  idx = np.flatnonzero(np.diff(np.r_[0, act.astype(int), 0]))
  return [(a, b) for a, b in zip(idx[::2], idx[1::2], strict=True) if b - a > 40]


def drive(D, a, b, ctl):
  """Re-drive frames [a, b) with ctl (None = boost off). Rows: (s, v, out, boost, log frame, v_cruise,
  lead present, lead gap); also the log's positions."""
  s_log = np.r_[0., np.cumsum(D['v_ego'][a:b - 1] * DT_MDL)]
  aT = D['a_target_logged'][a:b].astype(float)
  lag = np.empty_like(aT)
  lag[0] = D['a_ego'][a]
  for n in range(1, len(aT)):
    lag[n] = lag[n - 1] + DT_MDL / ACT_TAU * (aT[n - 1] - lag[n - 1])
  resid = np.r_[np.diff(D['v_ego'][a:b].astype(float)) / DT_MDL, 0.] - lag

  s, v, a_act = 0., float(D['v_ego'][a]), float(lag[0])
  a_cruise = a_act
  rows, jj, last = [], 0, b - a - 1
  for _ in range(b - a):
    # one frame on while the log sat still here, otherwise where the log was at this distance
    lo = min(int(np.searchsorted(s_log, s - 0.05, 'left')), last)
    hi = min(int(np.searchsorted(s_log, s + 0.05, 'right')) - 1, last)
    jj = int(np.clip(jj + 1, lo, max(hi, lo)))
    if s > s_log[-1] + 1. or jj >= last:
      break
    j = a + jj
    a_model = float(D['a_model'][j]) + K_PACE * (float(D['v_ego'][j]) - v)
    sm = make_sm(D, j, v, s_log[jj] - s)
    e2e = a_model
    if ctl is not None:
      e2e = ctl.update(sm, a_model, float(D['v_cruise'][j]), True, False, False, bool(D['allow_throttle'][j]),
                       bool(D['fcw'][j]), float(D['accel_coast'][j]))
    # upstream's e2e-mode cruise candidate reads nothing from CP
    a_cruise = get_cruise_accel(True, float(D['v_cruise'][j]), v, a_cruise, 0., None, DT_MDL,
                                float(D['accel_coast'][j]), bool(D['allow_throttle'][j]))
    out = min(e2e, a_cruise)
    if not D['source_e2e'][j]:
      out = min(out, float(D['a_target_logged'][j]) + K_TRACK * (float(D['v_ego'][j]) - v))
    if ctl is not None and hasattr(ctl, 'delivered'):
      ctl.delivered(out, e2e)
    a_act += DT_MDL / ACT_TAU * (out - a_act)
    lead = sm['radarState'].leadOne
    rows.append((s, v, out, e2e - a_model, jj, float(D['v_cruise'][j]), lead.present, lead.dRel))
    v = max(0., v + (a_act + resid[jj]) * DT_MDL)
    s += v * DT_MDL
  return np.asarray(rows), s_log


def events(D, a, b, s_log):
  """Log positions of curve apexes, stops and the hardest braking into each slowdown."""
  v = D['v_ego'][a:b]
  kap = np.abs(D['curvature'][a:b])
  lat = kap * v ** 2
  apex, stops, slows = [], [], []
  for k in range(5, len(v) - 5):
    if lat[k] > 1.0 and kap[k] == kap[k - 5:k + 6].max() and v[k] > 8. and (not apex or s_log[k] - s_log[apex[-1]] > 50.):
      apex.append(k)
  stops = [k for k in range(1, len(v)) if v[k] < 0.3 <= v[k - 1] and v[:k].max() > 5.]
  for k in range(10, len(v) - 10):
    if v[k] == v[k - 10:k + 11].min() and v[max(0, k - 300):k].max() - v[k] > 3. and v[k] > 0.3:
      if not slows or s_log[k] - s_log[slows[-1]] > 30.:
        slows.append(k)
  # the bottom of a slowdown is already the recovery: measure where it brakes hardest on the way in
  dv = np.r_[0., np.diff(v)]
  slows = [max(0, k - 200) + int(np.argmin(dv[max(0, k - 200):k])) for k in slows if k > 2]
  return [(s_log[k], kap[k]) for k in apex], [s_log[k] for k in stops], [s_log[k] for k in slows]


def clear_road(D, a, b):
  """Per log frame: nothing in the next CLEAR_T s gave a reason to be slow."""
  v = D['v_ego'][a:b]
  lat = np.abs(D['curvature'][a:b]) * v ** 2
  w = int(CLEAR_T / DT_MDL)
  out = np.zeros(b - a, bool)
  if b - a > w:
    n = b - a - w
    win = lambda x: sliding_window_view(x, w)[:n]  # noqa: E731
    out[:n] = ~win(D['lead0_present'][a:b]).any(1) & (win(lat).max(1) <= 1.0) & (win(v).min(1) >= np.maximum(v[:n] - 2., 0.3))
  return out


def at_s(rows, s_q):
  return float(np.interp(s_q, rows[:, 0], rows[:, 1])) if rows[-1, 0] >= s_q else np.nan


def score_segment(args):
  name, specs = args
  D = dict(np.load(os.path.join(CACHE, name + '.npz')))
  from e2e_set_speed_replay import AlwaysOn
  ctls = {k: load(sp)(params=AlwaysOn(), dt=DT_MDL) if sp else None for k, sp in specs.items()}
  out = {k: {f: [] for f in ('trk', 'apex', 'stop30', 'stop10', 'slow', 'gap', 'jerk', 'bjerk', 'fid')} for k in specs}
  for a, b in stretches(D):
    res = {}
    for k, c in ctls.items():
      res[k], s_log = drive(D, a, b, c)
    base = res['off']
    apexes, stops, slows = events(D, a, b, s_log)
    clear = clear_road(D, a, b)
    for k, R in res.items():
      if len(R) < 20 or len(base) < 20:
        continue
      o = out[k]
      n = min(len(R), len(base))
      Rn, Bn = R[:n], base[:n]
      mph = CV.MPH_TO_MS
      lazy = clear[Rn[:, 4].astype(int)] & clear[Bn[:, 4].astype(int)] & (Bn[:, 5] - Bn[:, 1] > mph)
      if lazy.any():
        o['trk'].append((lazy.sum(), np.clip(Rn[lazy, 5] - Rn[lazy, 1], 0, None).sum(), (Rn[lazy, 5] - Rn[lazy, 1] < mph).sum()))
      for sa, kap in apexes:
        vr, vb = at_s(R, sa), at_s(base, sa)
        if np.isfinite(vr) and np.isfinite(vb):
          o['apex'].append((vr - vb, kap * vr ** 2))
      for ss in stops:
        for key, back in (('stop30', 30.), ('stop10', 10.)):
          vr, vb = at_s(R, ss - back), at_s(base, ss - back)
          if np.isfinite(vr) and np.isfinite(vb):
            o[key].append(vr - vb)
      for ss in slows:
        vr, vb = at_s(R, ss), at_s(base, ss)
        if np.isfinite(vr) and np.isfinite(vb):
          o['slow'].append(vr - vb)
      lp = (Rn[:, 6] > 0) & (Rn[:, 1] > 5.)
      if lp.any():
        o['gap'].append((lp.sum(), (Rn[lp, 7] / Rn[lp, 1] < 1.0).sum()))
      o['jerk'].append(np.abs(np.diff(R[:, 2])) / DT_MDL)
      o['bjerk'].append(np.abs(np.diff(R[:, 3])) / DT_MDL)
      if k == 'off':
        o['fid'].append(R[:, 1] - D['v_ego'][a + R[:, 4].astype(int)])
  return out


def summarize(outs, keys):
  res = {}
  for k in keys:
    o = {f: [x for seg in outs for x in seg[k][f]] for f in outs[0][k]}
    def arr(f, width=None, o=o):
      if not o[f]:
        return np.zeros((1, width) if width else 1)
      return np.concatenate(o[f]) if f in ('jerk', 'bjerk') else np.asarray(o[f])
    trk, apex, gap = arr('trk', 3), arr('apex', 2), arr('gap', 2)
    s30, s10, slow, j, bj = (arr(f) for f in ('stop30', 'stop10', 'slow', 'jerk', 'bjerk'))
    frames = max(trk[:, 0].sum(), 1)
    res[k] = {
      'clear_deficit_mph': trk[:, 1].sum() / frames * CV.MS_TO_MPH,
      'clear_within1mph_%': trk[:, 2].sum() / frames * 100,
      'apex_n': len(o['apex']), 'apex_>0.5_%': float(np.mean(apex[:, 0] > 0.5) * 100),
      'apex_excess_p90': float(np.percentile(apex[:, 0], 90)), 'apex_lat_>2_%': float(np.mean(apex[:, 1] > 2.0) * 100),
      'stop_n': len(o['stop30']), 'stop30_>0.5_%': float(np.mean(s30 > 0.5) * 100),
      'stop10_>0.5_%': float(np.mean(s10 > 0.5) * 100), 'stop30_p90': float(np.percentile(s30, 90)),
      'slow_n': len(o['slow']), 'slow_>0.5_%': float(np.mean(slow > 0.5) * 100), 'slow_p90': float(np.percentile(slow, 90)),
      'lead_tg<1s_%': gap[:, 1].sum() / max(gap[:, 0].sum(), 1) * 100,
      'jerk_p99': float(np.percentile(j, 99)), 'jerk_p999': float(np.percentile(j, 99.9)),
      'boost_jerk_p99': float(np.percentile(bj, 99)), 'boost_jerk_>1.05_%': float(np.mean(bj > 1.05) * 100),
    }
    if o['fid']:
      res[k]['off_vs_log_rms_mps'] = float(np.sqrt(np.mean(np.concatenate(o['fid']) ** 2)))
  return res


def split(names, which):
  """dev / holdout: alternate routes, so no route is in both."""
  if which == 'all':
    return names
  dev = set(sorted({n.rsplit('--', 1)[0] for n in names})[::2])
  return [n for n in names if (n.rsplit('--', 1)[0] in dev) == (which == 'dev')]


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  sub = ap.add_subparsers(dest='cmd', required=True)
  c = sub.add_parser('cache')
  c.add_argument('rlogs', nargs='+')
  s = sub.add_parser('score')
  s.add_argument('--split', default='dev', choices=['dev', 'holdout', 'all'])
  s.add_argument('cands', nargs='+', help='label=path.py:Class[?CONST=json&...]')
  args = ap.parse_args()
  os.makedirs(CACHE, exist_ok=True)
  sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
  with Pool() as pool:
    if args.cmd == 'cache':
      pool.map(cache_segment, args.rlogs)
      print(f"{sum(f.endswith('.npz') for f in os.listdir(CACHE))} segments cached in {CACHE}")
      return
    specs = {'off': None, **dict(cand.split('=', 1) for cand in args.cands)}
    names = split(sorted(f[:-4] for f in os.listdir(CACHE) if f.endswith('.npz')), args.split)
    res = summarize(pool.map(score_segment, [(n, specs) for n in names]), list(specs))
  print(f"segments {len(names)} ({args.split})")
  print(f"{'metric':22s}" + ''.join(f"{k:>12s}" for k in specs))
  for col in res['off']:
    print(f"{col:22s}" + ''.join(f"{res[k].get(col, float('nan')):12.3f}" for k in specs))


if __name__ == "__main__":
  main()
