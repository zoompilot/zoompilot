"""Route 135 closed-loop replay driving the SHIPPED SmartCruiseControlVision.

scc_sim_135.py mirrors the solver so configs can be swept cheaply; this one instantiates
the real controller and feeds it reconstructed modelV2 messages, so what is scored is the
code that actually runs on the car. Used to confirm a change before and after landing it.
"""
import os, pickle, argparse
import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.params import Params
from opendbc.car import structs
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import vision_controller
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.vision_controller import (
    SmartCruiseControlVision)

from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import (
    DECEL_OVERSHOOT_PARAMS)

from scc_sim_135 import (DIR, MPH, DT, PLANT_GAP, PLANT_A, PLANT_TAU, BASE, load, prep, Servo)

MIN_V = 20 / 3.6


def make_cp():
  return structs.CarParams(brand="mazda", openpilotLongitudinalControl=False,
                           longitudinalActuatorDelay=0.15)


def build_sm(kap, dd, v_ego, cur_curv):
  """Rebuild the modelV2 the controller consumes from a recorded kappa-vs-distance path."""
  m = messaging.new_message('modelV2')
  pos = log.XYZTData.new_message()
  pos.x = [float(x) for x in dd]
  pos.y = [0.0] * len(dd)
  m.modelV2.position = pos
  vel = log.XYZTData.new_message()
  v = max(float(v_ego), 0.5)
  vel.x = [v] * len(dd)
  m.modelV2.velocity = vel
  orate = log.XYZTData.new_message()
  orate.z = [float(k * v) for k in kap]
  m.modelV2.orientationRate = orate
  cs = messaging.new_message('controlsState')
  cs.controlsState.curvature = float(cur_curv)
  return {'modelV2': m.modelV2, 'controlsState': cs.controlsState}


def run(cfg, d, s, k_real, paths, i0, i1, set_mph, map_v=None):
  scc = SmartCruiseControlVision(make_cp())
  scc.enabled = True
  v = float(d['v'][i0])
  pos = float(s[i0])
  servo = Servo(cfg, set_mph)
  servo.dash = float(np.round(d['set'][i0] * MPH))
  servo.overshoot = max(min((d['v'][i0] - d['set'][i0]) * MPH, cfg['max_gap']), 0.)
  a_state = float(d['a'][i0])
  out = dict(t=[], s=[], v=[], dash=[], a=[], src=[])
  t = 0.
  s_end = s[i1 - 1]
  for _ in range(int((i1 - i0) * 3)):
    if pos >= s_end:
      break
    k = int(np.clip(np.searchsorted(s, pos), 0, len(s) - 1))
    kap, dd = paths[k]
    scc.update(build_sm(kap, dd, v, float(np.interp(pos, s, k_real))), True, False, v, a_state,
               set_mph / MPH)
    v_tgt, a_tgt = scc.output_v_target, scc.output_a_target
    limiter = v_tgt < set_mph / MPH - 0.1
    if map_v is not None:
      v_map = float(np.interp(pos, s, map_v))
      if v_map < v_tgt:
        v_tgt, limiter = v_map, True
        a_tgt = min(a_tgt, -max(v - v_map, 0.) / max(cfg['t_lead'], 1.))
    dash = servo.step(v, v_tgt, a_tgt, limiter, min(scc.v_ahead_min, 255.))
    a_state += (DT / PLANT_TAU) * (float(np.interp((v - dash / MPH) * MPH, PLANT_GAP, PLANT_A))
                                   - a_state)
    v = max(v + a_state * DT, 1.0)
    pos += v * DT
    t += DT
    out['t'].append(t); out['s'].append(pos); out['v'].append(v)
    out['dash'].append(dash); out['a'].append(a_state); out['src'].append(limiter)
  return {k: np.asarray(val) for k, val in out.items()}


def score(cfg, d, s, k_real, paths, eps, map_v, run_in_s=14.0):
  rows = []
  for e in eps:
    i, hi = e['i'], min(e['hi'] + 40, len(s) - 1)
    lo = max(5, i - int(run_in_s / DT))
    if not d['eng'][lo:i].all():
      lo = e['lo']
    set_mph = float(np.round(np.max(d['set'][max(0, lo - 200):hi]) * MPH))
    r = run(cfg, d, s, k_real, paths, lo, hi, set_mph, map_v)
    if len(r['s']) < 5:
      continue
    j = int(np.argmin(np.abs(r['s'] - s[i])))
    rows.append(dict(t=e['t'], lat_real=e['lat_apex'], lat_sim=r['v'][j] ** 2 * k_real[i],
                     v_sim=r['v'][j], v_real=e['v_apex'], dt=r['t'][-1] - (hi - lo) * DT, r=r))
  return rows


def straight_cost(cfg, d, s, k_real, paths, map_v, apexes, n=6):
  near = np.zeros(len(s), dtype=bool)
  for i in apexes:
    near[max(0, i - 400):min(len(s), i + 200)] = True
  ok = d['eng'] & ~near & (d['v'] > 12)
  runs, i = [], 0
  while i < len(ok):
    if ok[i]:
      j = i
      while j < len(ok) and ok[j]:
        j += 1
      if (j - i) * DT >= 12:
        runs.append((i, j))
      i = j
    else:
      i += 1
  runs = runs[:n]
  lost, dur, lim = 0., 0., 0
  for i, j in runs:
    set_mph = float(np.round(np.max(d['set'][max(0, i - 200):j]) * MPH))
    r = run(cfg, d, s, k_real, paths, i, j, set_mph, map_v)
    if len(r['t']) < 5:
      continue
    lost += r['t'][-1] - (j - i) * DT
    dur += (j - i) * DT
    lim += int(r['src'].sum())
  return lost, dur, lim, len(runs)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--plant', default='fit', choices=('fit', 'weak', 'strong'))
  args = ap.parse_args()
  if args.plant != 'fit':
    k = 0.8 if args.plant == 'weak' else 1.2
    PLANT_A[PLANT_A < 0] *= k

  Params().put_bool("SmartCruiseControlVision", True, block=True)
  d, mp, eps = load()
  s, k_real, paths = prep(d, mp)
  eps = [e for e in eps if e['lo'] > 5]
  map_v = np.where(np.isin(d['mapState'], ('turning',)), d['mapVT'], 1e3)
  apexes = [e['i'] for e in eps]

  base_gain = list(vision_controller._KAPPA_BIAS_GAIN)
  off = [1.0] * len(base_gain)
  # both maps read from the shipped constant, so the harness cannot drift from the car
  new_map = dict(overshoot_gap=list(DECEL_OVERSHOOT_PARAMS['mazda']['gap_v']),
                 max_gap=DECEL_OVERSHOOT_PARAMS['mazda']['max_gap'])
  old_map = dict(overshoot_gap=[1.5, 2.5, 4.0, 6.0, 8.5], max_gap=10.)
  assert new_map['overshoot_gap'] != old_map['overshoot_gap'], "overshoot map change not present"

  variants = {
    'before (no gain, old map)': (off, old_map),
    'gain only': (base_gain, old_map),
    'overshoot map only': (off, new_map),
    'as landed (gain + map)': (base_gain, new_map),
  }

  lat_real = np.array([e['lat_apex'] for e in eps])
  print(f"plant: {args.plant}   controller: the shipped SmartCruiseControlVision\n")
  print(f"{'AS FLOWN':28} med {np.median(lat_real):.2f} p90 {np.percentile(lat_real,90):.2f} "
        f"max {lat_real.max():.2f}  >2.0 {(lat_real>2.0).sum():2} >2.2 {(lat_real>2.2).sum():2}\n")
  print(f"{'variant':28} {'med':>5} {'p90':>5} {'max':>5} {'>2.0':>5} {'>2.2':>5} "
        f"{'apexT':>8} {'strT':>7} {'strLim':>7}")
  print('-' * 92)
  base_dt = None
  for name, (gain, over) in variants.items():
    cfg = dict(BASE)
    cfg.update(over)
    vision_controller._KAPPA_BIAS_GAIN = gain
    try:
      rows = score(cfg, d, s, k_real, paths, eps, map_v)
      lost, dur, lim, nrun = straight_cost(cfg, d, s, k_real, paths, map_v, apexes)
    finally:
      vision_controller._KAPPA_BIAS_GAIN = base_gain
    lat = np.array([r['lat_sim'] for r in rows])
    dt = sum(r['dt'] for r in rows)
    if base_dt is None:
      base_dt = dt
    print(f"{name:28} {np.median(lat):5.2f} {np.percentile(lat,90):5.2f} {lat.max():5.2f} "
          f"{(lat>2.0).sum():5} {(lat>2.2).sum():5} {dt-base_dt:+8.1f} {lost:+7.1f} {lim:7}")
  print(f"\n  apexT  = s added over {len(eps)} curve run-ins vs the pre-change controller")
  print(f"  strT   = s added over {nrun} straight stretches ({dur:.0f}s); strLim = limiter frames there")


if __name__ == '__main__':
  main()
