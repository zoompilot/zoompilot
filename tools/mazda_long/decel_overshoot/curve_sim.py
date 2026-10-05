"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Closed-loop curve sweep of the checked-out stock-ACC stack: the real SmartCruiseControlVision
and the real ICBM servo against the fitted MRCC plant (fit_plant.py), with the map controller's
commit gate and published decel for one waypoint. The road is straight, then a curve whose
radius puts it exactly at the planner's allowed speed, with 40 m spirals. The model path is
rebuilt from the road with the measured range read of the chosen model (--model small|big) on
bends that need slowing, below 45 mph blending into its highway read above 55.

Servo at 100 Hz: a press registers 1 mph every 0.25 s (measured 4 mph/s walk).
LP_SP.aTarget is emulated as the MPC does it on stock ACC: railing at -1.2 below the target.
To compare code versions, run it on each checkout.

Usage: curve_sim.py [--perfect] [--plant-scale 1.0] [--model small|big] [--cache DIR]
"""
import argparse
import os
import pickle

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log, custom
from opendbc.car import structs
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import limits as lim_mod
try:
  from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import map_controller as mc, vision_controller as vc
except ImportError:  # a checkout from before the planner moved (a baseline worktree): vision rows only
  from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import vision_controller as vc
  mc = None
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import IntelligentCruiseButtonManagement

from extract import OUT, MPH
from model_reach import A_LAT  # the fixed scoring reference; route_sim reads CS.A_LAT

DT = 0.05
T_IDXS = np.array(ModelConstants.T_IDXS)
Src = custom.LongitudinalPlanSP.LongitudinalPlanSource
Send = custom.IntelligentCruiseButtonManagement.SendButtonState
SCENARIOS = [(35, 25), (45, 35), (45, 25), (55, 45), (55, 35), (55, 25), (65, 55), (65, 45), (65, 35), (75, 60), (75, 45)]


class Plant:
  def __init__(self, fit, scale=1.0):
    self.gb, self.coef, self.scale = fit['GB'], fit['coef'], scale
    self.tau, self.delay = fit['tau'], fit['delay']

  def accel(self, gap, v):
    nb = len(self.gb)
    g = float(np.clip(gap, self.gb[0], self.gb[-1]))
    a = np.interp(g, self.gb, self.coef[:nb]) + np.interp(g, self.gb, self.coef[nb:]) * (v - 20.) / 10.
    return a * self.scale if a < 0 else a


class Road:
  def __init__(self, s0, radius, length=150., spiral=40.):
    self.bp = [s0 - spiral, s0, s0 + length, s0 + length + spiral]
    self.k = 1. / radius

  def kappa(self, s):
    return np.interp(s, self.bp, [0., self.k, self.k, 0.], left=0., right=0.)


# median model curvature read / driven curvature on bends that need slowing, by distance ahead
# (every drive in the corpus, split by modelV2.big and by speed band)
_BIAS_D = [10., 30., 50., 70., 90., 110., 130., 155., 185., 225.]
_BIAS = {
  (False, 'low'): [1.07, 0.98, 0.86, 0.51, 0.26, 0.21, 0.13, 0.08, 0.04, 0.04],
  (True, 'low'): [1.02, 0.96, 0.89, 0.73, 0.48, 0.47, 0.24, 0.21, 0.15, 0.28],
  (False, 'hwy'): [1.06, 1.05, 1.04, 1.03, 1.01, 0.98, 0.92, 0.81, 0.60, 0.41],
  (True, 'hwy'): [0.93, 0.89, 0.94, 0.92, 0.90, 0.85, 0.86, 0.83, 0.78, 0.63],
}


def model_bias(d, v, big):
  w = float(np.interp(v, [20.1, 24.6], [0., 1.]))  # 45 -> 55 mph
  return (1 - w) * np.interp(d, _BIAS_D, _BIAS[(big, 'low')]) + w * np.interp(d, _BIAS_D, _BIAS[(big, 'hwy')])


def build_sm(road, pos, v, bias, big=False):
  v = max(v, 0.5)
  dd = v * T_IDXS
  k = road.kappa(pos + dd) * (model_bias(dd, v, big) if bias else 1.)
  m = messaging.new_message('modelV2')
  p = log.XYZTData.new_message()
  p.x, p.y = [float(x) for x in dd], [0.] * len(dd)
  m.modelV2.position = p
  vel = log.XYZTData.new_message()
  vel.x = [float(v)] * len(dd)
  m.modelV2.velocity = vel
  o = log.XYZTData.new_message()
  o.z = [float(x * v) for x in k]
  m.modelV2.orientationRate = o
  m.modelV2.big = big
  cs = messaging.new_message('controlsState')
  cs.controlsState.curvature = float(road.kappa(pos))
  return {'modelV2': m.modelV2, 'controlsState': cs.controlsState}


class MapWaypoint:
  """SmartCruiseControlMap's commit gate (target_binds) and published decel (target_decel) for a
  single waypoint. Its confirmation time at speed is left out: the waypoint is a real curve that
  never flickers, so it binds on the first frame the gate passes."""
  def __init__(self, CP, s_wp, v_wp):
    self.lim, self.s_wp, self.v_wp = lim_mod.get_planning_limits(CP), s_wp, v_wp
    self.active, self.a_out = False, 0.

  def update(self, pos, v, a_ego):
    d = self.s_wp - pos
    if d <= 0 or self.v_wp >= v:
      self.active = False
    elif not self.active:
      self.active = bool(mc.target_binds(v, [self.v_wp], [d], self.lim)[0])
    if self.active:
      self.a_out = lim_mod.publish_ramp(-mc.target_decel(v, self.v_wp, d, self.lim), self.a_out, self.lim, v)
    else:
      self.a_out = a_ego
    return (self.v_wp if self.active else 255.), self.a_out


class Stack:
  """The stock-ACC stack on the plant: the vision planner and the plan at 20 Hz, the ICBM servo,
  dash and plant at 100 Hz."""
  def __init__(self, plant, set_mph, pos=0.):
    Params().put_bool("SmartCruiseControlVision", True, block=True)
    self.CP = structs.CarParams(brand="mazda", openpilotLongitudinalControl=False, longitudinalActuatorDelay=0.15)
    self.scc = vc.SmartCruiseControlVision(self.CP)
    self.icbm = IntelligentCruiseButtonManagement(car.CarParams(pcmCruise=True, brand="mazda"), custom.CarParamsSP(pcmCruiseSpeed=False))
    self.CC = car.CarControl(enabled=True)
    self.plant, self.set_mph, self.v_set = plant, set_mph, set_mph / MPH
    self.pos, self.v, self.a, self.dash, self.a_mpc = pos, self.v_set, 0., float(set_mph), 0.
    self.press_dir, self.press_n = 0, 0
    self.delay = [0.] * int(plant.delay / 0.01 + 1)

  def step(self, sm, extra=None):
    """One 0.05 s step on the modelV2/controlsState sm; extra holds more plan candidates
    {source: (v_target, a_target)}. Returns position, speed, accel, dash mph and the plan's vTarget."""
    scc, icbm, plant = self.scc, self.icbm, self.plant
    pos, v, a, dash = self.pos, self.v, self.a, self.dash
    scc.update(sm, True, False, v, a, self.v_set)
    cands = {Src.cruise: (self.v_set, a), Src.sccVision: (scc.output_v_target, scc.output_a_target), **(extra or {})}
    src = min(cands, key=lambda k: cands[k][0])
    v_t = cands[src][0]
    self.a_mpc += DT / 0.5 * (float(np.clip(0.9 * (v_t - v), -1.2, 1.2)) - self.a_mpc)
    LP_SP = custom.LongitudinalPlanSP(vTarget=float(v_t), aTarget=float(self.a_mpc))
    LP_SP.longitudinalPlanSource = src
    LP_SP.smartCruiseControl.vision.vAheadMin = float(scc.v_ahead_min)
    LP_SP.smartCruiseControl.vision.aTarget = float(scc.output_a_target)
    if Src.sccMap in cands:
      LP_SP.smartCruiseControl.map.aTarget = float(cands[Src.sccMap][1])
    LP_SP.speedLimit.assist.vTarget = 255.
    for _ in range(5):
      CS = car.CarState(vEgo=float(v), aEgo=float(a), cruiseState={"speedCluster": float(dash / MPH), "enabled": True})
      CS.vCruise = float(self.set_mph * CV.MPH_TO_KPH)
      icbm.run(CS, self.CC, LP_SP, is_metric=False)
      b = icbm.cruise_button
      d = -1 if b in (Send.decrease, Send.decreaseHold) else 1 if b in (Send.increase, Send.increaseHold) else 0
      self.press_n = self.press_n + 1 if d == self.press_dir and d else (1 if d else 0)
      self.press_dir = d
      if d and self.press_n % 25 == 5:
        dash = min(dash + d, float(self.set_mph)) if d > 0 else max(dash + d, 20.)
      self.delay.append((v - dash / MPH) * MPH)
      a += 0.01 / plant.tau * (plant.accel(self.delay.pop(0), v) - a)
      v = max(v + a * 0.01, 0.5)
      pos += v * 0.01
    self.pos, self.v, self.a, self.dash = pos, v, a, dash
    return pos, v, a, dash, v_t


def run(plant, set_mph, curve_mph, use_map, bias, big=False):
  st = Stack(plant, set_mph)
  v_c, s0 = curve_mph / MPH, 1200.
  road = Road(s0, v_c ** 2 / A_LAT)
  wp = MapWaypoint(st.CP, s0 - 40., v_c) if use_map else None
  rows = []
  while st.pos < s0 + 190. + 25. * st.v_set:
    extra = {Src.sccMap: wp.update(st.pos, st.v, st.a)} if wp else None
    rows.append(st.step(build_sm(road, st.pos, st.v, bias, big), extra))
  s, vv, aa = np.array(rows).T[:3]
  i_in = int(np.searchsorted(s, s0))
  brake = np.flatnonzero(aa < -0.25)
  window = s < s0 + 150. + 60.
  return {'a_min': aa.min(), 'v_entry': vv[i_in] * MPH, 'under': (v_c - vv[window].min()) * MPH,
          'onset': s0 - s[brake[0]] if len(brake) else np.nan,
          'lat': float(np.max(vv ** 2 * road.kappa(s))),
          'crawl': float(np.sum((s < s0) & (vv <= v_c + 2. / MPH)) * DT)}


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--perfect', action='store_true', help='model path without the range bias')
  ap.add_argument('--plant-scale', type=float, default=1.0, help='scale the decel side of the plant')
  ap.add_argument('--model', choices=('small', 'big'), default='small')
  ap.add_argument('--cache', default=OUT)
  args = ap.parse_args()
  with open(os.path.join(args.cache, 'plant_fit.pkl'), 'rb') as f:
    plant = Plant(pickle.load(f), args.plant_scale)
  print("set->curve  src   peak a  v_entry  under  onset_m  apex_lat  crawl_s")
  for use_map in (False, True) if mc else (False,):
    for set_mph, curve_mph in SCENARIOS:
      m = run(plant, set_mph, curve_mph, use_map, not args.perfect, args.model == 'big')
      src = 'map' if use_map else 'vis'
      print(f"{set_mph:3d}->{curve_mph:<3d}    {src}  {m['a_min']:6.2f}  {m['v_entry']:6.1f}  {m['under']:5.1f}  "
            + f"{m['onset']:6.0f}  {m['lat']:7.2f}  {m['crawl']:6.1f}")


if __name__ == '__main__':
  main()
