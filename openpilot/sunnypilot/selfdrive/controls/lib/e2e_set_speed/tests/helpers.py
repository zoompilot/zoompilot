"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.controller import T_IDXS

# update() arguments for an engaged, lead-free e2e frame
ENGAGED = {'is_e2e': True, 'reset_state': False}


def plan_ramp(v_ego: float, drop: float) -> np.ndarray:
  """The model's plan easing from v_ego down by drop over the first 10 s of the horizon."""
  return v_ego - drop * np.minimum(T_IDXS, 10.) / 10.


def build_sm(v_ego: float, plan_v=None, yaw_rate=0., gas=False, brake=False, should_stop=False,
             hard_brake=False, force_decel=False, lane_change=False, lead=None) -> dict:
  """Readers for the four services the controller reads. plan_v and yaw_rate take a scalar or a
  33-point trajectory on the model's T_IDXS. lead: (dRel, vRel) of a present leadOne."""
  cs = messaging.new_message('carState')
  cs.carState.vEgo = v_ego
  cs.carState.gasPressed = gas
  cs.carState.brakePressed = brake

  md = messaging.new_message('modelV2')
  vel = np.full(len(T_IDXS), v_ego) if plan_v is None else np.broadcast_to(np.asarray(plan_v, dtype=float), T_IDXS.shape)
  md.modelV2.velocity.x = vel.tolist()
  md.modelV2.orientationRate.z = np.broadcast_to(np.asarray(yaw_rate, dtype=float), T_IDXS.shape).tolist()
  md.modelV2.action.shouldStop = should_stop
  md.modelV2.meta.hardBrakePredicted = hard_brake
  md.modelV2.meta.laneChangeState = log.LaneChangeState.laneChangeStarting if lane_change else log.LaneChangeState.off

  rs = messaging.new_message('radarState')
  if lead is not None:
    rs.radarState.leadOne.present = True
    rs.radarState.leadOne.dRel, rs.radarState.leadOne.vRel = (float(x) for x in lead)

  ctrl = messaging.new_message('controlsState')
  ctrl.controlsState.forceDecel = force_decel

  return {'carState': cs.carState.as_reader(), 'modelV2': md.modelV2.as_reader(),
          'radarState': rs.radarState.as_reader(), 'controlsState': ctrl.controlsState.as_reader()}
