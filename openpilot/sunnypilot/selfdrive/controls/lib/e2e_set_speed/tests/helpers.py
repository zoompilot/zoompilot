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
ENGAGED = {'is_e2e': True, 'reset_state': False, 'dec_active': False, 'allow_throttle': True, 'fcw': False, 'accel_coast': 0.}


class MockParams:
  def __init__(self, enabled: bool = True):
    self.enabled = enabled

  def get_bool(self, key: str) -> bool:
    return self.enabled


def build_sm(v_ego: float, plan_v=None, yaw_rate=0., curvature=0., lead=False, gas=False, brake=False,
             should_stop=False, hard_brake=False, force_decel=False, lane_change=False) -> dict:
  """Readers for the four services the controller reads. plan_v and yaw_rate take a scalar or a
  33-point trajectory on the model's T_IDXS."""
  cs = messaging.new_message('carState')
  cs.carState.vEgo = v_ego
  cs.carState.gasPressed = gas
  cs.carState.brakePressed = brake

  md = messaging.new_message('modelV2')
  vel = np.full(len(T_IDXS), v_ego) if plan_v is None else np.broadcast_to(np.asarray(plan_v, dtype=float), T_IDXS.shape)
  md.modelV2.velocity.x = vel.tolist()
  md.modelV2.orientationRate.z = np.broadcast_to(np.asarray(yaw_rate, dtype=float), T_IDXS.shape).tolist()
  md.modelV2.action.desiredCurvature = curvature
  md.modelV2.action.shouldStop = should_stop
  md.modelV2.meta.hardBrakePredicted = hard_brake
  md.modelV2.meta.laneChangeState = log.LaneChangeState.laneChangeStarting if lane_change else log.LaneChangeState.off

  rs = messaging.new_message('radarState')
  rs.radarState.leadOne.present = lead

  ctrl = messaging.new_message('controlsState')
  ctrl.controlsState.forceDecel = force_decel

  return {'carState': cs.carState.as_reader(), 'modelV2': md.modelV2.as_reader(),
          'radarState': rs.radarState.as_reader(), 'controlsState': ctrl.controlsState.as_reader()}
