"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

sunnypilot's own vision curve planner, plus the per-car lateral ceiling learner, for the brands
zoompilot has not measured. The planner is unchanged: its curve speed cap is still
v * sqrt(a_lat / predicted lateral acceleration), but a_lat is the car's learned ceiling instead
of the fixed 2 m/s2 (both scaled by the Curve Speed profile) once "Learn Steering Limit" is on and the car has learned. Off, or before the
first lesson, it is exactly sunnypilot's. See docs/zoompilot/scc-curve-planning.md.
"""
import openpilot.cereal.messaging as messaging
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.vision_controller import _A_LAT_REG_MAX
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.curve_speed_profile import (
  SmartCruiseControlVision as ProfileVision)
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.lat_limit_learner import LEARN_PARAM, LatLimitLearner

# the learner's speed breakpoints (35, 55, 70 mph); the starting ceiling is sunnypilot's flat cap
_BP = [15.6, 24.6, 31.3]
_PRIOR = [_A_LAT_REG_MAX] * 3


class SmartCruiseControlVision(ProfileVision):
  def __init__(self, CP):
    super().__init__()
    self.lat_limit = LatLimitLearner(_BP, _PRIOR, CP.carFingerprint, self.params)
    self.learn_lat_limit = self.params.get_bool(LEARN_PARAM)

  def _update_params(self) -> None:
    super()._update_params()
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.learn_lat_limit = self.params.get_bool(LEARN_PARAM)

  def _update_lat_limit(self, sm: messaging.SubMaster) -> None:
    valid = getattr(sm, 'valid', {})
    if not all(valid.get(s, False) for s in ('controlsState', 'carState', 'modelV2')):
      return
    cs = sm['controlsState']
    lac = getattr(cs.lateralControlState, cs.lateralControlState.which())
    v = max(self.v_ego, 0.3)
    # toggled off, the learner sees lateral as inactive: it pauses and keeps its saved ceiling
    self.lat_limit.update(self.v_ego, cs.curvature * v ** 2, sm['modelV2'].action.desiredCurvature * v ** 2,
                          lac.active and self.learn_lat_limit, lac.saturated, sm['carState'].steeringPressed)

  def _update_calculations(self, sm: messaging.SubMaster) -> None:
    super()._update_calculations(sm)
    self._update_lat_limit(sm)
    if self.long_enabled and self.learn_lat_limit and self.lat_limit.learned and self.max_pred_lat_acc > 0.:
      self.v_target = max(self.v_ego, 0.1) * (self.lat_limit.ceiling_at(self.v_ego) / self.max_pred_lat_acc) ** 0.5 * self.speed_scale
