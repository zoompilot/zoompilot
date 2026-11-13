"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Curve Speed profiles: one setting that scales the lateral-acceleration ceiling every curve
planner drives to, on every brand. Map targets are speeds, so they scale by the square root.
Budgets, gates and latches stay as tuned. The sunnypilot controllers are wrapped here, not
edited, so their files stay byte-identical to upstream.
"""
import math

from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.map_controller import SmartCruiseControlMap as SPMap
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.vision_controller import SmartCruiseControlVision as SPVision

PARAM = "CurveSpeedProfile"
# lateral-acceleration ceiling scale by stored value: Gentle, Normal, Sporty
SCALES = (0.85, 1.0, 1.1)
NORMAL = 1


def read_scale(params) -> float:
  try:
    idx = int(params.get(PARAM, return_default=True))
  except (TypeError, ValueError):
    idx = NORMAL
  return SCALES[min(max(idx, 0), len(SCALES) - 1)]


def scale_targets(targets: list, speed_scale: float) -> list:
  """Map target velocities at speed_scale, in place: every caller hands over the list
  velocities_from_param just parsed."""
  for p in targets:
    p["velocity"] *= speed_scale
  return targets


class CurveSpeedProfile:
  """lat_scale, and speed_scale for speeds, follow the param on the controller's own params refresh."""
  lat_scale = 1.0
  speed_scale = 1.0

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self._set_scale(read_scale(self.params))

  def _set_scale(self, scale: float) -> None:
    self.lat_scale, self.speed_scale = scale, math.sqrt(scale)

  def _refresh_profile(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0 and (scale := read_scale(self.params)) != self.lat_scale:
      self._set_scale(scale)


class VisionProfile(CurveSpeedProfile):
  def _update_params(self) -> None:
    super()._update_params()
    self._refresh_profile()


class MapProfile(CurveSpeedProfile):
  """The controller reads MapTargetVelocities into target_velocities every frame; scaling on
  that assignment covers the selection and the held-target match alike."""
  _target_velocities: list = []

  @property
  def target_velocities(self) -> list:
    return self._target_velocities

  @target_velocities.setter
  def target_velocities(self, targets: list) -> None:
    self._target_velocities = scale_targets(targets, self.speed_scale)

  def update_params(self) -> None:
    super().update_params()
    self._refresh_profile()


class SmartCruiseControlVision(VisionProfile, SPVision):
  """sunnypilot's vision controller with its flat ceiling scaled."""
  def _update_calculations(self, sm) -> None:
    super()._update_calculations(sm)
    # v_target is only computed with long enabled; elsewhere it is stale and must not compound
    if self.long_enabled:
      self.v_target *= self.speed_scale


class SmartCruiseControlMap(MapProfile, SPMap):
  """sunnypilot's map controller with its targets scaled."""
