"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from collections.abc import Callable
from functools import partial

from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.sunnypilot.livedelay.helpers import get_lat_delay
from openpilot.sunnypilot.modeld_v2.blendful import Blendful
from openpilot.sunnypilot.modeld_v2.camera_offset_helper import CameraOffsetHelper

PARAMS_FRAMES = 60


class ModelSettings:
  """The model settings both modelds run: Lane Center Assist (Blendful), camera offset and steer delay."""

  def __init__(self):
    self.blendful = Blendful()
    self.blendful_enabled = False
    self.camera_offset = CameraOffsetHelper()

  def refresh(self, sm, params: Params, model) -> bool:
    """Reads the params every PARAMS_FRAMES frames; True on the frames it read them."""
    if sm.frame % PARAMS_FRAMES != 0:
      return False
    model.lat_delay = get_lat_delay(params, sm["lateralDelay"].lateralDelay)
    self.camera_offset.set_offset(params.get("CameraOffset", return_default=True))
    self.blendful_enabled = params.get_bool("Blendful")
    return True

  def blendful_callback(self, model_output, v_ego: float, desire, lane_change_state) -> Callable[[float], float]:
    active = desire == log.Desire.none and lane_change_state == log.LaneChangeState.off
    return partial(self.blendful.update, model_output, v_ego, active, enabled=self.blendful_enabled)


def with_blendful(get_action_from_model, min_lat_control_speed: float):
  """Stock modeld's action function taking Blendful's callback as modeld_v2's does. Stock leaves the curvature
  unsmoothed (LAT_SMOOTH_SECONDS 0), so the correction added to its answer is the one modeld_v2 adds before smoothing."""
  def action(model_output, prev_action, lat_action_t, long_action_t, v_ego, blendful=None):
    action = get_action_from_model(model_output, prev_action, lat_action_t, long_action_t, v_ego)
    if blendful is not None:
      correction = blendful(action.desiredCurvature)
      if v_ego > min_lat_control_speed:
        action.desiredCurvature += correction
    return action
  return action
