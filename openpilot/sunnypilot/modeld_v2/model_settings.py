"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from collections.abc import Callable
from functools import partial

from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.drive_helpers import smooth_value
from openpilot.selfdrive.modeld.constants import MIN_LAT_CONTROL_SPEED
from openpilot.sunnypilot.livedelay.helpers import get_lat_delay
from openpilot.sunnypilot.modeld_v2.blendful import Blendful
from openpilot.sunnypilot.modeld_v2.camera_offset_helper import CameraOffsetHelper

PARAMS_FRAMES = 60

# the small model bundles' `lat` override. A big model runs comma's stock modeld constants, which smooth
# nothing since openpilot #36987; on straights that leaves a ~1 Hz wiggle in its curvature (11-11 drives)
BIG_MODEL_LAT_SMOOTH_SECONDS = 0.1


class ModelSettings:
  """The model settings both modelds run: Lane Center Assist (Blendful), camera offset, steer delay and
  big model smoothing."""

  def __init__(self):
    self.blendful = Blendful()
    self.blendful_enabled = False
    self.camera_offset = CameraOffsetHelper()
    self.big_model_smoothing = False

  def refresh(self, sm, params: Params, model) -> bool:
    """Reads the params every PARAMS_FRAMES frames; True on the frames it read them."""
    if sm.frame % PARAMS_FRAMES != 0:
      return False
    model.lat_delay = get_lat_delay(params, sm["lateralDelay"].lateralDelay)
    self.camera_offset.set_offset(params.get("CameraOffset", return_default=True))
    self.blendful_enabled = params.get_bool("Blendful")
    self.big_model_smoothing = params.get_bool("BigModelSmoothing")
    return True

  def big_lat_smooth(self, model) -> float:
    """Seconds of curvature smoothing this frame adds on top of the model's own: the small models' amount for a
    big model that smooths none itself, with the toggle on; 0 otherwise. Checked per frame, as jetlink can hand
    the drive between models at any frame. modeld adds it to the delay the model plans for, as it does a
    bundle's own, so the smoothing's lag is planned ahead rather than added."""
    if not (self.big_model_smoothing and model.chestnut) or getattr(model, 'LAT_SMOOTH_SECONDS', 0.0) > 0.0:
      return 0.0
    return BIG_MODEL_LAT_SMOOTH_SECONDS

  @staticmethod
  def smooth_curvature(action, prev_action, v_ego: float, seconds: float) -> None:
    """Smooth the action's curvature by `seconds`, as modeld_v2 smooths a bundle's: above the lateral control
    speed only (below it the action already holds the previous curvature)."""
    if seconds > 0.0 and v_ego > MIN_LAT_CONTROL_SPEED:
      action.desiredCurvature = float(smooth_value(action.desiredCurvature, prev_action.desiredCurvature, seconds))

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
