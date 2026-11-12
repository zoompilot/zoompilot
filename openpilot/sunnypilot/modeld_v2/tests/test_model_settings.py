"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace
from unittest import mock

import pytest

from openpilot.cereal import log
from openpilot.sunnypilot.modeld_v2.model_settings import BIG_MODEL_LAT_SMOOTH_SECONDS, PARAMS_FRAMES, ModelSettings


def _sm(frame: int, lateral_delay: float = 0.2):
  sm = mock.MagicMock()
  sm.frame = frame
  sm.__getitem__.side_effect = lambda k: SimpleNamespace(lateralDelay=lateral_delay)
  return sm


def _params(values: dict):
  params = mock.Mock()
  params.get_bool.side_effect = lambda k: bool(values.get(k, False))
  params.get.side_effect = lambda k, return_default=False: values.get(k, 0.0)
  return params


class TestModelSettings:
  def test_refresh_reads_params_every_params_frames(self):
    settings, model = ModelSettings(), SimpleNamespace(lat_delay=0.0)
    params = _params({"Blendful": True, "CameraOffset": 0.1, "LagdToggle": True})
    assert not settings.refresh(_sm(PARAMS_FRAMES - 1), params, model)
    assert (model.lat_delay, settings.blendful_enabled) == (0.0, False)
    assert settings.refresh(_sm(PARAMS_FRAMES), params, model)
    assert (model.lat_delay, settings.blendful_enabled, settings.camera_offset.camera_offset) == (0.2, True, 0.1)

  def test_fixed_steer_delay_is_used(self):
    settings, model = ModelSettings(), SimpleNamespace(lat_delay=0.0)
    settings.refresh(_sm(0), _params({"LagdToggle": False, "LagdValueCache": 0.35}), model)
    assert model.lat_delay == 0.35

  def test_callback_is_inactive_during_a_lane_change(self):
    settings = ModelSettings()
    settings.blendful.update = mock.Mock(return_value=0.0)
    settings.blendful_enabled = True
    for desire, lane_change, active in ((log.Desire.none, log.LaneChangeState.off, True),
                                        (log.Desire.laneChangeLeft, log.LaneChangeState.laneChangeStarting, False)):
      settings.blendful_callback({}, 20.0, desire, lane_change)(0.001)
      settings.blendful.update.assert_called_with({}, 20.0, active, 0.001, enabled=True)

  def test_big_model_smoothing_reads_its_param(self):
    settings, model = ModelSettings(), SimpleNamespace(lat_delay=0.0)
    settings.refresh(_sm(0), _params({"BigModelSmoothing": True}), model)
    assert settings.big_model_smoothing

  def test_big_model_smoothing_only_for_a_big_model_that_smooths_none(self):
    settings = ModelSettings()
    big, small = SimpleNamespace(chestnut=True, LAT_SMOOTH_SECONDS=0.0), SimpleNamespace(chestnut=False, LAT_SMOOTH_SECONDS=0.0)
    assert settings.big_lat_smooth(big) == 0.0, "off by default"
    settings.big_model_smoothing = True
    assert settings.big_lat_smooth(big) == BIG_MODEL_LAT_SMOOTH_SECONDS
    assert settings.big_lat_smooth(SimpleNamespace(chestnut=True)) == BIG_MODEL_LAT_SMOOTH_SECONDS, "jetlink's stock face may lack the attribute"
    assert settings.big_lat_smooth(small) == 0.0
    assert settings.big_lat_smooth(SimpleNamespace(chestnut=True, LAT_SMOOTH_SECONDS=0.2)) == 0.0, "a bundle's own smoothing is kept"

  def test_smooth_curvature_matches_a_bundles_smoothing(self):
    from openpilot.selfdrive.controls.lib.drive_helpers import smooth_value
    prev = log.ModelDataV2.Action(desiredCurvature=0.0)
    action = log.ModelDataV2.Action(desiredCurvature=0.01)
    ModelSettings.smooth_curvature(action, prev, 0.1)
    assert action.desiredCurvature == pytest.approx(smooth_value(0.01, 0.0, 0.1))
    assert 0.0 < action.desiredCurvature < 0.01

  def test_smooth_curvature_leaves_the_action_alone_when_off(self):
    action = log.ModelDataV2.Action(desiredCurvature=0.01)
    ModelSettings.smooth_curvature(action, log.ModelDataV2.Action(desiredCurvature=0.0), 0.0)
    assert action.desiredCurvature == pytest.approx(0.01)

  def test_smooth_curvature_keeps_a_held_curvature(self):
    """Below the lateral control speed the action holds the previous curvature; smoothing keeps it."""
    prev = log.ModelDataV2.Action(desiredCurvature=0.004)
    action = log.ModelDataV2.Action(desiredCurvature=0.004)
    ModelSettings.smooth_curvature(action, prev, 0.1)
    assert action.desiredCurvature == pytest.approx(0.004)
