"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace
from unittest import mock

from openpilot.cereal import log
from openpilot.sunnypilot.modeld_v2.model_settings import PARAMS_FRAMES, ModelSettings


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
