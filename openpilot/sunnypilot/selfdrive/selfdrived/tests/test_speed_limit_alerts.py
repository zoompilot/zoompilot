"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The speed limit assist prompt and its activation notice name the speed the car is going to,
not just that a limit exists.
"""
import pytest

from openpilot.cereal import custom, messaging
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.selfdrived import events as events_sp_module
from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import ET, AlertSize

EventNameSP = custom.OnroadEventSP.EventName


def _alert(event, target: float, set_speed: float, metric: bool = False, pcm_long: bool = False):
  """target and set_speed in display units; the resolver carries m/s, carState kph."""
  to_ms = CV.KPH_TO_MS if metric else CV.MPH_TO_MS
  lp_sp = messaging.new_message('longitudinalPlanSP').longitudinalPlanSP
  lp_sp.speedLimit.resolver.speedLimitFinalLast = target * to_ms
  lp_sp.speedLimit.assist.vTarget = target * to_ms
  sm = {
    'longitudinalPlanSP': lp_sp,
    'controlsState': messaging.new_message('controlsState').controlsState,
    'carStateSP': messaging.new_message('carStateSP').carStateSP,
  }
  CP = structs.CarParams()
  CP.openpilotLongitudinalControl = pcm_long
  CP.pcmCruise = pcm_long
  CS = structs.CarState()
  CS.vCruiseCluster = set_speed * to_ms * CV.MS_TO_KPH
  return EVENTS_SP[event][ET.WARNING](CP, CS, sm, metric, 0, None)


@pytest.fixture
def mici(monkeypatch):
  monkeypatch.setattr(events_sp_module, 'IS_MICI', True)


class TestPreActivePrompt:
  @pytest.mark.parametrize("set_speed, text", [(40, "Press + for\n45 mph"), (55, "Press - for\n45 mph")])
  def test_names_the_target(self, mici, set_speed, text):
    assert _alert(EventNameSP.speedLimitPreActive, 45, set_speed).alert_text_1 == text

  def test_metric(self, mici):
    assert _alert(EventNameSP.speedLimitPreActive, 80, 100, metric=True).alert_text_1 == "Press - for\n80 km/h"

  def test_tizi_draws_the_sign_instead(self):
    assert _alert(EventNameSP.speedLimitPreActive, 45, 40).alert_size == AlertSize.none

  def test_pcm_long_asks_for_the_required_max(self, mici):
    assert _alert(EventNameSP.speedLimitPreActive, 45, 40, pcm_long=True).alert_text_1 == \
      "Speed Limit Assist: set to 70 mph to engage"


class TestActiveNotice:
  @pytest.mark.parametrize("target, set_speed, speed", [
    (45, 60, 45),  # down to the limit
    (65, 60, 60),  # a limit over the set speed settles at the set speed
  ])
  def test_names_the_speed_it_settles_at(self, target, set_speed, speed):
    alert = _alert(EventNameSP.speedLimitActive, target, set_speed)
    assert alert.alert_text_1 == f"Adjusting to {speed} mph"
    assert alert.alert_size == AlertSize.small

  def test_metric(self):
    assert _alert(EventNameSP.speedLimitActive, 80, 100, metric=True).alert_text_1 == "Adjusting to 80 km/h"

  def test_below_the_confirm_threshold_says_the_same(self):
    assert _alert(EventNameSP.speedLimitChanged, 35, 45).alert_text_1 == "Adjusting to 35 mph"

  def test_mici_keeps_the_unit_with_its_number(self, mici):
    assert _alert(EventNameSP.speedLimitActive, 45, 60).alert_text_1 == "Adjusting to\n45 mph"

  def test_unset_set_speed_falls_back_to_the_target(self):
    # vCruiseCluster 0 falls back to controlsState's vCruise, 0 when nothing published
    assert _alert(EventNameSP.speedLimitActive, 45, 0).alert_text_1 == "Adjusting to 45 mph"
