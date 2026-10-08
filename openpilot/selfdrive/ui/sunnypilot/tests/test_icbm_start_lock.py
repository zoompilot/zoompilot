"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Engaged with ICBM off, the settings lock turning on a feature that needs ICBM: card would
# only bring ICBM up at the next engage.

from types import SimpleNamespace

import pytest

import openpilot.cereal.messaging as messaging
from opendbc.car import structs
from openpilot.cereal import custom
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.icbm_latch import IcbmActivation


def _locked(monkeypatch, *, has_icbm=True, started=True, op_engaged=False, cruise_engaged=False, activation=IcbmActivation.passive):
  cs_sp = messaging.new_message('carStateSP')
  cs_sp.carStateSP.zoompilot.icbmActivation = activation
  monkeypatch.setattr(ui_state, 'CP_SP', custom.CarParamsSP.new_message(pcmCruiseSpeed=True))
  monkeypatch.setattr(ui_state, 'has_icbm', has_icbm)
  monkeypatch.setattr(ui_state, 'started', started)
  monkeypatch.setattr(ui_state, 'sm', {
    'selfdriveState': SimpleNamespace(enabled=op_engaged),
    'carState': SimpleNamespace(cruiseState=SimpleNamespace(enabled=cruise_engaged)),
    'carStateSP': cs_sp.carStateSP,
  })
  ui_state._update_icbm_start_lock()
  return ui_state.icbm_start_locked


class TestIcbmStartLock:
  @pytest.mark.parametrize("op_engaged, cruise_engaged", [(True, False), (False, True), (True, True)])
  def test_engaged_with_icbm_off_locks(self, monkeypatch, op_engaged, cruise_engaged):
    assert _locked(monkeypatch, op_engaged=op_engaged, cruise_engaged=cruise_engaged)

  def test_disengaged_never_locks(self, monkeypatch):
    # card brings ICBM up on the next frame
    assert not _locked(monkeypatch)

  def test_icbm_already_running_never_locks(self, monkeypatch):
    assert not _locked(monkeypatch, op_engaged=True, activation=IcbmActivation.active)

  def test_car_without_the_buttons_never_locks(self, monkeypatch):
    assert not _locked(monkeypatch, op_engaged=True, has_icbm=False)

  def test_offroad_never_locks(self, monkeypatch):
    assert not _locked(monkeypatch, op_engaged=True, started=False)

  @pytest.mark.parametrize("op_long, key, on, allowed", [
    (False, "SmartCruiseControlVision", True, False),
    (False, "SmartCruiseControlVision", False, True),  # turning off is always allowed
    (True, "SmartCruiseControlVision", True, True),    # the planner runs curve control under op long
    (True, "CustomAccIncrementsEnabled", True, False),
    (True, "SpeedLimitMode", True, False),
    (False, "IsMetric", True, True),                   # not an ICBM feature
  ])
  def test_turn_on_allowed(self, monkeypatch, op_long, key, on, allowed):
    assert _locked(monkeypatch, op_engaged=True)
    monkeypatch.setattr(ui_state, 'CP', structs.CarParams(openpilotLongitudinalControl=op_long, pcmCruise=True))
    assert ui_state.icbm_turn_on_allowed(key, on) == allowed
