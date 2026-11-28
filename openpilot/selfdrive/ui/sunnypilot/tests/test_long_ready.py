"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Until openpilot longitudinal owns the stock ECU it stands in for (Mazda's radar), the onroad UI
# is the lateral-only one: the speed dial instead of the car and distance bars, no lead chevron.

import pytest

import openpilot.cereal.messaging as messaging
from openpilot.selfdrive.ui.sunnypilot.onroad.model_renderer import ModelRendererSP
from openpilot.selfdrive.ui.ui_state import ui_state


def _set(monkeypatch, *, state="notNeeded", has_long=True, started=True, received=True):
  cs_sp = messaging.new_message('carStateSP').carStateSP
  cs_sp.zoompilot.stockEcu = state
  monkeypatch.setattr(ui_state, 'has_longitudinal_control', has_long)
  monkeypatch.setattr(ui_state, 'started', started)
  monkeypatch.setattr(ui_state, 'sm', {'carStateSP': cs_sp})
  monkeypatch.setattr(ui_state, 'received_since_started', lambda service: received)


@pytest.mark.parametrize("state, ready", [
  ("notNeeded", True), ("ready", True),
  ("starting", False), ("parkToTakeOver", False), ("stockCruiseOn", False),
  ("restoring", False), ("restored", False), ("failed", False),
])
def test_long_ready_follows_the_stock_ecu(monkeypatch, state, ready):
  _set(monkeypatch, state=state)
  assert ui_state.long_ready == ready


def test_not_before_cards_first_report(monkeypatch):
  _set(monkeypatch, received=False)
  assert not ui_state.long_ready


def test_offroad_follows_the_setting(monkeypatch):
  _set(monkeypatch, state="starting", started=False, received=False)
  assert ui_state.long_ready
  _set(monkeypatch, started=False, has_long=False)
  assert not ui_state.long_ready


def test_never_without_longitudinal(monkeypatch):
  _set(monkeypatch, has_long=False)
  assert not ui_state.long_ready


def test_quick_toggle_waits_for_the_takeover(monkeypatch):
  switched = []
  monkeypatch.setattr(ui_state, 'set_speed_assist', switched.append)
  monkeypatch.setattr(ui_state, 'speed_assist', False)
  _set(monkeypatch, state="starting")
  ui_state.toggle_speed_assist()
  assert switched == []
  _set(monkeypatch, state="ready")
  ui_state.toggle_speed_assist()
  assert switched == [True]


@pytest.mark.parametrize("state, drawn", [("ready", True), ("starting", False)])
def test_model_renderer_draws_longitudinal_only_once_owned(monkeypatch, state, drawn):
  _set(monkeypatch, state=state)
  renderer = ModelRendererSP.__new__(ModelRendererSP)
  renderer._longitudinal_control = True  # as upstream assigns it from carParams
  assert renderer._longitudinal_control == drawn
  renderer._longitudinal_control = False
  assert not renderer._longitudinal_control
