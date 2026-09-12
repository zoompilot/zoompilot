"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

A SET/RES press before openpilot owns the stock ECU it stands in for is answered by an alert
in the shape of upstream's no-entry conditions, keyed on the published state, whatever
selfdrived and MADS are doing. Route 0000021b (2026-09-11): three minutes of presses under
parkToTakeOver with lateral off, nothing shown.
"""
import pytest

from openpilot.cereal import custom, messaging
from opendbc.car import DT_CTRL, structs
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.selfdrive.selfdrived.state import StateMachine
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP
from openpilot.sunnypilot.selfdrive.selfdrived.events import EVENTS_SP
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import ET, AudibleAlert

EventNameSP = custom.OnroadEventSP.EventName
ButtonType = structs.CarState.ButtonEvent.Type


def _car_events(brand: str = 'mazda') -> CarSpecificEventsSP:
  CP = structs.CarParams()
  CP.brand = brand
  return CarSpecificEventsSP(CP, structs.CarParamsSP())


def _update(car_events: CarSpecificEventsSP, state: str, button=None, pressed: bool = True, standstill: bool = False):
  CS = structs.CarState()
  CS.standstill = standstill
  if button is not None:
    CS.buttonEvents = [structs.CarState.ButtonEvent(type=button, pressed=pressed)]
  CS_SP = messaging.new_message('carStateSP').carStateSP
  CS_SP.zoompilot.stockEcu = state
  return car_events.update(CS, Events(), CS_SP)


def _events_for(state: str, button=ButtonType.accelCruise, pressed: bool = True, brand: str = 'mazda'):
  return _update(_car_events(brand), state, button, pressed)


def _car_state_sp(state: str):
  CS_SP = messaging.new_message('carStateSP').carStateSP
  CS_SP.zoompilot.stockEcu = state
  return CS_SP


def _alert(state: str, standstill: bool):
  CS = structs.CarState()
  CS.standstill = standstill
  alert = EVENTS_SP[EventNameSP.stockEcuNotReady][ET.PERMANENT](structs.CarParams(), CS, {'carStateSP': _car_state_sp(state)}, False, 0, None)
  return alert, {alert.alert_text_1, alert.alert_text_2}  # mici swaps the two lines


class TestStockEcuNotReady:
  @pytest.mark.parametrize("state", ["starting", "parkToTakeOver", "stockCruiseOn", "restoring", "failed"])
  def test_a_set_press_before_ownership_raises_it(self, state):
    assert _events_for(state).has(EventNameSP.stockEcuNotReady)

  @pytest.mark.parametrize("state", ["notNeeded", "ready", "restored"])
  def test_an_owned_or_stock_body_engages_without_comment(self, state):
    assert not _events_for(state).has(EventNameSP.stockEcuNotReady)

  def test_only_set_and_resume_presses_count(self):
    assert not _events_for("parkToTakeOver", button=ButtonType.mainCruise).has(EventNameSP.stockEcuNotReady)
    assert not _events_for("parkToTakeOver", button=ButtonType.cancel).has(EventNameSP.stockEcuNotReady)
    assert _events_for("parkToTakeOver", button=ButtonType.resumeCruise).has(EventNameSP.stockEcuNotReady)
    assert not _events_for("parkToTakeOver", pressed=False).has(EventNameSP.stockEcuNotReady)

  def test_brand_independent(self):
    assert _events_for("starting", brand='honda').has(EventNameSP.stockEcuNotReady)

  def test_shows_with_nothing_engaged(self):
    # selfdrived disabled, MADS not active: the state a driver presses SET in before the
    # takeover, and the one the *AlertOnly WARNING convention never reaches
    events_sp = _events_for("parkToTakeOver")
    machine = StateMachine()
    machine.update(Events())
    args = [structs.CarParams(), structs.CarState(), {'carStateSP': _car_state_sp("parkToTakeOver")}, False, 0, None]
    alerts = events_sp.create_alerts(machine.current_alert_types, args)
    assert [a.alert_type for a in alerts] == ["stockEcuNotReady/permanent"]

  def test_the_no_entry_shape(self):
    alert, _ = _alert("parkToTakeOver", standstill=False)
    assert alert.audible_alert == AudibleAlert.refuse
    assert alert.duration == int(3. / DT_CTRL)  # frames

  @pytest.mark.parametrize("state, standstill, expected", [
    ("starting", True, {"Longitudinal Initializing", "Remain parked"}),
    ("starting", False, {"Longitudinal Initializing", "Wait for the radar takeover"}),
    ("parkToTakeOver", False, {"Park to Engage Longitudinal", "Alpha longitudinal takes over at the next stop"}),
    ("stockCruiseOn", False, {"Turn Off Stock Cruise", "Alpha longitudinal waits for it"}),
    ("restoring", False, {"Longitudinal Handing Back", "Stock cruise returns when it completes"}),
    ("failed", True, {"Longitudinal Initializing Failed", "Restart the car to retry"}),
  ])
  def test_text_names_what_the_driver_has_to_do(self, state, standstill, expected):
    assert _alert(state, standstill)[1] == expected


class TestStockEcuUnprompted:
  """Two alerts need no press: parked with the takeover still starting (waiting helps; route
  0000021b pulled away 5 s short of it), and the edge into ready."""

  def test_parked_and_starting_shows_the_line_every_frame(self):
    car_events = _car_events()
    for _ in range(3):
      events_sp = _update(car_events, "starting", standstill=True)
      assert events_sp.has(EventNameSP.stockEcuInitializing)
      assert not events_sp.has(EventNameSP.stockEcuNotReady)  # no press, no press alert

  def test_moving_clears_it(self):
    car_events = _car_events()
    assert _update(car_events, "starting", standstill=True).has(EventNameSP.stockEcuInitializing)
    assert not _update(car_events, "starting", standstill=False).has(EventNameSP.stockEcuInitializing)

  @pytest.mark.parametrize("state", ["notNeeded", "parkToTakeOver", "stockCruiseOn", "ready", "restoring", "restored", "failed"])
  def test_only_the_starting_state(self, state):
    assert not _update(_car_events(), state, standstill=True).has(EventNameSP.stockEcuInitializing)

  def test_ready_edge_fires_once(self):
    car_events = _car_events()
    assert not _update(car_events, "starting", standstill=True).has(EventNameSP.stockEcuReady)
    assert _update(car_events, "ready").has(EventNameSP.stockEcuReady)
    assert not _update(car_events, "ready").has(EventNameSP.stockEcuReady)
    # a returned radar (accFaulted) and a fresh ownership: told again
    _update(car_events, "starting")
    assert _update(car_events, "ready").has(EventNameSP.stockEcuReady)

  def test_boot_straight_into_ready_is_still_told(self):
    # first frame of a session already owned (comma restart onroad adopting a quiet radar)
    assert _update(_car_events(), "ready").has(EventNameSP.stockEcuReady)

  def test_shape(self):
    initializing = EVENTS_SP[EventNameSP.stockEcuInitializing][ET.PERMANENT]
    ready = EVENTS_SP[EventNameSP.stockEcuReady][ET.PERMANENT]
    for alert in (initializing, ready):
      assert alert.audible_alert == AudibleAlert.none
    assert {initializing.alert_text_1, initializing.alert_text_2} == {"Longitudinal Initializing", "Remain parked"}
    assert ready.alert_text_1 == "Alpha Longitudinal Ready"
    assert ready.duration == int(2. / DT_CTRL)

  def test_both_show_with_nothing_engaged(self):
    car_events = _car_events()
    _update(car_events, "starting", standstill=True)
    machine = StateMachine()
    machine.update(Events())
    args = [structs.CarParams(), structs.CarState(), {'carStateSP': _car_state_sp("ready")}, False, 0, None]
    shown = lambda events_sp: [a.alert_type for a in events_sp.create_alerts(machine.current_alert_types, args)]  # noqa: E731
    assert shown(_update(car_events, "starting", standstill=True)) == ["stockEcuInitializing/permanent"]
    assert shown(_update(car_events, "ready")) == ["stockEcuReady/permanent"]
