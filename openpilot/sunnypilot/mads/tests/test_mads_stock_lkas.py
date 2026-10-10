"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs
from openpilot.cereal import custom
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.mads.helpers import MadsSteeringModeOnBrake
from openpilot.sunnypilot.mads.state import State
from openpilot.sunnypilot.mads.tests.mads_harness import car_state, make_mads

EventNameSP = custom.OnroadEventSP.EventName
ButtonType = structs.CarState.ButtonEvent.Type


class TestMadsStockLkas(OpenpilotTestCase):
  """The car's own lane keep switched off holds lateral paused (mads.py update_stock_lkas)."""

  def setup_method(self):
    self.mads, self.sd = make_mads(self._fixture("mocker"), "mazda", True)
    self.mads.steering_mode_on_brake = MadsSteeringModeOnBrake.REMAIN_ACTIVE

  def frame(self, cs=None, lka_off=False, arming=False, selfdrive_enabled=False):
    sd = self.sd
    sd.events.clear()
    sd.events_sp.clear()
    sd.enabled = selfdrive_enabled
    if lka_off:
      sd.events_sp.add(EventNameSP.stockLkasOff)
    if arming:
      sd.events_sp.add(EventNameSP.stockLkasArming)
    self.mads.update(cs if cs is not None else car_state(True))
    return self.mads.state_machine.state

  def engage(self):
    self.mads.state_machine.state = State.enabled
    self.mads.enabled, self.mads.active = True, True

  def test_lka_off_pauses_and_keeps_mads_enabled(self):
    for selfdrive_enabled in (False, True):
      self.engage()
      assert self.frame(lka_off=True, selfdrive_enabled=selfdrive_enabled) == State.paused
      assert self.mads.enabled, "the panda drops lateral on the MADS heartbeat if this goes false"
      assert not self.mads.active

  def test_paused_holds_while_lka_is_off(self):
    self.engage()
    for _ in range(50):
      assert self.frame(lka_off=True) == State.paused
      assert not self.sd.events_sp.has(EventNameSP.silentLkasEnable)

  def test_lka_on_resumes_without_a_cruise_cycle(self):
    self.engage()
    self.frame(lka_off=True)
    self.frame(lka_off=True)
    assert self.frame() == State.enabled
    assert self.mads.active

  def test_lka_on_resumes_under_engaged_cruise(self):
    self.engage()
    self.frame(lka_off=True, selfdrive_enabled=True)
    assert self.frame(selfdrive_enabled=True) == State.enabled

  def test_mads_button_while_lka_off_disables_for_good(self):
    self.engage()
    self.frame(lka_off=True)
    assert self.frame(car_state(True, button=ButtonType.lkas), lka_off=True) == State.disabled
    assert self.frame() == State.disabled

  def test_button_and_lka_off_on_one_frame_disable(self):
    self.engage()
    assert self.frame(car_state(True, button=ButtonType.lkas), lka_off=True) == State.disabled

  def test_main_off_while_lka_off_disables(self):
    self.engage()
    self.frame(lka_off=True)
    assert self.frame(car_state(False), lka_off=True) == State.disabled

  def test_request_while_lka_off_waits_in_paused(self):
    # the button press arms the panda too; lateral comes on when LKA does
    assert self.frame(car_state(True, button=ButtonType.lkas), lka_off=True) == State.paused
    assert self.frame(lka_off=True) == State.paused
    assert self.frame() == State.enabled

  def test_disabled_mads_stays_disabled_through_lka(self):
    assert self.frame(lka_off=True) == State.disabled
    assert self.frame() == State.disabled

  def test_brake_under_pause_holds_the_resume(self):
    self.mads.steering_mode_on_brake = MadsSteeringModeOnBrake.PAUSE
    self.engage()
    self.frame(lka_off=True)
    braking = car_state(True)
    braking.brakePressed = True
    assert self.frame(braking) == State.paused
    assert self.frame() == State.enabled

  def test_lateral_held_through_lka_off_and_the_eps_rearm(self):
    self.engage()
    self.frame(lka_off=True)
    assert self.mads.lateral_held
    self.frame(arming=True)
    assert self.mads.active and self.mads.lateral_held
    self.frame()
    assert not self.mads.lateral_held

  def test_a_brake_pause_is_not_held(self):
    self.mads.steering_mode_on_brake = MadsSteeringModeOnBrake.PAUSE
    self.mads.state_machine.state = State.paused
    self.mads.enabled = True
    braking = car_state(True)
    braking.brakePressed = True
    assert self.frame(braking) == State.paused
    assert not self.mads.lateral_held
