"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import custom
from opendbc.car import structs
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.mads.tests.test_mads_main_cruise_off_switch import make_mads

EventNameSP = custom.OnroadEventSP.EventName
ButtonType = structs.CarState.ButtonEvent.Type


def car_state(available, lkas_pressed=False):
  cs = structs.CarState()
  cs.cruiseState.available = available
  if lkas_pressed:
    be = structs.CarState.ButtonEvent()
    be.type = ButtonType.lkas
    be.pressed = True
    cs.buttonEvents = [be]
  return cs


class TestLatchingMadsButton(OpenpilotTestCase):
  """Mazda's physical TJA button is fitted to some trims only, so the fingerprint cannot say
  whether it exists. Lateral follows ACC main until a press proves otherwise, which keeps
  trims without the button on the path they have today."""

  def _mads(self, brand="mazda", prev_available=True):
    return make_mads(self._fixture("mocker"), brand, prev_available)[0]

  def test_starts_on_the_acc_main_path(self):
    mads = self._mads()
    mads.update_events(car_state(True))
    assert not mads.no_main_cruise
    assert not mads.allow_always

  def test_a_press_hands_lateral_to_the_button(self):
    mads = self._mads()
    mads.update_events(car_state(True, lkas_pressed=True))
    assert mads.no_main_cruise
    assert mads.allow_always

  def test_acc_main_no_longer_disables_once_latched(self):
    mads = self._mads()
    mads.enabled = True
    mads.update_events(car_state(True, lkas_pressed=True))
    mads.selfdrive.events_sp.clear()
    mads.update_events(car_state(False))
    assert not mads.selfdrive.events_sp.has(EventNameSP.lkasDisable)

  def test_acc_main_still_disables_before_any_press(self):
    mads = self._mads()
    mads.enabled = True
    mads.update_events(car_state(False))
    assert mads.selfdrive.events_sp.has(EventNameSP.lkasDisable)

  def test_other_brands_do_not_latch(self):
    mads = self._mads(brand="toyota")
    mads.update_events(car_state(True, lkas_pressed=True))
    assert not mads.no_main_cruise
    assert not mads.allow_always
