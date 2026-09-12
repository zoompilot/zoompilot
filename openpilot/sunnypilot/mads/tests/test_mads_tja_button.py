"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import custom, log
from opendbc.car import structs
from opendbc.sunnypilot.car.mazda.values import MazdaFlagsSP
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.mads.tests.test_mads_main_cruise_off_switch import make_mads

EventName = log.OnroadEvent.EventName
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


class TestMadsTjaButton(OpenpilotTestCase):
  """Mazda's physical TJA button is fitted to some trims only and the fingerprint cannot say,
  so the driver declares it. Declared, it is the only lateral switch: TJA changes MADS,
  MRCC changes cruise, neither touches the other. Undeclared cars keep the ACC-main path."""

  def _mads(self, tja=True, brand="mazda", prev_available=True):
    return make_mads(self._fixture("mocker"), brand, prev_available,
                     sp_flags=MazdaFlagsSP.TJA_BUTTON if tja else 0)[0]

  def test_declared_button_owns_lateral_from_start(self):
    mads = self._mads()
    assert mads.no_main_cruise
    assert mads.allow_always

  def test_undeclared_stays_on_the_acc_main_path(self):
    mads = self._mads(tja=False)
    assert not mads.no_main_cruise
    assert not mads.allow_always

  def test_other_brands_ignore_the_flag(self):
    mads = self._mads(brand="toyota")
    assert not mads.no_main_cruise
    assert not mads.allow_always

  def test_press_engages_without_acc_main(self):
    mads = self._mads()
    mads.update_events(car_state(False, lkas_pressed=True))
    assert mads.selfdrive.events_sp.has(EventNameSP.lkasEnable)

  def test_acc_main_arming_does_not_engage(self):
    mads = self._mads(prev_available=False)
    mads.update_events(car_state(True))
    assert not mads.selfdrive.events_sp.has(EventNameSP.lkasEnable)

  def test_acc_main_off_does_not_disengage(self):
    mads = self._mads()
    mads.enabled = True
    mads.update_events(car_state(False))
    assert not mads.selfdrive.events_sp.has(EventNameSP.lkasDisable)

  def test_acc_main_off_still_disengages_undeclared(self):
    mads = self._mads(tja=False)
    mads.enabled = True
    mads.update_events(car_state(False))
    assert mads.selfdrive.events_sp.has(EventNameSP.lkasDisable)

  def test_unified_engagement_does_not_enable(self):
    # TJA off, cancel, then SET/RES: the longitudinal enable must not pull MADS back on
    mads = self._mads()
    mads.selfdrive.events.add(EventName.pcmEnable)
    mads.update_events(car_state(True))
    assert not mads.selfdrive.events.has(EventName.pcmEnable)

  def test_unified_engagement_still_enables_undeclared(self):
    mads = self._mads(tja=False)
    mads.selfdrive.events.add(EventName.pcmEnable)
    mads.update_events(car_state(True))
    assert mads.selfdrive.events.has(EventName.pcmEnable)
