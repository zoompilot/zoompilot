"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import custom, log
from opendbc.car import structs
from opendbc.sunnypilot.car.mazda.values import MazdaFlagsSP
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.selfdrived.events import ET
from openpilot.sunnypilot.selfdrive.selfdrived.events import AudibleAlert
from openpilot.sunnypilot.mads.state import State
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


class TestMadsTjaButtonChimes(OpenpilotTestCase):
  """The longitudinal strips in mads.update_events land after the standard machine's
  transition but before alert creation, so with the button owning lateral the cruise
  engage and disable chimes die with the stripped events. The transitions are mirrored
  as sound-only PERMANENT events, and an explicit lateral enable keeps its engage chime
  while cruise is already active."""

  def _mads(self, tja=True, alpha_long=False):
    mads, sd = make_mads(self._fixture("mocker"), "mazda", True,
                         sp_flags=MazdaFlagsSP.TJA_BUTTON if tja else 0)
    if alpha_long:
      sd.CP.alphaLongitudinalAvailable = True
      sd.CP.openpilotLongitudinalControl = True
    return mads, sd

  def _button_state(self, btn, available=True):
    cs = car_state(available)
    be = structs.CarState.ButtonEvent()
    be.type = btn
    be.pressed = True
    cs.buttonEvents = [be]
    return cs

  def test_event_ordinals(self):
    assert int(EventNameSP.longitudinalEnableChime) == 35
    assert int(EventNameSP.longitudinalDisableChime) == 36

  def test_longitudinal_chimes_do_not_toggle_mads(self):
    mads, sd = self._mads()
    mads.enabled = False

    sd.enabled_prev = False
    sd.enabled = True
    mads.update_events(car_state(True))
    assert not mads.enabled
    assert sd.events_sp.has(EventNameSP.longitudinalEnableChime)
    enable_alerts = sd.events_sp.create_alerts([ET.PERMANENT])
    assert any(a.alert_type == "longitudinalEnableChime/permanent" and a.audible_alert == AudibleAlert.engage
               for a in enable_alerts)

    sd.events_sp.clear()
    sd.enabled_prev = True
    sd.enabled = False
    mads.update_events(car_state(True))
    assert not mads.enabled
    assert sd.events_sp.has(EventNameSP.longitudinalDisableChime)
    disable_alerts = sd.events_sp.create_alerts([ET.PERMANENT])
    assert any(a.alert_type == "longitudinalDisableChime/permanent" and a.audible_alert == AudibleAlert.disengage
               for a in disable_alerts)

  def test_longitudinal_chimes_preserve_enabled_mads(self):
    mads, sd = self._mads()
    mads.enabled = True

    sd.enabled_prev = False
    sd.enabled = True
    mads.update_events(car_state(True))
    assert mads.enabled
    assert sd.events_sp.has(EventNameSP.longitudinalEnableChime)

    sd.events_sp.clear()
    sd.enabled_prev = True
    sd.enabled = False
    mads.update_events(car_state(True))
    assert mads.enabled
    assert sd.events_sp.has(EventNameSP.longitudinalDisableChime)

  def test_undeclared_skips_longitudinal_chimes(self):
    mads, sd = self._mads(tja=False)
    sd.enabled_prev = False
    sd.enabled = True
    mads.update_events(car_state(True))
    assert not sd.events_sp.has(EventNameSP.longitudinalEnableChime)

  def test_mads_enable_chime_while_longitudinal_active(self):
    mads, sd = self._mads()
    sd.enabled = True
    mads.state_machine.state = State.disabled
    sd.state_machine.current_alert_types = []
    sd.events_sp.add(EventNameSP.lkasEnable)

    mads.state_machine.update()

    assert mads.state_machine.state == State.enabled
    assert ET.ENABLE in sd.state_machine.current_alert_types
    enable_alerts = sd.events_sp.create_alerts(sd.state_machine.current_alert_types)
    assert any(a.alert_type == "lkasEnable/enable" and a.audible_alert == AudibleAlert.engage
               for a in enable_alerts)

  def test_mads_disable_chime_while_longitudinal_active(self):
    mads, sd = self._mads()
    sd.enabled = True
    mads.state_machine.state = State.enabled
    mads.enabled = True
    sd.state_machine.current_alert_types = []
    sd.events_sp.add(EventNameSP.manualSteeringRequired)

    mads.state_machine.update()

    assert mads.state_machine.state == State.disabled
    assert ET.USER_DISABLE in sd.state_machine.current_alert_types
    disable_alerts = sd.events_sp.create_alerts(sd.state_machine.current_alert_types)
    assert any(a.alert_type == "manualSteeringRequired/userDisable" and a.audible_alert == AudibleAlert.disengage
               for a in disable_alerts)

  def test_set_res_cancel_do_not_toggle_mads_when_button_owns(self):
    mads, sd = self._mads()
    mads.enabled = True
    for btn in (ButtonType.setCruise, ButtonType.resumeCruise, ButtonType.cancel,
                ButtonType.accelCruise, ButtonType.decelCruise):
      mads.update_events(self._button_state(btn))
      assert mads.enabled
      assert not sd.events_sp.has(EventNameSP.lkasDisable)
      assert not sd.events_sp.has(EventNameSP.lkasEnable)

  def test_alpha_long_chimes_do_not_toggle_mads(self):
    mads, sd = self._mads(alpha_long=True)
    assert mads.button_owns_lateral
    mads.enabled = False
    mads.state_machine.state = State.disabled

    sd.enabled_prev = False
    sd.enabled = True
    mads.update(car_state(True))
    assert not mads.enabled
    assert sd.events_sp.has(EventNameSP.longitudinalEnableChime)

    # Level hold: already active longitudinal must not re-chime
    sd.events_sp.clear()
    mads.update(car_state(True))
    assert not mads.enabled
    assert not sd.events_sp.has(EventNameSP.longitudinalEnableChime)
    assert not sd.events_sp.has(EventNameSP.longitudinalDisableChime)

    sd.enabled = False
    mads.update(car_state(True))
    assert not mads.enabled
    assert sd.events_sp.has(EventNameSP.longitudinalDisableChime)

  def test_alpha_long_chimes_preserve_enabled_mads(self):
    mads, sd = self._mads(alpha_long=True)
    mads.enabled = True
    mads.state_machine.state = State.enabled

    sd.enabled_prev = False
    sd.enabled = True
    mads.update(car_state(True))
    assert mads.enabled
    assert sd.events_sp.has(EventNameSP.longitudinalEnableChime)

    sd.events_sp.clear()
    sd.enabled = False
    mads.update(car_state(True))
    assert mads.enabled
    assert sd.events_sp.has(EventNameSP.longitudinalDisableChime)

  def test_mrcc_available_only_no_longitudinal_chime(self):
    # Chimes key off selfdrive.enabled edges, not cruiseState.available / MRCC armed
    mads, sd = self._mads()
    mads.enabled = False
    sd.enabled = False
    sd.enabled_prev = False
    sd.CS_prev = car_state(False)

    mads.update_events(car_state(True))
    assert not sd.events_sp.has(EventNameSP.longitudinalEnableChime)

    sd.events_sp.clear()
    sd.CS_prev = car_state(True)
    mads.update_events(car_state(True))
    assert not sd.events_sp.has(EventNameSP.longitudinalEnableChime)
    assert not sd.events_sp.has(EventNameSP.longitudinalDisableChime)

  def test_set_res_while_long_active_no_extra_chime(self):
    # ON->ON: SET/RES/speed adjust must not emit another longitudinal engage chime
    mads, sd = self._mads()
    mads.enabled = True
    sd.enabled = True
    sd.enabled_prev = True
    for btn in (ButtonType.setCruise, ButtonType.resumeCruise,
                ButtonType.accelCruise, ButtonType.decelCruise):
      sd.events_sp.clear()
      mads.update_events(self._button_state(btn))
      assert not sd.events_sp.has(EventNameSP.longitudinalEnableChime)
      assert not sd.events_sp.has(EventNameSP.longitudinalDisableChime)
