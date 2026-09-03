"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.cereal import custom
from opendbc.car import structs
from opendbc.car.hyundai.values import HyundaiFlags
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.mads.mads import ModularAssistiveDrivingSystem
from openpilot.common.test import OpenpilotTestCase

EventNameSP = custom.OnroadEventSP.EventName
SafetyModel = structs.CarParams.SafetyModel


def make_car_state(available):
  cs = structs.CarState()
  cs.cruiseState.available = available
  return cs


def make_mads(mocker, brand, prev_available, flags=0, sp_flags=0):
  sd = mocker.MagicMock()
  sd.CP = structs.CarParams()
  sd.CP.brand = brand
  sd.CP.flags = int(flags)
  sd.CP_SP = structs.CarParamsSP()
  sd.CP_SP.flags = int(sp_flags)
  sd.params = mocker.MagicMock()
  sd.params.get_bool = mocker.MagicMock(side_effect=lambda k: {
    "Mads": True, "MadsMainCruiseAllowed": True,
    "DisengageOnAccelerator": True, "MadsUnifiedEngagementMode": True,
  }.get(k, False))
  sd.events = Events()
  sd.events_sp = EventsSP()
  sd.enabled = False
  sd.enabled_prev = False
  sd.initialized = True
  sd.CS_prev = make_car_state(prev_available)
  ps = mocker.MagicMock()
  ps.controlsAllowedLateral = True
  ps.safetyModel = SafetyModel.mazda
  sd.sm = {'pandaStates': [ps]}
  sd.state_machine = mocker.MagicMock()

  mads = ModularAssistiveDrivingSystem(sd)
  mads.enabled_toggle = True
  return mads, sd


class TestMainCruiseOffSwitch(OpenpilotTestCase):
  """On cars whose only MADS off-switch is the ACC main state, lateral must not be able to
  outlive it. A carstate that reports cruise enabled while main is off engages MADS with no
  falling edge behind it, which used to leave lateral on until ignition off (route 00000057)."""

  def _run(self, brand, prev_available, enabled, flags=0):
    mocker = self._fixture("mocker")
    mads, sd = make_mads(mocker, brand, prev_available, flags)
    mads.enabled = enabled
    mads.update_events(make_car_state(False))
    return sd.events_sp.has(EventNameSP.lkasDisable)

  def test_falling_edge_still_disables(self):
    assert self._run("mazda", prev_available=True, enabled=True)

  def test_enabled_with_availability_already_low_disables(self):
    assert self._run("mazda", prev_available=False, enabled=True)

  def test_disabled_mads_stays_quiet(self):
    assert not self._run("mazda", prev_available=False, enabled=False)

  def test_own_button_brands_keep_the_edge_only_behavior(self):
    # a hyundai with an LDA button engages MADS with main cruise off, so a low availability
    # level is a normal state there and must not disable it
    assert not self._run("hyundai", prev_available=False, enabled=True, flags=HyundaiFlags.HAS_LDA_BUTTON)
