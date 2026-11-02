"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Shared builders for the fork's MADS tests: a real ModularAssistiveDrivingSystem on a mocked
selfdrived, and the CarState it reads.
"""
from opendbc.car import structs
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.mads.mads import ModularAssistiveDrivingSystem
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

SafetyModel = structs.CarParams.SafetyModel


def car_state(available, button=None):
  """ACC main at `available`, with one pressed ButtonEvent of type `button` if given."""
  cs = structs.CarState()
  cs.cruiseState.available = available
  if button is not None:
    be = structs.CarState.ButtonEvent()
    be.type = button
    be.pressed = True
    cs.buttonEvents = [be]
  return cs


def make_mads(mocker, brand, prev_available, flags=0, sp_flags=0):
  """MADS enabled with ACC main and unified engagement allowed, on a panda that allows lateral."""
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
  sd.CS_prev = car_state(prev_available)
  ps = mocker.MagicMock()
  ps.controlsAllowedLateral = True
  ps.safetyModel = SafetyModel.mazda
  sd.sm = {'pandaStates': [ps]}
  sd.state_machine = mocker.MagicMock()

  mads = ModularAssistiveDrivingSystem(sd)
  mads.enabled_toggle = True
  return mads, sd
