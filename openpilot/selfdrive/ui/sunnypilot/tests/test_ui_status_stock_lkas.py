"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

MADS held by the car's own lane keep (mads.lateralHeld) reads as lateral off, not override.
"""
import os

os.environ.setdefault("SCALE", "1")  # the ui import probes the monitor for its scale otherwise

from openpilot.cereal import messaging, log, custom
from openpilot.selfdrive.ui.sunnypilot.ui_state import UIStateSP

MADSState = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState
OpenpilotState = log.SelfdriveState.OpenpilotState


def status(mads_state, selfdrive_enabled: bool, held: bool = False) -> str:
  ss = messaging.new_message('selfdriveState').selfdriveState
  ss.enabled = selfdrive_enabled
  ss.state = OpenpilotState.enabled if selfdrive_enabled else OpenpilotState.disabled
  ss_sp = messaging.new_message('selfdriveStateSP').selfdriveStateSP
  ss_sp.mads.available = True
  ss_sp.mads.state = mads_state
  ss_sp.mads.enabled = mads_state != MADSState.disabled
  ss_sp.mads.lateralHeld = held
  return UIStateSP.update_status(ss, ss_sp, [])


def test_held_reads_as_lateral_off():
  # LKA off (paused) and the EPS re-arm (enabled) alike
  for state in (MADSState.paused, MADSState.enabled):
    assert status(state, True, held=True) == "long_only"
    assert status(state, False, held=True) == "disengaged"


def test_other_pauses_still_read_as_override():
  assert status(MADSState.paused, True) == "override"
  assert status(MADSState.paused, False) == "override"


def test_unheld_is_untouched():
  assert status(MADSState.enabled, True) == "engaged"
  assert status(MADSState.enabled, False) == "lat_only"
  assert status(MADSState.disabled, True) == "long_only"
