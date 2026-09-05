"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Single source of truth for which torque tune version will run, shared by
# controlsd_ext and the settings UIs. Imports nothing heavy so UI code can use it.

import json
import os

TORQUE_VERSIONS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "latcontrol_torque_versions.json")


def load_versions() -> dict:
  """Raw {label: {"version": str}} contents of latcontrol_torque_versions.json."""
  with open(TORQUE_VERSIONS_PATH) as f:
    return json.load(f)


def resolved_tune_version(params, torque_lateral_tuning: bool = True) -> float | None:
  """The tune version the controller will actually run, or None for the upstream
  controller. Encodes two rules that are easy to re-implement wrong:

  - With EnforceTorqueControl off, torque-tuned cars run v0 regardless of the stored
    TorqueControlTune (FIXME-SP: revert when upstream fixes tuning issues with v1),
    and non-torque cars run the upstream controller.
  - An unset TorqueControlTune must resolve through the declared param default (2.0):
    a bare params.get() returns None for an unset param, and float(None) raises.
  """
  if not params.get_bool("EnforceTorqueControl"):
    return 0.0 if torque_lateral_tuning else None
  return float(params.get("TorqueControlTune", return_default=True))
