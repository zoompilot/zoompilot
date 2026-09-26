"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Single source of truth for which torque tune version will run, shared by
# controlsd_ext and the settings UIs. Imports nothing heavy so UI code can use it.

import json
import math
import os

TORQUE_VERSIONS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "latcontrol_torque_versions.json")

# one tune per model size, keyed by modelV2.big
TUNE_PARAM_BY_SIZE = {False: "TorqueControlTune", True: "TorqueControlTuneBig"}
V2 = 2.0


def load_versions() -> dict:
  """Raw {label: {"version": str}} contents of latcontrol_torque_versions.json."""
  with open(TORQUE_VERSIONS_PATH) as f:
    return json.load(f)


def versions_by_label() -> dict[str, float]:
  """{label: version} for the selectors, ascending, entries that do not parse dropped;
  empty when the file is unreadable so a selector can fall back."""
  try:
    data = load_versions()
  except (OSError, ValueError):
    return {}
  versions: dict[str, float] = {}
  for label, info in data.items():
    try:
      versions[label] = float(info["version"])
    except (KeyError, TypeError, ValueError):
      continue
  return dict(sorted(versions.items(), key=lambda kv: kv[1]))


def label_for(version: float, versions: dict[str, float]) -> str | None:
  for label, v in versions.items():
    if math.isclose(v, version, rel_tol=1e-5):
      return label
  return None


def stored_tune_versions(params) -> dict[bool, float]:
  """What the selectors store per size, unset resolved through the declared defaults (v0
  small, v1 big): a bare params.get() returns None for an unset param, and float(None) raises."""
  return {big: float(params.get(key, return_default=True)) for big, key in TUNE_PARAM_BY_SIZE.items()}


def resolved_tune_versions(params, torque_lateral_tuning: bool = True) -> dict[bool, float | None]:
  """The tune version the controller will actually run per model size, or None for the
  upstream controller. With EnforceTorqueControl off, torque-tuned cars run v0 regardless
  of the stored tune (FIXME-SP: revert when upstream fixes tuning issues with v1), and
  non-torque cars run the upstream controller."""
  if not params.get_bool("EnforceTorqueControl"):
    return dict.fromkeys(TUNE_PARAM_BY_SIZE, 0.0 if torque_lateral_tuning else None)
  return stored_tune_versions(params)


def jerk_aware_has_effect(params) -> bool:
  """v2 replaces the jerk-aware mechanisms and forces that controller off, so the toggle
  only does something while some model size runs another tune."""
  return any(version != V2 for version in resolved_tune_versions(params).values())
