"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The mici Cruise panel's Alpha Longitudinal entry.
"""
from openpilot.system.ui.lib.multilang import tr


def alpha_longitudinal_reachable(ui_state) -> bool:
  """Greyed only on a car known to have no openpilot longitudinal control, alpha or not."""
  CP = ui_state.CP
  return CP is None or ui_state.has_longitudinal_control or CP.alphaLongitudinalAvailable


def longitudinal_mode_labels(ui_state) -> list[str]:
  """The mode that drives, then speed assist where it acts. Empty without openpilot
  longitudinal control."""
  if not ui_state.has_longitudinal_control:
    return []
  if not ui_state.experimental_mode:
    return [tr("chill")]
  # beside a second badge experimental is "exp.", so both fit one row under the two-line card title.
  # DEC switches between chill and experimental itself, and speed assist never acts under it
  if ui_state.params.get_bool("DynamicExperimentalControl"):
    return [tr("exp."), tr("dynamic")]
  if ui_state.params.get_bool("ExperimentalModeSetSpeed"):
    return [tr("exp."), tr("speed assist")]
  return [tr("experimental")]
