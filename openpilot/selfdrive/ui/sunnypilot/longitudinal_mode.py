"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Cruise's Alpha Longitudinal entry, shared by the tizi and mici layouts.
"""
from openpilot.system.ui.lib.multilang import tr


def alpha_longitudinal_reachable(ui_state) -> bool:
  """Greyed only on a car known to have no openpilot longitudinal control, alpha or not."""
  CP = ui_state.CP
  return CP is None or ui_state.has_longitudinal_control or CP.alphaLongitudinalAvailable


def longitudinal_mode_labels(ui_state) -> list[str]:
  """The mode that drives, then the set speed nudge where it acts. Empty without openpilot
  longitudinal control."""
  if not ui_state.has_longitudinal_control:
    return []
  if not ui_state.experimental_mode:
    return [tr("chill")]
  # DEC switches between chill and experimental itself, and the nudge never acts under it
  if ui_state.params.get_bool("DynamicExperimentalControl"):
    return [tr("experimental"), tr("dec")]
  return [tr("experimental")] + ([tr("nudge")] if ui_state.params.get_bool("ExperimentalModeSetSpeed") else [])
