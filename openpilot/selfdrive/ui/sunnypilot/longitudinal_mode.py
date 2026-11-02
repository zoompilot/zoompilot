"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The mici Cruise panel's Alpha Longitudinal entry.
"""
from openpilot.system.ui.lib.multilang import tr

# the e2e assists, each named by its badge
E2E_ASSISTS = (("ExperimentalModeSetSpeed", "speed assist"),)


def alpha_longitudinal_reachable(ui_state) -> bool:
  """Greyed only on a car known to have no openpilot longitudinal control, alpha or not."""
  CP = ui_state.CP
  return CP is None or ui_state.has_longitudinal_control or CP.alphaLongitudinalAvailable


def longitudinal_mode_labels(ui_state) -> list[str]:
  """The e2e assists that act, or "default" without one. Empty without openpilot longitudinal
  control. Each state fits one row under the two-line card title (test_cruise_entry_shows_the_mode)."""
  if not ui_state.has_longitudinal_control:
    return []
  # the driving model always drives the gas and brakes, so it shows as its assists
  assists = [tr(label) for param, label in E2E_ASSISTS if ui_state.params.get_bool(param)]
  return assists or [tr("default")]
