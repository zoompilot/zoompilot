"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The mici Cruise panel's Alpha Longitudinal entry.
"""
from openpilot.system.ui.lib.multilang import tr

# mici's experimental mode icon; upstream retired its copy with chill mode
EXPERIMENTAL_ICON = "../../sunnypilot/selfdrive/assets/icons_mici/experimental_mode.png"

# the e2e assists, each named by its badge
E2E_ASSISTS = (("ExperimentalModeSetSpeed", "speed assist"),)


def alpha_longitudinal_reachable(ui_state) -> bool:
  """Greyed only on a car known to have no openpilot longitudinal control, alpha or not."""
  CP = ui_state.CP
  return CP is None or ui_state.has_longitudinal_control or CP.alphaLongitudinalAvailable


def longitudinal_mode_labels(ui_state) -> list[str]:
  """Legacy chill, or the e2e assists that act ("default" without one). Empty without openpilot
  longitudinal control. Each state fits one row under the two-line card title
  (test_cruise_entry_shows_the_mode)."""
  if not ui_state.has_longitudinal_control:
    return []
  if not ui_state.experimental_mode:
    return [tr("legacy chill")]
  # the driving model is the default, so it shows as its assists, or as "default" without one
  assists = [tr(label) for param, label in E2E_ASSISTS if ui_state.params.get_bool(param)]
  return assists or [tr("default")]
