"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Which cars show the longitudinal rows at the top of Cruise, on both UIs.
"""


def longitudinal_reachable(ui_state) -> bool:
  """openpilot longitudinal control is on or one alpha switch away. Before the first drive the car
  is unknown, so the rows show."""
  CP = ui_state.CP
  return CP is None or ui_state.has_longitudinal_control or CP.alphaLongitudinalAvailable
