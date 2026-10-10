"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.mici.layouts.settings.developer import DeveloperLayoutMici
from openpilot.selfdrive.ui.ui_state import ui_state


class DeveloperLayoutMiciSP(DeveloperLayoutMici):
  def __init__(self):
    super().__init__()
    # Jetlink holds the USB port that ADB needs
    self._adb_toggle.set_enabled(lambda: ui_state.is_offroad() and not ui_state.adb_blocked)
    # the alpha switch lives in the alpha longitudinal panel
    self._scroller.items.remove(self._alpha_long_toggle)
    self._long_maneuver_toggle.set_superseded(lambda: ui_state.CP is not None and not ui_state.has_longitudinal_control)
