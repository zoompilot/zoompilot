"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.layouts.settings.toggles import TogglesLayout


class TogglesLayoutSP(TogglesLayout):
  def __init__(self):
    super().__init__()
    # experimental mode lives in the Alpha Longitudinal panel; upstream never sets its visibility
    self._toggles["ExperimentalMode"].set_visible(False)
