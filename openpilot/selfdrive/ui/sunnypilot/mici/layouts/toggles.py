"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.selfdrive.ui.mici.layouts.settings.toggles import TogglesLayoutMici


class TogglesLayoutMiciSP(TogglesLayoutMici):
  def __init__(self):
    super().__init__()
    # experimental mode lives in the alpha longitudinal panel
    self._scroller.items.remove(self._experimental_btn)
