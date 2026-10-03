"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest


@pytest.fixture
def params(gui):
  """Isolated params for the UI. Each test module brings its own `gui` window (big or mici)."""
  from openpilot.common.params import Params
  from openpilot.selfdrive.ui.ui_state import ui_state

  p = Params()
  ui_state.params = p
  # on device update_params() runs every frame before anything draws, so attributes it
  # sets (always_offroad, screensaver_enabled, ...) exist by render time. Without this a
  # layout reading one of them fails in the test for a reason the device never sees.
  ui_state.update_params()
  return p
