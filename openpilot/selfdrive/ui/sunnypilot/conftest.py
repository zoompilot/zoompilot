"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest


@pytest.fixture(scope="module")
def gui(request):
  """A hidden raylib window and an isolated params dir, once per module: widgets need
  textures. A module picks the big or the mici UI by setting BIG before its imports."""
  import pyray as rl
  from openpilot.common.prefix import OpenpilotPrefix

  with OpenpilotPrefix():
    rl.set_config_flags(rl.FLAG_WINDOW_HIDDEN)
    from openpilot.system.ui.lib.application import gui_app
    gui_app.init_window(request.module.__name__.rsplit(".", 1)[-1], fps=30)
    yield gui_app
    gui_app.close()


@pytest.fixture
def params(gui):
  """Isolated params for the UI, in the module's window (big or mici)."""
  from openpilot.common.params import Params
  from openpilot.selfdrive.ui.ui_state import ui_state

  p = Params()
  ui_state.params = p
  # on device update_params() runs every frame before anything draws, so attributes it
  # sets (always_offroad, screensaver_enabled, ...) exist by render time. Without this a
  # layout reading one of them fails in the test for a reason the device never sees.
  ui_state.update_params()
  return p
