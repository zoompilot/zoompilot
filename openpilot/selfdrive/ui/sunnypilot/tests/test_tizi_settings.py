"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The tizi (comma three / 3X) developer page: the alpha-long switch is offroad-only, with no
status line. Its own module: the big UI is chosen at import, so it cannot share a window with
the mici tests.
"""
import os

import pytest

os.environ["BIG"] = "1"
os.environ.setdefault("SCALE", "0.5")  # a full-size 2160x1080 hidden window aborts on macOS


@pytest.fixture(scope="module")
def gui():
  import pyray as rl
  from openpilot.common.prefix import OpenpilotPrefix

  with OpenpilotPrefix():
    rl.set_config_flags(rl.FLAG_WINDOW_HIDDEN)
    from openpilot.system.ui.lib.application import gui_app
    gui_app.init_window("test_tizi_settings", fps=30)
    yield gui_app
    gui_app.close()


def test_alpha_toggle_is_offroad_only(gui, monkeypatch):
  from openpilot.common.params import Params
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.developer import DeveloperLayoutSP
  from openpilot.selfdrive.ui.ui_state import ui_state
  ui_state.params = Params()
  ui_state.update_params()
  layout = DeveloperLayoutSP()
  monkeypatch.setattr(ui_state, "started", True)
  assert not layout._alpha_long_toggle.action_item.enabled
  monkeypatch.setattr(ui_state, "started", False)
  assert layout._alpha_long_toggle.action_item.enabled
