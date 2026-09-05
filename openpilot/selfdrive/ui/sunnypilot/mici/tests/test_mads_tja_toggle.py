"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# The Mazda TJA button toggle is brand-gated on both device UIs.

import os

os.environ["BIG"] = "0"
os.environ.setdefault("SCALE", "1")

from openpilot.selfdrive.ui.sunnypilot.mici.tests.test_mici_settings import gui, params, render  # noqa: F401


def _bundle(p, brand):
  p.put("CarPlatformBundle", {"brand": brand, "platform": "X", "name": "X"})


class TestMadsTjaToggle:
  def test_mici_visible_only_for_mazda(self, params):  # noqa: F811
    from openpilot.selfdrive.ui.ui_state import ui_state
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import SteeringLayoutMici
    ui_state.CP = None
    layout = SteeringLayoutMici()
    _bundle(params, "toyota")
    render(layout)
    render(layout._mads_view)
    assert not layout._mads_tja.is_visible
    _bundle(params, "mazda")
    render(layout)
    render(layout._mads_view)
    assert layout._mads_tja.is_visible

  def test_tici_visible_only_for_mazda(self, params):  # noqa: F811
    from openpilot.selfdrive.ui.ui_state import ui_state
    from openpilot.selfdrive.ui.sunnypilot.layouts.settings.steering_sub_layouts.mads_settings import MadsSettingsLayout
    ui_state.CP = None
    layout = MadsSettingsLayout(lambda: None)
    _bundle(params, "toyota")
    render(layout)
    assert not layout._tja_button_toggle.is_visible
    _bundle(params, "mazda")
    render(layout)
    assert layout._tja_button_toggle.is_visible
