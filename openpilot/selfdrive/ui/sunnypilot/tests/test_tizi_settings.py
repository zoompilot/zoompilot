"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The tizi (comma three / 3X) settings: the top of Cruise owns the alpha switch
(offroad-only, no status line) and experimental mode, which Developer and Toggles no longer
show. Its own module: the big UI is chosen at import, so it cannot share a window with the mici
tests.
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


def test_alpha_toggle_is_offroad_only(params, monkeypatch):
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.alpha_longitudinal_toggles import AlphaLongitudinalToggles
  from openpilot.selfdrive.ui.ui_state import ui_state
  layout = AlphaLongitudinalToggles()
  monkeypatch.setattr(ui_state, "started", True)
  assert not layout._alpha_long_toggle.action_item.enabled
  monkeypatch.setattr(ui_state, "started", False)
  assert layout._alpha_long_toggle.action_item.enabled


def test_moved_switches_appear_once(params):
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.alpha_longitudinal_toggles import AlphaLongitudinalToggles
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.cruise import CruiseLayout
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.developer import DeveloperLayoutSP
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.toggles import TogglesLayoutSP

  developer, toggles = DeveloperLayoutSP(), TogglesLayoutSP()
  assert developer._alpha_long_toggle not in developer._scroller._items
  assert not toggles._toggles["ExperimentalMode"].is_visible
  assert not hasattr(CruiseLayout(), "dec_toggle")

  alpha = AlphaLongitudinalToggles()
  assert [key for key, _ in alpha._refresh_toggles] == [
    "AlphaLongitudinalEnabled", "ExperimentalMode", "ExperimentalModeSetSpeed", "ExperimentalModeLeadGap",
    "DynamicExperimentalControl"]


def test_alpha_longitudinal_heads_cruise(params, monkeypatch):
  from opendbc.car.structs import car
  from openpilot.cereal import custom
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.cruise import CruiseLayout
  from openpilot.selfdrive.ui.ui_state import ui_state

  monkeypatch.setattr(ui_state, "CP", car.CarParams.new_message(alphaLongitudinalAvailable=True, openpilotLongitudinalControl=True))
  monkeypatch.setattr(ui_state, "CP_SP", custom.CarParamsSP.new_message())
  monkeypatch.setattr(ui_state, "has_longitudinal_control", True)
  params.put_bool("ExperimentalModeSetSpeed", True, block=True)
  cruise = CruiseLayout()
  # plain toggles, no sub-panel
  assert cruise._scroller._items[:len(cruise._alpha_long.items)] == cruise._alpha_long.items
  cruise.show_event()
  cruise._update_state()
  assert cruise._alpha_long._alpha_long_toggle.is_visible
  assert cruise._alpha_long._set_speed_toggle.action_item.get_state()
  params.remove("ExperimentalModeSetSpeed")


def test_experimental_mode_text_matches_upstream(params, monkeypatch):
  # upstream writes this text inline in TogglesLayout._update_toggles; the panel keeps a copy with our name
  from opendbc.car.structs import car
  from openpilot.selfdrive.ui.layouts.settings.toggles import TogglesLayout
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.alpha_longitudinal_toggles import EXPERIMENTAL_MODE_DESCRIPTION
  from openpilot.selfdrive.ui.ui_state import ui_state

  monkeypatch.setattr(ui_state, "update_params", lambda: None)
  monkeypatch.setattr(ui_state, "CP", car.CarParams.new_message(openpilotLongitudinalControl=True))
  monkeypatch.setattr(ui_state, "has_longitudinal_control", True)
  toggles = TogglesLayout()
  toggles._update_toggles()
  assert toggles._toggles["ExperimentalMode"].description.replace("sunnypilot", "zoompilot") == EXPERIMENTAL_MODE_DESCRIPTION


def test_set_speed_greys_out_under_dec(params, monkeypatch):
  from opendbc.car.structs import car
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.alpha_longitudinal_toggles import AlphaLongitudinalToggles
  from openpilot.selfdrive.ui.ui_state import ui_state

  monkeypatch.setattr(ui_state, "CP", car.CarParams.new_message(alphaLongitudinalAvailable=True, openpilotLongitudinalControl=True))
  monkeypatch.setattr(ui_state, "has_longitudinal_control", True)
  layout = AlphaLongitudinalToggles()
  layout._dec_toggle.action_item.set_state(False)
  layout.update_state()
  assert layout._set_speed_toggle.action_item.enabled
  layout._dec_toggle.action_item.set_state(True)
  layout.update_state()
  assert not layout._set_speed_toggle.action_item.enabled
