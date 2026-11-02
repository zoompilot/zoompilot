"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The tizi (comma three / 3X) settings: the top of Cruise owns the alpha switch
(locked while engaged, no status line), which Developer no longer shows, and experimental mode.
Its own module: the big UI is chosen at import, so it cannot share a window with the mici
tests.
"""
import os

import pytest

os.environ["BIG"] = "1"
os.environ.setdefault("SCALE", "0.5")  # a full-size 2160x1080 hidden window aborts on macOS


@pytest.mark.parametrize("engaged", [True, False])
def test_alpha_toggle_locks_only_while_engaged(params, monkeypatch, engaged):
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.alpha_longitudinal_toggles import AlphaLongitudinalToggles
  from openpilot.selfdrive.ui.ui_state import UIState
  monkeypatch.setattr(UIState, "engaged", property(lambda _: engaged))
  assert AlphaLongitudinalToggles()._alpha_long_toggle.action_item.enabled == (not engaged)


def test_moved_switches_appear_once(params):
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.alpha_longitudinal_toggles import AlphaLongitudinalToggles
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.developer import DeveloperLayoutSP

  developer = DeveloperLayoutSP()
  assert developer._alpha_long_toggle not in developer._scroller._items

  alpha = AlphaLongitudinalToggles()
  assert [key for key, _ in alpha._refresh_toggles] == ["AlphaLongitudinalEnabled", "ExperimentalModeSetSpeed"]
  assert alpha.items == [item for _, item in alpha._refresh_toggles]


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


def test_assists_grey_out_without_long(params, monkeypatch):
  from opendbc.car.structs import car
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.alpha_longitudinal_toggles import AlphaLongitudinalToggles
  from openpilot.selfdrive.ui.ui_state import ui_state

  monkeypatch.setattr(ui_state, "CP", car.CarParams.new_message(alphaLongitudinalAvailable=True, openpilotLongitudinalControl=True))
  params.put_bool("ExperimentalModeSetSpeed", True, block=True)
  layout = AlphaLongitudinalToggles()
  for has_long in (True, False):
    monkeypatch.setattr(ui_state, "has_longitudinal_control", has_long)
    layout.update_state()
    assert layout._set_speed_toggle.action_item.enabled == has_long
  assert params.get_bool("ExperimentalModeSetSpeed")  # locked, not cleared
  params.remove("ExperimentalModeSetSpeed")


@pytest.mark.parametrize(("alpha", "has_long", "visible"), [
  (True, False, True),
  (False, True, True),
  (False, False, False),  # stock ACC only
])
def test_speed_assist_hidden_without_any_long(params, monkeypatch, alpha, has_long, visible):
  from opendbc.car.structs import car
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.alpha_longitudinal_toggles import AlphaLongitudinalToggles
  from openpilot.selfdrive.ui.ui_state import ui_state

  monkeypatch.setattr(ui_state, "CP", car.CarParams.new_message(alphaLongitudinalAvailable=alpha, openpilotLongitudinalControl=has_long))
  monkeypatch.setattr(ui_state, "has_longitudinal_control", has_long)
  layout = AlphaLongitudinalToggles()
  layout.update_state()
  assert layout._set_speed_toggle.is_visible == visible


def test_onroad_button_switches_speed_assist(params, monkeypatch):
  from openpilot.selfdrive.ui.sunnypilot.onroad.speed_assist_button import SpeedAssistButton
  from openpilot.selfdrive.ui.ui_state import ui_state

  button = SpeedAssistButton(192)
  monkeypatch.setattr(ui_state, "has_longitudinal_control", True)
  try:
    for start in (True, False):
      ui_state.set_speed_assist(start)
      button._handle_mouse_release(None)
      assert params.get_bool("ExperimentalModeSetSpeed") is (not start)
      assert ui_state.speed_assist is (not start)

    monkeypatch.setattr(ui_state, "has_longitudinal_control", False)
    ui_state.set_speed_assist(True)
    button._handle_mouse_release(None)
    assert params.get_bool("ExperimentalModeSetSpeed") is True
  finally:
    params.remove("ExperimentalModeSetSpeed")


@pytest.mark.parametrize("speed_assist", [True, False])
def test_onroad_button_draws(params, monkeypatch, speed_assist):
  import pyray as rl
  from openpilot.selfdrive.ui.sunnypilot.onroad.speed_assist_button import SpeedAssistButton
  from openpilot.selfdrive.ui.ui_state import ui_state

  monkeypatch.setattr(ui_state, "speed_assist", speed_assist)
  button = SpeedAssistButton(192)
  button.render(rl.Rectangle(0, 0, 192, 192))


def test_onroad_button_tap_leaves_the_sidebar(params, monkeypatch):
  from openpilot.selfdrive.ui.sunnypilot.onroad.augmented_road_view import AugmentedRoadViewSP
  from openpilot.selfdrive.ui.sunnypilot.onroad.hud_renderer import HudRendererSP
  from openpilot.system.ui.widgets import Widget

  class View(Widget, AugmentedRoadViewSP):
    def __init__(self):
      Widget.__init__(self)
      AugmentedRoadViewSP.__init__(self)
      self._hud_renderer = HudRendererSP()

    def _render(self, rect):
      pass

  view = View()
  assert view._touch_valid()
  monkeypatch.setattr(type(view._hud_renderer._speed_assist_button), "is_pressed", property(lambda self: True))
  assert not view._touch_valid()
