"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The tizi (comma three / 3X) settings: the top of Cruise owns the alpha switch
(locked while engaged, no status line), which Developer no longer shows, and experimental mode.
Its own module: the big UI is chosen at import, so it cannot share a window with the mici
tests.
"""
import itertools
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
  assert alpha.items[-1] is alpha._chill_toggle


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


def test_legacy_chill_toggle_needs_no_confirm(params, monkeypatch):
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.alpha_longitudinal_toggles import AlphaLongitudinalToggles
  from openpilot.system.ui.lib.application import gui_app

  pushed = []
  monkeypatch.setattr(gui_app, "push_widget", pushed.append)
  layout = AlphaLongitudinalToggles()
  for chill in (False, True):
    layout._chill_toggle.action_item.toggle._callback(chill)
    assert params.get_bool("ExperimentalMode") is not chill  # chill is the param's off
  assert not pushed


def test_assists_grey_out_without_long_or_in_chill(params, monkeypatch):
  from opendbc.car.structs import car
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.alpha_longitudinal_toggles import AlphaLongitudinalToggles
  from openpilot.selfdrive.ui.ui_state import ui_state

  monkeypatch.setattr(ui_state, "CP", car.CarParams.new_message(alphaLongitudinalAvailable=True, openpilotLongitudinalControl=True))
  params.put_bool("ExperimentalModeSetSpeed", True, block=True)
  layout = AlphaLongitudinalToggles()
  for has_long, chill in itertools.product((True, False), repeat=2):
    monkeypatch.setattr(ui_state, "has_longitudinal_control", has_long)
    layout._chill_toggle.action_item.set_state(chill)
    layout.update_state()
    assert layout._set_speed_toggle.action_item.enabled == (has_long and not chill)
  assert params.get_bool("ExperimentalModeSetSpeed")  # locked, not cleared
  params.remove("ExperimentalModeSetSpeed")


def test_onroad_button_switches_experimental_mode(params, monkeypatch):
  from openpilot.selfdrive.ui.sunnypilot.onroad.exp_button import ExpButton
  from openpilot.selfdrive.ui.ui_state import ui_state

  button = ExpButton(192, 144)
  monkeypatch.setattr(ui_state, "has_longitudinal_control", True)
  for start in (True, False):
    button._experimental_mode = start
    button._handle_mouse_release(None)
    assert params.get_bool("ExperimentalMode") is (not start)
    assert button._held_or_actual_mode() is (not start)  # shown until selfdrived publishes it

  monkeypatch.setattr(ui_state, "has_longitudinal_control", False)
  button._experimental_mode = True
  button._handle_mouse_release(None)
  assert params.get_bool("ExperimentalMode") is True


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
  monkeypatch.setattr(type(view._hud_renderer._exp_button), "is_pressed", property(lambda self: True))
  assert not view._touch_valid()
