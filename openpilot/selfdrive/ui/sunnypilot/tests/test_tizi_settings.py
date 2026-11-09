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
  import raylib_nb as rl
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


@pytest.mark.parametrize(("rhd", "dev_ui"), [(False, 0), (True, 0), (False, 3), (True, 3)])
def test_long_indicator_stacks_on_the_driver_monitor(params, monkeypatch, rhd, dev_ui):
  from types import SimpleNamespace

  import raylib_nb as rl
  from openpilot.selfdrive.ui.sunnypilot.onroad import long_indicator
  from openpilot.selfdrive.ui.sunnypilot.onroad.driver_state import driver_monitor_rect
  from openpilot.selfdrive.ui.ui_state import ui_state

  monkeypatch.setattr(ui_state, "sm", {"driverMonitoringState": SimpleNamespace(isRHD=rhd)})
  monkeypatch.setattr(ui_state, "developer_ui", dev_ui)
  drawn = []
  monkeypatch.setattr(long_indicator.rl, "draw_texture_ex", lambda tex, pos, *_: drawn.append((tex, pos)))
  indicator = long_indicator.LongIndicatorTizi()
  rect = rl.Rectangle(0, 0, 2160, 1080)
  last_bar = indicator._txt_distance[2][0]
  indicator._draw_centered(last_bar, rect, 152, 1.0)
  _, pos = drawn[-1]

  monitor = driver_monitor_rect(rect, rhd)
  assert (monitor.x < rect.width / 2) != rhd  # the driver's side
  assert pos.x + last_bar.width / 2 == pytest.approx(monitor.x + monitor.width / 2, abs=1)
  # mici's bar-to-wheel gap, doubled
  assert monitor.y - (pos.y + last_bar.height) == pytest.approx(2 * 18.5, abs=1.5)


class _SM(dict):
  """SubMaster stand-in: the messages, all just updated."""
  updated = {"modelDataV2SP": True}


@pytest.mark.parametrize(("trust", "curvature", "side"), [
  (0.0, 0.002, None),  # no lane center: no number
  (1.0, 0.0, ""),
  (1.0, -0.0005, "L "),
  (1.0, 0.0005, "R "),
])
def test_developer_ui_shows_lane_center_push(trust, curvature, side):
  from types import SimpleNamespace

  from openpilot.selfdrive.ui.sunnypilot.onroad.blendful_zp import pull_at_lookahead
  from openpilot.selfdrive.ui.sunnypilot.onroad.developer_ui.lane_center import LaneCenterElement

  sm = _SM(modelDataV2SP=SimpleNamespace(blendfulTrust=trust, blendfulCurvature=curvature), carState=SimpleNamespace(vEgo=25.0))
  element = LaneCenterElement()
  for _ in range(200):  # past the readout's 0.5 s smoothing
    shown = element.update(sm, False)
  assert (shown.label, shown.unit) == ("LANE CTR", "" if side is None else "cm")
  cm = round(abs(pull_at_lookahead(curvature, 25.0)) * 100)
  assert shown.value == ("-" if side is None else f"{side}{cm}")


@pytest.mark.parametrize("rhd", [False, True])
def test_confidence_ball_rides_above_the_stack_on_the_drivers_side(params, monkeypatch, rhd):
  from types import SimpleNamespace

  import raylib_nb as rl
  from openpilot.selfdrive.ui.onroad.hud_renderer import UI_CONFIG
  from openpilot.selfdrive.ui.sunnypilot.onroad import confidence_ball
  from openpilot.selfdrive.ui.sunnypilot.onroad.driver_state import driver_monitor_rect
  from openpilot.selfdrive.ui.sunnypilot.onroad.long_indicator import STACK_GAP, long_indicator_top
  from openpilot.selfdrive.ui.ui_state import UIStatus, ui_state

  monkeypatch.setattr(ui_state, "sm", {"driverMonitoringState": SimpleNamespace(isRHD=rhd)})
  monkeypatch.setattr(ui_state, "status", UIStatus.ENGAGED)
  for name in ("draw_rectangle_rounded", "draw_rectangle_rounded_lines_ex"):
    monkeypatch.setattr(confidence_ball.rl, name, lambda *a: None)
  drawn = []
  monkeypatch.setattr(confidence_ball, "draw_ball", lambda cx, cy, *a: drawn.append((cx, cy)))
  ball = confidence_ball.ConfidenceBallTizi()
  rect = rl.Rectangle(0, 0, 2160, 1080)
  monitor = driver_monitor_rect(rect, rhd)
  half = confidence_ball.RADIUS + confidence_ball.FRAME_PAD

  def render_at(confidence):
    ball._confidence_filter.x = confidence
    drawn.clear()
    ball._render(rect)
    return drawn

  assert render_at(1.0) == [(monitor.x + monitor.width / 2, UI_CONFIG.header_height + half)]  # sure: top, in the monitor's column
  assert render_at(0.0)[0][1] == pytest.approx(long_indicator_top(rect, rhd) - STACK_GAP - half)  # down to the car and bars
  assert render_at(-0.5) == []  # disengaged: faded out


def test_confidence_ball_is_a_visuals_toggle(params):
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.visuals import VisualsLayout

  assert "ConfidenceBall" in VisualsLayout()._toggle_defs
  assert params.get_bool("ConfidenceBall") is False  # off by default


def test_multi_button_follows_its_param(params):
  """The buttons read their param at construction: a change made elsewhere (the ICBM
  migration, the availability check, sunnylink) must show, or the next tap on the row writes
  the stale value back."""
  from openpilot.system.ui.sunnypilot.widgets.list_view import MultipleButtonActionSP
  from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode

  params.put("SpeedLimitMode", int(Mode.assist), block=True)
  buttons = MultipleButtonActionSP(["off", "info", "warn", "assist"], 100, param="SpeedLimitMode")
  params.put("SpeedLimitMode", int(Mode.information), block=True)
  buttons._update_state()
  assert buttons.selected_button == Mode.information


@pytest.mark.parametrize("lane_center", [False, True])
def test_lane_center_ui_sits_after_the_steering_arc(params, monkeypatch, lane_center):
  """Off / Gauge / Debug, after the steering arc's toggle; greyed out with lane center assist off."""
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.visuals import VisualsLayout
  from openpilot.selfdrive.ui.ui_state import ui_state

  monkeypatch.setattr(ui_state, "lane_center", lane_center)
  params.put("LaneCenterUI", 2, block=True)
  layout = VisualsLayout()
  layout._update_state()
  row = layout._lane_center_ui.item
  items = layout._scroller._items
  assert items.index(row) == items.index(layout._toggles["TorqueBar"]) + 1
  assert row.action_item.selected_button == 2
  assert row.action_item.enabled == lane_center


def test_lane_center_assist_is_off_on(params):
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.models import ModelsLayout

  layout = ModelsLayout()
  assert len(layout.blendful_toggle.action_item.buttons) == 2
  for on in (True, False):
    layout.blendful_toggle.action_item.callback(int(on))
    assert params.get_bool("Blendful") is on


@pytest.mark.parametrize(("lane_center", "stored", "shown"), [(False, 2, 0), (True, 0, 0), (True, 1, 1), (True, 2, 2)])
def test_lane_center_ui_is_off_with_lane_center_off(params, lane_center, stored, shown):
  """The gauge and the push follow LaneCenterUI only while lane center assist is on; its line follows Blendful."""
  from openpilot.selfdrive.ui.ui_state import ui_state

  params.put_bool("Blendful", lane_center, block=True)
  params.put("LaneCenterUI", stored, block=True)
  ui_state.update_params()
  assert (ui_state.lane_center, ui_state.lane_center_ui) == (lane_center, shown)


@pytest.mark.parametrize(("has_long", "vision", "map_", "enabled"), [
  (True, True, False, True),
  (True, False, True, True),
  (True, False, False, False),  # nothing reads it with both curve features off
  (False, True, True, False),
])
def test_curve_speed_follows_slow_for_curves(params, monkeypatch, has_long, vision, map_, enabled):
  from opendbc.car.structs import car
  from openpilot.cereal import custom
  from openpilot.selfdrive.ui.sunnypilot.layouts.settings.cruise import CruiseLayout
  from openpilot.selfdrive.ui.ui_state import ui_state

  monkeypatch.setattr(ui_state, "CP", car.CarParams.new_message(openpilotLongitudinalControl=has_long))
  monkeypatch.setattr(ui_state, "CP_SP", custom.CarParamsSP.new_message())
  monkeypatch.setattr(ui_state, "has_longitudinal_control", has_long)
  monkeypatch.setattr(ui_state, "has_icbm", False)
  params.put_bool("SmartCruiseControlVision", vision, block=True)
  params.put_bool("SmartCruiseControlMap", map_, block=True)
  params.put("CurveSpeedProfile", 2, block=True)
  try:
    cruise = CruiseLayout()
    items = cruise._scroller._items
    assert items.index(cruise.curve_speed.item) == items.index(cruise.scc_m_toggle) + 1 == items.index(cruise.scc_v_toggle) + 2
    cruise.show_event()
    cruise._update_state()
    assert cruise.curve_speed.item.action_item.enabled == enabled
    assert cruise.curve_speed.item.action_item.selected_button == 2
  finally:
    for key in ("SmartCruiseControlVision", "SmartCruiseControlMap", "CurveSpeedProfile"):
      params.remove(key)
