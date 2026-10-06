"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Regression coverage for MICI parameter-to-display contracts.

import os

import pytest
from unittest import mock

os.environ["BIG"] = "0"
os.environ.setdefault("SCALE", "1")


@pytest.fixture(scope="module")
def gui():
  """Hidden raylib window + isolated params dir. Widgets need textures, so a window is required."""
  import pyray as rl
  from openpilot.common.prefix import OpenpilotPrefix

  with OpenpilotPrefix():
    rl.set_config_flags(rl.FLAG_WINDOW_HIDDEN)
    from openpilot.system.ui.lib.application import gui_app
    gui_app.init_window("test_mici_settings", fps=30)
    yield gui_app
    gui_app.close()


def jetlink_status(**fields):
  """jetlink's snapshot as the UI's params pass takes it, nothing to show unless a field says so."""
  from jetlink.openpilot import Status
  base = {'enabled': False, 'mode': 'off', 'transport': 'USB', 'present': False, 'port': None, 'ready': False,
          'reason': None, 'progress': None, 'model': None, 'default_model': None}
  return Status(**{**base, **fields})


def render(widget):
  """Drive one frame through Widget.render, which calls _update_state."""
  import pyray as rl
  widget.render(rl.Rectangle(0, 0, 800, 600))


def wait_for_param(params, key, timeout=2.0):
  """Widgets write with a non-blocking put(), which lands on a background thread."""
  import time
  deadline = time.monotonic() + timeout
  last = params.get(key)
  while time.monotonic() < deadline:
    time.sleep(0.005)
    val = params.get(key)
    if val != last:
      return val
    last = val
  return last


class TestFloatParamScaling:
  """Float params store the physical value; the picker works in an x100 integer domain.

  Getting this wrong is a 100x error in a steering gain, and nothing about the UI looks broken:
  the label just reads 0.02 instead of 2.5. Mirrors OptionControlSP.use_float_scaling on TICI.
  """

  @pytest.mark.parametrize(("param", "stored", "expected_ui"), [
    ("TorqueParamsOverrideLatAccelFactor", 2.5, 250),   # params_keys.h default
    ("TorqueParamsOverrideLatAccelFactor", 1.0, 100),
    ("TorqueParamsOverrideFriction", 0.1, 10),          # params_keys.h default
  ])
  def test_reads_scaled_up(self, params, param, stored, expected_ui):
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigParamOption

    params.put(param, stored, block=True)
    opt = BigParamOption("t", param, min_value=1, max_value=500, float_param=True,
                         label_callback=lambda x: f"{x / 100}")
    assert opt._read_value() == expected_ui
    assert opt.value == str(stored)  # label divides back down

  def test_writes_scaled_down(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.number_picker import NumberPickerScreen

    param = "TorqueParamsOverrideLatAccelFactor"
    params.put(param, 1.0, block=True)
    picker = NumberPickerScreen(title="t", param=param, min_value=1, max_value=500, float_param=True)
    idx = next(i for i, item in enumerate(picker._picker_items) if item.raw_value == 250)
    picker._center_index = lambda: idx
    picker._commit_value()
    assert wait_for_param(params, param) == pytest.approx(2.5), "picker must divide by 100 on write"

  def test_round_trip_is_lossless(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.number_picker import NumberPickerScreen

    param = "TorqueParamsOverrideFriction"
    for physical in (0.05, 0.1, 0.5, 1.0):
      params.put(param, 0.99, block=True)  # something else, so the write below is observable
      picker = NumberPickerScreen(title="t", param=param, min_value=1, max_value=100, float_param=True)
      target = int(physical * 100)
      idx = next(i for i, item in enumerate(picker._picker_items) if item.raw_value == target)
      picker._center_index = lambda i=idx: i
      picker._commit_value()
      assert wait_for_param(params, param) == pytest.approx(physical), f"{physical} did not survive"

      # and the value we just wrote reads back as the same picker position
      assert NumberPickerScreen(title="t", param=param, min_value=1, max_value=100,
                                float_param=True)._read_value() == target

  def test_int_params_are_not_scaled(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigParamOption

    params.put("BlinkerMinLateralControlSpeed", 25, block=True)
    opt = BigParamOption("t", "BlinkerMinLateralControlSpeed", min_value=0, max_value=255)
    assert opt._read_value() == 25


class TestMultiParamValueMapping:
  """BigMultiParamToggleSP stores the option index by default, or a mapped value with `values=`."""

  def test_alc_modes_map_to_stored_value_not_index(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import ALC_LABELS
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigMultiParamToggleSP

    w = BigMultiParamToggleSP("t", "AutoLaneChangeTimer", list(ALC_LABELS.values()), values=list(ALC_LABELS))
    for mode, label in ALC_LABELS.items():
      params.put("AutoLaneChangeTimer", mode, block=True)
      w.refresh()
      assert w.value == label, f"mode {mode} showed {w.value}"

  def test_unset_resolves_to_declared_default(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import ALC_LABELS
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigMultiParamToggleSP
    from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode

    # The declared default is nudge (0), while off is stored as -1.
    params.remove("AutoLaneChangeTimer")
    w = BigMultiParamToggleSP("t", "AutoLaneChangeTimer", list(ALC_LABELS.values()), values=list(ALC_LABELS))
    assert w.value == ALC_LABELS[AutoLaneChangeMode.NUDGE]

  def test_tap_writes_mapped_value_and_wraps(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import ALC_LABELS
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigMultiParamToggleSP
    from openpilot.system.ui.lib.application import MousePos
    from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode

    modes = list(ALC_LABELS)
    params.put("AutoLaneChangeTimer", modes[-1], block=True)
    w = BigMultiParamToggleSP("t", "AutoLaneChangeTimer", list(ALC_LABELS.values()), values=modes)
    w.refresh()
    w._handle_mouse_release(MousePos(0, 0))
    # wraps to the first option, and stores -1 (the mode) rather than 0 (the index)
    assert w.value == ALC_LABELS[AutoLaneChangeMode.OFF]
    assert params.get("AutoLaneChangeTimer") == AutoLaneChangeMode.OFF

  @pytest.mark.parametrize("key", ["TorqueControlTune", "TorqueControlTuneBig"])
  def test_torque_tune_unset_shows_declared_default(self, params, key):
    """controlsd_ext resolves an unset param through the params_keys.h default with
    return_default, so each size's selector must agree. If these drift, the UI claims a tune
    the car isn't running."""
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigMultiParamToggleSP
    from openpilot.sunnypilot.selfdrive.controls.lib.torque_tune import versions_by_label

    versions = versions_by_label()
    assert list(versions.values()) == sorted(versions.values()), "must be oldest-first"

    params.remove(key)
    w = BigMultiParamToggleSP("t", key, list(versions), values=list(versions.values()))
    assert versions[w.value] == pytest.approx(float(params.get(key, return_default=True)))

    for label, version in versions.items():
      params.put(key, version, block=True)
      w.refresh()
      assert w.value == label


class TestDependentSettings:
  """A setting whose parent makes it inert locks, still showing the user's value on a grey pill."""

  def test_locks_while_dependency_unmet_and_keeps_param(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigParamControlSP

    applies = True
    params.put_bool("AutoLaneChangeBsmDelay", True, block=True)
    w = BigParamControlSP("t", "AutoLaneChangeBsmDelay", depends_on=lambda: applies)

    w.refresh()
    assert w._checked and w.enabled and not w.superseded

    applies = False
    w.refresh()
    assert w._checked, "the stored value stays on screen"
    assert w.superseded, "an inert on pill draws grey"
    assert not w.enabled, "must not accept input while inert"
    assert params.get_bool("AutoLaneChangeBsmDelay"), "user's choice must survive"

    applies = True
    w.refresh()
    assert w._checked and w.enabled and not w.superseded

  def test_no_dependency_behaves_like_upstream(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigParamControlSP

    params.put_bool("AutoLaneChangeBsmDelay", True, block=True)
    w = BigParamControlSP("t", "AutoLaneChangeBsmDelay")
    w.refresh()
    assert w._checked and w.enabled and not w.superseded


class TestSubPanelSelfRefresh:
  """gui_app renders only the top 2 nav-stack widgets, so a layout cannot drive a sub-panel
  nested under another sub-panel. Panels refresh themselves instead."""

  def test_rendering_the_panel_refreshes_its_items(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigParamControlSP, SubPanelSP

    params.put_bool("AutoLaneChangeBsmDelay", False, block=True)
    toggle = BigParamControlSP("t", "AutoLaneChangeBsmDelay")
    panel = SubPanelSP([toggle])
    assert not toggle._checked

    # an external writer (sunnylink, another panel) changes the param
    params.put_bool("AutoLaneChangeBsmDelay", True, block=True)
    render(panel)
    assert toggle._checked, "panel must pick up param changes without a parent driving it"

  def test_nested_panel_still_gates_itself(self, params):
    """The depth-3 case: self-tune sub-panel under the torque sub-panel."""
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import SteeringLayoutMici

    params.put_bool("LiveTorqueParamsToggle", False, block=True)
    params.put_bool("LiveTorqueParamsRelaxedToggle", True, block=True)
    layout = SteeringLayoutMici()

    render(layout._tq_self_tune_view)
    assert layout._tq_relaxed._checked and layout._tq_relaxed.superseded, "inert child shows its value, greyed"
    assert not layout._tq_relaxed.enabled
    assert params.get_bool("LiveTorqueParamsRelaxedToggle"), "and must keep its value"

    params.put_bool("LiveTorqueParamsToggle", True, block=True)
    render(layout._tq_self_tune_view)
    assert layout._tq_relaxed.enabled and not layout._tq_relaxed.superseded, "applies again when self-tune returns"


class TestSteeringLayoutBadges:
  def test_bsm_badge_hidden_when_auto_lane_change_cannot_feed_it(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import SteeringLayoutMici
    from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode

    layout = SteeringLayoutMici()
    params.put_bool("AutoLaneChangeBsmDelay", True, block=True)

    for mode, expected in [(AutoLaneChangeMode.NUDGELESS, True), (AutoLaneChangeMode.NUDGE, False),
                           (AutoLaneChangeMode.OFF, False), (AutoLaneChangeMode.THREE_SECONDS, True)]:
      params.put("AutoLaneChangeTimer", mode, block=True)
      layout._update_state()
      shown = "bsm-delay" in (layout._lane_change_btn._badge_labels or [])
      assert shown is expected, f"mode {mode}: badge shown={shown}"
      assert params.get_bool("AutoLaneChangeBsmDelay"), "badge suppression must not clear the param"


class TestRoadEdgeLaneChange:
  """RoadEdgeLaneChangeEnabled is ungated (matches TICI lane_change_settings) and works even
  with auto lane change off, so it must keep the lane-change entry button alive on its own."""

  def test_toggle_writes_param(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import SteeringLayoutMici
    from openpilot.system.ui.lib.application import MousePos

    params.put_bool("RoadEdgeLaneChangeEnabled", False, block=True)
    layout = SteeringLayoutMici()
    render(layout._lc_view)  # sub-panel self-refresh must not fight the tap
    assert not layout._lc_road_edge._checked
    assert layout._lc_road_edge.enabled, "toggle is ungated — no BSM/timer/offroad dependency"

    layout._lc_road_edge._handle_mouse_release(MousePos(0, 0))
    assert layout._lc_road_edge._checked
    assert params.get_bool("RoadEdgeLaneChangeEnabled")

    render(layout._lc_view)
    assert layout._lc_road_edge._checked, "self-refresh must not flip the tap back"

  def test_button_badge_when_only_road_edge_on(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import SteeringLayoutMici
    from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode

    params.put("AutoLaneChangeTimer", AutoLaneChangeMode.OFF, block=True)
    params.put_bool("AutoLaneChangeBsmDelay", False, block=True)
    params.put_bool("RoadEdgeLaneChangeEnabled", True, block=True)
    layout = SteeringLayoutMici()
    layout._update_state()
    assert "road-edge" in (layout._lane_change_btn._badge_labels or [])
    assert not layout._lane_change_btn._disabled

    params.put_bool("RoadEdgeLaneChangeEnabled", False, block=True)
    layout._update_state()
    assert layout._lane_change_btn._disabled, "everything off must still read disabled"


class TestDisplayScreenSaver:
  def test_timeout_gated_on_toggle_but_keeps_value(self, params):
    """Gated the way the file gates its brightness timer: set_enabled from _update_state.
    The stored timeout must survive the toggle being off."""
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.display import DisplayLayoutMici

    params.put_bool("ScreenSaverEnabled", False, block=True)
    params.put("ScreenSaverTimeout", 300, block=True)
    layout = DisplayLayoutMici()
    layout._update_state()
    assert not layout._screensaver_timeout.enabled, "timeout must reject input while saver is off"
    assert params.get("ScreenSaverTimeout") == 300, "gating must not touch the stored value"

    params.put_bool("ScreenSaverEnabled", True, block=True)
    layout._update_state()
    assert layout._screensaver_timeout.enabled
    assert layout._screensaver_timeout.value == "5 minutes", "300 s must read as minutes"

  def test_toggle_writes_param(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.display import DisplayLayoutMici
    from openpilot.system.ui.lib.application import MousePos

    params.put_bool("ScreenSaverEnabled", True, block=True)
    layout = DisplayLayoutMici()
    layout._update_state()
    layout._screensaver._handle_mouse_release(MousePos(0, 0))
    assert not params.get_bool("ScreenSaverEnabled")
    layout._update_state()
    assert not layout._screensaver_timeout.enabled, "gate must follow the tap"


class TestJerkAwareToggle:
  """LateralJerkTorqueController and NNLC are mutually exclusive (ui_state and the car interface
  both force-disable the pair); the layout must gate the toggles the same way or a tap on one
  while the other is on re-creates the conflict and gets both silently wiped at the next init."""

  def test_jerk_aware_locked_while_nnlc_on(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import SteeringLayoutMici

    params.put_bool("NeuralNetworkLateralControl", True, block=True)
    layout = SteeringLayoutMici()
    render(layout._tq_view)
    assert not layout._jerk_aware_toggle.enabled

    params.put_bool("NeuralNetworkLateralControl", False, block=True)
    render(layout._tq_view)
    assert layout._jerk_aware_toggle.enabled

  def test_jerk_aware_locked_while_every_model_size_runs_v2(self, params):
    """v2 forces the jerk-aware controller off; one size on another tune keeps it meaningful."""
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import SteeringLayoutMici

    params.put_bool("EnforceTorqueControl", True, block=True)
    params.put("TorqueControlTune", 2.0, block=True)
    params.put("TorqueControlTuneBig", 2.0, block=True)
    layout = SteeringLayoutMici()
    render(layout._tq_view)
    assert not layout._jerk_aware_toggle.enabled

    params.put("TorqueControlTuneBig", 1.0, block=True)
    render(layout._tq_view)
    assert layout._jerk_aware_toggle.enabled

  def test_nnlc_locked_while_jerk_aware_on(self, params):
    from opendbc.car.structs import car
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import SteeringLayoutMici
    from openpilot.selfdrive.ui.ui_state import ui_state

    class _CP:
      steerControlType = car.CarParams.SteerControlType.torque
      enableBsm = False

    old_cp = ui_state.CP
    ui_state.CP = _CP()
    try:
      params.put_bool("LateralJerkTorqueController", True, block=True)
      layout = SteeringLayoutMici()
      layout._update_state()
      assert not layout._nnlc_toggle.enabled

      params.put_bool("LateralJerkTorqueController", False, block=True)
      layout._update_state()
      assert layout._nnlc_toggle.enabled
    finally:
      ui_state.CP = old_cp

  def test_torque_button_reflects_jerk_aware_without_enforce(self, params):
    """Jerk-aware works without EnforceTorqueControl, so the entry button must not read
    'disabled' while it is the only thing on."""
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import SteeringLayoutMici

    layout = SteeringLayoutMici()
    params.put_bool("EnforceTorqueControl", False, block=True)
    params.put_bool("LateralJerkTorqueController", True, block=True)
    layout._update_state()
    assert "jerk-aware" in (layout._torque_settings_btn._badge_labels or [])
    assert not layout._torque_settings_btn._disabled

    params.put_bool("LateralJerkTorqueController", False, block=True)
    layout._update_state()
    assert layout._torque_settings_btn._disabled


class TestMadsLimitedCallSignature:
  """get_mads_limited_brands grew a params argument upstream (Tesla MADS screen activation).
  The call only executes with a fingerprinted car AND CarParamsSP present, which no other
  test provides, so a stale call site renders fine in tests and TypeErrors on the device."""

  def test_update_state_with_fingerprinted_car(self, params):
    from opendbc.car.structs import car
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import SteeringLayoutMici
    from openpilot.selfdrive.ui.ui_state import ui_state

    class _CP:
      brand = "mazda"
      steerControlType = car.CarParams.SteerControlType.torque
      enableBsm = True

    class _CPSP:
      flags = 0

    old_cp, old_cp_sp = ui_state.CP, ui_state.CP_SP
    ui_state.CP, ui_state.CP_SP = _CP(), _CPSP()
    try:
      layout = SteeringLayoutMici()
      layout._update_state()  # raises TypeError if the call site misses an argument
      assert layout._mads_limited is False
    finally:
      ui_state.CP, ui_state.CP_SP = old_cp, old_cp_sp


    # State-only tests do not cover widget overrides that depend on upstream drawing internals.

LAYOUT_TARGETS = [
  ("alpha_longitudinal", "AlphaLongitudinalLayoutMici"),
  ("cruise", "CruiseLayoutMici"),
  ("developer", "DeveloperLayoutMiciSP"),
  ("display", "DisplayLayoutMici"),
  ("models", "ModelsLayoutMici"),
  ("settings", "SettingsLayoutSP"),
  ("software", "SoftwareLayoutSP"),
  ("steering", "SteeringLayoutMici"),
  ("sunnylink", "SunnylinkLayoutMici"),
  ("toggles", "TogglesLayoutMiciSP"),
  ("trips", "TripsLayoutMici"),
  ("visuals", "VisualsLayoutMici"),
]


class TestCruiseBadges:
  """The speed limit button's icbm badge follows the stored mode. Which setups move speed limits
  through ICBM is icbm_moves_speed_limits, tested on its own; this checks the layout wiring."""

  @pytest.mark.parametrize(("mode", "shown"), [(3, True), (2, False)])
  def test_speed_limit_badge_follows_mode(self, params, mode, shown):
    from opendbc.car.structs import car
    from openpilot.cereal import custom
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.cruise import CruiseLayoutMici
    from openpilot.selfdrive.ui.ui_state import ui_state

    params.put("CarParamsPersistent", car.CarParams.new_message(
      brand="mazda", pcmCruise=True, alphaLongitudinalAvailable=True, openpilotLongitudinalControl=True).to_bytes(), block=True)
    params.put("CarParamsSPPersistent", custom.CarParamsSP.new_message(
      intelligentCruiseButtonManagementAvailable=True).to_bytes(), block=True)
    params.put_bool("AlphaLongitudinalEnabled", True, block=True)
    params.put_bool("IntelligentCruiseButtonManagement", True, block=True)
    params.put("SpeedLimitMode", mode, block=True)
    saved = ui_state.CP, ui_state.CP_SP, ui_state.has_longitudinal_control, ui_state.has_icbm
    try:
      ui_state.update_params()
      layout = CruiseLayoutMici()
      render(layout)
      assert ("icbm" in (layout._speed_limit_btn._badge_labels or [])) == shown
    finally:
      ui_state.CP, ui_state.CP_SP, ui_state.has_longitudinal_control, ui_state.has_icbm = saved
      for key in ("CarParamsPersistent", "CarParamsSPPersistent", "AlphaLongitudinalEnabled",
                  "IntelligentCruiseButtonManagement", "SpeedLimitMode"):
        params.remove(key)


class TestSubtitleAreaRenders:
  """BigButtonSP's three subtitle modes are drawn, not stored, so each needs a real frame."""

  def _button(self, **kwargs):
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigButtonSP
    return BigButtonSP("Lane change", **kwargs)

  def test_badges_render(self, params):
    btn = self._button()
    btn.set_badges([("road-edge", "on"), ("bsm-delay", "on")])
    assert btn._badge_labels == ["road-edge", "bsm-delay"]
    render(btn)

  def test_wrapped_badges_render(self, params):
    # more pills than fit on one row exercises the multi-row layout in _draw_badges
    btn = self._button()
    btn.set_badges([(f"badge-{i}", "on") for i in range(8)])
    render(btn)

  def test_disabled_pill_renders(self, params):
    btn = self._button()
    btn.set_disabled()
    assert btn._disabled
    render(btn)

  def test_plain_value_subtitle_renders(self, params):
    # upstream's own path: no badges, so _draw_content must fall through to BigButton
    btn = self._button(value="on")
    assert btn._badge_labels is None
    render(btn)


class TestReleaseNotes:
  """The notes only exist as params updated writes, so the button has to read the right pair."""

  def _button(self, params, current=b"<h1>old</h1>", new=b"<h1>new</h1>", available=False):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.software import ReleaseNotesButton

    params.put("UpdaterCurrentDescription", "0.1 / develop / abc / 2026-01-01", block=True)
    params.put("UpdaterNewDescription", "0.2 / develop / def / 2026-01-02", block=True)
    params.put("UpdaterCurrentReleaseNotes", current, block=True)
    params.put("UpdaterNewReleaseNotes", new, block=True)
    params.put_bool("UpdateAvailable", available, block=True)
    return ReleaseNotesButton()

  def test_shows_installed_version(self, params):
    btn = self._button(params)
    render(btn)
    assert btn.get_value() == "0.1"
    btn._click_callback()
    assert "old" in [e.content for e in btn._page._content.elements]

  def test_shows_pending_version(self, params):
    btn = self._button(params, available=True)
    render(btn)
    assert btn.get_value() == "0.2"
    btn._click_callback()
    assert "new" in [e.content for e in btn._page._content.elements]

  def test_page_renders_without_notes(self, params):
    btn = self._button(params, current=b"")
    btn._click_callback()
    render(btn._page)

  def test_page_renders_changelog(self, params):
    # the real thing: the first CHANGELOG.md block, parsed the way updated does it
    from openpilot.common.basedir import BASEDIR
    from openpilot.system.updated.updated import parse_release_notes

    btn = self._button(params, current=parse_release_notes(BASEDIR))
    btn._click_callback()
    render(btn._page)
    assert len(btn._page._content.elements) > 1


class TestUpdateAlert:
  """The alert is the only place a staged update announces itself, so it owns the notes flow."""

  def _alerts(self, params, available=True):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.offroad_alerts import UPDATE_KEY, MiciOffroadAlertsSP

    params.put("UpdaterNewDescription", "2026.09.07-13 / develop / 259190d / Sep 07", block=True)
    params.put("UpdaterNewReleaseNotes", b"<h1>zoompilot v2026.09.07-13</h1>", block=True)
    params.put_bool(UPDATE_KEY, available, block=True)

    layout = MiciOffroadAlertsSP()
    pending = {alert.key: params.get(alert.key) for alert in layout.sorted_alerts}
    pending["UpdaterNewDescription"] = params.get("UpdaterNewDescription")
    layout._refresh(pending)
    return layout

  def test_alert_points_at_the_notes(self, params):
    item = self._alerts(params)._update_item
    assert item.alert_data.visible
    assert "zoompilot 2026.09.07-13, Sep 07" in item.alert_data.text
    assert "blog.comma.ai" not in item.alert_data.text
    # the item re-split the new text, so the card knows how tall it has to be
    assert item._body_text == "Tap to read what's new."

  def test_no_alert_without_an_update(self, params):
    item = self._alerts(params, available=False)._update_item
    assert not item.alert_data.visible
    assert item.alert_data.text == ""

  def test_click_opens_the_new_notes(self, params):
    layout = self._alerts(params)
    layout._update_item._click_callback()
    assert "zoompilot v2026.09.07-13" in [e.content for e in layout._notes_page._content.elements]

  def test_install_slider_follows_the_staged_update(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.software import ReleaseNotesPage

    page = ReleaseNotesPage()
    params.put_bool("UpdateAvailable", True, block=True)
    assert page._install.is_visible
    render(page)  # the slider only has a rect once the page lays it out under the notes
    assert page._install.rect.height > 0

    params.put_bool("UpdateAvailable", False, block=True)
    assert not page._install.is_visible
    render(page)


class TestLayoutsSurviveRender:
  """Post-sync guard. A layout that imports and updates cleanly can still crash on draw."""

  @pytest.mark.parametrize(("module", "cls"), LAYOUT_TARGETS)
  def test_layout_renders(self, params, module, cls):
    import importlib

    mod = importlib.import_module(f"openpilot.selfdrive.ui.sunnypilot.mici.layouts.{module}")
    layout = getattr(mod, cls)()
    render(layout)
    render(layout)  # second frame: first one only populates the scroller's visible set

  @pytest.mark.parametrize(("module", "cls"), LAYOUT_TARGETS)
  def test_every_scroller_item_renders(self, params, module, cls):
    # The scroller culls anything off screen, so rendering the layout alone only proves the
    # top of the list draws. Draw every item so a break further down cannot hide behind a
    # scroll position no test ever reaches.
    import importlib

    mod = importlib.import_module(f"openpilot.selfdrive.ui.sunnypilot.mici.layouts.{module}")
    layout = getattr(mod, cls)()
    render(layout)
    items = layout._scroller.items
    assert items, f"{cls} rendered no items, so this guard would pass vacuously"
    for item in items:
      render(item)

  def test_home_layout_renders(self, params):
    # the boot screen, and the only SP layout that is not a scroller
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.home import MiciHomeLayoutSP

    render(MiciHomeLayoutSP())


class TestAcceleratorProgressRenders:
  """the models panel's provisioning line; the layout sweep above runs with no progress set"""

  STAGES = ['download', 'connect', 'upload', 'build', 'failed']

  def _info(self, stage, frac, msg='', drops=0, mode='usb'):
    from openpilot.selfdrive.ui.ui_state import ui_state
    saved = ui_state.jetlink
    ui_state.jetlink = jetlink_status(present=True, mode=mode, progress={'stage': stage, 'frac': frac, 'msg': msg, 'drops': drops})
    try:
      from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import _model_info
      return _model_info()
    finally:
      ui_state.jetlink = saved

  @pytest.mark.parametrize("stage", STAGES)
  def test_every_stage_gives_a_line(self, params, stage):
    active, header, info = self._info(stage, 0.42)
    assert active and header and info

  def test_a_percentage_is_shown_while_working(self, params):
    _, _, info = self._info('download', 0.42)
    assert '42%' in info

  @pytest.mark.parametrize("stage, frac, msg, drops, mode, shown", [
    # a join has nothing to measure, and "getting ready" alone does not separate an
    # unplugged Jetson from one six seconds from ready
    ('connect', 0.0, 'waiting for jetlink', 0, 'usb', 'waiting for jetlink'),
    ('download', 0.45, 'downloading', 0, 'usb', 'downloading 45%'),
    # jetlink counts the drops; the card names the cable, and on iOS the phone app too
    ('connect', 0.0, 'reconnecting', 2, 'usb', 'reconnecting, check cable (2 drops)'),
    ('connect', 0.0, 'waiting for jetlink', 3, 'ios', 'waiting for jetlink, check cable or app (3 drops)'),
  ])
  def test_the_line_is_jetlinks_message(self, params, stage, frac, msg, drops, mode, shown):
    assert self._info(stage, frac, msg, drops, mode)[2] == shown

  def test_the_stand_in_is_named_while_the_pick_is_not_ready(self, params):
    from openpilot.selfdrive.ui.ui_state import ui_state
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import _model_info
    saved = ui_state.jetlink
    ui_state.jetlink = jetlink_status(enabled=True, mode='usb', present=True, model='ResAction Preview',
                                      standin='Cinque Terre V3')
    try:
      assert _model_info()[1:] == ('big model', 'cinque terre v3 for now')
    finally:
      ui_state.jetlink = saved

  def test_a_jetlink_only_pick_downloading_with_no_files_renders(self, params):
    # the model manager reports such a pick downloading with no files for a
    # moment; dividing its progress by zero killed the UI (2026-10-06)
    from unittest import mock
    from openpilot.cereal import messaging
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import ModelsLayoutMici
    msg = messaging.new_message('modelManagerSP')
    bundle = msg.modelManagerSP.selectedBundle
    bundle.status = 'downloading'
    bundle.internalName = 'RESACT'
    bundle.init('models', 0)
    layout = ModelsLayoutMici()
    with mock.patch.object(ModelsLayoutMici, 'model_manager', new_callable=mock.PropertyMock,
                           return_value=msg.modelManagerSP):
      render(layout)
      render(layout)
    assert not layout.cancel_download_btn.is_visible

  def test_failure_says_so_rather_than_showing_100_percent(self, params):
    _, _, info = self._info('failed', 1.0)
    assert '100%' not in info

  def test_ready_falls_back_to_the_normal_line(self, params):
    # 'ready' is the steady state: the panel must go back to naming the model,
    # not sit on a finished progress bar forever.
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts import models as models_layout
    ready = self._info('ready', 1.0)
    from openpilot.selfdrive.ui.ui_state import ui_state
    saved = ui_state.jetlink
    ui_state.jetlink = jetlink_status(present=True)
    try:
      assert ready == models_layout._model_info()
    finally:
      ui_state.jetlink = saved

  @pytest.mark.parametrize("stage", STAGES)
  def test_the_panel_draws_with_progress_set(self, params, stage):
    from openpilot.selfdrive.ui.ui_state import ui_state
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import ModelsLayoutMici

    saved = ui_state.jetlink
    ui_state.jetlink = jetlink_status(present=True, progress={'stage': stage, 'frac': 0.5, 'msg': ''})
    try:
      layout = ModelsLayoutMici()
      render(layout)
      render(layout)
      for item in layout._scroller.items:
        render(item)
      render(layout.current_model_info)
    finally:
      ui_state.jetlink = saved


class TestAcceleratorIconState:
  """chestnut_state for an off-board accelerator comes from jetlink's snapshot and
  modeld's acceleratorState, not a USB id the comma never enumerates. a fitted
  chestnut keeps upstream's path"""

  class FakeSM:
    def __init__(self, big=False, alive=False, recv=0):
      self.recv_frame = {"modelV2": recv}
      self.alive = {"modelV2": alive}
      self.big = big

    def __getitem__(self, name):
      if name == "deviceState":
        return type("DS", (), {"chestnutPresent": False})()
      assert name == "modelV2"
      return type("M", (), {"big": self.big})()

  @staticmethod
  def _view(present=True, ready=False, progress=None, state='none'):
    """The link on, and modeld's acceleratorState by name."""
    return jetlink_status(enabled=True, mode='usb', present=present, ready=ready, progress=progress), state

  def _state(self, view, sm=None, started=False):
    from openpilot.selfdrive.ui.ui_state import ui_state
    saved = ui_state.sm, ui_state.started, ui_state.started_frame, ui_state.jetlink, ui_state._accelerator_state_name
    ui_state.sm, ui_state.started, ui_state.started_frame = sm or self.FakeSM(), started, 0
    ui_state.jetlink, ui_state._accelerator_state_name = view
    try:
      ui_state._update_chestnut_state()
      return ui_state.chestnut_state
    finally:
      ui_state.sm, ui_state.started, ui_state.started_frame, ui_state.jetlink, ui_state._accelerator_state_name = saved

  def test_offroad_states(self, params):
    from openpilot.selfdrive.ui.ui_state import ChestnutState
    assert self._state(self._view(present=False)) == ChestnutState.DISCONNECTED
    assert self._state(self._view(ready=True)) == ChestnutState.READY
    assert self._state(self._view()) == ChestnutState.UNCOMPILED
    assert self._state(self._view(progress={'stage': 'build', 'frac': 0.3})) == ChestnutState.LOADING
    assert self._state(self._view(progress={'stage': 'failed', 'frac': 1.0})) == ChestnutState.FAILED
    assert self._state(self._view(ready=True, progress={'stage': 'connect', 'frac': 0.0})) == ChestnutState.LOADING
    assert self._state(self._view(ready=True, progress={'stage': 'failed', 'frac': 1.0})) == ChestnutState.FAILED
    assert self._state(self._view(ready=True, progress={'stage': 'ready', 'frac': 1.0})) == ChestnutState.READY

  def test_absent_accelerator_does_not_pulse_onroad(self, params):
    from openpilot.selfdrive.ui.ui_state import ChestnutState
    view = self._view(present=False, ready=True, state='retrying')
    assert self._state(view, self.FakeSM(alive=True, recv=1), started=True) == ChestnutState.DISCONNECTED

  def test_onroad_states(self, params):
    from openpilot.selfdrive.ui.ui_state import ChestnutState
    driving = self.FakeSM(alive=True, recv=1)
    assert self._state(self._view(ready=True, state='joining'), driving, started=True) == ChestnutState.LOADING
    assert self._state(self._view(ready=True, state='retrying'), driving, started=True) == ChestnutState.LOADING
    assert self._state(self._view(ready=True, state='running'), driving, started=True) == ChestnutState.ACTIVE
    assert self._state(self._view(ready=True, state='unavailable'), driving, started=True) == ChestnutState.FAILED
    assert self._state(self._view(ready=False, state='none'), driving, started=True) == ChestnutState.UNCOMPILED
    # nothing from modeld yet is loading, not failed
    assert self._state(self._view(ready=True), self.FakeSM(), started=True) == ChestnutState.LOADING
    # a big frame is proof, whatever the status field says
    big = self.FakeSM(big=True, alive=True, recv=1)
    assert self._state(self._view(ready=True, state='unavailable'), big, started=True) == ChestnutState.ACTIVE


  @staticmethod
  def _usb(present):
    from contextlib import ExitStack
    from unittest import mock
    from openpilot.selfdrive.ui import ui_state as module
    stack = ExitStack()
    stack.enter_context(mock.patch.object(module, 'read_int', return_value=1))
    stack.enter_context(mock.patch.object(module, 'get_usb_state', return_value=[]))
    stack.enter_context(mock.patch("openpilot.sunnypilot.jetlink_adapter.status", return_value=jetlink_status(present=present)))
    return stack

  def test_a_present_accelerator_is_not_an_unknown_usb_device(self, params):
    import time
    from openpilot.selfdrive.ui.ui_state import ui_state
    saved = ui_state.usb_connected, ui_state.usb_connected_ts, ui_state.usb_unknown, ui_state.jetlink
    try:
      ui_state.usb_connected, ui_state.usb_connected_ts, ui_state.usb_unknown = True, time.monotonic() - 11.0, False
      with self._usb(present=True):
        ui_state.update_params()  # builds the view
        ui_state.usb_connected_ts = time.monotonic() - 11.0
        ui_state.update_params()  # decides
      assert ui_state.jetlink_view is not None
      assert ui_state.usb_unknown is False
      with self._usb(present=False):
        ui_state.update_params()
        ui_state.usb_connected_ts = time.monotonic() - 11.0
        ui_state.update_params()
      assert ui_state.jetlink_view is None
      assert ui_state.usb_unknown is True
    finally:
      ui_state.usb_connected, ui_state.usb_connected_ts, ui_state.usb_unknown, ui_state.jetlink = saved

  def test_an_accelerator_recognised_after_the_grace_period_clears_unknown(self, params):
    """the Jetson configures the gadget ~25 s after the UI starts, after the one-shot
    usb_unknown decision; presence arriving later must still clear it"""
    from openpilot.selfdrive.ui.ui_state import ui_state
    saved = ui_state.usb_connected, ui_state.usb_connected_ts, ui_state.usb_unknown, ui_state.jetlink
    try:
      ui_state.usb_connected, ui_state.usb_connected_ts, ui_state.usb_unknown = True, None, True
      with self._usb(present=False):
        ui_state.update_params()
        assert ui_state.usb_unknown is True
      with self._usb(present=True):
        ui_state.update_params()
        assert ui_state.usb_unknown is False
    finally:
      ui_state.usb_connected, ui_state.usb_connected_ts, ui_state.usb_unknown, ui_state.jetlink = saved


class TestAcceleratorLinkToggle:
  """off, usb or ios in one param, and the control is hidden on a device it means nothing to"""

  PARAM = "JetlinkLink"

  @staticmethod
  def _accelerators(installed=False, **fields):
    """ui_state's jetlink snapshot for the block: None, no jetlink here, unless
    it is installed or a field says there is something to show."""
    from unittest import mock
    from openpilot.selfdrive.ui.ui_state import ui_state
    status = jetlink_status(**fields) if installed or any(fields.values()) else None
    return mock.patch.object(ui_state, "jetlink", status)

  def _meaningful(self, **accelerators) -> bool:
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import link_toggle_meaningful
    with self._accelerators(**accelerators):
      return link_toggle_meaningful()

  def test_hidden_on_a_plain_device(self, params):
    params.remove(self.PARAM)
    assert not self._meaningful()

  def test_shown_when_an_accelerator_is_attached(self, params):
    params.remove(self.PARAM)
    assert self._meaningful(present=True)

  def test_shown_wherever_the_package_is_installed(self, params):
    # with the link off there is no gadget for a Jetson to enumerate, so present()
    # alone would hide the toggle that turns the link on
    params.remove(self.PARAM)
    assert self._meaningful(installed=True)

  def test_the_value_line_says_what_each_mode_is_for(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import AcceleratorLinkToggle

    params.remove(self.PARAM)
    toggle = AcceleratorLinkToggle()
    assert toggle.get_value() == "off"
    for index, value in enumerate(("off", "usb", "iOS")):
      params.put(self.PARAM, index, block=True)
      toggle.refresh()
      assert toggle.get_value() == value

  def test_shown_when_ready_with_the_hardware_out_of_the_car(self, params):
    # the engine is cached and the link may be on, so modeld will still try it at
    # the next ignition. this is the case the off position exists for
    params.remove(self.PARAM)
    assert self._meaningful(ready=True)

  def test_shown_when_the_backend_has_a_complaint(self, params):
    params.remove(self.PARAM)
    assert self._meaningful(reason="no gadget")

  def test_shown_once_the_user_has_turned_it_on(self, params):
    params.put(self.PARAM, 1, block=True)
    assert self._meaningful()

  def test_hidden_when_off_with_nothing_attached(self, params):
    params.put(self.PARAM, 0, block=True)
    assert not self._meaningful()

  def test_absent_reads_as_off(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import link_mode

    params.remove(self.PARAM)
    assert link_mode() == "off"
    for index, mode in enumerate(("off", "usb", "ios")):
      params.put(self.PARAM, index, block=True)
      assert link_mode() == mode

  def test_tap_cycles_off_usb_ios(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import AcceleratorLinkToggle
    from openpilot.system.ui.lib.application import MousePos

    params.remove(self.PARAM)
    toggle = AcceleratorLinkToggle()
    assert toggle._mode == "off"
    # the small model is the model manager's: the toggle never touches the runner cache
    with mock.patch.object(params, "remove", wraps=params.remove) as remove, \
         mock.patch('openpilot.selfdrive.ui.sunnypilot.mici.layouts.models.ui_state.is_offroad', return_value=True):
      for index in (1, 2, 0):
        toggle._handle_mouse_release(MousePos(0, 0))
        assert params.get(self.PARAM) == index
    assert "ModelRunnerTypeCache" not in {c.args[0] for c in remove.call_args_list}

  def test_refresh_follows_the_param(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import AcceleratorLinkToggle

    params.remove(self.PARAM)
    toggle = AcceleratorLinkToggle()
    params.put(self.PARAM, 2, block=True)
    toggle.refresh()
    assert toggle._mode == "ios"

  def test_link_toggle_cannot_change_after_ignition(self, params):
    from unittest import mock
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import AcceleratorLinkToggle
    from openpilot.system.ui.lib.application import MousePos

    params.put(self.PARAM, 2, block=True)
    toggle = AcceleratorLinkToggle()
    with mock.patch('openpilot.selfdrive.ui.sunnypilot.mici.layouts.models.ui_state.is_offroad', return_value=False):
      # drawn disabled, as the model buttons beside it are onroad: a refused
      # tap used to animate with nothing changing
      assert not toggle.enabled
      render(toggle)
      toggle._handle_mouse_release(MousePos(0, 0))
      assert params.get(self.PARAM) == 2
      assert toggle._mode == "ios", "the pills must not show a mode the param does not have"
    with mock.patch('openpilot.selfdrive.ui.sunnypilot.mici.layouts.models.ui_state.is_offroad', return_value=True):
      assert toggle.enabled
      toggle._handle_mouse_release(MousePos(0, 0))
      assert params.get(self.PARAM) == 0

  def test_layout_hides_the_toggle_until_it_means_something(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import ModelsLayoutMici

    params.remove(self.PARAM)
    with self._accelerators():
      layout = ModelsLayoutMici()
      assert not layout.link_toggle.is_visible
      assert layout.link_toggle in layout._scroller.items
    with self._accelerators(present=True):
      layout = ModelsLayoutMici()
      assert layout.link_toggle.is_visible
      render(layout)
      render(layout)
      render(layout.link_toggle)

  def test_refresh_spins_beside_the_toggle_until_both_catalogs_are_stamped(self, params):
    # sunnypilot's refresh, as upstream: the model manager restamps each catalog it
    # refetches, the big-model one extended for the accelerator or not
    import time
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import ModelsLayoutMici
    from openpilot.selfdrive.ui.sunnypilot.model_info import MODEL_SYNC_KEYS

    params.put(self.PARAM, 1, block=True)
    for key in MODEL_SYNC_KEYS:
      params.put(key, 1, block=True)
    with self._accelerators(present=True), \
         mock.patch('openpilot.selfdrive.ui.sunnypilot.mici.layouts.models.ui_state.is_offroad', return_value=True):
      layout = ModelsLayoutMici()
      button = layout.refresh_btn

      def shown():
        render(layout)
        return button.get_value(), button.enabled

      assert layout.link_toggle.is_visible and button in layout._scroller.items
      assert shown() == ("", True)
      layout._refresh_models()
      deadline = time.monotonic() + 2.0
      while any(params.get(key) for key in MODEL_SYNC_KEYS):
        assert time.monotonic() < deadline, "the refresh never zeroed the sync keys"
        time.sleep(0.005)
      assert shown() == ("fetching...", False)
      params.put(MODEL_SYNC_KEYS[0], 2, block=True)
      assert shown() == ("fetching...", False)
      params.put(MODEL_SYNC_KEYS[1], 2, block=True)
      assert shown() == ("", True)


class TestDefaultBigModelMici:
  """The big-models button names whose default an empty slot runs: the chestnut's
  model in the tree when a board is fitted, else the accelerator's."""

  def _big_models_value(self, monkeypatch, board):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import ModelsLayoutMici
    from openpilot.selfdrive.ui.ui_state import ui_state
    from openpilot.system.ui.lib.application import gui_app

    pushed = []
    monkeypatch.setattr(gui_app, "push_widget", lambda w: pushed.append(w))
    monkeypatch.setattr(ui_state, "chestnut_present", board)
    with mock.patch.object(ui_state, "jetlink", jetlink_status(present=True, default_model="Cinque Terre V3 Model")):
      layout = ModelsLayoutMici()
      render(layout)
      layout._show_folders()
      buttons = pushed[-1]._scroller.items
      for button in buttons:
        render(button)
    return buttons[1].get_value().removesuffix(" (active)")

  def test_a_fitted_chestnut_names_the_in_tree_model(self, params, monkeypatch):
    from openpilot.sunnypilot.models.model_name import DEFAULT_BIG_MODEL
    params.remove("ModelManager_ActiveBundleChestnut")
    assert self._big_models_value(monkeypatch, True) == f"{DEFAULT_BIG_MODEL} (Default)".lower()

  def test_without_a_chestnut_the_accelerator_names_its_default(self, params, monkeypatch):
    params.remove("ModelManager_ActiveBundleChestnut")
    assert self._big_models_value(monkeypatch, False) == "cinque terre v3 model (default)"


class TestAlphaLongSwitchMici:
  """The alpha switch is the saved preference, editable offroad only (forced offroad included)."""

  def test_switch_is_offroad_only(self, params, monkeypatch):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.alpha_longitudinal import AlphaLongitudinalLayoutMici
    from openpilot.selfdrive.ui.ui_state import ui_state
    layout = AlphaLongitudinalLayoutMici()
    monkeypatch.setattr(ui_state, "started", True)
    render(layout)
    assert not layout._alpha_long_toggle.enabled
    monkeypatch.setattr(ui_state, "started", False)  # naturally offroad or forced offroad
    render(layout)
    assert layout._alpha_long_toggle.enabled

  def test_force_offroad_writes_the_preference_only(self, params, monkeypatch):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.settings import SettingsLayoutSP
    from openpilot.system.ui.lib.application import gui_app
    layout = SettingsLayoutSP()
    pushed = []
    monkeypatch.setattr(gui_app, "push_widget", lambda w: pushed.append(w))
    layout._handle_always_offroad(True)
    pushed[-1]._confirm_callback()
    assert wait_for_param(params, "OffroadModeRequested") is True
    assert not params.get_bool("OffroadMode")  # applied by hardwared, never the UI


class TestAlphaLongitudinalPanelMici:
  """The alpha switch, experimental mode, speed assist and DEC live under Cruise, once."""

  def test_moved_switches_appear_once(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.alpha_longitudinal import AlphaLongitudinalLayoutMici
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.cruise import CruiseLayoutMici
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.developer import DeveloperLayoutMiciSP
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.toggles import TogglesLayoutMiciSP

    developer, toggles = DeveloperLayoutMiciSP(), TogglesLayoutMiciSP()
    assert developer._alpha_long_toggle not in developer._scroller.items
    assert toggles._experimental_btn not in toggles._scroller.items
    assert not hasattr(CruiseLayoutMici(), "_dec_toggle")

    alpha = AlphaLongitudinalLayoutMici()
    assert [key for key, _ in alpha._refresh_toggles] == [
      "AlphaLongitudinalEnabled", "ExperimentalMode", "ExperimentalModeSetSpeed", "ExperimentalModeLeadGap",
      "DynamicExperimentalControl"]

  def test_settings_opens_the_sp_panels(self, params, monkeypatch):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.developer import DeveloperLayoutMiciSP
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.settings import SettingsLayoutSP
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.toggles import TogglesLayoutMiciSP
    from openpilot.system.ui.lib.application import gui_app

    buttons = {btn.get_text(): btn for btn in SettingsLayoutSP()._scroller.items if hasattr(btn, "get_text")}
    pushed = []
    monkeypatch.setattr(gui_app, "push_widget", lambda w: pushed.append(w))
    for label, cls in (("toggles", TogglesLayoutMiciSP), ("developer", DeveloperLayoutMiciSP)):
      buttons[label]._click_callback()
      assert type(pushed[-1]) is cls

  @pytest.mark.parametrize(("has_long", "badges"), [
    (True, ["exp.", "speed", "follow"]),
    (False, None),  # alpha off: a grey "disabled" pill
  ])
  def test_cruise_entry_shows_the_mode(self, params, monkeypatch, has_long, badges):
    from opendbc.car.structs import car
    from openpilot.cereal import custom
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.alpha_longitudinal import AlphaLongitudinalLayoutMici
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.cruise import CruiseLayoutMici
    from openpilot.selfdrive.ui.ui_state import ui_state
    from openpilot.selfdrive.ui.sunnypilot.longitudinal_mode import E2E_ASSISTS
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import badge_rows
    from openpilot.system.ui.lib.application import gui_app

    monkeypatch.setattr(ui_state, "CP", car.CarParams.new_message(alphaLongitudinalAvailable=True, openpilotLongitudinalControl=has_long))
    monkeypatch.setattr(ui_state, "CP_SP", custom.CarParamsSP.new_message())
    monkeypatch.setattr(ui_state, "has_longitudinal_control", has_long)
    monkeypatch.setattr(ui_state, "experimental_mode", True)
    for param, _ in E2E_ASSISTS:
      params.put_bool(param, True, block=True)
    try:
      cruise = CruiseLayoutMici()
      render(cruise)
      btn = cruise._scroller.items[0]
      assert btn is cruise._alpha_long_btn
      assert btn._badge_labels == (badges or ["disabled"])
      assert btn._disabled == (badges is None)
      assert len(badge_rows(btn._badge_labels, btn._subtitle_width_hint())) == 1  # the widest state stays on one row

      pushed = []
      monkeypatch.setattr(gui_app, "push_widget", lambda w: pushed.append(w))
      btn._click_callback()
      assert type(pushed[-1]) is AlphaLongitudinalLayoutMici
    finally:
      for param, _ in E2E_ASSISTS:
        params.remove(param)

  def test_experimental_mode_waits_for_confirmation(self, params, monkeypatch):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts import alpha_longitudinal
    from openpilot.system.ui.lib.application import gui_app

    params.remove("ExperimentalModeConfirmed")
    params.put_bool("ExperimentalMode", False, block=True)
    confirms = []
    monkeypatch.setattr(alpha_longitudinal, "ExperimentalModeConfirmPage", confirms.append)
    monkeypatch.setattr(gui_app, "push_widget", lambda w: None)
    layout = alpha_longitudinal.AlphaLongitudinalLayoutMici()
    layout._on_experimental_mode(True)
    assert not layout._experimental_toggle._checked
    assert not params.get_bool("ExperimentalMode")
    confirms[-1]()
    assert wait_for_param(params, "ExperimentalMode") is True
    assert layout._experimental_toggle._checked


class TestOverriddenToggles:
  """A toggle another setting overrides draws a grey pill; one only locked onroad keeps its green one."""

  @pytest.mark.parametrize("experimental", [False, True])
  def test_e2e_toggles_lock_without_experimental(self, params, monkeypatch, experimental):
    from opendbc.car.structs import car
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.alpha_longitudinal import AlphaLongitudinalLayoutMici
    from openpilot.selfdrive.ui.ui_state import ui_state

    monkeypatch.setattr(ui_state, "CP", car.CarParams.new_message(openpilotLongitudinalControl=True))
    monkeypatch.setattr(ui_state, "has_longitudinal_control", True)
    for key in ("ExperimentalModeSetSpeed", "ExperimentalModeLeadGap"):
      params.put_bool(key, True, block=True)
    params.put_bool("DynamicExperimentalControl", False, block=True)
    params.put_bool("ExperimentalMode", experimental, block=True)
    try:
      layout = AlphaLongitudinalLayoutMici()
      layout._refresh()
      layout._update_state()
      for toggle in (layout._dec_toggle, layout._set_speed_toggle, layout._lead_gap_toggle):
        assert toggle.enabled == experimental
      for toggle in (layout._set_speed_toggle, layout._lead_gap_toggle):
        assert toggle._checked, "locking must keep the stored value"
        assert toggle.superseded == (not experimental)
    finally:
      for key in ("ExperimentalModeSetSpeed", "ExperimentalModeLeadGap", "ExperimentalMode"):
        params.remove(key)

  def test_onroad_lock_keeps_the_green_pill(self, params, monkeypatch):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import SteeringLayoutMici
    from openpilot.selfdrive.ui.ui_state import ui_state

    monkeypatch.setattr(ui_state, "is_offroad", lambda: False)
    layout = SteeringLayoutMici()
    layout._update_state()
    assert not layout._mads_toggle.enabled
    assert not layout._mads_toggle.superseded

  @pytest.mark.parametrize("bsm", [False, True])
  def test_blind_spot_locks_without_bsm(self, params, monkeypatch, bsm):
    from opendbc.car.structs import car
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.visuals import VisualsLayoutMici
    from openpilot.selfdrive.ui.ui_state import ui_state

    monkeypatch.setattr(ui_state, "CP", car.CarParams.new_message(enableBsm=bsm))
    toggle = VisualsLayoutMici()._toggles["BlindSpot"]
    assert toggle.enabled == bsm
    assert toggle.superseded == (not bsm)

  @pytest.mark.parametrize("mode", [0, 1])
  def test_speed_limit_settings_lock_with_the_mode_off(self, params, monkeypatch, mode):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.cruise import CruiseLayoutMici
    from openpilot.system.ui.lib.application import gui_app

    params.put("SpeedLimitMode", mode, block=True)
    params.put("SpeedLimitOffsetType", 1, block=True)
    monkeypatch.setattr(gui_app, "widget_in_stack", lambda w: True)
    try:
      layout = CruiseLayoutMici()
      layout._update_speed_limit_state(False, False, False, 1)
      for item in (layout._sl_source, layout._sl_offset_type, layout._sl_offset_value):
        assert item.enabled == bool(mode)
      for item in (layout._sl_source, layout._sl_offset_type):
        assert item.superseded == (not mode)
    finally:
      params.remove("SpeedLimitMode")
      params.remove("SpeedLimitOffsetType")

  def test_tja_button_overrides_main_cruise_and_unified(self, params, monkeypatch):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import SteeringLayoutMici

    monkeypatch.setattr(SteeringLayoutMici, "_is_mazda", staticmethod(lambda: True))
    layout = SteeringLayoutMici()
    layout._mads_limited = False
    layout._mads_toggle.set_checked(True)
    layout._mads_unified.set_checked(True)
    layout._mads_tja.set_checked(False)
    assert layout._mads_unified.enabled and not layout._mads_unified.superseded
    layout._mads_tja.set_checked(True)
    assert not layout._mads_unified.enabled and layout._mads_unified.superseded
    assert layout._mads_unified._checked

  def test_tja_onroad_lock_keeps_the_green_pill(self, params, monkeypatch):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import SteeringLayoutMici
    from openpilot.selfdrive.ui.ui_state import ui_state

    monkeypatch.setattr(ui_state, "is_offroad", lambda: False)
    layout = SteeringLayoutMici()
    layout._mads_toggle.set_checked(True)
    assert not layout._mads_tja.enabled
    assert not layout._mads_tja.superseded

  def test_manual_realtime_locks_self_tune(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import SteeringLayoutMici

    layout = SteeringLayoutMici()
    layout._tq_self_tune.set_checked(True)
    layout._tq_relaxed.set_checked(True)
    layout._tq_custom.set_checked(True)
    layout._tq_manual_rt.set_checked(True)
    for item in (layout._tq_self_tune, layout._tq_relaxed, layout._tq_speed_dep):
      assert not item.enabled and item.superseded
    layout._tq_manual_rt.set_checked(False)
    for item in (layout._tq_self_tune, layout._tq_relaxed):
      assert item.enabled and not item.superseded
