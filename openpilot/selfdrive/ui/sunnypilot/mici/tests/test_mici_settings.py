"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Regression coverage for MICI parameter-to-display contracts.

import os

import pytest

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


@pytest.fixture
def params(gui):
  from openpilot.common.params import Params
  from openpilot.selfdrive.ui.ui_state import ui_state

  p = Params()
  ui_state.params = p
  # on device update_params() runs every frame before anything draws, so attributes it
  # sets (always_offroad, screensaver_enabled, ...) exist by render time. Without this a
  # layout reading one of them fails in the test for a reason the device never sees.
  ui_state.update_params()
  return p


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

  def test_torque_tune_unset_shows_declared_default(self, params):
    """controlsd_ext resolves an unset param through the params_keys.h default with
    return_default, so the selector must agree. If these drift, the UI claims a tune the car
    isn't running."""
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.steering import SteeringLayoutMici
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigMultiParamToggleSP

    versions = SteeringLayoutMici._load_torque_versions()
    assert list(versions.values()) == sorted(versions.values()), "must be oldest-first"

    params.remove("TorqueControlTune")
    w = BigMultiParamToggleSP("t", "TorqueControlTune", list(versions), values=list(versions.values()))
    assert versions[w.value] == pytest.approx(float(params.get("TorqueControlTune", return_default=True)))

    for label, version in versions.items():
      params.put("TorqueControlTune", version, block=True)
      w.refresh()
      assert w.value == label


class TestDependentSettings:
  """A setting whose parent makes it inert must read off without losing the user's value."""

  def test_reads_off_while_dependency_unmet_but_keeps_param(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigParamControlSP

    applies = True
    params.put_bool("AutoLaneChangeBsmDelay", True, block=True)
    w = BigParamControlSP("t", "AutoLaneChangeBsmDelay", depends_on=lambda: applies)

    w.refresh()
    assert w._checked and w.enabled

    applies = False
    w.refresh()
    assert not w._checked, "must display off while inert"
    assert not w.enabled, "must not accept input, or the forced-off display gets written back"
    assert params.get_bool("AutoLaneChangeBsmDelay"), "user's choice must survive"

    applies = True
    w.refresh()
    assert w._checked, "setting must come back when the dependency is met again"

  def test_no_dependency_behaves_like_upstream(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import BigParamControlSP

    params.put_bool("AutoLaneChangeBsmDelay", True, block=True)
    w = BigParamControlSP("t", "AutoLaneChangeBsmDelay")
    w.refresh()
    assert w._checked and w.enabled


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
    params.put_bool("SpeedDependentTorqueToggle", True, block=True)
    layout = SteeringLayoutMici()

    render(layout._tq_self_tune_view)
    assert not layout._tq_speed_dep._checked, "inert child must read off"
    assert not layout._tq_speed_dep.enabled
    assert params.get_bool("SpeedDependentTorqueToggle"), "and must keep its value"

    params.put_bool("LiveTorqueParamsToggle", True, block=True)
    render(layout._tq_self_tune_view)
    assert layout._tq_speed_dep._checked, "comes back when self-tune returns"


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
  ("cruise", "CruiseLayoutMici"),
  ("display", "DisplayLayoutMici"),
  ("models", "ModelsLayoutMici"),
  ("settings", "SettingsLayoutSP"),
  ("steering", "SteeringLayoutMici"),
  ("sunnylink", "SunnylinkLayoutMici"),
  ("trips", "TripsLayoutMici"),
  ("visuals", "VisualsLayoutMici"),
]


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
  """The models panel's provisioning line.

  Provisioning an accelerator is an upload plus a build, minutes long, and this
  is the only place a user sees it happening. The layout sweep above runs with
  no progress set, so none of these branches is covered by it.
  """

  STAGES = ['download', 'connect', 'upload', 'build', 'failed']

  def _info(self, stage, frac):
    from openpilot.selfdrive.ui.ui_state import ui_state
    ui_state.accelerator_progress = {'stage': stage, 'frac': frac, 'msg': ''}
    try:
      from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import _model_info
      return _model_info()
    finally:
      ui_state.accelerator_progress = None

  @pytest.mark.parametrize("stage", STAGES)
  def test_every_stage_gives_a_line(self, params, stage):
    active, header, info = self._info(stage, 0.42)
    assert active and header and info

  def test_a_percentage_is_shown_while_working(self, params):
    _, _, info = self._info('download', 0.42)
    assert '42%' in info

  def test_a_message_is_shown_instead_of_a_percentage_that_means_nothing(self, params):
    # A join has nothing to measure: it is waiting for a Jetson to boot, or
    # for a safe frame to swap on. Rendering that as "connect 0%" tells the
    # driver nothing, and "getting ready" alone does not separate a Jetson
    # that is unplugged from one six seconds from ready.
    from openpilot.selfdrive.ui.ui_state import ui_state
    ui_state.accelerator_progress = {'stage': 'connect', 'frac': 0.0, 'msg': 'waiting for the jetson'}
    try:
      from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import _model_info
      _, _, info = _model_info()
    finally:
      ui_state.accelerator_progress = None
    assert 'waiting for the jetson' in info
    assert '%' not in info

  def test_failure_says_so_rather_than_showing_100_percent(self, params):
    _, _, info = self._info('failed', 1.0)
    assert '100%' not in info

  def test_ready_falls_back_to_the_normal_line(self, params):
    # 'ready' is the steady state: the panel must go back to naming the model,
    # not sit on a finished progress bar forever.
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts import models as models_layout
    ready = self._info('ready', 1.0)
    from openpilot.selfdrive.ui.ui_state import ui_state
    ui_state.accelerator_progress = None
    assert ready == models_layout._model_info()

  @pytest.mark.parametrize("stage", STAGES)
  def test_the_panel_draws_with_progress_set(self, params, stage):
    from openpilot.selfdrive.ui.ui_state import ui_state
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import ModelsLayoutMici

    ui_state.accelerator_progress = {'stage': stage, 'frac': 0.5, 'msg': ''}
    try:
      layout = ModelsLayoutMici()
      render(layout)
      render(layout)
      for item in layout._scroller.items:
        render(item)
      render(layout.current_model_info)
    finally:
      ui_state.accelerator_progress = None


class TestAcceleratorIconState:
  """The home screen and sidebar draw ui_state.chestnut_state. For a backend
  the comma is the USB gadget for, the state has to come from deviceState and
  the progress param, not from a USB id the comma will never enumerate."""

  class FakeSM:
    def __init__(self, present):
      self.present = present
      self.recv_frame = {"modelV2": 0}
      self.alive = {"modelV2": False}

    def __getitem__(self, name):
      assert name == "deviceState"
      return type("DS", (), {"chestnutPresent": self.present})()

  def _state(self, present, compiled, progress):
    from openpilot.selfdrive.ui.ui_state import ui_state
    saved = ui_state.sm, ui_state.started, ui_state.chestnut_compiled, ui_state.accelerator_progress
    ui_state.sm, ui_state.started = self.FakeSM(present), False
    ui_state.chestnut_compiled, ui_state.accelerator_progress = compiled, progress
    try:
      ui_state._update_chestnut_state()
      return ui_state.chestnut_state
    finally:
      ui_state.sm, ui_state.started, ui_state.chestnut_compiled, ui_state.accelerator_progress = saved

  def test_offroad_states(self, params):
    from openpilot.selfdrive.ui.ui_state import ChestnutState
    assert self._state(False, False, None) == ChestnutState.DISCONNECTED
    assert self._state(True, True, None) == ChestnutState.READY
    assert self._state(True, False, None) == ChestnutState.UNCOMPILED
    assert self._state(True, False, {'stage': 'build', 'frac': 0.3}) == ChestnutState.LOADING
    assert self._state(True, False, {'stage': 'failed', 'frac': 1.0}) == ChestnutState.FAILED
    assert self._state(True, True, {'stage': 'connect', 'frac': 0.0}) == ChestnutState.LOADING
    assert self._state(True, True, {'stage': 'failed', 'frac': 1.0}) == ChestnutState.FAILED
    assert self._state(True, True, {'stage': 'ready', 'frac': 1.0}) == ChestnutState.READY

  def test_absent_accelerator_does_not_pulse_onroad(self, params):
    from types import SimpleNamespace
    from openpilot.selfdrive.ui.ui_state import UIState, ChestnutState

    state = SimpleNamespace(sm=self.FakeSM(False), started=True, started_frame=0,
                            chestnut_loading=True)
    UIState._update_chestnut_state(state)
    assert state.chestnut_state == ChestnutState.DISCONNECTED


class TestAcceleratorModelSelection:
  def test_selection_does_not_write_model_manager_slots(self, params):
    from unittest import mock
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import ModelsLayoutMici

    choice = {'backend': 'jetlink', 'name': 'Cinque Terre', 'selected': False, 'cached': True}
    layout = ModelsLayoutMici()
    with mock.patch('openpilot.selfdrive.ui.sunnypilot.mici.layouts.models.accelerators.select_model') as select, \
         mock.patch('openpilot.selfdrive.ui.sunnypilot.mici.layouts.models.ui_state.is_offroad', return_value=True), \
         mock.patch.object(layout, '_pop_to_main'), mock.patch.object(params, 'put') as put:
      layout._choose_accelerator(choice)
      select.assert_called_once_with('jetlink', 'Cinque Terre')
      put.assert_not_called()

  def test_selection_that_crosses_ignition_does_not_change_the_model(self, params):
    from unittest import mock
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import ModelsLayoutMici

    layout = ModelsLayoutMici()
    with mock.patch('openpilot.selfdrive.ui.sunnypilot.mici.layouts.models.accelerators.select_model') as select, \
         mock.patch('openpilot.selfdrive.ui.sunnypilot.mici.layouts.models.ui_state.is_offroad', return_value=False):
      layout._choose_accelerator({'backend': 'jetlink', 'name': 'Cinque Terre'})
      select.assert_not_called()

  def test_a_present_accelerator_is_not_an_unknown_usb_device(self, params):
    import time
    from unittest import mock
    from openpilot.selfdrive.ui import ui_state as module
    from openpilot.selfdrive.ui.ui_state import ui_state
    saved = ui_state.usb_connected, ui_state.usb_connected_ts, ui_state.usb_unknown, ui_state.chestnut_present
    try:
      ui_state.usb_connected, ui_state.usb_connected_ts = True, time.monotonic() - 11.0
      ui_state.usb_unknown, ui_state.chestnut_present = False, True
      with mock.patch.object(module, 'read_int', return_value=1), \
           mock.patch.object(module, 'get_usb_state', return_value=[]):
        ui_state.update_params()
      assert ui_state.usb_unknown is False
      ui_state.usb_connected_ts, ui_state.chestnut_present = time.monotonic() - 11.0, False
      with mock.patch.object(module, 'read_int', return_value=1), \
           mock.patch.object(module, 'get_usb_state', return_value=[]):
        ui_state.update_params()
      assert ui_state.usb_unknown is True
    finally:
      ui_state.usb_connected, ui_state.usb_connected_ts, ui_state.usb_unknown, ui_state.chestnut_present = saved


class TestAcceleratorLinkToggle:
  """The models panel's auto / on / off control over the accelerator link.

  Absent means auto, which the stock index toggle cannot store, and the control
  is hidden on a device it means nothing to: a plain comma must not grow a
  setting for hardware it will never see.
  """

  PARAM = "JetlinkEnabled"

  @staticmethod
  def _accelerators(present=False, ready=False, reason=None):
    from contextlib import ExitStack
    from unittest import mock

    stack = ExitStack()
    stack.enter_context(mock.patch("openpilot.sunnypilot.accelerators.present", return_value=present))
    stack.enter_context(mock.patch("openpilot.sunnypilot.accelerators.ready", return_value=ready))
    stack.enter_context(mock.patch("openpilot.sunnypilot.accelerators.unavailable_reason", return_value=reason))
    return stack

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

  def test_shown_when_ready_with_the_hardware_out_of_the_car(self, params):
    # the engine is cached, so on auto modeld will still try the link at the next
    # ignition. this is the case the off position exists for
    params.remove(self.PARAM)
    assert self._meaningful(ready=True)

  def test_shown_when_the_backend_has_a_complaint(self, params):
    params.remove(self.PARAM)
    assert self._meaningful(reason="no gadget")

  def test_shown_once_the_user_has_set_it(self, params):
    params.put_bool(self.PARAM, False, block=True)
    assert self._meaningful()

  def test_reads_all_three_states(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import read_link_state

    params.remove(self.PARAM)
    assert read_link_state() == "auto"
    params.put_bool(self.PARAM, True, block=True)
    assert read_link_state() == "on"
    params.put_bool(self.PARAM, False, block=True)
    assert read_link_state() == "off"

  def test_tap_cycles_auto_on_off_and_back(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import AcceleratorLinkToggle, LINK_STATES
    from openpilot.system.ui.lib.application import MousePos

    params.remove(self.PARAM)
    toggle = AcceleratorLinkToggle()
    assert toggle.value == toggle._options[LINK_STATES.index("auto")]
    toggle._handle_mouse_release(MousePos(0, 0))
    assert params.get(self.PARAM) is True
    toggle._handle_mouse_release(MousePos(0, 0))
    assert params.get(self.PARAM) is False
    toggle._handle_mouse_release(MousePos(0, 0))
    assert params.get(self.PARAM) is None, "auto is the absence of the param, not a third value"

  def test_refresh_follows_the_param(self, params):
    from openpilot.selfdrive.ui.sunnypilot.mici.layouts.models import AcceleratorLinkToggle, LINK_STATES

    params.remove(self.PARAM)
    toggle = AcceleratorLinkToggle()
    params.put_bool(self.PARAM, False, block=True)
    toggle.refresh()
    assert toggle.value == toggle._options[LINK_STATES.index("off")]

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
