"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car.structs import car
from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl
from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import (
  BigButtonSP,
  BigMultiParamToggleSP,
  BigParamControlSP,
  BigParamOption,
  speed_unit,
)
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.mads.helpers import MadsSteeringModeOnBrake, get_mads_limited_brands, offroad_brand
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AUTO_LANE_CHANGE_TIMER, AutoLaneChangeMode
from openpilot.sunnypilot.selfdrive.controls.lib.lane_change_smoothing import LEVEL_OFF, read_level
from openpilot.sunnypilot.selfdrive.controls.lib.torque_tune import load_versions, resolved_tune_version
from openpilot.system.ui.lib.application import gui_app

MADS_STEERING_MODE_LABELS = [tr("remain"), tr("pause"), tr("disengage")]
# LaneChangeSmoothing stores the index into this list (lane_change_smoothing.LEVELS order, off first)
LC_LEVEL_LABELS = [tr("off"), tr("fast"), tr("medium"), tr("slow"), tr("extra slow")]

# Keys match the stored modes, including -1 for off. Reuse the controller's timer values.
ALC_LABELS = {
  AutoLaneChangeMode.OFF: tr("off"),
  AutoLaneChangeMode.NUDGE: tr("nudge"),
  AutoLaneChangeMode.NUDGELESS: tr("nudgeless"),
} | {mode: f"{AUTO_LANE_CHANGE_TIMER[mode]:g} {tr('s')}" for mode in
     (AutoLaneChangeMode.HALF_SECOND, AutoLaneChangeMode.ONE_SECOND,
      AutoLaneChangeMode.TWO_SECONDS, AutoLaneChangeMode.THREE_SECONDS)}


def _on_off(val: bool) -> str:
  return "on" if val else "off"


def _alc_label(v: int) -> str:
  return ALC_LABELS.get(v, ALC_LABELS[AutoLaneChangeMode.NUDGE])


class SteeringLayoutMici(NavScroller):
  """Steering settings for MADS, lane changes, blinker pause, and torque control."""

  def __init__(self):
    super().__init__()

    # None forces the initial cleanup pass.
    self._prev_torque_allowed: bool | None = None
    self._prev_mads_limited: bool | None = None

    # Cache state used by enable callbacks each frame.
    self._mads_limited = False
    self._alc_val = AutoLaneChangeMode.NUDGE
    self._torque_allowed = False
    self._enforce_torque = False
    self._v2_tune = False

    self._mads_settings_btn = BigButtonSP(tr("mads"))
    self._lane_change_btn = BigButtonSP(tr("lane change"))
    self._blinker_settings_btn = BigButtonSP(tr("blinker pause"))
    self._torque_settings_btn = BigButtonSP(tr("torque control"))
    self._nnlc_toggle = BigParamControl(tr("nnlc"), "NeuralNetworkLateralControl")

    for btn in [self._mads_settings_btn, self._lane_change_btn, self._blinker_settings_btn, self._torque_settings_btn]:
      btn.set_subtitle_font_size(24)

    self._scroller.add_widgets([
      self._mads_settings_btn, self._lane_change_btn,
      self._blinker_settings_btn,
      self._torque_settings_btn, self._nnlc_toggle,
    ])

    self._mads_toggle = BigParamControl(tr("enable mads"), "Mads")
    self._mads_toggle.set_enabled(ui_state.is_offroad)
    # Read the live toggle so dependent controls update in the same frame.
    self._mads_main_cruise = BigParamControlSP(tr("main cruise toggle"), "MadsMainCruiseAllowed",
                                               depends_on=lambda: self._mads_toggle._checked and not self._mads_limited)
    self._mads_unified = BigParamControlSP(tr("unified engagement"), "MadsUnifiedEngagementMode",
                                           depends_on=lambda: self._mads_toggle._checked and not self._mads_limited)
    self._mads_steering = BigMultiParamToggleSP(tr("steering on brake"), "MadsSteeringMode", MADS_STEERING_MODE_LABELS)
    # Mazda trims with the physical TJA button: it becomes the only lateral switch, so main
    # cruise and unified engagement stop touching MADS. Fingerprint cannot tell, so ask.
    self._mads_tja = BigParamControlSP(tr("tja button"), "MazdaTjaButton",
                                       depends_on=lambda: self._mads_toggle._checked and ui_state.is_offroad())
    self._mads_tja.set_visible(self._is_mazda)
    self._mads_view = self._mads_settings_btn.link_sub_panel([self._mads_toggle, self._mads_main_cruise, self._mads_unified,
                                                              self._mads_steering, self._mads_tja])

    # AutoLaneChangeTimer stores a mode from -1 through 5, not a boolean.
    self._lc_timer = BigMultiParamToggleSP(tr("auto lane change"), "AutoLaneChangeTimer",
                                           list(ALC_LABELS.values()), values=list(ALC_LABELS))
    self._lc_bsm = BigParamControlSP(tr("bsm delay"), "AutoLaneChangeBsmDelay",
                                     depends_on=lambda: self._bsm_applies(self._alc_val) and self._car_has_bsm())
    self._lc_road_edge = BigParamControl(tr("road edge block"), "RoadEdgeLaneChangeEnabled")
    # off = stock; every level is a slower lane change than stock
    self._lc_level = BigMultiParamToggleSP(tr("lane change") + "\n" + tr("smoothing"), "LaneChangeSmoothing", LC_LEVEL_LABELS)
    self._lc_view = self._lane_change_btn.link_sub_panel([self._lc_timer, self._lc_bsm, self._lc_road_edge,
                                                          self._lc_level])

    self._blinker_toggle = BigParamControl(tr("enable blinker pause"), "BlinkerPauseLateralControl")
    self._blinker_speed = BigParamOption(tr("blinker speed"), "BlinkerMinLateralControlSpeed",
                                         min_value=0, max_value=255, value_change_step=5,
                                         label_callback=lambda v: f"{v} {speed_unit()}", picker_unit=speed_unit)
    self._blinker_delay = BigParamOption(tr("blinker delay"), "BlinkerLateralReengageDelay",
                                         min_value=0, max_value=10,
                                         label_callback=lambda v: f"{v} " + tr("seconds"), picker_unit=tr("seconds"))
    for opt in (self._blinker_speed, self._blinker_delay):
      opt.set_enabled(lambda: self._blinker_toggle._checked)
    self._blinker_view = self._blinker_settings_btn.link_sub_panel([self._blinker_toggle, self._blinker_speed, self._blinker_delay])

    self._torque_toggle = BigParamControl(tr("enable torque control"), "EnforceTorqueControl")
    self._torque_toggle.set_enabled(lambda: self._torque_allowed and ui_state.is_offroad() and
                                    not ui_state.params.get_bool("NeuralNetworkLateralControl"))

    # Jerk-aware control is independent of EnforceTorqueControl on torque-native cars.
    # NNLC and the v2 tune use the same controller path, so they disable this option.
    self._jerk_aware_toggle = BigParamControl(tr("jerk aware"), "LateralJerkTorqueController")
    self._jerk_aware_toggle.set_enabled(lambda: ui_state.is_offroad() and
                                        not ui_state.params.get_bool("NeuralNetworkLateralControl") and
                                        not self._v2_tune)

    # An unset version resolves through the param default. Keep a fallback for unreadable metadata.
    tq_versions = self._load_torque_versions() or {tr("default"): 2.0}
    self._tq_version = BigMultiParamToggleSP(tr("tune version"), "TorqueControlTune",
                                             list(tq_versions), values=list(tq_versions.values()))

    self._tq_self_tune_btn = BigButtonSP(tr("self tune"))
    self._tq_self_tune_btn.set_subtitle_font_size(24)
    # Third-level panels do not run this layout's update loop, so gate their controls directly.
    self._tq_self_tune = BigParamControl(tr("enable self-tune"), "LiveTorqueParamsToggle")
    self._tq_self_tune.set_enabled(ui_state.is_offroad)
    # torqued reads both settings at startup.
    self._tq_relaxed = BigParamControlSP(tr("less restrict"), "LiveTorqueParamsRelaxedToggle",
                                         depends_on=lambda: self._tq_self_tune._checked and ui_state.is_offroad())
    self._tq_speed_dep = BigParamControlSP(tr("speed dependent"), "SpeedDependentTorqueToggle",
                                           depends_on=lambda: self._tq_self_tune._checked and ui_state.is_offroad())
    self._tq_self_tune_view = self._tq_self_tune_btn.link_sub_panel([self._tq_self_tune, self._tq_relaxed, self._tq_speed_dep])

    self._tq_custom_btn = BigButtonSP(tr("custom tune"))
    self._tq_custom_btn.set_subtitle_font_size(24)
    self._tq_custom = BigParamControl(tr("enable custom tuning"), "CustomTorqueParams")
    self._tq_custom.set_enabled(ui_state.is_offroad)
    self._tq_manual_rt = BigParamControlSP(tr("manual realtime"), "TorqueParamsOverrideEnabled",
                                           depends_on=lambda: self._tq_custom._checked)
    self._tq_lat_accel = BigParamOption(tr("lat accel"), "TorqueParamsOverrideLatAccelFactor",
                                        min_value=1, max_value=500, label_callback=lambda x: f"{x / 100} m/s\u00b2",
                                        picker_label_callback=lambda x: f"{x / 100}", float_param=True, picker_unit="m/s\u00b2")
    self._tq_friction = BigParamOption(tr("friction"), "TorqueParamsOverrideFriction",
                                       min_value=1, max_value=100, label_callback=lambda x: f"{x / 100}",
                                       picker_label_callback=lambda x: f"{x / 100}", float_param=True)
    for opt in (self._tq_lat_accel, self._tq_friction):
      opt.set_enabled(lambda: self._tq_custom._checked)
    self._tq_custom_view = self._tq_custom_btn.link_sub_panel([self._tq_custom, self._tq_manual_rt, self._tq_lat_accel, self._tq_friction])

    self._tq_items_rest = [self._tq_self_tune_btn, self._tq_custom_btn]
    for item in self._tq_items_rest:
      item.set_enabled(lambda: self._enforce_torque)
    # controlsd selects the tune at startup.
    self._tq_version.set_enabled(lambda: self._enforce_torque and ui_state.is_offroad())
    self._tq_view = self._torque_settings_btn.link_sub_panel([self._torque_toggle, self._jerk_aware_toggle,
                                                              self._tq_version] + self._tq_items_rest)

  @staticmethod
  def _load_torque_versions() -> dict[str, float]:
    """Load torque versions in ascending order for the selector."""
    try:
      data = load_versions()
    except (OSError, ValueError):
      return {}
    versions: dict[str, float] = {}
    for label, info in data.items():
      try:
        versions[label] = float(info["version"])
      except (KeyError, ValueError, TypeError):
        pass
    return dict(sorted(versions.items(), key=lambda kv: kv[1]))

  def _update_state(self):
    super()._update_state()

    self._nnlc_toggle.refresh()

    torque_allowed = self._torque_allowed = (ui_state.CP is not None and
                                             ui_state.CP.steerControlType != car.CarParams.SteerControlType.angle)
    # Wait for fingerprinting before clearing torque settings to avoid racing seeded defaults.
    if ui_state.CP is not None and not torque_allowed and self._prev_torque_allowed is not False:
      ui_state.params.remove("EnforceTorqueControl")
      ui_state.params.remove("NeuralNetworkLateralControl")
      ui_state.params.remove("LateralJerkTorqueController")
    self._prev_torque_allowed = torque_allowed

    mads_on = ui_state.params.get_bool("Mads")
    offroad = ui_state.is_offroad()
    self._mads_settings_btn.set_enabled(offroad)
    if not mads_on:
      self._mads_settings_btn.set_disabled()
    else:
      cruise = _on_off(ui_state.params.get_bool("MadsMainCruiseAllowed"))
      unified = _on_off(ui_state.params.get_bool("MadsUnifiedEngagementMode"))
      steer_idx = ui_state.params.get("MadsSteeringMode", return_default=True) or 0
      steer_mode = MADS_STEERING_MODE_LABELS[min(steer_idx, len(MADS_STEERING_MODE_LABELS) - 1)]
      badges = [(tr("enabled"), "on"), (tr("main-cruise"), cruise), (tr("unified"), unified), (steer_mode, "on")]
      if self._is_mazda():
        badges.append((tr("tja"), _on_off(ui_state.params.get_bool("MazdaTjaButton"))))
      self._mads_settings_btn.set_badges(badges)

    blinker_on = ui_state.params.get_bool("BlinkerPauseLateralControl")
    if not blinker_on:
      self._blinker_settings_btn.set_disabled()
    else:
      speed_val = ui_state.params.get("BlinkerMinLateralControlSpeed", return_default=True) or 0
      delay_val = ui_state.params.get("BlinkerLateralReengageDelay", return_default=True) or 0
      self._blinker_settings_btn.set_badges([(tr("enabled"), "on"), (tr("pause"), f"{speed_val}{speed_unit()}"), (tr("delay"), f"{delay_val}s")])

    alc_val = self._alc_val = int(ui_state.params.get("AutoLaneChangeTimer", return_default=True) or AutoLaneChangeMode.NUDGE)
    # Hide the BSM delay when inactive without discarding the stored preference.
    lc_bsm = _on_off(ui_state.params.get_bool("AutoLaneChangeBsmDelay") and self._bsm_applies(alc_val))
    road_edge = _on_off(ui_state.params.get_bool("RoadEdgeLaneChangeEnabled"))
    lc_level = read_level(ui_state.params)
    if alc_val <= AutoLaneChangeMode.OFF and lc_bsm == "off" and road_edge == "off" and lc_level == LEVEL_OFF:
      self._lane_change_btn.set_disabled()
    else:
      auto_badge = _alc_label(alc_val) if alc_val > AutoLaneChangeMode.OFF else "off"
      self._lane_change_btn.set_badges([(tr("auto"), auto_badge), (tr("bsm-delay"), lc_bsm),
                                        (tr("road-edge"), road_edge), (tr("smooth"), LC_LEVEL_LABELS[lc_level])])

    enforce_torque = self._enforce_torque = ui_state.params.get_bool("EnforceTorqueControl")
    self._v2_tune = resolved_tune_version(ui_state.params) == 2.0
    jerk_aware = ui_state.params.get_bool("LateralJerkTorqueController")
    self_tune_on = ui_state.params.get_bool("LiveTorqueParamsToggle")
    custom_on = ui_state.params.get_bool("CustomTorqueParams")

    self._torque_settings_btn.set_enabled(torque_allowed)
    if not enforce_torque and not jerk_aware:
      self._torque_settings_btn.set_disabled()
    else:
      # set_badges hides entries whose value is "off".
      self._torque_settings_btn.set_badges([(tr("enabled"), _on_off(enforce_torque)), (tr("jerk-aware"), _on_off(jerk_aware)),
                                            (tr("self-tune"), _on_off(self_tune_on)), (tr("custom-tuning"), _on_off(custom_on))])
    self._nnlc_toggle.set_enabled(torque_allowed and offroad and not enforce_torque and not jerk_aware)

    self._update_mads_state()
    self._update_torque_state(self_tune_on, custom_on)

  def _update_mads_state(self):
    # Apply safe defaults when the detected platform limits MADS operation.
    is_mads_limited = self._mads_limited = bool(ui_state.CP is not None and ui_state.CP_SP is not None and
                                                get_mads_limited_brands(ui_state.CP, ui_state.CP_SP, ui_state.params))
    if is_mads_limited and self._prev_mads_limited is not True:
      ui_state.params.remove("MadsMainCruiseAllowed")
      ui_state.params.put_bool("MadsUnifiedEngagementMode", True)
      ui_state.params.put("MadsSteeringMode", MadsSteeringModeOnBrake.DISENGAGE)
    self._prev_mads_limited = is_mads_limited

    # The platform lockout cannot be expressed as a control dependency.
    self._mads_steering.set_enabled(not is_mads_limited and ui_state.params.get_bool("Mads"))

  @staticmethod
  def _is_mazda() -> bool:
    return offroad_brand(ui_state.params, ui_state.CP, ui_state.is_offroad()) == "mazda"

  @staticmethod
  def _bsm_applies(alc_val: int) -> bool:
    """Return whether the selected lane-change mode uses the BSM delay."""
    return alc_val > AutoLaneChangeMode.NUDGE

  @staticmethod
  def _car_has_bsm() -> bool:
    return ui_state.CP is not None and ui_state.CP.enableBsm

  def _update_torque_state(self, self_tune_on: bool, custom_on: bool):
    # Controls refresh themselves; the parent only maintains summary badges.
    if not gui_app.widget_in_stack(self._tq_view):
      return

    if not self_tune_on:
      self._tq_self_tune_btn.set_disabled()
    else:
      self._tq_self_tune_btn.set_badges([(tr("enabled"), "on"), (tr("less-restrict"), _on_off(ui_state.params.get_bool("LiveTorqueParamsRelaxedToggle"))),
                                          (tr("speed-dependent"), _on_off(ui_state.params.get_bool("SpeedDependentTorqueToggle")))])

    if not custom_on:
      self._tq_custom_btn.set_disabled()
    else:
      manual_rt = _on_off(ui_state.params.get_bool("TorqueParamsOverrideEnabled"))
      # Float params return physical values and already fall back to declared defaults.
      lat_val = ui_state.params.get("TorqueParamsOverrideLatAccelFactor", return_default=True)
      fric_val = ui_state.params.get("TorqueParamsOverrideFriction", return_default=True)
      self._tq_custom_btn.set_badges([(tr("enabled"), "on"), (tr("realtime"), manual_rt), (f"{lat_val}m/s\u00b2", "on"), (str(fric_val), "on")])
