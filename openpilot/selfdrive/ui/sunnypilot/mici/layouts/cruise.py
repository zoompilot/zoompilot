"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.selfdrive.ui.mici.widgets.button import BigParamControl
from openpilot.selfdrive.ui.sunnypilot.longitudinal_mode import alpha_longitudinal_reachable, longitudinal_mode_labels
from openpilot.selfdrive.ui.sunnypilot.mici.layouts.alpha_longitudinal import AlphaLongitudinalLayoutMici
from openpilot.selfdrive.ui.sunnypilot.mici.widgets.button import (
  BigButtonSP,
  BigMultiParamToggleSP,
  BigParamOption,
  speed_unit,
)
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets.scroller import NavScroller
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import icbm_applicable, icbm_moves_speed_limits

SL_MODE_LABELS = [tr("off"), tr("info"), tr("warn"), tr("assist")]
SL_SOURCE_LABELS = [tr("car"), tr("map"), tr("car-first"), tr("map-first"), tr("combined")]
SL_MODE_OFF = 0
SL_MODE_WARN = 2
SL_MODE_ASSIST = 3
ACC_LONG_PRESS_MAP = {1: 1, 2: 5, 3: 10}


def _offset_unit():
  t = int(ui_state.params.get("SpeedLimitOffsetType", return_default=True))
  if t == 2:
    return "%"
  if t == 1:
    return speed_unit()
  return ""


def _offset_label(value):
  unit = _offset_unit()
  return f"{value}{unit}" if unit else tr("none")


class CruiseLayoutMici(NavScroller):
  """Cruise settings: alpha longitudinal, SCC, custom ACC increments, speed limit assist.

  State gating pattern:
    - _update_state runs every frame, reads params and enables/disables widgets
    - _prev_* fields track state transitions (None → first frame, True → False = cleanup)
    - ICBM has no toggle: the features below bring it up wherever the car can press its
      cruise buttons (icbm_applicable)
  """

  def __init__(self):
    super().__init__()

    self._prev_sla_available: bool | None = None

    self._alpha_long_btn = BigButtonSP(tr("alpha longitudinal"))
    alpha_long_view = AlphaLongitudinalLayoutMici()
    self._alpha_long_btn.set_click_callback(lambda: gui_app.push_widget(alpha_long_view))
    self._scc_v_toggle = BigParamControl(tr("slow for curves: vision"), "SmartCruiseControlVision")
    self._scc_m_toggle = BigParamControl(tr("slow for curves: map"), "SmartCruiseControlMap")
    self._custom_acc_btn = BigButtonSP(tr("custom increments"))
    self._speed_limit_btn = BigButtonSP(tr("speed limit"))

    for btn in [self._alpha_long_btn, self._custom_acc_btn, self._speed_limit_btn]:
      btn.set_subtitle_font_size(24)

    self._scroller.add_widgets([
      self._alpha_long_btn,
      self._scc_v_toggle, self._scc_m_toggle,
      self._custom_acc_btn, self._speed_limit_btn,
    ])

    self._custom_acc_toggle = BigParamControl(tr("enable custom increments"), "CustomAccIncrementsEnabled")

    def _speed_label(v):
      return f"{v} {speed_unit()}"
    self._acc_short = BigParamOption(tr("short press"), "CustomAccShortPressIncrement",
                                     min_value=1, max_value=10, label_callback=_speed_label, picker_unit=speed_unit)
    self._acc_long = BigParamOption(tr("long press"), "CustomAccLongPressIncrement",
                                    min_value=1, max_value=3, value_map=ACC_LONG_PRESS_MAP,
                                    label_callback=_speed_label, picker_unit=speed_unit)
    self._acc_view = self._custom_acc_btn.link_sub_panel([self._custom_acc_toggle, self._acc_short, self._acc_long])

    self._sl_mode = BigMultiParamToggleSP(tr("speed limit mode"), "SpeedLimitMode", SL_MODE_LABELS)
    self._sl_source = BigMultiParamToggleSP(tr("source"), "SpeedLimitPolicy", SL_SOURCE_LABELS)
    self._sl_offset_type = BigMultiParamToggleSP(tr("offset type"), "SpeedLimitOffsetType", [tr("none"), tr("fixed"), "%"])
    self._sl_offset_value = BigParamOption(tr("offset value"), "SpeedLimitValueOffset",
                                           min_value=-30, max_value=30, label_callback=_offset_label, picker_unit=_offset_unit)
    self._sl_view = self._speed_limit_btn.link_sub_panel([self._sl_mode, self._sl_source, self._sl_offset_type, self._sl_offset_value])

  def _update_state(self):
    super()._update_state()

    self._scc_v_toggle.refresh()
    self._scc_m_toggle.refresh()

    cp_ready = ui_state.CP is not None and ui_state.CP_SP is not None
    has_long = cp_ready and ui_state.has_longitudinal_control
    has_icbm = cp_ready and icbm_applicable(ui_state.CP, ui_state.CP_SP)

    self._alpha_long_btn.set_enabled(alpha_longitudinal_reachable(ui_state))
    for toggle in (self._scc_v_toggle, self._scc_m_toggle):
      toggle.set_superseded(not (has_long or has_icbm))
    # has_long and has_icbm are False until CP is ready, so CP is not read before then
    custom_acc_available = (has_long and not ui_state.CP.pcmCruise) or has_icbm
    self._custom_acc_toggle.set_superseded(not custom_acc_available)
    self._custom_acc_btn.set_enabled(custom_acc_available)

    # Custom ACC button subtitle
    acc_on = ui_state.params.get_bool("CustomAccIncrementsEnabled")
    if not acc_on:
      self._custom_acc_btn.set_disabled()
    else:
      unit = speed_unit()
      short_val = ui_state.params.get("CustomAccShortPressIncrement", return_default=True) or 1
      long_raw = ui_state.params.get("CustomAccLongPressIncrement", return_default=True) or 1
      long_val = ACC_LONG_PRESS_MAP.get(long_raw, long_raw)
      self._custom_acc_btn.set_badges([(f"{short_val}{unit}", "on"), (f"{long_val}{unit}", "on")])

    # Alpha longitudinal button subtitle: the mode that drives
    if labels := longitudinal_mode_labels(ui_state):
      self._alpha_long_btn.set_badges([(label, "on") for label in labels])
    else:
      self._alpha_long_btn.set_disabled()

    # Speed limit button subtitle
    sl_mode_idx = ui_state.params.get("SpeedLimitMode", return_default=True) or 0
    sl_mode = SL_MODE_LABELS[min(sl_mode_idx, len(SL_MODE_LABELS) - 1)]
    offset_type = ui_state.params.get("SpeedLimitOffsetType", return_default=True)
    if sl_mode_idx == SL_MODE_OFF:
      self._speed_limit_btn.set_disabled()
    else:
      sl_source_idx = ui_state.params.get("SpeedLimitPolicy", return_default=True) or 0
      sl_source = SL_SOURCE_LABELS[min(sl_source_idx, len(SL_SOURCE_LABELS) - 1)]
      sl_offset_val = ui_state.params.get("SpeedLimitValueOffset", return_default=True) or 0
      unit = "%" if offset_type == 2 else (speed_unit() if offset_type == 1 else "")
      icbm = icbm_moves_speed_limits(has_long, has_icbm, sl_mode_idx)
      badges = [(sl_mode, "on"), (tr("icbm"), "on" if icbm else "off"), (sl_source, "on")]
      if unit:
        sign = "+" if sl_offset_val > 0 else ""
        badges.append((f"{sign}{sl_offset_val}{unit}", "on"))
      self._speed_limit_btn.set_badges(badges)

    self._update_custom_acc_state()
    self._update_speed_limit_state(cp_ready, has_long, has_icbm, offset_type)

  def _update_custom_acc_state(self):
    if not gui_app.widget_in_stack(self._acc_view):
      return
    self._custom_acc_toggle.refresh()
    self._acc_short.refresh()
    self._acc_long.refresh()
    self._custom_acc_toggle.set_enabled(True)  # card reads it live; ICBM follows at the next engage
    # Lambda: short/long respond same-frame when toggle is tapped (see button.py docstring)
    self._acc_short.set_enabled(lambda: self._custom_acc_btn.enabled and self._custom_acc_toggle._checked)
    self._acc_long.set_enabled(lambda: self._custom_acc_btn.enabled and self._custom_acc_toggle._checked)

  def _update_speed_limit_state(self, cp_ready: bool, has_long: bool, has_icbm: bool, offset_type: int):
    # SLA availability gating (must always run)
    sla_available = False
    if cp_ready:
      brand = ui_state.CP.brand
      sla_disallow_in_release = brand == "tesla" and ui_state.is_sp_release
      sla_always_disallow = brand == "rivian"
      sla_available = (has_long or has_icbm) and not sla_disallow_in_release and not sla_always_disallow

    # Downgrade unavailable assist mode to warning once per transition.
    if not sla_available and self._prev_sla_available is not False:
      sl_mode_idx = ui_state.params.get("SpeedLimitMode", return_default=True) or 0
      if sl_mode_idx == SL_MODE_ASSIST:
        ui_state.params.put("SpeedLimitMode", SL_MODE_WARN)
    self._prev_sla_available = sla_available

    if not gui_app.widget_in_stack(self._sl_view):
      return
    self._sl_mode.refresh()
    self._sl_source.refresh()
    self._sl_offset_type.refresh()
    self._sl_mode.set_enabled(True)
    # nothing reads these with the mode off
    sl_on = self._sl_mode.value != SL_MODE_LABELS[SL_MODE_OFF]
    self._sl_source.set_superseded(not sl_on)
    self._sl_offset_type.set_superseded(not sl_on)
    self._sl_offset_value.set_enabled(sl_on and offset_type != 0)  # 0 = off/none
    self._sl_offset_value.refresh()
