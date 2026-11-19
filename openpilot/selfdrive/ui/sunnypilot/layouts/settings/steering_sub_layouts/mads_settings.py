"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from collections.abc import Callable
import raylib_nb as rl

from opendbc.sunnypilot.car.tesla.values import MadsScreenButtonType, TeslaFlagsSP
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.mads.helpers import MadsSteeringModeOnBrake
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.network import NavButton
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.sunnypilot.widgets.list_view import multiple_button_item_sp, toggle_item_sp

MADS_STEERING_MODE_OPTIONS = [
  (tr("Remain"), tr_noop("Remain: steering stays on.")),
  (tr("Pause"), tr_noop("Pause: steering pauses and resumes when you let go.")),
  (tr("Disengage"), tr_noop("Disengage: steering turns off.")),
]

MADS_MAIN_CRUISE_BASE_DESC = tr("Turning cruise on with its on/off button also turns steering on. "
                                + "Cars without a lane keep button need this on to steer.")
MADS_UNIFIED_ENGAGEMENT_MODE_BASE_DESC = tr("Setting or resuming a cruise speed also turns steering on. "
                                            + "Steering then stays on until you switch it off or turn off the car.")

MADS_TJA_BUTTON_DESC = tr("The wheel's TJA button becomes the only steering switch. The MRCC button then only controls cruise. "
                          + "Turn on only if your car has the button.")

STATUS_CHECK_COMPATIBILITY = tr("Start the vehicle to check vehicle compatibility.")
DEFAULT_TO_OFF = tr("This feature defaults to OFF, and does not allow selection due to vehicle limitations.")
DEFAULT_TO_ON = tr("This feature defaults to ON, and does not allow selection due to vehicle limitations.")
STATUS_DISENGAGE_ONLY = tr("This platform only supports Disengage mode due to vehicle limitations.")


class MadsSettingsLayout(Widget):
  def __init__(self, back_btn_callback: Callable):
    super().__init__()
    self._back_button = NavButton(tr("Back"))
    self._back_button.set_click_callback(back_btn_callback)
    self._initialize_items()
    self._scroller = Scroller(self.items, line_separator=True, spacing=0)

  def _initialize_items(self):
    self._main_cruise_toggle = toggle_item_sp(
      title=lambda: tr("Steer When Cruise On"),
      description=MADS_MAIN_CRUISE_BASE_DESC,
      param="MadsMainCruiseAllowed",
    )
    self._unified_engagement_toggle = toggle_item_sp(
      title=lambda: tr("Steer When Cruise Set"),
      description=MADS_UNIFIED_ENGAGEMENT_MODE_BASE_DESC,
      param="MadsUnifiedEngagementMode"
    )
    self._steering_mode = multiple_button_item_sp(
      param="MadsSteeringMode",
      title=lambda: tr("Steering on Brake"),
      description="",
      buttons=[opt[0] for opt in MADS_STEERING_MODE_OPTIONS],
      inline=False,
      button_width=350,
      callback=self._update_steering_mode_description,
    )

    self._tja_button_toggle = toggle_item_sp(
      title=lambda: tr("Mazda TJA Button"),
      description=MADS_TJA_BUTTON_DESC,
      param="MazdaTjaButton",
      enabled=ui_state.is_offroad,
    )
    self._tja_button_toggle.set_visible(lambda: ui_state.offroad_brand() == "mazda")

    self.items = [
      self._main_cruise_toggle,
      self._unified_engagement_toggle,
      self._steering_mode,
      self._tja_button_toggle,
    ]

  def _update_state(self):
    super()._update_state()
    self._update_toggles()

  def _render(self, rect):
    self._back_button.set_position(self._rect.x, self._rect.y + 20)
    self._back_button.render()
    # subtract button
    content_rect = rl.Rectangle(rect.x, rect.y + self._back_button.rect.height + 40, rect.width, rect.height - self._back_button.rect.height - 40)
    self._scroller.render(content_rect)

  def show_event(self):
    self._scroller.show_event()

  @staticmethod
  def _mads_limited_settings() -> bool:
    brand = ""
    if ui_state.is_offroad():
      bundle = ui_state.params.get("CarPlatformBundle")
      if bundle:
        brand = bundle.get("brand", "")
    if not brand:
      brand = ui_state.CP.brand if ui_state.CP is not None else ""

    if brand == "rivian":
      return True
    elif brand == "tesla":
      if ui_state.CP_SP is None or not ui_state.CP_SP.flags & TeslaFlagsSP.HAS_VEHICLE_BUS:
        return True
      screen_button = int(ui_state.params.get("TeslaMadsScreenButton", return_default=True))
      return screen_button == MadsScreenButtonType.OFF
    return False

  def _update_steering_mode_description(self, button_index: int):
    base_desc = tr("What steering does when you press the brake.")
    result = base_desc + "<br><br>"
    for opt in MADS_STEERING_MODE_OPTIONS:
      desc = "<b>" + opt[1] + "</b>" if button_index == MADS_STEERING_MODE_OPTIONS.index(opt) else opt[1]
      result += desc + "<br>"
    self._steering_mode.set_description(result)
    self._steering_mode.show_description(True)

  def _update_toggles(self):
    self._update_steering_mode_description(self._steering_mode.action_item.selected_button)
    if self._mads_limited_settings():
      ui_state.params.remove("MadsMainCruiseAllowed")
      ui_state.params.put_bool("MadsUnifiedEngagementMode", True)
      ui_state.params.put("MadsSteeringMode", MadsSteeringModeOnBrake.DISENGAGE)

      self._main_cruise_toggle.action_item.set_enabled(False)
      self._main_cruise_toggle.action_item.set_state(False)
      self._main_cruise_toggle.set_description("<b>" + DEFAULT_TO_OFF + "</b><br>" + MADS_MAIN_CRUISE_BASE_DESC)

      self._unified_engagement_toggle.action_item.set_enabled(False)
      self._unified_engagement_toggle.action_item.set_state(True)
      self._unified_engagement_toggle.set_description("<b>" + DEFAULT_TO_ON + "</b><br>" + MADS_UNIFIED_ENGAGEMENT_MODE_BASE_DESC)

      self._steering_mode.set_description(STATUS_DISENGAGE_ONLY)
      self._steering_mode.action_item.set_selected_button(MadsSteeringModeOnBrake.DISENGAGE)
      self._steering_mode.action_item.set_enabled_buttons({MadsSteeringModeOnBrake.DISENGAGE})
    else:
      self._main_cruise_toggle.action_item.set_enabled(True)
      self._main_cruise_toggle.set_description(MADS_MAIN_CRUISE_BASE_DESC)

      self._unified_engagement_toggle.action_item.set_enabled(True)
      self._unified_engagement_toggle.set_description(MADS_UNIFIED_ENGAGEMENT_MODE_BASE_DESC)

      self._steering_mode.action_item.set_enabled(True)
      self._steering_mode.action_item.set_enabled_buttons(None)
