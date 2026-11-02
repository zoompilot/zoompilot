"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

import pyray as rl

from openpilot.selfdrive.ui.mici.layouts.home import MiciHomeLayout, HOME_PADDING
from openpilot.selfdrive.ui.sunnypilot.onroad.speed_assist_badge import LongitudinalIcon, SpeedAssistIcon
from openpilot.selfdrive.ui.ui_state import ui_state, ChestnutState
from openpilot.system.ui.lib.application import FontWeight, TextAlignment
from openpilot.system.ui.widgets.label import UnifiedLabel, gui_label

MODEL_FONT_SIZE = 32
MODEL_ROW_HEIGHT = 48  # upstream's footer row
MODEL_ICON_GAP = 24
MODEL_COLOR = rl.Color(255, 255, 255, int(255 * 0.9 * 0.65))


class MiciHomeLayoutSP(MiciHomeLayout):
  def __init__(self):
    super().__init__()
    self._openpilot_label = UnifiedLabel("zoompilot", font_size=88, font_weight=FontWeight.AUDIOWIDE, max_width=480, wrap_text=False)
    # upstream's retired long press, which switched experimental mode, switches Set Speed Assist
    self.set_long_press_callback(ui_state.toggle_speed_assist)
    # after the network icon, where upstream showed experimental mode: alpha longitudinal, then
    # Set Speed Assist, each 40 px of ink like upstream's footer icons
    self._long_icon = LongitudinalIcon()
    self._long_icon.set_visible(lambda: ui_state.has_longitudinal_control)
    self._speed_assist_icon = SpeedAssistIcon()
    self._speed_assist_icon.set_visible(lambda: ui_state.speed_assist_active)
    for i, icon in enumerate((self._long_icon, self._speed_assist_icon)):
      self._status_bar_layout.add_widget(icon)
      self._status_bar_layout.widgets.insert(2 + i, self._status_bar_layout.widgets.pop())

  def _set_chestnut_visibility(self):
    usb_connected = ui_state.usb_connected
    usb_unknown = ui_state.usb_unknown
    chestnut_state = ui_state.chestnut_state
    loading = chestnut_state == ChestnutState.LOADING
    # ready, and only a swap window away: steady rather than the loading pulse,
    # dim rather than the solid green of a model that is driving
    waiting = chestnut_state == ChestnutState.WAITING

    # the port is known while Jetlink holds it, whatever enumerated
    self._usb_icon.set_visible(usb_connected and usb_unknown and not ui_state.adb_blocked)
    self._chestnut_loading_icon.set_opacity(0.35 + 0.65 * (0.5 - 0.5 * math.cos(rl.get_time() * 6.0)))
    self._chestnut_loading_icon.set_visible(not usb_unknown and loading)
    self._chestnut_icon.set_opacity(0.5 if waiting else 1.0)
    self._chestnut_icon.set_visible(not usb_unknown and not loading and
                                    chestnut_state in (ChestnutState.READY, ChestnutState.ACTIVE,
                                                       ChestnutState.WAITING))
    self._chestnut_failed_icon.set_visible(not usb_unknown and chestnut_state in (ChestnutState.UNCOMPILED, ChestnutState.FAILED))

  def _render(self, rect):
    super()._render(rect)
    if ui_state.home_model_name:
      self._render_model_name(ui_state.home_model_name.lower())

  def _render_model_name(self, name: str):
    # right aligned in the footer between the last status icon (settings always shows) and the
    # alerts pill; it elides rather than overlap either
    last_icon = next(w for w in reversed(self._status_bar_layout.widgets) if w.is_visible)
    left = last_icon.rect.x + last_icon.rect.width + MODEL_ICON_GAP
    right = self.rect.x + self.rect.width - HOME_PADDING
    if self._alert_count_callback and self._alert_count_callback() > 0:
      right = self._alerts_pill.rect.x - HOME_PADDING
    if right > left:
      gui_label(rl.Rectangle(left, self.rect.y + self.rect.height - MODEL_ROW_HEIGHT, right - left, MODEL_ROW_HEIGHT),
                name, MODEL_FONT_SIZE, MODEL_COLOR, FontWeight.ROMAN, alignment=TextAlignment.RIGHT)
