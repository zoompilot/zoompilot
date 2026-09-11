"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

import pyray as rl

from openpilot.selfdrive.ui.mici.layouts.home import MiciHomeLayout, HOME_PADDING
from openpilot.selfdrive.ui.sunnypilot.active_model import active_model_name
from openpilot.selfdrive.ui.ui_state import ui_state, ChestnutState
from openpilot.system.ui.lib.application import FontWeight, TextAlignment
from openpilot.system.ui.widgets.label import UnifiedLabel, gui_label

MODEL_FONT_SIZE = 32
MODEL_ROW_HEIGHT = 48
MODEL_ICON_GAP = 24
MODEL_COLOR = rl.Color(255, 255, 255, int(255 * 0.9 * 0.65))


class MiciHomeLayoutSP(MiciHomeLayout):
  def __init__(self):
    super().__init__()
    self._openpilot_label = UnifiedLabel("zoompilot", font_size=88, font_weight=FontWeight.AUDIOWIDE, max_width=480, wrap_text=False)

  def _set_chestnut_visibility(self):
    usb_connected = ui_state.usb_connected
    usb_unknown = ui_state.usb_unknown
    chestnut_state = ui_state.chestnut_state
    loading = chestnut_state == ChestnutState.LOADING

    self._usb_icon.set_visible(usb_connected and usb_unknown)
    self._chestnut_loading_icon.set_opacity(0.35 + 0.65 * (0.5 - 0.5 * math.cos(rl.get_time() * 6.0)))
    self._chestnut_loading_icon.set_visible(not usb_unknown and loading)
    self._chestnut_icon.set_visible(not usb_unknown and not loading and
                                    chestnut_state in (ChestnutState.READY, ChestnutState.ACTIVE))
    self._chestnut_failed_icon.set_visible(not usb_unknown and chestnut_state in (ChestnutState.UNCOMPILED, ChestnutState.FAILED))

  def _render(self, rect):
    super()._render(rect)
    if ui_state.home_show_active_model:
      self._render_model_name()

  def _render_model_name(self):
    icons = [widget for widget in self._status_bar_layout.widgets if widget.is_visible]
    left = icons[-1].rect.x + icons[-1].rect.width if icons else self.rect.x

    right = self.rect.x + self.rect.width - HOME_PADDING
    if self._alert_count_callback and self._alert_count_callback() > 0:
      right -= self._alerts_pill.rect.width + HOME_PADDING

    name_rect = rl.Rectangle(left + MODEL_ICON_GAP, self.rect.y + self.rect.height - MODEL_ROW_HEIGHT,
                             right - left - MODEL_ICON_GAP, MODEL_ROW_HEIGHT)
    if name_rect.width <= 0:
      return

    gui_label(name_rect, active_model_name().lower(), MODEL_FONT_SIZE, MODEL_COLOR, FontWeight.ROMAN,
              alignment=TextAlignment.RIGHT)
