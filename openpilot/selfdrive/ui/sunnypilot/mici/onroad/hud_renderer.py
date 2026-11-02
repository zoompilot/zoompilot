"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pyray as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.mici.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.sunnypilot.onroad.blind_spot_indicators import BlindSpotIndicators
from openpilot.selfdrive.ui.sunnypilot.onroad.speed_assist_badge import draw_badge
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app

# zoompilot's Set Speed Assist badge mirrors the steering wheel in the bottom-right corner
SPEED_ASSIST_FONT_SIZE = 15
SPEED_ASSIST_ALPHA = 0.9


class HudRendererSP(HudRenderer):
  def __init__(self):
    super().__init__()
    self.blind_spot_indicators = BlindSpotIndicators()
    self._sp_engaged: bool = False
    self._speed_assist_alpha_filter = FirstOrderFilter(0.0, 0.1, 1 / gui_app.target_fps)

  def _update_state(self) -> None:
    super()._update_state()
    self.blind_spot_indicators.update()

    sp_engaged = ui_state.engaged
    if sp_engaged != self._sp_engaged:
      self._chestnut_fade_time = rl.get_time() if sp_engaged else 0
    self._sp_engaged = sp_engaged

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    self.blind_spot_indicators.render(rect)
    self._draw_speed_assist(rect)

  def _draw_speed_assist(self, rect: rl.Rectangle) -> None:
    # gives the corner to the right blind spot and to the model icon that shares it
    shown = (ui_state.speed_assist_active and not self.blind_spot_indicators.right_detected and
             self._chestnut_alpha_filter.x < 1e-2)
    alpha = self._speed_assist_alpha_filter.update(SPEED_ASSIST_ALPHA if shown else 0.0)
    if alpha < 1e-2:
      return
    wheel = self._txt_wheel
    draw_badge(rect.x + rect.width - 21 - wheel.width / 2, rect.y + rect.height - 14 - wheel.height / 2,
               SPEED_ASSIST_FONT_SIZE, alpha)

  def left_blind_spot_detected(self) -> bool:
    return self.blind_spot_indicators.left_detected

  def _has_blind_spot_detected(self) -> bool:

    return self.blind_spot_indicators.detected
