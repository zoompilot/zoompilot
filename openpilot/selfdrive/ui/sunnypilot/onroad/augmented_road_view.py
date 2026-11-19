"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
import raylib_nb as rl
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.ui_state import UIStatus, ui_state
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.polygon import Gradient, draw_polygon

BORDER_COLORS_SP = {
  UIStatus.LAT_ONLY: rl.Color(0x00, 0xC8, 0xC8, 0xFF),  # Cyan for lateral-only state
  UIStatus.LONG_ONLY: rl.Color(0x96, 0x1C, 0xA8, 0xFF),  # Purple for longitudinal-only state
}

# The bottom fade is a gradient, not onroad_fade.png stretched over the view: the UI renders on the
# CPU, where blending a view-sized texture costs more than the rest of the frame put together.
# These follow the texture's mean alpha by row; above its lower half it is all but clear.
FADE_HEIGHT = 0.5  # of the content rect, from the bottom
FADE_STOPS = [0.0, 0.25, 0.5, 0.75, 1.0]  # 0 is the bottom edge
FADE_ALPHAS = [246, 184, 85, 16, 0]


def bottom_fade_rect(content_rect: rl.Rectangle) -> rl.Rectangle:
  height = content_rect.height * FADE_HEIGHT
  return rl.Rectangle(content_rect.x, content_rect.y + content_rect.height - height, content_rect.width, height)


def bottom_fade_gradient(alpha: float) -> Gradient:
  # the first stop is at `end`
  return Gradient(start=(0.0, 1.0), end=(0.0, 0.0), stops=FADE_STOPS,
                  colors=[rl.Color(0, 0, 0, int(a * alpha)) for a in FADE_ALPHAS])


class AugmentedRoadViewSP:
  def __init__(self):
    self._fade_alpha_filter = FirstOrderFilter(0, 0.1, 1 / gui_app.target_fps)
    # a tap on the experimental button must not also toggle the sidebar; the HUD renders, and the
    # button takes the press, before this view handles it
    self.set_touch_valid_callback(lambda: not self._hud_renderer.user_interacting())

  def update_fade_out_bottom_overlay(self, _content_rect):
    # Fade out bottom of overlays for looks (only when engaged)
    fade_alpha = self._fade_alpha_filter.update(ui_state.status != UIStatus.DISENGAGED)
    if ui_state.torque_bar and fade_alpha > 1e-2:
      fade = bottom_fade_rect(_content_rect)
      corners = np.array([[fade.x, fade.y], [fade.x + fade.width, fade.y],
                          [fade.x + fade.width, fade.y + fade.height], [fade.x, fade.y + fade.height]], dtype=np.float32)
      # a gradient is laid out with y up from the bottom of the screen, so give it the fade's rect mirrored
      mirrored = rl.Rectangle(fade.x, gui_app.height - (fade.y + fade.height), fade.width, fade.height)
      draw_polygon(mirrored, corners, gradient=bottom_fade_gradient(fade_alpha))
