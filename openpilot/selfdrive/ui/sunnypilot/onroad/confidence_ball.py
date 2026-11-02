"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

mici's confidence ball on tizi, behind the ConfidenceBall toggle. mici's ConfidenceBall drives it (the
confidence filter, its status and lat/long-only handling) and its colors are mici's; it rides a track on
the driver's side at twice mici's size, from under the MAX box down to the car and bars, in a column
with them and the driver monitor, inside a frame styled like the MAX box, standing in for mici's side panel.
mici hides it below that panel while disengaged; here it sinks to the frame's bottom and fades out.
"""
import math

import numpy as np
import pyray as rl

from openpilot.selfdrive.ui.mici.onroad.confidence_ball import ConfidenceBall
from openpilot.selfdrive.ui.onroad.hud_renderer import COLORS, UI_CONFIG
from openpilot.selfdrive.ui.sunnypilot.onroad.driver_state import driver_monitor_rect
from openpilot.selfdrive.ui.sunnypilot.onroad.long_indicator import SCALE, STACK_GAP, long_indicator_top
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.shader_polygon import Gradient, draw_polygon

RADIUS = 24 * SCALE  # mici's status_dot_radius
FRAME_PAD = 8  # px of black around the ball
SEGMENTS = 48

# the unit disc as draw_polygon's ribbon [L0..Lk-1, Rk-1..R0], each arc bottom to top like the model's paths
_ARC = np.linspace(math.pi / 2, -math.pi / 2, SEGMENTS)
_UNIT_DISC = np.vstack([np.column_stack([-np.cos(_ARC), np.sin(_ARC)]),
                        np.column_stack([np.cos(_ARC), np.sin(_ARC)])[::-1]]).astype(np.float32)


def _ball_colors(confidence: float) -> tuple[rl.Color, rl.Color]:
  """ConfidenceBall._render's zones, top and bottom of the gradient."""
  if ui_state.status == UIStatus.ENGAGED:
    if confidence > 0.5:
      return rl.Color(0, 255, 204, 255), rl.Color(0, 255, 38, 255)
    if confidence > 0.2:
      return rl.Color(255, 200, 0, 255), rl.Color(255, 115, 0, 255)
    return rl.Color(255, 0, 21, 255), rl.Color(255, 0, 89, 255)
  if ui_state.status in (UIStatus.LAT_ONLY, UIStatus.LONG_ONLY):
    color = ConfidenceBall.get_lat_long_dot_color()
    return color, color
  if ui_state.status == UIStatus.OVERRIDE:
    return rl.Color(255, 255, 255, 255), rl.Color(82, 82, 82, 255)
  return rl.Color(50, 50, 50, 255), rl.Color(13, 13, 13, 255)


def draw_ball(cx: float, cy: float, radius: float, top: rl.Color, bottom: rl.Color, alpha: float) -> None:
  """A vertically graded disc. mici paints a square's corners away in black over its black side panel;
  over the camera that would show, so this draws the disc itself, as a ribbon of its left and right arcs."""
  box = rl.Rectangle(cx - radius, cy - radius, 2 * radius, 2 * radius)
  gradient = Gradient(start=(0.5, 0.0), end=(0.5, 1.0), colors=[rl.color_alpha(top, alpha), rl.color_alpha(bottom, alpha)],
                      stops=[0.0, 1.0])
  draw_polygon(box, _UNIT_DISC * np.float32(radius) + np.array([cx, cy], dtype=np.float32), gradient=gradient)


class ConfidenceBallTizi(ConfidenceBall):
  def _render(self, rect: rl.Rectangle) -> None:
    is_rhd = ui_state.sm['driverMonitoringState'].isRHD
    monitor = driver_monitor_rect(rect, is_rhd)
    cx = monitor.x + monitor.width / 2
    top = rect.y + UI_CONFIG.header_height
    bottom = long_indicator_top(rect, is_rhd) - STACK_GAP
    half = RADIUS + FRAME_PAD
    frame = rl.Rectangle(cx - half, top, 2 * half, bottom - top)
    rl.draw_rectangle_rounded(frame, 1.0, 24, COLORS.BLACK_TRANSLUCENT)
    rl.draw_rectangle_rounded_lines_ex(frame, 1.0, 24, 6, COLORS.BORDER_TRANSLUCENT)
    # mici's filter runs -0.5 (disengaged) to 1 (sure): above 0 it rides the track, below it fades out at the bottom
    confidence = self._confidence_filter.x
    alpha = min(1.0, max(0.0, 2 * confidence + 1))
    if alpha < 0.01:
      return
    cy = top + half + (1 - max(confidence, 0.0)) * (bottom - top - 2 * half)
    draw_ball(cx, cy, RADIUS, *_ball_colors(confidence), alpha)
