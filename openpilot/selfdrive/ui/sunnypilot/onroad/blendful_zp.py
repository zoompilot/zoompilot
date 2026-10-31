"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Lane center assist on the comma 4: a muted center line that turns blue as the nudge grows, and the
torque bar's track painted blue from its middle toward the side it nudges. The track is drawn in screen
space, so it stays readable where road-space marks shrink to a few pixels; TorqueBar draws its own
track and fill over it after this. The cm readout only shows with BlendfulReadout (debug ui).
"""
import numpy as np
import pyray as rl
from openpilot.selfdrive.ui.mici.onroad.torque_bar import TORQUE_ANGLE_SPAN, arc_bar_pts
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.sunnypilot.onroad.blendful import BlendfulRenderer
from openpilot.selfdrive.ui.sunnypilot.ui_state import UIStateSP
from openpilot.system.ui.lib.shader_polygon import draw_polygon

BLUE = rl.Color(0, 200, 255, 255)
IDLE = rl.Color(150, 160, 168, 255)  # the line with no nudge: grey, so a nudge stands out
IDLE_ALPHA = 0.45
COLOR_FULL = 0.2  # m of push for full color; sqrt, so a 5 cm push is already halfway
# TorqueBar's track at rest: radius 1200, inner edge 22 px above the bottom of the view, 14 px tall, 7 px caps
TORQUE_RADIUS = 1200
TORQUE_OFFSET = 22  # px
TORQUE_HEIGHT = 14  # px
TORQUE_CAP = 7  # px
ARC_ALPHA = 0.9  # solid once shown; trust already decides whether it shows
ARC_MIN = 0.3  # nudge (0 to 1) below which the track stays plain, about 2 cm


class BlendfulRendererZP(BlendfulRenderer):
  def __init__(self, rect_relative: bool = False):
    super().__init__(readout=True, rect_relative=rect_relative)
    self._line_color = IDLE
    self._arc_extent = 0.0  # -1 to 1 of the half span, + is right

  def update(self, renderer, max_idx: int, *map_args) -> None:
    super().update(renderer, max_idx, *map_args)  # the line, filters, trust and the fade
    if not self._center_points.size:
      self._arc_extent = 0.0
      return
    pull = self._pull_display.x  # +y is right
    nudge = float(np.sqrt(np.clip(abs(pull) / COLOR_FULL, 0.0, 1.0)))
    line_alpha = np.clip(self._trust, 0.2, 1.0) * (IDLE_ALPHA + (1 - IDLE_ALPHA) * nudge)
    self._line_color = rl.color_alpha(rl.color_lerp(IDLE, BLUE, nudge), line_alpha)
    self._arc_extent = float(np.sign(pull)) * nudge if nudge > ARC_MIN else 0.0

  def draw(self, rect: rl.Rectangle) -> None:
    if not UIStateSP.lateral_active(ui_state) or not self._center_points.size:
      return
    if self._trust > 0.05:
      offset = np.array([rect.x, rect.y]) if self._rect_relative else np.zeros(2)
      fade = self._alpha.x  # the whole overlay fades in and out with trust
      draw_polygon(rect, self._center_points + offset, rl.color_alpha(self._line_color, self._line_color.a / 255 * fade))
      if self._arc_extent:
        self._draw_arc(rect, rl.color_alpha(BLUE, ARC_ALPHA * fade))
    if ui_state.blendful_readout:
      self._draw_readout(rect)

  def _draw_arc(self, rect: rl.Rectangle, color: rl.Color) -> None:
    # TorqueBar's circle and center, so the blue grows from the middle of the track the same way its fill does
    half_span = TORQUE_ANGLE_SPAN * min(1.0, rect.width / 476) / 2
    center = np.array([rect.x + rect.width / 2 + 8, rect.y + rect.height + TORQUE_RADIUS - TORQUE_OFFSET], dtype=np.float32)
    pts = arc_bar_pts(TORQUE_RADIUS + TORQUE_HEIGHT / 2, TORQUE_HEIGHT, -90, -90 + half_span * self._arc_extent, cap_radius=TORQUE_CAP)
    draw_polygon(rect, pts + center, color)
