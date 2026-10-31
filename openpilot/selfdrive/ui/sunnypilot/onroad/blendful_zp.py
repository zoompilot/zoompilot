"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Lane center assist on the comma 4: a thin, muted center line that turns blue as the nudge grows, and a
blue arc above the torque bar that grows toward the side it nudges. The arc is drawn in screen space, so
it stays readable where road-space marks shrink to a few pixels. The cm readout only shows with
BlendfulReadout (debug ui).
"""
import numpy as np
import pyray as rl
from openpilot.selfdrive.ui.mici.onroad.torque_bar import TORQUE_ANGLE_SPAN, arc_bar_pts
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.sunnypilot.onroad.blendful import BlendfulRenderer
from openpilot.selfdrive.ui.sunnypilot.ui_state import UIStateSP
from openpilot.sunnypilot.modeld_v2.blendful import LANE_WIDTH_CLIP
from openpilot.system.ui.lib.shader_polygon import draw_polygon

BLUE = rl.Color(0, 200, 255, 255)
IDLE = rl.Color(150, 160, 168, 255)  # the line with no nudge: thin and grey, so a nudge stands out
IDLE_ALPHA = 0.45
LINE_HALF_WIDTH = 0.08  # m
COLOR_FULL = 0.2  # m of push for full color; sqrt, so a 5 cm push is already halfway
# the nudge arc sits just above TorqueBar's track, on the same circle: radius 1200, its inner edge 22 px
# above the bottom of the view and 14 px tall at rest
TORQUE_RADIUS = 1200
TORQUE_OFFSET = 22  # px
TORQUE_HEIGHT = 14  # px
ARC_GAP = 6  # px
ARC_THICKNESS = 8  # px
ARC_MIN = 0.3  # nudge (0 to 1) below which the arc stays hidden, about 2 cm


def center_line(left: np.ndarray, right: np.ndarray, stds, blendful) -> np.ndarray:
  """The lane center Blendful steers toward, weighted by each line's confidence, as BlendfulRenderer draws it."""
  l_prob, r_prob = blendful.blendfulLeftProb, blendful.blendfulRightProb
  half_width = np.clip(right[:, 1] - left[:, 1], *LANE_WIDTH_CLIP) / 2 if l_prob * r_prob > 0.5 else blendful.blendfulLaneWidth / 2
  l_var = stds[1] ** 2 / max(l_prob, 1e-4)
  r_var = stds[2] ** 2 / max(r_prob, 1e-4)
  center = left.copy()
  center[:, 1] = (r_var * (left[:, 1] + half_width) + l_var * (right[:, 1] - half_width)) / (l_var + r_var)
  return center


class BlendfulRendererZP(BlendfulRenderer):
  def __init__(self, rect_relative: bool = False):
    super().__init__(readout=True, rect_relative=rect_relative)
    self._line_color = IDLE
    self._arc_color = BLUE
    self._arc_extent = 0.0  # -1 to 1 of the half span, + is right

  def update(self, renderer, max_idx: int, *map_args) -> None:
    super().update(renderer, max_idx, *map_args)  # filters, trust and the fade; its wide line is redrawn thin below
    if not self._center_points.size:
      self._arc_extent = 0.0
      return
    blendful = ui_state.sm['modelDataV2SP']
    if blendful.blendfulTrust <= 0.0 or blendful.blendfulLeftProb + blendful.blendfulRightProb <= 0.0:
      return  # the base kept the last line too

    left, right = renderer._lane_lines[1].raw_points, renderer._lane_lines[2].raw_points
    center = center_line(left, right, list(ui_state.sm['modelV2'].laneLineStds), blendful)
    self._center_points = renderer._map_line_to_polygon(center, LINE_HALF_WIDTH, 0.0, max_idx, *map_args)

    pull = self._pull_display.x  # +y is right
    nudge = float(np.sqrt(np.clip(abs(pull) / COLOR_FULL, 0.0, 1.0)))
    line_alpha = np.clip(self._trust, 0.2, 1.0) * (IDLE_ALPHA + (1 - IDLE_ALPHA) * nudge)
    self._line_color = rl.color_alpha(rl.color_lerp(IDLE, BLUE, nudge), line_alpha)
    self._arc_color = rl.color_alpha(BLUE, self._trust)
    self._arc_extent = float(np.sign(pull)) * nudge if nudge > ARC_MIN else 0.0

  def draw(self, rect: rl.Rectangle) -> None:
    if not UIStateSP.lateral_active(ui_state) or not self._center_points.size:
      return
    if self._trust > 0.05:
      offset = np.array([rect.x, rect.y]) if self._rect_relative else np.zeros(2)
      fade = self._alpha.x  # the whole overlay fades in and out with trust
      draw_polygon(rect, self._center_points + offset, rl.color_alpha(self._line_color, self._line_color.a / 255 * fade))
      if self._arc_extent:
        self._draw_arc(rect, rl.color_alpha(self._arc_color, self._arc_color.a / 255 * fade))
    if ui_state.blendful_readout:
      self._draw_readout(rect)

  def _draw_arc(self, rect: rl.Rectangle, color: rl.Color) -> None:
    # TorqueBar's circle and center, so the arc grows from the middle of the track the same way its fill does
    half_span = TORQUE_ANGLE_SPAN * min(1.0, rect.width / 476) / 2
    center = np.array([rect.x + rect.width / 2 + 8, rect.y + rect.height + TORQUE_RADIUS - TORQUE_OFFSET], dtype=np.float32)
    r_mid = TORQUE_RADIUS + TORQUE_HEIGHT + ARC_GAP + ARC_THICKNESS / 2
    pts = arc_bar_pts(r_mid, ARC_THICKNESS, -90, -90 + half_span * self._arc_extent, cap_radius=ARC_THICKNESS / 2)
    draw_polygon(rect, pts + center, color)
