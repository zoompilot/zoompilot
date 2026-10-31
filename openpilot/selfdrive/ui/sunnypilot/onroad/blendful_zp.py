"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Lane center assist on the comma 4: a thin, muted center line that turns blue as the nudge grows, with
chevrons pointing the way it nudges. The cm readout only shows with BlendfulReadout (debug ui).
"""
import numpy as np
import pyray as rl
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.sunnypilot.onroad.blendful import BlendfulRenderer
from openpilot.selfdrive.ui.sunnypilot.ui_state import UIStateSP
from openpilot.sunnypilot.modeld_v2.blendful import LANE_WIDTH_CLIP
from openpilot.system.ui.lib.shader_polygon import draw_polygon

BLUE = rl.Color(0, 200, 255, 255)
IDLE = rl.Color(150, 160, 168, 255)  # the line with no nudge: thin and grey, so a nudge stands out
WHITE = rl.Color(255, 255, 255, 255)
IDLE_ALPHA = 0.45
LINE_HALF_WIDTH = 0.08  # m
COLOR_FULL = 0.2  # m of push for full color; sqrt, so a 5 cm push is already halfway
# chevrons on the line near the car, where the lane is widest on screen
CHEVRON_X = (7.0, 11.0, 15.0)  # m ahead
CHEVRON_LENGTH = 1.2  # m, half the forward extent of each arm pair
CHEVRON_SPAN = 0.4  # m, apex to arm end sideways, at full nudge
CHEVRON_THICKNESS = 0.14  # m
CHEVRON_MIN = 0.3  # nudge (0 to 1) below which they stay hidden, about 2 cm


def center_line(left: np.ndarray, right: np.ndarray, stds, blendful) -> np.ndarray:
  """The lane center Blendful steers toward, weighted by each line's confidence, as BlendfulRenderer draws it."""
  l_prob, r_prob = blendful.blendfulLeftProb, blendful.blendfulRightProb
  half_width = np.clip(right[:, 1] - left[:, 1], *LANE_WIDTH_CLIP) / 2 if l_prob * r_prob > 0.5 else blendful.blendfulLaneWidth / 2
  l_var = stds[1] ** 2 / max(l_prob, 1e-4)
  r_var = stds[2] ** 2 / max(r_prob, 1e-4)
  center = left.copy()
  center[:, 1] = (r_var * (left[:, 1] + half_width) + l_var * (right[:, 1] - half_width)) / (l_var + r_var)
  return center


def chevron_quads(center: np.ndarray, side: float, nudge: float) -> list[np.ndarray]:
  """Road-space quads, two arms per chevron, apex toward side (+1 right, -1 left)."""
  half = CHEVRON_SPAN * (0.6 + 0.4 * nudge) / 2
  back = np.array([0.0, -side * CHEVRON_THICKNESS, 0.0])
  quads = []
  for x0 in CHEVRON_X:
    if not center[0, 0] <= x0 <= center[-1, 0]:
      continue
    apex = np.array([x0, np.interp(x0, center[:, 0], center[:, 1]) + side * half, np.interp(x0, center[:, 0], center[:, 2])])
    for end_x in (x0 - CHEVRON_LENGTH, x0 + CHEVRON_LENGTH):
      end = np.array([end_x, apex[1] - 2 * side * half, np.interp(end_x, center[:, 0], center[:, 2])])
      quads.append(np.array([apex, end, end + back, apex + back]))
  return quads


def _wound(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> np.ndarray:
  # raylib culls clockwise triangles; the winding flips with the chevron's side
  return np.array([a, c, b] if (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) > 0 else [a, b, c])


class BlendfulRendererZP(BlendfulRenderer):
  def __init__(self, rect_relative: bool = False):
    super().__init__(readout=True, rect_relative=rect_relative)
    self._triangles: list[np.ndarray] = []  # screen space, wound for raylib
    self._line_color = IDLE
    self._chevron_color = WHITE

  def update(self, renderer, max_idx: int, *map_args) -> None:
    super().update(renderer, max_idx, *map_args)  # filters, trust and the fade; its wide line is redrawn thin below
    if not self._center_points.size:
      self._triangles = []
      return
    blendful = ui_state.sm['modelDataV2SP']
    if blendful.blendfulTrust <= 0.0 or blendful.blendfulLeftProb + blendful.blendfulRightProb <= 0.0:
      return  # the base kept the last line too
    self._triangles = []

    left, right = renderer._lane_lines[1].raw_points, renderer._lane_lines[2].raw_points
    center = center_line(left, right, list(ui_state.sm['modelV2'].laneLineStds), blendful)
    self._center_points = renderer._map_line_to_polygon(center, LINE_HALF_WIDTH, 0.0, max_idx, *map_args)

    pull = self._pull_display.x  # +y is right
    nudge = float(np.sqrt(np.clip(abs(pull) / COLOR_FULL, 0.0, 1.0)))
    line_alpha = np.clip(self._trust, 0.2, 1.0) * (IDLE_ALPHA + (1 - IDLE_ALPHA) * nudge)
    chevron_alpha = self._trust * max(nudge - CHEVRON_MIN, 0.0) / (1 - CHEVRON_MIN)
    self._line_color = rl.color_alpha(rl.color_lerp(IDLE, BLUE, nudge), line_alpha)
    self._chevron_color = rl.color_alpha(WHITE, chevron_alpha)
    if nudge > CHEVRON_MIN:
      for quad in chevron_quads(center, float(np.sign(pull)), nudge):
        proj = renderer._car_space_transform @ quad.T
        if np.all(proj[2] > 1e-6):
          a, b, c, d = (proj[:2] / proj[2]).T
          self._triangles += [_wound(a, b, c), _wound(a, c, d)]

  def draw(self, rect: rl.Rectangle) -> None:
    if not UIStateSP.lateral_active(ui_state) or not self._center_points.size:
      return
    if self._trust > 0.05:
      offset = np.array([rect.x, rect.y]) if self._rect_relative else np.zeros(2)
      fade = self._alpha.x  # the whole overlay fades in and out with trust
      draw_polygon(rect, self._center_points + offset, rl.color_alpha(self._line_color, self._line_color.a / 255 * fade))
      chevron = rl.color_alpha(self._chevron_color, self._chevron_color.a / 255 * fade)
      for tri in self._triangles:
        rl.draw_triangle(*(rl.Vector2(*(p + offset)) for p in tri), chevron)
    if ui_state.blendful_readout:
      self._draw_readout(rect)
