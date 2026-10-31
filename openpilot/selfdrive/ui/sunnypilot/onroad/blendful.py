"""
Copyright (c) 2021-, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
import pyray as rl
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.sunnypilot.ui_state import UIStateSP
from openpilot.sunnypilot.modeld_v2.blendful import LANE_WIDTH_CLIP, LOOKAHEAD_TIME, MIN_LOOKAHEAD
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.shader_polygon import draw_polygon
from openpilot.system.ui.lib.text_measure import measure_text_cached

BLENDFUL_COLOR = (0, 200, 255)
IDLE_COLOR = (150, 160, 168)  # the line with no nudge: thin and grey, so a nudge stands out
IDLE_ALPHA = 0.45
FULL_BAR = 1.0
LINE_HALF_WIDTH = 0.08  # m
COLOR_FULL = 0.2  # m of push for full color; sqrt, so a 5 cm push is already halfway
# chevrons on the line near the car, where the lane is widest on screen, point the way it nudges
CHEVRON_X = (7.0, 11.0, 15.0)  # m ahead
CHEVRON_LENGTH = 1.2  # m, half the forward extent of each arm pair
CHEVRON_SPAN = 0.4  # m, apex to arm end sideways, at full nudge
CHEVRON_THICKNESS = 0.14  # m
CHEVRON_MIN = 0.3  # nudge (0 to 1) below which they stay hidden, about 2 cm


def chevron_quads(center: np.ndarray, side: float, nudge: float) -> list[np.ndarray]:
  """Road-space quads, two arms per chevron, apex toward side (+1 right, -1 left)."""
  span = CHEVRON_SPAN * (0.6 + 0.4 * nudge)
  quads = []
  for x0 in CHEVRON_X:
    if not center[0, 0] <= x0 <= center[-1, 0]:
      continue
    y0, z0 = np.interp(x0, center[:, 0], center[:, 1]), np.interp(x0, center[:, 0], center[:, 2])
    apex = np.array([x0, y0 + side * span / 2, z0])
    for end_x in (x0 - CHEVRON_LENGTH, x0 + CHEVRON_LENGTH):
      end = np.array([end_x, y0 - side * span / 2, np.interp(end_x, center[:, 0], center[:, 2])])
      back = np.array([0.0, -side * CHEVRON_THICKNESS, 0.0])
      quads.append(np.array([apex, end, end + back, apex + back]))
  return quads


def _draw_triangle(a: rl.Vector2, b: rl.Vector2, c: rl.Vector2, color: rl.Color) -> None:
  # raylib culls clockwise triangles; the winding flips with the chevron's side
  if (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) > 0:
    b, c = c, b
  rl.draw_triangle(a, b, c, color)


class BlendfulRenderer:
  def __init__(self, readout: bool = True, rect_relative: bool = False):
    self._readout = readout
    self._rect_relative = rect_relative
    self._center_points = np.empty((0, 2), dtype=np.float32)
    self._chevrons: list[np.ndarray] = []
    self._nudge = 0.0  # 0 to 1: how far the color has gone from idle toward full
    self._trust = 0.0
    self._font = gui_app.font(FontWeight.BOLD)
    self._pull_display = FirstOrderFilter(0.0, 0.5, DT_MDL)
    self._trust_display = FirstOrderFilter(0.0, 0.5, DT_MDL)
    self._alpha = FirstOrderFilter(0.0, 0.3, DT_MDL)

  def update(self, renderer, max_idx: int, *map_args) -> None:
    blendful = ui_state.sm['modelDataV2SP']
    if ui_state.sm.updated['modelDataV2SP']:
      self._alpha.update(float(blendful.blendfulTrust > 0.0))
      if blendful.blendfulTrust > 0.0:
        self._trust = blendful.blendfulTrust
      lookahead = max(ui_state.sm['carState'].vEgo * LOOKAHEAD_TIME, MIN_LOOKAHEAD)
      self._pull_display.update(blendful.blendfulCurvature * lookahead ** 2 / 2)
      self._trust_display.update(self._trust)

    left, right = renderer._lane_lines[1].raw_points, renderer._lane_lines[2].raw_points
    stds = list(ui_state.sm['modelV2'].laneLineStds)
    if self._alpha.x < 0.01 or len(left) == 0 or len(right) != len(left) or len(stds) < 3:
      self._center_points = np.empty((0, 2), dtype=np.float32)
      self._chevrons = []
      return

    l_prob, r_prob = blendful.blendfulLeftProb, blendful.blendfulRightProb
    if blendful.blendfulTrust > 0.0 and l_prob + r_prob > 0.0:
      half_width = np.clip(right[:, 1] - left[:, 1], *LANE_WIDTH_CLIP) / 2 if l_prob * r_prob > 0.5 else blendful.blendfulLaneWidth / 2
      l_var = stds[1] ** 2 / max(l_prob, 1e-4)
      r_var = stds[2] ** 2 / max(r_prob, 1e-4)
      center = left.copy()
      center[:, 1] = (r_var * (left[:, 1] + half_width) + l_var * (right[:, 1] - half_width)) / (l_var + r_var)
      self._center_points = renderer._map_line_to_polygon(center, LINE_HALF_WIDTH, 0.0, max_idx, *map_args)
      pull = self._pull_display.x  # +y is right
      self._nudge = float(np.sqrt(np.clip(abs(pull) / COLOR_FULL, 0.0, 1.0)))
      self._chevrons = []
      if self._nudge > CHEVRON_MIN:
        for quad in chevron_quads(center, float(np.sign(pull)), self._nudge):
          proj = renderer._car_space_transform @ quad.T
          if np.all(proj[2] > 1e-6):
            self._chevrons.append((proj[:2] / proj[2]).T)

  def _color(self, rgb, alpha: float) -> rl.Color:
    return rl.Color(*rgb, int(alpha * self._alpha.x))

  def draw(self, rect: rl.Rectangle) -> None:
    if not UIStateSP.lateral_active(ui_state):
      return
    if self._center_points.size and self._trust > 0.05:
      offset = (rect.x, rect.y) if self._rect_relative else (0, 0)
      rgb = tuple(int(i + (b - i) * self._nudge) for i, b in zip(IDLE_COLOR, BLENDFUL_COLOR, strict=True))
      draw_polygon(rect, self._center_points + offset, self._color(rgb, np.clip(self._trust, 0.2, 1.0) * (IDLE_ALPHA + (1 - IDLE_ALPHA) * self._nudge) * 255))
      chevron = self._color((255, 255, 255), self._trust * (self._nudge - CHEVRON_MIN) / (1 - CHEVRON_MIN) * 255)
      for quad in self._chevrons:
        a, b, c, d = (rl.Vector2(float(x + offset[0]), float(y + offset[1])) for x, y in quad)
        _draw_triangle(a, b, c, chevron)
        _draw_triangle(a, c, d, chevron)
    if self._center_points.size and self._readout and ui_state.blendful_readout:
      self._draw_readout(rect)

  def _draw_readout(self, rect: rl.Rectangle) -> None:
    pull, trust = self._pull_display.x, self._trust_display.x
    w, h = 190, 76
    x, y = rect.x + (rect.width - w) / 2, rect.y + 10
    cx, cy = x + w / 2, y + 30
    rl.draw_rectangle_rounded(rl.Rectangle(x, y, w, h), 0.5, 12, self._color((0, 0, 0), 170))

    text = f"{abs(pull) * 100:.0f}"
    text_w = measure_text_cached(self._font, text, 44, 0).x
    rl.draw_text_ex(self._font, text, rl.Vector2(cx - text_w / 2, cy - 24), 44, 0, self._color((255, 255, 255), 255))

    active = self._color(BLENDFUL_COLOR, np.clip(abs(pull) / 0.1, 0.0, 1.0) * 255)
    idle = self._color((255, 255, 255), 40)
    lx, rx = x + 18, x + w - 18
    rl.draw_triangle(rl.Vector2(lx, cy), rl.Vector2(lx + 16, cy + 12), rl.Vector2(lx + 16, cy - 12), active if pull < 0 else idle)
    rl.draw_triangle(rl.Vector2(rx - 16, cy - 12), rl.Vector2(rx - 16, cy + 12), rl.Vector2(rx, cy), active if pull > 0 else idle)

    bar = rl.Rectangle(x + 24, y + h - 16, w - 48, 6)
    rl.draw_rectangle_rounded(bar, 1.0, 6, self._color((255, 255, 255), 40))
    rl.draw_rectangle(int(cx - 1), int(bar.y - 2), 2, int(bar.height + 4), self._color((255, 255, 255), 120))
    fill = bar.width / 2 * float(np.sqrt(np.clip(abs(pull) / FULL_BAR, 0.0, 1.0)))
    if fill > 1:
      fill_x = cx if pull > 0 else cx - fill
      rl.draw_rectangle_rounded(rl.Rectangle(fill_x, bar.y, fill, bar.height), 1.0, 6,
                                self._color(BLENDFUL_COLOR, np.clip(0.3 + 0.7 * trust, 0.0, 1.0) * 255))
