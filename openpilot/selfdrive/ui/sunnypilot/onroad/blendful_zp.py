"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Lane center assist on the comma 4 and 3X: a muted center line that turns blue as the nudge grows, a thin gauge
under the torque bar filled blue from its middle toward the side it nudges, and with debug ui a box with the push
in cm. The gauge is drawn in screen space, so it stays readable where road-space marks shrink to a few pixels,
and apart from the torque bar so planning's nudge is not read as steering torque.
"""
import math

import numpy as np
import pyray as rl
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.ui.mici.onroad.torque_bar import TORQUE_ANGLE_SPAN, TorqueBar, arc_bar_pts
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.selfdrive.ui.sunnypilot.onroad.blendful import FULL_BAR, BlendfulRenderer
from openpilot.selfdrive.ui.sunnypilot.ui_state import UIStateSP
from openpilot.sunnypilot.modeld_v2.blendful import LOOKAHEAD_TIME, MIN_LOOKAHEAD
from openpilot.system.ui.lib.shader_polygon import draw_polygon
from openpilot.system.ui.lib.text_measure import measure_text_cached

BLUE = rl.Color(0, 200, 255, 255)
IDLE = rl.Color(150, 160, 168, 255)  # the line with no nudge: grey, so a nudge stands out
IDLE_ALPHA = 0.45
COLOR_FULL = 0.2  # m of push for full color; sqrt, so a 5 cm push is already halfway
READOUT_W, READOUT_H = 190, 58  # px, the box without its bar; the base's with it is 76 tall

# TorqueBar._render keeps its geometry as locals: mirror any change to it here
TORQUE_RADIUS = 1200  # px at scale 1
TORQUE_VIEW_W = 476  # px, the camera view width its span is laid out for
TORQUE_CENTER_SHIFT = 8  # px right of the view's middle
TORQUE_OFFSET = 22  # px, its track's inner edge above the bottom of the view at rest; it only grows upward
# the gauge, just under that track
GAUGE_GAP = 4  # px
GAUGE_THICKNESS = 6  # px
GAUGE_DOT = 4  # px radius, a touch wider than the gauge like TorqueBar's dot on its track


def pull_at_lookahead(curvature: float, v_ego: float) -> float:
  """The push in m at the lookahead, as BlendfulRenderer's readout computes it."""
  lookahead = max(v_ego * LOOKAHEAD_TIME, MIN_LOOKAHEAD)
  return curvature * lookahead ** 2 / 2


def nudge_share(pull: float, full: float = COLOR_FULL) -> float:
  """0 to 1 of how hard lane center pushes, sqrt of the push up to full m."""
  return math.sqrt(min(abs(pull) / full, 1.0))


class BlendfulRendererZP(BlendfulRenderer):
  """The line colored by the nudge, and the debug ui readout box. Each device passes its readout layout: the
  comma 4 keeps the base's spot, the 3X draws it larger under its speed and keeps the box's bar only while its
  torque bar, and so the gauge's context, is hidden."""

  def __init__(self, rect_relative: bool = False, readout_scale: float = 1.0, readout_top: float = 10,
               readout_bar=lambda: False):
    super().__init__(readout=True, rect_relative=rect_relative)
    self._readout_scale = readout_scale
    self._readout_top = readout_top  # px below the top of the view
    self._readout_bar = readout_bar
    self._line_color = IDLE

  def update(self, renderer, max_idx: int, *map_args) -> None:
    super().update(renderer, max_idx, *map_args)  # the line, filters, trust and the fade
    if not self._center_points.size:
      return
    nudge = nudge_share(self._pull_display.x)
    line_alpha = np.clip(self._trust, 0.2, 1.0) * (IDLE_ALPHA + (1 - IDLE_ALPHA) * nudge)
    self._line_color = rl.color_alpha(rl.color_lerp(IDLE, BLUE, nudge), line_alpha)

  def draw(self, rect: rl.Rectangle) -> None:
    if not UIStateSP.lateral_active(ui_state) or not self._center_points.size:
      return
    if self._trust > 0.05:
      offset = np.array([rect.x, rect.y]) if self._rect_relative else np.zeros(2)
      fade = self._alpha.x  # the whole overlay fades in and out with trust
      draw_polygon(rect, self._center_points + offset, rl.color_alpha(self._line_color, self._line_color.a / 255 * fade))
    if ui_state.blendful_readout:
      # the box is drawn about its top center, then placed and scaled for this device
      rl.rl_push_matrix()
      rl.rl_translatef(rect.x + rect.width / 2, rect.y + self._readout_top, 0)
      rl.rl_scalef(self._readout_scale, self._readout_scale, 1)
      if self._readout_bar():
        super()._draw_readout(rl.Rectangle(-READOUT_W / 2, -10, READOUT_W, 0))  # the base sets its box 10 px down
      else:
        self._draw_readout_box()
      rl.rl_pop_matrix()

  def _draw_readout_box(self) -> None:
    """BlendfulRenderer's box, number and arrows, without its bar, about the origin."""
    pull = self._pull_display.x
    x, y = -READOUT_W / 2, 0
    cx, cy = 0, READOUT_H / 2
    rl.draw_rectangle_rounded(rl.Rectangle(x, y, READOUT_W, READOUT_H), 0.5, 12, self._color((0, 0, 0), 170))

    text = f"{abs(pull) * 100:.0f}"
    text_w = measure_text_cached(self._font, text, 44, 0).x
    rl.draw_text_ex(self._font, text, rl.Vector2(cx - text_w / 2, cy - 24), 44, 0, self._color((255, 255, 255), 255))

    active = self._color((BLUE.r, BLUE.g, BLUE.b), np.clip(abs(pull) / 0.1, 0.0, 1.0) * 255)
    idle = self._color((255, 255, 255), 40)
    lx, rx = x + 18, x + READOUT_W - 18
    rl.draw_triangle(rl.Vector2(lx, cy), rl.Vector2(lx + 16, cy + 12), rl.Vector2(lx + 16, cy - 12), active if pull < 0 else idle)
    rl.draw_triangle(rl.Vector2(rx - 16, cy - 12), rl.Vector2(rx - 16, cy + 12), rl.Vector2(rx, cy), active if pull > 0 else idle)


class TorqueBarZP(TorqueBar):
  """TorqueBar with a thin lane center gauge under its track: a faint track while lane center is active, filled
  blue from the middle toward the side it nudges. It scales and fades like the readout's bar: sqrt of the push
  up to FULL_BAR, brighter with trust. Its filters share BlendfulRenderer's 0.5 s time constant; the push reads
  zero while lane center has no trust, so the gauge empties as it lets go."""

  def __init__(self, *args, show_bar=lambda: True, **kwargs):
    super().__init__(*args, **kwargs)
    self._show_bar = show_bar  # the 3X hides the torque bar behind a setting; the gauge shows either way
    self._pull = FirstOrderFilter(0.0, 0.5, DT_MDL)
    self._trust_filter = FirstOrderFilter(0.0, 0.5, DT_MDL)
    self._active = False
    self._extent = 0.0  # -1 to 1 of the half span, + is right

  def _update_state(self):
    if self._show_bar():  # its torque filter only feeds the bar
      super()._update_state()
    blendful = ui_state.sm['modelDataV2SP']
    if ui_state.sm.updated['modelDataV2SP']:
      trusted = blendful.blendfulTrust > 0.0
      self._pull.update(pull_at_lookahead(blendful.blendfulCurvature, ui_state.sm['carState'].vEgo) if trusted else 0.0)
      self._trust_filter.update(blendful.blendfulTrust)
    self._active = blendful.blendfulTrust > 0.05 and UIStateSP.lateral_active(ui_state)
    self._extent = math.copysign(nudge_share(self._pull.x, FULL_BAR), self._pull.x)

  def _render(self, rect: rl.Rectangle) -> None:
    if self._show_bar():
      super()._render(rect)
    else:  # TorqueBar._render runs the fade the gauge uses
      self._torque_line_alpha_filter.update(ui_state.status not in (UIStatus.DISENGAGED, UIStatus.LONG_ONLY))
    if not self._active:
      return
    # TorqueBar's circle, so the gauge runs parallel to its track and grows from the same middle
    fade = self._torque_line_alpha_filter.x
    half_span = fade * TORQUE_ANGLE_SPAN * min(1.0, rect.width / (TORQUE_VIEW_W * self._scale)) / 2
    radius = TORQUE_RADIUS * self._scale
    center = np.array([rect.x + rect.width / 2 + TORQUE_CENTER_SHIFT, rect.y + rect.height + radius - TORQUE_OFFSET * self._scale],
                      dtype=np.float32)
    r_mid = radius - (GAUGE_GAP + GAUGE_THICKNESS / 2) * self._scale
    thickness = GAUGE_THICKNESS * self._scale
    track = arc_bar_pts(r_mid, thickness, -90 - half_span, -90 + half_span, cap_radius=thickness / 2)
    draw_polygon(rect, track + center, rl.Color(255, 255, 255, int(255 * 0.15 * fade)))
    if abs(half_span * self._extent) * np.pi / 180 * radius >= 1:  # over a pixel, like the readout's bar
      fill = arc_bar_pts(r_mid, thickness, -90, -90 + half_span * self._extent, cap_radius=thickness / 2)
      draw_polygon(rect, fill + center, rl.color_alpha(BLUE, np.clip(0.3 + 0.7 * self._trust_filter.x, 0.0, 1.0) * fade))
    # the zero mark: TorqueBar's grey center dot, sized to the gauge
    rl.draw_circle(int(center[0]), int(center[1] - r_mid), GAUGE_DOT * self._scale, rl.Color(182, 182, 182, int(255 * 0.9 * fade)))
