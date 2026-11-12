"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Lane center assist on the comma 4 and 3X: a muted center line that turns blue as the nudge grows, and a thin
gauge under the torque bar filled blue from its middle toward the side it nudges. Both show whenever lane center
is on; the 3X can turn the gauge off (LaneCenterUI). The gauge is drawn in screen space, so it stays readable
where road-space marks shrink to a few pixels, and apart from the torque bar so planning's nudge is not read as
steering torque.
"""
import math

import numpy as np
import raylib_nb as rl
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.ui.mici.onroad.torque_bar import TORQUE_ANGLE_SPAN, TorqueBar, arc_bar_pts
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.selfdrive.ui.sunnypilot.onroad.blendful import BLENDFUL_COLOR, FULL_BAR, BlendfulRenderer
from openpilot.selfdrive.ui.sunnypilot.ui_state import UIStateSP
from openpilot.sunnypilot.modeld_v2.blendful import LOOKAHEAD_TIME, MIN_LOOKAHEAD
from openpilot.system.ui.lib.shader_polygon import draw_polygon

BLUE = rl.Color(*BLENDFUL_COLOR, 255)
IDLE = rl.Color(150, 160, 168, 255)  # the line with no nudge: grey, so a nudge stands out
IDLE_ALPHA = 0.45
COLOR_FULL = 0.2  # m of push for full color; sqrt, so a 5 cm push is already halfway

# TorqueBar._render keeps its geometry as locals: mirror any change to it here
TORQUE_RADIUS = 1200  # px at scale 1
TORQUE_VIEW_W = 476  # px, the camera view width its span is laid out for
TORQUE_CENTER_SHIFT = 8  # px right of the view's middle
TORQUE_OFFSET = 22  # px, its track's inner edge above the bottom of the view at rest; it only grows upward
# the gauge, just under that track
GAUGE_GAP = 2  # px
GAUGE_THICKNESS = 10  # px, a bit under the track's 14 at rest
GAUGE_DOT = 6  # px radius, a touch wider than the gauge like TorqueBar's dot on its track


def pull_at_lookahead(curvature: float, v_ego: float) -> float:
  """The push in m at the lookahead, as BlendfulRenderer computes it."""
  lookahead = max(v_ego * LOOKAHEAD_TIME, MIN_LOOKAHEAD)
  return curvature * lookahead ** 2 / 2


def color_lerp(a: rl.Color, b: rl.Color, t: float) -> rl.Color:
  """raylib's ColorLerp, which raylib_nb does not bind."""
  return rl.Color(int((1 - t) * a.r + t * b.r), int((1 - t) * a.g + t * b.g),
                  int((1 - t) * a.b + t * b.b), int((1 - t) * a.a + t * b.a))


def nudge_share(pull: float, full: float = COLOR_FULL) -> float:
  """0 to 1 of how hard lane center pushes, sqrt of the push up to full m."""
  return math.sqrt(min(abs(pull) / full, 1.0))


class BlendfulRendererZP(BlendfulRenderer):
  """The line colored by the nudge; draw replaces the base's, so its readout box never shows."""

  def __init__(self, rect_relative: bool = False):
    super().__init__(rect_relative=rect_relative)  # never readout=False: it also stops the push the color follows
    self._line_color = IDLE

  def update(self, renderer, max_idx: int, *map_args) -> None:
    super().update(renderer, max_idx, *map_args)  # the line, filters, trust and the fade
    if not self._center_points.size:
      return
    nudge = nudge_share(self._pull_display.x)
    line_alpha = np.clip(self._trust, 0.2, 1.0) * (IDLE_ALPHA + (1 - IDLE_ALPHA) * nudge)
    self._line_color = rl.fade(color_lerp(IDLE, BLUE, nudge), line_alpha)

  def draw(self, rect: rl.Rectangle) -> None:
    if not UIStateSP.lateral_active(ui_state) or not self._center_points.size:
      return
    if self._trust > 0.05:
      offset = np.array([rect.x, rect.y]) if self._rect_relative else np.zeros(2)
      fade = self._alpha.x  # the whole overlay fades in and out with trust
      draw_polygon(rect, self._center_points + offset, rl.fade(self._line_color, self._line_color.a / 255 * fade))


class TorqueBarZP(TorqueBar):
  """TorqueBar with a thin lane center gauge under its track: a faint track while lane center is active and
  lane center is on and `show_gauge`, filled blue from the middle toward the side it nudges. It scales like BlendfulRenderer's bar: sqrt
  of the push up to FULL_BAR, brighter with trust. Its filters share BlendfulRenderer's 0.5 s time constant; the push reads
  zero while lane center has no trust, so the gauge empties as it lets go."""

  def __init__(self, *args, show_bar=lambda: True, show_gauge=lambda: True, **kwargs):
    super().__init__(*args, **kwargs)
    self._show_bar = show_bar  # the 3X hides the torque bar behind a setting; the gauge shows either way
    self._show_gauge = show_gauge  # the 3X's LaneCenterUI
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
    self._active = blendful.blendfulTrust > 0.05 and UIStateSP.lateral_active(ui_state) and \
                    ui_state.lane_center and self._show_gauge()
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
    if abs(half_span * self._extent) * np.pi / 180 * radius >= 1:  # over a pixel
      fill = arc_bar_pts(r_mid, thickness, -90, -90 + half_span * self._extent, cap_radius=thickness / 2)
      draw_polygon(rect, fill + center, rl.fade(BLUE, np.clip(0.3 + 0.7 * self._trust_filter.x, 0.0, 1.0) * fade))
    # the zero mark: TorqueBar's grey center dot, sized to the gauge
    rl.draw_circle(int(center[0]), int(center[1] - r_mid), GAUGE_DOT * self._scale, rl.Color(182, 182, 182, int(255 * 0.9 * fade)))
