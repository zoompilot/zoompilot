"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Lane center assist on the comma 4: a muted center line that turns blue as the nudge grows, and a solid
blue stripe down the torque bar's track from its middle toward the side it nudges. The stripe is drawn in
screen space, after the torque bar, so it stays readable where road-space marks shrink to a few pixels
and nothing washes it out. The cm readout only shows with BlendfulReadout (debug ui).
"""
import numpy as np
import pyray as rl
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.ui.mici.onroad.torque_bar import TORQUE_ANGLE_SPAN, TorqueBar, arc_bar_pts
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.sunnypilot.onroad.blendful import BlendfulRenderer
from openpilot.selfdrive.ui.sunnypilot.ui_state import UIStateSP
from openpilot.sunnypilot.modeld_v2.blendful import LOOKAHEAD_TIME, MIN_LOOKAHEAD
from openpilot.system.ui.lib.shader_polygon import draw_polygon

BLUE = rl.Color(0, 200, 255, 255)
IDLE = rl.Color(150, 160, 168, 255)  # the line with no nudge: grey, so a nudge stands out
IDLE_ALPHA = 0.45
COLOR_FULL = 0.2  # m of push for full color; sqrt, so a 5 cm push is already halfway
STRIPE_SHARE = 0.55  # of the track's height, so the white torque fill still shows above and below
STRIPE_MIN = 0.3  # nudge (0 to 1) below which the track stays plain, about 2 cm


def nudge_share(pull: float) -> float:
  """0 to 1 of how hard lane center pushes, from the push in m at the lookahead."""
  return float(np.sqrt(np.clip(abs(pull) / COLOR_FULL, 0.0, 1.0)))


class BlendfulRendererZP(BlendfulRenderer):
  def __init__(self, rect_relative: bool = False):
    super().__init__(readout=True, rect_relative=rect_relative)
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
      self._draw_readout(rect)


class TorqueBarZP(TorqueBar):
  """TorqueBar with lane center's nudge as a solid blue stripe down its track, from the middle toward the
  side it nudges. The pull filter matches BlendfulRenderer's, so the stripe and the line agree."""

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    self._pull = FirstOrderFilter(0.0, 0.5, DT_MDL)
    self._extent = 0.0  # -1 to 1 of the half span, + is right

  def _update_state(self):
    super()._update_state()
    blendful = ui_state.sm['modelDataV2SP']
    if ui_state.sm.updated['modelDataV2SP']:
      lookahead = max(ui_state.sm['carState'].vEgo * LOOKAHEAD_TIME, MIN_LOOKAHEAD)
      self._pull.update(blendful.blendfulCurvature * lookahead ** 2 / 2 if blendful.blendfulTrust > 0.0 else 0.0)
    nudge = nudge_share(self._pull.x)
    active = blendful.blendfulTrust > 0.05 and UIStateSP.lateral_active(ui_state)
    self._extent = float(np.sign(self._pull.x)) * nudge if active and nudge > STRIPE_MIN else 0.0

  def _render(self, rect: rl.Rectangle) -> None:
    super()._render(rect)
    if not self._extent:
      return
    # the track's geometry as TorqueBar._render lays it out, which grows with torque past half
    torque = abs(self._torque_filter.x)
    offset = np.interp(torque, [0.5, 1], [22 * self._scale, 26 * self._scale])
    height = np.interp(torque, [0.5, 1], [14 * self._scale, 56 * self._scale])
    radius = 1200 * self._scale
    half_span = self._torque_line_alpha_filter.x * TORQUE_ANGLE_SPAN * min(1.0, rect.width / (476 * self._scale)) / 2
    center = np.array([rect.x + rect.width / 2 + 8, rect.y + rect.height + radius - offset], dtype=np.float32)
    thickness = height * STRIPE_SHARE
    pts = arc_bar_pts(radius + height / 2, thickness, -90, -90 + half_span * self._extent, cap_radius=thickness / 2)
    draw_polygon(rect, pts + center, rl.color_alpha(BLUE, self._torque_line_alpha_filter.x))
