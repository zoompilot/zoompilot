"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Set Speed Assist's mark: two steep chevrons, the upper one narrower like the distance bars under them. On mici
they sit over the lead car in the side panel, grey while armed; the lift on the model's acceleration
fades the lower one green over the first half of the personality's approach ceiling and the upper one
over the second. Each chevron is one outline (round ends, a round outer tip), so a translucent colour
lays each pixel once.
"""
import math
from functools import lru_cache

import numpy as np
import raylib_nb as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.approach import approach_ceiling
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.polygon import draw_polygon

# at scale 1: the pair is 21.5 px of ink, sized to the side panel's lead car (35 wide). Steep enough to
# read as up rather than a brow at that size, the stroke near the car's outline
LOWER_WIDTH, UPPER_WIDTH = 24.0, 20.0
CHEVRON_HEIGHT = 12.0
STROKE = 4.0
PITCH = 9.5  # top to top
PAIR_HEIGHT = CHEVRON_HEIGHT + PITCH
CAP_SEGMENTS = 8

GREEN = rl.Color(0, 255, 64, 255)
IDLE = rl.Color(255, 255, 255, round(255 * 0.35))  # an unlit distance bar
LIFT_RC = 0.3  # the lift lets go fast; ease it so the chevrons don't blink


def _arc(cx: float, cy: float, r: float, a0: float, a1: float) -> list[tuple[float, float]]:
  return [(cx + r * math.cos(a), cy + r * math.sin(a)) for a in np.linspace(a0, a1, CAP_SEGMENTS)]


@lru_cache(maxsize=8)
def chevron_outline(width: float, height: float, stroke: float) -> np.ndarray:
  """One chevron about its ink's top centre: a stroke along A-B-C (apex B on top) with round caps, a round
  outer join and a mitred inner one."""
  r = stroke / 2
  a, b, c = np.array([-width / 2 + r, height - r]), np.array([0.0, r]), np.array([width / 2 - r, height - r])
  d1, d2 = (b - a) / np.linalg.norm(b - a), (c - b) / np.linalg.norm(c - b)
  n1, n2 = np.array([d1[1], -d1[0]]), np.array([d2[1], -d2[0]])  # the outer, upward normals
  a1, a2 = math.atan2(n1[1], n1[0]), math.atan2(n2[1], n2[0])
  if a2 < a1:
    a2 += 2 * math.pi
  pts = _arc(*a, r, a1 - math.pi, a1) + _arc(*b, r, a1, a2) + _arc(*c, r, a2, a2 + math.pi)
  inner_a, inner_c = a - n1 * r, c - n2 * r
  t = np.linalg.solve(np.array([d1, -d2]).T, inner_c - inner_a)[0]
  pts.append(tuple(inner_a + d1 * t))
  return np.array(pts, dtype=np.float32)


def chevron_levels(lift: float) -> tuple[float, float]:
  """(lower, upper) green levels for a lift fraction: the lower fades in first, then the upper."""
  return float(np.clip(lift * 2, 0.0, 1.0)), float(np.clip(lift * 2 - 1, 0.0, 1.0))


def blend(a: rl.Color, b: rl.Color, t: float) -> rl.Color:
  return rl.Color(*(round(x + (y - x) * t) for x, y in zip((a.r, a.g, a.b, a.a), (b.r, b.g, b.b, b.a), strict=True)))


def draw_chevrons(cx: float, top: float, scale: float = 1.0, lift: float = 0.0, alpha: float = 1.0,
                  idle: rl.Color = IDLE, lit: rl.Color = GREEN) -> None:
  """The pair with its ink's top centre at (cx, top), each chevron blended from idle to lit by its level."""
  screen = rl.Rectangle(0, 0, gui_app.width, gui_app.height)
  for width, offset, level in ((UPPER_WIDTH, 0.0, chevron_levels(lift)[1]), (LOWER_WIDTH, PITCH, chevron_levels(lift)[0])):
    color = blend(idle, lit, level)
    color.a = round(color.a * alpha)
    if color.a == 0:
      continue
    outline = chevron_outline(width * scale, CHEVRON_HEIGHT * scale, STROKE * scale)
    draw_polygon(screen, outline + np.array([cx, top + offset * scale], dtype=np.float32), color=color)


def lift_fraction() -> float:
  """The lift on the model's acceleration over the personality's approach ceiling, 0..1. Only while
  openpilot drives longitudinal: not under MADS lateral-only."""
  sm = ui_state.sm
  if not ui_state.long_engaged or not sm.valid['longitudinalPlanSP']:
    return 0.0
  boost = sm['longitudinalPlanSP'].zoompilot.e2eSetSpeed.boost
  # the lift never goes negative, and the ceiling's floor is 0.5 m/s^2
  return float(np.clip(boost / approach_ceiling(sm['carState'].vEgo, sm['selfdriveState'].personality.raw), 0.0, 1.0))


class SpeedAssistLift:
  """The eased lift, stepped once per frame."""
  def __init__(self):
    self._filter = FirstOrderFilter(0.0, LIFT_RC, 1 / gui_app.target_fps)

  def update(self) -> float:
    return self._filter.update(lift_fraction() if ui_state.speed_assist else 0.0)

  @property
  def x(self) -> float:
    return self._filter.x
