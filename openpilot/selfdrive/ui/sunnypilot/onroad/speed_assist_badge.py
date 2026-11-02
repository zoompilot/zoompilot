"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The longitudinal marks. Set Speed Assist is a set-speed gauge with a swoosh over it, its arrow
ending on the needle's line: speed raised to the set speed. Alpha longitudinal is upstream's onroad
car over three bars, tightened to the home footer's 40 px.

Both are sized by their ink, not a padded box, so a status bar's spacing is the gap you see. The
gauge is drawn as polygons rather than a texture, so it stays sharp at any size and adds no LFS asset.
"""
import math

import numpy as np
import pyray as rl

from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget

STATUS_ICON_HEIGHT = 40  # upstream's footer icons (network, chestnut, usb) are 40 px of ink
STATUS_ICON_ALPHA = 0.9  # settings and network draw at 90%

# gauge, in units of its scale s, about the dial's centre (raylib degrees: 0 right, clockwise)
DIAL_FROM, DIAL_TO = 135, 405
DIAL_R, DIAL_T = 0.378, 0.09
NEEDLE_DEG = 285          # about 12:30
NEEDLE_W = 0.11
HUB_R = 0.75 * NEEDLE_W
SWOOSH_FROM = 195
SWOOSH_R = DIAL_R + 0.13
SWOOSH_T0, SWOOSH_T1 = 0.03, 0.08  # tapers up to the head
HEAD_HALF = 1.3 * SWOOSH_T1
HEAD_DEG = math.degrees(1.7 * HEAD_HALF / SWOOSH_R)  # the head's length along the arc
SWOOSH_TO = NEEDLE_DEG - HEAD_DEG  # so the arrow's tip lies on the needle's line
SWOOSH_SEGMENTS = 24


def _pt(cx: float, cy: float, r: float, deg: float) -> tuple[float, float]:
  return cx + r * math.cos(math.radians(deg)), cy + r * math.sin(math.radians(deg))


def _swoosh_t(f: float) -> float:
  return SWOOSH_T0 + (SWOOSH_T1 - SWOOSH_T0) * f


def _ink_bounds() -> tuple[float, float, float, float]:
  """(left, top, right, bottom) of the gauge's ink at s = 1 about the dial's centre."""
  pts = []
  for deg in np.linspace(DIAL_FROM, DIAL_TO, 271):
    pts += [_pt(0, 0, DIAL_R, deg), _pt(0, 0, DIAL_R - DIAL_T, deg)]
  for f in np.linspace(0, 1, 101):
    deg, t = SWOOSH_FROM + (SWOOSH_TO - SWOOSH_FROM) * f, _swoosh_t(f)
    pts += [_pt(0, 0, SWOOSH_R + t / 2, deg), _pt(0, 0, SWOOSH_R - t / 2, deg)]
  pts += [_pt(0, 0, SWOOSH_R, NEEDLE_DEG), _pt(0, 0, SWOOSH_R - HEAD_HALF, SWOOSH_TO), _pt(0, 0, SWOOSH_R + HEAD_HALF, SWOOSH_TO)]
  xs, ys = zip(*pts, strict=True)
  return min(xs), min(ys), max(xs), max(ys)


_LEFT, _TOP, _RIGHT, _BOTTOM = _ink_bounds()
GAUGE_ASPECT = (_RIGHT - _LEFT) / (_BOTTOM - _TOP)  # ink width / height


def _triangle(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float], color: rl.Color) -> None:
  # raylib culls one winding; drawing both keeps this free of orientation bookkeeping
  va, vb, vc = rl.Vector2(*a), rl.Vector2(*b), rl.Vector2(*c)
  rl.draw_triangle(va, vb, vc, color)
  rl.draw_triangle(va, vc, vb, color)


def draw_gauge(x: float, y: float, height: float, alpha: float = 1.0) -> None:
  """The Set Speed Assist gauge with its ink's top-left at (x, y), `height` tall."""
  s = height / (_BOTTOM - _TOP)
  cx, cy = x - _LEFT * s, y - _TOP * s
  color = rl.color_alpha(rl.WHITE, alpha)
  centre = rl.Vector2(cx, cy)

  rl.draw_ring(centre, (DIAL_R - DIAL_T) * s, DIAL_R * s, DIAL_FROM, DIAL_TO, 64, color)
  needle = (DIAL_R - 1.3 * DIAL_T) * s
  _triangle(_pt(cx, cy, needle, NEEDLE_DEG), _pt(cx, cy, NEEDLE_W * s / 2, NEEDLE_DEG - 90),
            _pt(cx, cy, NEEDLE_W * s / 2, NEEDLE_DEG + 90), color)
  rl.draw_circle_v(centre, HUB_R * s, color)

  step = (SWOOSH_TO - SWOOSH_FROM) / SWOOSH_SEGMENTS
  for i in range(SWOOSH_SEGMENTS):
    t = _swoosh_t((i + 1) / SWOOSH_SEGMENTS) * s
    start = SWOOSH_FROM + i * step
    # each piece overlaps the next by a hair so the joins never show a seam
    rl.draw_ring(centre, SWOOSH_R * s - t / 2, SWOOSH_R * s + t / 2, start, start + step + 0.6, 8, color)
  _triangle(_pt(cx, cy, SWOOSH_R * s, NEEDLE_DEG), _pt(cx, cy, (SWOOSH_R - HEAD_HALF) * s, SWOOSH_TO),
            _pt(cx, cy, (SWOOSH_R + HEAD_HALF) * s, SWOOSH_TO), color)


def draw_gauge_centered(cx: float, cy: float, height: float, alpha: float = 1.0) -> None:
  """Its ink box centred on (cx, cy), to sit beside other icons."""
  draw_gauge(cx - GAUGE_ASPECT * height / 2, cy - height / 2, height, alpha)


def draw_gauge_on_dial(cx: float, cy: float, height: float, alpha: float = 1.0) -> None:
  """The dial's centre on (cx, cy), for a round button: dial, swoosh and button ring share a centre,
  where centring the ink box would set the dial low and right under the swoosh."""
  s = height / (_BOTTOM - _TOP)
  draw_gauge(cx + _LEFT * s, cy + _TOP * s, height, alpha)


class SpeedAssistIcon(Widget):
  """The gauge as a status bar icon, its rect exactly its ink."""
  def __init__(self, height: int = STATUS_ICON_HEIGHT):
    super().__init__()
    self.set_rect(rl.Rectangle(0, 0, round(GAUGE_ASPECT * height), float(height)))
    self.set_enabled(False)

  def _render(self, _) -> None:
    draw_gauge(self._rect.x, self._rect.y, self._rect.height, STATUS_ICON_ALPHA)


# alpha longitudinal: upstream's onroad car over three bars (car 35 wide, bars 32/40/48), on whole
# pixels for 40 px: car 25x19, bars 23x3 / 29x4 / 33x5, 3 px apart
LONG_CAR = (25, 19)
LONG_BARS = ((23, 3), (29, 4), (33, 5))
LONG_GAP = 3


class LongitudinalIcon(Widget):
  """Alpha longitudinal on: the car and three bars the onroad view shows while it drives."""
  def __init__(self):
    super().__init__()
    self._car = gui_app.texture('icons_mici/longitudinal/car.png', *LONG_CAR, keep_aspect_ratio=False)
    self._bars = [gui_app.texture(f'icons_mici/longitudinal/distance_{i + 1}.png', w, h, keep_aspect_ratio=False)
                  for i, (w, h) in enumerate(LONG_BARS)]
    width = max(w for w, _ in (LONG_CAR, *LONG_BARS))
    height = LONG_CAR[1] + sum(h + LONG_GAP for _, h in LONG_BARS)
    self.set_rect(rl.Rectangle(0, 0, width, height))
    self.set_enabled(False)

  def _render(self, _) -> None:
    color = rl.color_alpha(rl.WHITE, STATUS_ICON_ALPHA)
    x, y = self._rect.x, self._rect.y
    for texture in (self._car, *self._bars):
      # whole pixels: the bars are a few px tall and blur on a half-pixel offset
      rl.draw_texture_ex(texture, rl.Vector2(round(x + (self._rect.width - texture.width) / 2), round(y)), 0.0, 1.0, color)
      y += texture.height + LONG_GAP
