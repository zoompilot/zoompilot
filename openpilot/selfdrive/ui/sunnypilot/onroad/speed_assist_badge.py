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
import raylib_nb as rl

from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import Widget

STATUS_ICON_HEIGHT = 40  # upstream's footer icons (network, chestnut, usb) are 40 px of ink
STATUS_ICON_ALPHA = 0.9  # settings and network draw at 90%

# gauge, in units of its scale s, about the dial's centre (raylib degrees: 0 right, clockwise).
# Strokes sit near the wheel's rim (~6 px at 50) at the onroad 44 px; the swoosh rides out with its
# thickness so the gap to the dial stays open, which is what keeps the weight from reading heavy.
DIAL_FROM, DIAL_TO = 135, 405
DIAL_R, DIAL_T = 0.378, 0.115
NEEDLE_DEG = 285          # about 12:30
NEEDLE_LEN = DIAL_R - DIAL_T - 0.04
HUB_R = 0.085
SWOOSH_FROM = 195
SWOOSH_T0, SWOOSH_T1 = 0.05, 0.10  # tapers up to the head
SWOOSH_GAP = 0.09
SWOOSH_R = DIAL_R + SWOOSH_GAP + SWOOSH_T1 / 2
HEAD_HALF = 1.3 * SWOOSH_T1
HEAD_DEG = math.degrees(1.7 * HEAD_HALF / SWOOSH_R)  # the head's length along the arc
SWOOSH_TO = NEEDLE_DEG - HEAD_DEG  # so the arrow's tip lies on the needle's line
SWOOSH_SEGMENTS = 24
DIAL_SEGMENTS = 64
HUB_SEGMENTS = 12

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


Point = tuple[float, float]


def _ribbon(left: list[Point], right: list[Point], color: rl.Color) -> None:
  """One triangle strip between two chains, so a translucent colour lays each pixel once: separate
  overlapping shapes would show darker seams wherever they meet."""
  (lx, ly), (rx, ry), (nx, ny) = left[0], right[0], left[1]
  if (rx - lx) * (ny - ly) - (ry - ly) * (nx - lx) > 0:
    left, right = right, left  # raylib culls clockwise triangles; the strip's first one sets the winding
  strip = [rl.Vector2(*p) for pair in zip(left, right, strict=True) for p in pair]
  rl.draw_triangle_strip(strip, len(strip), color)


def draw_gauge(x: float, y: float, height: float, alpha: float = 1.0) -> None:
  """The Set Speed Assist gauge with its ink's top-left at (x, y), `height` tall."""
  s = height / (_BOTTOM - _TOP)
  cx, cy = x - _LEFT * s, y - _TOP * s
  color = rl.fade(rl.WHITE, alpha)

  dial = np.linspace(DIAL_FROM, DIAL_TO, DIAL_SEGMENTS + 1)
  _ribbon([_pt(cx, cy, DIAL_R * s, d) for d in dial], [_pt(cx, cy, (DIAL_R - DIAL_T) * s, d) for d in dial], color)

  # needle and hub as one teardrop: the tip, then the hub's back half from where the sides meet it
  tip = _pt(cx, cy, NEEDLE_LEN * s, NEEDLE_DEG)
  side = math.degrees(math.acos(HUB_R / NEEDLE_LEN))
  back = np.linspace(side, 180, HUB_SEGMENTS)
  _ribbon([tip] + [_pt(cx, cy, HUB_R * s, NEEDLE_DEG + d) for d in back],
          [tip] + [_pt(cx, cy, HUB_R * s, NEEDLE_DEG - d) for d in back], color)

  # swoosh and arrowhead as one ribbon: the tapered shaft, its barbs, then the tip
  outer, inner = [], []
  for f in np.linspace(0, 1, SWOOSH_SEGMENTS + 1):
    deg, t = SWOOSH_FROM + (SWOOSH_TO - SWOOSH_FROM) * f, _swoosh_t(f)
    outer.append(_pt(cx, cy, (SWOOSH_R + t / 2) * s, deg))
    inner.append(_pt(cx, cy, (SWOOSH_R - t / 2) * s, deg))
  outer.append(_pt(cx, cy, (SWOOSH_R + HEAD_HALF) * s, SWOOSH_TO))
  inner.append(_pt(cx, cy, (SWOOSH_R - HEAD_HALF) * s, SWOOSH_TO))
  arrow_tip = _pt(cx, cy, SWOOSH_R * s, NEEDLE_DEG)
  _ribbon(outer + [arrow_tip], inner + [arrow_tip], color)

def draw_gauge_centered(cx: float, cy: float, height: float, alpha: float = 1.0) -> None:
  """Its ink box centred on (cx, cy), to sit beside other icons."""
  draw_gauge(cx - GAUGE_ASPECT * height / 2, cy - height / 2, height, alpha)


DIAL_DROP = 1 / 16  # of the height: the swoosh weighs the top, so a dial on the centre looks high


def draw_gauge_on_dial(cx: float, cy: float, height: float, alpha: float = 1.0) -> None:
  """For a round button: the dial's centre on (cx, cy), dropped a touch to look centred, so dial,
  swoosh and button ring read as one; centring the ink box sets the dial low and right instead."""
  s = height / (_BOTTOM - _TOP)
  draw_gauge(cx + _LEFT * s, cy + (_TOP + DIAL_DROP * (_BOTTOM - _TOP)) * s, height, alpha)


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
    color = rl.fade(rl.WHITE, STATUS_ICON_ALPHA)
    x, y = self._rect.x, self._rect.y
    for texture in (self._car, *self._bars):
      # whole pixels: the bars are a few px tall and blur on a half-pixel offset
      rl.draw_texture_ex(texture, rl.Vector2(round(x + (self._rect.width - texture.width) / 2), round(y)), 0.0, 1.0, color)
      y += texture.height + LONG_GAP
