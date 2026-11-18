"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The longitudinal marks. Set Speed Assist is a set-speed gauge with a swoosh over it, its arrow ending on
the needle's line: speed raised to the set speed. It marks the assist in the home footer and on the tizi
button. Alpha longitudinal is upstream's onroad car over three bars, tightened to the home footer's 40 px.

Both are sized by their ink, not a padded box, so a status bar's spacing is the gap you see. The gauge is
three outlines filled by draw_polygon, antialiased at any size, and adds no LFS asset.
"""
import math
from functools import lru_cache

import numpy as np
import raylib_nb as rl

from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.polygon import draw_polygon
from openpilot.system.ui.widgets import Widget

STATUS_ICON_HEIGHT = 40  # upstream's footer icons (network, chestnut, usb) are 40 px of ink
STATUS_ICON_ALPHA = 0.9  # settings and network draw at 90%

# gauge, in units of its scale, about the dial's centre (raylib degrees: 0 right, clockwise). The swoosh
# rides out with its thickness so the gap to the dial stays open, which keeps the weight from reading heavy.
DIAL_FROM, DIAL_TO = 135, 405
DIAL_R, DIAL_T = 0.378, 0.115
NEEDLE_DEG = 285  # about 12:30
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
DIAL_DROP = 1 / 16  # of the height: the swoosh weighs the top, so a dial on a button's centre looks high


def _pt(r: float, deg: float) -> tuple[float, float]:
  return r * math.cos(math.radians(deg)), r * math.sin(math.radians(deg))


def _unit_outlines() -> list[np.ndarray]:
  """The dial's band, the needle and hub as one teardrop, and the tapered swoosh with its arrowhead."""
  dial = np.linspace(DIAL_FROM, DIAL_TO, DIAL_SEGMENTS + 1)
  band = [_pt(DIAL_R, d) for d in dial] + [_pt(DIAL_R - DIAL_T, d) for d in dial[::-1]]

  # the tip, then round the hub's back from one tangent to the other
  side = math.degrees(math.acos(HUB_R / NEEDLE_LEN))
  needle = [_pt(NEEDLE_LEN, NEEDLE_DEG)] + [_pt(HUB_R, NEEDLE_DEG + d) for d in np.linspace(side, 360 - side, 2 * HUB_SEGMENTS - 1)]

  outer, inner = [], []
  for f in np.linspace(0, 1, SWOOSH_SEGMENTS + 1):
    deg, t = SWOOSH_FROM + (SWOOSH_TO - SWOOSH_FROM) * f, SWOOSH_T0 + (SWOOSH_T1 - SWOOSH_T0) * f
    outer.append(_pt(SWOOSH_R + t / 2, deg))
    inner.append(_pt(SWOOSH_R - t / 2, deg))
  head = [_pt(SWOOSH_R + HEAD_HALF, SWOOSH_TO), _pt(SWOOSH_R, NEEDLE_DEG), _pt(SWOOSH_R - HEAD_HALF, SWOOSH_TO)]
  swoosh = outer + head + inner[::-1]
  return [np.array(p, dtype=np.float32) for p in (band, needle, swoosh)]


_UNIT = _unit_outlines()
_LEFT, _TOP = np.vstack(_UNIT).min(axis=0)
_RIGHT, _BOTTOM = np.vstack(_UNIT).max(axis=0)
GAUGE_ASPECT = float((_RIGHT - _LEFT) / (_BOTTOM - _TOP))  # ink width / height


@lru_cache(maxsize=8)
def gauge_outlines(height: float) -> list[np.ndarray]:
  """The gauge's outlines `height` tall, its ink's top-left at the origin."""
  s = height / (_BOTTOM - _TOP)
  return [((unit - (_LEFT, _TOP)) * s).astype(np.float32) for unit in _UNIT]


def draw_gauge(x: float, y: float, height: float, alpha: float = 1.0) -> None:
  """The Set Speed Assist gauge with its ink's top-left at (x, y), `height` tall."""
  if alpha < 1e-2:
    return
  screen = rl.Rectangle(0, 0, gui_app.width, gui_app.height)
  fill = rl.fade(rl.WHITE, alpha)
  origin = np.array([x, y], dtype=np.float32)
  for outline in gauge_outlines(round(height, 3)):
    draw_polygon(screen, outline + origin, color=fill)


def draw_gauge_on_dial(cx: float, cy: float, height: float, alpha: float = 1.0) -> None:
  """For a round button: the dial's centre on (cx, cy), dropped a touch to look centred, so dial, swoosh
  and button ring read as one; centring the ink box sets the dial low and right instead."""
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
