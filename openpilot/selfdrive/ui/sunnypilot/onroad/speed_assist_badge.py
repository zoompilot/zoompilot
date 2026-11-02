"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Set Speed Assist's mark: two upward chevrons, the trailing one smaller and fainter, over SPEED. Drawn
as polygons rather than a texture, so it stays sharp at any size and adds no LFS asset.
"""
import numpy as np
import pyray as rl

from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.shader_polygon import draw_polygon
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

LABEL = "SPEED"
ASPECT = 0.40          # chevron height / width
THICKNESS = 0.56       # arm thickness, measured vertically, / height
TRAIL_OFFSET = 0.70    # trailing chevron's top below the leading one's, / height
TRAIL_SCALE = 0.80
TRAIL_ALPHA = 0.50
CHEVRON_WIDTH = 0.64   # / label width, in a badge
LABEL_GAP = 0.38       # chevrons to the label's caps, / cap height
CAP_TOP, CAP_BOTTOM = 0.20, 0.80  # Inter's caps within its measured line height


def _chevron(cx: float, top: float, width: float, color: rl.Color) -> None:
  """One mitered chevron, apex at (cx, top), feet cut flat: a ribbon of its outer and inner edges."""
  h = ASPECT * width
  t = THICKNESS * h
  foot = t * width / (2 * h)  # the flat foot's width for an arm parallel to the outer edge
  outer = [(cx - width / 2, top + h), (cx, top), (cx + width / 2, top + h)]
  inner = [(cx + width / 2 - foot, top + h), (cx, top + t), (cx - width / 2 + foot, top + h)]
  draw_polygon(rl.Rectangle(0, 0, 0, 0), np.array(outer + inner, dtype=np.float32), color=color)


def chevrons_height(width: float) -> float:
  return ASPECT * width * (TRAIL_OFFSET + TRAIL_SCALE)


def draw_chevrons(cx: float, top: float, width: float, alpha: float = 1.0) -> None:
  """The two chevrons in white, leading one's apex at (cx, top)."""
  h = ASPECT * width
  _chevron(cx, top + TRAIL_OFFSET * h, width * TRAIL_SCALE, rl.color_alpha(rl.WHITE, TRAIL_ALPHA * alpha))
  _chevron(cx, top, width, rl.color_alpha(rl.WHITE, alpha))


def draw_badge(cx: float, cy: float, font_size: int, alpha: float = 1.0) -> None:
  """Chevrons over SPEED, centred on (cx, cy), sized by the label's font size."""
  font = gui_app.font(FontWeight.BOLD)
  size = measure_text_cached(font, LABEL, font_size)
  cap = (CAP_BOTTOM - CAP_TOP) * size.y
  width = CHEVRON_WIDTH * size.x
  chevrons = chevrons_height(width)
  top = cy - (chevrons + LABEL_GAP * cap + cap) / 2
  draw_chevrons(cx, top, width, alpha)
  caps_top = top + chevrons + LABEL_GAP * cap
  rl.draw_text_ex(font, LABEL, rl.Vector2(round(cx - size.x / 2), round(caps_top - CAP_TOP * size.y)), font_size, 0,
                  rl.color_alpha(rl.WHITE, alpha))


class SpeedAssistIcon(Widget):
  """The chevrons alone, as a status bar icon of the given size."""
  def __init__(self, size: int):
    super().__init__()
    self.set_rect(rl.Rectangle(0, 0, float(size), float(size)))
    self.set_enabled(False)

  def _render(self, _) -> None:
    width = 0.78 * self._rect.width
    cx = self._rect.x + self._rect.width / 2
    draw_chevrons(cx, self._rect.y + (self._rect.height - chevrons_height(width)) / 2, width)
