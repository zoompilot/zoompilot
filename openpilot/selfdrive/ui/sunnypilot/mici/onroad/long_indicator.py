"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

mici's long indicator on whole pixels: upstream centres each icon on its height, so the odd ones (the
car is 31 tall, the middle bar 9) start on a half pixel and blur across two rows. Each icon snaps to
the nearest pixel.
"""
import math

import raylib_nb as rl

from openpilot.selfdrive.ui.mici.onroad.long_indicator import LongIndicator


class LongIndicatorSP(LongIndicator):
  @staticmethod
  def _draw_centered(texture: rl.Texture, rect: rl.Rectangle, y: float, alpha: float) -> None:
    pos = rl.Vector2(math.floor(rect.x + (rect.width - texture.width) / 2 + 0.5), math.floor(rect.y + y - texture.height / 2 + 0.5))
    rl.draw_texture_ex(texture, pos, 0.0, 1.0, rl.Color(255, 255, 255, round(255 * alpha)))
