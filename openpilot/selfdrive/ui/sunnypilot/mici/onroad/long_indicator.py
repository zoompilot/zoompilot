"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

mici's long indicator on whole pixels: upstream centres the car at 100 and it is 27 tall, so its top sits
at 86.5 and the textures blur across two rows. The stack draws half a pixel lower.
"""
import math

import raylib_nb as rl

from openpilot.selfdrive.ui.mici.onroad.long_indicator import LongIndicator

STACK_TOP = 86.5  # upstream's car top


class LongIndicatorSP(LongIndicator):
  def _render(self, rect: rl.Rectangle) -> None:
    top = rect.y + STACK_TOP
    super()._render(rl.Rectangle(rect.x, rect.y + math.floor(top + 0.5) - top, rect.width, rect.height))
