"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Set Speed Assist over mici's long indicator. Off, the car and bars sit where upstream puts them. On, the
stack moves down and the chevrons take the space above the car, so the side panel's free height between
the driver monitor (bottom at 62) and the wheel (top at 178) splits into even gaps.
"""
import raylib_nb as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.sunnypilot.onroad.speed_assist_chevrons import PAIR_HEIGHT, SpeedAssistLift, draw_chevrons
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app

DM_BOTTOM, WHEEL_TOP = 62.0, 178.0
STACK_TOP, STACK_HEIGHT = 86.5, 71.0  # upstream's car (centred at 100, 27 tall) down to the third bar
GAP = (WHEEL_TOP - DM_BOTTOM - PAIR_HEIGHT - STACK_HEIGHT) / 3
CHEVRONS_TOP = DM_BOTTOM + GAP
STACK_SHIFT = CHEVRONS_TOP + PAIR_HEIGHT + GAP - STACK_TOP
SLIDE_RC = 0.25  # the status fade's


class LongIndicatorSP:
  def __init__(self):
    self._speed_assist_filter = FirstOrderFilter(0.0, SLIDE_RC, 1 / gui_app.target_fps)
    self._speed_assist_lift = SpeedAssistLift()

  def render_speed_assist(self, rect: rl.Rectangle, alpha: float) -> rl.Rectangle:
    """Draw the chevrons and return the rect the car and bars draw in, shifted to make room."""
    shown = self._speed_assist_filter.update(float(ui_state.speed_assist))
    lift = self._speed_assist_lift.update()
    shift = STACK_SHIFT * shown
    if shown * alpha > 1e-2:
      # they ride down with the stack and fade in as it makes room
      draw_chevrons(rect.x + rect.width / 2, rect.y + CHEVRONS_TOP + shift - STACK_SHIFT, lift=lift, alpha=alpha * shown)
    return rl.Rectangle(rect.x, rect.y + shift, rect.width, rect.height)
