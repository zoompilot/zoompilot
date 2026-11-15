"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Set Speed Assist over mici's long indicator: the set speed itself above the car, white while the assist is
on and green while it lifts the model's acceleration toward that speed. On this screen a number reads at a
glance where a symbol of the size that fits doesn't, and it says what the assist is doing.

Off, the car and bars sit where upstream puts them. On, the number joins the top of the stack and the group
sits between the driver monitor (ink bottom at 62) and the wheel (ink top at 178), half a pixel above
centre: the wheel's glow and the monitor's rounded base make equal gaps look closer at the bottom.
"""
import math

import raylib_nb as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.mici.onroad.hud_renderer import CRUISE_DISABLED_CHAR, KM_TO_MILE, SET_SPEED_NA, WHEEL_GREEN
from openpilot.selfdrive.ui.mici.onroad.long_indicator import LongIndicator
from openpilot.selfdrive.ui.mici.onroad.status import lerp_color
from openpilot.selfdrive.ui.sunnypilot.onroad.speed_assist_lift import SpeedAssistLift, green_level
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import FontWeight, gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached

DM_BOTTOM, WHEEL_TOP = 62.0, 178.0
STACK_TOP, STACK_HEIGHT = 86.5, 71.0  # upstream's car (centred at 100, 27 tall) down to the third bar
NUMBER_SIZE = 36
NUMBER_SIZE_3 = 32  # three figures (km/h) a size down to stay inside the panel
FIGURE_HEIGHT = 0.722  # Inter's figures, of the font size
FIGURE_TOP = 0.22  # the figures' top below the text's origin, of the font size
NUMBER_HEIGHT = round(FIGURE_HEIGHT * NUMBER_SIZE)
NUMBER_GAP = 8.0  # number to car: a touch over the bars' 6 px pitch, the figures are heavier than a bar
MARGIN = (WHEEL_TOP - DM_BOTTOM - NUMBER_HEIGHT - NUMBER_GAP - STACK_HEIGHT) / 2
OPTICAL_LIFT = 0.5
NUMBER_TOP = DM_BOTTOM + MARGIN - OPTICAL_LIFT
STACK_SHIFT = NUMBER_TOP + NUMBER_HEIGHT + NUMBER_GAP - STACK_TOP
SLIDE_RC = 0.25  # the status fade's
WHITE = rl.Color(255, 255, 255, round(255 * 0.9))  # a lit bar's


def set_speed_text() -> str:
  """The set speed as upstream's MAX readout shows it."""
  sm = ui_state.sm
  speed = sm['carState'].vCruiseCluster
  if sm.recv_frame['carState'] < ui_state.started_frame or not 0 < speed < SET_SPEED_NA:
    return CRUISE_DISABLED_CHAR
  return str(round(speed if ui_state.is_metric else speed * KM_TO_MILE))


def draw_set_speed(cx: float, top: float, text: str, green: float, alpha: float) -> None:
  """The set speed with its figures' top centre at (cx, top), `green` of the way from white to green."""
  if alpha < 1e-2:
    return
  font = gui_app.font(FontWeight.DISPLAY)
  size = NUMBER_SIZE if len(text) < 3 else NUMBER_SIZE_3
  # a smaller size centres on the same line
  y = top + (NUMBER_HEIGHT - FIGURE_HEIGHT * size) / 2 - FIGURE_TOP * size
  width = measure_text_cached(font, text, size).x
  color = lerp_color(WHITE, WHEEL_GREEN, green)
  rl.draw_text_ex(font, text, rl.Vector2(round(cx - width / 2), round(y)), size, 0, rl.Color(color.r, color.g, color.b, round(color.a * alpha)))


class LongIndicatorSP(LongIndicator):
  def __init__(self):
    super().__init__()
    self._speed_assist_filter = FirstOrderFilter(0.0, SLIDE_RC, 1 / gui_app.target_fps)
    self._speed_assist_lift = SpeedAssistLift()

  def _render(self, rect: rl.Rectangle) -> None:
    shown = self._speed_assist_filter.update(float(ui_state.speed_assist))
    # whole pixels for the car's top: the textures blur across two rows at a fraction, and upstream's rest
    # position (86.5) is one, so off sits half a pixel lower
    top = rect.y + STACK_TOP
    shift = math.floor(top + STACK_SHIFT * shown + 0.5) - top
    super()._render(rl.Rectangle(rect.x, rect.y + shift, rect.width, rect.height))
    # the number rides down with the stack and fades in as the stack makes room, at the stack's own alpha
    # (super() zeroes it when there is no openpilot longitudinal)
    green = green_level(self._speed_assist_lift.update())
    draw_set_speed(rect.x + rect.width / 2, rect.y + NUMBER_TOP + shift - STACK_SHIFT, set_speed_text(), green,
                   self._alpha_filter.x * self._dim_filter.x * shown)
