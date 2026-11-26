"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

mici's lead car and following-distance bars on tizi. The behaviour is upstream's LongIndicator
unchanged (the lead car and distance bars on openpilot longitudinal control, the speed dial on stock
ACC, dimmed while disengaged); only its size and place are tizi's: twice the mici size, stacked on
the driver monitor with mici's spacing, so it follows the monitor's side.
"""
import raylib_nb as rl

from openpilot.selfdrive.ui.mici.onroad.long_indicator import LongIndicator
from openpilot.selfdrive.ui.sunnypilot.onroad.driver_state import driver_monitor_rect
from openpilot.selfdrive.ui.ui_state import ui_state

SCALE = 2.0
# in LongIndicator's y layout
CAR_TOP_Y = 96 - 31 / 2  # its car's top
LAST_BAR_BOTTOM_Y = 155 + 10 / 2
MICI_WHEEL_TOP_Y = 176
STACK_GAP = (MICI_WHEEL_TOP_Y - LAST_BAR_BOTTOM_Y) * SCALE  # mici's bars-to-wheel spacing, at tizi's size


def long_indicator_top(rect: rl.Rectangle, is_rhd: bool) -> float:
  """Where the stack's car begins; the confidence ball's track ends above it."""
  return driver_monitor_rect(rect, is_rhd).y - (MICI_WHEEL_TOP_Y - CAR_TOP_Y) * SCALE


class LongIndicatorTizi(LongIndicator):
  def __init__(self):
    super().__init__()
    self.set_should_draw(True)  # nothing on tizi shares its place

  def _texture(self, name: str, width: int, height: int) -> rl.Texture:
    # loaded at the drawn size, so it stays sharp rather than scaled up from mici's
    return LongIndicator._texture(name, round(width * SCALE), round(height * SCALE))

  def _render(self, rect: rl.Rectangle) -> None:
    # mici lights the car from its model renderer's lead bars; tizi's draws none, so the planner's lead
    sm = ui_state.sm
    self.set_has_lead(ui_state.started and sm.alive['longitudinalPlan'] and sm['longitudinalPlan'].hasLead)
    super()._render(rect)

  def _draw_centered(self, texture: rl.Texture, rect: rl.Rectangle, y: float, alpha: float) -> None:
    monitor = driver_monitor_rect(rect, ui_state.sm['driverMonitoringState'].isRHD)
    cx = monitor.x + monitor.width / 2
    cy = monitor.y - (MICI_WHEEL_TOP_Y - y) * SCALE
    pos = rl.Vector2(round(cx - texture.width / 2), round(cy - texture.height / 2))
    rl.draw_texture_ex(texture, pos, 0.0, 1.0, rl.Color(255, 255, 255, round(255 * alpha)))
