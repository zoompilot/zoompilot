"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

import pyray as rl

from openpilot.cereal import custom
from openpilot.selfdrive.ui import UI_BORDER_SIZE
from openpilot.selfdrive.ui.onroad.driver_state import BTN_SIZE
from openpilot.selfdrive.ui.ui_state import ui_state

# The gauge shows cylinder status: two arcs grow clockwise as the PCM ramps
# toward deactivation, the latch holds two opposite quadrant arcs (2 of 4
# cylinders), and fuel cut spins four dashes counter-clockwise (drag). State
# changes crossfade; on the return to all-cylinder operation the last look
# fades out and the gauge hides. A black bed under the gauge keeps it
# readable over any road. The gauge only draws with the user toggle on; cars
# without deactivation hardware never leave normal, so it stays hidden
# without a setting either way.
#
# The default ring wraps the driver-monitoring circle: same center as the
# icon, inner edge at the icon boundary. Mici passes smaller radii centered
# on its steering-wheel icon.
_DM_RADIUS = BTN_SIZE // 2
_DM_CENTER_OFFSET = UI_BORDER_SIZE + _DM_RADIUS
RING_INNER_R = float(_DM_RADIUS)
RING_OUTER_R = float(_DM_RADIUS + 24)
CORNER_OFFSET = float(_DM_CENTER_OFFSET)

# Turbine encoding: clockwise motion means load, counter-clockwise means drag.
QUAD_DEG = 90.0
LATCH_ARCS = ((-90.0, 0.0), (90.0, 180.0))  # raylib angles: top-right, bottom-left
CUT_DASH_COUNT = 4
CUT_SPAN_DEG = 45.0
CUT_PERIOD_S = 3.0  # seconds per counter-clockwise revolution
STATE_FADE_S = 0.3  # crossfade when the gauge switches states

STATE = custom.CarStateZP.CylinderDeactivation.State

RING_ACTIVE = rl.Color(0x4f, 0xd0, 0x84, 0xff)  # cylinders firing
RING_CUT = rl.Color(0x53, 0x9f, 0xc7, 0xff)     # fuel cut: no cylinders firing


def _alpha(color: rl.Color, a: int) -> rl.Color:
  return rl.Color(color.r, color.g, color.b, color.a * a // 255)


class CylinderDeactivationRenderer:
  def __init__(self, inner_r: float = RING_INNER_R, outer_r: float = RING_OUTER_R,
               corner_x: float = CORNER_OFFSET, corner_y: float = CORNER_OFFSET):
    self._inner_r = inner_r
    self._outer_r = outer_r
    self._corner_x = corner_x
    self._corner_y = corner_y
    self._last_state = STATE.normal
    self._prev_state = STATE.normal
    self._state_changed_t = 0.0
    self._vignette_a = 0
    self._progress = 0.0         # the ramp the current state last drew with
    self._prev_progress = 0.0    # latched at a transition, for the outgoing state's fade

  def render(self, rect: rl.Rectangle, sm) -> None:
    if not ui_state.cylinder_deactivation_ui:
      return

    cd = sm['carStateSP'].zoompilot.cylinderDeactivation
    state = cd.state
    now = time.monotonic()

    if state != self._last_state:
      self._prev_state = self._last_state
      # the decode resets entryProgress the same frame it leaves entry, so latch the
      # ramp the outgoing state was drawn with or its fade-out draws nothing
      self._prev_progress = self._progress
      self._last_state = state
      self._state_changed_t = now
    fade = max(0.0, min(1.0, (now - self._state_changed_t) / STATE_FADE_S))
    progress = max(0.0, min(1.0, float(cd.entryProgress)))
    self._progress = progress

    center = rl.Vector2(int(rect.x + self._corner_x), int(rect.y + rect.height - self._corner_y))

    if state == STATE.normal:
      # fade the previous state and its bed out, then hide
      if self._prev_state == STATE.normal or fade >= 1.0:
        return
      a = int(255 * (1.0 - fade))
      self._draw_vignette(center, a)
      self._draw_state(self._prev_state, center, a, self._prev_progress, now)
      return

    if state == STATE.entry:
      # the bed fades in with the ramp, like the bars
      self._vignette_a = int(255 * progress)
    elif self._prev_state == STATE.normal:
      # entering a visible state from hidden: fade the bed in with the state
      self._vignette_a = int(255 * fade)
    else:
      self._vignette_a = 255

    self._draw_vignette(center, self._vignette_a)
    if self._prev_state not in (STATE.normal, state) and fade < 1.0:
      self._draw_state(self._prev_state, center, int(255 * (1.0 - fade)), self._prev_progress, now)
    self._draw_state(state, center, int(255 * fade), progress, now)

  def _draw_vignette(self, center: rl.Vector2, alpha: int) -> None:
    if alpha <= 0:
      return
    # black annulus filling the zone between the icon and the ring, capped
    # flush at the ring's outer edge; the icon interior is never darkened
    rl.draw_ring(center, self._inner_r, self._outer_r, 0, 360, 64, rl.Color(0, 0, 0, alpha))

  def _draw_state(self, state, center: rl.Vector2, alpha: int, progress: float, now: float) -> None:
    if state == STATE.normal or alpha <= 0:
      return

    if state == STATE.entry:
      # two arcs grow clockwise from 12 and 6 o'clock, fading in with the ramp
      p = max(0.0, min(1.0, progress))
      if p <= 0:
        return
      color = _alpha(RING_ACTIVE, int(alpha * p))
      for start in (-90.0, 90.0):
        rl.draw_ring(center, self._inner_r, self._outer_r, start, start + QUAD_DEG * p, 24, color)
    elif state == STATE.deactivated:
      for start, end in LATCH_ARCS:
        rl.draw_ring(center, self._inner_r, self._outer_r, start, end, 24, _alpha(RING_ACTIVE, alpha))
    elif state == STATE.engineBraking:
      # four dashes spin counter-clockwise: drag, not load
      off = -360.0 * (now % CUT_PERIOD_S) / CUT_PERIOD_S
      for k in range(CUT_DASH_COUNT):
        a0 = -90 + 90 * k + off
        rl.draw_ring(center, self._inner_r, self._outer_r, a0, a0 + CUT_SPAN_DEG, 16, _alpha(RING_CUT, alpha))
