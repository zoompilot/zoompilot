"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

How hard Set Speed Assist is lifting the model's acceleration, for the marks that turn green while it does:
mici's set speed over the long indicator and the tizi button's gauge.
"""
import numpy as np

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.approach import approach_ceiling
from openpilot.system.ui.lib.application import gui_app

LIFT_RC = 0.3  # the lift lets go fast; ease it so the green doesn't flicker
GREEN_LIFT = (0.02, 0.15)  # the lift over which a mark turns green: one too small to feel stays white


def lift_fraction() -> float:
  """The lift on the model's acceleration over the personality's approach ceiling, 0..1. Only while
  openpilot drives longitudinal: not under MADS lateral-only."""
  sm = ui_state.sm
  if not ui_state.long_engaged or not sm.valid['longitudinalPlanSP']:
    return 0.0
  boost = sm['longitudinalPlanSP'].zoompilot.e2eSetSpeed.boost
  # the lift never goes negative, and the ceiling's floor is 0.5 m/s^2
  return float(np.clip(boost / approach_ceiling(sm['carState'].vEgo, sm['selfdriveState'].personality.raw), 0.0, 1.0))


def green_level(lift: float) -> float:
  return float(np.interp(lift, GREEN_LIFT, (0.0, 1.0)))


class SpeedAssistLift:
  """The eased lift, stepped once per frame."""
  def __init__(self):
    self._filter = FirstOrderFilter(0.0, LIFT_RC, 1 / gui_app.target_fps)

  def update(self) -> float:
    return self._filter.update(lift_fraction() if ui_state.speed_assist else 0.0)
