"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The features that act through ICBM. ICBM has no toggle of its own: it runs on a capable car
(helpers.icbm_applicable) while one of these is on. A new feature that moves a stock ECU's
set speed with the cruise buttons is one entry here.
"""
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode


def _scc(params: Params) -> bool:
  return params.get_bool("SmartCruiseControlVision") or params.get_bool("SmartCruiseControlMap")


def _sla_assist(params: Params) -> bool:
  return params.get("SpeedLimitMode", return_default=True) == Mode.assist


def _custom_increments(params: Params) -> bool:
  return params.get_bool("CustomAccIncrementsEnabled")


# (feature, also under openpilot longitudinal): with openpilot long the planner executes the
# curve speed itself, so only the features that move the dash setpoint need the buttons there
CONSUMERS = (
  (_scc, False),
  (_sla_assist, True),
  (_custom_increments, True),
)


def icbm_demanded(CP, params: Params) -> bool:
  op_long = CP.openpilotLongitudinalControl
  return any(wants(params) for wants, under_op_long in CONSUMERS if under_op_long or not op_long)
