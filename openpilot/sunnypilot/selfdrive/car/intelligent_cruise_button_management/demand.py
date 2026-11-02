"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The features that act through ICBM. ICBM has no toggle of its own: it runs on a capable car
(helpers.icbm_applicable) while one of these is on. A new feature that moves a stock ECU's
set speed with the cruise buttons is one entry here; the boot decision, the latch, the
settings migration and the settings locks all read this table.
"""
from dataclasses import dataclass

from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode


@dataclass(frozen=True)
class Consumer:
  key: str
  on: int | bool         # the param value that asks for ICBM
  off: int | bool        # what the migration writes when the feature was inert
  under_op_long: bool    # needs ICBM under openpilot longitudinal too (the planner runs SCC itself)
  inert_under_op_long: bool  # did nothing under openpilot long with the old toggle off

  def is_on(self, params: Params) -> bool:
    if isinstance(self.on, bool):
      return params.get_bool(self.key)
    return params.get(self.key, return_default=True) == self.on

  def needs_icbm(self, CP) -> bool:
    return self.under_op_long or not CP.openpilotLongitudinalControl


CONSUMERS = (
  Consumer("SmartCruiseControlVision", True, False, under_op_long=False, inert_under_op_long=False),
  Consumer("SmartCruiseControlMap", True, False, under_op_long=False, inert_under_op_long=False),
  # under alpha long the plannerd machine ran assist without the buttons (driver-confirm)
  Consumer("SpeedLimitMode", int(Mode.assist), int(Mode.warning), under_op_long=True, inert_under_op_long=False),
  Consumer("CustomAccIncrementsEnabled", True, False, under_op_long=True, inert_under_op_long=True),
)
CONSUMER_BY_KEY = {c.key: c for c in CONSUMERS}


def icbm_demanded(CP, params: Params) -> bool:
  return any(c.needs_icbm(CP) and c.is_on(params) for c in CONSUMERS)


def needs_icbm(CP, key: str) -> bool:
  """Whether turning `key` on brings ICBM up on this car's longitudinal mode."""
  consumer = CONSUMER_BY_KEY.get(key)
  return consumer is not None and consumer.needs_icbm(CP)
