"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

A bool param folded into a number control as its first step, so a feature's
on/off and its value are one setting: the comma 4's number picker and the
tici's +/- option both take one.
"""
from typing import NamedTuple


class Switch(NamedTuple):
  """Picking `label` sets `param` to `at` and keeps the value; picking a value
  sets `param` to the other state."""
  param: str
  at: bool
  label: str


# the switch step's raw value, below any control's range
SWITCH_VALUE = -(1 << 30)


def reads_switched(params, switch: Switch | None) -> bool:
  return switch is not None and params.get_bool(switch.param) == switch.at


def write_switch(params, switch: Switch | None, raw: int) -> None:
  """The switch param for a control now at `raw`: `at` on the switch step, the
  other state on any value. Nothing without a switch."""
  if switch is not None:
    params.put_bool(switch.param, switch.at == (raw == SWITCH_VALUE))
