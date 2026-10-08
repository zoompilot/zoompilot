"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Whether ICBM drives the set speed. card decides it (IcbmLatch) and publishes it as
carStateSP.zoompilot.icbmActivation; every other process reads that (icbm_active) instead of
CarParamsSP.pcmCruiseSpeed. See docs/zoompilot/icbm-auto-plan.md.
"""
from openpilot.cereal import custom
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import icbm_applicable

IcbmActivation = custom.CarStateZP.IcbmActivation


class IcbmLatch:
  """card's decision. It starts at the boot decision (pcmCruiseSpeed, set in interfaces.py
  from the same demand) and follows demand only while neither openpilot nor the stock cruise
  is engaged: every reader's two branches agree there (no longActive, no setpoint, no servo
  move, no open SLA session), so nothing is handed over mid-control."""

  def __init__(self, CP, CP_SP) -> None:
    self.capable = icbm_applicable(CP, CP_SP)
    self.active = not CP_SP.pcmCruiseSpeed
    self.demanded = self.active  # the params thread refreshes it (demand.icbm_demanded)

  def update(self, engaged: bool) -> bool:
    """Returns True on the frame the decision changes."""
    want = self.capable and self.demanded
    if want == self.active or engaged:
      return False
    self.active = want
    return True

  @property
  def activation(self):
    return IcbmActivation.active if self.active else IcbmActivation.passive


def icbm_active(car_state_sp, CP_SP) -> bool:
  """card's published decision; a log from before the field falls back to the boot flag."""
  activation = car_state_sp.zoompilot.icbmActivation
  if activation == IcbmActivation.unset:
    return not CP_SP.pcmCruiseSpeed
  return activation == IcbmActivation.active
