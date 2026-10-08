"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Whether ICBM drives the set speed. card decides it (IcbmLatch) and publishes it as
carStateSP.zoompilot.icbmActivation; every other process reads that (icbm_active) instead of
CarParamsSP.pcmCruiseSpeed. See docs/zoompilot/icbm-auto-plan.md.
"""
from openpilot.cereal import custom

IcbmActivation = custom.CarStateZP.IcbmActivation


class IcbmLatch:
  """card's decision. Constant for the drive: the boot flag, as before."""

  def __init__(self, CP_SP) -> None:
    self.active = not CP_SP.pcmCruiseSpeed

  @property
  def activation(self):
    return IcbmActivation.active if self.active else IcbmActivation.passive


def icbm_active(car_state_sp, CP_SP) -> bool:
  """card's published decision; a log from before the field falls back to the boot flag."""
  activation = car_state_sp.zoompilot.icbmActivation
  if activation == IcbmActivation.unset:
    return not CP_SP.pcmCruiseSpeed
  return activation == IcbmActivation.active
