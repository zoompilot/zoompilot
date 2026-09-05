"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Per-car actuation profile for Intelligent Cruise Button Management (ICBM).

On button-actuated (non-pcmCruiseSpeed) cars the stock ECU integrates the cruise buttons,
and every ECU does it differently: how fast discrete presses register, whether a held
button snaps the set speed along a coarse grid, and whether the ACC delays deceleration
while the set speed is still changing. The servo that plans button moves reads these
characteristics from here.

Cars without a measured profile get DEFAULT_PROFILE: discrete taps only, no grid, no hold;
the long-standing ICBM behavior. A brand only changes behavior by adding a measured entry.
Measurements behind the Mazda entry: docs/zoompilot/icbm.md.
"""
from dataclasses import dataclass

from opendbc.car import structs

SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

# the servo's hold sends are a stream of presses, valid wherever taps are; a brand without a
# native hold cadence acts on them as the tap of the same direction
TAP_EQUIVALENT = {
  SendButtonState.increaseHold: SendButtonState.increase,
  SendButtonState.decreaseHold: SendButtonState.decrease,
}


def tap_equivalent(send_button: SendButtonState) -> SendButtonState:
  return TAP_EQUIVALENT.get(send_button, send_button)


@dataclass(frozen=True)
class ICBMActuationProfile:
  # fastest tap cadence the body ECU reliably registers; beyond it presses are dropped and
  # faster is slower. Tap size is not modeled: the servo is closed-loop on the dash.
  tap_rate_hz: float = 5.

  # held-button behavior of the stock ECU: a hold snaps to the next multiple of
  # longpress_step, then steps by it per period (step k at ~first_step_s + (k-1) * period).
  # 0 disables hold planning (taps only).
  longpress_step: int = 0  # display units per hold step; also the alignment grid
  longpress_first_step_s: float = 0.
  longpress_step_period_s: float = 0.
  # grid measured in imperial display units only so far; metric users plan with taps
  longpress_metric_confirmed: bool = False

  # the stock ACC does not commit to decelerating while the set speed is still moving, so
  # the servo makes one decisive move and goes quiet instead of tracking continuously
  decel_needs_stable_setpoint: bool = False

  @property
  def has_longpress(self) -> bool:
    return self.longpress_step > 0

  def supports_longpress(self, is_metric: bool) -> bool:
    return self.has_longpress and (self.longpress_metric_confirmed or not is_metric)


# Mazda CX-5 2022: taps register at 5 Hz and move 1 mph; a physical hold snaps to multiples
# of 5 mph, first step ~0.6 s in, ~0.55 s per further step; MRCC will not start
# decelerating until the set speed stops changing.
ICBM_ACTUATION_PROFILES: dict[str, ICBMActuationProfile] = {
  'mazda': ICBMActuationProfile(
    tap_rate_hz=5.,
    longpress_step=5,
    longpress_first_step_s=0.6,
    longpress_step_period_s=0.55,
    longpress_metric_confirmed=False,
    decel_needs_stable_setpoint=True,
  ),
}

DEFAULT_PROFILE = ICBMActuationProfile()


def get_actuation_profile(brand: str) -> ICBMActuationProfile:
  return ICBM_ACTUATION_PROFILES.get(brand, DEFAULT_PROFILE)
