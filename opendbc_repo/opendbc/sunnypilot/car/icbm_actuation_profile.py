"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Per-car actuation profile for Intelligent Cruise Button Management (ICBM).

On button-actuated (non-pcmCruiseSpeed) cars the stock ECU integrates the cruise buttons,
and every ECU does it differently: how fast discrete presses register and whether the ACC
delays deceleration while the set speed is still changing. The servo that plans button
moves reads these characteristics from here, and the stock ACC plant behind them: how hard
it brakes for a dash gap, which the curve and limit planners budget for
(smart_cruise_control/limits.py).

Cars without a measured profile get DEFAULT_PROFILE: discrete taps only, no hold;
the long-standing ICBM behavior. A brand only changes behavior by adding a measured entry.
Measurements behind the Mazda entry: docs/zoompilot/icbm.md.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class ICBMActuationProfile:
  # fastest tap cadence the body ECU reliably registers; beyond it presses are dropped and
  # faster is slower. Tap size is not modeled: the servo is closed-loop on the dash.
  tap_rate_hz: float = 5.

  # the stock ACC does not commit to decelerating while the set speed is still moving, so
  # the servo makes one decisive move and goes quiet instead of tracking continuously
  decel_needs_stable_setpoint: bool = False

  # Stock ACC decelerates with the gap between the dash set speed and actual speed.
  # The decel the planners budget for, m/s2; decel overshoot holds the gap that delivers it and
  # no deeper. Unmeasured brands: a smaller budget only means braking starts earlier, the safe
  # way to be wrong.
  stock_a_budget: float = 0.5
  # what the servo's button stream actually moves the dash at, display units/s (0: tap_rate_hz)
  walk_rate: float = 0.
  # the ECU brakes on the gap as soon as it opens, and decel overshoot holds the dash at most this
  # far below actual speed (its gap at the budget), so only this much of a dip is walked before
  # the car brakes at budget; the rest is tracked down, not waited out. Display units (0: the
  # whole dip)
  track_gap: float = 0.
  # decel overshoot's plant inverse: the gap below vEgo (mph) that yields a requested decel, per
  # speed; None for brands without a measured plant, which get no overshoot
  decel_overshoot: dict | None = None

# Mazda CX-5 2022: taps register at 5 Hz and move 1 mph; MRCC will not start decelerating
# until the set speed stops changing. Its deceleration follows the dash gap in stages and keeps
# growing past 10 mph (harder still at highway speed).
ICBM_ACTUATION_PROFILES: dict[str, ICBMActuationProfile] = {
  'mazda': ICBMActuationProfile(
    tap_rate_hz=5.,
    decel_needs_stable_setpoint=True,
    stock_a_budget=0.75,
    walk_rate=4.0,  # mph/s, measured: forged hold frames register as discrete presses
    track_gap=8.0,  # mph
    decel_overshoot={
      'speed_bp': [20.1, 29.1],  # m/s (45, 65 mph)
      'decel_bp': [0.15, 0.30, 0.50, 0.60, 0.70, 0.75, 0.90, 1.05],  # requested decel magnitude, m/s^2
      # gap below vEgo, mph, per speed_bp row. Planners budget 0.75 (stock_a_budget); a request
      # past it means the car is late, and the columns beyond are the ECU's remaining range
      'gap_v': [[2.5, 3.0, 3.75, 6.5, 8.5, 10.0, 16.25, 20.25],
                [2.5, 3.0, 3.5, 6.5, 7.25, 7.75, 12.5, 17.25]],
      'min_decel': 0.15,  # m/s^2; leave gentle coast-downs to the stock behavior
    },
  ),
}

DEFAULT_PROFILE = ICBMActuationProfile()


def get_actuation_profile(brand: str) -> ICBMActuationProfile:
  return ICBM_ACTUATION_PROFILES.get(brand, DEFAULT_PROFILE)
