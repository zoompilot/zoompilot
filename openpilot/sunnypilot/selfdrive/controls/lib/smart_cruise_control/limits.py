"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Per-car deceleration planning limits for the curve and limit speed planners.

A car is either openpilot-long or stock ACC for the whole drive, so the budget and the
actuation lead are picked once at init. The numbers are measured, not aspirational:
what each path can actually deliver decides where the braking point goes.
See docs/zoompilot/scc-curve-planning.md.
"""
from dataclasses import dataclass

import numpy as np

from opendbc.car import structs
from opendbc.sunnypilot.car.icbm_actuation_profile import get_actuation_profile
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.speed_profile import required_decel

# openpilot long's cruise candidate: clip(v_cruise - v_ego, A_CRUISE_MIN, max) with a jerk
# clip from J_CRUISE_VALS. Mirrored from selfdrive/controls/lib/longitudinal_planner.py, which
# cannot be imported from here (it imports the SP overlay, which imports this package);
# tests/test_limits.py pins these to the upstream source.
_OP_LONG_A_BUDGET = 1.2
_OP_LONG_J_BP = [0., 10., 25., 40.]
_OP_LONG_J_VALS = [1.6, 1.2, 0.8, 0.6]

# stock ACC: the budget, servo walk rate and tracking gap are the car's measured plant, kept in
# its ICBM actuation profile.
# time from a stable lowered set speed to the ECU actually decelerating; an estimate erring large
_STOCK_RESPONSE_T = 1.0

_MPH_PER_MS = 2.23694

# shared solver gate: a constraint binds once the decel it requires reaches this fraction of
# the budget; below 1.0 leaves headroom for slope and curvature error
COMMIT_FRAC = 0.7

# Publication shaping for the plan aTarget wire, shared by the limiter sources. On stock ACC
# the wire keys ICBM's decel-overshoot lever and may ask deeper than any path delivers; on
# openpilot long it seeds mpc.set_cur_state (stage 0 is pinned to the seed), so there it is
# an actuator command and is clipped to the budget (PlanningLimits.a_pub_min).
A_PUB_MIN = -2.0  # m/s2
PUB_JERK = 2.0  # m/s3; publication ramp where the ECU does its own easing


@dataclass(frozen=True)
class PlanningLimits:
  a_budget: float  # deliverable deceleration, m/s2, positive
  t_lead: float  # fixed actuation lead, s
  op_long: bool
  # stock path only: the dash has to be walked down before the ECU sees the new set speed
  walk_rate: float = 5.  # display units per second the servo actually achieves
  track_gap: float = 0.  # display units walked before the ECU brakes at budget; 0 = the whole dip

  def jerk(self, v_ego: float) -> float:
    """The consumer's own jerk limit easing into a_budget; 0 where the ECU self-smooths."""
    if not self.op_long:
      return 0.
    return float(np.interp(v_ego, _OP_LONG_J_BP, _OP_LONG_J_VALS))

  @property
  def a_pub_min(self) -> float:
    """Deepest aTarget a predicted constraint may publish, m/s2: the budget. Only measured
    geometry may ask for the ECU's range past it (A_PUB_MIN), and its publisher says so."""
    return -self.a_budget

  def pub_jerk(self, v_ego: float) -> float:
    """Jerk the published aTarget ramps at: the consumer's own on openpilot long, PUB_JERK on stock."""
    return self.jerk(v_ego) or PUB_JERK

  def dash_traversal_time(self, delta_v_ms: float) -> float:
    """Seconds of dash walking before a dip of delta_v brakes at budget (stock path only).

    Display units are taken as mph: the measured rates are imperial-only so far, and for a
    lead estimate the ~1.6x metric error is inside the response-time uncertainty anyway.
    """
    if self.op_long or delta_v_ms <= 0.:
      return 0.
    walk = delta_v_ms * _MPH_PER_MS
    if self.track_gap > 0.:
      walk = min(walk, self.track_gap)
    return walk / max(self.walk_rate, 1.)


def get_planning_limits(CP: structs.CarParams) -> PlanningLimits:
  if CP.openpilotLongitudinalControl:
    return PlanningLimits(a_budget=_OP_LONG_A_BUDGET, t_lead=float(CP.longitudinalActuatorDelay), op_long=True)

  profile = get_actuation_profile(CP.brand)
  return PlanningLimits(a_budget=profile.stock_a_budget, t_lead=_STOCK_RESPONSE_T, op_long=False,
                        walk_rate=profile.walk_rate or profile.tap_rate_hz, track_gap=profile.track_gap)


def publish_ramp(a_des: float, a_prev: float, lim: PlanningLimits, v_ego: float, dt: float = DT_MDL,
                 a_floor: float | None = None) -> float:
  """Shape a decel request for the plan aTarget wire: clip to the path, then jerk-limit the step.

  The published aTarget seeds mpc.set_cur_state, which is not jerk-limited the way the
  cruise candidate is, so a one-frame step would reach the actuators as a snap. a_prev is
  the value published last frame. a_floor lets measured geometry ask past the budget.
  """
  floor = lim.a_pub_min if a_floor is None or lim.op_long else max(a_floor, A_PUB_MIN)
  a_des = max(a_des, floor)
  step = lim.pub_jerk(v_ego) * dt
  return float(min(max(a_des, a_prev - step), a_prev + step))


def publish_cap_decel(v_cap: float, d: float, t_min: float, a_prev: float, lim: PlanningLimits, v_ego: float) -> float:
  """publish_ramp toward the decel that brings v_ego down to a predicted cap v_cap in d metres, or
  in t_min seconds when d is shorter (a cap past its sign, a degenerate distance). A prediction
  asks for the budget at most: only the vision planner's measured near field may go past it."""
  a_des = -required_decel(v_ego, [v_cap], [max(d, v_ego * t_min)])
  return publish_ramp(a_des, a_prev, lim, v_ego)
