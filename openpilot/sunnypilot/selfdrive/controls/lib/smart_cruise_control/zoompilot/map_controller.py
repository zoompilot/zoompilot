"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Map-based curve speed targets on the shared decel solver: a target binds under the same
commit gate as the vision path, publishes the decel it needs (budget at most), and at
speed must persist before it binds. The state machine and update loop are sunnypilot's.
"""
import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.navd.helpers import Coordinate, coordinate_from_param
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.limits import COMMIT_FRAC, PlanningLimits, get_planning_limits, \
  publish_cap_decel
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.map_controller import (
  ACTIVE_STATES, ENABLED_STATES, R, TO_RADIANS, MapState, SmartCruiseControlMap as UpstreamMap, distance_to_point, velocities_from_param)
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.speed_profile import lead_distance, required_decel

__all__ = ['ACTIVE_STATES', 'ENABLED_STATES', 'MapState', 'R', 'SmartCruiseControlMap', 'target_binds']

_T_FALLBACK = 2.8  # s; decel horizon when the target's distance is degenerate
# a map target is a prediction that can flicker on over road that never curves (mapd matching
# the car to a side street: route 25c, seven targets of 19-49 mph at 55 mph, 0.7-2.1 s each).
# A new or deeper target must be the selection for this long before it binds. Corpus (6.2 h
# with the map on): phantoms at speed last 0.7-2.1 s (one 6 s), real targets 5-12.6 s and
# appear a median 10.5 s before the apex. None below 45 mph, where no phantom was seen and a
# curve comes up fast; over the ramp a real curve at 45-46 mph waits under 0.4 s. At 2.5 s
# this clears 7 of the 10 phantoms (4.3 to 2.2 per hour above 45 mph) and leaves the
# tightest real curve 0.6 s of slack after the actuation lead
_CONFIRM_V_BP = [20.1, 22.4]  # m/s (45, 50 mph)
_CONFIRM_T = [0., 2.5]  # s


def target_binds(v_ego: float, tv, d, lim: PlanningLimits) -> np.ndarray:
  """Which targets (speeds tv at distances d) bind: the decel each requires past the actuation
  lead reaches the commit fraction of the budget; same solver and gate as the vision path."""
  tv = np.asarray(tv, dtype=float)
  d = np.asarray(d, dtype=float)
  jerk = lim.jerk(v_ego)
  d_lead = [lead_distance(v_ego, lim.t_lead + lim.dash_traversal_time(v_ego - v), lim.a_budget, jerk) for v in tv]
  a_req = np.array([required_decel(v_ego, [v], [dd], dl) for v, dd, dl in zip(tv, d, d_lead, strict=True)])
  return (tv <= v_ego) & (a_req >= COMMIT_FRAC * lim.a_budget)


class SmartCruiseControlMap(UpstreamMap):
  def __init__(self, CP):
    super().__init__()
    self.limits = get_planning_limits(CP)
    self.v_target = 0.
    self.target_distance = 0.
    self.candidate = None  # (v, lat, lon) selected this frame, bound once confirmed
    self.candidate_age = 0  # frames the same candidate has been selected
    self._a_out = 0.

  def get_a_target_from_control(self) -> float:
    # keys ICBM's overshoot gap on stock ACC; ramped since the plan aTarget seeds the MPC
    if self.is_active and 0. < self.v_target < self.v_ego:
      self._a_out = publish_cap_decel(self.v_target, self.target_distance, _T_FALLBACK, self._a_out, self.limits, self.v_ego)
    else:
      self._a_out = self.a_ego
    return self._a_out

  def update_calculations(self) -> None:
    self.last_position = coordinate_from_param("LastGPSPosition", self.mem_params) or Coordinate(0.0, 0.0)
    self.target_velocities = velocities_from_param("MapTargetVelocities", self.mem_params) or []
    lat, lon = self.last_position.latitude * TO_RADIANS, self.last_position.longitude * TO_RADIANS
    distances = [distance_to_point(lat, lon, p["latitude"] * TO_RADIANS, p["longitude"] * TO_RADIANS) for p in self.target_velocities]

    # only the path from our position forward (the nearest point, if one is within 1 km)
    start = int(np.argmin(distances)) if distances and min(distances) < 1000. else 0
    forward = self.target_velocities[start:]
    forward_d = distances[start:]
    binds = target_binds(self.v_ego, [p["velocity"] for p in forward], forward_d, self.limits)
    valid = [(float(p["velocity"]), p["latitude"], p["longitude"], d) for p, d, b in zip(forward, forward_d, binds, strict=True) if b]
    best = min(valid, key=lambda x: x[0], default=None)  # the slowest target, nearest on a tie

    candidate = best[:3] if best else None
    self.candidate_age = self.candidate_age + 1 if candidate == self.candidate else 0
    self.candidate = candidate
    t_confirm = float(np.interp(self.v_ego, _CONFIRM_V_BP, _CONFIRM_T))
    confirmed = candidate is not None and self.candidate_age * DT_MDL >= t_confirm

    if self.v_target != 0. and (best is None or self.v_target < best[0] or not confirmed):
      for p, d in zip(forward, forward_d, strict=True):
        if p["velocity"] <= self.v_ego and (p["velocity"], p["latitude"], p["longitude"]) == (self.v_target, self.target_lat, self.target_lon):
          # still ahead, just under the commit gate (or a deeper target is not confirmed yet):
          # keep the target at its current distance, since the published decel divides by it
          self.target_distance = d
          return
      self.v_target, self.target_lat, self.target_lon, self.target_distance = 0., 0., 0., 0.

    if confirmed:
      self.v_target, self.target_lat, self.target_lon, self.target_distance = best
