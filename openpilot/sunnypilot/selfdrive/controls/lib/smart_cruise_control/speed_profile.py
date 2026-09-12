"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Speed profile solver shared by the curve and limit speed planners.

A curve is a region, not a point, and the path ahead can hold several. The standard
backward pass handles both and yields the required deceleration as a by-product:
hold the set speed as long as possible, then decelerate at the budget the platform
can actually deliver, arriving at each constraint at its allowed speed.

Pure functions over arrays; no I/O, no state. See docs/zoompilot/scc-curve-planning.md.
"""
import math
import numpy as np

KAPPA_MIN = 1e-5  # 1/m; flatter than this is straight for speed purposes (100 km radius)
D_FLOOR = 0.5  # m; distance floor so a constraint at the bumper reads as very late, not a div0


def allowed_speed(curvature, a_lat_max: float) -> np.ndarray:
  """Speed at which each path point sits at the lateral-acceleration ceiling; inf where straight."""
  kappa = np.maximum(np.asarray(curvature, dtype=float), 0.)
  return np.where(kappa > KAPPA_MIN, np.sqrt(a_lat_max / np.maximum(kappa, KAPPA_MIN)), np.inf)


def backward_pass(v_allowed, dist, a_budget: float) -> np.ndarray:
  """Max speed at each point such that braking at a_budget still meets every later constraint.

  dist is cumulative path distance from the car; out-of-order or repeated samples
  contribute zero ds and propagate the constraint through unchanged.
  """
  v_allowed = np.asarray(v_allowed, dtype=float)
  dist = np.asarray(dist, dtype=float)
  v_max = v_allowed.copy()
  for i in range(len(v_max) - 2, -1, -1):
    ds = max(dist[i + 1] - dist[i], 0.)
    if math.isinf(v_max[i + 1]):
      continue
    v_max[i] = min(v_allowed[i], math.sqrt(v_max[i + 1] ** 2 + 2. * a_budget * ds))
  return v_max


def required_decel(v_ego: float, v_allowed, dist, d_lead: float = 0.) -> float:
  """Deceleration needed now (m/s2, >= 0) to meet the tightest constraint ahead.

  d_lead is the actuation lead distance (see lead_distance); it is subtracted from every
  constraint's distance because braking only becomes effective after it. A constraint
  already inside the lead distance returns a very large value: the caller is late, and
  clipping to the deliverable budget is the caller's job.
  """
  v = np.asarray(v_allowed, dtype=float)
  d = np.asarray(dist, dtype=float) - d_lead
  binding = v < v_ego
  if not np.any(binding):
    return 0.
  a = (v_ego ** 2 - v[binding] ** 2) / (2. * np.maximum(d[binding], D_FLOOR))
  return float(np.max(a))


def lead_distance(v_ego: float, t_lead: float, a_budget: float = 0., jerk: float = 0.) -> float:
  """Distance eaten before braking is fully effective: actuation lead plus easing into the budget.

  The jerk term is the extra distance of ramping 0 -> a_budget at the consumer's jerk limit
  versus an ideal step, ~ v * a_budget / (2 * jerk).
  """
  d = v_ego * t_lead
  if jerk > 0.:
    d += v_ego * a_budget / (2. * jerk)
  return d


def min_profile_speed(v_max, dist, horizon_d: float) -> float:
  """Lowest profile speed within horizon_d of the car; the dip a slow consumer must
  pre-position for (a dash servo cannot track a continuous profile with 1 mph taps)."""
  v_max = np.asarray(v_max, dtype=float)
  d = np.asarray(dist, dtype=float)
  mask = d <= horizon_d
  if not np.any(mask):
    return float(v_max[0])
  return float(np.min(v_max[mask]))
