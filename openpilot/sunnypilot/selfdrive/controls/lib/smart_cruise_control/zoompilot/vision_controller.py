"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Vision-based curve speed planning over the model path.

Geometry-derived curvature remains independent of planned speed. A backward pass applies
the platform deceleration budget to produce the speed profile. The update loop and params
refresh are sunnypilot's; the solver and state machine are ours. See
docs/zoompilot/scc-curve-planning.md for tuning data and design details.
"""
import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.limits import (
  A_PUB_MIN, COMMIT_FRAC, get_planning_limits, publish_ramp)
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.speed_profile import (
  allowed_speed, backward_pass, lead_distance, min_profile_speed, required_decel)
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.vision_controller import (
  ACTIVE_STATES, ENABLED_STATES, VisionState, SmartCruiseControlVision as UpstreamVision)

__all__ = ['ACTIVE_STATES', 'ENABLED_STATES', 'SmartCruiseControlVision', 'VisionState']

# Curves are taken at or below this lateral acceleration, m/s2. Keyed on the set speed, like the
# escalation ceiling, so the ceiling does not rise as the car slows. Back roads (set 40 mph or
# less) get the higher one; highway curves keep 1.8, where hard braking was the complaint (doc).
_A_LAT_REG_V_BP = [17.9, 26.8]  # m/s set speed; 40 -> 60 mph
_A_LAT_REG_V = [2.0, 1.8]
# Reserve margin for actuation delay at the apex.
_PLAN_MARGIN = 0.95

# Compensate the measured distance-dependent curvature bias before solving. The gain returns
# to one in the near field, changing brake timing without changing apex speed.
_KAPPA_BIAS_D = [0., 30., 50., 70., 90., 110.]  # m along the path
# Reciprocal of the measured ratio, capped at the validated range.
_KAPPA_BIAS_GAIN = [1.0, 1.06, 1.14, 1.22, 1.42, 1.5]
# Fade the correction above its 30-50 mph fit range.
_KAPPA_BIAS_V_BP = [22.4, 26.8]  # m/s; full correction to 50 mph, none from 60 mph
_KAPPA_BIAS_V_FADE = [1.0, 0.0]
# Both models share the table: the big one under-reads less at range, and its own fit by
# seconds ahead entered curves hotter (doc).

# Below the bias band the far read also flickers: a bend seen at 150 m can vanish from the
# path at 100 m and come back at 50 m, and the planner released 100 m short of the apex (the
# old servo only arrived slow because its gap decayed slowly after release). On stock ACC a
# committed bend is therefore held, with its speed and remaining distance, until the near
# window reaches where it was seen. A phantom binds for a frame or two, a real bend for
# seconds before the read drops it, so the latch waits 2 s; the route-sim sweep is in the doc.
_HOLD_V_MAX = _KAPPA_BIAS_V_BP[0]  # m/s
_HOLD_SEEN_T = 2.0  # s

# Only the measured near field may ask past the budget, and only where arriving hot costs
# more than braking hard: cruising below 50 mph. From a 60 mph set speed the ECU's range past
# the budget is the braking that was reported as too hard, for one hot curve in 13 (route
# sim). Keyed on the set speed so the ceiling does not creep back up as the car slows.
_ESCALATION_V_BP = _KAPPA_BIAS_V_BP

# Use hysteresis below the commit threshold.
_RELEASE_FRAC = 0.3
# The car is still in a curve while the near field limits it below vEgo plus this; past it the
# plan hands back and the ICBM restore stays capped at vAheadMin.
_IN_CURVE_MARGIN = 1.0  # m/s

# The near field is measured rather than predicted: it holds the planner through a curve and is
# the only part of the path allowed to ask past the budget, so what bounds it is the model
# flagging a bend that is not there. The small model's read holds to 3 s
# and a 4 s window quadruples its past-budget phantoms above 50 mph; the big model (modelV2.big)
# reads and flags at 3-4 s as the small one does at 2-3 s (model_reach.py; numbers in the doc).
_NEAR_T = 3.0  # s
_NEAR_T_BIG = 4.0  # s

_V_FLOOR = 0.5  # m/s; model velocity floor when converting yaw rate to curvature

# UI state thresholds do not affect control output.
_TURNING_LAT_ACC_TH = 1.6  # current lat acc above this displays as turning
_LEAVING_LAT_ACC_TH = 1.3  # turning displays as leaving below this
_FINISH_LAT_ACC_TH = 1.1  # leaving ends below this


class SmartCruiseControlVision(UpstreamVision):
  def __init__(self, CP):
    super().__init__()
    self.limits = get_planning_limits(CP)
    self._reset_solver()
    self.a_out = 0.

  def _reset_solver(self) -> None:
    self.solver_valid = False
    self.solver_active = False
    self.a_required = 0.
    self.a_needed = 0.
    self.v_profile_now = float('inf')
    self.v_dip_ahead = float('inf')
    self.v_near_min = float('inf')
    self.v_dip_held = float('inf')  # committed bend below the bias band: its speed
    self.d_held = 0.  # and its remaining distance
    self.seen_frames = 0  # consecutive frames a bend has bound
    self.max_pred_lat_acc = 0.

  def _update_calculations(self, sm: messaging.SubMaster) -> None:
    if not self.long_enabled or not self.enabled:
      self._reset_solver()
      return

    model = sm['modelV2']
    rate_z = np.abs(np.asarray(model.orientationRate.z, dtype=float))
    vel = np.asarray(model.velocity.x, dtype=float)
    x = np.asarray(model.position.x, dtype=float)
    y = np.asarray(model.position.y, dtype=float)
    if len(rate_z) < 2 or not (len(rate_z) == len(vel) == len(x) == len(y)):
      self._reset_solver()
      return

    self.current_lat_acc = self.v_ego ** 2 * abs(sm['controlsState'].curvature)

    # Derive speed-independent curvature from model geometry.
    kappa = rate_z / np.maximum(vel, _V_FLOOR)
    dist = np.empty_like(x)
    dist[0] = 0.
    dist[1:] = np.cumsum(np.hypot(np.diff(x), np.diff(y)))

    lim = self.limits
    a_lat_max = float(np.interp(self.v_cruise_setpoint, _A_LAT_REG_V_BP, _A_LAT_REG_V)) * _PLAN_MARGIN
    # The raw near field says whether the car is in a curve.
    near_d = max(self.v_ego, MIN_V) * (_NEAR_T_BIG if model.big else _NEAR_T)
    near = dist <= near_d  # dist[0] is 0, so never empty
    self.v_near_min = float(np.min(allowed_speed(kappa[near], a_lat_max)))
    # Publish near-path lateral acceleration for UI state.
    self.max_pred_lat_acc = float(np.max(kappa[near]) * self.v_ego ** 2)

    # Use corrected curvature for brake timing.
    fade = np.interp(self.v_ego, _KAPPA_BIAS_V_BP, _KAPPA_BIAS_V_FADE)
    kappa = kappa * (1. + (np.interp(dist, _KAPPA_BIAS_D, _KAPPA_BIAS_GAIN) - 1.) * fade)
    v_allowed = allowed_speed(kappa, a_lat_max)

    # Stock ACC lead time includes the set-speed traversal to the lowest target.
    t_lead = lim.t_lead
    if not lim.op_long:
      v_dip = float(np.min(v_allowed))
      if np.isfinite(v_dip):
        t_lead += lim.dash_traversal_time(max(self.v_ego - max(v_dip, MIN_V), 0.))
    d_lead = lead_distance(self.v_ego, t_lead, lim.a_budget, lim.jerk(self.v_ego))

    self.a_required = required_decel(self.v_ego, v_allowed, dist, d_lead)
    # What the car must still shed from here. The commit gate budgets the stock dash walk in
    # d_lead, but once committed the servo is already walking, so only the ECU's response is
    # still ahead. Past the budget it means the car is late, which only measured geometry may
    # claim: a predicted constraint beyond the near window asks for the budget at most.
    d_resp = lead_distance(self.v_ego, lim.t_lead)
    if lim.op_long:
      self.a_needed = self.a_required
    else:
      a_near = required_decel(self.v_ego, np.where(near, v_allowed, np.inf), dist, d_resp)
      a_far = required_decel(self.v_ego, np.where(near, np.inf, v_allowed), dist, d_resp)
      a_ceiling = float(np.interp(self.v_cruise_setpoint, _ESCALATION_V_BP, [a_near, lim.a_budget]))
      self.a_needed = max(min(a_near, a_ceiling), min(a_far, lim.a_budget))
    v_max = backward_pass(v_allowed, dist, lim.a_budget)
    self.v_profile_now = float(v_max[0])
    self.v_dip_ahead = min_profile_speed(v_max, dist, float(dist[-1]))

    # A held bend keeps its speed and distance on the plan until the near window reaches it.
    self.d_held -= self.v_ego * DT_MDL
    held = np.isfinite(self.v_dip_held) and self.d_held > near_d
    if not held:
      self.v_dip_held = float('inf')
    else:
      # a prediction like the far field: it asks for the budget at most
      a_held = required_decel(self.v_ego, [self.v_dip_held], [self.d_held], d_resp)
      self.a_needed = max(self.a_needed, min(a_held, lim.a_budget))
      self.v_dip_ahead = min(self.v_dip_ahead, self.v_dip_held)

    # Commit near the platform budget and retain control through the curve with hysteresis.
    commit = self.a_required >= COMMIT_FRAC * lim.a_budget
    seen = self.a_required >= _RELEASE_FRAC * lim.a_budget
    in_curve = np.isfinite(self.v_near_min) and self.v_near_min < self.v_ego + _IN_CURVE_MARGIN
    self.solver_active = commit or (self.solver_active and (seen or in_curve or held))
    self.solver_valid = True

    # Latch the bend that binds, or a deeper one, while committed below the bias band, once
    # it has bound long enough not to be a single-frame flicker.
    self.seen_frames = self.seen_frames + 1 if seen else 0
    i_dip = int(np.argmin(v_max))
    if lim.op_long or not self.solver_active or self.v_ego >= _HOLD_V_MAX:
      self.v_dip_held = float('inf')
    elif self.seen_frames * DT_MDL >= _HOLD_SEEN_T and v_max[i_dip] <= self.v_dip_held:
      self.v_dip_held = float(v_max[i_dip])
      self.d_held = float(dist[i_dip])

  def _update_state_machine(self) -> tuple[bool, bool]:
    if self.state != VisionState.disabled:
      # Longitudinal and feature disable take priority over active states.
      if not self.long_enabled or not self.enabled:
        self.state = VisionState.disabled
      elif self.long_override:
        self.state = VisionState.overriding

      else:
        if self.state == VisionState.enabled:
          # Do not start a turn-control cycle below the operating speed.
          if self.v_ego <= MIN_V:
            pass
          elif self.solver_active:
            self.state = VisionState.entering

        elif self.state == VisionState.overriding:
          if not self.long_override:
            self.state = VisionState.enabled

        elif self.state == VisionState.entering:
          if self.current_lat_acc >= _TURNING_LAT_ACC_TH:
            self.state = VisionState.turning
          elif not self.solver_active:
            self.state = VisionState.enabled

        elif self.state == VisionState.turning:
          if self.current_lat_acc <= _LEAVING_LAT_ACC_TH:
            self.state = VisionState.leaving

        elif self.state == VisionState.leaving:
          if self.current_lat_acc >= _TURNING_LAT_ACC_TH:
            self.state = VisionState.turning
          elif self.current_lat_acc < _FINISH_LAT_ACC_TH and not self.solver_active:
            self.state = VisionState.enabled

    elif self.state == VisionState.disabled:
      if self.long_enabled and self.enabled:
        if self.long_override:
          self.state = VisionState.overriding
        else:
          self.state = VisionState.enabled

    enabled = self.state in ENABLED_STATES
    active = self.state in ACTIVE_STATES

    return enabled, active

  @property
  def _controlling(self) -> bool:
    return self.is_active and self.solver_active

  @property
  def v_ahead_min(self) -> float:
    """Lowest planned speed on the horizon for the ICBM restore gate, m/s.

    0 means no lookahead (feature off or no fresh profile) and the servo falls back to
    its stillness heuristic; a clear road caps at 255 (V_CRUISE_UNSET convention).
    """
    if not (self.enabled and self.solver_valid):
      return 0.
    return float(min(self.v_dip_ahead, 255.))

  def get_a_target_from_control(self) -> float:
    if not self._controlling:
      # Seed the activation ramp from the idle acceleration wire.
      self.a_out = self.a_ego
      return self.a_out

    # Bound near-distance required deceleration by the speed error to the profile minimum.
    a_need = self.a_needed
    if np.isfinite(self.v_dip_ahead):
      a_need = min(a_need, max(self.v_ego - self.v_dip_ahead, 0.))
    # Measured geometry may use the ECU's range past the budget; jerk-limit the published command.
    self.a_out = publish_ramp(-a_need, self.a_out, self.limits, self.v_ego, a_floor=A_PUB_MIN)
    return self.a_out

  def get_v_target_from_control(self) -> float:
    if not self._controlling:
      return V_CRUISE_UNSET

    v_lead = self.v_ego + max(-self.a_required, A_PUB_MIN)
    if self.limits.op_long:
      # Lead vEgo by the required decel for the unit-gain cruise candidate, within the profile,
      # and not below the plan minimum once the P candidate reaches its budget.
      v = min(self.v_profile_now, v_lead)
      if np.isfinite(self.v_dip_ahead):
        v = max(v, self.v_dip_ahead)
    else:
      # Pre-position the discrete stock-ACC setpoint at the horizon minimum, and lead vEgo down
      # only while braking: with nothing to brake for the dash follows the profile up.
      v = min(self.v_profile_now, self.v_dip_ahead)
      if self.a_required > 0.:
        v = min(v, v_lead)
    return max(v, MIN_V)

  def _update_solution(self) -> float:
    # the decel comes from the speed profile (get_a_target_from_control), not the state
    return 0.
