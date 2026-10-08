"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Set-speed floor for experimental mode.

The driving model has no set-speed input. With no lead it cruises at its own pace, and the
planner's min() keeps the car there: in our experimental-mode logs the model was the binding
candidate for 90% of lead-free straight cruise, a median 5.5 mph under the set speed. The model
barely defends that pace (about -0.02 m/s^2 per m/s above it, a ~50 s time constant); it
accelerates lazily and then holds.

This raises the e2e candidate toward the set speed while the model shows no reason to slow:

  e2e = a_model + gain * max(0, min(floor, bound) - a_model)

floor approaches the target the way stock cruise does and gain fades the boost out as the model
starts to decelerate. bound reads the model's plan over its horizon: the steepest average
slowdown it holds over the next few seconds, and the curve speeds along it. Both move smoothly
with the plan, so a slowdown or a bend shrinks the boost as it approaches instead of tripping it,
and nothing has to be held off once it passes. bound drops at once and recovers through a filter,
which keeps a wobbling plan from pumping the boost. Hazards (FCW, a stop, a lead, the driver, a
lane change) still cut the boost and hold it off for a few seconds.

Without a lead nothing above the e2e candidate bounds it (the MPC's synthetic lead and the e2e
cruise candidate both sit near +2 m/s^2), so the bound and the hazards are the whole safety case.
Design and data: docs/zoompilot/e2e-set-speed.md.
"""
import math

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom, log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD

Inhibit = custom.LongitudinalPlanZP.E2ESetSpeed.Inhibit
LaneChangeState = log.LaneChangeState

PARAM = "ExperimentalModeSetSpeed"

# Floor: close the gap the way stock MRCC does after a set-speed step (~7.7 s, peaking at a
# median 0.57 m/s^2), faded in above walking pace.
TAU = 8.0  # s
FLOOR_MAX = 0.6  # m/s^2
MIN_SPEED = 5.0  # m/s
FULL_SPEED = 8.0  # m/s

# The model's acceleration wanders +-0.15 m/s^2 below 0.3 Hz in steady cruise, so the gain ramps
# outside that band.
GAIN_BP = [-0.2, -0.05]  # m/s^2

# Plan bound: the steepest average acceleration the plan holds from now to any point in
# PLAN_T, measured from its own v(0) (vEgo reads 2-3% low against GPS). Shorter windows are the
# model's wander. The bound closes from FLOOR_MAX to 0 across PLAN_BP.
PLAN_T = (2.0, 6.0)  # s
PLAN_BP = [-0.1, 0.]  # m/s^2
# Curve bound: never plan to pass a point of the plan faster than CURVE_LAT_ACCEL allows there,
# and no boost at all while turning harder than that now.
CURVE_LAT_ACCEL = 0.6  # m/s^2
CURVE_T = (0.5, 10.0)  # s
# bound drops at once and recovers with this time constant
BOUND_TAU = 1.0  # s

# The boost rises gently and backs off faster; a hazard cuts it at the rate a full boost used to
# fall when authority dropped 4/s.
BOOST_RISE = 0.5  # m/s^3
BOOST_FALL = 1.0  # m/s^3
HAZARD_FALL = 2.4  # m/s^3
HOLD_TIME = 3.0  # s after the last hazard

LAT_HORIZON = 10.0  # s, for the friction circle and the stop check
STOP_SPEED = 2.0  # m/s; a plan dipping below this is a stop

# Upstream's ACC friction circle (selfdrive/controls/lib/longitudinal_planner.py), mirrored
# because importing it from here is circular; tests pin the copy.
A_TOTAL_MAX_BP = [20., 40.]
A_TOTAL_MAX_V = [1.7, 3.2]

T_IDXS = np.asarray(ModelConstants.T_IDXS)
LAT_MASK = T_IDXS <= LAT_HORIZON
PLAN_MASK = (T_IDXS >= PLAN_T[0]) & (T_IDXS <= PLAN_T[1])
CURVE_MASK = (T_IDXS >= CURVE_T[0]) & (T_IDXS <= CURVE_T[1])


def plan_bound(vel: np.ndarray) -> float:
  worst_mean = float(np.min((vel[PLAN_MASK] - vel[0]) / T_IDXS[PLAN_MASK]))
  return float(np.interp(worst_mean, PLAN_BP, [0., FLOOR_MAX]))


def curve_bound(vel: np.ndarray, yaw_rate: np.ndarray, lat_now: float) -> float:
  if lat_now > CURVE_LAT_ACCEL:
    return 0.
  curvature = np.abs(yaw_rate[CURVE_MASK]) / np.maximum(vel[CURVE_MASK], 1.)
  v_curve = np.sqrt(CURVE_LAT_ACCEL / np.maximum(curvature, 1e-6))
  return min(float(np.min((v_curve - vel[0]) / T_IDXS[CURVE_MASK])), FLOOR_MAX)


class E2ESetSpeedController:
  def __init__(self, params=None, dt: float = DT_MDL):
    self.params = params or Params()
    self.dt = dt
    self.hold_frames = int(round(HOLD_TIME / dt))
    self.params_frames = int(PARAMS_UPDATE_PERIOD / dt)
    self.bound_k = dt / (BOUND_TAU + dt)
    self.frame = -1
    self.enabled = False
    self._reset(Inhibit.disabled)

  def _reset(self, reason) -> None:
    self.gain = 0.
    self.floor = 0.
    self.bound = math.nan  # unset: the next bound is taken as is
    self.boost = 0.
    # DEC here means FCW, standstill or a predicted stop, and invalid means no model to trust: both
    # hazards. Engaging or leaving experimental mode is not.
    self.hold_left = self.hold_frames if reason in (Inhibit.decActive, Inhibit.invalid) else 0
    self.inhibit = reason

  @property
  def authority(self) -> float:
    """1 unless a hazard holds the boost off."""
    return 1. if self.inhibit in (Inhibit.none, Inhibit.modelBraking, Inhibit.planSlowing, Inhibit.lateral) else 0.

  def _update_params(self) -> None:
    if self.frame % self.params_frames == 0:
      self.enabled = self.params.get_bool(PARAM)

  @staticmethod
  def _hazard(sm: messaging.SubMaster, plan_min_v: float, fcw: bool):
    CS, md, rs = sm['carState'], sm['modelV2'], sm['radarState']
    if fcw:
      return Inhibit.fcw
    if md.meta.hardBrakePredicted:
      return Inhibit.hardBrake
    if sm['controlsState'].forceDecel:
      return Inhibit.forceDecel
    if md.action.shouldStop or plan_min_v < STOP_SPEED:
      return Inhibit.stop
    if rs.leadOne.present or rs.leadTwo.present:
      return Inhibit.lead
    if CS.gasPressed or CS.brakePressed:
      return Inhibit.driver
    if md.meta.laneChangeState != LaneChangeState.off:
      return Inhibit.laneChange
    return None

  def update(self, sm: messaging.SubMaster, a_model: float, v_cruise: float, is_e2e: bool, reset_state: bool,
             dec_active: bool, allow_throttle: bool, fcw: bool, accel_coast: float, steer_lat_accel: float = 0.) -> float:
    """Return the e2e candidate; a_model unchanged whenever the feature is off or idle.
    steer_lat_accel: lateral acceleration from the measured steering angle."""
    self.frame += 1
    self._update_params()

    if not self.enabled:
      self._reset(Inhibit.disabled)
      return a_model
    if not is_e2e or reset_state:
      self._reset(Inhibit.inactive)
      return a_model
    # DEC on a radarless car only blends to slow down (FCW, standstill, a predicted stop)
    if dec_active:
      self._reset(Inhibit.decActive)
      return a_model

    md = sm['modelV2']
    v_ego = sm['carState'].vEgo
    vel = np.asarray(md.velocity.x, dtype=float)
    yaw_rate = np.asarray(md.orientationRate.z, dtype=float)
    curvature = md.action.desiredCurvature
    if (len(vel) != ModelConstants.IDX_N or len(yaw_rate) != ModelConstants.IDX_N or
        not all(math.isfinite(x) for x in (v_ego, a_model, v_cruise, accel_coast, curvature)) or
        not (np.isfinite(vel).all() and np.isfinite(yaw_rate).all())):
      self._reset(Inhibit.invalid)
      return a_model

    lat_now = max(abs(curvature) * v_ego ** 2, abs(steer_lat_accel))
    lat_accel = max(lat_now, float(np.max(np.abs(yaw_rate * vel)[LAT_MASK])))

    floor = float(np.clip((v_cruise - v_ego) / TAU, 0., FLOOR_MAX)) * float(np.interp(v_ego, [MIN_SPEED, FULL_SPEED], [0., 1.]))
    a_total_max = float(np.interp(v_ego, A_TOTAL_MAX_BP, A_TOTAL_MAX_V))
    floor = min(floor, math.sqrt(max(a_total_max ** 2 - lat_accel ** 2, 0.)))
    if not allow_throttle:
      floor = min(floor, accel_coast)
    self.floor = floor

    a_plan, a_curve = plan_bound(vel), curve_bound(vel, yaw_rate, lat_now)
    bound = min(a_plan, a_curve)
    if math.isnan(self.bound) or bound < self.bound:
      self.bound = bound
    else:
      self.bound += self.bound_k * (bound - self.bound)
    self.gain = float(np.interp(a_model, GAIN_BP, [0., 1.]))

    hazard = self._hazard(sm, float(np.min(vel[LAT_MASK])), fcw)
    if hazard is not None:
      self.hold_left = self.hold_frames
      self.inhibit = hazard
    elif self.hold_left > 0:
      self.hold_left -= 1
      self.inhibit = Inhibit.hold
    elif self.gain == 0.:
      self.inhibit = Inhibit.modelBraking
    elif self.bound < floor:
      self.inhibit = Inhibit.planSlowing if a_plan <= a_curve else Inhibit.lateral
    else:
      self.inhibit = Inhibit.none

    if self.authority:
      target, fall = self.gain * max(0., min(floor, self.bound) - a_model), BOOST_FALL
    else:
      target, fall = 0., HAZARD_FALL
    self.boost = max(0., min(max(target, self.boost - fall * self.dt), self.boost + BOOST_RISE * self.dt))
    return a_model + self.boost
