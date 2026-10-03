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

  e2e = a_model + authority * gain * max(0, floor - a_model)

floor approaches the target the way stock cruise does, gain fades the boost out as the model
starts to decelerate, and authority drops on any sign of a purposeful slowdown and re-arms only
after a hold. Without a lead nothing above the e2e candidate bounds it (the MPC's synthetic lead
and the e2e cruise candidate both sit near +2 m/s^2), so the trips are the whole safety case.
Design, data and the IQ.Pilot comparison: docs/zoompilot/e2e-set-speed.md.
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

# Authority re-arms slowly after a hold and drops fast. Single-frame trips are early warnings,
# so they are not filtered; the hold keeps a flickering trip from re-arming mid-slowdown.
AUTHORITY_RISE = 0.5  # 1/s
AUTHORITY_FALL = 4.0  # 1/s
HOLD_TIME = 3.0  # s
# A speed target restored with full authority (SCC-V or SLA releasing) must not step the boost in.
BOOST_RISE = 0.5  # m/s^3

# The model's acceleration wanders +-0.15 m/s^2 below 0.3 Hz in steady cruise, so the gain ramps
# outside that band, and it is already zero where the hard trip fires.
GAIN_BP = [-0.2, -0.05]  # m/s^2
MODEL_BRAKE_ACCEL = -0.2  # m/s^2
# Speed the plan loses over 5 s, measured from its own v(0): vEgo reads 2-3% low against GPS.
PLAN_SLOWDOWN = 0.75  # m/s
PLAN_SLOWDOWN_T = 5.0  # s
# Lateral acceleration now or anywhere on the plan. The look-ahead fired before every curve
# entry in the logs, a median 5 s ahead; trips on deceleration alone missed half of them.
LAT_ACCEL_MAX = 1.0  # m/s^2
LAT_HORIZON = 10.0  # s
STOP_SPEED = 2.0  # m/s; a plan dipping below this is a stop

# Upstream's ACC friction circle (selfdrive/controls/lib/longitudinal_planner.py), mirrored
# because importing it from here is circular; tests pin the copy.
A_TOTAL_MAX_BP = [20., 40.]
A_TOTAL_MAX_V = [1.7, 3.2]

T_IDXS = np.asarray(ModelConstants.T_IDXS)
LAT_MASK = T_IDXS <= LAT_HORIZON


class E2ESetSpeedController:
  def __init__(self, params=None, dt: float = DT_MDL):
    self.params = params or Params()
    self.dt = dt
    self.hold_frames = int(round(HOLD_TIME / dt))
    self.params_frames = int(PARAMS_UPDATE_PERIOD / dt)
    self.frame = -1
    self.enabled = False
    self._reset(Inhibit.disabled)

  def _reset(self, reason) -> None:
    self.authority = 0.
    self.gain = 0.
    self.floor = 0.
    self.boost = 0.
    self.hold_left = self.hold_frames
    self.inhibit = reason

  def _update_params(self) -> None:
    if self.frame % self.params_frames == 0:
      self.enabled = self.params.get_bool(PARAM)

  @staticmethod
  def _trip(sm: messaging.SubMaster, a_model: float, v_ego: float, plan_drop: float, plan_min_v: float,
            lat_accel: float, allow_throttle: bool, fcw: bool):
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
    if a_model < MODEL_BRAKE_ACCEL:
      return Inhibit.modelBraking
    if plan_drop < -PLAN_SLOWDOWN:
      return Inhibit.planSlowing
    if lat_accel > LAT_ACCEL_MAX:
      return Inhibit.lateral
    if not allow_throttle:
      return Inhibit.coast
    if md.meta.laneChangeState != LaneChangeState.off:
      return Inhibit.laneChange
    if v_ego < MIN_SPEED:
      return Inhibit.lowSpeed
    return None

  def update(self, sm: messaging.SubMaster, a_model: float, v_cruise: float, is_e2e: bool, reset_state: bool,
             dec_active: bool, allow_throttle: bool, fcw: bool, accel_coast: float) -> float:
    """Return the e2e candidate; a_model unchanged whenever the feature is off or idle."""
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

    lat_accel = max(abs(curvature) * v_ego ** 2, float(np.max(np.abs(yaw_rate * vel)[LAT_MASK])))
    plan_drop = float(np.interp(PLAN_SLOWDOWN_T, T_IDXS, vel)) - vel[0]
    plan_min_v = float(np.min(vel[LAT_MASK]))

    trip = self._trip(sm, a_model, v_ego, plan_drop, plan_min_v, lat_accel, allow_throttle, fcw)
    if trip is not None:
      self.authority = max(0., self.authority - AUTHORITY_FALL * self.dt)
      self.hold_left = self.hold_frames
      self.inhibit = trip
    elif self.hold_left > 0:
      self.hold_left -= 1
      self.inhibit = Inhibit.hold
    else:
      self.authority = min(1., self.authority + AUTHORITY_RISE * self.dt)
      self.inhibit = Inhibit.none

    self.gain = float(np.interp(a_model, GAIN_BP, [0., 1.]))

    floor = float(np.clip((v_cruise - v_ego) / TAU, 0., FLOOR_MAX)) * float(np.interp(v_ego, [MIN_SPEED, FULL_SPEED], [0., 1.]))
    a_total_max = float(np.interp(v_ego, A_TOTAL_MAX_BP, A_TOTAL_MAX_V))
    floor = min(floor, math.sqrt(max(a_total_max ** 2 - lat_accel ** 2, 0.)))
    if not allow_throttle:
      floor = min(floor, accel_coast)
    self.floor = floor

    boost = self.authority * self.gain * max(0., floor - a_model)
    self.boost = min(boost, self.boost + BOOST_RISE * self.dt)
    return a_model + self.boost
