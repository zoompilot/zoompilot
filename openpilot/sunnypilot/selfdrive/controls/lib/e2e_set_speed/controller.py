"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Set-speed floor for experimental mode.

The driving model has no set-speed input. With no lead it cruises at its own pace, and the
planner's min() keeps the car there: in our experimental-mode logs the model was the binding
candidate for 90% of lead-free straight cruise, a median 5.5 mph under the set speed. It
accelerates lazily and then holds.

This raises the e2e candidate toward the set speed, inside a speed envelope read from the
model's own plan:

  e2e = a_model + boost,  boost -> gain * max(0, min(floor, bound) - a_model)

floor approaches the target the way stock cruise does, a little firmer. bound is the envelope:
the model's plan with the speed we added taken back out (the model keeps speed it did not
choose, so its plan carries ours), scaled up toward the target where the plan is flat and left
at the model's own speed where it means to slow, capped at a comfortable curve speed. A backward
pass in distance turns that into the fastest speed at every point from which a gentle slowdown
still meets every ceiling further on, and the boost may only accelerate toward it.

Because the model will not shed speed we added before a bend or a stop, the boost goes negative
when the car is over the envelope: it gives back the speed it put on, never more, so the car
reaches each slowdown at the model's own speed. Hazards (FCW, a lead, the driver, a lane change)
cut the boost and hold it off for a few seconds.

Without a lead nothing above the e2e candidate bounds it (the MPC's synthetic lead and the e2e
cruise candidate both sit near +2 m/s^2), so the envelope and the hazards are the whole safety
case. Design and data: docs/zoompilot/e2e-set-speed.md.
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

# Floor: close the gap a little firmer than stock MRCC after a set-speed step (~7.7 s, peaking at
# a median 0.57 m/s^2, p90 0.72), faded in above walking pace.
TAU = 6.0  # s
FLOOR_MAX = 0.8  # m/s^2
MIN_SPEED = 5.0  # m/s
FULL_SPEED = 8.0  # m/s

# The model's acceleration wanders +-0.15 m/s^2 below 0.3 Hz in steady cruise, so the gain ramps
# outside that band.
GAIN_BP = [-0.2, -0.05]  # m/s^2

# Envelope. Where the plan has dropped TAPER_DROP below where it starts, the model means it and
# the ceiling is its own speed; where it is flat, the ceiling scales its pace up to the target.
TAPER_DROP = 1.5  # m/s
RATIO_MAX = 1.5
CURVE_LAT_ACCEL = 1.0  # m/s^2, no boost past this in a bend the model takes slower than it allows
# Fastest speed from which A_DEC still meets every ceiling further on; the boost accelerates
# toward the value LOOK_T ahead.
A_DEC = 0.4  # m/s^2
LOOK_T = 2.0  # s
# Where the plan has dropped SLOWING_DROP or more, the boost must also reach the model's speed there
# at a constant rate from now: a slowdown starts shaping the boost as soon as it shows.
SLOWING_DROP = 0.5  # m/s
SLOWING_T = (1.0, 10.0)  # s
# The envelope drops at once and recovers with this time constant, so a wobbling plan cannot pump
# the boost.
BOUND_TAU = 1.0  # s

# Give-back: speed this controller put on the car, shed by the model at SHED_RATE in cruise and
# not at all before a slowdown. Over the envelope it is handed back over GIVE_T, at most GIVE_MAX.
SHED_RATE = 0.038  # 1/s
# While another candidate holds the car below the model itself (a lead's MPC), a model-only car would
# be held to the same limit: the speed we added converges out about as fast as the MPC tracks. One
# that binds only under our lift (cruise near the set speed) would not have held a model-only car.
CONVERGE_T = 2.0  # s
GIVE_T = 2.0  # s
GIVE_MAX = 0.6  # m/s^2

BOOST_RATE = 0.5  # m/s^3, both ways
HAZARD_FALL = 2.4  # m/s^3, cutting a boost for a hazard
HOLD_TIME = 3.0  # s after the last hazard

LAT_HORIZON = 10.0  # s, for the friction circle

# Upstream's ACC friction circle (selfdrive/controls/lib/longitudinal_planner.py), mirrored
# because importing it from here is circular; tests pin the copy.
A_TOTAL_MAX_BP = [20., 40.]
A_TOTAL_MAX_V = [1.7, 3.2]

T_IDXS = np.asarray(ModelConstants.T_IDXS)
LAT_MASK = T_IDXS <= LAT_HORIZON
SLOWING_MASK = (T_IDXS >= SLOWING_T[0]) & (T_IDXS <= SLOWING_T[1])


def envelope_bound(vel: np.ndarray, yaw_rate: np.ndarray, pos: np.ndarray, v_target: float, added: float) -> float:
  """Most acceleration the envelope allows now. vel, pos and v_target in the plan's own frame."""
  v_now = float(vel[0])
  curvature = np.abs(yaw_rate) / np.maximum(vel, 1.)
  # the model's own plan: ours taken back out. It holds speed it did not choose before a slowdown
  # (it sheds it only in open cruise, slowly), so its plan carries all of it.
  vel = np.maximum(vel - added, 0.)
  pos = np.maximum.accumulate(pos - pos[0] - added * T_IDXS)
  v0 = max(float(vel[0]), 0.1)
  drop = v0 - np.minimum.accumulate(vel)

  ratio = 1. + (min(max(v_target / v0, 1.), RATIO_MAX) - 1.) * np.clip(1. - drop / TAPER_DROP, 0., 1.)
  v_curve = np.sqrt(CURVE_LAT_ACCEL / np.maximum(curvature, 1e-6))
  ceil = np.minimum(vel * ratio, np.maximum(vel, v_curve))

  # vmax[n] = min over m >= n of sqrt(ceil[m]^2 + 2 A_DEC (pos[m] - pos[n])): a running min in v^2
  reach = ceil ** 2 + 2. * A_DEC * pos
  vmax = np.sqrt(np.minimum.accumulate(reach[::-1])[::-1] - 2. * A_DEC * pos)
  bound = (float(np.interp(max(v_now, 0.5) * LOOK_T, pos, vmax)) - v_now) / LOOK_T

  slowing = SLOWING_MASK & (drop > SLOWING_DROP)
  if slowing.any():
    bound = min(bound, float(np.min((ceil[slowing] - v_now) / T_IDXS[slowing])))
  return bound


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
    self.added = 0.
    self.a_model = 0.
    # DEC here means FCW, standstill or a predicted stop, and invalid means no model to trust: both
    # hazards. Engaging or leaving experimental mode is not.
    self.hold_left = self.hold_frames if reason in (Inhibit.decActive, Inhibit.invalid) else 0
    self.inhibit = reason

  @property
  def authority(self) -> float:
    """1 unless a hazard holds the boost off."""
    return 1. if self.inhibit in (Inhibit.none, Inhibit.modelBraking, Inhibit.planSlowing) else 0.

  def _update_params(self) -> None:
    if self.frame % self.params_frames == 0:
      self.enabled = self.params.get_bool(PARAM)

  def delivered(self, a_out: float, a_e2e: float) -> None:
    """The planner's output and its final e2e candidate this frame. Everything lifted onto the e2e
    candidate over the model (ours and the gap assist's) that reached the car is speed added."""
    if self.inhibit in (Inhibit.disabled, Inhibit.inactive, Inhibit.decActive, Inhibit.invalid):
      return
    lift = a_e2e - self.a_model
    self.added = max(0., self.added + float(np.clip(a_out - self.a_model, min(lift, 0.), max(lift, 0.))) * self.dt)
    if a_out < min(a_e2e, self.a_model) - 0.01:
      self.added *= 1. - self.dt / CONVERGE_T

  def fill(self, msg) -> None:
    """Report into LongitudinalPlanZP.e2eSetSpeed."""
    msg.authority = float(self.authority)
    msg.gain = float(self.gain)
    msg.floor = float(self.floor)
    msg.boost = float(self.boost)
    msg.inhibit = self.inhibit
    msg.bound = float(self.bound) if math.isfinite(self.bound) else 0.
    msg.added = float(self.added)

  @staticmethod
  def _hazard(sm: messaging.SubMaster, fcw: bool):
    CS, md, rs, cs = sm['carState'], sm['modelV2'], sm['radarState'], sm['controlsState']
    if fcw:
      return Inhibit.fcw
    if md.meta.hardBrakePredicted:
      return Inhibit.hardBrake
    if cs.forceDecel:
      return Inhibit.forceDecel
    if rs.leadOne.present or rs.leadTwo.present:
      return Inhibit.lead
    if CS.gasPressed or CS.brakePressed:
      return Inhibit.driver
    if md.meta.laneChangeState != LaneChangeState.off:
      return Inhibit.laneChange
    return None

  def update(self, sm: messaging.SubMaster, a_model: float, v_cruise: float, is_e2e: bool, reset_state: bool,
             dec_active: bool, allow_throttle: bool, fcw: bool, accel_coast: float, steer_lat_accel: float = 0.) -> float:
    """Return the e2e candidate; a_model unchanged whenever the feature is off or idle. Call delivered()
    with the planner's output once it has chosen.
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
    pos = np.asarray(md.position.x, dtype=float)
    curvature = md.action.desiredCurvature
    plan = (vel, yaw_rate, pos)
    if (any(len(x) != ModelConstants.IDX_N for x in plan) or not all(np.isfinite(x).all() for x in plan) or
        not all(math.isfinite(x) for x in (v_ego, a_model, v_cruise, accel_coast, curvature))):
      self._reset(Inhibit.invalid)
      return a_model

    self.a_model = a_model
    self.added *= 1. - SHED_RATE * self.dt

    lat_now = max(abs(curvature) * v_ego ** 2, abs(steer_lat_accel))
    lat_accel = max(lat_now, float(np.max(np.abs(yaw_rate * vel)[LAT_MASK])))

    floor = float(np.clip((v_cruise - v_ego) / TAU, 0., FLOOR_MAX)) * float(np.interp(v_ego, [MIN_SPEED, FULL_SPEED], [0., 1.]))
    a_total_max = float(np.interp(v_ego, A_TOTAL_MAX_BP, A_TOTAL_MAX_V))
    floor = min(floor, math.sqrt(max(a_total_max ** 2 - lat_accel ** 2, 0.)))
    if not allow_throttle:
      floor = min(floor, accel_coast)
    self.floor = floor

    # the plan's speeds run a little above vEgo (which reads 2-3% low); compare like with like
    bound = envelope_bound(vel, yaw_rate, pos, v_cruise + float(vel[0]) - v_ego, self.added)
    if math.isnan(self.bound) or bound < self.bound:
      self.bound = bound
    else:
      self.bound += self.bound_k * (bound - self.bound)
    self.gain = float(np.interp(a_model, GAIN_BP, [0., 1.]))

    hazard = self._hazard(sm, fcw)
    if hazard is not None:
      self.hold_left = self.hold_frames
      self.inhibit = hazard
    elif self.hold_left > 0:
      self.hold_left -= 1
      self.inhibit = Inhibit.hold
    elif self.gain == 0.:
      self.inhibit = Inhibit.modelBraking
    elif self.bound < floor:
      self.inhibit = Inhibit.planSlowing
    else:
      self.inhibit = Inhibit.none

    target = self.gain * max(0., min(floor, self.bound) - a_model) if self.authority else 0.
    # over the envelope, hand back what we added; behind a lead the MPC and the gap assist own the
    # speed, so nothing new is handed back there and what was is let go at the normal rate
    give = min(self.added / GIVE_T, GIVE_MAX)
    if self.bound < a_model and give > 0. and hazard != Inhibit.lead:
      target = -min(give, a_model - self.bound)
    # a hazard cuts a boost fast; handing speed back is never urgent
    fall = HAZARD_FALL if not self.authority and self.boost > 0. else BOOST_RATE
    self.boost = max(-give, float(np.clip(target, self.boost - fall * self.dt, self.boost + BOOST_RATE * self.dt)))
    return a_model + self.boost
