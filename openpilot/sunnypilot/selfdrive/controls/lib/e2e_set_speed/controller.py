"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Set Speed Assist for experimental mode.

The driving model has no set-speed input and keeps its own following distance. With no lead it
cruises at its own pace, a median 5.5 mph under the set speed in our experimental-mode logs; behind
a lead it followed a median 57 m further back than long_mpc's gap on aggressive. The planner's
min() keeps the car at the model's pace either way.

This lifts the e2e candidate toward what chill mode would do and never lowers it:

  e2e = a_model + lift,  lift -> gain * fade * max(0, floor - a_model)

floor is chill mode's answer, the lower of the planner's own lead MPC and cruise candidates
(upstream's cruise law with SCC and SLA's targets), so with nothing to slow for experimental mode
drives and follows as chill does, closing on the target speed no harder than a production ACC
(approach.py, a step gentler on Relaxed and firmer on Aggressive). gain fades the lift out as
the model eases off and below walking pace, and fade as the road bends, now or ahead, at the
current speed. Every slowdown is the model's: the lift
only lets go of it, and a lead that brakes binds through the MPC candidate at once.

A lift that could go negative made the car pump on a straight road: the plan's far end flickers
by several mph from one frame to the next, and handing back added speed on each flicker
alternated throttle and brake every 15-20 s. Read only through the model's own acceleration and
the road's geometry, the lift has no such input to chase. Design and data:
docs/zoompilot/e2e-set-speed.md.
"""
import math

import numpy as np

from opendbc.car import rate_limit
import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom, log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.limits import op_long_cruise_jerk

Inhibit = custom.LongitudinalPlanZP.E2ESetSpeed.Inhibit
LaneChangeState = log.LaneChangeState

PARAM = "ExperimentalModeSetSpeed"

# The lift fades in above walking pace: stop-and-go stays the model's.
MIN_SPEED = 5.0  # m/s
FULL_SPEED = 8.0  # m/s

# The model's acceleration wanders +-0.15 m/s^2 below 0.3 Hz in cruise and settles near -0.05..-0.1
# at the set speed, where it pushes back toward its own pace; the gain ramps below both.
GAIN_BP = [-0.3, -0.1]  # m/s^2
# No lift where the road ahead bends harder than this at the current speed.
FADE_LAT_BP = [0.6, 1.0]  # m/s^2
LAT_HORIZON = 10.0  # s

# The lift rises at upstream's cruise jerk, as the cruise candidate it stands in for. This is the
# slowest it eases off, e.g. on a lane change or a bend.
LIFT_FALL = 2.4  # m/s^3

T_IDXS = np.asarray(ModelConstants.T_IDXS)
LAT_MASK = T_IDXS <= LAT_HORIZON


class E2ESetSpeedController:
  def __init__(self, params=None, dt: float = DT_MDL):
    self.params = params or Params()
    self.dt = dt
    self.params_frames = int(PARAMS_UPDATE_PERIOD / dt)
    self.frame = -1
    self.enabled = False
    self._reset(Inhibit.disabled)

  def _reset(self, reason) -> None:
    self.gain = 0.
    self.fade = 0.
    self.floor = 0.
    self.lift = 0.
    self.inhibit = reason

  def _update_params(self) -> None:
    if self.frame % self.params_frames == 0:
      self.enabled = self.params.get_bool(PARAM)

  def fill(self, msg) -> None:
    """Report into LongitudinalPlanZP.e2eSetSpeed."""
    msg.gain = float(self.gain)
    msg.fade = float(self.fade)
    msg.floor = float(self.floor)
    msg.boost = float(self.lift)
    msg.inhibit = self.inhibit

  def update(self, sm: messaging.SubMaster, a_model: float, a_chill: float, is_e2e: bool, reset_state: bool,
             steer_lat_accel: float = 0.) -> float:
    """Return the e2e candidate; a_model unchanged whenever the feature is off or idle.
    a_chill: the floor, the lower of the planner's lead MPC and cruise candidates and the approach pace.
    steer_lat_accel: lateral acceleration from the measured steering angle."""
    self.frame += 1
    self._update_params()

    # switched off with a lift on: it eases out below, then the controller idles
    if not self.enabled and self.lift == 0.:
      self._reset(Inhibit.disabled)
      return a_model
    if not is_e2e or reset_state:
      self._reset(Inhibit.inactive)
      return a_model

    md = sm['modelV2']
    v_ego = sm['carState'].vEgo
    vel = np.asarray(md.velocity.x, dtype=float)
    yaw_rate = np.asarray(md.orientationRate.z, dtype=float)
    if (len(vel) != ModelConstants.IDX_N or len(yaw_rate) != ModelConstants.IDX_N or
        not (np.isfinite(vel).all() and np.isfinite(yaw_rate).all()) or
        not all(math.isfinite(x) for x in (v_ego, a_model, a_chill, steer_lat_accel))):
      self._reset(Inhibit.invalid)
      return a_model

    # the road's geometry at the current speed, not the plan's speeds, which flicker. The path
    # reads a sustained bend 10-20% low, so the measured one counts too
    kappa_ahead = float(np.max((np.abs(yaw_rate) / np.maximum(vel, 1.))[LAT_MASK]))
    lat_ahead = max(abs(steer_lat_accel), kappa_ahead * v_ego ** 2)

    self.floor = a_chill
    # stop-and-go stays the model's: the gain fades in above walking pace
    self.gain = float(np.interp(a_model, GAIN_BP, [0., 1.])) * float(np.interp(v_ego, [MIN_SPEED, FULL_SPEED], [0., 1.]))
    self.fade = float(np.interp(lat_ahead, FADE_LAT_BP, [1., 0.]))

    if not self.enabled:
      self.inhibit = Inhibit.disabled
    elif md.meta.laneChangeState != LaneChangeState.off:
      self.inhibit = Inhibit.laneChange
    else:
      self.inhibit = Inhibit.none
    target = self.gain * self.fade * max(0., self.floor - a_model) if self.inhibit == Inhibit.none else 0.
    # Normally the lift eases off at LIFT_FALL. When the model starts to brake the gain drops, and
    # the lift may drop with it in one frame, so at gain 0 the model's braking is not delayed.
    fall = max(LIFT_FALL * self.dt, (1. - self.gain) * self.lift)
    self.lift = rate_limit(target, self.lift, -fall, op_long_cruise_jerk(v_ego) * self.dt)
    return a_model + self.lift
