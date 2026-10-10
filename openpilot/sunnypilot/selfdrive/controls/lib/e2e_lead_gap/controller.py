"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Follow-distance assist for experimental mode.

Behind a lead the driving model keeps its own gap, not the personality's. In our experimental-mode
logs it followed a median 57 m further back than long_mpc's gap on aggressive and 7.5 m on
standard, the excess growing with speed (+27 m at 15-20 m/s, +52 m at 20-25), and the e2e
candidate was the one the planner took 49-89% of that time. Set Speed Nudge stands down whenever
there is a lead, so nothing pulls the car in.

This lifts the e2e candidate toward the MPC's, the plan that holds the personality's gap:

  e2e = a_model + authority * gain * weight * min(cap, a_mpc - a_model)

It can at most tie the MPC candidate, so the planner's min() never follows closer or accelerates
harder than chill mode would at that moment. weight opens with the gap excess and ego speed, gain
fades the lift out as the model starts braking (so it settles where the model pushes back rather
than overruling it), and the cap is per personality. Authority uses the nudge's original trips and
single 3 s hold (none of its soft holds or hysteresis) with the lead conditions on top: a new or
changed lead, a lead that is braking or slow, or a forecast that has it slowing. Design and data: docs/zoompilot/e2e-lead-gap.md.
"""
import math

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom, log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import COMFORT_BRAKE, STOP_DISTANCE, get_T_FOLLOW
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed import controller as nudge

Inhibit = custom.LongitudinalPlanZP.E2ELeadGap.Inhibit
LaneChangeState = log.LaneChangeState
Personality = log.LongitudinalPersonality

PARAM = "ExperimentalModeLeadGap"

# Gap beyond long_mpc's that opens the lift: a deadband for the model's own wander, then full by
# GAP_BP[1]. Chill mode itself rides a median 4-12 m beyond it.
GAP_BP = [3., 10.]  # m
# Ego speed the lift fades in over; stop-and-go stays the model's.
SPEED_BP = [5., 8.]  # m/s
# Most the lift adds, per personality.
CAP = {int(Personality.aggressive): 0.5, int(Personality.standard): 0.4, int(Personality.relaxed): 0.3}  # m/s^2
# The model braking to hold its own gap ends the lift here: the car settles where it pushes back.
GAIN_BP = [-0.4, -0.1]  # m/s^2
MODEL_BRAKE_ACCEL = -0.5  # m/s^2
PLAN_SLOWDOWN = 1.0  # m/s the model's plan loses within PLAN_SLOWDOWN_T
PLAN_SLOWDOWN_T = 5.0  # s
LAT_ACCEL_MAX = 1.0  # m/s^2, now or anywhere on the plan
STOP_SPEED = 2.0  # m/s; a plan dipping below this is a stop
# The lead: steady, not braking, moving, and the same car for a while.
LEAD_BRAKE_ACCEL = -0.5  # m/s^2
LEAD_SLOWDOWN = 1.0  # m/s the lead's forecast loses within LEAD_SLOWDOWN_T
LEAD_SLOWDOWN_T = 3.0  # s
LEAD_MIN_SPEED = 5.0  # m/s
LEAD_PROB = 0.8
LEAD_JUMP = 3.0  # m off where the last frame's lead should be
# A gentler lift than the no-lead nudge; it is closing on a car.
BOOST_RISE = 0.25  # m/s^3
# Re-arms at the nudge's original rate: behind a lead the picture changes faster than on an empty road.
AUTHORITY_RISE = 0.5  # 1/s
AUTHORITY_FALL = 4.0  # 1/s

LEAD_T_IDXS = np.asarray(ModelConstants.LEAD_T_IDXS)
T_IDXS = nudge.T_IDXS
LAT_MASK = nudge.LAT_MASK
SLOWDOWN_MASK = T_IDXS <= PLAN_SLOWDOWN_T


def plan_drop(vel: np.ndarray) -> float:
  """Most speed the model's plan loses within PLAN_SLOWDOWN_T, from its own v(0): a dip that recovers
  by then counts too."""
  return min(float(np.min(vel[SLOWDOWN_MASK])), float(np.interp(PLAN_SLOWDOWN_T, T_IDXS, vel))) - float(vel[0])


def desired_gap(v_ego: float, v_lead: float, personality) -> float:
  """The gap long_mpc's obstacle cost settles at for this personality."""
  return get_T_FOLLOW(personality) * v_ego + STOP_DISTANCE + (v_ego ** 2 - v_lead ** 2) / (2 * COMFORT_BRAKE)


class E2ELeadGapController:
  def __init__(self, params=None, dt: float = DT_MDL):
    self.params = params or Params()
    self.dt = dt
    self.hold_frames = int(round(nudge.HOLD_TIME / dt))
    self.params_frames = int(PARAMS_UPDATE_PERIOD / dt)
    self.frame = -1
    self.enabled = False
    self.last_d_rel = math.nan
    self.last_track_id = -1
    self._reset(Inhibit.disabled)

  def _reset(self, reason) -> None:
    self.authority = 0.
    self.gain = 0.
    self.weight = 0.
    self.gap_excess = 0.
    self.boost = 0.
    self.hold_left = self.hold_frames
    self.inhibit = reason
    self.last_d_rel = math.nan
    self.last_track_id = -1

  def _update_params(self) -> None:
    if self.frame % self.params_frames == 0:
      self.enabled = self.params.get_bool(PARAM)

  def fill(self, msg) -> None:
    """Report into LongitudinalPlanZP.e2eLeadGap."""
    msg.authority = float(self.authority)
    msg.gain = float(self.gain)
    msg.weight = float(self.weight)
    msg.gapExcess = float(self.gap_excess)
    msg.boost = float(self.boost)
    msg.inhibit = self.inhibit

  def _lead_trip(self, sm: messaging.SubMaster):
    rs, md = sm['radarState'], sm['modelV2']
    lead = rs.leadOne
    # a cut-in, a lane change ahead or a vision swap: let the model judge the new car first
    expected = self.last_d_rel + lead.vRel * self.dt
    # a radar car's track can swap to another car at a similar range; camera leads are id -1
    track_changed = lead.radar and lead.radarTrackId != self.last_track_id
    self.last_d_rel = lead.dRel if lead.present else math.nan
    self.last_track_id = lead.radarTrackId if lead.present else -1
    if not lead.present:
      return Inhibit.noLead
    leads = [lead] + ([rs.leadTwo] if rs.leadTwo.present else [])
    if any(ld.modelProb < LEAD_PROB for ld in leads):
      return Inhibit.leadUncertain
    if any(ld.vLead < LEAD_MIN_SPEED for ld in leads):
      return Inhibit.leadSlow
    if any(ld.aLeadK < LEAD_BRAKE_ACCEL for ld in leads):
      return Inhibit.leadBraking
    # each present lead's own forecast, lowest point within LEAD_SLOWDOWN_T
    for k in range(len(leads)):
      if k < len(md.leadsV3) and len(md.leadsV3[k].v) == len(LEAD_T_IDXS):
        v = np.asarray(md.leadsV3[k].v, dtype=float)
        v_min = min(float(np.min(v[LEAD_T_IDXS <= LEAD_SLOWDOWN_T])), float(np.interp(LEAD_SLOWDOWN_T, LEAD_T_IDXS, v)))
        if np.isfinite(v).all() and v_min < v[0] - LEAD_SLOWDOWN:
          return Inhibit.leadBraking
    if track_changed or not (abs(lead.dRel - expected) < LEAD_JUMP):
      return Inhibit.leadChanged
    return None

  def update(self, sm: messaging.SubMaster, a_model: float, a_mpc: float, is_e2e: bool, reset_state: bool,
             dec_active: bool, allow_throttle: bool, fcw: bool, a_cruise: float = math.inf,
             steer_lat_accel: float = 0.) -> float:
    """a_cruise: the planner's cruise candidate (last frame's is fine), so the lift idles while cruise
    binds. steer_lat_accel: lateral acceleration from the measured steering angle."""
    """Return the e2e candidate; a_model unchanged whenever the feature is off or idle."""
    self.frame += 1
    self._update_params()

    if not self.enabled:
      self._reset(Inhibit.disabled)
      return a_model
    if not is_e2e or reset_state:
      self._reset(Inhibit.inactive)
      return a_model
    if dec_active:
      self._reset(Inhibit.decActive)
      return a_model

    CS, md, rs = sm['carState'], sm['modelV2'], sm['radarState']
    v_ego = CS.vEgo
    vel = np.asarray(md.velocity.x, dtype=float)
    yaw_rate = np.asarray(md.orientationRate.z, dtype=float)
    curvature = md.action.desiredCurvature
    lead = rs.leadOne
    if (len(vel) != ModelConstants.IDX_N or len(yaw_rate) != ModelConstants.IDX_N or
        not all(math.isfinite(x) for x in (v_ego, a_model, a_mpc, curvature, lead.dRel, lead.vLead, lead.vRel)) or
        not (np.isfinite(vel).all() and np.isfinite(yaw_rate).all())):
      self._reset(Inhibit.invalid)
      return a_model

    lat_accel = max(abs(curvature) * v_ego ** 2, float(np.max(np.abs(yaw_rate * vel)[LAT_MASK])), abs(steer_lat_accel))
    plan_drop_v = plan_drop(vel)
    plan_min_v = float(np.min(vel[LAT_MASK]))

    trip = self._lead_trip(sm)
    if trip is None:
      trip = self._trip(sm, a_model, v_ego, plan_drop_v, plan_min_v, lat_accel, allow_throttle, fcw)
    if trip is not None:
      # another car (or none) restarts from nothing; the rest are early warnings on this one
      fall = 1. if trip in (Inhibit.leadChanged, Inhibit.noLead) else AUTHORITY_FALL * self.dt
      self.authority = max(0., self.authority - fall)
      self.hold_left = self.hold_frames
      self.inhibit = trip
    elif self.hold_left > 0:
      self.hold_left -= 1
      self.inhibit = Inhibit.hold
    else:
      self.authority = min(1., self.authority + AUTHORITY_RISE * self.dt)
      self.inhibit = Inhibit.none

    personality = int(getattr(sm['selfdriveState'].personality, 'raw', sm['selfdriveState'].personality))
    if lead.present:
      leads = [lead] + ([rs.leadTwo] if rs.leadTwo.present else [])
      self.gap_excess = min(ld.dRel - desired_gap(v_ego, ld.vLead, personality) for ld in leads)
    else:
      self.gap_excess = 0.
    self.weight = float(np.interp(self.gap_excess, GAP_BP, [0., 1.]) * np.interp(v_ego, SPEED_BP, [0., 1.]))
    self.gain = float(np.interp(a_model, GAIN_BP, [0., 1.]))

    headroom = max(0., min(a_mpc, a_cruise) - a_model)
    target = self.authority * self.gain * self.weight * min(CAP[personality], headroom)
    # falls at once (a binding MPC or a trip), rises at BOOST_RISE
    self.boost = min(target, self.boost + BOOST_RISE * self.dt, headroom)
    return a_model + self.boost

  @staticmethod
  def _trip(sm: messaging.SubMaster, a_model: float, v_ego: float, plan_drop: float, plan_min_v: float,
            lat_accel: float, allow_throttle: bool, fcw: bool):
    CS, md = sm['carState'], sm['modelV2']
    if fcw:
      return Inhibit.fcw
    if md.meta.hardBrakePredicted:
      return Inhibit.hardBrake
    if sm['controlsState'].forceDecel:
      return Inhibit.forceDecel
    if md.action.shouldStop or plan_min_v < STOP_SPEED:
      return Inhibit.stop
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
    if v_ego < nudge.MIN_SPEED:
      return Inhibit.lowSpeed
    return None
