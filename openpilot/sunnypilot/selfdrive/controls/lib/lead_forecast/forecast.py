"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The driving model's lead forecast as the long MPC's obstacle.

Upstream's MPC sees a lead as (dRel, vLead, aLeadK) now and extrapolates it, fading the
acceleration as exp(-tau t^2 / 2). A vision lead gets tau 0.3: a braking lead is assumed to stop
braking within ~3 s whatever it is doing. The model already predicts the lead's speed over 10 s
(leadsV3, knots at 0/2/4/6/8/10 s); comma's openpilot#37824 tried feeding that to the MPC.

Here the forecast replaces the extrapolation for camera leads only, shaped from what our
alpha-long logs show (35k samples, lead tracked 6 s, docs/zoompilot/lead-forecast.md):

- Speed is the model's predicted change, anchored on radarState's vLead. Behind a slowing lead
  the 4 s speed error halves (+2.65 -> +1.35 m/s), and it improves when the lead brakes hard.
- Distance is that speed integrated from dRel, not the model's own x: the model's x reads the
  lead 2 m further than it turns out at 6 s, the integral is unbiased and tighter.
- A slow lead may not speed up in the forecast. Under 3 m/s, a predicted pull-away was wrong a
  quarter of the time, and the plan must not aim through a stopped car that stays put. The
  allowance fades in to 8 m/s; above that predicted speed-ups held 85% of the time.
- A radar lead takes the forecast only with LeadForecastRadar on, and only while the model's lead
  is the radar track: radard's own pairing is loose (25% of range, 10 m/s), so the forecast asks
  for the model's lead within RADAR_MATCH_* of the track, and drops the last shape when the track
  changes. Radar measures the lead's acceleration now, which the model's 2 s knot spacing cannot,
  so the first RADAR_HANDOVER_T hands over from upstream's measured extrapolation, and while radar
  measures the lead braking the forecast is never faster than that extrapolation. A measured
  pull-away lifts the low-speed limit. On 299k radar-lead samples (same track the next 6 s): 4 s
  position error MAE 1.49 vs upstream 1.62 m, optimistic p90 2.15 vs 2.69; behind slowing leads
  3.93 vs 5.19 m; brake taps as upstream. Any lead the forecast does not match keeps upstream's
  extrapolation.

Switching between the two blends over a few frames so the obstacle never steps. The hook wraps the
MPC instance's process_lead; long_mpc.py stays upstream's.
"""
import math

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS, MIN_X_LEAD_FACTOR
from openpilot.selfdrive.controls.radard import RADAR_TO_CAMERA
from openpilot.selfdrive.modeld.constants import ModelConstants
from opendbc.car.interfaces import ACCEL_MIN
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD

Inhibit = custom.LongitudinalPlanZP.LeadForecast.Inhibit

PARAM = "LeadForecast"
PARAM_RADAR = "LeadForecastRadar"

LEAD_T_IDXS = np.asarray(ModelConstants.LEAD_T_IDXS)
# integrate the forecast speed on a fine grid so x is exact for the piecewise-linear v
FINE_T = np.linspace(0., LEAD_T_IDXS[-1], 201)

# Below this the forecast may not speed the lead up; above the second it may fully.
SPEEDUP_V_BP = [3., 8.]  # m/s
# Nor while the lead is braking now. Rare (1.2% of braking-and-closing frames forecast a speed-up)
# but the costly direction: under -1 m/s^2 a quarter of those leads kept slowing.
SPEEDUP_A_BP = [-1.0, -0.3]  # m/s^2
# radarState's vision lead is leadsV3 x[0] less RADAR_TO_CAMERA from the same or the previous
# model frame. Beyond this the radarState lead is some other object.
MATCH_DIST = 4.0  # m
# Blend weight rate; a full switch takes 0.25 s.
BLEND_RATE = 4.0  # 1/s

# Radar leads. The model's lead must sit on the radar track: range within the larger of the floor
# and a share of range, relative speed within RADAR_MATCH_SPEED.
RADAR_MATCH_DIST = 5.0  # m
RADAR_MATCH_DIST_REL = 0.15
RADAR_MATCH_SPEED = 2.0  # m/s
# radard's track keeps modelProb above this while the model sees it; its low-speed override (a
# close stationary track no model lead confirmed) reports 0
RADAR_MIN_MODEL_PROB = 0.5
# Over the first RADAR_HANDOVER_T s the speed hands over linearly from upstream's extrapolation of
# the measured aLeadK to the forecast.
RADAR_HANDOVER_T = 3.0  # s
# While radar measures the lead braking the forecast may not be faster than upstream's
# extrapolation: the floor fades in from the first to the second aLeadK (the model's 4 s speed was
# +0.7 m/s optimistic behind leads braking under -1, upstream -0.8).
RADAR_BRAKING_A_BP = [-0.1, -0.5]  # m/s^2
# A slow lead radar measures pulling away may speed up in the forecast (the low-speed limit is for
# a stopped car that may stay put); lifted from the first to the second aLeadK.
RADAR_LAUNCH_A_BP = [0.1, 0.5]  # m/s^2


def forecast_speed(v_lead: float, model_v, a_lead: float = 0., radar: bool = False) -> np.ndarray:
  """Lead speed at LEAD_T_IDXS: the model's predicted change from now, anchored on vLead. A radar
  lead's measured acceleration may lift the low-speed limit."""
  dv = np.asarray(model_v, dtype=float) - model_v[0]
  low_speed = float(np.interp(v_lead, SPEEDUP_V_BP, [0., 1.]))
  if radar:
    low_speed = max(low_speed, float(np.interp(a_lead, RADAR_LAUNCH_A_BP, [0., 1.])))
  allowance = low_speed * float(np.interp(a_lead, SPEEDUP_A_BP, [0., 1.]))
  dv = np.where(dv > 0., allowance * dv, dv)
  return np.maximum(v_lead + dv, 0.)


def _integrate(x_lead: float, v_fine: np.ndarray) -> np.ndarray:
  return x_lead + np.concatenate(([0.], np.cumsum(np.diff(FINE_T) * (v_fine[1:] + v_fine[:-1]) / 2.)))


def forecast_trajectory(x_lead: float, v_knots: np.ndarray) -> np.ndarray:
  """(x, v) at the MPC's T_IDXS: v interpolated from the knots, x its integral from x_lead."""
  x_fine = _integrate(x_lead, np.interp(FINE_T, LEAD_T_IDXS, v_knots))
  return np.column_stack((np.interp(T_IDXS, FINE_T, x_fine), np.interp(T_IDXS, LEAD_T_IDXS, v_knots)))


def radar_speed(v_knots: np.ndarray, v_lead: float, a_lead: float, a_lead_tau: float) -> np.ndarray:
  """Lead speed on FINE_T for a radar lead: upstream's extrapolation of the measured aLeadK (its
  clip and decay) handing over linearly to the forecast by RADAR_HANDOVER_T, and no faster than
  that extrapolation while radar measures the lead braking."""
  a = np.clip(a_lead, -10., 5.) * np.exp(-a_lead_tau * FINE_T ** 2 / 2.)
  v_measured = np.maximum(v_lead + np.concatenate(([0.], np.cumsum(np.diff(FINE_T) * (a[1:] + a[:-1]) / 2.))), 0.)
  w = np.interp(FINE_T, [0., RADAR_HANDOVER_T], [1., 0.]) if RADAR_HANDOVER_T > 0. else np.zeros_like(FINE_T)
  v = w * v_measured + (1. - w) * np.interp(FINE_T, LEAD_T_IDXS, v_knots)
  floor = float(np.interp(a_lead, RADAR_BRAKING_A_BP[::-1], [1., 0.]))
  return np.maximum(v - floor * np.maximum(v - v_measured, 0.), 0.)


def radar_trajectory(x_lead: float, v_fine: np.ndarray) -> np.ndarray:
  """(x, v) at the MPC's T_IDXS from a FINE_T speed."""
  return np.column_stack((np.interp(T_IDXS, FINE_T, _integrate(x_lead, v_fine)), np.interp(T_IDXS, FINE_T, v_fine)))


def radar_matches(lead, model_lead, v_ego: float, model_v_ego: float) -> bool:
  """The model's lead is this radar track: range and relative speed agree."""
  if lead.modelProb < RADAR_MIN_MODEL_PROB:
    return False
  if abs(model_lead.x[0] - RADAR_TO_CAMERA - lead.dRel) > max(RADAR_MATCH_DIST, RADAR_MATCH_DIST_REL * lead.dRel):
    return False
  return abs((model_lead.v[0] - model_v_ego) - (lead.vLead - v_ego)) <= RADAR_MATCH_SPEED


class LeadForecast:
  def __init__(self, params=None, dt: float = DT_MDL):
    self.params = params or Params()
    self.dt = dt
    self.params_frames = int(PARAMS_UPDATE_PERIOD / dt)
    self.frame = -1
    self.enabled = False
    self.radar_enabled = False
    self.weights = [0., 0.]
    self.inhibits = [Inhibit.disabled, Inhibit.disabled]
    self.v_knots: list[np.ndarray | None] = [None, None]
    self.lead_xv: list[np.ndarray | None] = [None, None]  # what the MPC got, for the plan report
    self.track_ids: list[int | None] = [None, None]  # the radar track each lead last was
    self._calls = 0

  def install(self, mpc) -> None:
    """Route the MPC's lead extrapolation through here. upstream update() calls process_lead
    once for leadOne then once for leadTwo; update() below resets the count every frame."""
    self.mpc = mpc
    self._upstream_process_lead = mpc.process_lead
    mpc.process_lead = self.process_lead

  def _target(self, lead, model_lead, v_ego: float, model_v_ego: float):
    if not self.enabled:
      return Inhibit.disabled, None
    if not lead.present:
      return Inhibit.noLead, None
    if lead.radar and not self.radar_enabled:
      return Inhibit.radar, None
    if len(model_lead.x) != len(LEAD_T_IDXS) or len(model_lead.v) != len(LEAD_T_IDXS):
      return Inhibit.invalid, None
    if not (np.isfinite(model_lead.x).all() and np.isfinite(model_lead.v).all() and
            math.isfinite(lead.dRel) and math.isfinite(lead.vLead) and math.isfinite(lead.aLeadK)):
      return Inhibit.invalid, None
    if lead.radar:
      if not (math.isfinite(v_ego) and math.isfinite(model_v_ego) and math.isfinite(lead.aLeadTau)):
        return Inhibit.invalid, None
      if not radar_matches(lead, model_lead, v_ego, model_v_ego):
        return Inhibit.mismatch, None
    elif abs(model_lead.x[0] - RADAR_TO_CAMERA - lead.dRel) > MATCH_DIST:
      return Inhibit.mismatch, None
    return Inhibit.none, forecast_speed(lead.vLead, model_lead.v, lead.aLeadK, radar=bool(lead.radar))

  def update(self, sm: messaging.SubMaster) -> None:
    self.frame += 1
    if self.frame % self.params_frames == 0:
      self.enabled = self.params.get_bool(PARAM)
      self.radar_enabled = self.params.get_bool(PARAM_RADAR)
    self._calls = 0

    rs, md = sm['radarState'], sm['modelV2']
    leads_v3 = md.leadsV3
    try:
      v_ego = float(sm['carState'].vEgo)
    except KeyError:  # callers without carState (replays of camera leads) get no radar forecast
      v_ego = float('nan')
    model_v_ego = float(md.velocity.x[0]) if len(md.velocity.x) else v_ego
    for i, lead in enumerate((rs.leadOne, rs.leadTwo)):
      if i < len(leads_v3):
        inhibit, v_knots = self._target(lead, leads_v3[i], v_ego, model_v_ego)
      else:
        inhibit, v_knots = (Inhibit.invalid if self.enabled else Inhibit.disabled), None
      # a new radar track is another car: the last one's shape must not fade out on it
      if self.radar_enabled and lead.present and lead.radar:
        if self.track_ids[i] is not None and lead.radarTrackId != self.track_ids[i]:
          self.v_knots[i] = None
          if v_knots is None:
            self.weights[i] = 0.
        self.track_ids[i] = int(lead.radarTrackId)
      elif not lead.present:
        self.track_ids[i] = None
      self.inhibits[i] = inhibit
      if v_knots is not None:
        self.v_knots[i] = v_knots
        self.weights[i] = min(1., self.weights[i] + BLEND_RATE * self.dt)
      else:
        self.weights[i] = max(0., self.weights[i] - BLEND_RATE * self.dt)
        if self.weights[i] == 0.:
          self.v_knots[i] = None

  def fill(self, msg) -> None:
    """Report into LongitudinalPlanZP.leadForecast: what the MPC got for each lead."""
    for report, weight, inhibit, lead_xv in zip((msg.leadOne, msg.leadTwo), self.weights, self.inhibits, self.lead_xv, strict=True):
      report.weight = float(weight)
      report.inhibit = inhibit
      if lead_xv is not None:
        report.x = lead_xv[:, 0].tolist()
        report.v = lead_xv[:, 1].tolist()

  def process_lead(self, lead) -> np.ndarray:
    i = self._calls
    self._calls += 1
    lead_xv = self._upstream_process_lead(lead)
    if (i < 2 and lead is not None and lead.present and self.weights[i] > 0. and self.v_knots[i] is not None and
        math.isfinite(lead.dRel) and math.isfinite(lead.vLead)):
      v_knots = self.v_knots[i]
      if self.inhibits[i] != Inhibit.none:
        # fading out: the last forecast's shape, carried on this frame's lead
        v_knots = np.maximum(lead.vLead + v_knots - v_knots[0], 0.)
      # upstream's clip: never start closer than what the MPC can still brake for
      v_ego = self.mpc.x0[1]
      v_lead = float(v_knots[0])
      min_x_lead = MIN_X_LEAD_FACTOR * (v_ego + v_lead) * (v_ego - v_lead) / (-ACCEL_MIN * 2)
      x_lead = max(lead.dRel, min_x_lead)
      if self.radar_enabled and lead.radar:
        if not (math.isfinite(lead.aLeadK) and math.isfinite(lead.aLeadTau)):
          self.lead_xv[i] = lead_xv
          return lead_xv
        forecast_xv = radar_trajectory(x_lead, radar_speed(v_knots, lead.vLead, lead.aLeadK, lead.aLeadTau))
      else:
        forecast_xv = forecast_trajectory(x_lead, v_knots)
      w = self.weights[i]
      lead_xv = w * forecast_xv + (1. - w) * lead_xv
    if i < 2:
      self.lead_xv[i] = lead_xv
    return lead_xv
