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
- A radar lead keeps upstream's extrapolation (it is measured, the model's lead may be another
  object), and so does any lead the model forecast does not match.

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


def forecast_speed(v_lead: float, model_v, a_lead: float = 0.) -> np.ndarray:
  """Lead speed at LEAD_T_IDXS: the model's predicted change from now, anchored on vLead."""
  dv = np.asarray(model_v, dtype=float) - model_v[0]
  allowance = float(np.interp(v_lead, SPEEDUP_V_BP, [0., 1.]) * np.interp(a_lead, SPEEDUP_A_BP, [0., 1.]))
  dv = np.where(dv > 0., allowance * dv, dv)
  return np.maximum(v_lead + dv, 0.)


def forecast_trajectory(x_lead: float, v_knots: np.ndarray) -> np.ndarray:
  """(x, v) at the MPC's T_IDXS: v interpolated from the knots, x its integral from x_lead."""
  v_fine = np.interp(FINE_T, LEAD_T_IDXS, v_knots)
  x_fine = x_lead + np.concatenate(([0.], np.cumsum(np.diff(FINE_T) * (v_fine[1:] + v_fine[:-1]) / 2.)))
  return np.column_stack((np.interp(T_IDXS, FINE_T, x_fine), np.interp(T_IDXS, LEAD_T_IDXS, v_knots)))


class LeadForecast:
  def __init__(self, params=None, dt: float = DT_MDL):
    self.params = params or Params()
    self.dt = dt
    self.params_frames = int(PARAMS_UPDATE_PERIOD / dt)
    self.frame = -1
    self.enabled = False
    self.weights = [0., 0.]
    self.inhibits = [Inhibit.disabled, Inhibit.disabled]
    self.v_knots: list[np.ndarray | None] = [None, None]
    self.lead_xv: list[np.ndarray | None] = [None, None]  # what the MPC got, for the plan report
    self._calls = 0

  def install(self, mpc) -> None:
    """Route the MPC's lead extrapolation through here. upstream update() calls process_lead
    once for leadOne then once for leadTwo; update() below resets the count every frame."""
    self.mpc = mpc
    self._upstream_process_lead = mpc.process_lead
    mpc.process_lead = self.process_lead

  def _target(self, lead, model_lead):
    if not self.enabled:
      return Inhibit.disabled, None
    if not lead.present:
      return Inhibit.noLead, None
    if lead.radar:
      return Inhibit.radar, None
    if len(model_lead.x) != len(LEAD_T_IDXS) or len(model_lead.v) != len(LEAD_T_IDXS):
      return Inhibit.invalid, None
    if not (np.isfinite(model_lead.x).all() and np.isfinite(model_lead.v).all() and
            math.isfinite(lead.dRel) and math.isfinite(lead.vLead) and math.isfinite(lead.aLeadK)):
      return Inhibit.invalid, None
    if abs(model_lead.x[0] - RADAR_TO_CAMERA - lead.dRel) > MATCH_DIST:
      return Inhibit.mismatch, None
    return Inhibit.none, forecast_speed(lead.vLead, model_lead.v, lead.aLeadK)

  def update(self, sm: messaging.SubMaster) -> None:
    self.frame += 1
    if self.frame % self.params_frames == 0:
      self.enabled = self.params.get_bool(PARAM)
    self._calls = 0

    rs, leads_v3 = sm['radarState'], sm['modelV2'].leadsV3
    for i, lead in enumerate((rs.leadOne, rs.leadTwo)):
      if i < len(leads_v3):
        inhibit, v_knots = self._target(lead, leads_v3[i])
      else:
        inhibit, v_knots = (Inhibit.invalid if self.enabled else Inhibit.disabled), None
      self.inhibits[i] = inhibit
      if v_knots is not None:
        self.v_knots[i] = v_knots
        self.weights[i] = min(1., self.weights[i] + BLEND_RATE * self.dt)
      else:
        self.weights[i] = max(0., self.weights[i] - BLEND_RATE * self.dt)
        if self.weights[i] == 0.:
          self.v_knots[i] = None

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
      forecast_xv = forecast_trajectory(max(lead.dRel, min_x_lead), v_knots)
      w = self.weights[i]
      lead_xv = w * forecast_xv + (1. - w) * lead_xv
    if i < 2:
      self.lead_xv[i] = lead_xv
    return lead_xv
