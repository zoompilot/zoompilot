"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The planner hooks: the forecast reaches the MPC through the planner and the plan reports it, and
the follow-distance assist reports from the same plan.
"""
import numpy as np
import pytest

from openpilot.cereal import messaging
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS
from openpilot.selfdrive.controls.radard import RADAR_TO_CAMERA
from openpilot.sunnypilot.selfdrive.controls.lib.dec.tests.test_dec_planner_gate import V_EGO, build_planner, build_sm
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_lead_gap.controller import Inhibit as GapInhibit
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.tests.helpers import MockParams
from openpilot.sunnypilot.selfdrive.controls.lib.lead_forecast.forecast import LEAD_T_IDXS, Inhibit, forecast_trajectory

SLOWING = np.array([V_EGO - 2., V_EGO - 6., V_EGO - 10., V_EGO - 12., V_EGO - 13., V_EGO - 13.])
D_REL = 40.


def lead_sm():
  sm = build_sm(experimental_mode=True)
  rs = messaging.new_message('radarState').radarState
  rs.leadOne.present = True
  rs.leadOne.dRel = D_REL
  rs.leadOne.vLead = float(SLOWING[0])
  rs.leadOne.vRel = float(SLOWING[0] - V_EGO)
  rs.leadOne.aLeadTau = 0.3
  rs.leadOne.modelProb = 0.95
  sm['radarState'] = rs.as_reader()
  md = messaging.new_message('modelV2').modelV2
  md.orientationRate.z = [0.01] * 33
  md.velocity.x = [V_EGO] * 33
  md.position.x = [float(i) for i in range(33)]
  md.action.desiredAcceleration = 0.
  leads = md.init('leadsV3', 3)
  for lead in leads:
    lead.prob = 0.95
    lead.t = LEAD_T_IDXS.tolist()
    lead.x = [D_REL + RADAR_TO_CAMERA] * len(LEAD_T_IDXS)
    lead.v = SLOWING.tolist()
  sm['modelV2'] = md.as_reader()
  return sm


def run(enabled: bool, frames: int = 20):
  planner = build_planner(dec_active=False, dec_mode='acc')
  planner.lead_forecast.params = MockParams(enabled)
  planner.lead_forecast.frame = -1  # re-read the param on the next update
  planner.e2e_lead_gap.params = MockParams(enabled)
  planner.e2e_lead_gap.frame = -1
  sm = lead_sm()
  for _ in range(frames):
    planner.update(sm)
  return planner, sm


def publish(planner, sm):
  pm = messaging.PubMaster(['longitudinalPlanSP'])
  sent = {}
  pm.send = lambda service, msg: sent.update({service: msg})
  planner.publish_longitudinal_plan_sp(sm, pm)
  return sent['longitudinalPlanSP'].longitudinalPlanSP.zoompilot


def test_off_reaches_the_mpc_as_upstream():
  planner, sm = run(False)
  report = publish(planner, sm)
  assert report.leadForecast.leadOne.inhibit == Inhibit.disabled
  assert report.leadForecast.leadOne.weight == 0.
  assert report.e2eLeadGap.inhibit == GapInhibit.disabled


def test_on_reaches_the_mpc():
  off, _ = run(False)
  planner, sm = run(True)
  lead = publish(planner, sm).leadForecast.leadOne
  assert lead.inhibit == Inhibit.none and lead.weight == pytest.approx(1.)
  assert len(lead.x) == len(T_IDXS)
  np.testing.assert_allclose(lead.v, forecast_trajectory(D_REL, SLOWING)[:, 1], rtol=1e-5)
  # a lead the model sees slowing hard: the plan brakes harder than upstream's extrapolation
  assert planner.mpc.a_solution[:6].min() < off.mpc.a_solution[:6].min() - 0.1


def test_lead_gap_reports_why_it_idles():
  planner, sm = run(True)
  # the lead's forecast slows by far more than LEAD_SLOWDOWN
  assert publish(planner, sm).e2eLeadGap.inhibit == GapInhibit.leadBraking
