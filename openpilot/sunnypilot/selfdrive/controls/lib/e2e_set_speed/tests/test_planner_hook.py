"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The planner hook: off is upstream behaviour, on raises the e2e candidate, and the plan reports it.
"""
import numpy as np
import pytest

from openpilot.cereal import messaging
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner, LongitudinalPlanSource
from openpilot.sunnypilot.selfdrive.controls.lib.dec.tests.test_dec_planner_gate import build_planner, build_sm
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.controller import FLOOR_MAX, E2ESetSpeedController, Inhibit
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.tests.helpers import MockParams

MODEL_ACCEL = 0.


def no_lead_mpc(mpc) -> None:
  """The MPC with no lead chases its synthetic fast lead: the logged candidate sits near +2 m/s^2
  (p50 1.98), far above anything the floor asks for. Stub that rather than depend on the solver."""
  def update(radarstate, personality=None):
    mpc.v_solution = mpc.x0[1] + 2. * T_IDXS_MPC
    mpc.a_solution = np.full(len(T_IDXS_MPC), 2.)
    mpc.j_solution = np.zeros(len(T_IDXS_MPC) - 1)
    mpc.source = LongitudinalPlanSource.lead0
  mpc.update = update


def planner_sm():
  # DEC's gate fixture: experimental mode, 20 m/s under a 100 km/h set speed; a model holding its pace
  sm = build_sm(experimental_mode=True)
  model = sm['modelV2'].as_builder()
  model.action.desiredAcceleration = MODEL_ACCEL
  sm['modelV2'] = model.as_reader()
  return sm


def run_planner(enabled: bool, frames: int = 200) -> LongitudinalPlanner:
  planner = build_planner(dec_active=False, dec_mode='acc')
  no_lead_mpc(planner.mpc)
  planner.e2e_set_speed = E2ESetSpeedController(params=MockParams(enabled))
  sm = planner_sm()
  for _ in range(frames):
    planner.update(sm)
  return planner


def test_off_is_upstream():
  planner = run_planner(False)
  assert planner.mpc.source == LongitudinalPlanSource.e2e
  assert planner.output_a_target == pytest.approx(MODEL_ACCEL)
  assert planner.e2e_set_speed.inhibit == Inhibit.disabled


def test_on_raises_the_e2e_candidate():
  planner = run_planner(True)
  assert planner.e2e_set_speed.inhibit == Inhibit.none
  assert planner.mpc.source == LongitudinalPlanSource.e2e
  assert planner.output_a_target == pytest.approx(FLOOR_MAX)


def test_plan_reports_the_floor():
  planner = run_planner(True)
  pm = messaging.PubMaster(['longitudinalPlanSP'])
  sent = {}
  pm.send = lambda service, msg: sent.update({service: msg})
  planner.publish_longitudinal_plan_sp(planner_sm(), pm)
  report = sent['longitudinalPlanSP'].longitudinalPlanSP.zoompilot.e2eSetSpeed
  assert report.inhibit == Inhibit.none
  assert report.boost == pytest.approx(planner.e2e_set_speed.boost)
  assert report.authority == pytest.approx(1.)
