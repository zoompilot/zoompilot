"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The planner hook: off is upstream behaviour, on raises the e2e candidate, and the plan reports it.
"""
import numpy as np
import pytest

from openpilot.cereal import messaging
from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner, LongitudinalPlanSource
from openpilot.sunnypilot.selfdrive.car.tests.fakes import FakeParams
from openpilot.sunnypilot.selfdrive.controls.lib.tests.test_experimental_gate import build_planner, build_sm
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.controller import E2ESetSpeedController, Inhibit, approach_accel

MODEL_ACCEL = 0.


def lead_mpc(mpc, accel: float) -> None:
  """The MPC holding a lead's gap at a steady accel, below the cruise candidate."""
  def update(radarstate, personality=None):
    mpc.v_solution = mpc.x0[1] + accel * T_IDXS_MPC
    mpc.a_solution = np.full(len(T_IDXS_MPC), accel)
    mpc.j_solution = np.zeros(len(T_IDXS_MPC) - 1)
    mpc.source = LongitudinalPlanSource.lead0
  mpc.update = update


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
  # the experimental gate's fixture: openpilot long, 20 m/s under a 100 km/h set speed; a model holding its pace
  sm = build_sm()
  model = sm['modelV2'].as_builder()
  model.action.desiredAcceleration = MODEL_ACCEL
  sm['modelV2'] = model.as_reader()
  return sm


def run_planner(enabled: bool, frames: int = 200) -> LongitudinalPlanner:
  planner = build_planner()
  no_lead_mpc(planner.mpc)
  planner.e2e_set_speed = E2ESetSpeedController(params=FakeParams(ExperimentalModeSetSpeed=enabled))
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
  # the model holding at 0 under the set speed: lifted toward it at a production ACC's pace, not
  # the cruise law's 2.0
  approach = approach_accel(planner.v_ego, planner.output_v_target)
  assert planner.e2e_set_speed.lift == pytest.approx(approach)
  assert planner.output_a_target == pytest.approx(approach)
  assert planner.a_cruise > 1. > approach


def test_behind_a_lead_follows_as_chill():
  # the model hanging back behind a lead the MPC would close on: lifted to the MPC, not past it
  planner = run_planner(True, frames=0)
  lead_mpc(planner.mpc, 0.3)
  sm = planner_sm()
  for _ in range(200):
    planner.update(sm)
  assert planner.e2e_set_speed.lift == pytest.approx(0.3)
  assert planner.output_a_target == pytest.approx(0.3)


def test_plan_reports_the_floor():
  planner = run_planner(True)
  pm = messaging.PubMaster(['longitudinalPlanSP'])
  sent = {}
  pm.send = lambda service, msg: sent.update({service: msg})
  planner.publish_longitudinal_plan_sp(planner_sm(), pm)
  report = sent['longitudinalPlanSP'].longitudinalPlanSP.zoompilot.e2eSetSpeed
  assert report.inhibit == Inhibit.none
  assert report.boost == pytest.approx(planner.e2e_set_speed.lift)
  assert (report.gain, report.fade) == (1., 1.)


def test_throttle_probability_reaches_the_floor():
  planner = run_planner(True, frames=0)
  sm = planner_sm()
  model = sm['modelV2'].as_builder()
  model.meta.disengagePredictions.gasPressProbs = [0.9, 0.25, 0.1]
  sm['modelV2'] = model.as_reader()
  for _ in range(200):
    planner.update(sm)
  assert planner.e2e_set_speed.gain == pytest.approx(0.25)
  assert planner.e2e_set_speed.lift == pytest.approx(0.25 * planner.e2e_set_speed.floor, rel=1e-3)


def gas_pressed_planner(enabled: bool) -> LongitudinalPlanner:
  """Model-limited with the gas held: upstream's boost latches eligibility, then learns."""
  planner = run_planner(enabled, frames=0)
  planner.accel_boost.total_boost = 0.1
  planner.accel_boost.boost_eligible = True
  sm = planner_sm()
  car_state = sm['carState'].as_builder()
  car_state.gasPressed = True
  sm['carState'] = car_state.as_reader()
  for _ in range(50):
    planner.update(sm)
  return planner


def test_gas_boost_only_without_the_floor():
  # the names the hook writes: a rename upstream must fail here, not silently boost again
  assert {'total_boost', 'boost_eligible'} <= set(vars(AccelBoost()))
  assert gas_pressed_planner(False).accel_boost.total_boost > 0.1
  on = gas_pressed_planner(True).accel_boost
  assert (on.total_boost, on.boost_eligible) == (0., False)
