"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math

import numpy as np
import pytest

import openpilot.cereal.messaging as messaging
from openpilot.common.realtime import DT_MDL as DT
from openpilot.selfdrive.controls.lib import longitudinal_planner as upstream
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed import controller as c
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.controller import E2ESetSpeedController, Inhibit
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.tests.helpers import ENGAGED, T_IDXS, MockParams, build_sm

V_EGO = 20.
V_CRUISE = 24.  # floor (24 - 20) / 8 = 0.5
HOLD_FRAMES = round(c.HOLD_TIME / DT)
SETTLE_FRAMES = round(c.FLOOR_MAX / (c.BOOST_RISE * DT)) + 10


def run(ctl, n, sm, a_model=0., v_cruise=V_CRUISE, **kwargs):
  args = {**ENGAGED, **kwargs}
  out = a_model
  for _ in range(n):
    out = ctl.update(sm, a_model, v_cruise, **args)
  return out


def new_controller(enabled=True):
  return E2ESetSpeedController(params=MockParams(enabled), dt=DT)


def settled(v_cruise=V_CRUISE):
  ctl = new_controller()
  run(ctl, SETTLE_FRAMES, build_sm(V_EGO), v_cruise=v_cruise)
  assert ctl.boost == pytest.approx(ctl.floor)
  return ctl


def _plan(f):
  return np.asarray([f(t) for t in T_IDXS])


def test_friction_circle_matches_upstream():
  assert c.A_TOTAL_MAX_BP == upstream._A_TOTAL_MAX_BP
  assert c.A_TOTAL_MAX_V == upstream._A_TOTAL_MAX_V


def test_disabled_passes_model_through():
  ctl = new_controller(enabled=False)
  assert run(ctl, 200, build_sm(V_EGO), a_model=0.1) == 0.1
  assert ctl.inhibit == Inhibit.disabled


@pytest.mark.parametrize("kwargs, reason", [
  ({"is_e2e": False}, Inhibit.inactive),
  ({"reset_state": True}, Inhibit.inactive),
  ({"dec_active": True}, Inhibit.decActive),
])
def test_idle_resets_at_once(kwargs, reason):
  ctl = settled()
  assert run(ctl, 1, build_sm(V_EGO), a_model=0.05, **kwargs) == 0.05
  assert (ctl.inhibit, ctl.authority, ctl.boost) == (reason, 0., 0.)


def test_boosts_from_the_first_frame_at_the_rise_limit():
  ctl = new_controller()
  boosts = [run(ctl, 1, build_sm(V_EGO), v_cruise=V_EGO + 10.) - 0. for _ in range(SETTLE_FRAMES)]
  assert boosts[0] == pytest.approx(c.BOOST_RISE * DT)
  assert max(np.diff([0.] + boosts)) <= c.BOOST_RISE * DT + 1e-9
  assert boosts[-1] == pytest.approx(c.FLOOR_MAX)
  assert ctl.inhibit == Inhibit.none


def test_floor_closes_the_gap_like_stock_cruise():
  ctl = settled()
  assert ctl.boost == pytest.approx((V_CRUISE - V_EGO) / c.TAU)


@pytest.mark.parametrize("v, floor", [
  ((c.MIN_SPEED + c.FULL_SPEED) / 2, c.FLOOR_MAX / 2),
  (c.MIN_SPEED - 1., 0.),
])
def test_floor_fades_in_above_walking_pace(v, floor):
  ctl = new_controller()
  run(ctl, 1, build_sm(v), v_cruise=v + 10.)
  assert ctl.floor == pytest.approx(floor)


def test_gain_fades_the_boost_as_the_model_slows():
  ctl = new_controller()
  floor = (V_CRUISE - V_EGO) / c.TAU
  a_model = -0.1
  gain = (a_model - c.GAIN_BP[0]) / (c.GAIN_BP[1] - c.GAIN_BP[0])
  run(ctl, SETTLE_FRAMES, build_sm(V_EGO), a_model=a_model)
  assert ctl.boost == pytest.approx(gain * (floor - a_model))
  run(ctl, SETTLE_FRAMES, build_sm(V_EGO), a_model=c.GAIN_BP[0])
  assert (ctl.boost, ctl.inhibit) == (0., Inhibit.modelBraking)


def test_never_adds_throttle_at_set_speed():
  ctl = new_controller()
  assert run(ctl, 5, build_sm(V_EGO), a_model=0., v_cruise=V_EGO) == 0.
  assert run(ctl, 5, build_sm(V_EGO), a_model=0.1, v_cruise=V_EGO - 1.) == pytest.approx(0.1)


def test_backs_off_at_the_fall_limit():
  ctl = settled()
  start = ctl.boost
  run(ctl, 1, build_sm(V_EGO), a_model=-0.5)
  assert ctl.boost == pytest.approx(start - c.BOOST_FALL * DT)


@pytest.mark.parametrize("sm_kwargs, update_kwargs, reason", [
  ({"lead": True}, {}, Inhibit.lead),
  ({"gas": True}, {}, Inhibit.driver),
  ({"brake": True}, {}, Inhibit.driver),
  ({"should_stop": True}, {}, Inhibit.stop),
  ({"plan_v": _plan(lambda t: max(V_EGO - 3. * t, 1.))}, {}, Inhibit.stop),
  ({"hard_brake": True}, {}, Inhibit.hardBrake),
  ({"force_decel": True}, {}, Inhibit.forceDecel),
  ({"lane_change": True}, {}, Inhibit.laneChange),
  ({}, {"fcw": True}, Inhibit.fcw),
])
def test_hazards_cut_the_boost_and_hold(sm_kwargs, update_kwargs, reason):
  ctl = settled()
  start = ctl.boost
  run(ctl, 1, build_sm(V_EGO, **sm_kwargs), **update_kwargs)
  assert (ctl.inhibit, ctl.authority) == (reason, 0.)
  assert ctl.boost == pytest.approx(start - c.HAZARD_FALL * DT)
  run(ctl, math.ceil(start / (c.HAZARD_FALL * DT)), build_sm(V_EGO, **sm_kwargs), **update_kwargs)
  assert ctl.boost == 0.
  # clear: nothing until the hold has run out
  assert run(ctl, HOLD_FRAMES, build_sm(V_EGO)) == 0.
  assert ctl.inhibit == Inhibit.hold
  assert run(ctl, 1, build_sm(V_EGO)) > 0.


@pytest.mark.parametrize("kwargs, hold", [
  ({"dec_active": True}, HOLD_FRAMES),
  ({"is_e2e": False}, 0),
])
def test_leaving_dec_holds_like_a_hazard(kwargs, hold):
  ctl = settled()
  run(ctl, 1, build_sm(V_EGO), **kwargs)
  assert run(ctl, hold, build_sm(V_EGO)) == 0.
  assert run(ctl, 1, build_sm(V_EGO)) > 0.


@pytest.mark.parametrize("mean_accel, bound", [
  (0., c.FLOOR_MAX),
  (-0.05, c.FLOOR_MAX / 2),
  (-0.1, 0.),
  (-0.2, 0.),
])
def test_plan_bound_reads_the_plans_average_slowdown(mean_accel, bound):
  assert c.plan_bound(_plan(lambda t: V_EGO + mean_accel * t)) == pytest.approx(bound)


def test_plan_bound_ignores_the_first_seconds_wander():
  # a dip that is gone by PLAN_T[0] is the model's wander, not a slowdown
  assert c.plan_bound(_plan(lambda t: V_EGO - (0.2 if t < c.PLAN_T[0] - 0.5 else 0.))) == pytest.approx(c.FLOOR_MAX)


@pytest.mark.parametrize("offset", [1.0, -0.6])
def test_plan_bound_is_relative_to_the_plan(offset):
  # the model reads vEgo a few percent off; a flat plan offset from it is not a slowdown
  ctl = settled()
  run(ctl, 1, build_sm(V_EGO, plan_v=V_EGO + offset))
  assert ctl.inhibit == Inhibit.none


def test_a_slowing_plan_shrinks_the_boost_without_a_hold():
  ctl = settled(v_cruise=V_EGO + 10.)
  slowing = build_sm(V_EGO, plan_v=_plan(lambda t: V_EGO - 0.05 * t))
  run(ctl, SETTLE_FRAMES, slowing, v_cruise=V_EGO + 10.)
  assert ctl.inhibit == Inhibit.planSlowing
  assert ctl.boost == pytest.approx(c.FLOOR_MAX / 2)
  # the plan clears: recovers through the filter, no hold
  run(ctl, 1, build_sm(V_EGO), v_cruise=V_EGO + 10.)
  assert ctl.boost > c.FLOOR_MAX / 2


def test_bound_drops_at_once_and_recovers_through_the_filter():
  ctl = settled(v_cruise=V_EGO + 10.)
  run(ctl, 1, build_sm(V_EGO, plan_v=_plan(lambda t: V_EGO - 0.1 * t)), v_cruise=V_EGO + 10.)
  assert ctl.bound == 0.
  run(ctl, round(c.BOUND_TAU / DT), build_sm(V_EGO), v_cruise=V_EGO + 10.)
  assert ctl.bound == pytest.approx(c.FLOOR_MAX * (1 - math.exp(-1.)), abs=0.02)


def test_curve_bound_limits_speed_into_a_bend():
  flat = np.full(len(T_IDXS), V_EGO)
  # a bend 6 s out tight enough for 1.2 m/s^2 at today's speed: no boost into it
  assert c.curve_bound(flat, _plan(lambda t: 1.2 / V_EGO if t >= 6. else 0.), 0.) < 0.
  # a gentle one far enough off leaves room
  assert c.curve_bound(flat, _plan(lambda t: 0.5 / V_EGO if t >= 8. else 0.), 0.) > 0.
  # turning harder than CURVE_LAT_ACCEL now: none
  assert c.curve_bound(flat, np.zeros(len(T_IDXS)), c.CURVE_LAT_ACCEL + 0.1) == 0.


def test_curve_ahead_reports_lateral():
  ctl = settled(v_cruise=V_EGO + 10.)
  run(ctl, 1, build_sm(V_EGO, yaw_rate=_plan(lambda t: 1.2 / V_EGO if t >= 6. else 0.)), v_cruise=V_EGO + 10.)
  assert ctl.inhibit == Inhibit.lateral
  assert ctl.boost == pytest.approx(c.FLOOR_MAX - c.BOOST_FALL * DT)


def test_friction_circle_caps_the_floor():
  ctl = new_controller()
  lat = 1.65
  run(ctl, 1, build_sm(V_EGO, curvature=lat / V_EGO ** 2), v_cruise=V_EGO + 10.)
  assert ctl.floor == pytest.approx(math.sqrt(1.7 ** 2 - lat ** 2))


@pytest.mark.parametrize("accel_coast", [0.2, -0.1])
def test_coast_caps_the_floor(accel_coast):
  ctl = new_controller()
  assert run(ctl, SETTLE_FRAMES, build_sm(V_EGO), v_cruise=V_EGO + 10., allow_throttle=False, accel_coast=accel_coast) == \
    pytest.approx(max(accel_coast, 0.))
  assert ctl.floor == pytest.approx(accel_coast)


def test_invalid_model_passes_through():
  ctl = settled()
  sm = build_sm(V_EGO, plan_v=_plan(lambda t: float('nan') if t > 5. else V_EGO))
  assert run(ctl, 1, sm, a_model=0.05) == 0.05
  assert (ctl.inhibit, ctl.authority) == (Inhibit.invalid, 0.)

  md = messaging.new_message('modelV2')
  md.modelV2.velocity.x = [V_EGO] * 10
  md.modelV2.orientationRate.z = [0.] * 10
  sm = {**build_sm(V_EGO), 'modelV2': md.modelV2.as_reader()}
  assert run(ctl, 1, sm, a_model=0.05) == 0.05
  assert ctl.inhibit == Inhibit.invalid
