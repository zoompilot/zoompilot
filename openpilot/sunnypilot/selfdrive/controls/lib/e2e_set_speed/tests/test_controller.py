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
from openpilot.sunnypilot.selfdrive.car.tests.fakes import FakeParams
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed import controller as c
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.controller import E2ESetSpeedController, Inhibit
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.tests.helpers import ENGAGED, T_IDXS, build_sm, plan_pos

V_EGO = 20.
V_CRUISE = 23.  # floor (23 - 20) / 6 = 0.5
HOLD_FRAMES = round(c.HOLD_TIME / DT)
SETTLE_FRAMES = round(c.FLOOR_MAX / (c.BOOST_RATE * DT)) + 10


def run(ctl, n, sm, a_model=0., v_cruise=V_CRUISE, deliver=False, **kwargs):
  """deliver: the planner drives the e2e candidate, so the boost reaches the car."""
  args = {**ENGAGED, **kwargs}
  out = a_model
  for _ in range(n):
    out = ctl.update(sm, a_model, v_cruise, **args)
    if deliver:
      ctl.delivered(out, out)
  return out


def new_controller(enabled=True):
  return E2ESetSpeedController(params=FakeParams(ExperimentalModeSetSpeed=enabled), dt=DT)


def settled(v_cruise=V_CRUISE, **kwargs):
  ctl = new_controller()
  run(ctl, SETTLE_FRAMES, build_sm(V_EGO), v_cruise=v_cruise, **kwargs)
  assert ctl.boost == pytest.approx(ctl.floor)
  return ctl


def _plan(f):
  return np.asarray([f(t) for t in T_IDXS])


def bound(vel, yaw=None, v_target=V_CRUISE, added=0.):
  vel = np.asarray(vel, dtype=float)
  yaw = np.zeros(len(T_IDXS)) if yaw is None else yaw
  return c.envelope_bound(vel, yaw, plan_pos(vel), v_target, added)


STOPPING = _plan(lambda t: max(V_EGO - 2. * t, 0.))
BEND_AHEAD = _plan(lambda t: 1.5 / V_EGO if 5. <= t <= 8. else 0.)  # 1.5 m/s^2 at today's speed


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
  ctl = settled(deliver=True)
  assert run(ctl, 1, build_sm(V_EGO), a_model=0.05, **kwargs) == 0.05
  assert (ctl.inhibit, ctl.authority, ctl.boost, ctl.added) == (reason, 0., 0., 0.)


def test_boosts_from_the_first_frame_at_the_rate_limit():
  ctl = new_controller()
  boosts = [run(ctl, 1, build_sm(V_EGO), v_cruise=V_EGO + 10.) for _ in range(SETTLE_FRAMES)]
  assert boosts[0] == pytest.approx(c.BOOST_RATE * DT)
  assert max(np.diff([0.] + boosts)) <= c.BOOST_RATE * DT + 1e-9
  assert boosts[-1] == pytest.approx(c.FLOOR_MAX)
  assert ctl.inhibit == Inhibit.none


def test_floor_closes_the_gap():
  assert settled().boost == pytest.approx((V_CRUISE - V_EGO) / c.TAU)


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


@pytest.mark.parametrize("sm_kwargs, update_kwargs, reason", [
  ({"lead": True}, {}, Inhibit.lead),
  ({"gas": True}, {}, Inhibit.driver),
  ({"brake": True}, {}, Inhibit.driver),
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


def test_an_empty_road_does_not_limit_the_floor():
  assert bound(np.full(len(T_IDXS), V_EGO)) > c.FLOOR_MAX


def test_the_model_wobbling_does_not_limit_the_floor():
  # a 0.4 m/s dip and back, well under the target: not a slowdown
  assert bound(_plan(lambda t: V_EGO - 0.4 * math.sin(math.pi * min(t, 4.) / 4.))) > c.FLOOR_MAX


def test_a_stop_ahead_closes_the_envelope():
  assert bound(STOPPING) < 0.


def test_a_planned_slowdown_is_met_at_the_models_speed():
  # the plan slows 3 m/s over 6 s: the boost may not take the car past it
  slowing = _plan(lambda t: V_EGO - 0.5 * min(t, 6.))
  assert bound(slowing) <= min(float(np.min((slowing[c.SLOWING_MASK] - V_EGO) / T_IDXS[c.SLOWING_MASK])), 0.) + 1e-9


def test_a_bend_the_model_takes_slower_than_allowed_limits_the_boost():
  assert bound(np.full(len(T_IDXS), V_EGO), BEND_AHEAD) < c.FLOOR_MAX
  # a gentle one leaves the floor alone
  assert bound(np.full(len(T_IDXS), V_EGO), BEND_AHEAD * 0.4 / 1.5) > c.FLOOR_MAX


@pytest.mark.parametrize("offset", [1.0, -0.6])
def test_envelope_is_relative_to_the_plan(offset):
  # the model reads vEgo a few percent off; a flat plan offset from it is not a slowdown
  ctl = settled()
  run(ctl, 1, build_sm(V_EGO, plan_v=V_EGO + offset))
  assert ctl.inhibit == Inhibit.none


def test_envelope_drops_at_once_and_recovers_through_the_filter():
  ctl = settled(v_cruise=V_EGO + 10.)
  run(ctl, 1, build_sm(V_EGO, plan_v=STOPPING), v_cruise=V_EGO + 10.)
  closed = ctl.bound
  assert closed < 0.
  run(ctl, round(c.BOUND_TAU / DT), build_sm(V_EGO), v_cruise=V_EGO + 10.)
  assert closed < ctl.bound < bound(np.full(len(T_IDXS), V_EGO), v_target=V_EGO + 10.)


def test_added_speed_counts_what_reached_the_car():
  assert settled().added == 0.
  ctl = settled(deliver=True)
  assert ctl.added > 0.
  # the gap assist's lift on top of ours reached the car too
  ctl.update(build_sm(V_EGO), 0., V_CRUISE, **ENGAGED)
  shed = ctl.added
  ctl.delivered(ctl.boost + 0.3, ctl.boost + 0.3)
  assert ctl.added == pytest.approx(shed + (ctl.boost + 0.3) * DT)
  # a lower candidate holds the car: none of it reached the car, and what we added converges out
  ctl.update(build_sm(V_EGO), 0., V_CRUISE, **ENGAGED)
  shed = ctl.added
  ctl.delivered(-0.5, ctl.boost)
  assert ctl.added == pytest.approx(shed * (1. - DT / c.CONVERGE_T))


def test_following_a_lead_clears_the_speed_we_added():
  ctl = settled(deliver=True)
  run(ctl, 100, build_sm(V_EGO), deliver=True)
  before = ctl.added
  assert before > 0.5
  # behind a lead the MPC holds the car below the e2e candidate
  for _ in range(round(5 * c.CONVERGE_T / DT)):
    out = ctl.update(build_sm(V_EGO, lead=True), 0., V_CRUISE, **ENGAGED)
    ctl.delivered(-0.3, out)
  assert ctl.added < 0.01 * before


def test_added_speed_is_shed_like_the_model_sheds_it():
  ctl = settled(deliver=True)
  added = ctl.added
  run(ctl, 40, build_sm(V_EGO, lead=True), deliver=True)  # boost cut, nothing more added
  before = ctl.added
  run(ctl, 1, build_sm(V_EGO, lead=True), deliver=True)
  assert ctl.added == pytest.approx(before * (1. - c.SHED_RATE * DT))
  assert before < added + 1.


def test_gives_back_what_it_added_before_a_stop():
  ctl = settled(deliver=True)
  run(ctl, 100, build_sm(V_EGO), deliver=True)
  added = ctl.added
  assert added > 0.5
  boosts = []
  for _ in range(200):
    run(ctl, 1, build_sm(V_EGO, plan_v=STOPPING), deliver=True)
    boosts.append(ctl.boost)
  boosts = np.asarray(boosts)
  # under the model, through the rate limit, never past what it put on
  assert boosts.min() < 0.
  assert np.abs(np.diff(boosts)).max() <= c.BOOST_RATE * DT + 1e-9
  assert boosts.min() >= -c.GIVE_MAX - 1e-9
  assert -boosts.sum() * DT <= added + 1e-6


def test_gives_nothing_back_when_it_added_nothing():
  ctl = settled()
  run(ctl, 200, build_sm(V_EGO, plan_v=STOPPING))
  assert ctl.boost == 0.


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
  md.modelV2.position.x = [0.] * 10
  sm = {**build_sm(V_EGO), 'modelV2': md.modelV2.as_reader()}
  assert run(ctl, 1, sm, a_model=0.05) == 0.05
  assert ctl.inhibit == Inhibit.invalid


def test_no_give_back_behind_a_lead():
  ctl = settled(deliver=True)
  run(ctl, 100, build_sm(V_EGO), deliver=True)
  run(ctl, 200, build_sm(V_EGO, plan_v=STOPPING, lead=True), deliver=True)
  assert ctl.boost == 0.


def test_a_lead_lets_a_give_back_go_at_the_rate_limit():
  ctl = settled(deliver=True)
  run(ctl, 100, build_sm(V_EGO), deliver=True)
  run(ctl, 60, build_sm(V_EGO, plan_v=STOPPING), deliver=True)
  giving = ctl.boost
  assert giving < -0.1
  boosts = [giving]
  for _ in range(100):
    run(ctl, 1, build_sm(V_EGO, plan_v=STOPPING, lead=True), deliver=True)
    boosts.append(ctl.boost)
  assert np.abs(np.diff(boosts)).max() <= c.BOOST_RATE * DT + 1e-9
  assert boosts[-1] == 0.
