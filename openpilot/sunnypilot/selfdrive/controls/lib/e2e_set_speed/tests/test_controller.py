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
RISE_FRAMES = round(1. / (c.AUTHORITY_RISE * DT))
FALL_FRAMES = math.ceil(1. / (c.AUTHORITY_FALL * DT))


def run(ctl, n, sm, a_model=0., v_cruise=V_CRUISE, **kwargs):
  args = {**ENGAGED, **kwargs}
  out = a_model
  for _ in range(n):
    out = ctl.update(sm, a_model, v_cruise, **args)
  return out


def new_controller(enabled=True):
  return E2ESetSpeedController(params=MockParams(enabled), dt=DT)


def armed():
  ctl = new_controller()
  run(ctl, HOLD_FRAMES + RISE_FRAMES + 40, build_sm(V_EGO))
  assert ctl.authority == pytest.approx(1.)
  return ctl


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
  ctl = armed()
  assert run(ctl, 1, build_sm(V_EGO), a_model=0.05, **kwargs) == 0.05
  assert (ctl.inhibit, ctl.authority, ctl.boost) == (reason, 0., 0.)


def test_holds_then_rises():
  ctl = new_controller()
  sm = build_sm(V_EGO)
  assert run(ctl, HOLD_FRAMES, sm) == 0.
  assert ctl.inhibit == Inhibit.hold
  run(ctl, RISE_FRAMES // 2, sm)
  assert ctl.inhibit == Inhibit.none
  assert ctl.authority == pytest.approx(0.5)
  run(ctl, RISE_FRAMES // 2, sm)
  assert ctl.authority == pytest.approx(1.)


def test_floor_closes_the_gap_like_stock_cruise():
  ctl = armed()
  assert run(ctl, 1, build_sm(V_EGO)) == pytest.approx((V_CRUISE - V_EGO) / c.TAU)
  assert run(ctl, 40, build_sm(V_EGO), v_cruise=V_EGO + 10.) == pytest.approx(c.FLOOR_MAX)


def test_floor_fades_in_above_walking_pace():
  ctl = armed()
  v = (c.MIN_SPEED + c.FULL_SPEED) / 2
  run(ctl, 1, build_sm(v), v_cruise=v + 10.)
  assert ctl.floor == pytest.approx(c.FLOOR_MAX / 2)


def test_gain_fades_the_boost_as_the_model_slows():
  ctl = armed()
  floor = (V_CRUISE - V_EGO) / c.TAU
  a_model = -0.1
  gain = (a_model - c.GAIN_BP[0]) / (c.GAIN_BP[1] - c.GAIN_BP[0])
  assert run(ctl, 1, build_sm(V_EGO), a_model=a_model) == pytest.approx(a_model + gain * (floor - a_model))
  # zero gain where the hard trip fires, so the trip never steps the output
  assert run(ctl, 1, build_sm(V_EGO), a_model=c.MODEL_BRAKE_ACCEL) == pytest.approx(c.MODEL_BRAKE_ACCEL)


def test_never_adds_throttle_at_set_speed():
  ctl = armed()
  assert run(ctl, 5, build_sm(V_EGO), a_model=0., v_cruise=V_EGO) == 0.
  assert run(ctl, 5, build_sm(V_EGO), a_model=0.1, v_cruise=V_EGO - 1.) == pytest.approx(0.1)


def test_boost_rise_is_limited():
  ctl = armed()
  run(ctl, 5, build_sm(V_EGO), v_cruise=V_EGO)
  assert ctl.boost == 0.
  boosts = []
  for _ in range(40):
    run(ctl, 1, build_sm(V_EGO), v_cruise=V_EGO + 10.)
    boosts.append(ctl.boost)
  assert max(np.diff([0.] + boosts)) <= c.BOOST_RISE * DT + 1e-9
  assert boosts[-1] == pytest.approx(c.FLOOR_MAX)


def _plan(f):
  return np.asarray([f(t) for t in T_IDXS])


@pytest.mark.parametrize("sm_kwargs, update_kwargs, a_model, reason", [
  ({"lead": True}, {}, 0., Inhibit.lead),
  ({"gas": True}, {}, 0., Inhibit.driver),
  ({"brake": True}, {}, 0., Inhibit.driver),
  ({"should_stop": True}, {}, 0., Inhibit.stop),
  ({"plan_v": _plan(lambda t: max(V_EGO - 3. * t, 1.))}, {}, 0., Inhibit.stop),
  ({"hard_brake": True}, {}, 0., Inhibit.hardBrake),
  ({"force_decel": True}, {}, 0., Inhibit.forceDecel),
  ({"lane_change": True}, {}, 0., Inhibit.laneChange),
  ({}, {"fcw": True}, 0., Inhibit.fcw),
  ({}, {"allow_throttle": False}, 0., Inhibit.coast),
  ({}, {}, -0.25, Inhibit.modelBraking),
  ({"plan_v": _plan(lambda t: V_EGO - 0.2 * t)}, {}, 0., Inhibit.planSlowing),
  ({"curvature": 1.2 / V_EGO ** 2}, {}, 0., Inhibit.lateral),
  # a bend 6 s out, nothing yet at the wheel
  ({"yaw_rate": _plan(lambda t: 1.2 / V_EGO if t >= 6. else 0.)}, {}, 0., Inhibit.lateral),
])
def test_trips_drop_authority_fast_and_hold(sm_kwargs, update_kwargs, a_model, reason):
  ctl = armed()
  sm = build_sm(V_EGO, **sm_kwargs)
  run(ctl, 1, sm, a_model=a_model, **update_kwargs)
  assert ctl.inhibit == reason
  assert ctl.authority == pytest.approx(1. - c.AUTHORITY_FALL * DT)
  assert run(ctl, FALL_FRAMES, sm, a_model=a_model, **update_kwargs) == a_model
  assert ctl.authority == 0.
  # clear: nothing until the hold has run out
  assert run(ctl, HOLD_FRAMES, build_sm(V_EGO)) == 0.
  run(ctl, 1, build_sm(V_EGO))
  assert ctl.authority > 0.


def test_low_speed_trips():
  ctl = armed()
  run(ctl, 1, build_sm(c.MIN_SPEED - 1.))
  assert ctl.inhibit == Inhibit.lowSpeed


def test_single_frame_trip_freezes_authority_through_the_hold():
  ctl = armed()
  run(ctl, 1, build_sm(V_EGO, lead=True))
  frozen = ctl.authority
  run(ctl, HOLD_FRAMES, build_sm(V_EGO))
  assert ctl.authority == frozen
  run(ctl, 1, build_sm(V_EGO))
  assert ctl.authority > frozen


@pytest.mark.parametrize("offset", [1.0, -0.6])
def test_plan_slowdown_is_relative_to_the_plan(offset):
  # the model reads vEgo a few percent off; a flat plan offset from it is not a slowdown
  ctl = armed()
  run(ctl, 1, build_sm(V_EGO, plan_v=V_EGO + offset))
  assert ctl.inhibit == Inhibit.none


def test_friction_circle_caps_the_floor():
  ctl = armed()
  lat = 1.65
  run(ctl, 1, build_sm(V_EGO, curvature=lat / V_EGO ** 2), v_cruise=V_EGO + 10.)
  assert ctl.inhibit == Inhibit.lateral
  assert ctl.floor == pytest.approx(math.sqrt(1.7 ** 2 - lat ** 2))


def test_coast_caps_the_floor():
  ctl = armed()
  run(ctl, 1, build_sm(V_EGO), v_cruise=V_EGO + 10., allow_throttle=False, accel_coast=0.2)
  assert ctl.floor == pytest.approx(0.2)


def test_invalid_model_passes_through():
  ctl = armed()
  sm = build_sm(V_EGO, plan_v=_plan(lambda t: float('nan') if t > 5. else V_EGO))
  assert run(ctl, 1, sm, a_model=0.05) == 0.05
  assert (ctl.inhibit, ctl.authority) == (Inhibit.invalid, 0.)

  md = messaging.new_message('modelV2')
  md.modelV2.velocity.x = [V_EGO] * 10
  md.modelV2.orientationRate.z = [0.] * 10
  sm = {**build_sm(V_EGO), 'modelV2': md.modelV2.as_reader()}
  assert run(ctl, 1, sm, a_model=0.05) == 0.05
  assert ctl.inhibit == Inhibit.invalid
