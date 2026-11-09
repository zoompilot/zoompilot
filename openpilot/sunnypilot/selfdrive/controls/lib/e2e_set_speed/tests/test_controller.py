"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np
import pytest

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL as DT
from openpilot.sunnypilot.selfdrive.car.tests.fakes import FakeParams
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed import approach
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed import controller as c
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.controller import E2ESetSpeedController, Inhibit
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.tests.helpers import ENGAGED, T_IDXS, build_sm, plan_ramp
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.limits import op_long_cruise_jerk

V_EGO = 20.
STANDARD = log.LongitudinalPersonality.standard
A_CRUISE = 0.5  # the planner's cruise candidate
RISE = op_long_cruise_jerk(V_EGO)
SETTLE_FRAMES = round(2. / (op_long_cruise_jerk(40.) * DT)) + 10


def run(ctl, n, sm, a_model=0., a_cruise=A_CRUISE, **kwargs):
  args = {**ENGAGED, **kwargs}
  out = a_model
  for _ in range(n):
    out = ctl.update(sm, a_model, a_cruise, **args)
  return out


def new_controller(enabled=True):
  return E2ESetSpeedController(params=FakeParams(ExperimentalModeSetSpeed=enabled), dt=DT)


def settled(a_cruise=A_CRUISE, **kwargs):
  ctl = new_controller()
  run(ctl, SETTLE_FRAMES, build_sm(V_EGO), a_cruise=a_cruise, **kwargs)
  assert ctl.lift == pytest.approx(ctl.floor)
  return ctl


def _plan(f):
  return np.asarray([f(t) for t in T_IDXS])


def bend_ahead(lat):
  """A bend 5-8 s out that takes lat m/s^2 at today's speed."""
  return _plan(lambda t: lat / V_EGO if 5. <= t <= 8. else 0.)


def test_disabled_passes_model_through():
  ctl = new_controller(enabled=False)
  assert run(ctl, 200, build_sm(V_EGO), a_model=0.1) == 0.1
  assert ctl.inhibit == Inhibit.disabled


@pytest.mark.parametrize("kwargs", [{"is_e2e": False}, {"reset_state": True}])
def test_idle_resets_at_once(kwargs):
  ctl = settled()
  assert run(ctl, 1, build_sm(V_EGO), a_model=0.05, **kwargs) == 0.05
  assert (ctl.inhibit, ctl.lift) == (Inhibit.inactive, 0.)


def test_lifts_from_the_first_frame_at_the_rate_limit():
  ctl = new_controller()
  lifts = [run(ctl, 1, build_sm(V_EGO), a_cruise=1.5) for _ in range(SETTLE_FRAMES)]
  assert lifts[0] == pytest.approx(RISE * DT)
  assert max(np.diff([0.] + lifts)) <= RISE * DT + 1e-9
  assert lifts[-1] == pytest.approx(1.5)
  assert ctl.inhibit == Inhibit.none


def test_floor_is_the_cruise_candidate():
  # with the model holding, the e2e candidate meets chill mode's acceleration and goes no higher
  settled()
  assert run(new_controller(), SETTLE_FRAMES, build_sm(V_EGO), a_model=0.2) == pytest.approx(A_CRUISE)
  assert run(new_controller(), SETTLE_FRAMES, build_sm(V_EGO), a_model=0.7) == 0.7


def test_never_adds_throttle_over_the_set_speed():
  # the cruise candidate braking: nothing to lift toward
  assert run(new_controller(), SETTLE_FRAMES, build_sm(V_EGO), a_model=-0.05, a_cruise=-0.3) == -0.05


@pytest.mark.parametrize("v, gain", [((c.MIN_SPEED + c.FULL_SPEED) / 2, 0.5), (c.MIN_SPEED - 1., 0.)])
def test_lift_fades_in_above_walking_pace(v, gain):
  ctl = new_controller()
  run(ctl, 1, build_sm(v), a_cruise=1.5)
  assert ctl.gain == pytest.approx(gain)


def test_leaves_stop_and_go_braking_alone():
  # a gentle stop under walking pace: nothing pulls the model's braking toward the cruise candidate
  assert run(new_controller(), SETTLE_FRAMES, build_sm(c.MIN_SPEED - 1.), a_model=-0.15) == -0.15


def test_gain_fades_the_lift_as_the_model_slows():
  ctl = new_controller()
  floor = A_CRUISE
  a_model = sum(c.GAIN_BP) / 2
  run(ctl, SETTLE_FRAMES, build_sm(V_EGO), a_model=a_model)
  assert ctl.lift == pytest.approx(0.5 * (floor - a_model))
  run(ctl, SETTLE_FRAMES, build_sm(V_EGO), a_model=c.GAIN_BP[0])
  assert ctl.lift == 0.


def test_never_lowers_the_model():
  # over the set speed, braking or a bend ahead: the e2e candidate is the model's or above
  for sm, a_model, a_cruise in [(build_sm(V_EGO), 0.2, -0.5), (build_sm(V_EGO), -1.5, A_CRUISE),
                                (build_sm(V_EGO, yaw_rate=bend_ahead(2.)), 0., A_CRUISE)]:
    ctl = settled()
    for _ in range(200):
      assert run(ctl, 1, sm, a_model=a_model, a_cruise=a_cruise) >= a_model
    assert ctl.lift == 0.


def test_lets_go_of_the_models_braking_at_once():
  ctl = settled(a_cruise=1.5)
  # halfway into the fade, the lift drops to its new value in one frame instead of easing off
  a_model = sum(c.GAIN_BP) / 2
  run(ctl, 1, build_sm(V_EGO), a_model=a_model, a_cruise=1.5)
  assert ctl.lift == pytest.approx(0.5 * (ctl.floor - a_model))
  ctl = settled(a_cruise=1.5)
  assert run(ctl, 1, build_sm(V_EGO), a_model=-0.5, a_cruise=1.5) == -0.5
  assert ctl.lift == 0.


def test_far_plan_flicker_does_not_move_the_lift():
  # the plan's far end flickering on a straight road, up to the drop band
  ctl = settled()
  lifts = []
  for i in range(100):
    dip = 0.99 * -c.GAIN_BP[1] * c.HORIZON if (i // 5) % 2 else 0.
    lifts.append(run(ctl, 1, build_sm(V_EGO, plan_v=plan_ramp(V_EGO, dip))))
  assert np.ptp(lifts) == 0.
  assert ctl.plan_fade == 1.


@pytest.mark.parametrize("drop, plan_fade", [(0., 1.), (1., 1.), (2., 0.5), (3., 0.), (8., 0.)])
def test_plan_drop_fades_the_lift(drop, plan_fade):
  # a gentle coast (inside the gain's band) toward a stop or a light: the lift lets it coast
  ctl = new_controller()
  a_model = -0.05
  run(ctl, SETTLE_FRAMES, build_sm(V_EGO, plan_v=plan_ramp(V_EGO, drop)), a_model=a_model)
  assert ctl.plan_fade == pytest.approx(plan_fade)
  assert ctl.gain == pytest.approx(plan_fade)
  assert ctl.lift == pytest.approx(plan_fade * (A_CRUISE - a_model))


def test_coast_toward_a_stop_lets_go_of_the_lift():
  # lifted on an open road, then the plan starts ending at a stop: the lift goes, never below the model
  ctl = settled()
  drops = np.linspace(0., 6., 60)
  out = [run(ctl, 1, build_sm(V_EGO, plan_v=plan_ramp(V_EGO, d)), a_model=-0.05) for d in drops]
  assert min(out) >= -0.05
  assert all(o == -0.05 for o, d in zip(out, drops, strict=True) if d >= c.HORIZON * -c.GAIN_BP[0])
  assert ctl.lift == 0.


@pytest.mark.parametrize("lead, plan_fade", [
  ((60., 0.), 1.),     # steady lead
  ((60., -0.9), 1.),   # barely closing
  ((30., -3.), 0.),    # closing hard, 10 s to contact
  ((60., -3.), 0.75),  # closing hard, 20 s to contact
  ((150., -3.), 1.),   # 50 s to contact
  ((20., -2.), 0.5),   # closing gently, 10 s to contact
  ((60., 2.), 1.),     # pulling away
])
def test_closing_lead_fades_the_lift(lead, plan_fade):
  ctl = new_controller()
  run(ctl, SETTLE_FRAMES, build_sm(V_EGO, lead=lead))
  assert ctl.plan_fade == pytest.approx(plan_fade)
  assert ctl.lift == pytest.approx(plan_fade * A_CRUISE)


def test_closing_lead_fade_is_smooth():
  ctl = new_controller()
  fades = []
  for d in np.linspace(120., 20., 101):  # closing at 3 m/s from 40 s to 7 s out
    run(ctl, 1, build_sm(V_EGO, lead=(d, -3.)))
    fades.append(ctl.plan_fade)
  assert fades[0] == 1. and fades[-1] == 0.
  assert max(np.abs(np.diff(fades))) <= 0.05 + 1e-9  # 1 m of 100 per step


@pytest.mark.parametrize("lat, fade", [(0.4, 1.), (0.8, 0.5), (1.2, 0.)])
def test_a_bend_ahead_fades_the_lift(lat, fade):
  ctl = new_controller()
  run(ctl, SETTLE_FRAMES, build_sm(V_EGO, yaw_rate=bend_ahead(lat)))
  assert ctl.fade == pytest.approx(fade)
  assert ctl.lift == pytest.approx(fade * A_CRUISE)


def test_lane_change_drops_the_lift():
  ctl = settled()
  run(ctl, 1, build_sm(V_EGO, lane_change=True))
  assert ctl.inhibit == Inhibit.laneChange
  assert ctl.lift == pytest.approx(ctl.floor - c.LIFT_FALL * DT)


def test_invalid_model_passes_through():
  ctl = settled()
  sm = build_sm(V_EGO, plan_v=_plan(lambda t: float('nan') if t > 5. else V_EGO))
  assert run(ctl, 1, sm, a_model=0.05) == 0.05
  assert (ctl.inhibit, ctl.lift) == (Inhibit.invalid, 0.)

  md = messaging.new_message('modelV2')
  md.modelV2.velocity.x = [V_EGO] * 10
  md.modelV2.orientationRate.z = [0.] * 10
  sm = {**build_sm(V_EGO), 'modelV2': md.modelV2.as_reader()}
  assert run(ctl, 1, sm, a_model=0.05) == 0.05
  assert ctl.inhibit == Inhibit.invalid

  assert run(ctl, 1, build_sm(float('nan')), a_model=0.05) == 0.05
  assert ctl.inhibit == Inhibit.invalid


FAR = 40.  # m/s under the target: the root is past every ceiling


def test_approach_closes_small_gaps_gently():
  # the floor's pace, up to the speed's ceiling
  for v, ceiling in zip(approach.APPROACH_BP, approach.APPROACH_V, strict=True):
    assert approach.approach_accel(v, v + FAR, STANDARD) == pytest.approx(ceiling)
  assert approach.approach_accel(V_EGO, V_EGO + 1., STANDARD) < 0.42  # a CX-5 5>4 kickdown at highway speed


@pytest.mark.parametrize("v", [5., 15., 25.])
def test_approach_is_continuous_at_the_target(v):
  # no step where the car reaches the target: the pace falls smoothly to 0 and stays there
  step = 1e-3
  gaps = np.linspace(-1., 10., 11001)
  paces = np.array([approach.approach_accel(v - g, v, STANDARD) for g in gaps])
  assert all(np.diff(paces) >= 0.)
  assert max(np.diff(paces)) <= np.sqrt(2 * approach.APPROACH_JERK * step) + 1e-9  # the root's steepest step, off the target
  assert paces[gaps <= 0.].max() == 0.


@pytest.mark.parametrize("over", [0., 0.5, 5.])
def test_approach_asks_nothing_at_or_over_the_target(over):
  for personality in (log.LongitudinalPersonality.relaxed, STANDARD, log.LongitudinalPersonality.aggressive):
    for v in approach.APPROACH_BP:
      assert approach.approach_accel(v + over, v, personality) == 0.


def test_approach_follows_the_personality():
  # Standard is the ceiling; Relaxed and Aggressive sit a 0.15 step either side of it at a full
  # shortfall. Read from a message as the planner does.
  def personality(name):
    msg = messaging.new_message('selfdriveState')
    msg.selfdriveState.personality = name
    return msg.as_reader().selfdriveState.personality.raw
  relaxed_p, standard_p, aggressive_p = (personality(n) for n in ('relaxed', 'standard', 'aggressive'))
  for v in approach.APPROACH_BP:
    relaxed, standard, aggressive = (approach.approach_accel(v, v + FAR, p) for p in (relaxed_p, standard_p, aggressive_p))
    assert relaxed == pytest.approx(standard - 0.15) and aggressive == pytest.approx(standard + 0.15)
