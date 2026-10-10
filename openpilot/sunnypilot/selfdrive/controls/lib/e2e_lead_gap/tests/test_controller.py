"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL as DT
from openpilot.sunnypilot.selfdrive.car.tests.fakes import FakeParams
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_lead_gap import controller as c
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_lead_gap.controller import CAP, E2ELeadGapController, Inhibit
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_lead_gap.tests.helpers import ENGAGED, build_sm, desired_gap

V = 20.
STANDARD = log.LongitudinalPersonality.standard
FAR = desired_gap(V, V, STANDARD) + 30.  # well past the band
A_MPC = 1.0  # the MPC wants to close in
SETTLED = int((c.nudge.HOLD_TIME + 1. / c.AUTHORITY_RISE + CAP[int(STANDARD)] / c.BOOST_RISE) / DT) + 10


def run(sm, frames=SETTLED, a_model=0., a_mpc=A_MPC, enabled=True, ctl=None, **overrides):
  ctl = ctl or E2ELeadGapController(params=FakeParams(ExperimentalModeLeadGap=enabled), dt=DT)
  args = dict(ENGAGED, **overrides)
  out = None
  for _ in range(frames):
    out = ctl.update(sm, a_model, a_mpc, **args)
  return ctl, out


def test_off_is_the_model():
  ctl, out = run(build_sm(V, FAR), enabled=False)
  assert out == 0. and ctl.inhibit == Inhibit.disabled


def test_closes_a_long_gap_up_to_the_cap():
  ctl, out = run(build_sm(V, FAR))
  assert ctl.inhibit == Inhibit.none
  assert out == pytest.approx(CAP[int(STANDARD)])


@pytest.mark.parametrize("personality", list(log.LongitudinalPersonality.schema.enumerants.values()))
def test_cap_per_personality(personality):
  ctl, out = run(build_sm(V, desired_gap(V, V, personality) + 30., personality=personality))
  assert out == pytest.approx(CAP[personality])


def test_never_above_the_mpc():
  ctl, out = run(build_sm(V, FAR), a_mpc=0.1)
  assert out == pytest.approx(0.1)


def test_mpc_braking_is_left_alone():
  ctl, out = run(build_sm(V, FAR), a_model=-0.05, a_mpc=-0.3)
  assert out == -0.05


def test_inside_the_deadband_does_nothing():
  ctl, out = run(build_sm(V, desired_gap(V, V, STANDARD) + c.GAP_BP[0] - 0.5))
  assert ctl.inhibit == Inhibit.none and out == 0.


def test_lift_fades_as_the_model_pushes_back():
  _, full = run(build_sm(V, FAR), a_model=c.GAIN_BP[1])
  _, half = run(build_sm(V, FAR), a_model=np.mean(c.GAIN_BP))
  _, none = run(build_sm(V, FAR), a_model=c.GAIN_BP[0])
  assert full - c.GAIN_BP[1] > half - np.mean(c.GAIN_BP) > 0.
  assert none == c.GAIN_BP[0]


def test_rise_is_rate_limited():
  ctl = E2ELeadGapController(params=FakeParams(ExperimentalModeLeadGap=True), dt=DT)
  sm = build_sm(V, FAR)
  outs = [ctl.update(sm, 0., A_MPC, **ENGAGED) for _ in range(SETTLED)]
  assert np.max(np.diff(outs)) <= c.BOOST_RISE * DT + 1e-9


@pytest.mark.parametrize("sm_kwargs, args, reason", [
  ({'a_lead': -1.0}, {}, Inhibit.leadBraking),
  ({'lead_v': [V, V - 1, V - 2, V - 3, V - 4, V - 5]}, {}, Inhibit.leadBraking),
  ({'v_lead': 4.}, {}, Inhibit.leadSlow),
  ({'prob': 0.6}, {}, Inhibit.leadUncertain),
  ({'gas': True}, {}, Inhibit.driver),
  ({'brake': True}, {}, Inhibit.driver),
  ({'lane_change': True}, {}, Inhibit.laneChange),
  ({'curvature': 0.01}, {}, Inhibit.lateral),
  ({'should_stop': True}, {}, Inhibit.stop),
  ({'hard_brake': True}, {}, Inhibit.hardBrake),
  ({}, {'fcw': True}, Inhibit.fcw),
  ({}, {'allow_throttle': False}, Inhibit.coast),
  ({'force_decel': True}, {}, Inhibit.forceDecel),
  # a dip that recovers by 5 s still counts
  ({'plan_v': np.interp(c.T_IDXS, [0., 2.5, 5., 10.], [V, V - 1.5, V, V])}, {}, Inhibit.planSlowing),
  ({}, {'steer_lat_accel': 1.2}, Inhibit.lateral),
])
def test_trips_drop_the_lift(sm_kwargs, args, reason):
  ctl, _ = run(build_sm(V, FAR))
  _, out = run(build_sm(V, FAR, **sm_kwargs), frames=1, ctl=ctl, **args)
  assert ctl.inhibit == reason
  assert out < CAP[int(STANDARD)]
  _, out = run(build_sm(V, FAR, **sm_kwargs), frames=int(0.25 / DT), ctl=ctl, **args)
  assert out == 0.


def test_lead_braking_on_lead_two():
  ctl, out = run(build_sm(V, FAR, lead2=(FAR + 20., 4.)))
  assert ctl.inhibit == Inhibit.leadSlow and out == 0.


def test_closer_lead_two_sets_the_gap():
  ctl, out = run(build_sm(V, FAR, lead2=(desired_gap(V, V, STANDARD), V)))
  assert ctl.gap_excess == pytest.approx(0.) and out == 0.


def test_new_lead_waits_out_the_hold():
  ctl, _ = run(build_sm(V, FAR))
  _, out = run(build_sm(V, FAR - 15.), frames=1, ctl=ctl)
  assert ctl.inhibit == Inhibit.leadChanged
  _, out = run(build_sm(V, FAR - 15.), frames=int(c.nudge.HOLD_TIME / DT) - 1, ctl=ctl)
  assert ctl.inhibit == Inhibit.hold and out == 0.


def test_tracked_lead_is_not_a_jump():
  # a lead closing at 2 m/s moves 0.1 m a frame; the expected position follows it
  ctl = E2ELeadGapController(params=FakeParams(ExperimentalModeLeadGap=True), dt=DT)
  for i in range(SETTLED):
    ctl.update(build_sm(V, FAR + 40. - 2. * DT * i, v_lead=V - 2.), 0., A_MPC, **ENGAGED)
  assert ctl.inhibit == Inhibit.none


def test_no_lead():
  ctl, _ = run(build_sm(V, FAR))
  sm = build_sm(V, FAR)
  rs = sm['radarState'].as_builder()
  rs.leadOne.present = False
  sm['radarState'] = rs.as_reader()
  _, out = run(sm, frames=int(0.25 / DT), ctl=ctl)
  assert ctl.inhibit == Inhibit.noLead and out == 0.


@pytest.mark.parametrize("args, reason", [
  ({'is_e2e': False}, Inhibit.inactive),
  ({'reset_state': True}, Inhibit.inactive),
  ({'dec_active': True}, Inhibit.decActive),
])
def test_idle_states_reset(args, reason):
  ctl, _ = run(build_sm(V, FAR))
  _, out = run(build_sm(V, FAR), frames=1, ctl=ctl, **args)
  assert ctl.inhibit == reason and out == 0. and ctl.authority == 0.


def test_non_finite_is_the_model():
  ctl, out = run(build_sm(V, FAR), a_mpc=float('nan'))
  assert ctl.inhibit == Inhibit.invalid and out == 0.


def test_low_speed_fades_out():
  _, out = run(build_sm(c.SPEED_BP[0], desired_gap(c.SPEED_BP[0], c.SPEED_BP[0], STANDARD) + 30.))
  assert out == 0.


def test_model_braking_trips():
  ctl, _ = run(build_sm(V, FAR))
  _, out = run(build_sm(V, FAR), frames=int(0.25 / DT), ctl=ctl, a_model=c.MODEL_BRAKE_ACCEL - 0.1)
  assert ctl.inhibit == Inhibit.modelBraking and out == c.MODEL_BRAKE_ACCEL - 0.1


def test_low_speed_trips():
  _, out = run(build_sm(c.nudge.MIN_SPEED - 1., FAR))
  assert out == 0.


def test_lead_forecast_dip_trips():
  # slowing 1.5 m/s by 2 s and back to -1.0 by 3 s: the lowest point counts
  ctl, out = run(build_sm(V, FAR, lead_v=[V, V - 1.5, V - 0.5, V, V, V]))
  assert ctl.inhibit == Inhibit.leadBraking and out == 0.


def test_lead_two_forecast_slowing_trips():
  sm = build_sm(V, FAR, lead2=(FAR + 20., V))
  md = sm['modelV2'].as_builder()
  md.leadsV3[1].v = [V, V - 3, V - 6, V - 8, V - 9, V - 9]
  sm['modelV2'] = md.as_reader()
  ctl, out = run(sm)
  assert ctl.inhibit == Inhibit.leadBraking and out == 0.


def test_radar_track_swap_is_a_new_lead():
  def radar_sm(track_id):
    sm = build_sm(V, FAR)
    rs = sm['radarState'].as_builder()
    rs.leadOne.radar = True
    rs.leadOne.radarTrackId = track_id
    sm['radarState'] = rs.as_reader()
    return sm
  ctl, _ = run(radar_sm(7))
  assert ctl.inhibit == Inhibit.none
  run(radar_sm(8), frames=1, ctl=ctl)
  assert ctl.inhibit == Inhibit.leadChanged


def test_idles_while_cruise_binds():
  ctl, out = run(build_sm(V, FAR), a_cruise=0.)
  assert out == 0. and ctl.boost == 0.


def test_mpc_dropping_below_the_lift_cuts_it_at_once():
  ctl, out = run(build_sm(V, FAR))
  assert out == pytest.approx(CAP[int(STANDARD)])
  _, out = run(build_sm(V, FAR), frames=1, ctl=ctl, a_mpc=0.1)
  assert out == pytest.approx(0.1)


def test_rearms_after_a_lead_change():
  ctl, _ = run(build_sm(V, FAR))
  run(build_sm(V, FAR - 15.), frames=1, ctl=ctl)
  _, out = run(build_sm(V, FAR - 15.), frames=SETTLED, ctl=ctl)
  assert ctl.inhibit == Inhibit.none and ctl.authority == 1.


def test_personality_change_mid_lift():
  ctl, out = run(build_sm(V, FAR))
  relaxed = log.LongitudinalPersonality.relaxed
  _, out = run(build_sm(V, FAR, personality=relaxed), frames=1, ctl=ctl)
  assert out == pytest.approx(CAP[int(relaxed)])
