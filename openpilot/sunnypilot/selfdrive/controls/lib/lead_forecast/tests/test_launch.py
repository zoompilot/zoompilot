"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

A lead pulling away from a stop: the launch gate, and the forecast it lifts on camera and radar leads.
"""
import numpy as np
import pytest

from openpilot.sunnypilot.selfdrive.controls.lib.lead_forecast import forecast as F
from openpilot.sunnypilot.selfdrive.controls.lib.lead_forecast.forecast import (BLEND_RATE, LAUNCH_A, LAUNCH_DROP_V, LAUNCH_RECENT_T,
                                                                                LAUNCH_STALL_T, LAUNCH_STOPPED_V, LAUNCH_T, LAUNCH_V,
                                                                                LEAD_T_IDXS, Inhibit, LaunchGate, forecast_speed)
from openpilot.sunnypilot.selfdrive.controls.lib.lead_forecast.tests.test_forecast import build, build_sm
from openpilot.sunnypilot.selfdrive.controls.lib.lead_forecast.tests.test_radar import radar_sm

DT = 0.05
FIRE_FRAMES = int(round(LAUNCH_T / DT))
# the model sees a stopped lead pulling away: 0 now, 3 m/s in 2 s, 6 in 4 s
PULL_AWAY = [0., 3., 6., 8., 9., 10.]


def feed(gate, samples):
  return [gate.update(v, a) for v, a in samples]


def launch(v0=0.3, a=1.0, frames=40):
  """A lead stopped for a second, then accelerating at a."""
  return [(0., 0.)] * 20 + [(v0 + a * DT * i, a) for i in range(frames)]


class TestLaunchGate:
  def test_stopped_lead_never_lifts(self):
    assert max(feed(LaunchGate(), [(0., 0.)] * 200)) == 0.

  def test_launch_lifts_after_it_persists(self):
    w = feed(LaunchGate(), launch())
    assert w[:20 + FIRE_FRAMES - 1] == [0.] * (20 + FIRE_FRAMES - 1)
    assert w[20 + FIRE_FRAMES - 1] == 1.

  @pytest.mark.parametrize("v, a", [(LAUNCH_V * 0.9, 1.0), (0.5, LAUNCH_A * 0.9)])
  def test_motion_under_either_threshold_does_not_lift(self, v, a):
    assert max(feed(LaunchGate(), [(0., 0.)] * 20 + [(v, a)] * 40)) == 0.

  def test_noise_shorter_than_the_persistence_does_not_lift(self):
    burst = [(0.4, 0.6)] * (FIRE_FRAMES - 1) + [(0., 0.)] * 3
    assert max(feed(LaunchGate(), [(0., 0.)] * 20 + burst * 20)) == 0.

  def test_lead_that_was_not_stopped_does_not_lift(self):
    # a slow car speeding up that has not been stopped within LAUNCH_RECENT_T
    gate = LaunchGate()
    assert max(feed(gate, [(LAUNCH_STOPPED_V + 0.5, 0.)] * int(LAUNCH_RECENT_T / DT + 5) + [(1.0, 1.0)] * 40)) == 0.

  def test_stall_fades_back(self):
    gate = LaunchGate()
    feed(gate, launch(frames=20))
    assert gate.weight == 1.
    # the lead stops accelerating at 1.5 m/s: the lift holds for LAUNCH_STALL_T, then fades
    w = feed(gate, [(1.5, 0.)] * 40)
    hold = int(round(LAUNCH_STALL_T / DT))
    assert w[:hold - 1] == [1.] * (hold - 1)
    falling = [x for x in w if 0. < x < 1.]
    assert falling and np.allclose(-np.diff([1.] + falling), BLEND_RATE * DT)
    assert w[-1] == 0.

  def test_drop_back_to_a_stop_fades_at_once(self):
    gate = LaunchGate()
    feed(gate, launch(frames=10))
    assert gate.weight == 1.
    w = feed(gate, [(0.05, -1.0)])
    assert w[0] == pytest.approx(1. - BLEND_RATE * DT)

  def test_speed_falling_from_its_peak_fades_back(self):
    # a creep that stops: the model's acceleration lags the measured speed coming back down
    gate = LaunchGate()
    feed(gate, launch(frames=20))
    peak = gate.peak_v
    assert gate.weight == 1.
    w = feed(gate, [(peak - LAUNCH_DROP_V + 0.05, 0.5), (peak - LAUNCH_DROP_V - 0.05, 0.5)])
    assert w == [1., pytest.approx(1. - BLEND_RATE * DT)]

  def test_relaunch_after_a_stall(self):
    gate = LaunchGate()
    feed(gate, launch(frames=10))
    feed(gate, [(0., -0.5)] * 40)
    assert gate.weight == 0.
    feed(gate, launch(frames=10))
    assert gate.weight == 1.


def run_frames(forecast, mpc, sms):
  for sm in sms:
    forecast.update(sm)
    mpc.update(sm['radarState'])
  return forecast.lead_xv[0]


def camera_launch_sm(v_lead, a_lead, d_rel=4.):
  sm = build_sm(d_rel=d_rel, v_lead=v_lead, model_v=np.array(PULL_AWAY) + v_lead)
  rs = sm['radarState'].as_builder()
  rs.leadOne.aLeadK = a_lead
  return {**sm, 'radarState': rs.as_reader()}


class TestCameraLaunch:
  def test_nothing_changes_before_motion(self):
    # a stopped lead the model sees pulling away keeps the low-speed limit
    mpc, forecast, _ = build()
    lead_xv = run_frames(forecast, mpc, [camera_launch_sm(0., 0.)] * 40)
    assert forecast.inhibits[0] == Inhibit.none
    np.testing.assert_allclose(lead_xv[:, 1], 0.)

  def test_measured_launch_takes_the_models_pull_away(self):
    mpc, forecast, _ = build()
    run_frames(forecast, mpc, [camera_launch_sm(0., 0.)] * 20)
    sms = [camera_launch_sm(0.3 + 0.05 * i, 1.0) for i in range(FIRE_FRAMES)]
    lead_xv = run_frames(forecast, mpc, sms)
    v_lead = sms[-1]['radarState'].leadOne.vLead
    np.testing.assert_allclose(forecast.v_knots[0], v_lead + np.array(PULL_AWAY))
    assert lead_xv[-1, 1] > 9.

  def test_one_frame_short_keeps_the_limit(self):
    mpc, forecast, _ = build()
    run_frames(forecast, mpc, [camera_launch_sm(0., 0.)] * 20)
    run_frames(forecast, mpc, [camera_launch_sm(0.3, 1.0)] * (FIRE_FRAMES - 1))
    np.testing.assert_allclose(forecast.v_knots[0], 0.3)

  def test_another_car_restarts_the_gate(self):
    mpc, forecast, _ = build()
    run_frames(forecast, mpc, [camera_launch_sm(0., 0.)] * 20)
    run_frames(forecast, mpc, [camera_launch_sm(0.3, 1.0)] * (FIRE_FRAMES - 1))
    # the range jumps 6 m in a frame: another car, which the gate has not seen stopped
    run_frames(forecast, mpc, [camera_launch_sm(0.6, 1.0, d_rel=10.)] * 1)
    assert forecast.launch_gates[0].weight == 0.


class TestRadarLaunch:
  def test_radar_launch_lifts(self):
    mpc, forecast, _ = build(radar=True)
    stopped = radar_sm(d_rel=4., v_lead=0., model_v=PULL_AWAY, a_lead=0.)
    run_frames(forecast, mpc, [stopped] * 20)
    np.testing.assert_allclose(forecast.v_knots[0], 0.)
    moving = radar_sm(d_rel=4., v_lead=0.4, model_v=np.array(PULL_AWAY) + 0.4, a_lead=LAUNCH_A)
    run_frames(forecast, mpc, [moving] * FIRE_FRAMES)
    np.testing.assert_allclose(forecast.v_knots[0], 0.4 + np.array(PULL_AWAY))

  def test_new_track_restarts_the_gate(self):
    mpc, forecast, _ = build(radar=True)
    run_frames(forecast, mpc, [radar_sm(d_rel=4., v_lead=0., model_v=PULL_AWAY, a_lead=0.)] * 20)
    moving = radar_sm(d_rel=4., v_lead=0.4, model_v=np.array(PULL_AWAY) + 0.4, a_lead=LAUNCH_A)
    run_frames(forecast, mpc, [moving] * (FIRE_FRAMES - 1))
    other = radar_sm(d_rel=4., v_lead=0.4, model_v=np.array(PULL_AWAY) + 0.4, a_lead=LAUNCH_A, track=8)
    run_frames(forecast, mpc, [other])
    assert forecast.launch_gates[0].weight == 0.


class TestForecastSpeed:
  def test_launch_weight_lifts_the_low_speed_limit(self):
    np.testing.assert_allclose(forecast_speed(0.5, PULL_AWAY, 0.5, launch=1.), 0.5 + np.array(PULL_AWAY))
    np.testing.assert_allclose(forecast_speed(0.5, PULL_AWAY, 0.5, launch=0.), 0.5)

  def test_braking_lead_guard_still_applies(self):
    np.testing.assert_allclose(forecast_speed(0.5, PULL_AWAY, -1.5, launch=1.), 0.5)

  def test_module_constants_are_ordered(self):
    assert 0. < F.LAUNCH_V < F.LAUNCH_STOPPED_V
    assert F.LAUNCH_STALL_A < F.LAUNCH_A
    assert len(LEAD_T_IDXS) == len(PULL_AWAY)
