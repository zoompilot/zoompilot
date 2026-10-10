"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
import pytest

import openpilot.cereal.messaging as messaging
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, T_IDXS
from openpilot.selfdrive.controls.radard import RADAR_TO_CAMERA, _LEAD_ACCEL_TAU
from openpilot.sunnypilot.selfdrive.car.tests.fakes import FakeParams
from openpilot.sunnypilot.selfdrive.controls.lib.lead_forecast.forecast import (BLEND_RATE, LEAD_T_IDXS, Inhibit, LeadForecast,
                                                                                forecast_speed, forecast_trajectory)

V_EGO = 20.
FULL_BLEND_FRAMES = int(np.ceil(1. / (BLEND_RATE * 0.05)))


def build_sm(d_rel=40., v_lead=15., model_v=None, radar=False, present=True, d_rel2=60., model_v2=None, present2=True):
  """A vision radarState built from leadsV3 the way radard does: dRel = x[0] - RADAR_TO_CAMERA."""
  model_v = np.full(len(LEAD_T_IDXS), v_lead) if model_v is None else np.asarray(model_v, dtype=float)
  model_v2 = np.full(len(LEAD_T_IDXS), v_lead) if model_v2 is None else np.asarray(model_v2, dtype=float)
  md = messaging.new_message('modelV2')
  leads = md.modelV2.init('leadsV3', 3)
  for lead, x0, v in ((leads[0], d_rel, model_v), (leads[1], d_rel2, model_v2), (leads[2], d_rel2, model_v2)):
    lead.prob = 0.9
    lead.t = LEAD_T_IDXS.tolist()
    lead.v = v.tolist()
    lead.x = (x0 + RADAR_TO_CAMERA + np.concatenate(([0.], np.cumsum(np.diff(LEAD_T_IDXS) * (v[1:] + v[:-1]) / 2)))).tolist()

  rs = messaging.new_message('radarState')
  for lead, x0, v, p in ((rs.radarState.leadOne, d_rel, model_v, present), (rs.radarState.leadTwo, d_rel2, model_v2, present2)):
    lead.present = p
    lead.dRel = x0
    lead.vLead = float(v[0])
    lead.aLeadK = 0.
    lead.aLeadTau = 0.3
    lead.modelProb = 0.9
    lead.radar = radar
  return {'modelV2': md.modelV2.as_reader(), 'radarState': rs.radarState.as_reader()}


def build(enabled=True):
  mpc = LongitudinalMpc()
  mpc.set_cur_state(V_EGO, 0.)
  forecast = LeadForecast(params=FakeParams(LeadForecast=enabled))
  upstream = mpc.process_lead
  forecast.install(mpc)
  return mpc, forecast, upstream


def run(forecast, mpc, sm, frames=FULL_BLEND_FRAMES):
  for _ in range(frames):
    forecast.update(sm)
    mpc.update(sm['radarState'])
  return forecast.lead_xv[0]


SLOWING = [15., 11., 7., 4., 2., 1.]


class TestForecastShape:
  def test_speed_is_the_models_change_on_vlead(self):
    # the model sees the lead 1 m/s faster than radarState does; only its change counts
    v = forecast_speed(15., np.array(SLOWING) + 1.)
    np.testing.assert_allclose(v, SLOWING)

  def test_slow_lead_may_not_speed_up(self):
    v = forecast_speed(2., [2., 4., 6., 8., 10., 12.])
    np.testing.assert_allclose(v, 2.)

  def test_moving_lead_may_speed_up(self):
    v = forecast_speed(10., [10., 12., 14., 14., 14., 14.])
    np.testing.assert_allclose(v, [10., 12., 14., 14., 14., 14.])

  def test_speed_up_fades_in(self):
    v = forecast_speed(5.5, [5.5, 7.5, 7.5, 7.5, 7.5, 7.5])
    assert 5.5 < v[1] < 7.5

  def test_slowdown_is_never_limited(self):
    v = forecast_speed(2., [2., 0.5, 0., 0., 0., 0.])
    np.testing.assert_allclose(v, [2., 0.5, 0., 0., 0., 0.])

  def test_never_negative(self):
    assert (forecast_speed(3., [10., 0., -5., -5., -5., -5.]) >= 0.).all()

  def test_distance_is_the_integral_of_speed(self):
    xv = forecast_trajectory(30., np.full(len(LEAD_T_IDXS), 10.))
    np.testing.assert_allclose(xv[:, 0], 30. + 10. * T_IDXS, atol=1e-6)
    np.testing.assert_allclose(xv[:, 1], 10.)

  def test_distance_never_decreases(self):
    xv = forecast_trajectory(30., forecast_speed(15., SLOWING))
    assert (np.diff(xv[:, 0]) >= 0.).all()


class TestMpcHook:
  def test_disabled_is_upstream(self):
    mpc, forecast, upstream = build(enabled=False)
    sm = build_sm(model_v=SLOWING)
    lead_xv = run(forecast, mpc, sm)
    assert forecast.inhibits[0] == Inhibit.disabled
    np.testing.assert_array_equal(lead_xv, upstream(sm['radarState'].leadOne))

  def test_radar_lead_is_upstream(self):
    mpc, forecast, upstream = build()
    sm = build_sm(model_v=SLOWING, radar=True)
    lead_xv = run(forecast, mpc, sm)
    assert forecast.inhibits[0] == Inhibit.radar
    np.testing.assert_array_equal(lead_xv, upstream(sm['radarState'].leadOne))

  def test_other_object_is_upstream(self):
    mpc, forecast, upstream = build()
    sm = build_sm(model_v=SLOWING)
    rs = sm['radarState'].as_builder()
    rs.leadOne.dRel = 15.
    sm['radarState'] = rs.as_reader()
    lead_xv = run(forecast, mpc, sm)
    assert forecast.inhibits[0] == Inhibit.mismatch
    np.testing.assert_array_equal(lead_xv, upstream(sm['radarState'].leadOne))

  def test_slowing_lead_closes_in_sooner(self):
    mpc, forecast, upstream = build()
    sm = build_sm(model_v=SLOWING)
    lead_xv = run(forecast, mpc, sm)
    assert forecast.inhibits[0] == Inhibit.none and forecast.weights[0] == 1.
    np.testing.assert_allclose(lead_xv, forecast_trajectory(40., np.array(SLOWING)))
    old = upstream(sm['radarState'].leadOne)
    assert (lead_xv[1:, 0] < old[1:, 0]).all()

  def test_each_lead_gets_its_own_forecast(self):
    # upstream update() calls process_lead for leadOne, then leadTwo
    mpc, forecast, _ = build()
    lead2_v = [16., 16., 16., 16., 16., 16.]
    sm = build_sm(model_v=SLOWING, model_v2=lead2_v)
    run(forecast, mpc, sm)
    np.testing.assert_allclose(forecast.lead_xv[0], forecast_trajectory(40., np.array(SLOWING)))
    np.testing.assert_allclose(forecast.lead_xv[1], forecast_trajectory(60., np.array(lead2_v)))

  def test_blends_in_and_out(self):
    mpc, forecast, upstream = build()
    sm = build_sm(model_v=SLOWING)
    run(forecast, mpc, sm, frames=1)
    assert forecast.weights[0] == pytest.approx(BLEND_RATE * 0.05)
    run(forecast, mpc, sm)
    assert forecast.weights[0] == 1.
    rs = sm['radarState'].as_builder()
    rs.leadOne.radar = True
    radar_sm = {'modelV2': sm['modelV2'], 'radarState': rs.as_reader()}
    lead_xv = run(forecast, mpc, radar_sm, frames=1)
    w = forecast.weights[0]
    assert 0. < w < 1.
    old = upstream(radar_sm['radarState'].leadOne)
    np.testing.assert_allclose(lead_xv, w * forecast_trajectory(40., np.array(SLOWING)) + (1 - w) * old)
    run(forecast, mpc, radar_sm)
    assert forecast.weights[0] == 0.

  def test_lost_lead_is_upstreams_fake_lead(self):
    mpc, forecast, upstream = build()
    run(forecast, mpc, build_sm(model_v=SLOWING))
    sm = build_sm(model_v=SLOWING, present=False)
    forecast.update(sm)
    lead_xv = mpc.process_lead(sm['radarState'].leadOne)
    np.testing.assert_array_equal(lead_xv, upstream(sm['radarState'].leadOne))
    assert lead_xv[0, 0] == 50.  # upstream's synthetic fast lead

  def test_start_is_clipped_like_upstream(self):
    # a lead 2 m ahead and 15 m/s slower is beyond braking; upstream starts it further out
    mpc, forecast, upstream = build()
    sm = build_sm(d_rel=2., v_lead=5.)
    lead_xv = run(forecast, mpc, sm)
    assert lead_xv[0, 0] == pytest.approx(upstream(sm['radarState'].leadOne)[0, 0])
    assert lead_xv[0, 0] > 2.

  def test_upstream_tau_unchanged(self):
    # the comparison in docs/zoompilot/lead-forecast.md assumes upstream's vision tau and fallback tau
    assert _LEAD_ACCEL_TAU == 1.5


class TestGuards:
  def test_braking_lead_may_not_speed_up(self):
    v = forecast_speed(10., [10., 12., 14., 14., 14., 14.], a_lead=-1.5)
    np.testing.assert_allclose(v, 10.)

  def test_braking_lead_slowdown_kept(self):
    v = forecast_speed(10., [10., 8., 6., 6., 6., 6.], a_lead=-1.5)
    np.testing.assert_allclose(v, [10., 8., 6., 6., 6., 6.])

  def test_stopped_lead_with_a_predicted_launch_stays_put(self):
    mpc, forecast, _ = build()
    lead_xv = run(forecast, mpc, build_sm(d_rel=20., v_lead=0., model_v=[0., 2., 5., 8., 10., 12.]))
    assert forecast.inhibits[0] == Inhibit.none
    # the start may sit further out (upstream's min_x_lead clip), but the car never moves
    np.testing.assert_allclose(lead_xv[:, 0], lead_xv[0, 0])
    np.testing.assert_allclose(lead_xv[:, 1], 0.)

  def test_one_model_lead_is_invalid_for_lead_two(self):
    mpc, forecast, _ = build()
    sm = build_sm(model_v=SLOWING)
    first = sm['modelV2'].leadsV3[0]
    prob, x, v = first.prob, list(first.x), list(first.v)
    md = messaging.new_message('modelV2').modelV2
    leads = md.init('leadsV3', 1)
    leads[0].prob, leads[0].x, leads[0].v = prob, x, v
    sm['modelV2'] = md.as_reader()
    run(forecast, mpc, sm)
    assert forecast.inhibits[0] == Inhibit.none
    assert forecast.inhibits[1] == Inhibit.invalid and forecast.weights[1] == 0.

  def test_non_finite_model_is_invalid(self):
    mpc, forecast, upstream = build()
    sm = build_sm(model_v=[15., float('nan'), 7., 4., 2., 1.])
    lead_xv = run(forecast, mpc, sm)
    assert forecast.inhibits[0] == Inhibit.invalid
    np.testing.assert_array_equal(lead_xv, upstream(sm['radarState'].leadOne))

  def test_extra_mpc_update_gets_upstream(self):
    # a second solve without forecast.update() in between must not reuse the forecast
    mpc, forecast, upstream = build()
    sm = build_sm(model_v=SLOWING)
    run(forecast, mpc, sm)
    lead = sm['radarState'].leadOne
    forecast.process_lead(lead)
    forecast.process_lead(sm['radarState'].leadTwo)
    np.testing.assert_array_equal(forecast.process_lead(lead), upstream(lead))

  def test_fading_out_on_a_non_finite_lead_is_upstream(self):
    mpc, forecast, upstream = build()
    run(forecast, mpc, build_sm(model_v=SLOWING))
    sm = build_sm(model_v=SLOWING)
    rs = sm['radarState'].as_builder()
    rs.leadOne.vLead = float('nan')
    sm['radarState'] = rs.as_reader()
    forecast.update(sm)
    assert 0. < forecast.weights[0] < 1.
    lead_xv = forecast.process_lead(sm['radarState'].leadOne)
    np.testing.assert_array_equal(lead_xv, upstream(sm['radarState'].leadOne))

  def test_previous_frame_lead_still_matches(self):
    # radarState can lag modelV2 a frame: closing at 10 m/s moves the lead 0.5 m
    mpc, forecast, _ = build()
    sm = build_sm(model_v=SLOWING)
    rs = sm['radarState'].as_builder()
    rs.leadOne.dRel = 40.5
    sm['radarState'] = rs.as_reader()
    run(forecast, mpc, sm)
    assert forecast.inhibits[0] == Inhibit.none
