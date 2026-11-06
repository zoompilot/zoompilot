"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Radar leads (LeadForecastRadar): matched to the model's lead, measured accel handed over to the
forecast, a new track never inherits the last one's shape.
"""
import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS
from openpilot.sunnypilot.selfdrive.controls.lib.lead_forecast.forecast import (FINE_T, LEAD_T_IDXS, RADAR_BRAKING_A_BP, RADAR_HANDOVER_T,
                                                                                RADAR_LAUNCH_A_BP, RADAR_MATCH_DIST,
                                                                                RADAR_MATCH_DIST_REL, RADAR_MATCH_SPEED, Inhibit,
                                                                                forecast_speed, radar_speed, radar_trajectory)
from openpilot.sunnypilot.selfdrive.controls.lib.lead_forecast.tests import test_forecast
from openpilot.sunnypilot.selfdrive.controls.lib.lead_forecast.tests.test_forecast import SLOWING, V_EGO, build_sm, run

D_REL = 40.


def radar_sm(d_rel=D_REL, v_lead=15., model_v=SLOWING, track=7, a_lead=0., a_tau=1.5, model_prob=0.9, **kw):
  """build_sm's leads reported as radar tracks, with the relative speed radard measured and the
  model's own ego speed."""
  sm = build_sm(d_rel=d_rel, v_lead=v_lead, model_v=model_v, **kw)
  rs = sm['radarState'].as_builder()
  for lead in (rs.leadOne, rs.leadTwo):
    lead.radar = True
    lead.vRel = lead.vLead - V_EGO
  rs.leadOne.radarTrackId = track
  rs.leadTwo.radarTrackId = track + 100
  rs.leadOne.aLeadK = a_lead
  rs.leadOne.aLeadTau = a_tau
  rs.leadOne.modelProb = model_prob
  sm['radarState'] = rs.as_reader()
  md = sm['modelV2'].as_builder()
  md.velocity.x = [V_EGO] * 33
  sm['modelV2'] = md.as_reader()
  return sm


def with_lead(sm, **fields):
  rs = sm['radarState'].as_builder()
  for k, v in fields.items():
    setattr(rs.leadOne, k, v)
  return {**sm, 'radarState': rs.as_reader()}


def build(radar=True):
  return test_forecast.build(radar=radar)


def expected(sm, lead=0):
  ld = (sm['radarState'].leadOne, sm['radarState'].leadTwo)[lead]
  v_knots = forecast_speed(ld.vLead, sm['modelV2'].leadsV3[lead].v, ld.aLeadK, radar=True)
  return radar_trajectory(ld.dRel, radar_speed(v_knots, ld.vLead, ld.aLeadK, ld.aLeadTau))


class TestShape:
  def test_starts_on_the_measured_accel(self):
    # radar sees the lead braking at -2 now; the model's first knot is 2 s out
    v_knots = np.array([15., 13., 11., 9., 7., 5.])
    v = radar_speed(v_knots, 15., -2., 0.)
    assert (v[1] - v[0]) / (FINE_T[1] - FINE_T[0]) == pytest.approx(-2., rel=0.05)

  def test_ends_on_the_forecast(self):
    v_knots = np.array([15., 13., 11., 9., 7., 5.])
    v = radar_speed(v_knots, 15., 1., 1.5)
    late = FINE_T >= RADAR_HANDOVER_T
    np.testing.assert_allclose(v[late], np.interp(FINE_T[late], LEAD_T_IDXS, v_knots))

  def test_never_negative(self):
    assert (radar_speed(np.array([2., 0., 0., 0., 0., 0.]), 2., -8., 0.) >= 0.).all()

  def test_measured_braking_is_a_floor(self):
    # radar measures -2 and upstream holds it (tau 0); the model sees the lead easing off
    v_knots = np.array([15., 14., 14., 14., 14., 14.])
    v = radar_speed(v_knots, 15., RADAR_BRAKING_A_BP[0] - 1.5, 0.)
    measured = np.maximum(15. + (RADAR_BRAKING_A_BP[0] - 1.5) * FINE_T, 0.)
    assert (v <= measured + 1e-6).all()

  def test_floor_never_slows_a_harder_forecast(self):
    # the model sees harder braking than radar measures: the forecast stands
    v_knots = np.array([15., 9., 4., 1., 0., 0.])
    v = radar_speed(v_knots, 15., -1., 1.5)
    late = FINE_T >= RADAR_HANDOVER_T
    np.testing.assert_allclose(v[late], np.interp(FINE_T[late], LEAD_T_IDXS, v_knots))

  def test_no_floor_without_measured_braking(self):
    v_knots = np.array([15., 15., 16., 17., 18., 18.])
    v = radar_speed(v_knots, 15., RADAR_BRAKING_A_BP[1] + 0.05, 1.5)
    late = FINE_T >= RADAR_HANDOVER_T
    np.testing.assert_allclose(v[late], np.interp(FINE_T[late], LEAD_T_IDXS, v_knots))

  def test_measured_launch_lifts_the_low_speed_limit(self):
    model_v = [1., 4., 8., 10., 10., 10.]
    np.testing.assert_allclose(forecast_speed(1., model_v, a_lead=RADAR_LAUNCH_A_BP[1], radar=True), model_v)
    np.testing.assert_allclose(forecast_speed(1., model_v, a_lead=RADAR_LAUNCH_A_BP[1]), 1.)  # camera: no lift
    np.testing.assert_allclose(forecast_speed(1., model_v, a_lead=0., radar=True), 1.)  # nothing measured

  def test_no_lift_while_braking(self):
    np.testing.assert_allclose(forecast_speed(1., [1., 4., 8., 10., 10., 10.], a_lead=-1.5, radar=True), 1.)

  def test_distance_is_the_integral_of_speed(self):
    xv = radar_trajectory(30., np.full(len(FINE_T), 10.))
    np.testing.assert_allclose(xv[:, 0], 30. + 10. * T_IDXS, atol=1e-6)


class TestMatch:
  def test_off_is_upstream(self):
    mpc, forecast, upstream = build(radar=False)
    sm = radar_sm()
    lead_xv = run(forecast, mpc, sm)
    assert forecast.inhibits[0] == Inhibit.radar
    np.testing.assert_array_equal(lead_xv, upstream(sm['radarState'].leadOne))

  def test_matched_track_takes_the_forecast(self):
    mpc, forecast, upstream = build()
    sm = radar_sm()
    lead_xv = run(forecast, mpc, sm)
    assert forecast.inhibits[0] == Inhibit.none and forecast.weights[0] == 1.
    np.testing.assert_allclose(lead_xv, expected(sm))
    assert (lead_xv[1:, 0] < upstream(sm['radarState'].leadOne)[1:, 0]).all()

  def test_lead_two_too(self):
    mpc, forecast, _ = build()
    sm = radar_sm()
    run(forecast, mpc, sm)
    assert forecast.inhibits[1] == Inhibit.none
    np.testing.assert_allclose(forecast.lead_xv[1], expected(sm, lead=1))

  @pytest.mark.parametrize("d_rel,offset,ok", [
    (D_REL, RADAR_MATCH_DIST - 0.5, True),
    (D_REL, RADAR_MATCH_DIST + 0.5, False),
    # a far lead gets the tolerance relative to the radar's range
    (100., RADAR_MATCH_DIST_REL * 100. * 0.8, True),
    (100., RADAR_MATCH_DIST_REL * 100. * 1.2, False),
  ])
  def test_range_tolerance(self, d_rel, offset, ok):
    mpc, forecast, upstream = build()
    sm = with_lead(radar_sm(d_rel=d_rel), dRel=d_rel - offset)
    lead_xv = run(forecast, mpc, sm)
    assert forecast.inhibits[0] == (Inhibit.none if ok else Inhibit.mismatch)
    if not ok:
      np.testing.assert_array_equal(lead_xv, upstream(sm['radarState'].leadOne))

  @pytest.mark.parametrize("dv,ok", [(RADAR_MATCH_SPEED - 0.2, True), (RADAR_MATCH_SPEED + 0.2, False)])
  def test_relative_speed_tolerance(self, dv, ok):
    mpc, forecast, _ = build()
    sm = with_lead(radar_sm(), vRel=SLOWING[0] - V_EGO + dv)
    run(forecast, mpc, sm)
    assert forecast.inhibits[0] == (Inhibit.none if ok else Inhibit.mismatch)

  def test_compares_relative_speeds(self):
    # the model reads its own ego 1 m/s fast and the lead 1 m/s fast: they agree
    mpc, forecast, _ = build()
    sm = with_lead(radar_sm(model_v=np.array(SLOWING) + 1.), vLead=SLOWING[0], vRel=SLOWING[0] - V_EGO)
    md = sm['modelV2'].as_builder()
    md.velocity.x = [V_EGO + 1.] * 33
    sm['modelV2'] = md.as_reader()
    run(forecast, mpc, sm)
    assert forecast.inhibits[0] == Inhibit.none

  def test_unconfirmed_low_speed_track_is_upstream(self):
    # radard's low-speed override: a close stationary track with modelProb 0
    mpc, forecast, upstream = build()
    sm = radar_sm(model_prob=0.)
    lead_xv = run(forecast, mpc, sm)
    assert forecast.inhibits[0] == Inhibit.mismatch
    np.testing.assert_array_equal(lead_xv, upstream(sm['radarState'].leadOne))

  def test_no_model_ego_speed_is_invalid(self):
    mpc, forecast, upstream = build()
    sm = radar_sm()
    md = sm['modelV2'].as_builder()
    md.velocity.x = []
    sm['modelV2'] = md.as_reader()
    lead_xv = run(forecast, mpc, sm)
    assert forecast.inhibits[0] == Inhibit.invalid
    np.testing.assert_array_equal(lead_xv, upstream(sm['radarState'].leadOne))

  def test_non_finite_tau_is_invalid(self):
    mpc, forecast, _ = build()
    run(forecast, mpc, with_lead(radar_sm(), aLeadTau=float('nan')))
    assert forecast.inhibits[0] == Inhibit.invalid

  def test_stopped_lead_with_a_predicted_launch_stays_put(self):
    mpc, forecast, _ = build()
    lead_xv = run(forecast, mpc, radar_sm(d_rel=20., v_lead=0., model_v=[0., 2., 5., 8., 10., 12.]))
    assert forecast.inhibits[0] == Inhibit.none
    np.testing.assert_allclose(lead_xv[:, 1], 0.)
    np.testing.assert_allclose(lead_xv[:, 0], lead_xv[0, 0])


class TestTrackSwitch:
  def test_new_matched_track_keeps_the_weight(self):
    mpc, forecast, _ = build()
    run(forecast, mpc, radar_sm(track=7))
    sm = radar_sm(track=8)
    lead_xv = run(forecast, mpc, sm, frames=1)
    assert forecast.weights[0] == 1.
    np.testing.assert_allclose(lead_xv, expected(sm))

  def test_new_unmatched_track_drops_at_once(self):
    mpc, forecast, upstream = build()
    run(forecast, mpc, radar_sm(track=7))
    sm = with_lead(radar_sm(track=8), dRel=15.)
    lead_xv = run(forecast, mpc, sm, frames=1)
    assert forecast.weights[0] == 0. and forecast.inhibits[0] == Inhibit.mismatch
    np.testing.assert_array_equal(lead_xv, upstream(sm['radarState'].leadOne))

  def test_same_track_unmatched_fades(self):
    mpc, forecast, _ = build()
    run(forecast, mpc, radar_sm(track=7))
    run(forecast, mpc, with_lead(radar_sm(track=7), dRel=15.), frames=1)
    assert 0. < forecast.weights[0] < 1.

  def test_camera_to_radar_keeps_the_weight(self):
    # radard falls back to the model's lead between radar matches; the same car comes back as a track
    mpc, forecast, _ = build()
    run(forecast, mpc, build_sm(model_v=SLOWING))
    assert forecast.weights[0] == 1.
    sm = radar_sm()
    run(forecast, mpc, sm, frames=1)
    assert forecast.weights[0] == 1. and forecast.inhibits[0] == Inhibit.none
