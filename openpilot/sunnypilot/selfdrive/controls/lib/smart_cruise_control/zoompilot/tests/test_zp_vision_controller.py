"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Vision controller: lifecycle, holding the set speed, braking at the budget, arriving at
the allowed speed, and the far-field curvature bias correction. The highway horizon,
per-path budgets, publication ramp and lookahead wire live in test_zp_vision_horizon.py;
the road rendering and the base case live in vision_harness.py.
"""
import pytest

from openpilot.cereal import custom
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import vision_controller
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.vision_controller import SmartCruiseControlVision
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.tests.vision_harness import (
  A_LAT_MAX, CURVE_KAPPA, CURVE_V, SETPOINT, V_EGO, VisionCase, curve_at, flat_ceiling, make_cp, patch_gain)

VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.VisionState


class TestLifecycle(VisionCase):

  def test_initial_state(self):
    assert self.scc_v.state == VisionState.disabled
    assert not self.scc_v.is_active
    assert self.scc_v.output_v_target == V_CRUISE_UNSET
    assert self.scc_v.output_a_target == 0.

  def test_param_disable(self):
    self.params.put_bool("SmartCruiseControlVision", False, block=True)
    self.scc_v.enabled = False
    self.run_road(V_EGO, curve_at(50.))
    assert self.scc_v.state == VisionState.disabled

  def test_long_disabled(self):
    self.run_road(V_EGO, curve_at(50.), enabled=False)
    assert self.scc_v.state == VisionState.disabled
    assert self.scc_v.output_v_target == V_CRUISE_UNSET

  def test_override_suspends_control(self):
    self.run_road(V_EGO, curve_at(50.), override=True)
    assert self.scc_v.state == VisionState.overriding
    assert self.scc_v.output_v_target == V_CRUISE_UNSET


class TestHoldSetSpeed(VisionCase):

  def test_straight_road_never_acts(self):
    self.run_road(V_EGO, lambda s: 0., n=10)
    assert self.scc_v.state == VisionState.enabled
    assert self.scc_v.output_v_target == V_CRUISE_UNSET
    assert self.scc_v.a_required == 0.

  def test_distant_curve_holds_set_speed(self):
    # a curve 185 m out, as the model actually reports one at that range: even with the
    # under-read corrected it asks well under the 0.7 * 1.2 commit, so the car holds
    self.run_road(V_EGO, curve_at(185., kappa=0.012), attenuate=True)
    assert self.scc_v.state == VisionState.enabled
    assert self.scc_v.output_v_target == V_CRUISE_UNSET
    assert 0. < self.scc_v.a_required < 0.84


class TestBrakeAtBudget(VisionCase):

  def test_curve_inside_braking_distance_engages(self):
    self.run_road(V_EGO, curve_at(100.))
    assert self.scc_v.state == VisionState.entering
    assert self.scc_v.is_active
    # target leads v_ego by the required decel, capped by the profile
    assert MIN_V < self.scc_v.output_v_target < V_EGO - 0.5
    assert self.scc_v.output_a_target < 0.

  def test_a_target_is_jerk_ramped(self):
    sm = self.make_sm(V_EGO, curve_at(100.))
    prev = 0.
    j = self.scc_v.limits.jerk(V_EGO)
    for i in range(45):
      self.scc_v.update(sm, True, False, V_EGO, 0., SETPOINT)
      a = self.scc_v.output_a_target
      if i:
        assert a <= prev + 1e-9
        assert prev - a <= j * DT_MDL + 1e-6
      prev = a
    assert prev < -1.0  # converged to a material deceleration request

  def test_planned_slowdown_does_not_lower_the_estimate(self):
    # Geometry-derived curvature must remain stable as the model velocity plan slows.
    self.run_road(V_EGO, curve_at(100.), v_model=0.7 * V_EGO)
    assert self.scc_v.is_active

  def test_slowing_toward_the_curve_stays_committed(self):
    self.run_road(V_EGO, curve_at(100.))
    assert self.scc_v.is_active
    self.run_road(14., curve_at(40.), n=1)
    assert self.scc_v.is_active
    assert self.scc_v.solver_active


class TestArriveAtAllowedSpeed(VisionCase):

  def test_holds_allowed_speed_inside_the_curve(self):
    # approach a touch fast, curve at the bumper, read as the model reports it: on a perfect
    # sensor the gain reads the curve's own far half 1.44x tighter at 95 m and plans 7.7 m/s
    self.run_road(12., curve_at(0.), cur_curvature=CURVE_KAPPA, attenuate=True)
    assert self.scc_v.is_active
    # settled at the allowed speed: hold it, do not re-accelerate toward the setpoint
    self.run_road(CURVE_V, curve_at(0.), cur_curvature=CURVE_KAPPA, n=2, attenuate=True)
    assert self.scc_v.state == VisionState.turning
    assert abs(self.scc_v.output_v_target - CURVE_V) < 1.0

  def test_releases_when_the_road_straightens(self):
    self.run_road(12., curve_at(0.), cur_curvature=CURVE_KAPPA)
    assert self.scc_v.is_active
    self.run_road(CURVE_V, lambda s: 0., cur_curvature=0., n=3)
    assert self.scc_v.state == VisionState.enabled
    assert self.scc_v.output_v_target == V_CRUISE_UNSET

  def test_hairpin_floors_at_min_v(self):
    # kappa 0.12 allows about 4 m/s, below the 20 km/h operating floor
    self.run_road(6., curve_at(0., kappa=0.12), cur_curvature=0.12)
    assert self.scc_v.is_active
    assert self.scc_v.output_v_target == MIN_V


class TestFarFieldCurvatureBias(VisionCase):

  def test_recovers_an_attenuated_far_corner(self):
    # a corner the model reports at 55% of its real curvature: the correction pulls the
    # planned speed back toward the truth instead of planning for the corner it was told
    road = curve_at(110., kappa=0.012)
    self.run_road(V_EGO, road, attenuate=True)
    truth = (A_LAT_MAX * vision_controller._PLAN_MARGIN / 0.012) ** 0.5
    with patch_gain([1.0] * len(vision_controller._KAPPA_BIAS_GAIN)):
      raw = SmartCruiseControlVision(make_cp())
      self.run_road(V_EGO, road, scc=raw, attenuate=True)
    # as reported the corner looks far faster than it is; corrected it lands much closer
    assert raw.v_dip_ahead > truth + 4.
    assert self.scc_v.v_dip_ahead < raw.v_dip_ahead - 2.
    assert self.scc_v.v_dip_ahead > truth  # under-corrects: the cap is deliberate

  def test_bias_correction_commits_earlier(self):
    # Bias correction moves the reported corner above the commit threshold.
    road = curve_at(110., kappa=0.013)
    self.run_road(V_EGO, road, attenuate=True)
    assert self.scc_v.is_active

    with patch_gain([1.0] * len(vision_controller._KAPPA_BIAS_GAIN)):
      raw = SmartCruiseControlVision(make_cp())
      self.run_road(V_EGO, road, scc=raw, attenuate=True)
    assert not raw.is_active
    assert self.scc_v.a_required > raw.a_required

  def test_gentle_near_bend_does_not_block_braking_for_a_tighter_corner(self):
    # a gentle bend under the nose must not stop the car braking for a hairpin beyond it
    def road(s):
      return 0.008 if s < 90. else 0.06
    self.run_road(V_EGO, road, n=5)
    assert self.scc_v.is_active
    assert self.scc_v.output_v_target < V_EGO - 2.

  def test_straight_road_is_unaffected_by_the_gain(self):
    # a gain on a kappa of zero is still zero; no false braking is bought with it
    self.run_road(V_EGO, lambda s: 0., n=10)
    assert self.scc_v.a_required == 0.
    assert self.scc_v.output_v_target == V_CRUISE_UNSET

  @pytest.mark.parametrize("kappa, d0", [(1. / 645., 60.), (1. / 556., 60.)], ids=["r645", "r556"])
  @pytest.mark.parametrize("op_long", [True, False], ids=["op_long", "stock"])
  def test_highway_bend_the_raw_path_allows_never_commits(self, kappa, d0, op_long):
    # 70 mph, perfect geometry, inside the near window: an r=645 m bend 60 m out sits at
    # 1.49 m/s2 at the set speed, under the 1.71 planned, and an r=556 m bend 60 m out is
    # take-able at sqrt(1.71 * 556) = 30.8 m/s (the r=500 m case at the old 1.9 planned;
    # r=500 m is 1.92 m/s2 at 31 m/s, over the 1.8 ceiling, a real corner now). Multiplied
    # by the far-field gain both read as corners; above the fitted speed band the gain is
    # gone, so neither may commit, on either path.
    v = 31.
    scc = SmartCruiseControlVision(make_cp(op_long=op_long))
    self.run_road(v, curve_at(d0, kappa), n=40, setpoint=v, scc=scc)
    assert not scc.is_active, (kappa, d0, op_long)
    assert scc.output_v_target == V_CRUISE_UNSET
    assert scc.output_a_target == 0.

  def test_gain_fades_out_above_the_fitted_speed_band(self):
    # the same attenuated corner at 70 mph, inside the near window, plans exactly as it
    # would with no gain
    v, road = 31., curve_at(80., kappa=0.012)
    self.run_road(v, road, setpoint=v, attenuate=True)
    with patch_gain([1.0] * len(vision_controller._KAPPA_BIAS_GAIN)):
      raw = SmartCruiseControlVision(make_cp())
      self.run_road(v, road, setpoint=v, scc=raw, attenuate=True)
    assert self.scc_v.a_required > 0.
    assert self.scc_v.a_required == pytest.approx(raw.a_required)
    assert self.scc_v.v_dip_ahead == pytest.approx(raw.v_dip_ahead)
    road = curve_at(150., kappa=0.012)
    # and is still whole at the top of the band
    v = 22.
    self.run_road(v, road, setpoint=v, attenuate=True)
    with patch_gain([1.0] * len(vision_controller._KAPPA_BIAS_GAIN)):
      raw = SmartCruiseControlVision(make_cp())
      self.run_road(v, road, setpoint=v, scc=raw, attenuate=True)
    assert self.scc_v.v_dip_ahead < raw.v_dip_ahead - 1.

  @flat_ceiling(1.8)
  def test_real_curve_commits_exactly_as_before_the_fade(self):
    # The fitted speed band retains the calibrated correction. Both paths bind at the 95.2 m
    # sample: 0.02 * 0.631 attenuation * 1.441 gain = 0.01819, allowed sqrt(1.71 / 0.01819) = 9.695
    v = 15.6
    road = curve_at(90., kappa=CURVE_KAPPA)
    self.run_road(v, road, setpoint=v, attenuate=True)
    assert self.scc_v.is_active
    # lead 15.6 * 0.36 + 15.6 * 1.2 / (2 * 1.051 jerk) = 14.52 m: (15.6^2 - 9.695^2) / (2 * 80.69)
    assert self.scc_v.a_required == pytest.approx(0.926, abs=2e-3)
    assert self.scc_v.output_v_target == pytest.approx(14.674, abs=2e-3)  # 15.6 - 0.926

    stock = self.stock()
    self.run_road(v, road, setpoint=v, scc=stock, attenuate=True)
    assert stock.is_active
    # lead walks only the tracking gap: 15.6 * (1.0 + 8 mph / 4 mph/s) = 46.8 m, over 48.41 m
    assert stock.a_required == pytest.approx(1.543, abs=2e-3)
    assert stock.output_v_target == pytest.approx(9.695, abs=2e-3)  # sent to the dip


class TestCurveRecovery(VisionCase):
  """Inside a curve on stock ACC the target follows the profile up, and the plan lets go once
  the near field no longer limits the car."""
  KAPPA = 0.01  # r = 100 m

  def commit(self, scc, setpoint=SETPOINT):
    # brake for the curve from the set speed, then drive into it
    self.run_road(setpoint, curve_at(60., self.KAPPA), n=5, setpoint=setpoint, scc=scc, attenuate=True)
    assert scc.solver_active
    return scc

  def in_curve(self, scc, v, setpoint=SETPOINT):
    return self.run_road(v, lambda s: self.KAPPA, n=5, cur_curvature=self.KAPPA, setpoint=setpoint, scc=scc,
                         attenuate=True)

  def test_lateral_ceiling_follows_the_set_speed(self):
    (sp_lo, sp_hi), (a_lo, a_hi) = vision_controller._A_LAT_REG_V_BP, vision_controller._A_LAT_REG_V
    back_road, highway = self.stock(), self.stock()
    self.in_curve(back_road, 12., setpoint=sp_lo)
    self.in_curve(highway, 12., setpoint=sp_hi)
    assert back_road.v_near_min == pytest.approx(highway.v_near_min * (a_lo / a_hi) ** 0.5)

  def test_stock_target_rises_to_the_profile_below_the_allowed_speed(self):
    scc = self.commit(self.stock())
    v = 13.  # just under the curve's allowed speed
    self.in_curve(scc, v)
    assert scc.solver_active and scc.a_required == 0.
    assert v < scc.v_near_min < v + 1.
    assert scc.output_v_target > v
    assert scc.output_v_target <= scc.v_dip_ahead

  def test_releases_once_the_near_field_stops_limiting(self):
    scc = self.commit(self.stock())
    v = 11.  # well under the allowed speed, still far under the set speed
    self.in_curve(scc, v)
    assert scc.v_near_min < SETPOINT
    assert not scc.solver_active
    assert scc.output_v_target == V_CRUISE_UNSET
    # the restore is still capped at the curve by the lookahead
    assert scc.v_ahead_min < SETPOINT
