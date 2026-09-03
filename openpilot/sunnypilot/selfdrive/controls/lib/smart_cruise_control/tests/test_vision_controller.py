"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Vision controller: lifecycle, holding the set speed, braking at the budget, arriving at
the allowed speed, and the far-field curvature bias correction. The highway horizon,
per-path budgets, publication ramp and lookahead wire live in test_vision_horizon.py;
the road rendering and the base case live in vision_harness.py.
"""
import pytest

from openpilot.cereal import custom
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import vision_controller
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.vision_controller import SmartCruiseControlVision
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.tests.vision_harness import (
  CURVE_KAPPA, CURVE_V, SETPOINT, V_EGO, VisionCase, curve_at, make_cp, patch_gain)

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
    # approach a touch fast, curve at the bumper
    self.run_road(12., curve_at(0.), cur_curvature=CURVE_KAPPA)
    assert self.scc_v.is_active
    # settled at the allowed speed: hold it, do not re-accelerate toward the setpoint
    self.run_road(CURVE_V, curve_at(0.), cur_curvature=CURVE_KAPPA, n=2)
    assert self.scc_v.state == VisionState.turning
    assert abs(self.scc_v.output_v_target - CURVE_V) < 1.0

  def test_releases_when_the_road_straightens(self):
    self.run_road(12., curve_at(0.), cur_curvature=CURVE_KAPPA)
    assert self.scc_v.is_active
    self.run_road(CURVE_V, lambda s: 0., cur_curvature=0., n=3)
    assert self.scc_v.state == VisionState.enabled
    assert self.scc_v.output_v_target == V_CRUISE_UNSET

  def test_hairpin_floors_at_min_v(self):
    # kappa 0.12 allows 4.1 m/s, below the 20 km/h operating floor
    self.run_road(6., curve_at(0., kappa=0.12), cur_curvature=0.12)
    assert self.scc_v.is_active
    assert self.scc_v.output_v_target == MIN_V


class TestFarFieldCurvatureBias(VisionCase):

  def test_recovers_an_attenuated_far_corner(self):
    # a corner the model reports at 55% of its real curvature: the correction pulls the
    # planned speed back toward the truth instead of planning for the corner it was told
    road = curve_at(110., kappa=0.012)
    self.run_road(V_EGO, road, attenuate=True)
    truth = (2.0 * 0.95 / 0.012) ** 0.5
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

  def test_near_field_is_never_outvoted(self):
    # a constant-radius curve the car is already in: the far half of the SAME curve is
    # attenuated, so correcting it would settle the car below the speed the road requires.
    # The near floor holds it at the true allowed speed.
    self.run_road(12., curve_at(0.), cur_curvature=CURVE_KAPPA, attenuate=True)
    self.run_road(CURVE_V, curve_at(0.), cur_curvature=CURVE_KAPPA, n=2, attenuate=True)
    # the near field plans at the margin, and that is exactly where it settles
    near_allowed = (2.0 * 0.95 / CURVE_KAPPA) ** 0.5
    assert abs(self.scc_v.output_v_target - near_allowed) < 0.05
    # uncorrected the far half of the same curve reads gentler, so nothing drags it under
    with patch_gain([1.0] * len(vision_controller._KAPPA_BIAS_GAIN)):
      raw = SmartCruiseControlVision(make_cp())
      self.run_road(CURVE_V, curve_at(0.), cur_curvature=CURVE_KAPPA, n=3, scc=raw, attenuate=True)
    assert self.scc_v.output_v_target >= raw.output_v_target - 0.05

  def test_near_floor_does_not_block_braking_for_a_tighter_corner(self):
    # the floor only applies once the near field is what binds; a gentle bend under the
    # nose must not stop the car braking for a hairpin beyond it
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

  @pytest.mark.parametrize("kappa, d0", [(1. / 645., 60.), (1. / 500., 60.)], ids=["r645", "r500"])
  @pytest.mark.parametrize("op_long", [True, False], ids=["op_long", "stock"])
  def test_highway_bend_the_raw_path_allows_never_commits(self, kappa, d0, op_long):
    # 70 mph, perfect geometry, inside the near window: an r=645 m bend 60 m out sits at
    # 1.49 m/s2 at the set speed, under the ceiling, and an r=500 m bend 60 m out is
    # take-able at 30.8 m/s. Multiplied by the far-field gain both read as corners; above
    # the fitted speed band the gain is gone, so neither may commit, on either path.
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

  def test_real_curve_commits_exactly_as_before_the_fade(self):
    # The fitted speed band retains the calibrated correction.
    v = 15.6
    road = curve_at(90., kappa=CURVE_KAPPA)
    self.run_road(v, road, setpoint=v, attenuate=True)
    assert self.scc_v.is_active
    assert self.scc_v.a_required == pytest.approx(0.861, abs=2e-3)
    assert self.scc_v.output_v_target == pytest.approx(14.739, abs=2e-3)

    stock = SmartCruiseControlVision(make_cp(op_long=False))
    self.run_road(v, road, setpoint=v, scc=stock, attenuate=True)
    assert stock.is_active
    assert stock.a_required == pytest.approx(2.126, abs=2e-3)
    assert stock.output_v_target == pytest.approx(10.219, abs=2e-3)
