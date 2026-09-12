"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.speed_profile import (
  allowed_speed, backward_pass, required_decel, lead_distance, min_profile_speed)

A_LAT = 2.0
A_BUDGET = 1.2


def path(n=33, spacing=10.):
  return np.arange(n) * spacing


class TestAllowedSpeed:
  def test_zero_curvature_is_unconstrained(self):
    v = allowed_speed(np.zeros(10), A_LAT)
    assert np.all(np.isinf(v))

  def test_matches_lat_acc_ceiling(self):
    # kappa 0.02 -> r = 50 m -> v = sqrt(2.0 * 50) = 10 m/s
    v = allowed_speed([0.02], A_LAT)
    assert math.isclose(v[0], 10.0)

  def test_negative_curvature_treated_as_straight(self):
    v = allowed_speed([-0.05], A_LAT)
    assert math.isinf(v[0])


class TestBackwardPass:
  def test_straight_path_stays_unconstrained(self):
    d = path()
    v_max = backward_pass(allowed_speed(np.zeros(33), A_LAT), d, A_BUDGET)
    assert np.all(np.isinf(v_max))

  def test_single_constraint_propagates_analytically(self):
    # one 10 m/s constraint 100 m out: v_max at the car = sqrt(10^2 + 2*1.2*100)
    d = path(11, 10.)
    v_allowed = np.full(11, np.inf)
    v_allowed[10] = 10.
    v_max = backward_pass(v_allowed, d, A_BUDGET)
    assert math.isclose(v_max[0], math.sqrt(100. + 2. * A_BUDGET * 100.))

  def test_extended_curve_pins_the_whole_region(self):
    # points 5..8 all allow 10; inside the region the profile is the local limit,
    # not the braking parabola from the region's end
    d = path(11, 10.)
    v_allowed = np.full(11, np.inf)
    v_allowed[5:9] = 10.
    v_max = backward_pass(v_allowed, d, A_BUDGET)
    assert np.allclose(v_max[5:9], 10.)
    assert v_max[4] > 10.

  def test_two_curves_second_tighter(self):
    # nearer gentle curve (15 m/s at 50 m), farther sharp one (8 m/s at 100 m);
    # the profile respects both, and at the gentle curve it may be pinned by the sharp one
    d = path(11, 10.)
    v_allowed = np.full(11, np.inf)
    v_allowed[5] = 15.
    v_allowed[10] = 8.
    v_max = backward_pass(v_allowed, d, A_BUDGET)
    from_sharp_at_5 = math.sqrt(64. + 2. * A_BUDGET * 50.)
    assert math.isclose(v_max[5], min(15., from_sharp_at_5))
    assert v_max[0] <= math.sqrt(v_max[1] ** 2 + 2. * A_BUDGET * 10.) + 1e-9

  def test_never_exceeds_local_allowed(self):
    rng = np.random.default_rng(0)
    v_allowed = rng.uniform(5., 40., 33)
    v_max = backward_pass(v_allowed, path(), A_BUDGET)
    assert np.all(v_max <= v_allowed + 1e-9)

  def test_degenerate_spacing_propagates_constraint(self):
    # repeated distance samples (ds = 0) must carry the constraint through unchanged
    d = np.array([0., 10., 10., 20.])
    v_allowed = np.array([np.inf, np.inf, 9., np.inf])
    v_max = backward_pass(v_allowed, d, A_BUDGET)
    assert math.isclose(v_max[1], 9.)


class TestRequiredDecel:
  def test_nothing_binding(self):
    assert required_decel(20., np.full(5, np.inf), path(5)) == 0.
    assert required_decel(20., np.full(5, 25.), path(5)) == 0.

  def test_single_constraint_analytic(self):
    # 20 -> 10 m/s over 100 m: (400 - 100) / 200 = 1.5
    a = required_decel(20., np.array([np.inf, 10.]), np.array([0., 100.]))
    assert math.isclose(a, 1.5)

  def test_binding_constraint_wins(self):
    # far gentle need vs near urgent need: the max is returned
    v = np.array([np.inf, 18., 10.])
    d = np.array([0., 20., 200.])
    a = required_decel(20., v, d)
    assert math.isclose(a, (400. - 324.) / 40.)

  def test_lead_distance_makes_it_more_urgent(self):
    base = required_decel(20., np.array([10.]), np.array([100.]))
    led = required_decel(20., np.array([10.]), np.array([100.]), d_lead=30.)
    assert led > base
    assert math.isclose(led, 300. / 140.)

  def test_constraint_inside_lead_reads_as_very_late(self):
    a = required_decel(20., np.array([10.]), np.array([10.]), d_lead=30.)
    assert a >= 300.  # (400-100) / (2 * D_FLOOR)


class TestLeadDistance:
  def test_plain_actuation_lead(self):
    assert math.isclose(lead_distance(20., 0.36), 7.2)

  def test_jerk_ramp_term(self):
    # v * a / (2j) = 20 * 1.2 / (2 * 0.8) = 15 on top of the actuation lead
    assert math.isclose(lead_distance(20., 0.36, a_budget=1.2, jerk=0.8), 7.2 + 15.)


class TestMinProfileSpeed:
  def test_picks_dip_within_horizon(self):
    v_max = np.array([30., 25., 12., 28.])
    d = np.array([0., 40., 80., 120.])
    assert min_profile_speed(v_max, d, 100.) == 12.
    assert min_profile_speed(v_max, d, 50.) == 25.

  def test_empty_horizon_falls_back_to_now(self):
    v_max = np.array([30., 25.])
    assert min_profile_speed(v_max, np.array([5., 40.]), 1.) == 30.
