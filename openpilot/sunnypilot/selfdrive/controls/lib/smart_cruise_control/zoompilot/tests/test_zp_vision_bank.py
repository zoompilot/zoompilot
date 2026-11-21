"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Vision controller: road bank. The ceiling moves only by a curve's departure from the typical
bank for its speed, read from the road roll now plus the model's predicted change ahead.
"""
import numpy as np
import pytest

from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY as G
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import vision_controller as vc
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.tests.vision_harness import (
  VisionCase, curve_at, model_for_road, vehicle_params)

HIGHWAY_KAPPA = 0.004  # r = 250 m, a ~49 mph curve: typical bank +0.48 m/s2
TYPICAL_ROLL = float(np.interp(float(vc.curve_speed(HIGHWAY_KAPPA)), vc._BANK_TYPICAL_BP, vc._BANK_TYPICAL)) / G


class TestBank(VisionCase):

  def near_min(self, roll, kappa=HIGHWAY_KAPPA):
    """v_near_min driving a constant curve (sign of kappa = side) at its allowed speed."""
    v = float(vc.curve_speed(abs(kappa)))
    scc = self.stock()
    self.run_road(v, lambda s: kappa, n=3, cur_curvature=kappa, setpoint=v + 5., scc=scc, roll=roll)
    return scc.v_near_min

  def test_typical_bank_plans_as_without_a_roll_reading(self):
    assert self.near_min(TYPICAL_ROLL) == pytest.approx(self.near_min(None), rel=1e-6)

  def test_flat_curve_slows_and_adverse_camber_slows_more(self):
    typical, flat, adverse = self.near_min(TYPICAL_ROLL), self.near_min(0.), self.near_min(-0.03)
    assert adverse < flat < typical
    # flat: the ceiling gives up the typical bank's carry
    a_flat = flat ** 2 * HIGHWAY_KAPPA
    assert a_flat == pytest.approx(typical ** 2 * HIGHWAY_KAPPA - TYPICAL_ROLL * G * vc._PLAN_MARGIN, rel=1e-3)

  def test_steep_bank_lets_through_up_to_the_cap(self):
    typical = self.near_min(TYPICAL_ROLL)
    steeper = self.near_min(TYPICAL_ROLL + 0.02)
    capped = self.near_min(vc._ROLL_MAX)
    assert typical < steeper < capped
    assert capped ** 2 * HIGHWAY_KAPPA == pytest.approx(typical ** 2 * HIGHWAY_KAPPA + vc._BANK_SHIFT_MAX * vc._PLAN_MARGIN, rel=1e-3)

  def test_roll_carries_by_the_curve_side(self):
    # positive roll banks a left curve and tilts a right one the wrong way
    assert self.near_min(TYPICAL_ROLL, -HIGHWAY_KAPPA) < self.near_min(0., HIGHWAY_KAPPA)
    assert self.near_min(-TYPICAL_ROLL, -HIGHWAY_KAPPA) == pytest.approx(self.near_min(TYPICAL_ROLL, HIGHWAY_KAPPA), rel=1e-6)

  def test_ceiling_keeps_its_floor(self):
    a_lat = float(np.interp(HIGHWAY_KAPPA, *vc.curve_table()))
    shifted = vc.curve_speed(HIGHWAY_KAPPA, shift=-10.)
    assert float(shifted) ** 2 * HIGHWAY_KAPPA == pytest.approx(vc._BANK_FLOOR * a_lat)

  def test_predicted_bank_ahead_raises_the_plan(self):
    """A flat approach to a curve the model sees banked: the plan takes the predicted bank."""
    v, d_curve = 27., 150.
    plans = []
    for roll_fn in (None, lambda s: 0.08 if s >= d_curve else 0.):
      scc = self.stock()
      sm = self.make_sm(v, curve_at(d_curve, HIGHWAY_KAPPA), roll=0.)
      sm['modelV2'] = model_for_road(v, curve_at(d_curve, HIGHWAY_KAPPA), roll_fn=roll_fn).modelV2
      for _ in range(3):
        scc.update(sm, True, False, v, 0., v)
      plans.append(scc.v_dip_ahead)
    flat, banked = plans
    assert banked > flat

  def test_path_roll_falls_back_to_the_roll_now(self):
    no_orientation = model_for_road(20., lambda s: 0.).modelV2
    assert vc.path_roll(0.03, no_orientation) == 0.03
    predicted = model_for_road(20., lambda s: 0., roll_fn=lambda s: 0.01).modelV2
    assert np.allclose(vc.path_roll(0.03, predicted), 0.04)

  def test_invalid_roll_is_ignored(self):
    vp = vehicle_params(None)
    vp_with_roll = vehicle_params(0.1)
    vp_with_roll.valid = False
    v = float(vc.curve_speed(HIGHWAY_KAPPA))
    results = []
    for p in (vp, vp_with_roll):
      scc = self.stock()
      sm = self.make_sm(v, lambda s: HIGHWAY_KAPPA, cur_curvature=HIGHWAY_KAPPA)
      sm['vehicleParameters'] = p
      for _ in range(3):
        scc.update(sm, True, False, v, 0., v + 5.)
      results.append(scc.v_near_min)
    assert results[0] == results[1]
