"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The per-car curve lateral ceiling: driven with synthetic curves on a simulated EPS whose
torque runs out at a fixed lateral acceleration.
"""
import numpy as np
import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import lat_limit_learner as llm
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.lat_limit_learner import LatLimitLearner
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.vision_controller import (
  _A_LAT_CURVE, _A_LAT_CURVE_BP, _PLAN_MARGIN, curve_speed)

V = 20.  # m/s, between the first two breakpoints


class FakeParams:
  def __init__(self):
    self.store = {}

  def get(self, key):
    return self.store.get(key)

  def put(self, key, dat):
    self.store[key] = dat


def make(params=None, fingerprint="MAZDA_CX5_2022"):
  return LatLimitLearner(_A_LAT_CURVE_BP, _A_LAT_CURVE, fingerprint, params)


def settle(lrn: LatLimitLearner, frames: int = int(3 / DT_MDL)):
  """Straight road with lateral engaged: clears the steering-pressed hold."""
  for _ in range(frames):
    lrn.update(V, 0., 0., True, False, False)


def drive_curve(lrn: LatLimitLearner, eps_limit: float, v: float = V, steering_pressed: bool = False, t: float = 4.):
  """A curve the planner takes at the learned ceiling (planning margin included). The EPS
  delivers at most eps_limit; past it the controller saturates and the car undershoots."""
  demand = lrn.ceiling_at(v) * _PLAN_MARGIN
  for _ in range(int(t / DT_MDL)):
    actual = min(demand, eps_limit)
    lrn.update(v, actual, demand, True, demand > eps_limit, steering_pressed)
  lrn.update(v, 0., 0., True, False, False)  # out of the curve


class TestLatLimitLearner:
  def test_starts_on_the_shared_table(self):
    lrn = make()
    assert not lrn.learned
    np.testing.assert_allclose(lrn.ceiling, _A_LAT_CURVE)

  def test_saturation_lowers_the_ceiling_at_that_speed_only(self):
    lrn = make()
    settle(lrn)
    drive_curve(lrn, eps_limit=1.5)
    assert lrn.learned
    assert lrn.ceiling_at(V) < np.interp(V, _A_LAT_CURVE_BP, _A_LAT_CURVE)
    assert lrn.ceiling[-1] == pytest.approx(_A_LAT_CURVE[-1])  # 70 mph untouched by a 45 mph curve

  def test_weak_steering_converges_below_its_limit_and_stops_saturating(self):
    eps_limit = 1.6
    lrn = make()
    settle(lrn)
    for _ in range(200):
      drive_curve(lrn, eps_limit)
    # the plan sits at or just under the torque limit, short of the warning's 1.2x undershoot
    planned = lrn.ceiling_at(V) * _PLAN_MARGIN
    assert eps_limit * 0.9 < planned <= eps_limit + 0.01
    sats = lrn.saturations
    for _ in range(50):
      drive_curve(lrn, eps_limit)
    # converged: the step is at its floor, so probing past the limit is rare
    assert lrn.saturations - sats <= 2

  def test_strong_steering_climbs_to_the_iso_bound(self):
    lrn = make()
    settle(lrn)
    for _ in range(100):
      drive_curve(lrn, eps_limit=5.)
    assert lrn.ceiling_at(V) == pytest.approx(llm.A_LAT_MAX)
    assert lrn.saturations == 0

  def test_driver_steering_learns_nothing(self):
    lrn = make()
    settle(lrn)
    drive_curve(lrn, eps_limit=1.5, steering_pressed=True)
    drive_curve(lrn, eps_limit=5.)  # within 2 s of the press: still excluded at its start
    assert not lrn.learned

  def test_a_curve_below_the_ceiling_is_no_evidence(self):
    lrn = make()
    settle(lrn)
    for _ in range(int(4 / DT_MDL)):
      lrn.update(V, 1.2, 1.2, True, False, False)
    lrn.update(V, 0., 0., True, False, False)
    assert not lrn.learned

  def test_persists_per_fingerprint(self):
    params = FakeParams()
    lrn = make(params)
    settle(lrn)
    drive_curve(lrn, eps_limit=1.5)
    restored = make(params)
    assert restored.learned
    np.testing.assert_allclose(restored.ceiling, lrn.ceiling)
    np.testing.assert_allclose(restored.step, lrn.step)
    other = make(params, fingerprint="MAZDA_3_2019")
    assert not other.learned

  def test_rejects_a_bad_cache(self):
    params = FakeParams()
    params.put(llm.PARAM_KEY, {"version": llm.VERSION, "fingerprint": "MAZDA_CX5_2022", "bp": _A_LAT_CURVE_BP,
                               "ceiling": [float('nan')] * 3, "step": [0.05] * 3})
    assert not make(params).learned
    params.put(llm.PARAM_KEY, {"version": llm.VERSION, "fingerprint": "MAZDA_CX5_2022", "bp": [1., 2.],
                               "ceiling": [2., 2.], "step": [0.05] * 2})
    assert not make(params).learned


class TestCurveSpeedTable:
  def test_shared_table_is_the_default(self):
    kappa = np.linspace(0.0005, 0.03, 50)
    np.testing.assert_allclose(curve_speed(kappa), curve_speed(kappa, list(_A_LAT_CURVE)))

  def test_learned_ceiling_meets_each_breakpoint(self):
    a_lat = [1.6, 1.7, 1.8]
    for v, a in zip(_A_LAT_CURVE_BP, a_lat, strict=True):
      assert float(curve_speed(a * _PLAN_MARGIN / v ** 2, a_lat)) == pytest.approx(v)
