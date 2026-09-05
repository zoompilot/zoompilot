"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Steering on the 2022+ EPS: the torque parameters (gated on the EPS, not the model), the
EPS ceiling and rail, the driver-torque headroom against the panda's window, and the
non-delivery latch's zeroing of the command.
"""
import importlib

import numpy as np
import pytest

from opendbc.car import DT_CTRL, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.mazda.tests.conftest import LongCtrlState, car_controller, controller_params, mazda_car_state, step
from opendbc.car.mazda.values import CAR, CarControllerParams, STEER_TO_ZERO_EPS_FW

Ecu = structs.CarParams.Ecu


def _eps_fw(version: bytes) -> list[structs.CarParams.CarFw]:
  fw = structs.CarParams.CarFw()
  fw.ecu = Ecu.eps
  fw.address = 0x730
  fw.subAddress = 0
  fw.fwVersion = version
  return [fw]


SWAPPED_EPS_FW = _eps_fw(sorted(STEER_TO_ZERO_EPS_FW)[0])


def cx5_2022_params():
  return controller_params(CAR.MAZDA_CX5_2022)


def eps_swap_params():
  # A CX-5 2022+ EPS swapped into (or shared by) another Mazda: different model, same EPS.
  return controller_params(CAR.MAZDA_CX9_2021, car_fw=SWAPPED_EPS_FW)


def pre_2022_params():
  # no CX-5 EPS -> low-speed lockout
  return controller_params(CAR.MAZDA_CX5)


class TestCarControllerParams:

  def test_eps_ceiling_never_exceeds_steer_max_scale(self):
    params = cx5_2022_params()
    # The ceiling is a clamp on delivered-torque counts; the scale is STEER_MAX. The clamp is
    # only meaningful if it sits at or below the scale at every speed.
    bp, vals = params.EPS_CEILING_LOOKUP
    for v in np.arange(0.0, 40.0, 0.25):
      ceiling = np.interp(v, bp, vals)
      steer_max = np.interp(v, params.STEER_MAX_LOOKUP[0], params.STEER_MAX_LOOKUP[1])
      assert 0 < ceiling <= steer_max, f"ceiling {ceiling} vs steer_max {steer_max} at {v} m/s"

  def test_eps_ceiling_is_monotone_and_matches_the_measured_rails(self):
    params = cx5_2022_params()
    # Measured over 11.4M clean frames: 1148 below 18 mph, a monotone rolloff, hard 620 from
    # 32.5 mph up (docs/mazda-lkas-camera-tx-census.md). Nothing above 620 was ever delivered
    # above 32.5 mph in 7.5M frames, so the high-speed leg must not drift back up.
    bp, vals = params.EPS_CEILING_LOOKUP
    assert list(vals) == sorted(vals, reverse=True), "ceiling must fall monotonically with speed"
    assert np.interp(5.0, bp, vals) == 1148
    assert np.interp(14.5, bp, vals) == 620
    assert np.interp(35.0, bp, vals) == 620

  def test_steer_delta_matches_the_eps_rate_limit_at_this_steer_step(self):
    params = cx5_2022_params()
    # Per-frame controller deltas must match the 100 Hz EPS hardware slew in both directions.
    rate_hz = 1.0 / DT_CTRL / CarControllerParams.STEER_STEP
    assert params.STEER_DELTA_UP * rate_hz == pytest.approx(1200, rel=0.01)
    assert params.STEER_DELTA_DOWN * rate_hz == pytest.approx(1200, rel=0.01)

  def test_cx5_2022_has_lookup(self):
    params = cx5_2022_params()
    assert hasattr(params, 'STEER_MAX_LOOKUP')
    assert params.STEER_MAX == 1200

  @pytest.mark.parametrize("v_ego, steer_max", [(0.0, 1200), (5.0, 1200), (10.0, 1200), (14.2, 1200),
                                                (14.5, 800), (20.0, 800), (30.0, 800)])
  def test_cx5_2022_steer_max_by_speed(self, v_ego, steer_max):
    p = cx5_2022_params()
    assert round(float(np.interp(v_ego, p.STEER_MAX_LOOKUP[0], p.STEER_MAX_LOOKUP[1]))) == steer_max

  def test_cx5_2022_rate_limits(self):
    params = cx5_2022_params()
    assert params.STEER_DELTA_UP == 12
    assert params.STEER_DELTA_DOWN == 12

  @pytest.mark.parametrize("params, panda", [
    (cx5_2022_params, "TestMazdaSteerToZeroEpsSafety"),
    (eps_swap_params, "TestMazdaSteerToZeroEpsSafety"),
    (pre_2022_params, "TestMazdaSafety"),
  ], ids=["cx5_2022", "eps_swap", "pre_2022"])
  def test_rate_limits_equal_the_pandas_for_each_eps(self, params, panda):
    # The panda's driver_limit_check rejects any frame that retreats by less than max_rate_down
    # once the driver bound is below the last command, and any frame that climbs by more than
    # max_rate_up, so "tighter than the panda" is not allowed: the controller's deltas must
    # equal the panda's for the EPS class it is driving. Route 00000148 lost 171 consecutive
    # frames to a 12-count retreat against a 25-count requirement. The safety test classes
    # carry the panda numbers and are themselves proven against the compiled safety model.
    params = params()
    panda = getattr(importlib.import_module("opendbc.safety.tests.test_mazda"), panda)
    assert params.STEER_DELTA_UP == panda.MAX_RATE_UP
    assert params.STEER_DELTA_DOWN == panda.MAX_RATE_DOWN
    assert params.STEER_MAX == max(panda.MAX_TORQUE_LOOKUP[1])
    assert params.STEER_DRIVER_MULTIPLIER == panda.DRIVER_TORQUE_FACTOR
    assert params.STEER_DRIVER_ALLOWANCE == panda.DRIVER_TORQUE_ALLOWANCE

  def test_cx5_eps_driver_multiplier(self):
    # 15 is the CX-5-EPS tune (upstream stock is 1)
    assert cx5_2022_params().STEER_DRIVER_MULTIPLIER == 15

  def test_eps_swap_gets_cx5_tune(self):
    params = eps_swap_params()
    # EPS present (STEER_TO_ZERO_EPS) on a non-CX-5 model still gets the higher-authority tune
    assert params.STEER_MAX == 1200
    assert params.STEER_DRIVER_MULTIPLIER == 15
    assert hasattr(params, 'STEER_MAX_LOOKUP')

  def test_no_eps_no_lookup(self):
    params = pre_2022_params()
    assert not hasattr(params, 'STEER_MAX_LOOKUP')
    assert not hasattr(params, 'STEER_UNDELIVERED_FRAMES')
    assert params.STEER_MAX == 800
    assert params.STEER_DRIVER_MULTIPLIER == 1

  def test_undelivered_threshold_clears_normal_operation(self):
    # 20 frames is an order of magnitude clear of both populations: across 96k unblocked
    # frames with |request| > 200 the longest run of LKAS_EFFECTIVE == 0 is 2 frames, while
    # blocked runs reach 183. Derivation: tools/mazda_long/analyze_lkas_nondelivery.py
    params = cx5_2022_params()
    assert params.STEER_UNDELIVERED_FRAMES > 2 * 5
    assert params.STEER_UNDELIVERED_FRAMES < 183 // 2
    # the request has to clear the rate limiter's walk before the count can start, so the
    # minimum must sit above what one STEER_DELTA_UP step delivers
    assert params.STEER_UNDELIVERED_MIN > params.STEER_DELTA_UP

  def test_the_alert_thresholds_sit_between_the_benign_and_faulting_populations(self):
    # The camera latches CAM_LKAS.ERR_BIT_1 on a budget of LKAS requests the EPS never applies
    # (route 00000139 seg 14). carstate owns the latch (test_mazda_carstate.py); the controller
    # obeys it (test_carstate_undelivered_latch_zeroes_the_steer_command below).
    params = cx5_2022_params()
    # above the speed gate, non-delivery runs reach 30 frames on every route that never
    # faulted and 315 on the two that did, so the hold has to clear the first and not the
    # second. Same separation argument as STEER_UNDELIVERED_FRAMES itself.
    total = params.STEER_UNDELIVERED_FRAMES + params.STEER_UNDELIVERED_ALERT_FRAMES
    assert total > 30 * (100 / 85)
    assert total < 315 * (100 / 85)
    # Honda's per-car low-speed alert minimums span 2-15 mph; ours has to sit in that range,
    # above the block's own release band (p90 4.97 m/s) so a creep-away cannot arm the alert
    # on its way out, and below where route 148's fault began (5.9 m/s)
    assert params.STEER_UNDELIVERED_ALERT_MIN_SPEED > 4.97
    assert params.STEER_UNDELIVERED_ALERT_MIN_SPEED < 5.9
    assert params.STEER_UNDELIVERED_ALERT_MIN_SPEED < 15. * CV.MPH_TO_MS
    # 1660 of 1915 LKAS_BLOCK episodes in 64 h begin below 0.5 m/s (the EPS's standby from a
    # stop, read through wheel-speed quantisation); every latched block that began above it
    # and armed the alert was a fault, the slowest of them route 00000148's at 4.6 m/s
    # (tools/mazda_long/replay_undelivered_alert.py)
    assert params.STEER_UNDELIVERED_ALERT_ORIGIN_SPEED > 0.5
    assert params.STEER_UNDELIVERED_ALERT_ORIGIN_SPEED < 4.6


def test_carstate_undelivered_latch_zeroes_the_steer_command(stock_cc, stock_cs):
  # carstate owns the latch (STEER_RATE request vs delivery); the controller only obeys it
  lat = dict(long_active=False, enabled=True, lat_active=True, torque=1.0, v_ego=10.)
  for _ in range(50):
    step(stock_cc, stock_cs, **lat)
  assert stock_cc.apply_torque_last > stock_cc.params.STEER_UNDELIVERED_MIN
  step(stock_cc, stock_cs, steer_undelivered=True, **lat)
  assert stock_cc.apply_torque_last == 0
  # the latch clearing walks the command back up from zero at STEER_DELTA_UP
  step(stock_cc, stock_cs, steer_undelivered=False, **lat)
  assert stock_cc.apply_torque_last == stock_cc.params.STEER_DELTA_UP


class TestDriverTorqueHeadroom:
  """The panda enforces the same driver-torque envelope from the min/max of its own last 6
  STEER_TORQUE samples, while the controller sees one sample that is already a control cycle
  old. At a multiplier of 15 that staleness is worth 15 counts of ceiling per count of driver
  torque, and route 00000148 lost 1721 ms of LKAS delivery to it."""

  ALLOWANCE = CarControllerParams.STEER_DRIVER_ALLOWANCE
  MULTIPLIER = 15

  @staticmethod
  def drive(cc, cs, torques, sign=1.0):
    """Feed a driver-torque sequence, commanding hard the whole way, and return the command."""
    out = None
    for dt in torques:
      actuators, _ = step(cc, cs, long_active=False, enabled=False, accel=0., long_state=LongCtrlState.off,
                          lat_active=True, torque=sign, v_ego=6.0, driver_torque=dt, steering_pressed=True)
      out = actuators.torqueOutputCan
    return out

  def panda_ceiling(self, window, steer_max):
    # driver_limit_check: max_torque + (allowance + torque_driver.max) * multiplier
    return steer_max + (self.ALLOWANCE + max(window)) * self.MULTIPLIER

  def test_the_margin_holds_the_ceiling_clear_of_the_pandas(self):
    params = cx5_2022_params()
    # the window's overlap argument dies once the controller falls a full panda window
    # behind, so the margin is what covers the rest. Replay put the requirement at 2 counts.
    assert params.STEER_DRIVER_MARGIN >= 2
    # and it must stay small enough to be a margin rather than a torque cut
    assert params.STEER_DRIVER_MARGIN * self.MULTIPLIER < 0.1 * params.STEER_MAX_LOOKUP[1][0]

  def test_command_stays_under_the_panda_ceiling_while_the_driver_fights(self, cc, cs):
    params = cx5_2022_params()
    # the recorded run: driver torque walking more negative while we command hard positive.
    # Every frame here was rejected on car, starving the EPS of 0x243 entirely.
    seq = [-25, -25, -25, -27, -28, -29, -30, -28, -26, -26, -29, -29, -31, -31, -31]
    out = self.drive(cc, cs, [-20] * 20 + seq)
    steer_max = int(np.interp(6.0, params.STEER_MAX_LOOKUP[0], params.STEER_MAX_LOOKUP[1]))
    # the panda's window holds only the last 6 samples, so its ceiling uses the least
    # adverse of those -- the controller must stay at or below it
    assert out <= self.panda_ceiling(seq[-6:], steer_max)

  def test_a_steady_driver_torque_costs_nothing(self, cc, cs):
    # the window only bites when the samples disagree; a constant hand on the wheel must
    # produce exactly what the single-sample limiter always did
    windowed = self.drive(cc, cs, [-10] * 40)
    cc2 = car_controller(alpha_long=True)
    assert windowed == self.drive(cc2, mazda_car_state(cc2.CP, cc2.CP_SP), [-10] * 40)
    assert windowed > 0

  def test_the_adverse_extreme_follows_the_commanded_direction(self, cc, cs):
    # commanding negative, the binding bound is the low one, so the window's high end is
    # the adverse extreme -- picking the same end for both signs would give away authority
    seq = [30, 30, 30, 25, 20, 15, 10, 5, 0, 0]
    out = self.drive(cc, cs, [30] * 20 + seq, sign=-1.0)
    params = cx5_2022_params()
    steer_max = int(np.interp(6.0, params.STEER_MAX_LOOKUP[0], params.STEER_MAX_LOOKUP[1]))
    assert out >= -steer_max + (-self.ALLOWANCE + min(seq[-6:])) * self.MULTIPLIER

  def test_no_window_on_platforms_without_the_2022_eps(self):
    # pre-2022 params carry no STEER_DRIVER_SAMPLES, so the deque stays one deep and the
    # behavior is the single newest sample, exactly as before
    assert not hasattr(pre_2022_params(), 'STEER_DRIVER_SAMPLES')
