"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature, MAX_LATERAL_JERK
from openpilot.sunnypilot.selfdrive.controls.lib.lane_change_smoothing import (
  LaneChangeSmoothing, level_jerk_factor, lane_change_time_extra, read_level,
  LEVELS, LEVEL_OFF, LEVEL_FAST, LEVEL_MEDIUM, LEVEL_EXTRA_SLOW, UNWIND_JERK_MAX, SMOOTH_RELEASE_T,
)

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

LANE_WIDTH = 3.5  # m


def make_cs(v_ego):
  return SimpleNamespace(vEgo=v_ego)


def make_model(state=LaneChangeState.off, direction=LaneChangeDirection.none):
  return SimpleNamespace(meta=SimpleNamespace(laneChangeState=state, laneChangeDirection=direction))


LEFT_STARTING = make_model(LaneChangeState.laneChangeStarting, LaneChangeDirection.left)
LEFT_FINISHING = make_model(LaneChangeState.laneChangeFinishing, LaneChangeDirection.left)
RIGHT_STARTING = make_model(LaneChangeState.laneChangeStarting, LaneChangeDirection.right)
OFF = make_model()


class TestLevelMapping:
  def test_levels_are_contiguous_indices(self):
    # the UIs store the position in their label lists, so the levels must be 0..n-1
    assert LEVELS == tuple(range(len(LEVELS)))
    assert LEVELS[0] == LEVEL_OFF

  def test_every_level_is_slower_than_stock(self):
    # the model's own lane change asks for 2-3 m/s^3 (0.4-0.65 of ISO); a cap above that
    # would not bind and the level would be stock in disguise
    assert level_jerk_factor(LEVEL_OFF) == 1.0
    factors = [level_jerk_factor(lv) for lv in LEVELS[1:]]
    assert all(0.0 < f <= 0.4 for f in factors)
    assert factors == sorted(factors, reverse=True)  # fast -> extra slow tightens
    assert level_jerk_factor(LEVEL_FAST) < UNWIND_JERK_MAX

  def test_time_extra_grows_with_gentleness(self):
    extras = [lane_change_time_extra(lv) for lv in LEVELS]
    assert extras == sorted(extras)
    assert extras[0] == 0.0

  def test_read_level_clamps(self):
    params = SimpleNamespace(get=lambda key, return_default=True: 99)
    assert read_level(params) == LEVEL_EXTRA_SLOW
    params = SimpleNamespace(get=lambda key, return_default=True: -3)
    assert read_level(params) == LEVEL_OFF


class TestLaneChangeSmoothing(OpenpilotTestCase):
  def setup_method(self):
    self.params = Params()
    self.params.put("LaneChangeSmoothing", LEVEL_MEDIUM, block=True)
    self.lcs = LaneChangeSmoothing()

  def update(self, model, new_k, prev_k, v_ego=15.0, lat_active=True):
    return self.lcs.update(make_cs(v_ego), model, lat_active, new_k, prev_k)

  def test_disabled_returns_stock(self):
    self.params.put("LaneChangeSmoothing", LEVEL_OFF, block=True)
    self.lcs.get_params()
    assert self.update(LEFT_STARTING, 0.002, 0.0) == 1.0

  def test_stock_outside_lane_change(self):
    assert self.update(OFF, 0.002, 0.0) == 1.0

  def test_clamped_during_maneuver(self):
    jf = self.update(LEFT_STARTING, 0.001, 0.0)
    assert jf == pytest.approx(self.lcs.set_jerk)
    assert jf < 0.2  # the default level is a real clamp

  def test_finishing_state_keeps_clamp(self):
    jf = self.update(LEFT_FINISHING, 0.001, 0.0)
    assert jf == pytest.approx(self.lcs.set_jerk)

  def test_lateral_inactive_resets(self):
    # disengaging mid-maneuver must not leave a clamp or an armed unwind for the re-engage
    self.update(LEFT_STARTING, 0.001, 0.0)
    assert self.update(LEFT_STARTING, 0.001, 0.0, lat_active=False) == 1.0
    assert self.lcs.release_timer == 0.0 and self.lcs.entry_sign == 0.0
    assert self.update(OFF, 0.001, 0.0) == 1.0

  def test_taper_back_to_stock_after_maneuver(self):
    self.update(LEFT_STARTING, 0.001, 0.0)
    jfs = [self.update(OFF, 0.0, 0.0) for _ in range(int(SMOOTH_RELEASE_T / DT_CTRL) + 10)]
    assert jfs[-1] == 1.0
    assert all(b >= a - 1e-9 for a, b in zip(jfs[:-1], jfs[1:], strict=True))  # monotonic release

  def test_unwind_tracks_the_model(self):
    # against the entry direction the factor follows the lag, up to the unwind cap
    self.update(LEFT_STARTING, 0.001, 0.0)
    small = self.update(LEFT_STARTING, -0.001, 0.0)
    large = self.update(LEFT_STARTING, -0.005, 0.0)
    assert self.lcs.set_jerk < small < large <= UNWIND_JERK_MAX + 1e-9

  def test_unwind_direction_follows_lane_change_direction(self):
    # a right lane change unwinds with positive curvature steps
    self.update(RIGHT_STARTING, -0.001, 0.0)
    assert self.update(RIGHT_STARTING, 0.005, 0.0) > self.lcs.set_jerk
    assert self.update(RIGHT_STARTING, -0.005, 0.0) == pytest.approx(self.lcs.set_jerk)

  def test_unwind_is_continuous_in_lag(self):
    # no bang-bang at the crest: a vanishing lag gets a vanishing boost
    self.update(LEFT_STARTING, 0.001, 0.0)
    assert self.update(LEFT_STARTING, -1e-6, 0.0) == pytest.approx(self.lcs.set_jerk, abs=1e-3)

  def test_unwind_stays_armed_through_release(self):
    self.update(LEFT_STARTING, 0.001, 0.0)
    for _ in range(10):
      self.update(OFF, 0.0, 0.0)
    assert self.update(OFF, -0.005, 0.0) > self.lcs.set_jerk

  def test_clip_curvature_scales_with_factor(self):
    full, _ = clip_curvature(15.0, 0.0, 0.01, 0.0)
    half, _ = clip_curvature(15.0, 0.0, 0.01, 0.0, jerk_factor=0.5)
    assert half == pytest.approx(full / 2)
    stock, _ = clip_curvature(15.0, 0.0, 0.01, 0.0, jerk_factor=1.0)
    assert stock == full


class TestClosedLoop(OpenpilotTestCase):
  """A toy lane change: the planner is a PD on the remaining lateral offset (closed loop on
  the car, like the model), the car follows the clipped curvature command, and the
  lane-change state machine flips on the lane line and at the new center."""

  V_EGO = 25.0
  A_MAX = 1.5  # m/s^2, lateral accel the planner is willing to use
  T_MAX = 20.0

  def simulate(self, smoothing: LaneChangeSmoothing | None):
    y, y_dot, k_cmd = 0.0, 0.0, 0.0
    state = LaneChangeState.laneChangeStarting
    jerks, t_done = [], None
    for i in range(int(self.T_MAX / DT_CTRL)):
      t = i * DT_CTRL
      err = LANE_WIDTH - y
      if state == LaneChangeState.laneChangeStarting and y > LANE_WIDTH / 2:
        state = LaneChangeState.laneChangeFinishing
      elif state == LaneChangeState.laneChangeFinishing and abs(err) < 0.15 and abs(y_dot) < 0.15:
        state = LaneChangeState.off
        t_done = t_done or t
      a_des = float(np.clip(1.0 * err - 1.8 * y_dot, -self.A_MAX, self.A_MAX))
      k_model = a_des / self.V_EGO ** 2
      jf = 1.0
      if smoothing is not None:
        jf = smoothing.update(make_cs(self.V_EGO), make_model(state, LaneChangeDirection.left), True, k_model, k_cmd)
      k_next, _ = clip_curvature(self.V_EGO, k_cmd, k_model, 0.0, jf)
      jerks.append(abs(k_next - k_cmd) / DT_CTRL * self.V_EGO ** 2)
      k_cmd = k_next
      y_dot += k_cmd * self.V_EGO ** 2 * DT_CTRL
      y += y_dot * DT_CTRL
    return y, max(jerks), t_done

  def setup_method(self):
    self.params = Params()

  def test_smoothed_lane_change_completes_without_snap(self):
    y_stock, jerk_stock, t_stock = self.simulate(None)
    assert t_stock is not None
    for level in LEVELS[1:]:
      self.params.put("LaneChangeSmoothing", level, block=True)
      lcs = LaneChangeSmoothing()
      y, jerk, t_done = self.simulate(lcs)
      assert t_done is not None, f"level {level} never settled"
      assert t_done >= t_stock
      assert abs(y - LANE_WIDTH) < 0.2, f"level {level} ended {y:.2f} m"
      assert jerk <= UNWIND_JERK_MAX * MAX_LATERAL_JERK + 1e-6
      assert jerk < jerk_stock
