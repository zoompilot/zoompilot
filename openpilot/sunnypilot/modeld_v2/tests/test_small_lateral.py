"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

import numpy as np
import pytest
from jetlink.openpilot.testing import FakeJoinedModel

from openpilot.cereal import log
from openpilot.sunnypilot.modeld_v2.small_lateral import SmallLateral, FADE

BIG_CURVATURE = 0.02
SPEED = 5.0


def lateral(small, speed=SPEED):
  sl = SmallLateral(small)
  sl.speed = speed
  return sl


class FakeSmall:
  chestnut = False
  CURVATURE = 0.1
  LAT_SMOOTH_SECONDS = 0.1
  PLANPLUS_CONTROL = 1.0
  constants = SimpleNamespace(DESIRE_LEN=8)
  desire_key = 'desire_pulse'
  vision_input_names = ['img', 'big_img']
  numpy_inputs = {'lateral_control_params': None}

  def __init__(self, fail=False):
    self.calls = []
    self.fail = fail
    self.resets = 0
    self.prev_desire = np.ones(8, np.float32)
    self.adapter = SimpleNamespace(reset_warmup_buffers=self._reset)

  def _reset(self):
    self.resets += 1

  def run(self, bufs, transforms, inputs, after_enqueue=None):
    self.calls.append((bufs, transforms, inputs))
    if self.fail:
      raise RuntimeError("boom")
    if after_enqueue is not None:
      after_enqueue()
    return {'out': len(self.calls)}

  def get_action_from_model(self, output, prev_action, lat_action_t, long_action_t, v_ego):
    self.action_t = (lat_action_t, long_action_t)
    return log.ModelDataV2.Action(desiredCurvature=self.CURVATURE, desiredAcceleration=-9.0)


def big(chestnut=True):
  return SimpleNamespace(chestnut=chestnut, LAT_SMOOTH_SECONDS=0.0, PLANPLUS_CONTROL=0.75, lat_delay=0.2)


def big_inputs():
  return {'desire': np.zeros(8, np.float32), 'traffic_convention': np.array([1, 0], np.float32),
          'action_t': np.array([0.3, 0.4], np.float32)}


def frame(sl, model, v_ego, then=None, desire=0):
  after = sl.after_enqueue(model, then, desire, ('main', 'extra'), ('t_main', 't_extra'), big_inputs(), v_ego)
  if after is not None:
    after()
  action = log.ModelDataV2.Action(desiredCurvature=BIG_CURVATURE, desiredAcceleration=0.5)
  sl.curvature(action, log.ModelDataV2.Action(), v_ego)
  return action


@pytest.mark.parametrize("v_ego, curvature, runs", [
  (0.0, FakeSmall.CURVATURE, True),
  (SPEED, FakeSmall.CURVATURE, True),
  (SPEED + FADE / 2, (FakeSmall.CURVATURE + BIG_CURVATURE) / 2, True),
  (SPEED + FADE, BIG_CURVATURE, False),
  (30.0, BIG_CURVATURE, False),
])
def test_small_curvature_below_speed_large_above(v_ego, curvature, runs):
  small = FakeSmall()
  action = frame(lateral(small), big(), v_ego)
  assert action.desiredCurvature == pytest.approx(curvature)
  assert action.desiredAcceleration == 0.5   # longitudinal stays the large model's
  assert len(small.calls) == runs            # only where it steers


def test_each_slow_stretch_starts_from_a_reset_history():
  small = FakeSmall()
  sl = lateral(small)
  for v_ego in (1.0, 1.0, 1.5, 10.0, 10.0, 1.0, 0.5):
    frame(sl, big(), v_ego)
  assert len(small.calls) == 5
  assert small.resets == 2
  assert not small.prev_desire.any()


def test_the_small_model_gets_its_own_slots_and_timing():
  small = FakeSmall()
  model = big()
  frame(lateral(small), model, 1.0, desire=3)
  bufs, transforms, inputs = small.calls[0]
  assert bufs == {'img': 'main', 'big_img': 'extra'}
  assert transforms == {'img': 't_main', 'big_img': 't_extra'}
  assert inputs[small.desire_key].tolist() == [0, 0, 0, 1, 0, 0, 0, 0]
  # the large model's lateral horizon, moved by the small bundle's own smoothing
  assert small.action_t == pytest.approx((0.3 + small.LAT_SMOOTH_SECONDS, 0.4))
  assert inputs['lateral_control_params'] == pytest.approx([1.0, model.lat_delay + small.LAT_SMOOTH_SECONDS])
  assert small.PLANPLUS_CONTROL == model.PLANPLUS_CONTROL


def test_nothing_without_a_large_model():
  small = FakeSmall()
  sl = lateral(small)

  def then():
    pass
  # the small model drives, or jetlink's joining model drives as it
  for model in (small, FakeJoinedModel(small)):
    assert sl.after_enqueue(model, then, 0, (None, None), (None, None), big_inputs(), 0.0) is then
  assert frame(lateral(None), big(), 0.0).desiredCurvature == pytest.approx(BIG_CURVATURE)


def test_the_chestnut_state_still_goes_out():
  calls = []
  frame(lateral(FakeSmall()), big(), 1.0, then=lambda: calls.append(1))
  assert calls == [1]


def test_a_hand_back_inside_run_does_not_reenter_the_small_model():
  # jetlink hands the frame back inside run() and runs the small model with this callback
  small = FakeSmall()
  sl = lateral(small)
  joined = FakeJoinedModel(small, chestnut=True)
  after = sl.after_enqueue(joined, None, 0, (None, None), (None, None), big_inputs(), 0.0)
  joined.chestnut = False
  joined.run({}, {}, {}, after)
  assert len(small.calls) == 1
  assert sl.output is None


def test_a_failing_small_model_leaves_the_large_one_steering():
  small = FakeSmall(fail=True)
  sl = lateral(small)
  assert frame(sl, big(), 0.5).desiredCurvature == pytest.approx(BIG_CURVATURE)
  assert frame(sl, big(), 0.5).desiredCurvature == pytest.approx(BIG_CURVATURE)
  assert len(small.calls) == 1   # tried once, then left alone


def test_off_never_runs_the_small_model():
  small = FakeSmall()
  assert frame(lateral(small, speed=0.0), big(), 0.0).desiredCurvature == pytest.approx(BIG_CURVATURE)
  assert small.calls == []
