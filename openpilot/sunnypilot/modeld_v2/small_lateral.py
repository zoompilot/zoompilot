"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The small model's curvature at low speed while a large model drives.

A large model's lateral is unstable below about 2 m/s (upstream stops steering
there for it). The small model, loaded anyway as the large one's fallback,
steers there instead, below `speed` (the SmallModelSteering param, m/s, 0
for never). It runs beside the large one only below speed + FADE,
from a reset history, as at a modeld start: its first frames fall in the fade,
where they weigh little. It runs in the large model's after_enqueue, while the
chestnut or the Jetson works. Only desiredCurvature is taken: acceleration,
the plan and everything else published stay the large model's.
"""
import numpy as np

from openpilot.common.swaglog import cloudlog

FADE = 0.5  # m/s above the speed over which the large model takes the curvature back


class SmallLateral:
  def __init__(self, small):
    self.small = small
    self.output = None
    self.action_t = (0.0, 0.0)
    self.running = False
    self.speed = 0.0  # m/s, at and below which the small model steers; 0 never

  def runs_beside(self, model, v_ego: float) -> bool:
    """A large model drives slowly enough to want the small one beside it."""
    return (self.small is not None and model is not self.small and bool(model.chestnut) and
            self.speed > 0 and v_ego < self.speed + FADE)

  def after_enqueue(self, model, then, desire: int, bufs, transforms, inputs: dict[str, np.ndarray], v_ego: float):
    """model.run's after_enqueue for this frame: `then`, then the small model
    on the frame the large one got, `bufs` and `transforms` as (main, extra)."""
    self.output = None
    if not self.runs_beside(model, v_ego):
      self.running = False
      return then

    def run():
      if then is not None:
        then()
      # again at the call: jetlink may hand the frame back to the small model
      # inside run(), and would call this from the small model's own run
      if not self.runs_beside(model, v_ego):
        return
      try:
        if not self.running:
          self.small.adapter.reset_warmup_buffers()
          self.small.prev_desire[:] = 0
          self.running = True
        self.output = self.small.run(*self._inputs(model, desire, bufs, transforms, inputs, v_ego))
      except Exception:
        cloudlog.exception("small model failed beside the large one, large model steers at low speed")
        self.small = None
    return run

  def _inputs(self, model, desire, bufs, transforms, inputs, v_ego):
    """The large model's inputs in the small model's slots, timed by its own
    bundle's lateral smoothing."""
    small = self.small
    small.PLANPLUS_CONTROL = model.PLANPLUS_CONTROL
    smoothing = small.LAT_SMOOTH_SECONDS - model.LAT_SMOOTH_SECONDS
    self.action_t = (float(inputs['action_t'][0]) + smoothing, float(inputs['action_t'][1]))
    vec_desire = np.zeros(small.constants.DESIRE_LEN, dtype=np.float32)
    if 0 <= desire < small.constants.DESIRE_LEN:
      vec_desire[desire] = 1
    small_inputs = {
      small.desire_key: vec_desire,
      'traffic_convention': inputs['traffic_convention'],
      'action_t': np.array(self.action_t, dtype=np.float32),
    }
    if 'lateral_control_params' in small.numpy_inputs:
      small_inputs['lateral_control_params'] = np.array([v_ego, model.lat_delay + small.LAT_SMOOTH_SECONDS], dtype=np.float32)
    return ({name: bufs[1] if 'big' in name else bufs[0] for name in small.vision_input_names},
            {name: transforms[1] if 'big' in name else transforms[0] for name in small.vision_input_names},
            small_inputs)

  def curvature(self, action, prev_action, v_ego: float) -> None:
    """Give the large model's action the small model's curvature below the speed."""
    if self.output is None:
      return
    weight = float(np.interp(v_ego, [self.speed, self.speed + FADE], [1.0, 0.0]))
    if weight == 0.0:
      return
    small = self.small.get_action_from_model(self.output, prev_action, *self.action_t, v_ego)
    action.desiredCurvature = weight * small.desiredCurvature + (1.0 - weight) * action.desiredCurvature
