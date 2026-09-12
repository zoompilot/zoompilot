"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Lane-change smoothing: a user-selected lateral jerk limit on automatic lane changes.

Three rules, applied as a jerk factor on clip_curvature's curvature-rate limit:

1. Entry: while the model steers into the new lane, the curvature rate is capped at the
   selected level's fraction of the ISO limit.
2. Unwind: whenever the model moves curvature back against the entry direction (the
   arrest at the far side and the counter-steer that stops the lateral drift) the command
   is instead allowed to track the model as a first-order filter with UNWIND_TAU, up to
   UNWIND_JERK_MAX of stock. Only the jerk is tightened, never the lateral accel: capping
   accel strangles the arrest and lets the car glide past the new lane center before it
   can build enough counter-curvature. The tracker replaces a fixed fast unwind rate on
   purpose: the slow entry leaves the command lagging the model, and a fixed rate closes
   that lag as a step at the maneuver crest (a ~2 deg wheel snap in 0.2 s). A rate
   proportional to the lag crosses zero smoothly at the crest.
3. Release: both limits taper linearly back to stock over SMOOTH_RELEASE_T after the
   maneuver, so the final recenter is shaped instead of stepping through an open clamp.

The rate limiter sits downstream of a planner that is closed-loop on the car's lateral
position, so the model replans against it and the true maneuver duration is set by that
loop, not by the level's nominal time. Measure durations in logs.
"""
from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import MAX_LATERAL_JERK, MIN_SPEED

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

# Entry jerk cap per level as a fraction of the 5 m/s^3 ISO limit, each about half the
# last. Off is stock: the model's own lane change, which plans ~4 s and rides the ISO
# clamp. The model asks for 2-3 m/s^3 at its peak, so every level sits under that and
# all of them are longer than stock. Nominal sinusoid times at these caps (peak jerk
# j = pi^3 W / T^3 for W = 3.5 m, 1.3x headroom): fast ~4.7 s, medium ~5.8 s,
# slow ~6.9 s, extra slow ~8 s.
LEVEL_OFF, LEVEL_FAST, LEVEL_MEDIUM, LEVEL_SLOW, LEVEL_EXTRA_SLOW = 0, 1, 2, 3, 4
LEVEL_JERK_FACTOR = {
  LEVEL_OFF: 1.0,
  LEVEL_FAST: 0.30,
  LEVEL_MEDIUM: 0.15,
  LEVEL_SLOW: 0.08,
  LEVEL_EXTRA_SLOW: 0.05,
}
# seconds added to DesireHelper's lane-change timeout, so a gentle maneuver is not
# aborted mid-change by the stock cap
LEVEL_TIME_EXTRA = {
  LEVEL_OFF: 0.0,
  LEVEL_FAST: 0.5,
  LEVEL_MEDIUM: 1.0,
  LEVEL_SLOW: 2.0,
  LEVEL_EXTRA_SLOW: 3.0,
}
LEVELS = tuple(LEVEL_JERK_FACTOR)

# Taper of the limits back to stock after the maneuver.
SMOOTH_RELEASE_T = 2.0  # s

# Unwind tracker: rate = lag / UNWIND_TAU, capped at UNWIND_JERK_MAX of the ISO limit.
# Well above every entry cap, tracks the arrest demand seen in logs with margin.
UNWIND_TAU = 0.2        # s
UNWIND_JERK_MAX = 0.6

# Sign of the curvature that steers into the new lane (openpilot: left is positive).
ENTRY_SIGN = {LaneChangeDirection.left: 1.0, LaneChangeDirection.right: -1.0}


def read_level(params) -> int:
  """The clamped level setting, the single sanitization point for every consumer
  (controller, desire helper, settings badges)."""
  return min(max(int(params.get("LaneChangeSmoothing", return_default=True)), LEVELS[0]), LEVELS[-1])


def level_jerk_factor(level: int) -> float:
  return LEVEL_JERK_FACTOR[level]


def lane_change_time_extra(level: int) -> float:
  return LEVEL_TIME_EXTRA[level]


class LaneChangeSmoothing:
  """Stateful jerk-factor source for clip_curvature during automatic lane changes."""

  def __init__(self):
    self.params = Params()
    self.enabled = False
    self.set_jerk = 1.0
    self.release_timer = 0.0
    self.entry_sign = 0.0
    self.jerk_factor = 1.0
    self.get_params()

  def get_params(self) -> None:
    level = read_level(self.params)
    self.enabled = level != LEVEL_OFF
    self.set_jerk = level_jerk_factor(level)

  def reset(self) -> None:
    self.release_timer = 0.0
    self.entry_sign = 0.0
    self.jerk_factor = 1.0

  def update(self, CS, model_v2, lat_active: bool, new_desired_curvature: float, prev_desired_curvature: float) -> float:
    """Returns the jerk factor for clip_curvature (1.0 = stock limits)."""
    if not self.enabled or not lat_active:
      self.reset()
      return 1.0

    meta = model_v2.meta
    if meta.laneChangeState in (LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing):
      self.release_timer = SMOOTH_RELEASE_T
      self.entry_sign = ENTRY_SIGN.get(meta.laneChangeDirection, self.entry_sign)
    else:
      self.release_timer = max(self.release_timer - DT_CTRL, 0.0)

    if self.release_timer <= 0.0 or self.entry_sign == 0.0:
      self.reset()
      return 1.0

    release = 1.0 - self.release_timer / SMOOTH_RELEASE_T  # 0 in maneuver -> 1 at the end of the taper
    jerk_factor = self.set_jerk + (1.0 - self.set_jerk) * release

    lag = new_desired_curvature - prev_desired_curvature
    if lag * self.entry_sign < 0.0:
      v_ego = max(CS.vEgo, MIN_SPEED)
      track_factor = (abs(lag) / UNWIND_TAU) * v_ego ** 2 / MAX_LATERAL_JERK
      unwind_cap = UNWIND_JERK_MAX + (1.0 - UNWIND_JERK_MAX) * release
      jerk_factor = max(jerk_factor, min(unwind_cap, track_factor))

    self.jerk_factor = jerk_factor
    return jerk_factor
