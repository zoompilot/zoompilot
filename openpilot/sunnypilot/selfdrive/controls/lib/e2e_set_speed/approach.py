"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Set Speed Assist's approach pace: how hard the lift may pull toward the target speed, by speed and
personality. Its own module so the UI can read the ceiling without importing the controller.
"""
import math

import numpy as np

from openpilot.cereal import log

# Stock ACC approach to the target speed, gentler than the cruise planner's: max accel by speed
# (docs/zoompilot/e2e-set-speed.md).
APPROACH_BP = [0., 4., 9., 14., 18., 25.]  # m/s
APPROACH_V = [1.5, 1.75, 1.45, 1.05, 0.85, 0.65]  # m/s^2
# Under the ceiling the pace is the acceleration that eases off at a steady jerk and arrives at the
# target with none left, sqrt(2 * jerk * gap). Within 2 * jerk (0.07 m/s at speed) of the target the gap
# itself is the pace, a 1 s line through the target that goes negative over it, so the floor settles
# on the target by itself: upstream's cruise candidate, which the planner also takes the minimum
# with, lags the gap by 1 s since #39266. At speed the jerk is the CX-5's stock MRCC, its median
# command with no lead by shortfall at 12-38 m/s: 0.32 m/s^2 1 m/s under, 0.40 at 2.2, 0.48 at 4,
# 0.5-0.7 at 7. In town it is brisker, as stock is, so pulling away from a stop is not held to the
# highway's pace.
APPROACH_JERK_BP = [8., 14.]  # m/s
APPROACH_JERK_V = [0.1, 0.035]  # m/s^3
# The ceiling is Standard; the others move it by sunnypilot's accel boost step as of #2089
# (docs/zoompilot/e2e-set-speed.md).
APPROACH_PERSONALITY_OFFSET = {
  log.LongitudinalPersonality.relaxed: -0.15,
  log.LongitudinalPersonality.standard: 0.,
  log.LongitudinalPersonality.aggressive: 0.15,
}  # m/s^2


def approach_ceiling(v_ego: float, personality: int) -> float:
  """The most the floor asks for at v_ego, a long way under the target. personality: the enum's raw
  value (a capnp enum read from a message compares equal to it but hashes differently)."""
  return float(np.interp(v_ego, APPROACH_BP, APPROACH_V) + APPROACH_PERSONALITY_OFFSET[personality])


def approach_accel(v_ego: float, v_target: float, personality: int) -> float:
  """Max floor accel while closing on v_target; the (negative) gap at or above it."""
  gap = v_target - v_ego
  jerk = float(np.interp(v_ego, APPROACH_JERK_BP, APPROACH_JERK_V))
  return min(math.sqrt(2. * jerk * max(gap, 0.)), gap, approach_ceiling(v_ego, personality))
