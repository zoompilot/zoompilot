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
# target with none left, sqrt(2 * jerk * gap): continuous through the target and 0 over it. Near
# the target upstream's cruise candidate, which the planner takes the minimum with, is the gentler
# of the two. The jerk matches the stock-derived table it replaced (0.38-1 of the ceiling over
# 0-10 m/s under) on the drives' frames 2 m/s or more under the target.
APPROACH_JERK = 0.065  # m/s^3
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
  """Max floor accel while closing on v_target; 0 at or above it."""
  return min(math.sqrt(2. * APPROACH_JERK * max(v_target - v_ego, 0.)), approach_ceiling(v_ego, personality))
