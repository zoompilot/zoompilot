"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Set Speed Assist's approach pace: how hard the lift may pull toward the target speed, by speed and
personality. Its own module so the UI can read the ceiling without importing the controller.
"""
import numpy as np

from openpilot.cereal import log

# Stock ACC approach to the target speed, gentler than the cruise planner's: max accel by speed,
# scaled by the shortfall (docs/zoompilot/e2e-set-speed.md).
APPROACH_BP = [0., 4., 9., 14., 18., 25.]  # m/s
APPROACH_V = [1.5, 1.75, 1.45, 1.05, 0.85, 0.65]  # m/s^2
APPROACH_GAP_BP = [0., 1., 3., 5., 10.]  # m/s below the target speed
APPROACH_GAP_FRACTION = [0.38, 0.47, 0.57, 0.72, 1.]
# The table is Standard; the others move it by sunnypilot's accel boost step as of #2089
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
  if v_ego >= v_target:
    return 0.
  return approach_ceiling(v_ego, personality) * float(np.interp(v_target - v_ego, APPROACH_GAP_BP, APPROACH_GAP_FRACTION))
