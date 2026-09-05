"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Steer-limit classifier.

controlsd derives steer_limited_by_safety from |CC.actuators.torque - carOutput.torque| > 0.01,
and the lateral controller freezes its integrator on it and suppresses the saturation alert
while it is set. On a torque car with a rate-limited carcontroller that flag fires on ordinary
motion, so the integrator spends much of a drive frozen and an EPS pinned at its ceiling never
raises the alert. classify() splits the mismatch into rate-limited, at the EPS rail and
driver-limited, and hands the tunes a directional, rail-aware flag under the same name:

  limited = mismatch and not at_rail and deepening,  deepening = error_prev * pid.i >= 0

Rate limiting stays in the flag: while the command outruns the EPS slew the plant is not
following it, and integrating that error is actuator-rate windup. The freeze is directional so
decay toward a reversing error stays live, and the rail is carved out because the PID limits
already sit on it through LatControlTorqueExt (steer_max) and a False flag lets the tune's own
saturation test raise the alert. Numbers and the rejected variants: docs/zoompilot/lateral-tune.md.

Blind spot: a frame the panda rejects is still reported as delivered in carOutput, so a starved
EPS looks like a clean actuator here; the carcontroller's non-delivery latch covers that path.
"""
from typing import NamedTuple

MISMATCH_THRESHOLD = 1e-2  # controlsd.publish: abs(CC.actuators.torque - CO.actuatorsOutput.torque) > 1e-2
RAIL_EPS = 1e-3            # the tunes' own saturation test: steer_max - |output| < 1e-3
# the carcontroller reads the scale at vEgoRaw and rounds it, the classifier interpolates at
# vEgo, so accept a slightly short step
RATE_STEP_FRACTION = 0.9


class SteerLimit(NamedTuple):
  limited: bool         # what the tunes receive as steer_limited_by_safety
  driver_limited: bool
  rate_limited: bool
  at_rail: bool


CLEAN = SteerLimit(False, False, False, False)


def classify(cmd: float, applied: float, applied_prev: float | None, slew_up: float, slew_down: float,
             rail_scale: float, upstream_flag: bool, error_prev: float, integrator: float) -> SteerLimit:
  """Classify controlsd's steer-limit mismatch for one frame. All torques are normalized, in the
  actuator's sign convention (CC.actuators.torque and carOutput.actuatorsOutput.torque).

  cmd           this frame's commanded torque
  applied       the carcontroller's applied torque as of this frame
  applied_prev  the applied torque one frame earlier, None on the first frame after lateral
                became active (no history: limited and driver_limited fall back to upstream_flag)
  slew_up/down  the carcontroller's per-frame normalized rate limits at the current speed,
                up = growing in magnitude, down = shrinking
  rail_scale    the EPS ceiling as a fraction of scale at the current speed (1.0 when none)
  upstream_flag controlsd's own steer_limited_by_safety for this frame
  error_prev    the tune's pid_log.error from the frame just computed
  integrator    the tune's live pid.i
  """
  mag = abs(applied)
  at_rail = mag >= rail_scale - RAIL_EPS or mag >= 1.0 - RAIL_EPS
  if abs(cmd - applied) <= MISMATCH_THRESHOLD:
    return SteerLimit(False, False, False, at_rail)
  if applied_prev is None:
    return SteerLimit(bool(upstream_flag), bool(upstream_flag), False, at_rail)

  move = applied - applied_prev
  step = slew_up if mag > abs(applied_prev) else slew_down
  toward = move * (cmd - applied_prev) > 0.0
  # A full step: the rate limit is what holds the actuator back. A gap no wider than the move
  # it just made: the actuator is tracking a command that walks slower than the rate limit,
  # and the mismatch is only the one-frame lag between CC and carOutput.
  rate_limited = toward and (abs(move) >= RATE_STEP_FRACTION * step or abs(cmd - applied) <= abs(move) + MISMATCH_THRESHOLD)
  # freeze only integration that would deepen |i|; decay toward a reversing error stays live
  deepening = error_prev * integrator >= 0.0
  return SteerLimit(not at_rail and deepening, not rate_limited and not at_rail, rate_limited, at_rail)
