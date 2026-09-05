"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import time

from openpilot.cereal import log

# Hold ignition through short CAN gaps while the ignition line remains high. The five-second
# window tolerates panda's two-second CAN timeout without inheriting Mazda's roughly 30-second
# ignition-line delay after key-off.
IGNITION_CAN_DROP_HOLD_S = 5.0

# Each process tracks the monotonic time it last observed CAN ignition.
_ignition_can_last_seen: float | None = None


def get_ignition_state(panda_states, now: float | None = None) -> bool:
  global _ignition_can_last_seen
  if now is None:
    now = time.monotonic()

  valid = [ps for ps in panda_states if ps.pandaType != log.PandaState.PandaType.unknown]
  if not valid:
    _ignition_can_last_seen = None
    return False

  if any(ps.ignitionCan for ps in valid):
    _ignition_can_last_seen = now
    return True

  if not any(ps.ignitionLine for ps in valid):
    return False

  # Line-only cars follow the ignition line; CAN-capable cars use the bounded hold window.
  if _ignition_can_last_seen is None:
    return True
  return (now - _ignition_can_last_seen) < IGNITION_CAN_DROP_HOLD_S
