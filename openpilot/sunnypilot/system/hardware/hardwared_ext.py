"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.params import Params, ParamKeyFlag


class HardwaredExt:
  """zoompilot's hooks into hardwared's hardware_thread, one object so hardwared.py carries one-line call sites.

  Ordering contract with hardware_thread: on_onroad_cycle runs inside the OnroadCycleRequested
  branch, after the request param is cleared.
  """

  def __init__(self, params: Params) -> None:
    self.params = params

  def on_onroad_cycle(self) -> None:
    # pandad races manager's onroad-transition param clearing when the cycle restarts.
    # If it wins, it applies the previous session's CarParams safety immediately and
    # opens the harness relay seconds before controls come up, cutting the camera off
    # from the car long enough to fault it. Run the same clear early so the new
    # session sequences like a normal boot: ELM327 (relay closed) until the fresh
    # CarParams is ready.
    self.params.clear_all(ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION)
