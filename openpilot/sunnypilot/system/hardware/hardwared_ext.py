"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.params import Params, ParamKeyFlag
from openpilot.sunnypilot.system.offroad_request import OffroadRequestGate


class HardwaredExt:
  """zoompilot's hooks into hardwared's hardware_thread, one object so hardwared.py carries one-line call sites.

  Ordering contract with hardware_thread: on_onroad_cycle runs inside the OnroadCycleRequested
  branch, after the request param is cleared; update runs once per loop, after sm.update and
  before the onroad conditions are evaluated. sm is hardwared's SubMaster and must carry SERVICE.
  """

  # extra SubMaster service: vEgo for the standstill side of the offroad-request grant
  SERVICE = "carState"

  def __init__(self, params: Params, rate_hz: float) -> None:
    self.params = params
    self.offroad_request_gate = OffroadRequestGate(rate_hz)

  def on_onroad_cycle(self) -> None:
    # pandad races manager's onroad-transition param clearing when the cycle restarts.
    # If it wins, it applies the previous session's CarParams safety immediately and
    # opens the harness relay seconds before controls come up, cutting the camera off
    # from the car long enough to fault it. Run the same clear early so the new
    # session sequences like a normal boot: ELM327 (relay closed) until the fresh
    # CarParams is ready.
    self.params.clear_all(ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION)

  def update(self, sm, session_active: bool, engaged: bool) -> bool:
    """Grants OffroadModeRequested when the gate allows it. Returns True on a grant."""
    # Force-offroad requests defer to card so brands that silence a stock ECU can hand
    # it back first (openpilot/sunnypilot/selfdrive/car/alpha_long_toggle.py). Grant
    # directly when there is no onroad session to hand back from, or if card has not
    # finished in time. Never grant a fallback while the car is moving.
    v_ego = sm[self.SERVICE].vEgo if sm.alive[self.SERVICE] else None
    if not self.offroad_request_gate.update(self.params.get_bool("OffroadModeRequested"), session_active, engaged, v_ego):
      return False
    self.params.put_bool("OffroadMode", True)
    self.params.put_bool("OffroadModeRequested", False)
    return True
