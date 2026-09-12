"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.params import Params, ParamKeyFlag
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.car.stock_ecu_handback import StockEcuHandBackGate


class HardwaredExt:
  """zoompilot's hooks into hardwared's hardware_thread, one object so hardwared.py carries one-line call sites.

  Ordering contract with hardware_thread: update runs at the top of every loop, before the
  onroad conditions are evaluated. It returns True on the loop that starts an onroad cycle.

  hardwared owns the requested started/offroad transitions. Two intents reach it:
  OnroadCycleRequested (upstream's) and OffroadModeRequested, the forced-offroad preference
  that the UI and remote settings write. OffroadMode itself is written here alone, because
  pandad reads it and drops the panda's ignition within 100 ms, before any hand-back could
  run. Entering waits on the stock ECU hand-back while onroad (stock_ecu_handback.py); a
  failed hand-back keeps the forced-offroad request open (the exit button withdraws it)
  rather than stopping on a timer.
  Exiting clears the previous session's CarParams and readiness first, so pandad sequences
  the fresh session like a boot (ELM327 until the new CarParams is ready) instead of
  applying the old safety and opening the relay seconds before controls come up.
  """

  def __init__(self, params: Params) -> None:
    self.params = params
    self.handback = StockEcuHandBackGate(params)

  def update(self, started: bool) -> bool:
    cycle = self.params.get_bool("OnroadCycleRequested")
    offroad_wanted = self.params.get_bool("OffroadModeRequested")
    offroad = self.params.get_bool("OffroadMode")
    if offroad and not offroad_wanted:
      self.on_offroad_exit()
    enter = offroad_wanted and not offroad
    if not (cycle or enter):
      self.handback.reset()
      return False
    # forced offroad has an exit button to withdraw it, so it may hold on a failed hand-back;
    # a cycle has no cancel and proceeds on any answer, like a reboot
    if not self.handback.ready(started, hold_on_failure=enter):
      return False
    if enter:
      cloudlog.warning("entering forced offroad")
      self.params.put_bool("OffroadMode", True, block=True)
    if cycle:
      self.params.put_bool("OnroadCycleRequested", False, block=True)
      self.prepare_onroad_entry()
    return cycle

  def on_offroad_exit(self) -> None:
    cloudlog.warning("leaving forced offroad")
    self.prepare_onroad_entry()
    self.params.put_bool("OffroadMode", False, block=True)

  def prepare_onroad_entry(self) -> None:
    # pandad races manager's onroad-transition param clearing on every software-initiated
    # entry (an onroad cycle, a forced-offroad exit). If it wins, it applies the previous
    # session's CarParams safety immediately and opens the harness relay seconds before
    # controls come up, cutting the camera off from the car long enough to fault it. Run the
    # same clear early so the new session sequences like a normal boot: ELM327 (relay closed)
    # until the fresh CarParams is ready.
    self.params.clear_all(ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION)
