"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Onroad orchestration for the AlphaLongitudinalEnabled toggle.

The alpha-long param is read once at fingerprint, so applying a change requires an
onroad cycle. The UI only writes the param; card owns the cycle request so brands that
silence a stock ECU can hand it back first. pandad blocks TX within ~100 ms of
`started` dropping, so the hand-back must finish before the cycle is requested
(docs/mazda-alpha-long-setup-teardown.md).

The cycle also waits for the car to stand still: dropping the session mid-drive leaves
the radar to recover through its ~5 s S3 timeout in a degraded state that blocks cruise
until the next ignition. The UI only refuses the flip while engaged, so a toggle flip
while rolling would otherwise end the session at speed.

Mazda op-long hand-back: assert CarControlSP.stockEcuHandBack -> carcontroller stops
tester present and requests the radar's default session -> the stock radar's CRZ_INFO
returns (carstate raises accFaulted, its "stock radar heard" guard) -> take the action.
"""

from opendbc.car import DT_CTRL, structs
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

# Seconds before the monitor stops waiting on the radar's return and acts anyway; past the
# radar's ~5 s S3 self-recovery and inside the session manager's 10 s HANDBACK budget. The
# hand-back stays asserted after done, so this only decides when the cycle happens.
HANDBACK_TIMEOUT_T = 8.0
HANDBACK_TIMEOUT_FRAMES = int(HANDBACK_TIMEOUT_T / DT_CTRL)

STANDSTILL_V = 0.1   # m/s, below this the car counts as stopped
STANDSTILL_T = 0.5   # s the car has to stay below STANDSTILL_V before the cycle is acted on


class StandstillGate:
  """Debounced 'stopped for STANDSTILL_T' at a fixed update rate."""

  def __init__(self, rate_hz: float):
    self.frames_needed = max(1, int(round(STANDSTILL_T * rate_hz)))
    self.stopped_frames = 0

  def update(self, v_ego: float) -> bool:
    if v_ego < STANDSTILL_V:
      self.stopped_frames = min(self.stopped_frames + 1, self.frames_needed)
    else:
      self.stopped_frames = 0
    return self.stopped_frames >= self.frames_needed

  @property
  def stopped(self) -> bool:
    return self.stopped_frames >= self.frames_needed


class AlphaLongToggleMonitor:
  def __init__(self, CP: structs.CarParams, params: Params):
    self.CP = CP
    self.params = params
    self.toggle_enabled = CP.openpilotLongitudinalControl
    self.handback_frames = 0
    self.done = False
    self.repeat_logged = False
    self.standstill = StandstillGate(1 / DT_CTRL)
    # One cycle per ignition: the marker is CLEAR_ON_IGNITION_ON and card restarts on the
    # cycle, so a mismatch that survives the restart would otherwise cycle forever. Read
    # once; request_cycle is the only writer and it latches done.
    self.cycle_attempted = params.get_bool("AlphaLongCycleAttempted")
    if self.cycle_attempted and params.get_bool("AlphaLongitudinalEnabled") == CP.openpilotLongitudinalControl:
      # the cycle took; a later flip this ignition is a new request, not a persisting one
      params.put_bool("AlphaLongCycleAttempted", False)
      self.cycle_attempted = False

  def update_params(self) -> None:
    # called from card's 10 Hz params thread
    self.toggle_enabled = self.params.get_bool("AlphaLongitudinalEnabled")

  def request_cycle(self) -> None:
    self.params.put_bool("AlphaLongCycleAttempted", True)
    self.params.put_bool("OnroadCycleRequested", True)
    self.done = True

  def update(self, CS: structs.CarState, CC: structs.CarControl, CC_SP: structs.CarControlSP) -> None:
    """Runs at 100 Hz from controls_update, before CI.apply."""
    # tracked every frame so a flip made while parked is acted on at once
    stopped = self.standstill.update(CS.vEgo)
    if self.done:
      # Keep hand-back asserted because CC_SP is rebuilt each frame and the session manager
      # treats a cleared request as a new takeover.
      if self.handback_frames > 0:
        CC_SP.stockEcuHandBack = True
      return
    toggle_mismatch = self.CP.alphaLongitudinalAvailable and self.toggle_enabled != self.CP.openpilotLongitudinalControl
    if toggle_mismatch and self.cycle_attempted:
      if not self.repeat_logged:
        cloudlog.warning("alpha long toggle mismatch persists after this ignition's onroad cycle, not cycling again")
        self.repeat_logged = True
      toggle_mismatch = False
    if not toggle_mismatch:
      self.handback_frames = 0
      return

    # Wait for disengagement and standstill because parameters can change outside the UI.
    # Once started, hand-back remains asserted while the final cycle waits.
    if self.CP.brand != "mazda" or not self.CP.openpilotLongitudinalControl:
      # No ECU hand-back is required when enabling or on unaffected platforms.
      if not CC.enabled and stopped:
        self.request_cycle()
      return

    if CC.enabled and self.handback_frames == 0:
      return

    CC_SP.stockEcuHandBack = True
    self.handback_frames += 1
    # Under openpilot longitudinal, accFaulted also indicates restored stock radar traffic.
    if (CS.accFaulted or self.handback_frames >= HANDBACK_TIMEOUT_FRAMES) and not CC.enabled and stopped:
      self.request_cycle()
