"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The stock ECU hand-back: one correlated request/result pair between whoever ends the onroad
state and card, which owns the vehicle's diagnostic session.

Any software-initiated end of the onroad state (onroad cycle for a calibration reset or a
restart-needed toggle, forced offroad, reboot, shutdown, uninstall) kills TX within about
100 ms of `started` dropping. A brand that silences a stock ECU for the drive needs those
processes alive long enough to hand it back first, or the ECU recovers unattended through its
S3 timeout in a degraded state that blocks cruise until the next ignition.

Consumer (`StockEcuHandBackGate`, in hardwared and manager): writes StockEcuHandBackRequest
{id} and holds its action until StockEcuHandBackResult carries the same id. A second consumer
arriving while one request is open joins it, so exactly one hand-back is ever in flight.
Producer (`StockEcuHandBackServer`, in card): asserts the brand's hand-back off the control loop
while the request stands and writes the result with the vehicle's StockEcuState once it is one
of restored, failed or notNeeded. Ids are monotonic past every record on file and both records
clear on the offroad transition, so a stale answer never satisfies a new request, and a
withdrawn request (the record removed) is a fresh start for the vehicle.

A stop that can be withdrawn (forced offroad, through its exit button) proceeds only on
restored or notNeeded and stays open on failed: the vehicle keeps neutral replacement traffic
going and a late recovery still completes it. A stop with no cancel (cycle, reboot, shutdown,
uninstall) proceeds on any answer. Either proceeds after HANDBACK_WAIT_T with no answer at all,
which means no card is alive to give one. Ignition off and power loss cannot be held.
"""
import time

from opendbc.sunnypilot.car.stock_ecu import StockEcuState
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

REQUEST_KEY = "StockEcuHandBackRequest"
RESULT_KEY = "StockEcuHandBackResult"

# The producer's own budget is one UDS attempt (10 s) plus the restore window; past this with no
# answer at all the processes are assumed dead, and the stop goes ahead regardless.
HANDBACK_WAIT_T = 15.0

SETTLED = (StockEcuState.RESTORED, StockEcuState.NOT_NEEDED)
ANSWERS = (*SETTLED, StockEcuState.FAILED)


def read_record(params: Params, key: str) -> dict | None:
  rec = params.get(key)
  return rec if isinstance(rec, dict) and isinstance(rec.get("id"), int) else None


class StockEcuHandBackGate:
  """Consumer side. Call ready() every loop while the stop is wanted, reset() if it is withdrawn."""

  def __init__(self, params: Params, now=time.monotonic) -> None:
    self.params = params
    self.now = now
    self.request_id: int | None = None
    self.requested_ts: float | None = None
    self.failed = False

  @property
  def pending(self) -> bool:
    return self.request_id is not None

  def _open(self) -> None:
    existing = read_record(self.params, REQUEST_KEY)
    result = read_record(self.params, RESULT_KEY)
    settled = result is not None and existing is not None and result["id"] == existing["id"] and result.get("outcome") in SETTLED
    if existing is not None and not settled:
      # one hand-back in flight: join it rather than queue another behind it
      self.request_id = existing["id"]
    else:
      self.request_id = max([rec["id"] for rec in (existing, result) if rec is not None] + [0]) + 1
      self.params.put(REQUEST_KEY, {"id": self.request_id}, block=True)
    self.requested_ts = self.now()
    cloudlog.warning(f"stock ECU hand-back {self.request_id} requested before ending onroad")

  def _close(self) -> None:
    self.request_id = None
    self.requested_ts = None
    self.failed = False

  def ready(self, started: bool, hold_on_failure: bool = False) -> bool:
    """True when the caller may end the onroad state now; the wait is cleared with it.
    hold_on_failure: the stop can be withdrawn, so a failed hand-back holds it open."""
    if not started:
      self._close()
      return True
    if self.request_id is None:
      self._open()
      return False
    result = read_record(self.params, RESULT_KEY)
    if result is not None and result["id"] == self.request_id:
      outcome = result.get("outcome")
      if outcome in SETTLED:
        self._close()
        return True
      if outcome == StockEcuState.FAILED:
        if not self.failed:
          cloudlog.error(f"stock ECU hand-back {self.request_id} failed")
        self.failed = True
        if not hold_on_failure:
          self._close()
          return True
        return False  # held open; a late recovery or a withdrawal ends it
    if self.now() - self.requested_ts > HANDBACK_WAIT_T:
      cloudlog.error(f"stock ECU hand-back {self.request_id} unanswered, ending onroad anyway")
      self._close()
      return True
    return False

  def reset(self) -> None:
    if self.request_id is not None:
      # withdraw only a request nobody else is waiting on; card sees the record vanish
      existing = read_record(self.params, REQUEST_KEY)
      if existing is not None and existing["id"] == self.request_id:
        self.params.remove(REQUEST_KEY)
    self._close()


class StockEcuHandBackServer:
  """Producer side, on card's control loop."""

  def __init__(self, params: Params) -> None:
    self.params = params
    self.request: dict | None = None
    self.started = False       # asserted once: held for as long as the request stands
    self.answered: tuple[int, StockEcuState] | None = None

  def update_params(self) -> None:
    # rides card's 10 Hz params thread
    self.request = read_record(self.params, REQUEST_KEY)

  def _answer(self, request_id: int, outcome: StockEcuState) -> None:
    if self.answered == (request_id, outcome):
      return
    self.params.put(RESULT_KEY, {"id": request_id, "outcome": str(outcome)})
    self.answered = (request_id, outcome)
    (cloudlog.error if outcome == StockEcuState.FAILED else cloudlog.warning)(f"stock ECU hand-back {request_id} answered {outcome}")

  def update(self, enabled: bool, state: StockEcuState, CC_SP) -> None:
    """Runs at 100 Hz before CI.apply with the controller's stock ECU state. CC_SP is rebuilt
    every frame, so the assert is re-applied here; the session manager treats a dropped assert
    as a withdrawal."""
    if self.request is None:
      self.started = False
      return
    request_id = self.request["id"]
    if state == StockEcuState.NOT_NEEDED:
      self._answer(request_id, state)
      return
    # Never start on an engaged car: the hand-back revokes availability under the driver.
    # Once started it runs to its end.
    if not self.started and enabled:
      return
    self.started = True
    CC_SP.stockEcuHandBack = True
    if state in ANSWERS:
      self._answer(request_id, state)
