"""Mazda radar diagnostic-session ownership, independent of the acceleration controller."""
from enum import StrEnum

from opendbc.car import DT_CTRL, make_tester_present_msg, uds
from opendbc.car.can_definitions import CanData
from opendbc.car.carlog import carlog
from opendbc.car.mazda.values import CarControllerParams
from opendbc.sunnypilot.car.stock_ecu import StockEcuState

RADAR_ADDR = 0x764
RADAR_BUS = 0
RADAR_SESSION_LIMIT_FRAMES = int(CarControllerParams.RADAR_SESSION_LIMIT_T / DT_CTRL)
# Require more than one fresh stock frame before allowing a process restart.
RADAR_RESTORE_FRAMES = 2 * round(CarControllerParams.STOCK_RADAR_ALIVE_T / DT_CTRL)


def create_radar_session_msg(session_type: int) -> CanData:
  return CanData(RADAR_ADDR, bytes([2, uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, session_type, 0, 0, 0, 0, 0]), RADAR_BUS)


class RadarSessionState(StrEnum):
  STOCK = "stock"
  SILENCING = "silencing"
  SILENCED = "silenced"
  HANDBACK = "handback"


class RadarSessionManager:
  """Keep request scheduling, radar ownership and restoration results in one place.

  Silence is evidence only while the vehicle bus is healthy. A requested teardown
  uses the short traffic window; adoption without a request uses the longer guard.
  A restoration timeout is a failure, never proof that stock control recovered.

  The first takeover of a session may run while moving on a configuration validated for
  it (`moving_takeover`). A moving request the radar refuses or never answers, or a radar
  heard again under our frames (its own S3 recovery), closes moving attempts for the
  session and leaves the parked attempt open, the path every configuration has on record.
  A refusal at a stop ends the episode for the drive.

  An ordered hand-back (the lifecycle's request) keeps the radar stock for as long as the
  request stands; a withdrawn request is a fresh start under the normal takeover gate.
  Undoing our own unanswered or refused request latches nothing.
  """

  def __init__(self, moving_takeover: bool = False):
    self.moving_open = moving_takeover  # a moving request may still be made this session
    self.state = RadarSessionState.STOCK
    self.state_frames = 0
    self.silencing_failed = False
    self.handback_completed = False
    self.handback_failed = False
    self.programming_sent = False
    self.programming_confirmed = False
    self.default_sent = False
    self.default_confirmed = False
    self.stock_frames = 0
    self.replacement_active = False
    self.diagnostic_message: CanData | None = None
    self.attempt_moving = False
    self.status = StockEcuState.STARTING

  def _transition(self, state: RadarSessionState, reason: str) -> None:
    if state != self.state:
      carlog.info({"event": "mazdaRadarSession", "from": self.state, "to": state, "reason": reason,
                   "programmingConfirmed": self.programming_confirmed, "defaultConfirmed": self.default_confirmed})
      self.state = state
      self.state_frames = 0
      if state == RadarSessionState.SILENCING:
        self.programming_sent = self.programming_confirmed = False
      elif state == RadarSessionState.HANDBACK:
        self.default_sent = self.default_confirmed = False
        self.stock_frames = 0

  def _close_moving(self, reason: str) -> None:
    if self.moving_open:
      carlog.warning({"event": "mazdaRadarMovingTakeoverClosed", "reason": reason})
    self.moving_open = False

  def _silencing_gave_up(self, reason: str) -> None:
    # A moving attempt that the radar refused or never answered says nothing about the parked
    # path, which every configuration has on record: keep that open. Parked, it is definitive.
    if self.attempt_moving:
      self._close_moving(reason)
    else:
      self.silencing_failed = True
    self._transition(RadarSessionState.HANDBACK, f"programming {reason}")

  def update(self, gate_passed: bool, stock_radar_alive: bool, handback: bool,
             standstill: bool, session_refused: bool, stock_radar_gone: bool, *,
             bus_healthy: bool = True, session_response: int = 0, frame: int = 0,
             stock_engaged: bool = False, owned: bool = False) -> RadarSessionState:
    # the controller's frame is the one clock for every cadence, the UDS schedule included
    self.frame = frame
    self.diagnostic_message = None
    self.state_frames += 1
    self.stock_frames = self.stock_frames + 1 if bus_healthy and stock_radar_alive else 0
    if self.state == RadarSessionState.SILENCING and self.programming_sent and session_response == uds.SESSION_TYPE.PROGRAMMING:
      self.programming_confirmed = True
    if self.state == RadarSessionState.HANDBACK and self.default_sent and session_response == uds.SESSION_TYPE.DEFAULT:
      self.default_confirmed = True

    if handback:
      if self.state in (RadarSessionState.SILENCING, RadarSessionState.SILENCED):
        self._transition(RadarSessionState.HANDBACK, "requested")
      elif self.state == RadarSessionState.STOCK and not self.handback_completed:
        if not self.programming_sent and bus_healthy and stock_radar_alive:
          self.handback_completed = self.stock_frames >= RADAR_RESTORE_FRAMES
        else:
          self._transition(RadarSessionState.HANDBACK, "restore uncertain ownership")
    else:
      # a withdrawn stop: the next takeover is a first one again
      self.handback_completed = False

    # the takeover gate: the camera's boot check done and no stock engagement to pull the radar from under
    gate_open = gate_passed and not stock_engaged

    # A withdrawn request does not leave HANDBACK: the in-flight default request completes
    # before another programming request, so a toggle reversal never alternates sessions.
    if self.state == RadarSessionState.HANDBACK:
      # A response alone cannot prove that periodic radar traffic has resumed.
      if self.default_sent and self.stock_frames >= RADAR_RESTORE_FRAMES:
        # An ordered hand-back keeps the radar stock while the request stands. Undoing our
        # own unanswered or refused request does not: the next attempt is still open.
        self.handback_completed = handback
        self.handback_failed = False
        self._transition(RadarSessionState.STOCK, "stock traffic restored")
      elif self.state_frames >= RADAR_SESSION_LIMIT_FRAMES and not self.handback_failed:
        self.handback_failed = True
        carlog.error({"event": "mazdaRadarRestoreFailed", "reason": "stock traffic did not recover"})
      # After the bounded request budget, stop diagnostics but continue neutral replacement
      # traffic while quiet. A late stock recovery can still complete the handback.
      if self.state == RadarSessionState.HANDBACK and not self.handback_failed and \
         (not self.default_sent or not stock_radar_alive) and self.frame % CarControllerParams.RADAR_UDS_STEP == 0:
        self.diagnostic_message = create_radar_session_msg(uds.SESSION_TYPE.DEFAULT)
        self.default_sent = True
    elif not handback:
      if self.state == RadarSessionState.SILENCED and stock_radar_alive:
        self._close_moving("stock radar returned")
        self._transition(RadarSessionState.STOCK, "stock radar returned")
      takeover_allowed = standstill or self.moving_open

      if self.state == RadarSessionState.STOCK and gate_open and bus_healthy and not self.silencing_failed:
        if stock_radar_gone:
          self._transition(RadarSessionState.SILENCED, "adopt quiet radar on live bus")
        elif takeover_allowed and stock_radar_alive:
          self.attempt_moving = not standstill
          self._transition(RadarSessionState.SILENCING, "moving takeover" if self.attempt_moving else "parked takeover")

      if self.state == RadarSessionState.SILENCING:
        if session_refused:
          self._silencing_gave_up("refused")
        elif not bus_healthy or not gate_open or not takeover_allowed:
          # A request may already have been queued: undo it instead of abandoning it.
          self._transition(RadarSessionState.HANDBACK if self.programming_sent else RadarSessionState.STOCK,
                           "takeover prerequisites lost")
        elif self.programming_sent and not stock_radar_alive:
          self._transition(RadarSessionState.SILENCED, "requested radar silence")
        elif self.state_frames >= RADAR_SESSION_LIMIT_FRAMES:
          self._silencing_gave_up("timed out")
        elif self.frame % CarControllerParams.RADAR_UDS_STEP == 0:
          self.diagnostic_message = create_radar_session_msg(uds.SESSION_TYPE.PROGRAMMING)
          self.programming_sent = True

      if self.state == RadarSessionState.SILENCED and self.frame % CarControllerParams.RADAR_UDS_STEP == 0:
        self.diagnostic_message = make_tester_present_msg(RADAR_ADDR, RADAR_BUS, suppress_response=True)

    self._update_replacement(stock_radar_alive, bus_healthy, stock_radar_gone)
    self._update_status(gate_passed, stock_engaged, standstill, owned)
    return self.state

  def _update_replacement(self, stock_radar_alive: bool, bus_healthy: bool, stock_radar_gone: bool) -> None:
    # Synthetic radar frames while the stock radar is quiet and ours: held through a bus
    # blip in HANDBACK, since a gap for the camera is worse than a late stop.
    if stock_radar_alive or self.state == RadarSessionState.STOCK:
      self.replacement_active = False
    elif self.state == RadarSessionState.SILENCED or \
         (self.state == RadarSessionState.HANDBACK and bus_healthy and (self.programming_sent or stock_radar_gone)):
      self.replacement_active = True

  def _update_status(self, gate_passed: bool, stock_engaged: bool, standstill: bool, owned: bool) -> None:
    """The driver's view. `owned` is carstate's silence guard on the owned radar."""
    if self.state == RadarSessionState.HANDBACK:
      self.status = StockEcuState.FAILED if self.handback_failed else StockEcuState.RESTORING
    elif self.state == RadarSessionState.SILENCED:
      self.status = StockEcuState.READY if owned else StockEcuState.STARTING
    elif self.handback_completed:
      self.status = StockEcuState.RESTORED
    elif self.silencing_failed:
      self.status = StockEcuState.FAILED
    elif self.state == RadarSessionState.STOCK and gate_passed:
      self.status = StockEcuState.STOCK_CRUISE_ON if stock_engaged else \
                    StockEcuState.PARK_TO_TAKE_OVER if not standstill and not self.moving_open else StockEcuState.STARTING
    else:
      self.status = StockEcuState.STARTING
