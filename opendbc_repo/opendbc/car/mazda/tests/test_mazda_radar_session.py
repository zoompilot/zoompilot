"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The radar UDS session: RadarSessionManager's bounds on its own, then what goes on the bus in
each state driven through the real CarController.update_longitudinal.
"""
import pytest

from opendbc.car import DT_CTRL
from opendbc.car.mazda.radar_session import RADAR_SESSION_LIMIT_FRAMES, RADAR_RESTORE_FRAMES, RadarSessionManager, RadarSessionState
from opendbc.car.mazda.tests.conftest import (CRZ_CTRL, CRZ_INFO, RADAR_STATIC, RADAR_UDS, SESSION_DFLT_DAT, SESSION_PROG_DAT,
                                              TESTER_PRESENT_DAT, LongCtrlState, frames, step_long)
from opendbc.car.mazda.values import CarControllerParams
from opendbc.sunnypilot.car.stock_ecu import StockEcuState


class RadarSessionManager(RadarSessionManager):
  """Direct-driven manager: counts the controller frame the production caller passes in."""

  def __init__(self, **kwargs):
    super().__init__(**kwargs)
    self.n = -1

  def update(self, *args, **kwargs):
    self.n += 1
    return super().update(*args, frame=self.n, **kwargs)


class TestRadarSessionBounds:
  """Diagnostic attempts are bounded; completion requires observed stock recovery."""

  def test_silencing_gives_up_bounded(self):
    m = RadarSessionManager()
    for _ in range(RADAR_SESSION_LIMIT_FRAMES + 2):
      state = m.update(True, True, False, standstill=True, session_refused=False, stock_radar_gone=False)
    assert state == RadarSessionState.HANDBACK and m.silencing_failed
    for _ in range(CarControllerParams.RADAR_UDS_STEP + RADAR_RESTORE_FRAMES):
      m.update(True, True, False, standstill=True, session_refused=False, stock_radar_gone=False)
    # and stays given up for the drive: stock keeps the bus
    for _ in range(10):
      assert m.update(True, True, False, standstill=True, session_refused=False, stock_radar_gone=False) == RadarSessionState.STOCK

  def test_negative_response_gives_up_immediately(self):
    # route 000000fe t+15.0 shows the radar answers a session request within 10 ms, so a
    # negative response is definitive: no reason to burn the silence budget
    m = RadarSessionManager()
    m.update(True, True, False, standstill=True, session_refused=False, stock_radar_gone=False)
    assert m.state == RadarSessionState.SILENCING
    assert m.update(True, True, False, standstill=True, session_refused=True, stock_radar_gone=False) == RadarSessionState.HANDBACK
    assert m.silencing_failed

  def test_handback_stops_waiting_for_a_dead_radar(self):
    m = RadarSessionManager()
    m.update(True, False, False, standstill=True, session_refused=False, stock_radar_gone=True)
    assert m.state == RadarSessionState.SILENCED
    for _ in range(RADAR_SESSION_LIMIT_FRAMES + 2):
      state = m.update(True, False, True, standstill=True, session_refused=False, stock_radar_gone=True)
    assert state == RadarSessionState.HANDBACK
    assert m.handback_failed and not m.handback_completed
    assert m.diagnostic_message is None

  def test_ordered_handback_stays_stock_while_the_request_stands(self):
    # the lifecycle holds its assert for the whole stop: no re-silencing right before shutdown
    m = RadarSessionManager()
    m.update(True, False, False, standstill=True, session_refused=False, stock_radar_gone=True)
    assert m.state == RadarSessionState.SILENCED
    m.update(True, False, True, standstill=True, session_refused=False, stock_radar_gone=True)
    assert m.state == RadarSessionState.HANDBACK
    for _ in range(CarControllerParams.RADAR_UDS_STEP + RADAR_RESTORE_FRAMES):
      m.update(True, True, True, standstill=True, session_refused=False, stock_radar_gone=False)
    assert m.state == RadarSessionState.STOCK and m.handback_completed
    for alive in (True, False):
      for _ in range(5):
        assert m.update(True, alive, True, standstill=True, session_refused=False, stock_radar_gone=not alive) == RadarSessionState.STOCK
    assert m.status == StockEcuState.RESTORED

  def test_withdrawn_request_after_the_restore_is_a_fresh_start(self):
    # forced offroad cancelled once the radar was handed back: the next takeover is a first one
    m = RadarSessionManager()
    m.update(True, False, False, standstill=True, session_refused=False, stock_radar_gone=True)
    m.update(True, False, True, standstill=True, session_refused=False, stock_radar_gone=True)
    for _ in range(CarControllerParams.RADAR_UDS_STEP + RADAR_RESTORE_FRAMES):
      m.update(True, True, True, standstill=True, session_refused=False, stock_radar_gone=False)
    assert m.handback_completed
    assert m.update(True, True, False, standstill=True, session_refused=False, stock_radar_gone=False) == RadarSessionState.SILENCING
    assert not m.handback_completed

  def test_withdrawn_handback_finishes_restoration(self):
    # A reversal must finish restoration; card cycles using the latest toggle value.
    m = RadarSessionManager()
    m.update(True, False, False, standstill=True, session_refused=False, stock_radar_gone=True)
    m.update(True, False, True, standstill=True, session_refused=False, stock_radar_gone=True)
    assert m.state == RadarSessionState.HANDBACK
    state = m.update(True, False, False, standstill=True, session_refused=False, stock_radar_gone=True)
    assert state == RadarSessionState.HANDBACK and not m.handback_completed

  def test_silencing_waits_for_standstill_but_adoption_does_not(self):
    # actively silencing disables AEB, so it only starts pre-motion like disable_ecu;
    # adopting an already-quiet radar disables nothing and proceeds anywhere
    m = RadarSessionManager()
    for _ in range(10):
      assert m.update(True, True, False, standstill=False, session_refused=False, stock_radar_gone=False) == RadarSessionState.STOCK
    assert m.update(True, True, False, standstill=True, session_refused=False, stock_radar_gone=False) == RadarSessionState.SILENCING
    m2 = RadarSessionManager()
    assert m2.update(True, False, False, standstill=False, session_refused=False, stock_radar_gone=True) == RadarSessionState.SILENCED

  def test_a_dropped_stock_frame_is_not_adopted(self):
    # 7.25M stock CRZ_INFO frames: 9 inter-arrival gaps past the 50 ms alive window, 2 past
    # 100 ms (max 105.7 ms, one mid-drive at speed). Silence inside the guard window is a
    # dropped frame, not a torn-down radar: nothing goes on the bus, and no session request
    # either while the car is moving
    m = RadarSessionManager()
    for _ in range(int(CarControllerParams.STOCK_RADAR_GUARD_T / DT_CTRL)):
      assert m.update(True, False, False, standstill=False, session_refused=False, stock_radar_gone=False) == RadarSessionState.STOCK
    # the guard-long silence is the adoption
    assert m.update(True, False, False, standstill=False, session_refused=False, stock_radar_gone=True) == RadarSessionState.SILENCED

  def test_a_returned_radar_is_resilenced_under_the_teardown_gate(self):
    # a radar heard again under our synthetic frames: our frames stop at once (two masters),
    # and the session request follows the same pre-motion gate as the first teardown. Moving,
    # stock keeps the bus until the next stop
    m = RadarSessionManager()
    m.update(True, False, False, standstill=False, session_refused=False, stock_radar_gone=True)
    assert m.state == RadarSessionState.SILENCED
    assert m.update(True, True, False, standstill=False, session_refused=False, stock_radar_gone=False) == RadarSessionState.STOCK
    for _ in range(300):
      assert m.update(True, True, False, standstill=False, session_refused=False, stock_radar_gone=False) == RadarSessionState.STOCK
    assert m.update(True, True, False, standstill=True, session_refused=False, stock_radar_gone=False) == RadarSessionState.SILENCING
    while not m.programming_sent:
      m.update(True, True, False, standstill=True, session_refused=False, stock_radar_gone=False)
    # the handover back is on the alive window, not the guard: the radar stops within a frame
    # of accepting the session, and every frame in between is a radar gap for the camera
    assert m.update(True, False, False, standstill=True, session_refused=False, stock_radar_gone=False) == RadarSessionState.SILENCED
    # stopped when it comes back: re-requested right away, as before
    m2 = RadarSessionManager()
    m2.update(True, False, False, standstill=True, session_refused=False, stock_radar_gone=True)
    assert m2.update(True, True, False, standstill=True, session_refused=False, stock_radar_gone=False) == RadarSessionState.SILENCING


def boot_step(cc, cs, stock_radar_alive, fsc_settled, handback=False, cruise_engaged=False, standstill=True,
              stock_radar_gone=None):
  # standstill=True models the parked boot; actively silencing a live radar is gated on it
  return step_long(cc, cs, long_active=False, accel=0., long_state=LongCtrlState.off, lead_visible=False, available=False,
                   stock_radar_alive=stock_radar_alive, stock_radar_gone=stock_radar_gone, fsc_settled=fsc_settled,
                   handback=handback, cruise_engaged=cruise_engaged, standstill=standstill)


def uds(sends):
  return frames(sends, RADAR_UDS)


def synthetic(sends):
  return [a for a, _, _ in sends if a in (CRZ_INFO, CRZ_CTRL, RADAR_STATIC)]


class TestRadarSessionSequencing:
  """Boot teardown deferral and the ordered hand-back: what goes on the bus in each
  radar session state, driven through the real CarController.update_longitudinal."""

  @pytest.mark.parametrize("phase", range(CarControllerParams.RADAR_UDS_STEP))
  def test_cancel_inflight_teardown_restores_default_before_finishing(self, cc, cs, phase):
    assert SESSION_PROG_DAT in uds(boot_step(cc, cs, stock_radar_alive=True, fsc_settled=True))
    for _ in range(phase):
      boot_step(cc, cs, stock_radar_alive=True, fsc_settled=True)
    # The last radar frame remains fresh when the toggle flips. No synthetic traffic
    # may overlap it, but the outstanding programming request still needs undoing.
    saw_default = False
    for _ in range(CarControllerParams.RADAR_UDS_STEP + 2):
      sends = boot_step(cc, cs, stock_radar_alive=True, fsc_settled=True, handback=True)
      saw_default |= SESSION_DFLT_DAT in uds(sends)
      assert synthetic(sends) == []
      assert SESSION_PROG_DAT not in uds(sends)
    assert saw_default
    assert cc.radar_session.handback_completed
    # the request withdrawn after the restore (toggle flipped back): a fresh parked takeover
    resumed = False
    for _ in range(CarControllerParams.RADAR_UDS_STEP + 1):
      resumed |= SESSION_PROG_DAT in uds(boot_step(cc, cs, stock_radar_alive=True, fsc_settled=True))
    assert resumed

  def test_cancel_inflight_teardown_covers_late_silence(self, cc, cs):
    boot_step(cc, cs, stock_radar_alive=True, fsc_settled=True)
    boot_step(cc, cs, stock_radar_alive=True, fsc_settled=True, handback=True)
    saw_default = False
    saw_synthetic = False
    for _ in range(100):
      sends = boot_step(cc, cs, stock_radar_alive=False, fsc_settled=True, handback=True)
      saw_default |= SESSION_DFLT_DAT in uds(sends)
      saw_synthetic |= bool(synthetic(sends))
      assert TESTER_PRESENT_DAT not in uds(sends)
      assert SESSION_PROG_DAT not in uds(sends)
    assert saw_default and saw_synthetic
    assert cc.radar_session.state == RadarSessionState.HANDBACK
    assert boot_step(cc, cs, stock_radar_alive=True, fsc_settled=True, handback=True) == []

  def test_returned_radar_requires_setup_gate_again(self, cc, cs):
    for settled, engaged in ((False, False), (True, True)):
      boot_step(cc, cs, stock_radar_alive=False, fsc_settled=True)
      for _ in range(100):
        assert boot_step(cc, cs, stock_radar_alive=True, fsc_settled=settled, cruise_engaged=engaged) == []

  def test_stock_state_is_silent(self, cc, cs):
    # radar alive, gate not yet passed: nothing at all goes on the bus
    for _ in range(200):
      assert boot_step(cc, cs, stock_radar_alive=True, fsc_settled=False) == []

  def test_boot_teardown_sequence(self, cc, cs):
    # gate passes with the stock radar alive: programming-session requests at 2 Hz,
    # still no synthetic frames and no tester present
    for i in range(100):
      sends = boot_step(cc, cs, stock_radar_alive=True, fsc_settled=True)
      if i % CarControllerParams.RADAR_UDS_STEP == 0:
        assert uds(sends) == [SESSION_PROG_DAT]
      else:
        assert uds(sends) == []
      assert synthetic(sends) == []
    # radar goes quiet: synthetic frames + tester present take over, session requests stop
    saw_tester = False
    for _ in range(100):
      frame = cc.frame
      sends = boot_step(cc, cs, stock_radar_alive=False, fsc_settled=True)
      assert SESSION_PROG_DAT not in uds(sends)
      if frame % CarControllerParams.LONG_STEP == 0:
        assert len(synthetic(sends)) > 0
      saw_tester |= TESTER_PRESENT_DAT in uds(sends)
    assert saw_tester

  def test_handback_sequence(self, cc, cs):
    # reach SILENCED
    boot_step(cc, cs, stock_radar_alive=False, fsc_settled=True)
    # hand-back requested: default-session requests at 2 Hz, tester present stops,
    # synthetic frames continue while the radar is still quiet
    saw_default = False
    for _ in range(100):
      frame = cc.frame
      sends = boot_step(cc, cs, stock_radar_alive=False, fsc_settled=True, handback=True)
      assert TESTER_PRESENT_DAT not in uds(sends)
      saw_default |= SESSION_DFLT_DAT in uds(sends)
      if frame % CarControllerParams.LONG_STEP == 0:
        assert len(synthetic(sends)) > 0
    assert saw_default
    # stock radar returns: everything stops
    for _ in range(200):
      assert boot_step(cc, cs, stock_radar_alive=True, fsc_settled=True, handback=True) == []

  def test_handback_before_teardown_stops_everything(self, cc, cs):
    # toggle-off while still waiting on the gate: no session ever entered, so no
    # hand-back traffic either
    boot_step(cc, cs, stock_radar_alive=True, fsc_settled=False)
    for _ in range(120):
      assert boot_step(cc, cs, stock_radar_alive=True, fsc_settled=False, handback=True) == []

  def test_teardown_waits_for_stock_cruise_disengage(self, cc, cs):
    # driver engaged stock MRCC before the gate passed (warm boot): hold the teardown
    for _ in range(120):
      assert boot_step(cc, cs, stock_radar_alive=True, fsc_settled=True, cruise_engaged=True) == []
    # driver disengages: teardown proceeds
    cc.frame = 0
    sends = boot_step(cc, cs, stock_radar_alive=True, fsc_settled=True, cruise_engaged=False)
    assert SESSION_PROG_DAT in uds(sends)

  def test_completed_handback_stays_stock_while_asserted(self, cc, cs):
    # the hand-back server holds CC_SP.stockEcuHandBack for the whole stop; nothing goes on the
    # bus once stock traffic is back
    boot_step(cc, cs, stock_radar_alive=False, fsc_settled=True)
    boot_step(cc, cs, stock_radar_alive=False, fsc_settled=True, handback=True)
    for _ in range(CarControllerParams.RADAR_UDS_STEP + RADAR_RESTORE_FRAMES):
      boot_step(cc, cs, stock_radar_alive=True, fsc_settled=True, handback=True)
    assert cc.radar_session.handback_completed
    for _ in range(200):
      assert boot_step(cc, cs, stock_radar_alive=True, fsc_settled=True, handback=True) == []

  def test_s3_recovery_resilences(self, cc, cs):
    # radar reappears mid-drive (dropped tester present, S3 timeout): re-request the session
    boot_step(cc, cs, stock_radar_alive=False, fsc_settled=True)
    cc.frame = CarControllerParams.RADAR_UDS_STEP  # align to a session-request frame
    sends = boot_step(cc, cs, stock_radar_alive=True, fsc_settled=True)
    assert SESSION_PROG_DAT in uds(sends)
    # and settles back to silenced once quiet again
    sends = boot_step(cc, cs, stock_radar_alive=False, fsc_settled=True)
    assert SESSION_PROG_DAT not in uds(sends)

  def test_a_returned_radar_at_speed_sends_no_session_request(self, cc, cs):
    # radar heard again while rolling: synthetic frames and tester present stop, and the
    # programming-session request waits for a stop like the first teardown did
    boot_step(cc, cs, stock_radar_alive=False, fsc_settled=True, standstill=False)
    for _ in range(300):
      assert boot_step(cc, cs, stock_radar_alive=True, stock_radar_gone=False, fsc_settled=True, standstill=False) == []
    cc.frame = CarControllerParams.RADAR_UDS_STEP
    sends = boot_step(cc, cs, stock_radar_alive=True, stock_radar_gone=False, fsc_settled=True, standstill=True)
    assert uds(sends) == [SESSION_PROG_DAT]

  def test_a_stock_frame_gap_at_speed_puts_nothing_on_the_bus(self, cc, cs):
    # gate passed, car rolling, the stock radar drops a few frames (alive window expired,
    # guard window not): no synthetic frames, no tester present, no session request
    for _ in range(300):
      assert boot_step(cc, cs, stock_radar_alive=False, stock_radar_gone=False, fsc_settled=True, standstill=False) == []
    # the guard-long silence is the adoption
    cc.frame = 0
    sends = boot_step(cc, cs, stock_radar_alive=False, stock_radar_gone=True, fsc_settled=True, standstill=False)
    assert len(synthetic(sends)) > 0


def _stock(m, standstill, alive=True, refused=False, gate=True, handback=False, **kw):
  return m.update(gate, alive, handback, standstill=standstill, session_refused=refused, stock_radar_gone=not alive, **kw)


class TestMovingTakeover:
  """A fresh session started with the car rolling (forced offroad exit, process restart): a
  radar with a moving handover on record is requested at speed; every other radar waits for
  the stop, and the status says which."""

  def test_capable_radar_is_requested_while_moving(self):
    m = RadarSessionManager(moving_takeover=True)
    assert _stock(m, standstill=False) == RadarSessionState.SILENCING
    assert m.attempt_moving
    while not m.programming_sent:
      _stock(m, standstill=False)
    assert _stock(m, standstill=False, alive=False) == RadarSessionState.SILENCED
    assert m.status == StockEcuState.STARTING  # owned, guard not yet passed
    _stock(m, standstill=False, alive=False, owned=True)
    assert m.status == StockEcuState.READY

  def test_default_configuration_waits_for_the_stop(self):
    m = RadarSessionManager()
    for _ in range(300):
      assert _stock(m, standstill=False) == RadarSessionState.STOCK
    assert m.status == StockEcuState.PARK_TO_TAKE_OVER
    assert _stock(m, standstill=True) == RadarSessionState.SILENCING

  @pytest.mark.parametrize("how", ["refused", "timeout"])
  def test_moving_refusal_leaves_the_parked_attempt_open(self, how):
    # the radar saying no at speed says nothing about the parked path every configuration has
    # on record: no more moving attempts this session, the next stop is requested as usual
    m = RadarSessionManager(moving_takeover=True)
    _stock(m, standstill=False)
    if how == "refused":
      assert _stock(m, standstill=False, refused=True) == RadarSessionState.HANDBACK
    else:
      for _ in range(RADAR_SESSION_LIMIT_FRAMES + 1):
        state = _stock(m, standstill=False)
      assert state == RadarSessionState.HANDBACK
    assert not m.moving_open and not m.silencing_failed
    for _ in range(CarControllerParams.RADAR_UDS_STEP + RADAR_RESTORE_FRAMES):
      _stock(m, standstill=False)
    assert m.state == RadarSessionState.STOCK and not m.handback_completed
    for _ in range(200):
      assert _stock(m, standstill=False) == RadarSessionState.STOCK
    assert m.status == StockEcuState.PARK_TO_TAKE_OVER
    assert _stock(m, standstill=True) == RadarSessionState.SILENCING
    assert not m.attempt_moving

  def test_parked_refusal_is_definitive(self):
    m = RadarSessionManager(moving_takeover=True)
    _stock(m, standstill=True)
    _stock(m, standstill=True, refused=True)
    assert m.silencing_failed
    for _ in range(CarControllerParams.RADAR_UDS_STEP + RADAR_RESTORE_FRAMES + 50):
      _stock(m, standstill=True)
    for standstill in (True, False):
      assert _stock(m, standstill=standstill) == RadarSessionState.STOCK
    assert m.status == StockEcuState.FAILED

  def test_motion_change_mid_attempt(self):
    # capable: a parked attempt carries on when the car pulls away and a moving one when it
    # stops; default: pulling away undoes the queued request, as before
    m = RadarSessionManager(moving_takeover=True)
    _stock(m, standstill=True)
    while not m.programming_sent:
      _stock(m, standstill=True)
    assert _stock(m, standstill=False) == RadarSessionState.SILENCING
    m2 = RadarSessionManager(moving_takeover=True)
    _stock(m2, standstill=False)
    assert _stock(m2, standstill=True) == RadarSessionState.SILENCING
    m3 = RadarSessionManager()
    _stock(m3, standstill=True)
    while not m3.programming_sent:
      _stock(m3, standstill=True)
    assert _stock(m3, standstill=False) == RadarSessionState.HANDBACK

  def test_undone_request_leaves_the_next_attempt_open(self):
    # undoing our own request (motion on a parked-only radar) is not an ordered hand-back
    m = RadarSessionManager()
    _stock(m, standstill=True)
    while not m.programming_sent:
      _stock(m, standstill=True)
    _stock(m, standstill=False)
    for _ in range(CarControllerParams.RADAR_UDS_STEP + RADAR_RESTORE_FRAMES):
      _stock(m, standstill=False)
    assert m.state == RadarSessionState.STOCK and not m.handback_completed
    assert _stock(m, standstill=True) == RadarSessionState.SILENCING

  def test_returned_radar_is_only_resilenced_parked(self):
    # a radar heard again under our frames came back through its own S3 timeout; a moving
    # re-request is not on record for any configuration
    m = RadarSessionManager(moving_takeover=True)
    _stock(m, standstill=False, alive=False)
    assert m.state == RadarSessionState.SILENCED
    assert _stock(m, standstill=False) == RadarSessionState.STOCK
    for _ in range(200):
      assert _stock(m, standstill=False) == RadarSessionState.STOCK
    assert m.status == StockEcuState.PARK_TO_TAKE_OVER
    assert _stock(m, standstill=True) == RadarSessionState.SILENCING


class TestStockEcuStatus:
  """The transition contract: the driver's view of ownership, in place on the manager."""

  def test_starting_covers_every_prerequisite(self):
    m = RadarSessionManager(moving_takeover=True)
    _stock(m, standstill=True, gate=False)
    assert m.status == StockEcuState.STARTING
    _stock(m, standstill=True, bus_healthy=False)
    assert m.status == StockEcuState.STARTING
    _stock(m, standstill=True, stock_engaged=True)
    assert m.status == StockEcuState.STOCK_CRUISE_ON
    _stock(m, standstill=True)
    assert m.state == RadarSessionState.SILENCING and m.status == StockEcuState.STARTING

  def test_ready_restoring_restored(self):
    m = RadarSessionManager()
    _stock(m, standstill=True)
    while not m.programming_sent:
      _stock(m, standstill=True)
    _stock(m, standstill=True, alive=False, owned=True)
    assert m.status == StockEcuState.READY
    _stock(m, standstill=True, alive=False, handback=True)
    assert m.status == StockEcuState.RESTORING
    for _ in range(CarControllerParams.RADAR_UDS_STEP + RADAR_RESTORE_FRAMES):
      _stock(m, standstill=True, handback=True)
    assert (m.status, m.handback_completed) == (StockEcuState.RESTORED, True)

  def test_restore_timeout_is_a_failure_until_late_recovery(self):
    m = RadarSessionManager()
    _stock(m, standstill=True, alive=False)
    for _ in range(RADAR_SESSION_LIMIT_FRAMES + 2):
      _stock(m, standstill=True, alive=False, handback=True)
    assert (m.status, m.handback_failed) == (StockEcuState.FAILED, True)
    for _ in range(RADAR_RESTORE_FRAMES):
      _stock(m, standstill=True, handback=True)
    assert (m.status, m.handback_failed, m.handback_completed) == (StockEcuState.RESTORED, False, True)


class TestControllerStatus:
  def test_alpha_long_controller_publishes_the_contract(self, cc, cs):
    boot_step(cc, cs, stock_radar_alive=True, fsc_settled=False)
    assert cc.stock_ecu_state == StockEcuState.STARTING
    boot_step(cc, cs, stock_radar_alive=True, fsc_settled=True, cruise_engaged=True)
    assert cc.stock_ecu_state == StockEcuState.STOCK_CRUISE_ON
    boot_step(cc, cs, stock_radar_alive=True, fsc_settled=True)
    assert cc.radar_session.state == RadarSessionState.SILENCING

  def test_stock_long_controller_needs_nothing(self, stock_cc):
    assert stock_cc.stock_ecu_state == StockEcuState.NOT_NEEDED

  def test_ready_follows_carstate_guard(self, cc, cs):
    boot_step(cc, cs, stock_radar_alive=False, fsc_settled=True)
    assert cc.radar_session.state == RadarSessionState.SILENCED and cc.stock_ecu_state == StockEcuState.STARTING
    cs.radar_owned = True
    boot_step(cc, cs, stock_radar_alive=False, fsc_settled=True)
    assert cc.stock_ecu_state == StockEcuState.READY
