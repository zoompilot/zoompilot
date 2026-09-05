"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The radar UDS session: RadarSessionManager's bounds on its own, then what goes on the bus in
each state driven through the real CarController.update_longitudinal.
"""
from opendbc.car import DT_CTRL
from opendbc.car.mazda.longitudinal import RADAR_SESSION_LIMIT_FRAMES, RadarSessionManager, RadarSessionState
from opendbc.car.mazda.tests.conftest import (CRZ_CTRL, CRZ_INFO, RADAR_STATIC, RADAR_UDS, SESSION_DFLT_DAT, SESSION_PROG_DAT,
                                              TESTER_PRESENT_DAT, LongCtrlState, frames, step_long)
from opendbc.car.mazda.values import CarControllerParams


class TestRadarSessionBounds:
  """The fire-and-forget UDS session has no readable NRC, so every episode is bounded the
  way disable_ecu bounds its retries."""

  def test_silencing_gives_up_bounded(self):
    m = RadarSessionManager()
    for _ in range(RADAR_SESSION_LIMIT_FRAMES + 2):
      state = m.update(True, True, False, standstill=True, session_refused=False, stock_radar_gone=False)
    assert state == RadarSessionState.STOCK and m.silencing_failed
    # and stays given up for the drive: stock keeps the bus
    for _ in range(10):
      assert m.update(True, True, False, standstill=True, session_refused=False, stock_radar_gone=False) == RadarSessionState.STOCK

  def test_negative_response_gives_up_immediately(self):
    # route 000000fe t+15.0 shows the radar answers a session request within 10 ms, so a
    # negative response is definitive: no reason to burn the silence budget
    m = RadarSessionManager()
    m.update(True, True, False, standstill=True, session_refused=False, stock_radar_gone=False)
    assert m.state == RadarSessionState.SILENCING
    assert m.update(True, True, False, standstill=True, session_refused=True, stock_radar_gone=False) == RadarSessionState.STOCK
    assert m.silencing_failed

  def test_handback_stops_waiting_for_a_dead_radar(self):
    m = RadarSessionManager()
    m.update(True, False, False, standstill=True, session_refused=False, stock_radar_gone=True)
    assert m.state == RadarSessionState.SILENCED
    for _ in range(RADAR_SESSION_LIMIT_FRAMES + 2):
      state = m.update(True, False, True, standstill=True, session_refused=False, stock_radar_gone=True)
    assert state == RadarSessionState.STOCK

  def test_completed_handback_never_resilences(self):
    # the parked toggle-off regression: the monitor's CC_SP assert used to drop after its done
    # latch, the manager read that as a withdrawal, fell to STOCK, and re-entered SILENCING on
    # the same call (parked, gate still passed) -- re-silencing the radar it had just handed
    # back, right before shutdown, leaving it to a degraded unattended S3 recovery
    m = RadarSessionManager()
    m.update(True, False, False, standstill=True, session_refused=False, stock_radar_gone=True)
    assert m.state == RadarSessionState.SILENCED
    m.update(True, False, True, standstill=True, session_refused=False, stock_radar_gone=True)
    assert m.state == RadarSessionState.HANDBACK
    assert m.update(True, True, True, standstill=True, session_refused=False, stock_radar_gone=False) == RadarSessionState.STOCK
    for handback in (True, False):
      for alive in (True, False):
        for _ in range(5):
          assert m.update(True, alive, handback, standstill=True, session_refused=False, stock_radar_gone=not alive) == RadarSessionState.STOCK

  def test_withdrawn_handback_allows_retakeover(self):
    # only a hand-back that ran to completion latches: a genuine toggle-flip-back
    # mid-hand-back gets the normal takeover again
    m = RadarSessionManager()
    m.update(True, False, False, standstill=True, session_refused=False, stock_radar_gone=True)
    m.update(True, False, True, standstill=True, session_refused=False, stock_radar_gone=True)
    assert m.state == RadarSessionState.HANDBACK
    state = m.update(True, False, False, standstill=True, session_refused=False, stock_radar_gone=True)
    assert state == RadarSessionState.SILENCED and not m.handback_completed

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

  def test_completed_handback_stays_stock_after_the_assert_drops(self, cc, cs):
    # CC_SP is rebuilt every frame, so once the toggle monitor's done latch stops asserting
    # the hand-back the manager sees handback=False; a completed hand-back must not turn
    # into a fresh takeover on the very next frame (parked => standstill, gate still passed)
    boot_step(cc, cs, stock_radar_alive=False, fsc_settled=True)
    boot_step(cc, cs, stock_radar_alive=False, fsc_settled=True, handback=True)
    boot_step(cc, cs, stock_radar_alive=True, fsc_settled=True, handback=True)
    for _ in range(200):
      assert boot_step(cc, cs, stock_radar_alive=True, fsc_settled=True, handback=False) == []

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
