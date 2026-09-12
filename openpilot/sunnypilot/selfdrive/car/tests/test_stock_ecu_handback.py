"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs
from opendbc.sunnypilot.car.stock_ecu import StockEcuState
from openpilot.sunnypilot.selfdrive.car.stock_ecu_handback import (HANDBACK_WAIT_T, REQUEST_KEY, RESULT_KEY,
                                                                   StockEcuHandBackGate, StockEcuHandBackServer)
from openpilot.sunnypilot.selfdrive.car.tests.fakes import FakeClock, FakeParams, answer

RESTORED, FAILED, NOT_NEEDED = StockEcuState.RESTORED, StockEcuState.FAILED, StockEcuState.NOT_NEEDED


def _gate(**values):
  params = FakeParams(**values)
  clock = FakeClock()
  gate = StockEcuHandBackGate(params, now=clock)
  return gate, params, clock


class TestStockEcuHandBackGate:
  def test_offroad_is_always_ready_and_asks_nothing(self):
    gate, params, _ = _gate()
    assert gate.ready(started=False)
    assert params.get(REQUEST_KEY) is None

  def test_onroad_asks_once_then_waits_for_its_own_answer(self):
    gate, params, _ = _gate()
    assert not gate.ready(started=True)
    assert params.get(REQUEST_KEY) == {"id": 1}
    for _ in range(5):
      assert not gate.ready(started=True)
    assert params.get(REQUEST_KEY)["id"] == 1  # asked once
    answer(params, RESTORED, 7)  # somebody else's answer
    assert not gate.ready(started=True)
    answer(params, RESTORED)
    assert gate.ready(started=True)
    assert not gate.pending

  def test_stale_answer_from_the_previous_request_cannot_satisfy_a_new_one(self):
    gate, params, _ = _gate()
    gate.ready(started=True)
    answer(params, RESTORED)
    assert gate.ready(started=True)
    # the same consumer asks again later in the session: a fresh id, the old answer is not it
    assert not gate.ready(started=True)
    assert params.get(REQUEST_KEY)["id"] == 2
    assert not gate.ready(started=True)
    answer(params, NOT_NEEDED, 2)
    assert gate.ready(started=True)

  def test_second_consumer_joins_the_open_request(self):
    # a reboot pressed while forced offroad is already waiting: one hand-back, both proceed on it
    hw, params, _ = _gate()
    hw.ready(started=True)
    mgr = StockEcuHandBackGate(params, now=FakeClock())
    assert not mgr.ready(started=True)
    assert params.get(REQUEST_KEY) == {"id": 1}
    answer(params, RESTORED)
    assert hw.ready(started=True) and mgr.ready(started=True)

  def test_a_withdrawable_stop_holds_on_failure_until_late_recovery(self):
    gate, params, clock = _gate()
    gate.ready(started=True, hold_on_failure=True)
    answer(params, FAILED)
    assert not gate.ready(started=True, hold_on_failure=True)
    assert gate.failed and gate.pending
    clock.t = HANDBACK_WAIT_T * 3  # the no-answer bound does not apply: card answered
    assert not gate.ready(started=True, hold_on_failure=True)
    answer(params, RESTORED)
    assert gate.ready(started=True, hold_on_failure=True)

  def test_a_stop_with_no_cancel_proceeds_on_failure(self):
    gate, params, _ = _gate()
    gate.ready(started=True)
    answer(params, FAILED)
    assert gate.ready(started=True)

  def test_no_answer_at_all_is_bounded(self):
    # no card alive to answer: nothing holds the ECU, so nothing is gained by waiting
    for hold in (True, False):
      gate, params, clock = _gate()
      assert not gate.ready(started=True, hold_on_failure=hold)
      clock.t = HANDBACK_WAIT_T - 0.1
      assert not gate.ready(started=True, hold_on_failure=hold)
      clock.t = HANDBACK_WAIT_T + 0.1
      assert gate.ready(started=True, hold_on_failure=hold)

  def test_going_offroad_mid_wait_releases(self):
    gate, params, _ = _gate()
    assert not gate.ready(started=True)
    assert gate.ready(started=False)
    assert not gate.pending

  def test_withdrawal_removes_only_its_own_request(self):
    gate, params, _ = _gate()
    gate.ready(started=True)
    gate.reset()
    assert params.get(REQUEST_KEY) is None and not gate.pending and not gate.failed
    # a request opened by someone else stays
    gate.ready(started=True)
    params.put(REQUEST_KEY, {"id": 5})
    gate.reset()
    assert params.get(REQUEST_KEY)["id"] == 5

  def test_re_ask_after_withdrawal_gets_a_new_id(self):
    gate, params, _ = _gate()
    gate.ready(started=True, hold_on_failure=True)
    answer(params, FAILED)
    gate.ready(started=True, hold_on_failure=True)
    gate.reset()
    assert not gate.ready(started=True)
    assert params.get(REQUEST_KEY)["id"] == 2
    assert not gate.ready(started=True)  # the failed answer for id 1 is not an answer for id 2


def _server(request_id=1):
  params = FakeParams()
  if request_id is not None:
    params.put(REQUEST_KEY, {"id": request_id})
  server = StockEcuHandBackServer(params)
  server.update_params()
  return server, params


def _step(server, state=StockEcuState.STARTING, enabled=False):
  cc_sp = structs.CarControlSP()
  server.update(enabled, state, cc_sp)
  return cc_sp


class TestStockEcuHandBackServer:
  def test_nothing_to_hand_back_answers_not_needed_at_once(self):
    server, params = _server()
    cc_sp = _step(server, NOT_NEEDED)
    assert not cc_sp.stockEcuHandBack
    assert params.get(RESULT_KEY) == {"id": 1, "outcome": "notNeeded"}

  def test_nothing_requested_nothing_asserted(self):
    server, params = _server(request_id=None)
    for _ in range(10):
      assert not _step(server).stockEcuHandBack
    assert params.get(RESULT_KEY) is None

  def test_asserts_until_the_session_restores_then_answers(self):
    server, params = _server()
    for state in (StockEcuState.READY, StockEcuState.RESTORING, StockEcuState.RESTORING):
      assert _step(server, state).stockEcuHandBack
      assert params.get(RESULT_KEY) is None
    assert _step(server, RESTORED).stockEcuHandBack
    assert params.get(RESULT_KEY)["outcome"] == "restored"
    # the assert holds while the request stands
    for _ in range(10):
      assert _step(server, RESTORED).stockEcuHandBack

  def test_waits_for_disengagement_to_start_then_runs_to_the_end(self):
    server, params = _server()
    for _ in range(20):
      assert not _step(server, StockEcuState.READY, enabled=True).stockEcuHandBack
    assert _step(server, StockEcuState.READY, enabled=False).stockEcuHandBack
    assert _step(server, RESTORED, enabled=True).stockEcuHandBack
    assert params.get(RESULT_KEY)["outcome"] == "restored"

  def test_failure_then_late_recovery_are_two_answers(self):
    server, params = _server()
    _step(server, StockEcuState.RESTORING)
    _step(server, FAILED)
    assert params.get(RESULT_KEY) == {"id": 1, "outcome": "failed"}
    params.put(RESULT_KEY, None)  # a consumer would not clear it; make the re-answer observable
    for _ in range(5):
      _step(server, FAILED)
    assert params.get(RESULT_KEY) is None  # answered once per outcome
    _step(server, RESTORED)
    assert params.get(RESULT_KEY)["outcome"] == "restored"

  def test_new_request_id_is_answered_again(self):
    server, params = _server()
    _step(server, RESTORED)
    assert params.get(RESULT_KEY)["id"] == 1
    params.put(REQUEST_KEY, {"id": 2})
    server.update_params()
    _step(server, RESTORED)
    assert params.get(RESULT_KEY)["id"] == 2

  def test_withdrawn_request_drops_the_assert(self):
    # the record gone is the withdrawal: the vehicle's session manager finishes any in-flight
    # restoration on its own and treats the next takeover as a first one
    server, params = _server()
    assert _step(server, StockEcuState.READY).stockEcuHandBack
    params.remove(REQUEST_KEY)
    server.update_params()
    assert not _step(server, StockEcuState.RESTORING).stockEcuHandBack
    assert not server.started
