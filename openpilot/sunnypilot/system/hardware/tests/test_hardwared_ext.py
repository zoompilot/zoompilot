"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from openpilot.common.params import ParamKeyFlag
from openpilot.sunnypilot.selfdrive.car.stock_ecu_handback import HANDBACK_WAIT_T, REQUEST_KEY
from openpilot.sunnypilot.selfdrive.car.tests.fakes import FakeClock, FakeParams, answer
from openpilot.sunnypilot.system.hardware import hardwared_ext
from openpilot.sunnypilot.system.hardware.hardwared_ext import ACCELERATOR_OFF_WAIT_T, HardwaredExt


def _ext(**values):
  params = FakeParams(**values)
  ext = HardwaredExt(params)
  clock = FakeClock()
  ext.handback.now = clock
  return ext, params, clock


class TestOnroadCycle:
  def test_clears_onroad_transition_params(self):
    params = FakeParams()
    HardwaredExt(params).prepare_onroad_entry()
    assert params.cleared == [ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION]

  def test_nothing_requested(self):
    ext, params, _ = _ext()
    assert not ext.update(started=True)
    assert params.get(REQUEST_KEY) is None

  def test_offroad_cycle_needs_no_handback(self):
    ext, params, _ = _ext(OnroadCycleRequested=True)
    assert ext.update(started=False)
    assert not params.get_bool("OnroadCycleRequested")
    assert params.get(REQUEST_KEY) is None
    assert params.cleared == [ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION]

  def test_onroad_cycle_waits_for_the_handback(self):
    ext, params, _ = _ext(OnroadCycleRequested=True)
    assert not ext.update(started=True)
    assert params.get(REQUEST_KEY) == {"id": 1}
    assert params.get_bool("OnroadCycleRequested")  # still pending, not consumed
    assert not ext.update(started=True)
    answer(params, "restored")
    assert ext.update(started=True)
    assert not params.get_bool("OnroadCycleRequested")
    assert params.cleared == [ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION]

  def test_onroad_cycle_gives_up_waiting_with_no_card(self):
    ext, params, clock = _ext(OnroadCycleRequested=True)
    assert not ext.update(started=True)
    clock.t = HANDBACK_WAIT_T + 1
    assert ext.update(started=True)
    assert not params.get_bool("OnroadCycleRequested")

  def test_failed_handback_does_not_hold_the_cycle(self):
    # a cycle has no cancel control, so like a reboot it proceeds on any answer
    ext, params, _ = _ext(OnroadCycleRequested=True)
    ext.update(started=True)
    answer(params, "failed")
    assert ext.update(started=True)
    assert not params.get_bool("OnroadCycleRequested")


class TestForcedOffroad:
  def test_request_is_applied_at_once_when_offroad(self):
    ext, params, _ = _ext(OffroadModeRequested=True)
    assert not ext.update(started=False)  # not a cycle
    assert params.get_bool("OffroadMode")
    assert params.get_bool("OffroadModeRequested")  # the preference stays; it is the state

  def test_entering_waits_for_the_handback_when_onroad(self):
    ext, params, _ = _ext(OffroadModeRequested=True)
    assert not ext.update(started=True)
    assert not params.get_bool("OffroadMode")
    assert params.get(REQUEST_KEY) == {"id": 1}
    answer(params, "restored")
    assert not ext.update(started=True)
    assert params.get_bool("OffroadMode")
    assert params.cleared == []

  def test_entering_holds_on_a_failed_handback(self):
    # the radar never came back: stopping now leaves the camera without radar frames, so the
    # request stays open and visible; the user withdraws it or a late recovery completes it
    ext, params, clock = _ext(OffroadModeRequested=True)
    ext.update(started=True)
    answer(params, "failed")
    clock.t = HANDBACK_WAIT_T * 2
    for _ in range(5):
      assert not ext.update(started=True)
    assert ext.handback.failed and not params.get_bool("OffroadMode")
    answer(params, "restored")
    ext.update(started=True)
    assert params.get_bool("OffroadMode")

  def test_withdrawn_request_resets_the_wait(self):
    ext, params, clock = _ext(OffroadModeRequested=True)
    ext.update(started=True)
    params.put_bool("OffroadModeRequested", False)
    ext.update(started=True)
    assert not ext.handback.pending
    assert params.get(REQUEST_KEY) is None
    assert not params.get_bool("OffroadMode")

  def test_exit_clears_the_old_session_before_pandad_sees_ignition(self):
    # nothing cleared CarParams / ControlsReady while the device sat forced offroad with the
    # ignition on: pandad would apply the old safety and open the relay seconds before controls
    ext, params, _ = _ext(OffroadMode=True, OffroadModeRequested=False)
    assert not ext.update(started=False)
    assert params.cleared == [ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION]
    assert not params.get_bool("OffroadMode")
    assert params.get(REQUEST_KEY) is None  # no hand-back on the way out: nothing was taken over

  def test_steady_states_do_nothing(self):
    for offroad in (True, False):
      ext, params, _ = _ext(OffroadMode=offroad, OffroadModeRequested=offroad)
      assert not ext.update(started=not offroad)
      assert params.cleared == [] and params.get(REQUEST_KEY) is None
      assert params.get_bool("OffroadMode") == offroad

  def test_alpha_edits_while_forced_offroad_touch_nothing(self):
    ext, params, _ = _ext(OffroadMode=True, OffroadModeRequested=True, AlphaLongitudinalEnabled=True)
    for _ in range(5):
      params.put_bool("AlphaLongitudinalEnabled", not params.get_bool("AlphaLongitudinalEnabled"))
      assert not ext.update(started=False)
    assert params.cleared == [] and params.get(REQUEST_KEY) is None and params.get_bool("OffroadMode")


class FakeAccelerator:
  """The adapter's power-off hooks and offroad reason, as HardwaredExt sees them."""

  def __init__(self, pending: bool = True, why: str | None = None):
    self.pending = pending
    self.why = why
    self.requests: list[str] = []

  def request_shutdown(self, reason: str) -> bool:
    self.requests.append(reason)
    return self.pending

  def shutdown_pending(self) -> bool:
    return self.pending

  def reason(self) -> str | None:
    return self.why


def _power_off(monkeypatch, **kwargs):
  ext, _, clock = _ext()
  ext.now = clock
  jetlink = FakeAccelerator(**kwargs)
  monkeypatch.setattr(hardwared_ext, "jetlink_adapter", jetlink)
  return ext, jetlink, clock


class TestPowerOff:
  """A shutdown asks the accelerator once and DoShutdown waits until it took the request or
  ACCELERATOR_OFF_WAIT_T has passed; meanwhile powering_off holds any new drive back."""

  def test_no_shutdown_asks_nothing(self, monkeypatch):
    ext, jetlink, _ = _power_off(monkeypatch)
    assert not any(ext.shutdown_ready(False, 12.0) for _ in range(3))
    assert jetlink.requests == [] and not ext.powering_off

  def test_nothing_to_ask_goes_down_at_once(self, monkeypatch):
    ext, jetlink, _ = _power_off(monkeypatch, pending=False)
    assert ext.shutdown_ready(True, 12.0)
    assert jetlink.requests == ["comma shutting down, offroad since 12.0"]

  def test_it_asks_once_and_goes_down_when_the_request_is_taken(self, monkeypatch):
    ext, jetlink, clock = _power_off(monkeypatch)
    assert not ext.shutdown_ready(True, 12.0)
    assert ext.powering_off
    clock.t = 5.0
    assert not ext.shutdown_ready(False, 12.0)  # once asked, the power monitor is not asked again
    jetlink.pending = False
    assert ext.shutdown_ready(False, 12.0)
    assert len(jetlink.requests) == 1

  def test_nobody_taking_it_costs_the_wait_and_no_more(self, monkeypatch):
    ext, _, clock = _power_off(monkeypatch)
    assert not ext.shutdown_ready(True, 12.0)
    clock.t = ACCELERATOR_OFF_WAIT_T - 0.1
    assert not ext.shutdown_ready(True, 12.0)
    clock.t = ACCELERATOR_OFF_WAIT_T
    assert ext.shutdown_ready(True, 12.0)

  def test_the_unavailable_alert_follows_the_reason(self, monkeypatch):
    ext, jetlink, _ = _power_off(monkeypatch)
    shown = []
    for why in (None, "no Jetson on USB"):
      jetlink.why = why
      ext.alerts(lambda *args, **kwargs: shown.append((*args, kwargs["extra_text"])))
    assert shown == [("Offroad_AcceleratorUnavailable", False, None), ("Offroad_AcceleratorUnavailable", True, "no Jetson on USB")]
