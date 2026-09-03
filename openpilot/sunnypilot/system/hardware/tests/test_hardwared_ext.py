"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

from openpilot.common.params import ParamKeyFlag
from openpilot.sunnypilot.system.hardware.hardwared_ext import HardwaredExt
from openpilot.sunnypilot.system.offroad_request import OFFROAD_REQUEST_TIMEOUT, STANDSTILL_T

RATE = 10.0  # hardwared runs its loop at the pandaStates rate
TIMEOUT_FRAMES = int(OFFROAD_REQUEST_TIMEOUT * RATE)
STOP_FRAMES = int(STANDSTILL_T * RATE)


class FakeParams:
  def __init__(self, **bools):
    self.bools = dict(bools)
    self.cleared: list[ParamKeyFlag] = []

  def get_bool(self, key):
    return self.bools.get(key, False)

  def put_bool(self, key, value, block=False):
    self.bools[key] = value

  def clear_all(self, flag):
    self.cleared.append(flag)


class FakeSubMaster:
  """Only what the ext reads: sm[SERVICE].vEgo and sm.alive[SERVICE]."""

  def __init__(self, v_ego=0.0, alive=True):
    self.v_ego = v_ego
    self.alive = {HardwaredExt.SERVICE: alive}

  def __getitem__(self, service):
    assert service == HardwaredExt.SERVICE
    return SimpleNamespace(vEgo=self.v_ego)


def _ext(requested=True):
  params = FakeParams(OffroadModeRequested=requested)
  return HardwaredExt(params, RATE), params


def _run(ext, n, v_ego=0.0, session_active=True, engaged=False, alive=True):
  granted = False
  sm = FakeSubMaster(v_ego, alive)
  for _ in range(n):
    granted = ext.update(sm, session_active, engaged)
  return granted


class TestOnroadCycle:
  def test_service_is_carstate(self):
    assert HardwaredExt.SERVICE == "carState"

  def test_clears_onroad_transition_params(self):
    ext, params = _ext(requested=False)
    ext.on_onroad_cycle()
    assert params.cleared == [ParamKeyFlag.CLEAR_ON_ONROAD_TRANSITION]


class TestOffroadRequestGrant:
  def test_no_request_no_grant(self):
    ext, params = _ext(requested=False)
    assert not _run(ext, TIMEOUT_FRAMES * 2)
    assert "OffroadMode" not in params.bools

  def test_no_session_grants_at_once_even_moving(self):
    # nothing to hand back from and nothing rolling that openpilot owns: parked device, remote or local request
    ext, params = _ext()
    assert _run(ext, 1, v_ego=20.0, session_active=False)
    assert params.bools["OffroadMode"] is True
    assert params.bools["OffroadModeRequested"] is False

  def test_session_waits_for_card_then_grants_stopped(self):
    ext, params = _ext()
    assert not _run(ext, TIMEOUT_FRAMES - 1)
    assert "OffroadMode" not in params.bools
    assert _run(ext, 1)
    assert params.bools["OffroadMode"] is True
    assert params.bools["OffroadModeRequested"] is False

  def test_session_timeout_never_grants_moving(self):
    ext, params = _ext()
    assert not _run(ext, TIMEOUT_FRAMES * 3, v_ego=12.0)
    assert "OffroadMode" not in params.bools
    # stops: the standstill debounce still applies
    assert not _run(ext, STOP_FRAMES - 1)
    assert _run(ext, 1)

  def test_engaged_holds(self):
    ext, _ = _ext()
    assert not _run(ext, TIMEOUT_FRAMES * 2, engaged=True)
    assert _run(ext, 1)

  def test_dead_carstate_counts_as_stopped(self):
    # Without card, no openpilot control process can be driving the car.
    ext, _ = _ext()
    assert _run(ext, TIMEOUT_FRAMES, v_ego=30.0, alive=False)

  def test_dropped_request_restarts_the_timeout(self):
    ext, params = _ext()
    _run(ext, TIMEOUT_FRAMES - 1)
    params.put_bool("OffroadModeRequested", False)
    _run(ext, 1)
    params.put_bool("OffroadModeRequested", True)
    assert not _run(ext, TIMEOUT_FRAMES - 1)
    assert _run(ext, 1)

  def test_grant_clears_the_request_so_it_fires_once(self):
    ext, params = _ext()
    assert _run(ext, TIMEOUT_FRAMES)
    params.bools["OffroadMode"] = False  # as if the session ended and the user came back onroad
    assert not _run(ext, TIMEOUT_FRAMES * 2)
    assert params.bools["OffroadMode"] is False
