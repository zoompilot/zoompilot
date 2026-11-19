"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from types import SimpleNamespace

import pytest

from openpilot.cereal import log
from openpilot.sunnypilot.common import ignition
from openpilot.sunnypilot.common.ignition import get_ignition_state, IGNITION_CAN_DROP_HOLD_S

PandaType = log.PandaState.PandaType


def ps(can=False, line=False, panda_type=PandaType.tres):
  return SimpleNamespace(pandaType=panda_type, ignitionCan=can, ignitionLine=line)


@pytest.fixture(autouse=True)
def _fresh_state():
  ignition._ignition_can_last_seen = None
  yield
  ignition._ignition_can_last_seen = None


class TestIgnition:
  def test_can_on(self):
    assert get_ignition_state([ps(can=True)], now=0.0)
    assert get_ignition_state([ps(can=True, line=True)], now=1.0)

  def test_can_drop_with_line_high_holds_then_releases(self):
    assert get_ignition_state([ps(can=True, line=True)], now=10.0)
    for dt in (0.0, 1.0, IGNITION_CAN_DROP_HOLD_S - 0.1):
      assert get_ignition_state([ps(can=False, line=True)], now=10.0 + dt)
    assert not get_ignition_state([ps(can=False, line=True)], now=10.0 + IGNITION_CAN_DROP_HOLD_S)
    # Mazda: line stays high ~30 s after key-off, it must not bring ignition back
    assert not get_ignition_state([ps(can=False, line=True)], now=35.0)

  def test_can_drop_with_line_low_is_off_immediately(self):
    assert get_ignition_state([ps(can=True, line=True)], now=10.0)
    assert not get_ignition_state([ps(can=False, line=False)], now=10.0)

  def test_line_only_car_follows_the_line(self):
    assert get_ignition_state([ps(line=True)], now=0.0)
    assert get_ignition_state([ps(line=True)], now=100.0)
    assert not get_ignition_state([ps(line=False)], now=101.0)
    assert get_ignition_state([ps(line=True)], now=102.0)

  def test_no_valid_panda_is_off_and_resets(self):
    assert get_ignition_state([ps(can=True, line=True)], now=0.0)
    assert not get_ignition_state([ps(can=True, line=True, panda_type=PandaType.unknown)], now=0.5)
    assert not get_ignition_state([], now=0.6)
    assert ignition._ignition_can_last_seen is None
    # back to line-only semantics until CAN is seen again
    assert get_ignition_state([ps(line=True)], now=1.0)

  def test_can_recovery_inside_hold_window_restarts_it(self):
    assert get_ignition_state([ps(can=True, line=True)], now=0.0)
    assert get_ignition_state([ps(can=False, line=True)], now=3.0)
    assert get_ignition_state([ps(can=True, line=True)], now=4.0)
    # a new drop measures from the recovery, not the first drop
    assert get_ignition_state([ps(can=False, line=True)], now=4.0 + IGNITION_CAN_DROP_HOLD_S - 0.1)
    assert not get_ignition_state([ps(can=False, line=True)], now=4.0 + IGNITION_CAN_DROP_HOLD_S)

  def test_any_panda_counts(self):
    states = [ps(can=False, line=False), ps(can=True, line=False)]
    assert get_ignition_state(states, now=0.0)
    states = [ps(can=False, line=True), ps(can=False, line=False)]
    assert get_ignition_state(states, now=2.0)
    assert not get_ignition_state(states, now=2.0 + IGNITION_CAN_DROP_HOLD_S)

  def test_default_clock(self):
    # the callers pass nothing; the monotonic default must work
    assert get_ignition_state([ps(can=True)])
    assert get_ignition_state([ps(line=True)])
