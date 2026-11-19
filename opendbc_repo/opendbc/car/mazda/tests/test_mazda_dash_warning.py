"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The hands-on-wheel frame on the car's own dash: the rules are DashSteerWarning's docstring, the
frame the cluster draws is mazdacan.create_alert_command's.
"""

import pytest

from opendbc.car.mazda.carcontroller import DASH_STEER_WARNING_QUIET_FRAMES, HUD_ALERT_FRAMES
from opendbc.car.mazda.tests.conftest import CAM_LANEINFO, VisualAlert, frames, hands_code, step

WARNING = (0b111, 1, 1)
IDLE = (0, 0, 0)
ALERT = dict(visual_alert=VisualAlert.steerRequired)

OFF = dict(enabled=False, long_active=False, lat_active=False)
CRUISE = dict(enabled=True, long_active=False, lat_active=True)
MADS_ONLY = dict(enabled=False, long_active=False, lat_active=True)


def drive(cc, cs, cycles, **kwargs) -> list[tuple[int, int, int]]:
  """The hands code of every dash frame sent."""
  hud = []
  for _ in range(cycles):
    _, out = step(cc, cs, **kwargs)
    hud.extend(hands_code(d) for d in frames(out, CAM_LANEINFO))
  return hud


def settle(cc, cs, **mode):
  """Run past the post-engage quiet window with no alert up."""
  drive(cc, cs, DASH_STEER_WARNING_QUIET_FRAMES + 1, **mode)


def warning_up(cc, cs, **mode):
  settle(cc, cs, **mode)
  assert drive(cc, cs, HUD_ALERT_FRAMES, **ALERT, **mode) == [WARNING]


@pytest.mark.parametrize("mode, extra, expected", [
  (CRUISE, ALERT, WARNING),
  (MADS_ONLY, ALERT, WARNING),
  (CRUISE, {}, IDLE),
  (CRUISE, dict(ALERT, lkas_allowed_speed=False), IDLE),
  (OFF, ALERT, IDLE),
], ids=["cruise", "mads_only", "no_alert", "below_speed_floor", "not_steering"])
def test_mirror(stock_cc, stock_cs, mode, extra, expected):
  settle(stock_cc, stock_cs, **mode)
  # exactly the two cadence slots in 2 * HUD_ALERT_FRAMES cycles, nothing off-cadence
  assert drive(stock_cc, stock_cs, 2 * HUD_ALERT_FRAMES, **extra, **mode) == [expected, expected]


class TestQuietWindow:
  def test_alert_at_engagement_waits_out_the_window(self, stock_cc, stock_cs):
    drive(stock_cc, stock_cs, 1, **OFF)
    quiet = drive(stock_cc, stock_cs, DASH_STEER_WARNING_QUIET_FRAMES, **ALERT, **MADS_ONLY)
    after = drive(stock_cc, stock_cs, HUD_ALERT_FRAMES, **ALERT, **MADS_ONLY)
    assert quiet and all(code == IDLE for code in quiet)
    assert after == [WARNING]

  def test_cruise_engaging_under_mads_reopens_the_window(self, stock_cc, stock_cs):
    settle(stock_cc, stock_cs, **MADS_ONLY)
    hud = drive(stock_cc, stock_cs, DASH_STEER_WARNING_QUIET_FRAMES, **ALERT, **CRUISE)
    assert hud and all(code == IDLE for code in hud)


class TestWithdraw:
  def test_brake_press_withdraws_immediately(self, stock_cc, stock_cs):
    warning_up(stock_cc, stock_cs, **CRUISE)
    # the next frame is off-cadence; the withdraw does not wait for the slot
    assert drive(stock_cc, stock_cs, 1, brake_pressed=True, **ALERT, **CRUISE) == [IDLE]

  def test_cruise_cancel_withdraws_and_holds_until_the_alert_clears(self, stock_cc, stock_cs):
    warning_up(stock_cc, stock_cs, **CRUISE)
    hud = drive(stock_cc, stock_cs, 2 * HUD_ALERT_FRAMES, **ALERT, **MADS_ONLY)
    assert hud and all(code == IDLE for code in hud)
    # the alert clears; a fresh one shows again in the steady state
    drive(stock_cc, stock_cs, 1, **MADS_ONLY)
    assert drive(stock_cc, stock_cs, HUD_ALERT_FRAMES, **ALERT, **MADS_ONLY) == [WARNING]

  def test_lateral_dropping_withdraws(self, stock_cc, stock_cs):
    warning_up(stock_cc, stock_cs, **MADS_ONLY)
    assert drive(stock_cc, stock_cs, 1, **ALERT, **OFF) == [IDLE]

  def test_no_transition_no_extra_frames(self, stock_cc, stock_cs):
    warning_up(stock_cc, stock_cs, **CRUISE)
    assert drive(stock_cc, stock_cs, HUD_ALERT_FRAMES - 1, **ALERT, **CRUISE) == []
