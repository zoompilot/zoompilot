"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

CRZ_BTNS from the controller: the resume button's ownership under alpha long and the cancel
carve-out while the stock radar still owns the bus.
"""
import pytest

from opendbc.car.mazda.longitudinal import RELEASE_DEBOUNCE_FRAMES
from opendbc.car.mazda.tests.conftest import CRZ_BTNS, LongCtrlState, addrs, car_control, step, step_long


class TestResumeButton:

  @pytest.mark.parametrize("accel", [0.3, -1.024])
  @pytest.mark.parametrize("standstill", [True, False])
  def test_no_resume_button_while_openpilot_owns_longitudinal(self, cc, accel, standstill):
    # We are the ACC here, so the hold is released in-protocol. The car's own MRCC never presses
    # RES either: 0 of 23 stock body-latched-hold releases put one on the bus. A press would also
    # put a second writer on CRZ_BTNS, which ICBM owns.
    assert not cc.resume_requested(car_control(accel=accel, resume=True))

  def test_resume_button_still_sent_with_stock_longitudinal(self, stock_cc):
    # stock ACC owns the hold there, and the button is the only lever openpilot has on it
    assert stock_cc.resume_requested(car_control(accel=0.3, resume=True))
    assert not stock_cc.resume_requested(car_control(accel=0.3, resume=False))

  def test_body_latched_hold_releases_in_protocol(self, cc, cs):
    # the release the button used to stand in for: stop bits already relaxed to the body, then
    # the plan asks to move and the unlatch pulse fires with the release
    for _ in range(200):
      step_long(cc, cs, long_state=LongCtrlState.stopping, accel=-1.024, standstill=True, cruise_engaged=True, brake_hold=True)
    assert cc.stop_and_go.holding and cc.stop_and_go.car_has_hold
    assert not cc.stop_and_go.stop_bits  # body owns the brakes, stock relaxes here

    for _ in range(RELEASE_DEBOUNCE_FRAMES):
      sends = step_long(cc, cs, accel=0.3, standstill=True, cruise_engaged=True, brake_hold=True)
      assert CRZ_BTNS not in addrs(sends), "CRZ_BTNS written at the release"
    assert not cc.stop_and_go.holding
    assert cc.stop_and_go.resume_unlatching, "the pulse must fire with the release"


def cancel_frame(cc, cs, cancel, radar_was_silenced, stock_radar_alive):
  cc.frame = 10  # off the 50-frame alert cadence, on the 10-frame cancel cadence
  _, sends = step(cc, cs, long_active=False, enabled=False, accel=0., long_state=LongCtrlState.off, available=False,
                  cruise_engaged=True, cancel=cancel, stock_radar_alive=stock_radar_alive, fsc_settled=False,
                  radar_was_silenced=radar_was_silenced)
  return addrs(sends)


class TestCancelCarveOut:
  """controlsd raises cruiseControl.cancel whenever cruiseState.enabled has no matching
  CC.enabled (mazda reports pcmCruise). While the stock radar still owns the bus that
  engagement is the driver's own stock MRCC and a CANCEL turns its main off within ~100 ms,
  so the documented stay-stock fallback used to leave the driver with no cruise at all. Once
  the radar has been silenced a stock engagement is impossible and cancel handles desync."""

  def test_no_cancel_while_the_radar_is_stock(self, cc, cs):
    # pre-teardown settle window, and equally the silencing-failed drive: a driver SET is
    # their own stock MRCC and must be left alone
    sent = cancel_frame(cc, cs, cancel=True, radar_was_silenced=False, stock_radar_alive=True)
    assert CRZ_BTNS not in sent, "CANCELed the driver's own stock MRCC"

  def test_cancel_still_sent_after_the_teardown(self, cc, cs):
    # post-teardown a stock engagement is impossible: cancel keeps handling state desync
    sent = cancel_frame(cc, cs, cancel=True, radar_was_silenced=True, stock_radar_alive=False)
    assert CRZ_BTNS in sent

  def test_stock_longitudinal_cancel_unaffected(self, stock_cc, stock_cs):
    sent = cancel_frame(stock_cc, stock_cs, cancel=True, radar_was_silenced=False, stock_radar_alive=True)
    assert CRZ_BTNS in sent
