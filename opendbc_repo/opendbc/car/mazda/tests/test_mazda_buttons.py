"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

CRZ_BTNS from the controller: the resume button's ownership under alpha long and the cancel
carve-out while the stock radar still owns the bus.
"""
import pytest

from opendbc.car.mazda.carcontroller import CANCEL_SETTLE_FRAMES
from opendbc.car.mazda.tests.conftest import CRZ_BTNS, LongCtrlState, addrs, car_control, step


class TestResumeButton:

  @pytest.mark.parametrize("accel", [0.3, -1.0])
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


def cancel_kwargs(**over):
  """openpilot disengaged, the car still reporting cruise on, controlsd asking for a cancel."""
  kw = dict(long_active=False, enabled=False, accel=0., long_state=LongCtrlState.off, available=False,
            cruise_engaged=True, cancel=True, stock_radar_alive=True, fsc_settled=False, radar_was_silenced=False)
  kw.update(over)
  return kw


def drive(cc, cs, frames, **kw) -> set[int]:
  sent = []
  for _ in range(frames):
    sent.extend(step(cc, cs, **kw)[1])
  return addrs(sent)


def cancel_frames(cc, cs, **over) -> set[int]:
  """Everything sent through the settle wait and the first cancel-cadence slot after it."""
  return drive(cc, cs, CANCEL_SETTLE_FRAMES + 10, **cancel_kwargs(**over))


class TestCancelCarveOut:
  """controlsd raises cruiseControl.cancel whenever cruiseState.enabled has no matching
  CC.enabled (mazda reports pcmCruise). While the stock radar still owns the bus that
  engagement is the driver's own stock MRCC and a CANCEL turns its main off within ~100 ms,
  so the documented stay-stock fallback used to leave the driver with no cruise at all. Once
  the radar has been silenced a stock engagement is impossible and cancel handles desync."""

  def test_no_cancel_while_the_radar_is_stock(self, cc, cs):
    # pre-teardown settle window, and equally the silencing-failed drive: a driver SET is
    # their own stock MRCC and must be left alone
    assert CRZ_BTNS not in cancel_frames(cc, cs), "CANCELed the driver's own stock MRCC"

  def test_cancel_still_sent_after_the_teardown(self, cc, cs):
    # post-teardown a stock engagement is impossible: cancel keeps handling state desync
    assert CRZ_BTNS in cancel_frames(cc, cs, radar_was_silenced=True, stock_radar_alive=False)

  def test_stock_longitudinal_cancel_unaffected(self, stock_cc, stock_cs):
    assert CRZ_BTNS in cancel_frames(stock_cc, stock_cs)


class TestCancelSettle:
  """The car answers its own cancels 60 to 90 ms after openpilot disengages on them (brake, the
  wheel CANCEL through MADS), and a CANCEL of ours landing with cruise already off is the stock
  main-off. The first press waits for the request to hold CANCEL_SETTLE_T; a request the car
  answers inside that never produces a press. See mazda-longitudinal.md, The driver's own CANCEL."""

  def test_first_press_waits_out_the_settle(self, stock_cc, stock_cs):
    assert CRZ_BTNS not in drive(stock_cc, stock_cs, CANCEL_SETTLE_FRAMES, **cancel_kwargs())
    assert CRZ_BTNS in drive(stock_cc, stock_cs, 10, **cancel_kwargs())

  def test_request_the_car_answers_never_presses(self, stock_cc, stock_cs):
    # the wheel-cancel shape: the request lasts 6 to 10 frames, then cruise reports off
    assert CRZ_BTNS not in drive(stock_cc, stock_cs, 10, **cancel_kwargs())
    assert CRZ_BTNS not in drive(stock_cc, stock_cs, CANCEL_SETTLE_FRAMES, **cancel_kwargs(cancel=False, cruise_engaged=False))

  def test_request_dropping_restarts_the_wait(self, stock_cc, stock_cs):
    drive(stock_cc, stock_cs, CANCEL_SETTLE_FRAMES - 1, **cancel_kwargs())
    drive(stock_cc, stock_cs, 1, **cancel_kwargs(cancel=False, cruise_engaged=False))
    assert CRZ_BTNS not in drive(stock_cc, stock_cs, CANCEL_SETTLE_FRAMES, **cancel_kwargs())
