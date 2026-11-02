"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

openpilot never presses the TJA button. On the car's side it toggles MADS and arms MRCC; on the
camera's side it switches the car's own lane-keep setting off (CAM_SETTINGS), after which the
EPS applies none of our request (route 7c735af5fce56485/00000105, 2026-09-12).

The MRCC undo (zoompilot/opendbc#17): on a declared TJA car the physical press also arms MRCC in
the body. When MRCC was off before the press, the controller answers with the wheel's own MRCC
master press on bus 0 until raw PEDALS confirms the arm is gone; a pre-armed MRCC is the driver's
and is left alone.
"""
import pytest

from opendbc.car.mazda.tests.conftest import CRZ_BTNS, SendButtonState, car_controller, frames, mazda_car_state, step
from opendbc.sunnypilot.car.mazda.values import MazdaFlagsSP

HOLD_CYCLES = 15  # past the shared button pacing before the release


def rig(alpha_long=False, tja_button=False):
  cc = car_controller(alpha_long=alpha_long)
  cc.CP_SP.flags |= MazdaFlagsSP.TJA_BUTTON if tja_button else 0
  cs = mazda_car_state(cc.CP, cc.CP_SP)
  return cc, cs


def drive(cc, cs, cycles, **kwargs):
  sends = []
  for _ in range(cycles):
    sends.extend(step(cc, cs, **kwargs)[1])
  return sends


def mrcc_off_frames(sends) -> list[bytes]:
  # the wheel's MRCC master press on the car's bus: exact shape, CTR the only variable bits
  return [d for d in frames(sends, CRZ_BTNS)
          if d[:3] == b"\x00\x81\xfe" and d[3] & 0xc3 == 0xc0 and d[4:] == bytes(4)]


def undo_episode(alpha_long):
  # cruise off before a TJA press-and-hold: the press-induced arm is live, undo pending
  cc, cs = rig(alpha_long, tja_button=True)
  drive(cc, cs, 5, available=False)
  held = drive(cc, cs, HOLD_CYCLES, available=True, mrcc_armed_raw=True, tja_button=1)
  return cc, cs, held


@pytest.mark.parametrize("alpha_long", [False, True])
class TestMrccUndo:

  def test_off_before_press_undoes_the_arm(self, alpha_long):
    cc, cs, held = undo_episode(alpha_long)
    assert mrcc_off_frames(held) == []  # nothing while the driver holds the button
    released = drive(cc, cs, 40, available=True, mrcc_armed_raw=True)
    assert 0 < len(mrcc_off_frames(released)) <= 3  # the hold: at most the episode budget
    settled = drive(cc, cs, 10, available=False)  # arm reconciled
    assert mrcc_off_frames(settled) == []
    assert not cc.mrcc_undo_pending
    assert cc.mrcc_undo_frames == 0  # budget returned for the next press

  @pytest.mark.parametrize("pre_armed", [True, False])
  def test_the_drivers_own_arm_and_no_arm_are_left_alone(self, alpha_long, pre_armed):
    # cruise already on before the press, or a press that never arms: no undo either way
    cc, cs = rig(alpha_long, tja_button=True)
    drive(cc, cs, 5, available=pre_armed, mrcc_armed_raw=pre_armed)
    sends = drive(cc, cs, HOLD_CYCLES, available=pre_armed, mrcc_armed_raw=pre_armed, tja_button=1)
    sends += drive(cc, cs, 40, available=pre_armed, mrcc_armed_raw=pre_armed)
    assert mrcc_off_frames(sends) == []

  def test_budget_caps_the_hold_and_returns_after_reconciliation(self, alpha_long):
    cc, cs, _ = undo_episode(alpha_long)
    stuck = drive(cc, cs, 80, available=True, mrcc_armed_raw=True)  # the arm never clears
    assert len(mrcc_off_frames(stuck)) == 3
    assert not cc.mrcc_undo_pending
    drive(cc, cs, 10, available=False)  # reconciled: the budget is back
    drive(cc, cs, 5, available=False)
    again = drive(cc, cs, HOLD_CYCLES, available=True, mrcc_armed_raw=True, tja_button=1)
    again += drive(cc, cs, 40, available=True, mrcc_armed_raw=True)
    assert mrcc_off_frames(again) != []

  def test_a_second_press_before_reconciliation_keeps_the_episode(self, alpha_long):
    # a fast double-press: the second edge lands while the first press's arm is still
    # live, so the "armed before the press" sample is the first press's artifact; the
    # episode must survive and keep undoing, or MRCC stays armed until the driver clears it
    cc, cs, _ = undo_episode(alpha_long)
    released = drive(cc, cs, 10, available=True, mrcc_armed_raw=True)  # one press out, arm unreconciled
    assert len(mrcc_off_frames(released)) == 1
    again = drive(cc, cs, HOLD_CYCLES, available=True, mrcc_armed_raw=True, tja_button=1)  # second press
    assert mrcc_off_frames(again) == []  # nothing while held
    assert cc.mrcc_undo_pending  # not canceled by the second edge
    after = drive(cc, cs, 40, available=True, mrcc_armed_raw=True)
    assert 0 < len(mrcc_off_frames(after)) <= 3  # the budget returned and undoes the second arm

  def test_a_second_press_at_reconciliation_waits_for_its_own_arm(self, alpha_long):
    # the first arm clears under the second press: PEDALS confirms the disarm inside the
    # hold, and the second press's own arm lands only after the release; the episode must
    # wait it out instead of standing down on the confirmed disarm
    cc, cs, _ = undo_episode(alpha_long)
    drive(cc, cs, 10, available=True, mrcc_armed_raw=True)
    held = drive(cc, cs, HOLD_CYCLES, available=False, mrcc_armed_raw=False, tja_button=1)
    assert mrcc_off_frames(held) == []
    assert cc.mrcc_undo_pending  # waiting for this press's arm, not stood down
    armed = drive(cc, cs, 40, available=True, mrcc_armed_raw=True)
    assert 0 < len(mrcc_off_frames(armed)) <= 3

  def test_brake_dropout_does_not_end_the_episode(self, alpha_long):
    # both PEDALS cruise bits read low through a brake transition; the filtered state
    # bridges it, so three raw-off cycles stay under the five-frame confirmation
    cc, cs, _ = undo_episode(alpha_long)
    drive(cc, cs, 3, available=True, mrcc_armed_raw=False)
    assert cc.mrcc_undo_pending

  def test_arm_clearing_stands_down_without_a_second_press(self, alpha_long):
    # the undo's own press disarms the car ~80 ms before PEDALS confirms it; the raw gate
    # and the 200 ms slot must hold the second press back, or it would re-arm the car
    cc, cs, _ = undo_episode(alpha_long)
    pressing = drive(cc, cs, 10, available=True, mrcc_armed_raw=True)  # one press out, arm still reflected
    assert len(mrcc_off_frames(pressing)) == 1
    clearing = drive(cc, cs, 40, available=False, mrcc_armed_raw=False)  # the arm clears mid-budget
    assert mrcc_off_frames(clearing) == []
    assert not cc.mrcc_undo_pending  # one press was enough: stood down without spending the budget

  def test_no_arm_wait_is_bounded_and_brake_held(self, alpha_long):
    # a car that never arms through the press: the undo stands down after 1 s brake-free
    # and ICBM gets the button stream back; a held brake restarts the clock, since PEDALS
    # cannot witness an arm under braking
    cc, cs = rig(alpha_long, tja_button=True)
    drive(cc, cs, 5, available=False)
    drive(cc, cs, HOLD_CYCLES, available=False, tja_button=1)
    drive(cc, cs, 60, available=False)  # brake-free wait, still inside 1 s
    assert cc.mrcc_undo_pending
    held = drive(cc, cs, 100, available=False, brake_pressed=True, send_button=SendButtonState.increase)
    assert cc.mrcc_undo_pending  # the brake restarts the clock
    assert frames(held, CRZ_BTNS, bus=0) == []  # ICBM still suppressed while the undo owns the stream
    resumed = drive(cc, cs, 110, available=False, send_button=SendButtonState.increase)
    assert not cc.mrcc_undo_pending  # 1 s brake-free elapsed
    assert frames(resumed, CRZ_BTNS, bus=0) != []  # ICBM has the stream back

  @pytest.mark.parametrize("activity", [dict(mrcc_button=1), dict(cancel_button=1), dict(accel_button=1),
                                        dict(handback=True)])
  def test_driver_activity_aborts(self, alpha_long, activity):
    # the driver's own cruise presses and a stock ECU hand-back all stand down at once
    cc, cs, _ = undo_episode(alpha_long)
    drive(cc, cs, 30, available=True, mrcc_armed_raw=True, **activity)
    assert not cc.mrcc_undo_pending

  def test_openpilot_cancel_waits_then_resumes(self, alpha_long):
    cc, cs, _ = undo_episode(alpha_long)
    waiting = drive(cc, cs, 20, available=True, mrcc_armed_raw=True, cancel=True)
    assert mrcc_off_frames(waiting) == []  # no frame races openpilot's own cancel
    assert cc.mrcc_undo_pending  # waited out, not aborted
    resumed = drive(cc, cs, 30, available=True, mrcc_armed_raw=True)
    assert 0 < len(mrcc_off_frames(resumed)) <= 3


class TestMrccUndoShipsDark:

  def test_radar_handback_aborts_with_the_budget_frozen(self):
    # under alpha long the radar session recomputes handback_active every frame, so the abort
    # runs on a stock-long controller, whose session never updates
    cc, cs, _ = undo_episode(alpha_long=False)
    drive(cc, cs, 3, available=True, mrcc_armed_raw=True)  # episode live, at most one frame out
    spent = cc.mrcc_undo_frames
    cc.radar_session.handback_active = True
    sends = drive(cc, cs, 30, available=True, mrcc_armed_raw=True)
    assert not cc.mrcc_undo_pending
    assert cc.mrcc_undo_frames == spent  # aborted, not budget-exhausted
    assert mrcc_off_frames(sends) == []

  def test_undeclared_button_never_sends(self):
    cc, cs = rig()
    drive(cc, cs, 5, available=False)
    drive(cc, cs, HOLD_CYCLES, available=True, mrcc_armed_raw=True, tja_button=1)
    sends = drive(cc, cs, 60, available=True, mrcc_armed_raw=True)
    assert mrcc_off_frames(sends) == []
    assert not cc.mrcc_undo_pending

  def test_icbm_suppressed_through_the_hold(self):
    cc, cs = rig(tja_button=True)
    held = drive(cc, cs, HOLD_CYCLES, available=True, mrcc_armed_raw=True, tja_button=1,
                 send_button=SendButtonState.increase)
    # the wheel owns the counter stream during the hold: ICBM stays quiet too
    assert frames(held, CRZ_BTNS, bus=0) == []
