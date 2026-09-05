"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

ICBM servo: limiter-scoped deadband, fast decisive down-moves, patient exact restores,
the fast stream with its tap fallback, and the route 126 restore-responsiveness fixes.
"""
from openpilot.cereal import custom
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import (
  DRIVER_PRESS_GRACE_T, REACT_DEADBAND, RESTORE_QUIET_TIME)
from openpilot.sunnypilot.selfdrive.car.tests.icbm_servo_harness import make_icbm, run_frames

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState


class TestServo:
  """ButtonActuator servo: limiter-scoped deadband, fast decisive down-moves, patient
  exact restores, hold planning from the per-car profile, tap fallback."""

  def setup_method(self):
    self.icbm = make_icbm()

  def make_icbm(self, brand=""):
    return make_icbm(brand)

  def run_frames(self, *args, icbm=None, **kwargs):
    return run_frames(icbm or self.icbm, *args, **kwargs)

  def test_limiter_jitter_within_deadband_no_send(self):
    self.run_frames(35, 35, n=60)
    assert self.icbm.state == State.holding

    sends = self.run_frames(35 - (REACT_DEADBAND - 1), 35, n=100)
    assert self.icbm.state == State.holding
    assert all(s == SendButtonState.none for s in sends)

  def test_limiter_down_move_beyond_deadband(self):
    self.run_frames(35, 35, n=60)

    sends = self.run_frames(35 - REACT_DEADBAND, 35, n=100)
    assert self.icbm.state == State.decreasing
    assert any(s == SendButtonState.decrease for s in sends)

  def test_transient_glitch_filtered(self):
    """A short-lived target drop (e.g. one bad map sample) must not trigger buttons."""
    self.run_frames(45, 45, n=60)
    assert self.icbm.state == State.holding

    sends = self.run_frames(25, 45, n=20)  # glitch shorter than REACT_TIMER (0.3s)
    sends += self.run_frames(45, 45, n=100)
    assert all(s == SendButtonState.none for s in sends)
    assert self.icbm.state == State.holding

  def test_runs_to_exact_target(self):
    """Once moving, ICBM steps all the way to the target, not just inside the deadband."""
    self.run_frames(45, 45, n=60)
    cluster = 45.
    for _ in range(600):
      sends = self.run_frames(35, cluster, n=1)
      if sends[0] == SendButtonState.decrease:
        cluster -= 1  # dash responds ~1 mph per press
    assert cluster == 35.
    assert self.icbm.state == State.holding

  def test_restore_waits_for_quiet_target(self):
    """After a limiter dip ends, the restore up must wait out RESTORE_QUIET_TIME on cars
    whose profile declares decel_needs_stable_setpoint: curves arrive in trains, and
    these ECUs won't decel while the set speed is moving."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(55, 55, n=60, icbm=icbm)
    assert icbm.state == State.holding

    # target back at the driver's 60; less than the quiet time elapsed -> no buttons yet
    sends = self.run_frames(60, 55, n=int(RESTORE_QUIET_TIME / DT_CTRL) - 50, source='cruise', icbm=icbm)
    assert all(s == SendButtonState.none for s in sends)
    assert icbm.state == State.holding

    # quiet time satisfied -> restore fires and runs
    self.run_frames(60, 55, n=100, source='cruise', icbm=icbm)
    assert icbm.state == State.increasing

  def test_default_profile_restores_without_patience(self):
    """Cars without decel_needs_stable_setpoint keep their fast restores; the quiet
    window is per-car behavior, not a global regression."""
    self.run_frames(55, 55, n=60)
    assert self.icbm.state == State.holding

    self.run_frames(60, 55, n=60, source='cruise')
    assert self.icbm.state == State.increasing

  def test_restore_is_exact_to_one_unit(self):
    """The F2 ratchet: a 1 mph residual against the driver setpoint must self-heal (no
    deadband against a cruise-source target)."""
    self.run_frames(59, 59, n=60, source='cruise')
    assert self.icbm.state == State.holding

    self.run_frames(60, 59, n=int(RESTORE_QUIET_TIME / DT_CTRL) + 100, source='cruise')
    assert self.icbm.state == State.increasing

  def test_moving_target_resets_restore_quiet(self):
    """An up-target that keeps moving (another dip building) never triggers a restore."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(55, 55, n=60, icbm=icbm)
    for _ in range(4):
      self.run_frames(60, 55, n=100, source='cruise', icbm=icbm)
      self.run_frames(59, 55, n=100, source='cruise', icbm=icbm)
    assert icbm.state == State.holding

  def test_fast_stream_for_large_moves(self):
    """A move with real distance runs the stream, drops to taps for the small remainder,
    and lands exactly. The dash walks in 1 mph presses: that is all the ECU ever does
    with forged frames."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(60, 60, n=60, icbm=icbm)

    self.run_frames(45, 60, n=60, icbm=icbm)
    assert icbm.state == State.decreasing
    assert icbm.cruise_button == SendButtonState.decreaseHold

    # dash walks down 1 mph at a time; the stream holds until the remainder is small
    self.run_frames(45, 48, n=5, icbm=icbm)
    assert icbm.cruise_button == SendButtonState.decreaseHold
    self.run_frames(45, 47, n=5, icbm=icbm)
    assert icbm.cruise_button == SendButtonState.decrease

    self.run_frames(45, 45, n=5, icbm=icbm)
    assert icbm.state == State.holding

  def test_stream_falls_back_to_taps_when_dash_frozen(self):
    """If the dash never moves under the stream, this ECU is not registering it; taps
    are the proven fallback for the rest of the drive."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(60, 60, n=60, icbm=icbm)

    self.run_frames(45, 60, n=60, icbm=icbm)
    assert icbm.cruise_button == SendButtonState.decreaseHold

    # dash frozen past the stall window -> fault and tap from here on
    self.run_frames(45, 60, n=160, icbm=icbm)
    assert icbm.fast_faulted
    assert icbm.cruise_button == SendButtonState.decrease

    # a later large move stays taps-only
    self.run_frames(60, 60, n=200, icbm=icbm)
    self.run_frames(40, 60, n=60, icbm=icbm)
    assert icbm.cruise_button == SendButtonState.decrease

  def test_metric_uses_the_stream_too(self):
    """The stream carries no grid assumption (it is just presses), so metric users get
    the fast walk as well."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(60, 60, n=60, icbm=icbm, is_metric=True)

    self.run_frames(45, 60, n=60, icbm=icbm, is_metric=True)
    assert icbm.state == State.decreasing
    assert icbm.cruise_button == SendButtonState.decreaseHold



class TestRestoreResponsiveness:
  """Route 126 fixes: the quiet timer keys on the raw plan target (the overshoot lever's
  decay is not plan motion), the vision lookahead replaces stillness when present, and a
  genuine driver SET+ press parks down-moves for a grace window."""

  def make_icbm(self, brand=""):
    return make_icbm(brand)

  def run_frames(self, *args, icbm, **kwargs):
    return run_frames(icbm, *args, **kwargs)

  def test_restore_not_stalled_by_overshoot_decay(self):
    """After a limiter release with a built-up overshoot gap, the restore must start about
    a quiet-window after the flip, not after the residual finishes bleeding off."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(40, 40, n=60, icbm=icbm, v_ego_mph=40., overshoot=True)
    # curve: deep decel demand builds the full gap and walks the dash down
    self.run_frames(30, 31, n=100, icbm=icbm, v_ego_mph=39., a_target=-1.2, overshoot=True)
    assert icbm.overshoot_mph > 5.

    # road straightens: source back to cruise, target back at the driver's 40
    first_up = None
    for i in range(400):
      sends = self.run_frames(40, 31, n=1, icbm=icbm, source='cruise', v_ego_mph=33., overshoot=True)
      if sends[0] == SendButtonState.increase or sends[0] == SendButtonState.increaseHold:
        first_up = i * DT_CTRL
        break
    assert first_up is not None, "restore never started"
    assert first_up < RESTORE_QUIET_TIME + 0.6, f"restore stalled {first_up:.2f}s behind the decay"

  def test_lookahead_dip_blocks_restore(self):
    """A dip below the target on the vision horizon holds the restore regardless of how
    quiet the target is: restoring between bends feeds the next apex."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(40, 40, n=60, icbm=icbm)
    sends = self.run_frames(40, 30, n=300, icbm=icbm, source='cruise', v_ahead_min_mph=25.)
    assert all(s == SendButtonState.none for s in sends)
    assert icbm.state == State.holding

  def test_lookahead_clear_skips_quiet_window(self):
    """With the horizon clear the profile is the churn oracle; stillness is redundant and
    the restore fires on the react timer alone."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(40, 40, n=60, icbm=icbm)
    first_up = None
    for i in range(200):
      sends = self.run_frames(40, 30, n=1, icbm=icbm, source='cruise', v_ahead_min_mph=255. / CV.MPH_TO_MS)
      if sends[0] in (SendButtonState.increase, SendButtonState.increaseHold):
        first_up = i * DT_CTRL
        break
    assert first_up is not None
    assert first_up < RESTORE_QUIET_TIME, f"lookahead-clear restore still waited {first_up:.2f}s"

  def test_dip_appearing_mid_restore_aborts(self):
    """The commit gate trails the profile; a dip appearing while stepping up must stop the
    restore instead of accelerating until the limiter takes the source."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(40, 40, n=60, icbm=icbm)
    self.run_frames(40, 34, n=150, icbm=icbm, source='cruise', v_ahead_min_mph=200.)
    assert icbm.state == State.increasing

    sends = self.run_frames(40, 35, n=100, icbm=icbm, source='cruise', v_ahead_min_mph=25.)
    assert icbm.state == State.holding
    assert all(s == SendButtonState.none for s in sends[5:])

  def test_driver_up_press_grace_blocks_down(self):
    """A genuine SET+ press parks synthesized down-moves for the grace window even while a
    limiter demands them; the servo resumes once the window expires."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(40, 40, n=60, icbm=icbm)
    press = [ButtonEvent(type=ButtonType.accelCruise, pressed=True)]
    release = [ButtonEvent(type=ButtonType.accelCruise, pressed=False)]
    self.run_frames(30, 40, n=5, icbm=icbm, button_events=press)
    self.run_frames(30, 45, n=1, icbm=icbm, button_events=release)

    quiet, resumed = [], []
    for i in range(int(DRIVER_PRESS_GRACE_T / DT_CTRL) + 200):
      sends = self.run_frames(30, 45, n=1, icbm=icbm, a_target=-1.0)
      (quiet if i * DT_CTRL < DRIVER_PRESS_GRACE_T - 0.1 else resumed).extend(sends)
    assert all(s == SendButtonState.none for s in quiet), "servo fought the driver inside the grace window"
    assert any(s in (SendButtonState.decrease, SendButtonState.decreaseHold) for s in resumed), \
      "servo never resumed after the grace window"

  def test_driver_down_press_ends_grace(self):
    """A SET- press is aligned intent and cancels the SET+ grace immediately."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(40, 40, n=60, icbm=icbm)
    self.run_frames(30, 40, n=5, icbm=icbm, button_events=[ButtonEvent(type=ButtonType.accelCruise, pressed=True)])
    self.run_frames(30, 45, n=1, icbm=icbm, button_events=[ButtonEvent(type=ButtonType.accelCruise, pressed=False)])
    self.run_frames(30, 45, n=3, icbm=icbm, button_events=[ButtonEvent(type=ButtonType.decelCruise, pressed=True)])
    self.run_frames(30, 45, n=1, icbm=icbm, button_events=[ButtonEvent(type=ButtonType.decelCruise, pressed=False)])
    assert icbm.down_grace_timer == 0

    sends = self.run_frames(30, 45, n=150, icbm=icbm)
    assert any(s in (SendButtonState.decrease, SendButtonState.decreaseHold) for s in sends)

  def test_driver_down_press_grace_blocks_up(self):
    """The mirror: after a genuine SET- press a refused re-anchor must not restore the
    baseline over the driver's head, even with the lookahead clear."""
    icbm = self.make_icbm(brand="mazda")
    self.run_frames(40, 40, n=60, icbm=icbm, source='cruise')
    self.run_frames(40, 40, n=5, icbm=icbm, source='cruise',
                    button_events=[ButtonEvent(type=ButtonType.decelCruise, pressed=True)])
    self.run_frames(40, 35, n=1, icbm=icbm, source='cruise',
                    button_events=[ButtonEvent(type=ButtonType.decelCruise, pressed=False)])

    quiet, resumed = [], []
    for i in range(int(DRIVER_PRESS_GRACE_T / DT_CTRL) + 300):
      sends = self.run_frames(40, 35, n=1, icbm=icbm, source='cruise', v_ahead_min_mph=200.)
      (quiet if i * DT_CTRL < DRIVER_PRESS_GRACE_T - 0.1 else resumed).extend(sends)
    assert all(s == SendButtonState.none for s in quiet), "servo restored over the driver's SET-"
    assert any(s in (SendButtonState.increase, SendButtonState.increaseHold) for s in resumed), \
      "restore never resumed after the grace window"
