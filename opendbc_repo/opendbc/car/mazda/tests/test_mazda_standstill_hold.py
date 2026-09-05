"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The StandstillHold state machine on its own: the hold, the body-latched relax, the release
debounce, the RESUME_UNLATCHING pulse and its one retry.
"""
import pytest

from opendbc.car import DT_CTRL
from opendbc.car.mazda.longitudinal import (RELEASE_DEBOUNCE_FRAMES, RESUME_REPULSE_FRAMES, RESUME_UNLATCH_LATCHED_FRAMES,
                                            StandstillHold)


def drive(sm, frames, **kwargs):
  defaults = dict(long_engaged=True, stopping=False, standstill=False, plan_accel=-1.024,
                  brake_hold=False, gas_pressed=False)
  defaults.update(kwargs)
  for _ in range(frames):
    sm.update(**defaults)
  return sm


def latched_release(sm):
  # a body-latched hold released by the plan: the first pulse fires with the release
  drive(sm, 1, stopping=True)
  drive(sm, 100, stopping=True, standstill=True, brake_hold=True)
  drive(sm, RELEASE_DEBOUNCE_FRAMES, standstill=True, brake_hold=True, plan_accel=0.5)
  assert sm.latched_release and sm.resume_unlatching


def test_holds_while_the_plan_is_stopping():
  sm = StandstillHold()
  drive(sm, 1)
  assert not sm.holding
  drive(sm, 1, stopping=True)
  assert sm.holding and sm.stop_bits and sm.acc_active_2
  # arriving at a standstill changes nothing: the plan is still asking for the brakes
  drive(sm, 500, stopping=True, standstill=True)
  assert sm.holding and sm.stop_bits


def test_hold_never_relaxes_on_its_own():
  sm = StandstillHold()
  # the creep-into-the-lead regression: without the car taking the hold over, the command
  # must stay on the plan's brake no matter how long the stop lasts
  drive(sm, 1, stopping=True)
  drive(sm, int(30.0 / DT_CTRL), stopping=True, standstill=True)
  assert sm.holding and sm.stop_bits and sm.acc_active_2
  assert not sm.car_has_hold


def test_relax_follows_the_car_taking_the_hold():
  sm = StandstillHold()
  drive(sm, 1, stopping=True)
  drive(sm, 10, stopping=True, standstill=True)
  assert not sm.car_has_hold
  drive(sm, 1, stopping=True, standstill=True, brake_hold=True)
  # stop bits and ACC_ACTIVE_2 drop with the command, together, exactly as stock does
  assert sm.car_has_hold and not sm.stop_bits and not sm.acc_active_2
  # and it is not a latch: if the car lets go, we brake again
  drive(sm, 1, stopping=True, standstill=True, brake_hold=False)
  assert not sm.car_has_hold and sm.stop_bits and sm.acc_active_2


def test_released_when_the_plan_asks_to_move():
  sm = StandstillHold()
  drive(sm, 1, stopping=True)
  drive(sm, 500, stopping=True, standstill=True, brake_hold=True)
  assert sm.holding
  # the release is debounced: a plan asking to move for less than the window changes nothing
  # (the body keeps its own latch until the pulse plays, so brake_hold stays up here)
  drive(sm, RELEASE_DEBOUNCE_FRAMES - 1, standstill=True, brake_hold=True, plan_accel=0.1)
  assert sm.holding and not sm.resume_unlatching
  drive(sm, 1, standstill=True, brake_hold=True, plan_accel=0.1)
  assert not sm.holding and not sm.car_has_hold
  # the body owned the brakes, so this is the latched family: the pulse fires with the
  # release. The body answers nothing else -- deferring behind silence (route 0000011d)
  # and behind a positive nudge (route 0000012c) both just delayed the resume.
  assert sm.latched_release and sm.resume_unlatching


def test_release_holds_for_as_long_as_the_plan_wants_to_move():
  sm = StandstillHold()
  # the failed-resume regression: no release window to run out from under the plan
  drive(sm, 1, stopping=True)
  drive(sm, 100, stopping=True, standstill=True)
  drive(sm, int(5.0 / DT_CTRL), standstill=True, plan_accel=0.4)
  assert not sm.holding and not sm.stop_bits


def test_hold_comes_back_if_the_plan_changes_its_mind():
  sm = StandstillHold()
  drive(sm, 1, stopping=True)
  drive(sm, 100, stopping=True, standstill=True)
  drive(sm, RELEASE_DEBOUNCE_FRAMES, standstill=True, plan_accel=0.2)
  assert not sm.holding
  # nothing was latched, so this release emits no unlatch bit at all, deferred or otherwise
  assert sm.unlatch_frames == 0 and not sm.resume_unlatching
  drive(sm, 1, stopping=True, standstill=True, plan_accel=-1.0)
  assert sm.holding
  assert not sm.resume_unlatching and sm.unlatch_frames == 0
  assert sm.stop_bits


def test_never_latched_release_emits_no_pulse():
  sm = StandstillHold()
  # a never-latched release has nothing latched to unlatch, so it puts no RESUME_UNLATCHING
  # on the wire at all. Stock blips here; we dropped it back when every pulse this port sent
  # latched the camera (the CRZ_INFO checksum, since fixed), and it has stayed dropped
  # because a blip that unlatches nothing buys nothing. Restoring it needs a drive, not a fix
  drive(sm, 1, stopping=True)
  drive(sm, 100, stopping=True, standstill=True)
  assert not sm.resume_unlatching
  drive(sm, RELEASE_DEBOUNCE_FRAMES, standstill=True, plan_accel=0.1)
  assert not sm.holding and not sm.latched_release
  drive(sm, int(1.0 / DT_CTRL), standstill=True, plan_accel=0.1)
  assert not sm.resume_unlatching and sm.unlatch_frames == 0


def test_latched_release_pulses_immediately_and_runs_its_length():
  sm = StandstillHold()
  # the pulse is the release protocol: the body ignores everything else (routes 0000011d
  # and 0000012c), so waiting only delays the resume. One pulse, stock's latched length.
  drive(sm, 1, stopping=True)
  drive(sm, 100, stopping=True, standstill=True, brake_hold=True)
  drive(sm, RELEASE_DEBOUNCE_FRAMES - 1, standstill=True, brake_hold=True, plan_accel=0.1)
  assert not sm.resume_unlatching
  drive(sm, 1, standstill=True, brake_hold=True, plan_accel=0.1)
  assert sm.resume_unlatching, "the pulse must fire with the release"
  drive(sm, RESUME_UNLATCH_LATCHED_FRAMES, standstill=True, brake_hold=True, plan_accel=0.1)
  assert not sm.resume_unlatching, "pulse outran its length"


def test_long_disengage_resets():
  sm = StandstillHold()
  drive(sm, 1, stopping=True)
  drive(sm, 100, stopping=True, standstill=True, brake_hold=True)
  drive(sm, 1, long_engaged=False)
  assert not sm.holding and not sm.car_has_hold and not sm.stop_bits


def test_gas_override_drive_off_releases_the_hold():
  sm = StandstillHold()
  # a driver-gas drive-off under an override zeroes the plan's command, so the plan never
  # asks to move but the car does; the stop bits must not follow it up to speed. Stock keeps
  # STOPPING strictly to the final creep, below 0.55 m/s across all rolling frames.
  drive(sm, 1, stopping=True)
  drive(sm, 100, stopping=True, standstill=True)
  assert sm.holding
  drive(sm, 1, plan_accel=0.0)
  assert not sm.holding and not sm.stop_bits and not sm.resume_unlatching


def test_stop_abort_releases():
  sm = StandstillHold()
  drive(sm, 1, stopping=True)
  assert sm.holding
  # lead speeds up again before the car reaches standstill
  drive(sm, 1, stopping=False, plan_accel=0.3)
  assert not sm.holding


def test_driver_gas_releases_the_hold_without_a_pulse():
  sm = StandstillHold()
  # the driver's pedal outranks the hold, the way Toyota's PCM lets the pedal outrank its
  # standstill request -- but the pulse is the ACC's resume protocol, not the driver's:
  # stock's captured gas-ended hold drops the stop bits with no RESUME_UNLATCHING at all.
  # (The SCBS latch that also motivated this, route 00000103 t+163.8, was the checksum)
  drive(sm, 1, stopping=True)
  drive(sm, 100, stopping=True, standstill=True)
  assert sm.holding
  drive(sm, 1, stopping=True, standstill=True, gas_pressed=True)
  assert not sm.holding and not sm.resume_unlatching, "gas release must not fire the ACC resume pulse"
  # no re-hold while the pedal is down, and a fresh hold once it lifts with the car stopped
  drive(sm, RESUME_UNLATCH_LATCHED_FRAMES + 5, stopping=True, standstill=True, gas_pressed=True)
  assert not sm.holding and not sm.resume_unlatching
  drive(sm, 1, stopping=True, standstill=True)
  assert sm.holding


def test_plan_flap_below_the_debounce_never_releases():
  sm = StandstillHold()
  # the phantom-release shape: at a held standstill the lead inches forward and
  # stops, the plan flapping across zero. Sub-debounce flaps must not release at all, and
  # no frame may ever carry the stop bits and the release pulse together
  drive(sm, 1, stopping=True)
  drive(sm, 100, stopping=True, standstill=True)
  for i in range(600):
    accel = 0.3 if (i // 10) % 2 == 0 else -1.0  # 0.1 s swings, below the 0.2 s debounce
    sm.update(long_engaged=True, stopping=accel < 0, standstill=True, plan_accel=accel,
              brake_hold=False, gas_pressed=False)
    assert not (sm.stop_bits and sm.resume_unlatching), "stop bits and pulse on one frame"
    assert not sm.resume_unlatching, "a sub-debounce flap fired a release pulse"
  assert sm.holding


@pytest.mark.parametrize("brake_hold", [False, True])
def test_slow_flap_never_mixes_stop_bits_with_the_pulse(brake_hold):
  sm = StandstillHold()
  # swings long enough to release each time. Nothing latched (brake_hold False) must never
  # put an unlatch bit on the wire; a body that holds on through every swing falls back to
  # at most one pulse per release, and a re-hold mid-pulse waits it out before re-asserting
  # the stop bits
  drive(sm, 1, stopping=True)
  drive(sm, 100, stopping=True, standstill=True, brake_hold=brake_hold)
  pulses = 0
  prev_unlatch = False
  swing = RELEASE_DEBOUNCE_FRAMES + 30  # long enough for the release and its pulse to play
  for i in range(1200):
    accel = 0.3 if (i // swing) % 2 == 0 else -1.0
    sm.update(long_engaged=True, stopping=accel < 0, standstill=True, plan_accel=accel,
              brake_hold=brake_hold, gas_pressed=False)
    assert not (sm.stop_bits and sm.resume_unlatching), "stop bits and pulse on one frame"
    pulses += int(sm.resume_unlatching and not prev_unlatch)
    prev_unlatch = sm.resume_unlatching
  if brake_hold:
    assert pulses > 0
    assert pulses <= 1 + 1200 // (2 * swing), "more pulses than releases"
  else:
    assert pulses == 0, "a never-latched release put an unlatch bit on the wire"


def test_latched_pulse_runs_to_completion_through_a_re_hold():
  sm = StandstillHold()
  # a latched pulse spans the body's actual unlatch, so a re-hold mid-pulse waits it out
  # (stop bits blocked, stock never emits STOPPING with RESUME_UNLATCHING) instead of
  # canceling it; a second release cannot fire a fresh pulse before the first ends because
  # the release debounce is at least as long as any pulse window
  assert RELEASE_DEBOUNCE_FRAMES >= RESUME_UNLATCH_LATCHED_FRAMES
  drive(sm, 1, stopping=True)
  drive(sm, 100, stopping=True, standstill=True, brake_hold=True)
  drive(sm, RELEASE_DEBOUNCE_FRAMES, standstill=True, brake_hold=True, plan_accel=0.3)
  assert sm.latched_release and sm.resume_unlatching  # the pulse fires with the release
  drive(sm, 3, standstill=True, plan_accel=0.3)
  drive(sm, 1, stopping=True, standstill=True)  # re-hold mid-pulse, body already let go
  assert sm.holding and not sm.stop_bits
  assert sm.resume_unlatching, "a latched pulse mid-release must run to completion"
  drive(sm, RESUME_UNLATCH_LATCHED_FRAMES, stopping=True, standstill=True, plan_accel=-1.0)
  assert sm.holding and sm.stop_bits and not sm.resume_unlatching


def test_missed_pulse_is_retried_exactly_once():
  sm = StandstillHold()
  latched_release(sm)
  # the body ignores the pulse: GEAR.BRAKE_HOLD stays set, plan still positive, car still
  pulses = [0]

  def run(frames):
    prev = sm.resume_unlatching
    for _ in range(frames):
      sm.update(long_engaged=True, stopping=False, standstill=True, plan_accel=0.5, brake_hold=True, gas_pressed=False)
      pulses[0] += sm.resume_unlatching and not prev
      prev = sm.resume_unlatching
  run(RESUME_REPULSE_FRAMES - 2)
  assert pulses[0] == 0, "retried before the window ran out"
  assert not sm.resume_unlatching
  run(4)
  assert pulses[0] == 1, "the retry did not fire at the window"
  assert sm.resume_unlatching and not sm.stop_bits
  run(RESUME_UNLATCH_LATCHED_FRAMES)
  assert not sm.resume_unlatching, "retry pulse outran the latched length"
  # then it gives up: no third pulse however long the body keeps holding
  run(3 * RESUME_REPULSE_FRAMES)
  assert pulses[0] == 1, "a second retry has no stock shape behind it"


def test_answered_pulse_is_not_retried():
  sm = StandstillHold()
  latched_release(sm)
  # the body lets go 2-3 wire frames in, as in every capture; the retry window never fills
  drive(sm, 4, standstill=True, brake_hold=True, plan_accel=0.5)
  drive(sm, 2 * RESUME_REPULSE_FRAMES, standstill=True, brake_hold=False, plan_accel=0.5)
  assert not (sm.resume_unlatching and sm.repulsed)
  assert not sm.repulsed


def test_retry_window_needs_an_unbroken_run_of_the_body_holding():
  sm = StandstillHold()
  latched_release(sm)
  drive(sm, RESUME_REPULSE_FRAMES - 5, standstill=True, brake_hold=True, plan_accel=0.5)
  drive(sm, 1, standstill=True, brake_hold=False, plan_accel=0.5)  # the body did let go
  drive(sm, 10, standstill=True, brake_hold=True, plan_accel=0.5)   # ...then latched again
  assert not sm.repulsed, "a broken run must not count toward the retry"


@pytest.mark.parametrize("override", [dict(gas_pressed=True), dict(stopping=True, plan_accel=-1.0)],
                         ids=["driver_gas", "re_hold"])
def test_no_retry_under_driver_gas_or_a_re_hold(override):
  sm = StandstillHold()
  latched_release(sm)
  drive(sm, RESUME_UNLATCH_LATCHED_FRAMES + 1, standstill=True, brake_hold=True, plan_accel=0.5)
  args = dict(standstill=True, brake_hold=True, plan_accel=0.5)
  args.update(override)
  drive(sm, 2 * RESUME_REPULSE_FRAMES, **args)
  assert not sm.repulsed


def test_long_disengage_forgets_the_retry():
  sm = StandstillHold()
  latched_release(sm)
  drive(sm, 1, long_engaged=False)
  assert sm.latched_frames == 0 and not sm.repulsed and not sm.latched_release
