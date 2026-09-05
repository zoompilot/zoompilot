"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Closed-loop driver-interaction tests for the ICBM + SLA + driver-setpoint stack: long
presses, confirm and decline prompts, and the prompt-freeze overshoot report. See
sla_loop_harness for the co-simulation these run on.
"""
import pytest

from openpilot.sunnypilot.selfdrive.car.tests.sla_loop_harness import (
  ButtonType, EventNameSP, Loop, MPH_MS, PRESS_OFFSETS_S, PlanSource, SlaState)


class TestDriverInteractions:
  def test_settled_longpress_climbs_and_reanchors(self):
    """The most common real exit from a zone: settled at the limit, the driver HOLDS +
    to climb. The ECU snaps along its 5 mph grid (possibly with a trailing step), the
    increments stay suppressed (SLA owned the press), and the setpoint re-anchors to
    wherever the ECU landed, with no servo fight afterward."""
    loop = Loop(baseline_mph=60, seed=12)
    loop.limit_mph = 45
    loop.run(2.0)
    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)
    loop.run(11.0)
    assert loop.ecu.dash == 45

    loop.driver_press(ButtonType.accelCruise, in_seconds=0.1, hold_s=1.3)
    loop.run(6.0)
    assert loop.sla.state == SlaState.inactive
    assert loop.ecu.dash % 5 == 0 and loop.ecu.dash >= 50, f"no grid climb: {loop.ecu.dash}"
    assert loop.v_cruise_mph == loop.ecu.dash, \
      f"setpoint must re-anchor to the ECU result: dash {loop.ecu.dash}, setpoint {loop.v_cruise_mph}"
    dash_settled = loop.ecu.dash
    loop.run(4.0)
    assert loop.ecu.dash == dash_settled, "servo fought the driver's hold result"

  def test_settled_longpress_descends_and_reanchors(self):
    """The mirror exit: settled at the limit, the driver HOLDS - to ride below it. The
    press dismisses the session, the ECU grid-descends, the setpoint re-anchors to the
    result, and the SET- grace keeps the servo from restoring the old baseline over it."""
    loop = Loop(baseline_mph=60, seed=14)
    loop.limit_mph = 45
    loop.run(2.0)
    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)
    loop.run(11.0)
    assert loop.ecu.dash == 45

    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1, hold_s=1.3)
    loop.run(6.0)
    assert loop.sla.state == SlaState.inactive
    assert loop.ecu.dash % 5 == 0 and loop.ecu.dash <= 40, f"no grid descent: {loop.ecu.dash}"
    assert loop.v_cruise_mph == loop.ecu.dash, \
      f"setpoint must re-anchor to the ECU result: dash {loop.ecu.dash}, setpoint {loop.v_cruise_mph}"
    dash_settled = loop.ecu.dash
    loop.run(4.0)
    assert loop.ecu.dash == dash_settled, "servo fought the driver's hold result"

  def test_up_confirm_adopts_limit(self):
    """Drive 0000000b t=415/461: cruising below a rising limit, + on the prompt must take
    the setpoint and the dash TO the limit, not leave a +1 orphan with an inert session
    (min() source selection can never let an above-setpoint SLA target win)."""
    loop = Loop(baseline_mph=40, seed=20)
    loop.limit_mph = 45
    loop.run(2.0)
    assert loop.sla.state == SlaState.preActive, loop.sla.state

    loop.driver_press(ButtonType.accelCruise, in_seconds=0.1)
    loop.run(1.0)
    assert loop.sla.state == SlaState.active, loop.sla.state
    assert loop.v_cruise_mph == 45, f"setpoint must adopt the confirmed limit: {loop.v_cruise_mph}"
    assert any(e == EventNameSP.speedLimitActive for _, e in loop.sla_events), \
      "an explicit up-confirm must announce the adjustment"

    loop.run(10.0)
    assert loop.ecu.dash == 45, f"dash never walked up to the limit: {loop.ecu.dash}"
    assert loop.sla.state == SlaState.active
    assert loop.v_cruise_mph == 45

  def test_up_confirm_keeps_higher_baseline(self):
    """Zone reopens mid-session: settled at 40 under a 48 baseline, limit rises to 45,
    the confirm walks the dash up to 45 but the 48 baseline survives (the session caps
    the plan; the setpoint is only ever raised toward the limit, never lowered by it)."""
    loop = Loop(baseline_mph=48, seed=21)
    loop.limit_mph = 40
    loop.run(2.0)
    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)
    loop.run(11.0)
    assert loop.ecu.dash == 40

    loop.limit_mph = 45
    loop.run(1.0)
    assert loop.sla.state == SlaState.preActive
    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)  # cluster 48 > 45: confirm is -
    loop.run(12.0)
    assert loop.sla.state == SlaState.active, loop.sla.state
    assert loop.ecu.dash == 45, f"dash: {loop.ecu.dash}"
    assert loop.v_cruise_mph == 48, f"baseline corrupted: {loop.v_cruise_mph}"

  def test_pre_active_holds_dash_until_answered(self):
    """Drive 0000000b t=180.8: limit rises mid-session and ICBM restored the dash toward
    the baseline while the confirm prompt was still showing. The prompt must freeze the
    plan: no un-confirmed acceleration; the restore may only run after the timeout."""
    loop = Loop(baseline_mph=48, seed=22)
    loop.limit_mph = 40
    loop.run(2.0)
    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)
    loop.run(11.0)
    assert loop.ecu.dash == 40

    loop.limit_mph = 45
    def frozen(lo):
      if lo.sla.state == SlaState.preActive:
        assert lo.ecu.dash <= 41, f"dash restored during the prompt: {lo.ecu.dash}"
    loop.run(4.9, assert_each=frozen)
    assert loop.sla.state == SlaState.preActive, loop.sla.state
    loop.run(12.0)  # timeout -> inactive -> quiet window -> restore to baseline
    assert loop.sla.state == SlaState.inactive
    assert loop.ecu.dash == 48, f"restore after timeout stopped short: {loop.ecu.dash}"

  def test_pre_active_decline_by_opposite_press(self):
    """A release against the confirm direction declines the prompt: the session ends at
    once (no lingering hold shadowing the driver's dialing) and the press still counts
    as a normal increment."""
    loop = Loop(baseline_mph=50, seed=23)
    loop.limit_mph = 35
    loop.run(2.0)
    assert loop.sla.state == SlaState.preActive  # confirm would be -

    loop.driver_press(ButtonType.accelCruise, in_seconds=0.1)
    loop.run(1.0)
    assert loop.sla.state == SlaState.inactive, loop.sla.state
    assert loop.v_cruise_mph == 51, f"declining press must still increment: {loop.v_cruise_mph}"
    assert not any(e == EventNameSP.speedLimitActive for _, e in loop.sla_events)

  def test_engage_on_limit_is_silent(self):
    """Drive 0000000b t=155.1: resuming with the setpoint already at the limit fired
    'Auto adjusting to speed limit'. Activation that changes nothing must be silent."""
    loop = Loop(baseline_mph=45, seed=24)
    loop.limit_mph = 45
    loop.run(3.0)
    assert loop.sla.state == SlaState.active, loop.sla.state
    assert not loop.sla_events, f"silent activation expected: {loop.sla_events}"

  def test_dial_to_target_activates_silently_and_sticks(self):
    """Drive 0000000b t=187.05: dialing onto the limit activated SLA with an alert and
    the same press's latch dismissed it one frame later. It must latch silently and
    survive its own activating press."""
    loop = Loop(baseline_mph=43, seed=25)
    loop.limit_mph = 45
    loop.run(2.0)
    assert loop.sla.state == SlaState.preActive
    loop.run(6.0)  # let the prompt time out (driver ignores it)
    assert loop.sla.state == SlaState.inactive

    loop.sla_events.clear()
    loop.driver_press(ButtonType.accelCruise, in_seconds=0.1)
    loop.run(1.0)
    loop.driver_press(ButtonType.accelCruise, in_seconds=0.1)
    loop.run(2.0)
    assert loop.v_cruise_mph == 45, loop.v_cruise_mph
    assert loop.sla.state == SlaState.active, f"dial-to-target must latch: {loop.sla.state}"
    states = set()
    loop.run(3.0, assert_each=lambda lo: states.add(lo.sla.state))
    assert states == {SlaState.active}, f"activation did not stick: {states}"
    assert not any(e == EventNameSP.speedLimitActive for _, e in loop.sla_events), \
      "dial-to-target activation must be silent"

  def test_decline_waits_full_quiet_window_before_restore(self):
    """The prompt must not pre-pay the servo's patience: after a decline, the restore
    toward the (incremented) baseline starts only after a FULL quiet window, giving
    card time to settle the decline press's own effects first."""
    loop = Loop(baseline_mph=48, seed=27)
    loop.limit_mph = 40
    loop.run(2.0)
    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)
    loop.run(11.0)
    assert loop.ecu.dash == 40

    loop.limit_mph = 45
    loop.run(1.0)
    assert loop.sla.state == SlaState.preActive
    loop.driver_press(ButtonType.accelCruise, in_seconds=0.1)  # against the - confirm: decline
    loop.run(0.5)
    assert loop.sla.state == SlaState.inactive, loop.sla.state
    assert loop.v_cruise_mph == 49, f"declining press must still increment: {loop.v_cruise_mph}"

    dash_at_decline = loop.ecu.dash
    loop.run(0.3)  # still inside the quiet window (1 s, counted from the decline)
    assert loop.ecu.dash <= dash_at_decline + 1, \
      f"restore began inside the quiet window: {loop.ecu.dash} from {dash_at_decline}"
    loop.run(12.0)
    assert loop.ecu.dash == 49, f"restore never completed: {loop.ecu.dash}"

  def test_no_emission_escapes_at_prompt_onset(self):
    """The servo's own freeze is one hop stale; card's same-frame veto must stop any
    button frame from reaching the ECU from the first prompting frame on."""
    loop = Loop(baseline_mph=48, seed=28)
    loop.limit_mph = 40
    loop.run(2.0)
    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)
    loop.run(11.0)
    assert loop.ecu.dash == 40

    loop.limit_mph = 45
    def frozen(lo):
      if lo.helper.cruise_arbiter.prompting:
        assert lo.ecu.dash == 40, f"dash moved during the prompt: {lo.ecu.dash}"
    loop.run(4.9, assert_each=frozen)
    assert loop.sla.state == SlaState.preActive

  @pytest.mark.parametrize("offset", PRESS_OFFSETS_S, ids=lambda o: f"{o:.2f}s")
  def test_up_confirm_press_at_any_phase_converges(self, offset):
    """The up-confirm press swept across the 20 Hz SLA cycle: at every phase the outcome
    must be the full adoption (setpoint == dash == limit, session active), never the
    logged hybrid of a +1 increment with an inert active session."""
    loop = Loop(baseline_mph=40, seed=26)
    loop.limit_mph = 45
    loop.run(2.0 + offset)
    assert loop.sla.state == SlaState.preActive
    loop.driver_press(ButtonType.accelCruise, in_seconds=0.01)
    loop.run(12.0)
    assert loop.sla.state == SlaState.active, f"offset {offset}: {loop.sla.state}"
    assert loop.v_cruise_mph == 45, f"offset {offset}: setpoint {loop.v_cruise_mph}"
    assert loop.ecu.dash == 45, f"offset {offset}: dash {loop.ecu.dash}"

  def test_press_during_scc_dip_with_sla_session(self):
    """Two limiters overlapping: settled SLA session, then a curve dips below it. A +
    press dismisses the SLA session but must NOT lift the curve limit or corrupt the
    baseline; once the dip clears, the restore goes all the way to the baseline (the
    dismissed session must not re-grab at 45)."""
    loop = Loop(baseline_mph=60, seed=13)
    loop.limit_mph = 45
    loop.run(2.0)
    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)
    loop.run(11.0)
    assert loop.ecu.dash == 45

    loop.scc_dip_mph = 40
    loop.run(5.0)
    assert loop.ecu.dash <= 41, f"dash never followed the dip: {loop.ecu.dash}"

    loop.driver_press(ButtonType.accelCruise, in_seconds=0.1)
    loop.run(2.0)
    assert loop.sla.state == SlaState.inactive
    assert loop.ecu.dash <= 42, "the press must not lift the still-active curve limit"
    assert loop.v_cruise_mph == 60, f"baseline corrupted: {loop.v_cruise_mph}"

    loop.scc_dip_mph = 0.
    loop.run(14.0)
    assert loop.ecu.dash == 60, f"restore stopped short: {loop.ecu.dash}"
    assert loop.v_cruise_mph == 60


class TestPromptFreezeOvershoot:
  """User report 2026-08-29 (routes ...acdc83b60f/3 and ...821e28d2fa/12): engaging near a
  known limit dropped the set speed 2-4 mph roughly 5 s later, then walked it back.

  The prompt's own freeze produced it. Capping the plan at the cluster round-tripped
  through whole mph and landed ~7 mm/s under v_cruise, so SLA won the plan min() by
  rounding error and relabelled the source as a limiter. That armed decel overshoot
  against an ordinary cruise convergence, the servo's freeze banked the resulting gap
  for the whole 5 s window, and the timeout dumped it as a SET- burst."""

  def test_ignored_prompt_never_moves_the_dash(self):
    """The reported drive: engaged at 40 with a 45 target, car 1.3 mph over the setpoint,
    prompt left unanswered. Nothing may move -- during the prompt or after it times out."""
    loop = Loop(baseline_mph=40, seed=31)
    loop.decel_overshoot = True
    loop.v_ego_mph = 41.3
    loop.a_target = -0.5  # the plan converging on the setpoint the car is sitting above
    loop.limit_mph = 45
    loop.run(1.0)
    assert loop.sla.state == SlaState.preActive

    def never_moves(lo):
      assert lo.ecu.dash == 40, f"dash moved at tick {lo.tick_n}: {lo.ecu.dash}"
      assert lo.servo.overshoot_mph == 0., f"overshoot banked behind the freeze: {lo.servo.overshoot_mph}"

    loop.run(10.0, assert_each=never_moves)  # 5 s prompt + timeout + the restore window
    assert loop.sla.state == SlaState.inactive
    assert loop.v_cruise_mph == 40

  def test_prompt_does_not_relabel_the_plan_source(self):
    """Layer 1 in isolation: prompting from idle must leave the plan on `cruise`. A cap
    equal to the baseline changes no speed but does change the source, and the source is
    what arms the overshoot."""
    loop = Loop(baseline_mph=40, seed=32)
    loop.v_ego_mph = 41.3
    loop.limit_mph = 45
    loop.run(1.0)
    assert loop.sla.state == SlaState.preActive

    def stays_cruise(lo):
      if lo.sla.prompting:
        assert lo._lp_sp().longitudinalPlanSource == PlanSource.cruise, "prompt relabelled the plan source"

    loop.run(4.0, assert_each=stays_cruise)

  def test_session_hold_still_freezes_an_active_session(self):
    """Layer 1 must not cost the freeze its real job: prompting OUT OF an active session
    still holds that session's cap, so the dash cannot restore un-confirmed."""
    loop = Loop(baseline_mph=48, seed=33)
    loop.limit_mph = 40
    loop.run(2.0)
    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)
    loop.run(11.0)
    assert loop.ecu.dash == 40

    loop.limit_mph = 45  # a limit change out of an active session re-prompts
    loop.run(0.5)
    assert loop.sla.state == SlaState.preActive
    assert loop.sla.v_cap < 45 * MPH_MS, f"active-session hold released: {loop.sla.v_cap}"
