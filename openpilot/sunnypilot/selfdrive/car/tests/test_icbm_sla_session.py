"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Closed-loop tests for the ICBM + SLA + driver-setpoint stack: curve restores, the SLA session
lifecycle, the driver's presses, long presses and prompt answers, the press timing sweeps and
the prompt-freeze overshoot report. See sla_loop_harness for the co-simulation these run on.
"""
import pytest

from openpilot.sunnypilot.selfdrive.car.tests.sla_loop_harness import (
  ButtonType, EventNameSP, Loop, MPH_MS, PRESS_OFFSETS_S, PlanSource, SlaState)


def confirm_lower(loop, limit):
  """A limit below the setpoint appears, the prompt shows, and one - press confirms it."""
  loop.limit_mph = limit
  loop.run(2.0)  # disabled->preActive engagement path
  assert loop.sla.state == SlaState.preActive, loop.sla.state
  loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)
  loop.run(1.0)
  assert loop.sla.state == SlaState.active, loop.sla.state


def settle_in_zone(loop, limit):
  """confirm_lower, then ICBM walks the dash down to the limit."""
  confirm_lower(loop, limit)
  loop.run(10.0)
  assert loop.ecu.dash == limit, f"dash never reached the limit: {loop.ecu.dash}"


class TestCurveRestore:
  def test_dip_restores_exactly(self):
    """F2 end-to-end: an SCC dip walks the dash down; after it clears, the dash comes back
    to exactly the driver's baseline, across ECU press drops and grid snaps."""
    loop = Loop(baseline_mph=60, seed=1)
    loop.scc_dip_mph = 55
    loop.run(6.0)
    assert loop.ecu.dash <= 56, f"dash never followed the dip: {loop.ecu.dash}"

    loop.scc_dip_mph = 0.
    loop.run(12.0)  # quiet window + restore move + latency
    assert loop.ecu.dash == 60, f"restore not exact: dash={loop.ecu.dash}"
    assert loop.v_cruise_mph == 60, f"baseline corrupted: {loop.v_cruise_mph}"

  def test_dip_train_does_not_churn(self):
    """Back-to-back dips with the horizon in view: the lookahead veto must hold the
    dash down through the gap (the second dip is visible before its source commits)."""
    loop = Loop(baseline_mph=60, seed=2)
    loop.lookahead_mph = 55  # the vision profile sees the dip train the whole time
    loop.scc_dip_mph = 55
    loop.run(5.0)
    dash_after_first = loop.ecu.dash

    loop.scc_dip_mph = 0.
    loop.run(1.5)  # gap between commits; the dip is still on the horizon
    assert loop.ecu.dash == dash_after_first, "servo restored between back-to-back dips"
    loop.scc_dip_mph = 55
    loop.run(3.0)
    loop.scc_dip_mph = 0.
    loop.lookahead_mph = 255. / MPH_MS  # horizon clear
    loop.run(12.0)
    assert loop.ecu.dash == 60


class TestSlaSession:
  @pytest.mark.parametrize("op_long", [False, True], ids=["stock_acc", "alpha_long"])
  def test_confirm_sticks_and_dash_reaches_limit(self, op_long):
    """F1 end-to-end: one - press confirms; SLA must stay active while ICBM walks the
    dash all the way to the limit (hold + taps), and the baseline must survive. Under
    Mazda alpha long the body keeps the setpoint, so the session walks it the same way."""
    loop = Loop(baseline_mph=60, seed=3, op_long=op_long)
    confirm_lower(loop, limit=45)

    states = set()
    loop.run(10.0, assert_each=lambda lo: states.add(lo.sla.state))
    assert loop.ecu.dash == 45, f"dash never reached the limit: {loop.ecu.dash}"
    assert states == {SlaState.active}, f"SLA flickered: {states}"
    assert loop.v_cruise_mph == 60, f"baseline corrupted: {loop.v_cruise_mph}"

  def test_settled_press_reanchors(self):
    """Settled at the limit, one + press: SLA steps aside, the ECU's +1 becomes the new
    setpoint, and the servo must NOT drag the dash back to the old baseline."""
    loop = Loop(baseline_mph=60, seed=4)
    settle_in_zone(loop, limit=45)

    loop.driver_press(ButtonType.accelCruise, in_seconds=0.1)
    loop.run(5.0)
    assert loop.sla.state == SlaState.inactive, loop.sla.state
    assert loop.ecu.dash == 46, f"dash: {loop.ecu.dash}"
    assert round(loop.v_cruise_mph) == 46, f"setpoint must re-anchor to 46: {loop.v_cruise_mph}"

  @pytest.mark.parametrize("is_metric", [False, True], ids=["mph", "kph"])
  def test_mid_move_abort_restores_baseline(self, is_metric):
    """+ while ICBM is still walking down: session aborts and the servo restores the
    exact baseline; the driver is never stranded mid-way (the upstream failure mode)."""
    loop = Loop(baseline_mph=60, seed=5, is_metric=is_metric)
    confirm_lower(loop, limit=45)

    loop.run(1.2)  # servo mid-move, dash somewhere between 60 and 45
    assert 45 < loop.ecu.dash < 60, loop.ecu.dash
    loop.driver_press(ButtonType.accelCruise, in_seconds=0.05)
    loop.run(14.0)  # abort + quiet window + restore
    assert loop.sla.state == SlaState.inactive
    assert loop.ecu.dash == 60, f"metric={is_metric}: baseline not restored: {loop.ecu.dash}"
    assert loop.v_cruise_mph == 60, f"metric={is_metric}: setpoint corrupted: {loop.v_cruise_mph}"

  @pytest.mark.parametrize("is_metric", [False, True], ids=["mph", "kph"])
  def test_mid_move_minus_dismiss_never_restores_upward(self, is_metric):
    """- while ICBM is still walking down: the driver asked for slower. The session ends
    and the ECU's own -1 lands, and from there the dash must never climb: the
    in-transit dash failed the reconciler's agreement checks, nothing re-anchored, and
    3 s later the servo walked 51 -> 60 inside the 45 zone. The setpoint floors to the
    dash once the press settles and the servo has nothing to restore."""
    loop = Loop(baseline_mph=60, seed=5, is_metric=is_metric)
    confirm_lower(loop, limit=45)

    loop.run(1.2)  # servo mid-move
    assert 45 < loop.ecu.dash < 60, loop.ecu.dash
    loop.driver_press(ButtonType.decelCruise, in_seconds=0.05)
    loop.run(0.35)  # press + ECU latency: the driver's -1 (if registered) has landed
    dash_after_press = loop.ecu.dash
    assert dash_after_press <= 52, dash_after_press

    def never_up(lo, ceiling=dash_after_press, m=is_metric):
      assert lo.ecu.dash <= ceiling, f"metric={m}: servo restored upward at {lo.tick_n / 100.:.2f}s: {lo.ecu.dash} > {ceiling}"
    loop.run(14.0, assert_each=never_up)  # grace + quiet window + whatever restore would follow
    assert loop.sla.state == SlaState.inactive
    assert loop.v_cruise_mph == loop.ecu.dash, \
      f"metric={is_metric}: setpoint {loop.v_cruise_mph} left off the dash {loop.ecu.dash}"

  @pytest.mark.parametrize("forged_mode, seed", [('taps', 6), ('ignored', 7)])
  def test_synthesized_holds_still_reach_limit(self, forged_mode, seed):
    """An ECU that registers synthesized holds as paced presses makes the same net progress
    as taps with no fault; one that rejects them outright (zero movement) must trip the
    long-press fallback. Either way the session lands the limit."""
    loop = Loop(baseline_mph=60, seed=seed, forged_mode=forged_mode)
    confirm_lower(loop, limit=45)

    loop.run(15.0)
    assert loop.servo.fast_faulted == (forged_mode == 'ignored')
    assert loop.ecu.dash == 45, f"dash never landed: {loop.ecu.dash}"
    assert loop.sla.state == SlaState.active


class TestDriverInteractions:
  @pytest.mark.parametrize("button, seed", [(ButtonType.accelCruise, 12), (ButtonType.decelCruise, 14)], ids=["climb", "descend"])
  def test_settled_longpress_reanchors(self, button, seed):
    """The most common real exit from a zone: settled at the limit, the driver HOLDS + to
    climb (or - to ride below it, which dismisses the session). The ECU snaps along its
    5 mph grid (possibly with a trailing step), the increments stay suppressed (SLA owned
    the press), the setpoint re-anchors to wherever the ECU landed, and neither the servo
    nor the SET- grace's restore fights it afterward."""
    loop = Loop(baseline_mph=60, seed=seed)
    settle_in_zone(loop, limit=45)

    loop.driver_press(button, in_seconds=0.1, hold_s=1.3)
    loop.run(6.0)
    assert loop.sla.state == SlaState.inactive
    landed = loop.ecu.dash >= 50 if button == ButtonType.accelCruise else loop.ecu.dash <= 40
    assert loop.ecu.dash % 5 == 0 and landed, f"no grid step: {loop.ecu.dash}"
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
    settle_in_zone(loop, limit=40)

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
    settle_in_zone(loop, limit=40)

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
    settle_in_zone(loop, limit=40)

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
    settle_in_zone(loop, limit=40)

    loop.limit_mph = 45
    def frozen(lo):
      if lo.helper.cruise_arbiter.prompting:
        assert lo.ecu.dash == 40, f"dash moved during the prompt: {lo.ecu.dash}"
    loop.run(4.9, assert_each=frozen)
    assert loop.sla.state == SlaState.preActive

  def test_press_during_scc_dip_with_sla_session(self):
    """Two limiters overlapping: settled SLA session, then a curve dips below it. A +
    press dismisses the SLA session but must NOT lift the curve limit or corrupt the
    baseline; once the dip clears, the restore goes all the way to the baseline (the
    dismissed session must not re-grab at 45)."""
    loop = Loop(baseline_mph=60, seed=13)
    settle_in_zone(loop, limit=45)

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


class TestPressTimingSweeps:
  """Every shipped bug in this stack was a single driver press racing the 20 Hz SLA
  cycle, the reconcile window, or the servo state. These sweeps land the same press at
  offsets spanning more than one full SLA cycle and assert the outcome INVARIANTS:
  the system must converge to one coherent state at every phase, never a hybrid."""

  @pytest.mark.parametrize("offset", PRESS_OFFSETS_S, ids=lambda o: f"{o:.2f}s")
  def test_settled_press_at_any_phase_reanchors(self, offset):
    loop = Loop(baseline_mph=60, seed=10)
    settle_in_zone(loop, limit=45)
    loop.run(offset)
    loop.driver_press(ButtonType.accelCruise, in_seconds=0.01)
    loop.run(6.0)
    assert loop.sla.state == SlaState.inactive, f"offset {offset}"
    assert loop.ecu.dash == 46, f"offset {offset}: dash {loop.ecu.dash}"
    assert loop.v_cruise_mph == 46, f"offset {offset}: setpoint {loop.v_cruise_mph}"

  @pytest.mark.parametrize("offset", PRESS_OFFSETS_S, ids=lambda o: f"{o:.2f}s")
  def test_mid_move_press_at_any_phase_converges(self, offset):
    """Abort mid-walk at every phase. Deep in the walk the baseline must survive and
    restore exactly; within the 2 mph agreement band of the limit the press counts as
    settled and re-anchors; either way the system converges (setpoint == dash) and the
    baseline is never left corrupted at some in-between value."""
    loop = Loop(baseline_mph=60, seed=11)
    confirm_lower(loop, limit=45)

    loop.run(0.9 + offset)  # somewhere in the walk
    dash_at_press = loop.ecu.dash
    loop.driver_press(ButtonType.accelCruise, in_seconds=0.01)
    loop.run(14.0)

    assert loop.sla.state == SlaState.inactive, f"offset {offset}"
    assert loop.v_cruise_mph == loop.ecu.dash, \
      f"offset {offset}: diverged (dash {loop.ecu.dash}, setpoint {loop.v_cruise_mph})"
    if abs(dash_at_press - 45) > 3:
      assert loop.ecu.dash == 60, f"offset {offset}: baseline not restored from {dash_at_press}: {loop.ecu.dash}"

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


class TestAlphaLong:
  """Mazda alpha long: openpilot brakes, the body keeps the setpoint. The speed limit
  session walks the dash exactly as under stock cruise (TestSlaSession); a curve does not
  touch the dash because the planner slows the car itself."""

  @pytest.mark.parametrize("lookahead", [None, 30])
  def test_curve_leaves_the_dash_alone(self, lookahead):
    loop = Loop(baseline_mph=60, seed=1, op_long=True)
    loop.scc_dip_mph = 45
    loop.lookahead_mph = lookahead
    loop.run(8.0)
    assert loop.ecu.dash == 60, f"the dash followed a curve under openpilot longitudinal: {loop.ecu.dash}"
    assert loop.v_cruise_mph == 60

  @pytest.mark.parametrize("lookahead", [None, 30])
  def test_curve_inside_a_zone_holds_the_limit(self, lookahead):
    # the production case: SCC Vision's lookahead sees the curve, the zone cap must hold
    loop = Loop(baseline_mph=60, seed=3, op_long=True)
    settle_in_zone(loop, limit=45)
    loop.scc_dip_mph = 30
    loop.lookahead_mph = lookahead
    dashes = set()
    loop.run(8.0, assert_each=lambda lo: dashes.add(lo.ecu.dash))
    assert dashes == {45}, f"the dash left the zone cap during a curve: {sorted(dashes)}"
    loop.scc_dip_mph = 0.
    loop.lookahead_mph = None
    loop.run(4.0)
    assert loop.ecu.dash == 45
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
    settle_in_zone(loop, limit=40)

    loop.limit_mph = 45  # a limit change out of an active session re-prompts
    loop.run(0.5)
    assert loop.sla.state == SlaState.preActive
    assert loop.sla.v_cap < 45 * MPH_MS, f"active-session hold released: {loop.sla.v_cap}"
