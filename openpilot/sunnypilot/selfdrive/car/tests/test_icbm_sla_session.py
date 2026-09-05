"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Closed-loop session tests for the ICBM + SLA + driver-setpoint stack: curve restores,
the SLA session lifecycle and the press timing sweeps. See sla_loop_harness for the
co-simulation these run on.
"""
import pytest

from openpilot.sunnypilot.selfdrive.car.tests.sla_loop_harness import (
  ButtonType, Loop, MPH_MS, PRESS_OFFSETS_S, SlaState)


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
  def _confirm_lower(self, loop, limit):
    loop.limit_mph = limit
    loop.run(2.0)  # disabled->preActive engagement path
    assert loop.sla.state == SlaState.preActive, loop.sla.state
    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)
    loop.run(1.0)
    assert loop.sla.state == SlaState.active, loop.sla.state

  def test_confirm_sticks_and_dash_reaches_limit(self):
    """F1 end-to-end: one - press confirms; SLA must stay active while ICBM walks the
    dash all the way to the limit (hold + taps), and the baseline must survive."""
    loop = Loop(baseline_mph=60, seed=3)
    self._confirm_lower(loop, limit=45)

    states = set()
    def watch(lo):
      states.add(lo.sla.state)
    loop.run(10.0, assert_each=watch)
    assert loop.ecu.dash == 45, f"dash never reached the limit: {loop.ecu.dash}"
    assert states == {SlaState.active}, f"SLA flickered: {states}"
    assert loop.v_cruise_mph == 60, f"baseline corrupted: {loop.v_cruise_mph}"

  def test_settled_press_reanchors(self):
    """Settled at the limit, one + press: SLA steps aside, the ECU's +1 becomes the new
    setpoint, and the servo must NOT drag the dash back to the old baseline."""
    loop = Loop(baseline_mph=60, seed=4)
    self._confirm_lower(loop, limit=45)
    loop.run(10.0)
    assert loop.ecu.dash == 45

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
    self._confirm_lower(loop, limit=45)

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
    self._confirm_lower(loop, limit=45)

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

  def test_holds_read_as_taps_still_reaches_limit(self):
    """An ECU that registers synthesized holds as paced presses: same net progress as
    taps and no fault needed; the session still lands the limit."""
    loop = Loop(baseline_mph=60, seed=6)
    self._confirm_lower(loop, limit=45)

    loop.run(15.0)
    assert loop.ecu.dash == 45, f"dash never landed: {loop.ecu.dash}"
    assert loop.sla.state == SlaState.active

  def test_holds_ignored_faults_and_taps_land(self):
    """An ECU that rejects synthesized holds outright: zero movement must trip the
    long-press fallback, and the session still lands the limit on taps."""
    loop = Loop(baseline_mph=60, seed=7, forged_mode='ignored')
    self._confirm_lower(loop, limit=45)

    loop.run(15.0)
    assert loop.servo.fast_faulted
    assert loop.ecu.dash == 45, f"taps fallback never landed: {loop.ecu.dash}"
    assert loop.sla.state == SlaState.active


class TestPressTimingSweeps:
  """Every shipped bug in this stack was a single driver press racing the 20 Hz SLA
  cycle, the reconcile window, or the servo state. These sweeps land the same press at
  offsets spanning more than one full SLA cycle and assert the outcome INVARIANTS:
  the system must converge to one coherent state at every phase, never a hybrid."""

  def _settled_session(self, seed):
    loop = Loop(baseline_mph=60, seed=seed)
    loop.limit_mph = 45
    loop.run(2.0)
    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)
    loop.run(1.0)
    assert loop.sla.state == SlaState.active
    loop.run(10.0)
    assert loop.ecu.dash == 45
    return loop

  @pytest.mark.parametrize("offset", PRESS_OFFSETS_S, ids=lambda o: f"{o:.2f}s")
  def test_settled_press_at_any_phase_reanchors(self, offset):
    loop = self._settled_session(seed=10)
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
    loop.limit_mph = 45
    loop.run(2.0)
    loop.driver_press(ButtonType.decelCruise, in_seconds=0.1)
    loop.run(1.0)
    assert loop.sla.state == SlaState.active

    loop.run(0.9 + offset)  # somewhere in the walk
    dash_at_press = loop.ecu.dash
    loop.driver_press(ButtonType.accelCruise, in_seconds=0.01)
    loop.run(14.0)

    assert loop.sla.state == SlaState.inactive, f"offset {offset}"
    assert loop.v_cruise_mph == loop.ecu.dash, \
      f"offset {offset}: diverged (dash {loop.ecu.dash}, setpoint {loop.v_cruise_mph})"
    if abs(dash_at_press - 45) > 3:
      assert loop.ecu.dash == 60, f"offset {offset}: baseline not restored from {dash_at_press}: {loop.ecu.dash}"
