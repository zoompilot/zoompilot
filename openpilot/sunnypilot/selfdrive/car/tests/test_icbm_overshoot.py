"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

ICBM deceleration overshoot: the down-only lever that holds the dash below vEgo so the
stock ACC delivers the limiter's requested deceleration, and the gates that keep it a
lever rather than a destination.
"""
from openpilot.cereal import custom
from openpilot.sunnypilot.selfdrive.car.tests.icbm_servo_harness import make_icbm, run_frames

State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState
SessionState = custom.LongitudinalPlanSP.SpeedLimit.AssistState


class TestDecelOvershoot:
  """The dash is held the gap below vEgo that makes the stock ACC deliver the limiter's own
  requested decel (its decel follows the dash-vs-vEgo gap in stages and keeps growing past
  10 mph), bounded by the plan target on the way in."""

  def make_icbm(self, brand="mazda"):
    return make_icbm(brand)

  def run_frames(self, icbm, target_mph, v_ego_mph, a_target, n=1, source='sccVision', mpc_a_target=None, dash_mph=None):
    run_frames(icbm, target_mph, target_mph if dash_mph is None else dash_mph, n=n, source=source, v_ego_mph=v_ego_mph,
               a_target=a_target, mpc_a_target=mpc_a_target)

  def test_gap_is_sized_by_the_request(self):
    """A gentle request holds a small gap and tracks vEgo; it does not walk to a deep target."""
    icbm = self.make_icbm()
    self.run_frames(icbm, target_mph=30, v_ego_mph=45, a_target=-0.3, n=100, dash_mph=45)
    assert icbm.v_target == 42, icbm.v_target  # 45 - 3.0

  def test_deep_dip_brakes_at_budget_not_at_depth(self):
    """Routes 24c/24d/128: walking the dash to a dip 20-30 mph down opened a 15-20 mph gap and
    the ECU braked at -1.0 to -1.15 m/s^2. The gap stops at the stock budget's."""
    icbm = self.make_icbm()
    self.run_frames(icbm, target_mph=20, v_ego_mph=45, a_target=-0.75, n=100, dash_mph=45)
    assert icbm.v_target == 35, icbm.v_target  # 45 - 10

  def test_a_late_request_uses_the_ecus_remaining_range(self):
    """Past the budget the planner is saying the car is late; the gap goes on past 10 mph,
    where the ECU still brakes harder (-1.05 m/s^2 at 20 mph)."""
    icbm = self.make_icbm()
    self.run_frames(icbm, target_mph=20, v_ego_mph=45, a_target=-1.05, n=300, dash_mph=45)
    assert icbm.v_target == 25, icbm.v_target  # 45 - 20.25

  def test_highway_gap_is_smaller(self):
    """The ECU brakes harder per mph of gap at highway speed; the same request asks less."""
    icbm = self.make_icbm()
    self.run_frames(icbm, target_mph=40, v_ego_mph=65, a_target=-0.75, n=100, dash_mph=65)
    assert icbm.v_target == 57, icbm.v_target  # 65 - 7.75

  def test_reads_the_limiters_request_not_the_mpc(self):
    """LP_SP.aTarget is the MPC output and rails at -1.2 whenever the target sits below vEgo."""
    icbm = self.make_icbm()
    self.run_frames(icbm, target_mph=30, v_ego_mph=45, a_target=-0.3, n=100, mpc_a_target=-1.2, dash_mph=45)
    assert icbm.v_target == 42, icbm.v_target

  def test_lands_on_the_target(self):
    """Near the target the dash sits no further below it than the car is above it, so the
    ECU's lag does not carry the car 4-5 mph under the curve's speed."""
    icbm = self.make_icbm()
    self.run_frames(icbm, target_mph=40, v_ego_mph=42, a_target=-0.7, n=100)
    assert icbm.v_target == 38, icbm.v_target  # 42 - 8.5 = 33.5, floored at 40 - 2

  def test_never_raises_the_dash_above_the_target(self):
    """With the dash already on the target a gentle request asks for a gap the car already
    has; the servo holds rather than walking the dash up over the plan."""
    icbm = self.make_icbm()
    self.run_frames(icbm, target_mph=40, v_ego_mph=45, a_target=-0.3, n=100, dash_mph=40)
    assert icbm.v_target == 40, icbm.v_target

  def test_never_above_the_target_once_below_it(self):
    icbm = self.make_icbm()
    self.run_frames(icbm, target_mph=40, v_ego_mph=45, a_target=-0.7, n=100)
    self.run_frames(icbm, target_mph=40, v_ego_mph=39, a_target=-0.7, n=50)
    assert icbm.v_target == 40, icbm.v_target

  def test_releases_back_to_target(self):
    icbm = self.make_icbm()
    self.run_frames(icbm, target_mph=40, v_ego_mph=45, a_target=-0.45, n=100)
    assert icbm.v_target < 45
    # decel demand ends; command must return to the target (slew-limited release)
    self.run_frames(icbm, target_mph=40, v_ego_mph=40, a_target=0.0, n=400)
    assert icbm.v_target == 40, icbm.v_target

  def test_cruise_source_never_overshoots(self):
    icbm = self.make_icbm()
    self.run_frames(icbm, target_mph=40, v_ego_mph=45, a_target=-0.45, n=100, source='cruise')
    assert icbm.v_target == 40, icbm.v_target

  def test_mazda_only(self):
    icbm = self.make_icbm(brand="hyundai")
    self.run_frames(icbm, target_mph=40, v_ego_mph=45, a_target=-0.45, n=100)
    assert icbm.v_target == 40, icbm.v_target



class TestDecelOvershootIsALever:
  """The overshoot commands the dash BELOW vEgo to buy real decel from the stock ACC. It
  is a lever the servo pulls, not a destination, so it is only valid while the servo can
  actually pull it and while the limiter that asked for it is still live. Both halves
  were missing: a pending SLA confirm prompt banked a gap for its whole 5 s window and
  the timeout dumped it as a SET- burst (user report 2026-08-29)."""

  def make_icbm(self):
    return make_icbm("mazda")

  def run_frames(self, *args, icbm, **kwargs):
    return run_frames(icbm, *args, **kwargs)

  def test_a_prompt_held_dash_is_not_a_stalled_stream(self):
    """card's gate alone keeps a restore off the dash while a confirm prompt is open
    (cruise_arbiter), so the dash stands still under the servo's up stream. That must not read
    as a stalled stream and fall back to taps for the drive; the restore goes on once the
    prompt closes."""
    icbm = self.make_icbm()
    self.run_frames(30, 30, n=60, icbm=icbm, v_ego_mph=30.)
    self.run_frames(45, 30, n=500, icbm=icbm, source='cruise', v_ego_mph=30., session_state=SessionState.preActive)
    assert not icbm.fast_faulted, "a prompt-held dash faulted the stream"
    sends = self.run_frames(45, 30, n=100, icbm=icbm, source='cruise', v_ego_mph=30.)
    assert any(s in (SendButtonState.increase, SendButtonState.increaseHold) for s in sends), "restore never resumed after the prompt"

  def test_a_limiter_descends_through_a_prompt(self):
    """A prompt is the driver's decision about the limit, not about the road: route 269 t=185
    the old freeze parked a -1.2 m/s2 vision request 4 s before the apex and let the gap bleed
    off; route 26b t=480 it froze a whole approach."""
    icbm = self.make_icbm()
    self.run_frames(40, 40, n=60, icbm=icbm)
    sends = self.run_frames(28, 40, n=100, icbm=icbm, source='sccVision', v_ego_mph=38.,
                            a_target=-1.2, session_state=SessionState.preActive)
    assert icbm.overshoot_mph > 5., f"gap parked behind the prompt: {icbm.overshoot_mph}"
    down = (SendButtonState.decrease, SendButtonState.decreaseHold)
    assert any(s in down for s in sends), "curve decrease parked behind the prompt"

  def test_residual_gap_after_source_flip_starts_no_descent(self):
    """Layer 3: the lever outlives its limiter by design (slow release), but a residual
    must not START a fresh descent once the plan is back on cruise. Set directly: the
    gate is a single boolean and the state that reaches it is what matters."""
    icbm = self.make_icbm()
    self.run_frames(40, 40, n=60, icbm=icbm)
    icbm.overshoot_mph = 5.  # left over from a curve that just ended

    # off-limiter the residual drops at the build rate; check inside the bleed window
    sends = self.run_frames(40, 40, n=40, icbm=icbm, source='cruise', v_ego_mph=41.3)
    assert icbm.overshoot_mph > 0., "precondition: the residual is still bleeding off"
    assert icbm.state == State.holding, f"descended on a residual: {icbm.state}"
    assert all(s == SendButtonState.none for s in sends)
    sends = self.run_frames(40, 40, n=60, icbm=icbm, source='cruise', v_ego_mph=41.3)
    assert icbm.overshoot_mph == 0., "the residual must clear at the build rate once on cruise"
    assert all(s == SendButtonState.none for s in sends)

  def test_plain_setpoint_correction_still_unconditional(self):
    """Layer 3 must stay narrow: with no overshoot in play, a dash sitting above the
    driver's setpoint is a plain residual (a dropped press) and still self-heals."""
    icbm = self.make_icbm()
    self.run_frames(40, 40, n=60, icbm=icbm, source='cruise')

    sends = self.run_frames(40, 42, n=100, icbm=icbm, source='cruise')
    assert any(s == SendButtonState.decrease for s in sends), "dash residual stranded high"
