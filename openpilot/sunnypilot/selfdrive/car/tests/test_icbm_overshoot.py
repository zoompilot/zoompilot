"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

ICBM deceleration overshoot: the down-only lever that commands the dash below vEgo so
the stock ACC delivers the requested deceleration, and the gates that keep it a lever
rather than a destination.
"""
from openpilot.cereal import custom
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.car.tests.icbm_servo_harness import make_icbm, run_frames

State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState
SessionState = custom.LongitudinalPlanSP.SpeedLimit.AssistState


class TestDecelOvershoot:
  """Down-only overshoot: command the dash below the planner target so the stock ACC
  delivers the requested deceleration (its decel scales with dash-vs-vEgo gap)."""

  def make_icbm(self, brand="mazda"):
    return make_icbm(brand)

  def run_frames(self, icbm, target_mph, v_ego_mph, a_target, n=1, source='sccVision', enabled=True):
    Params().put_bool("SmartCruiseDecelOvershoot", enabled)
    icbm.decel_overshoot_enabled = enabled
    for _ in range(n):
      CS = car.CarState(vEgo=v_ego_mph * CV.MPH_TO_MS,
                        cruiseState={"speedCluster": target_mph * CV.MPH_TO_MS})
      CC = car.CarControl(enabled=True)
      LP_SP = custom.LongitudinalPlanSP(vTarget=target_mph * CV.MPH_TO_MS, aTarget=a_target)
      LP_SP.longitudinalPlanSource = source
      icbm.run(CS, CC, LP_SP, is_metric=False)

  def test_commands_below_target_when_decelerating(self):
    icbm = self.make_icbm()
    # planner wants -0.45 m/s^2 at 45 mph toward a 40 mph target: gap_v asks ~8.5 mph below
    # vEgo, leading the steady-state inverse to pay back the dash walk
    self.run_frames(icbm, target_mph=40, v_ego_mph=45, a_target=-0.45, n=100)
    assert icbm.v_target <= 37, icbm.v_target
    assert icbm.v_target >= 35, icbm.v_target

  def test_deep_dip_is_a_no_op(self):
    """When the target is already far below vEgo the plant is saturated; never go deeper."""
    icbm = self.make_icbm()
    self.run_frames(icbm, target_mph=20, v_ego_mph=45, a_target=-1.0, n=100)
    assert icbm.v_target == 20, icbm.v_target

  def test_releases_back_to_target(self):
    icbm = self.make_icbm()
    self.run_frames(icbm, target_mph=40, v_ego_mph=45, a_target=-0.45, n=100)
    assert icbm.v_target < 40
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

  def test_toggle_off_disables_overshoot(self):
    icbm = self.make_icbm()
    self.run_frames(icbm, target_mph=40, v_ego_mph=45, a_target=-0.45, n=100, enabled=False)
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

  def test_no_wind_up_behind_a_confirm_prompt(self):
    """Layer 2: a limiter asking for decel while a prompt is open must not accumulate a
    gap the servo is forbidden to emit."""
    icbm = self.make_icbm()
    self.run_frames(40, 40, n=60, icbm=icbm, overshoot=True)

    sends = self.run_frames(40, 40, n=500, icbm=icbm, source='speedLimitAssist', v_ego_mph=41.3,
                            a_target=-0.5, overshoot=True, session_state=SessionState.preActive)
    assert icbm.overshoot_mph == 0., f"banked behind the freeze: {icbm.overshoot_mph}"
    assert all(s == SendButtonState.none for s in sends)

    # prompt times out with the limiter gone: nothing is owed, so nothing moves
    sends = self.run_frames(40, 40, n=200, icbm=icbm, source='cruise', v_ego_mph=41.3, overshoot=True)
    assert all(s == SendButtonState.none for s in sends), "stale gap dumped at the timeout"
    assert icbm.state == State.holding

  def test_freeze_does_not_blunt_a_real_limiter(self):
    """Layer 2 must cost nothing: if the limiter is still asking for decel when the prompt
    clears, the gap rebuilds at DECEL_OVERSHOOT_RISE and the descent still happens."""
    icbm = self.make_icbm()
    self.run_frames(40, 40, n=60, icbm=icbm, overshoot=True)
    self.run_frames(40, 40, n=500, icbm=icbm, source='speedLimitAssist', v_ego_mph=41.3,
                    a_target=-0.5, overshoot=True, session_state=SessionState.preActive)
    assert icbm.overshoot_mph == 0.

    sends = self.run_frames(40, 40, n=100, icbm=icbm, source='speedLimitAssist', v_ego_mph=41.3,
                            a_target=-0.5, overshoot=True)
    assert icbm.overshoot_mph > 2., f"gap did not rebuild: {icbm.overshoot_mph}"
    # tap or hold is the profile's call from the remaining distance; either is a descent
    down = (SendButtonState.decrease, SendButtonState.decreaseHold)
    assert any(s in down for s in sends), "real limiter decel was blunted"

  def test_residual_gap_after_source_flip_starts_no_descent(self):
    """Layer 3: the lever outlives its limiter by design (slow release), but a residual
    must not START a fresh descent once the plan is back on cruise. Set directly: the
    gate is a single boolean and the state that reaches it is what matters."""
    icbm = self.make_icbm()
    self.run_frames(40, 40, n=60, icbm=icbm)
    icbm.overshoot_mph = 5.  # left over from a curve that just ended

    # off-limiter the residual drops at the build rate; check inside the bleed window
    sends = self.run_frames(40, 40, n=40, icbm=icbm, source='cruise', v_ego_mph=41.3, overshoot=True)
    assert icbm.overshoot_mph > 0., "precondition: the residual is still bleeding off"
    assert icbm.state == State.holding, f"descended on a residual: {icbm.state}"
    assert all(s == SendButtonState.none for s in sends)
    sends = self.run_frames(40, 40, n=60, icbm=icbm, source='cruise', v_ego_mph=41.3, overshoot=True)
    assert icbm.overshoot_mph == 0., "the residual must clear at the build rate once on cruise"
    assert all(s == SendButtonState.none for s in sends)

  def test_plain_setpoint_correction_still_unconditional(self):
    """Layer 3 must stay narrow: with no overshoot in play, a dash sitting above the
    driver's setpoint is a plain residual (a dropped press) and still self-heals."""
    icbm = self.make_icbm()
    self.run_frames(40, 40, n=60, icbm=icbm, source='cruise')

    sends = self.run_frames(40, 42, n=100, icbm=icbm, source='cruise')
    assert any(s == SendButtonState.decrease for s in sends), "dash residual stranded high"
