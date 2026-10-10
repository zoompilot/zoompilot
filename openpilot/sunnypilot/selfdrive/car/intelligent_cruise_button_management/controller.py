"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Intelligent Cruise Button Management: a servo that walks a stock ACC's dash set speed
onto the plan target with synthesized cruise button presses (non-pcmCruiseSpeed cars).
Measurements behind the constants and the rejected alternatives: docs/zoompilot/icbm.md.
"""
import numpy as np

from openpilot.cereal import custom
from opendbc.car.structs import car
from opendbc.car import structs
from opendbc.sunnypilot.car.icbm_actuation_profile import get_actuation_profile
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import minimum_set_speed_ms
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.icbm_latch import icbm_active
from openpilot.sunnypilot.selfdrive.car.cruise_ext import CRUISE_BUTTON_TIMER, update_manual_button_timers
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import V_CRUISE_UNSET

ButtonType = car.CarState.ButtonEvent.Type
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState
SessionState = custom.LongitudinalPlanSP.SpeedLimit.AssistState

INACTIVE_TIMER = 0.4
# Match selfdrived's 0.1 s parameter refresh cadence.
# After a driver press, defer synthesized movement in the opposite direction.
DRIVER_PRESS_GRACE_T = 3.0
DRIVER_PRESS_GRACE_FRAMES = int(DRIVER_PRESS_GRACE_T / DT_CTRL)
# Limiter targets use a display-unit deadband; driver setpoints are tracked exactly.
REACT_DEADBAND = 2
# Require persistent error before sending a button burst.
REACT_TIMER = 0.3
# Some ECUs require a stable set speed before they begin decelerating.
RESTORE_QUIET_TIME = 1.0
RESTORE_QUIET_FRAMES = int(RESTORE_QUIET_TIME / DT_CTRL)

# Stock ACC deceleration follows the gap between dash set speed and actual speed, in stages,
# and keeps growing past 10 mph (harder still at highway speed). Hold the dash the gap that
# yields the limiter's own requested decel below vEgo instead of walking it to the whole dip.
DECEL_OVERSHOOT_PARAMS = {
  'mazda': {
    'speed_bp': [20.1, 29.1],  # m/s (45, 65 mph)
    'decel_bp': [0.15, 0.30, 0.50, 0.60, 0.70, 0.75, 0.90, 1.05],  # requested decel magnitude, m/s^2
    # gap below vEgo, mph, per speed_bp row. Planners budget 0.75 (limits._STOCK_A_BUDGET); a
    # request past it means the car is late, and the columns beyond are the ECU's remaining range
    'gap_v': [[2.5, 3.0, 3.75, 6.5, 8.5, 10.0, 16.25, 20.25],
              [2.5, 3.0, 3.5, 6.5, 7.25, 7.75, 12.5, 17.25]],
    'min_decel': 0.15,  # m/s^2; leave gentle coast-downs to the stock behavior
  },
}
# Apply quickly and release slowly across the ECU's discrete deceleration stages.
DECEL_OVERSHOOT_RISE = 10.  # mph/s
DECEL_OVERSHOOT_RELEASE = 3.  # mph/s
# Under openpilot longitudinal the planner executes these targets directly; the servo only
# keeps the dash on the speed limit session (Mazda alpha long).
OP_LONG_PLANNER_SOURCES = (LongitudinalPlanSource.sccVision, LongitudinalPlanSource.sccMap)
DECEL_OVERSHOOT_SOURCES = (*OP_LONG_PLANNER_SOURCES, LongitudinalPlanSource.speedLimitAssist)

# A 10 Hz hold stream registers as paced one-unit presses. Use taps for the final steps to
# avoid overshoot from in-flight stream frames.
FAST_MODE_MIN = 3  # display units of remaining error to run the stream
FAST_STALL_T = 1.5  # s; a dash that never moves under the stream means taps for the rest of the drive

TAP_BUTTONS = {
  State.increasing: SendButtonState.increase,
  State.decreasing: SendButtonState.decrease,
}
HOLD_BUTTONS = {
  State.increasing: SendButtonState.increaseHold,
  State.decreasing: SendButtonState.decreaseHold,
}


class IntelligentCruiseButtonManagement:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.profile = get_actuation_profile(CP.brand)
    self.active = not CP_SP.pcmCruiseSpeed  # card's decision, kept current by update_activation
    self.overshoot_params = DECEL_OVERSHOOT_PARAMS.get(CP.brand)  # brands with a measured plant
    self.fast_faulted = False  # disable the stream for this drive if the dash never moves
    self.reset()

  def reset(self) -> None:
    """Servo state for one engagement's worth of moves; what the drive learned about the dash
    (fast_faulted) is kept."""
    self.v_target = 0
    self.v_cruise_cluster = 0
    self.v_cruise_min = 0
    self.cruise_button = SendButtonState.none
    self.state = State.inactive
    self.pre_active_timer = 0
    self.restore_quiet_timer = 0
    self.v_target_raw = 0
    self.v_target_raw_prev = 0
    self.react_deadband = REACT_DEADBAND
    self.lookahead_valid = False
    self.dip_ahead = False
    self.down_grace_timer = 0
    self.up_grace_timer = 0
    self.press_target = 0  # plan target at the driver's SET+: the grace parks down moves to it, not to a lower one

    self.is_ready = False
    self.is_ready_prev = False
    self.is_metric = False
    # A pending SLA confirmation caps the dash where it was when the prompt opened, so a
    # restore cannot raise it past the limit and be adopted as a confirm; down moves (a curve)
    # go through. card also vetoes up moves from its same-frame session state, since this view
    # is a message hop old.
    self.prompting = False
    self.prompt_ceiling: int | None = None
    self.overshoot_mph = 0.0
    self.limiter_active = False

    self.fast_active = False
    self.fast_stall_frames = 0
    self.fast_last_cluster = 0

    self.cruise_button_timers = dict(CRUISE_BUTTON_TIMER)

  @staticmethod
  def source_a_target(LP_SP: custom.LongitudinalPlanSP) -> float:
    """The active limiter's own decel request. LP_SP.aTarget is the MPC output, which rails at
    A_CRUISE_MIN whenever the target sits a few mph below vEgo, whatever the limiter asked for."""
    source = LP_SP.longitudinalPlanSource
    if source == LongitudinalPlanSource.sccVision:
      return LP_SP.smartCruiseControl.vision.aTarget
    if source == LongitudinalPlanSource.sccMap:
      return LP_SP.smartCruiseControl.map.aTarget
    if source == LongitudinalPlanSource.speedLimitAssist:
      return LP_SP.speedLimit.assist.aTarget
    return LP_SP.aTarget

  def update_decel_overshoot(self, CS: car.CarState, LP_SP: custom.LongitudinalPlanSP) -> float:
    if self.overshoot_params is None:
      return 0.0

    p = self.overshoot_params
    want = 0.0
    a_request = -self.source_a_target(LP_SP)
    # Do not accumulate a gap while button emission is blocked.
    # the gap trick drives a stock ACC's own deceleration; openpilot longitudinal brakes itself
    if (not self.CP.openpilotLongitudinalControl
        and self.is_ready and not self.down_parked
        and LP_SP.longitudinalPlanSource in DECEL_OVERSHOOT_SOURCES
        and a_request > p['min_decel'] and CS.vEgo > LP_SP.vTarget):
      gaps = [np.interp(a_request, p['decel_bp'], row) for row in p['gap_v']]
      want = float(np.interp(CS.vEgo, p['speed_bp'], gaps))

    if want > self.overshoot_mph:
      self.overshoot_mph = min(want, self.overshoot_mph + DECEL_OVERSHOOT_RISE * DT_CTRL)
    else:
      # Release gently under a limiter, but clear residual gap quickly after returning to cruise.
      release = DECEL_OVERSHOOT_RELEASE if self.limiter_active else DECEL_OVERSHOOT_RISE
      self.overshoot_mph = max(want, self.overshoot_mph - release * DT_CTRL)

    return self.overshoot_mph

  def update_calculations(self, CS: car.CarState, LP_SP: custom.LongitudinalPlanSP) -> None:
    speed_conv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH

    self.limiter_active = LP_SP.longitudinalPlanSource != LongitudinalPlanSource.cruise

    v_target_ms = LP_SP.vTarget
    if self.CP.openpilotLongitudinalControl and LP_SP.longitudinalPlanSource in OP_LONG_PLANNER_SOURCES:
      # openpilot brakes for curves itself: the dash keeps the driver's setpoint, or the
      # session's cap inside a zone (the mirror publishes it, V_CRUISE_UNSET when idle), and
      # never walks down to a curve target nor back up over the cap during one
      session_cap = LP_SP.speedLimit.assist.vTarget
      v_target_ms = min(CS.vCruise * CV.KPH_TO_MS, session_cap if session_cap > 0. else V_CRUISE_UNSET)
    overshoot_ms = self.update_decel_overshoot(CS, LP_SP) * CV.MPH_TO_MS
    if overshoot_ms > 0:
      # Track actual speed at the requested gap, even above a deeper plan target: walking the dash
      # to the whole dip brakes as hard as the dip is deep. Near the target, sit no further below
      # it than the car is above it, so the ECU's lag lands the car on it instead of under it.
      # Down-only: the dash rises during a descent only to land on the target, never above it.
      v_plan = LP_SP.vTarget
      v_command = max(CS.vEgo - overshoot_ms, v_plan - max(CS.vEgo - v_plan, 0.))
      v_target_ms = min(v_command, max(CS.cruiseState.speedCluster, v_plan))

    self.v_target = round(v_target_ms * speed_conv)
    # Judge restore intent against the unmodified plan target.
    self.v_target_raw_prev = self.v_target_raw
    self.v_target_raw = round(LP_SP.vTarget * speed_conv)
    self.v_cruise_min = round(minimum_set_speed_ms(self.CP_SP, self.is_metric) * speed_conv)
    self.v_cruise_cluster = round(CS.cruiseState.speedCluster * speed_conv)
    if not self.prompting:
      self.prompt_ceiling = None
    elif self.prompt_ceiling is None:
      self.prompt_ceiling = self.v_cruise_cluster

    # Track driver setpoints exactly and apply a jitter band to generated targets.
    self.react_deadband = REACT_DEADBAND if self.limiter_active or self.overshoot_mph > 0 else 1

    # A zero vision lookahead falls back to the stable-target restore gate.
    v_ahead_min = LP_SP.smartCruiseControl.vision.vAheadMin
    self.lookahead_valid = v_ahead_min > 0.
    self.dip_ahead = self.lookahead_valid and v_ahead_min * speed_conv < self.v_target_raw - self.react_deadband
    if self.dip_ahead and self.v_target > self.v_cruise_cluster:
      # A dip ahead caps a restore at its own level rather than freezing a dash that sits below it.
      v_ahead = int(v_ahead_min * speed_conv)
      if v_ahead - self.v_cruise_cluster >= self.react_deadband:
        self.v_target = min(self.v_target, v_ahead)
        self.dip_ahead = False
    if self.prompt_ceiling is not None:
      self.v_target = min(self.v_target, self.prompt_ceiling)

  def update_restore_quiet_timer(self) -> None:
    # Measure stable restore demand against the unmodified target. Confirmation prompts
    # reset the window so restoration still requires a full quiet period afterward.
    up_error = self.v_target_raw - self.v_cruise_cluster
    if self.prompting:
      self.restore_quiet_timer = 0
    elif up_error >= self.react_deadband and self.v_target_raw == self.v_target_raw_prev:
      self.restore_quiet_timer += 1
    else:
      self.restore_quiet_timer = 0

  def plan_fast_mode(self) -> None:
    # Use the stream for large errors and taps for the remainder.
    remaining = abs(self.v_target - self.v_cruise_cluster)
    use_fast = not self.fast_faulted and remaining >= FAST_MODE_MIN

    if use_fast and not self.fast_active:
      self.fast_active = True
      self.fast_stall_frames = 0
      self.fast_last_cluster = self.v_cruise_cluster
    elif self.fast_active:
      if remaining < FAST_MODE_MIN:
        self.fast_active = False
      elif self.v_cruise_cluster != self.fast_last_cluster:
        self.fast_last_cluster = self.v_cruise_cluster
        self.fast_stall_frames = 0
      else:
        self.fast_stall_frames += 1
        if self.fast_stall_frames * DT_CTRL > FAST_STALL_T:
          self.fast_faulted = True
          self.fast_active = False
          cloudlog.event("icbm_fast_mode_fallback", brand=self.CP.brand)

  def update_state_machine(self) -> custom.IntelligentCruiseButtonManagement.SendButtonState:
    self.pre_active_timer = max(0, self.pre_active_timer - 1)
    self.update_restore_quiet_timer()

    if self.state != State.inactive:
      if not self.is_ready:
        self.state = State.inactive

      else:
        # Vision lookahead permits restore only when no lower target is approaching. Without
        # lookahead, platforms that require a stable setpoint use the quiet-time gate.
        if self.lookahead_valid:
          up_allowed = not self.dip_ahead
        else:
          up_allowed = ((self.overshoot_mph > 0 and self.limiter_active)
                        or not self.profile.decel_needs_stable_setpoint
                        or self.restore_quiet_timer >= RESTORE_QUIET_FRAMES)
        up_allowed = up_allowed and self.up_grace_timer <= 0

        # Live limiters may decrease immediately. Residual overshoot after returning to cruise
        # may only release, while ordinary setpoint corrections remain unconditional.
        down_allowed = (self.limiter_active or self.overshoot_mph <= 0) and not self.down_parked

        if self.state == State.preActive:
          if self.pre_active_timer <= 0:
            if self.v_target - self.v_cruise_cluster >= self.react_deadband and up_allowed:
              self.state = State.increasing

            elif self.v_cruise_cluster - self.v_target >= self.react_deadband \
                 and self.v_cruise_cluster > self.v_cruise_min and down_allowed:
              self.state = State.decreasing

            else:
              self.state = State.holding

        elif self.state == State.holding:
          down_pending = self.v_cruise_cluster - self.v_target >= self.react_deadband and down_allowed
          up_pending = self.v_target - self.v_cruise_cluster >= self.react_deadband
          if down_pending or (up_pending and up_allowed):
            self.pre_active_timer = int(REACT_TIMER / DT_CTRL)
            self.state = State.preActive

        elif self.state == State.increasing:
          # Abort restoration when a lower target enters the lookahead.
          if self.v_target <= self.v_cruise_cluster or self.dip_ahead:
            self.state = State.holding

        elif self.state == State.decreasing:
          if self.v_target >= self.v_cruise_cluster or self.v_cruise_cluster <= self.v_cruise_min:
            self.state = State.holding

    elif self.state == State.inactive:
      if self.is_ready and not self.is_ready_prev:
        self.pre_active_timer = int(INACTIVE_TIMER / DT_CTRL)
        self.state = State.preActive

    if self.state in TAP_BUTTONS:
      self.plan_fast_mode()
      send_button = HOLD_BUTTONS[self.state] if self.fast_active else TAP_BUTTONS[self.state]
    else:
      self.fast_active = False
      send_button = SendButtonState.none

    return send_button

  @property
  def down_parked(self) -> bool:
    """A driver SET+ parks down moves to the target it overrode for the grace window; a lower
    target that appears afterwards (a curve) is new information and goes through."""
    return self.down_grace_timer > 0 and self.v_target_raw >= self.press_target - self.react_deadband

  def update_readiness(self, CS: car.CarState, CC: car.CarControl) -> None:
    update_manual_button_timers(CS, self.cruise_button_timers)

    ready = CC.enabled and not CC.cruiseControl.override and not CC.cruiseControl.cancel and not CC.cruiseControl.resume
    button_pressed = any(self.cruise_button_timers[k] > 0 for k in self.cruise_button_timers)

    # buttonEvents contains only physical wheel presses, not synthesized frames.
    if self.cruise_button_timers[ButtonType.accelCruise] > 0:
      self.down_grace_timer = DRIVER_PRESS_GRACE_FRAMES
      self.press_target = self.v_target_raw
      self.up_grace_timer = 0
    elif self.cruise_button_timers[ButtonType.decelCruise] > 0:
      self.up_grace_timer = DRIVER_PRESS_GRACE_FRAMES
      self.down_grace_timer = 0
    else:
      self.down_grace_timer = max(0, self.down_grace_timer - 1)
      self.up_grace_timer = max(0, self.up_grace_timer - 1)

    self.is_ready = ready and not button_pressed

  def update_activation(self, CS_SP: custom.CarStateSP) -> None:
    active = icbm_active(CS_SP, self.CP_SP)
    if active == self.active:
      return
    # card moves this only while disengaged, where the servo idles: start the next one clean
    self.reset()
    self.active = active

  def run(self, CS: car.CarState, CC: car.CarControl, LP_SP: custom.LongitudinalPlanSP, is_metric: bool) -> None:
    if not self.active:
      return


    self.is_metric = is_metric
    self.prompting = LP_SP.speedLimit.assist.state == SessionState.preActive

    self.update_calculations(CS, LP_SP)
    self.update_readiness(CS, CC)

    self.cruise_button = self.update_state_machine()

    self.is_ready_prev = self.is_ready
