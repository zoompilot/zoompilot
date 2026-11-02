"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from opendbc.car import DT_CTRL, rate_limit
from opendbc.car.mazda.values import CarControllerParams

# Stock body-latched releases use a RESUME_UNLATCHING pulse of 9 wire frames, the latched-family mode.
RESUME_UNLATCH_LATCHED_FRAMES = int(0.18 / DT_CTRL)
# Retry one unanswered body-latched release after this, the body still holding, then return control to the plan.
RESUME_REPULSE_FRAMES = int(1.0 / DT_CTRL)
# Debounce movement requests before releasing a standstill hold.
RELEASE_DEBOUNCE_FRAMES = int(0.2 / DT_CTRL)
# Arriving at a stop, the e2e plan drifts up to +0.11 for up to 0.4 s before its shouldStop lands,
# always starting within 0.2 s of standstill. For RELEASE_SETTLE_FRAMES of standstill a hold opens
# only above RELEASE_ACCEL; after that any sustained positive plan is a request to move, since a
# gentle drive-off behind a creeping lead can peak at +0.11 (routes 4a3fe2b0e8aec968/0000006e,
# 00000100 t+613). Keying this off longControlState instead releases on three arrival blips.
RELEASE_SETTLE_FRAMES = int(1.0 / DT_CTRL)
RELEASE_ACCEL = 0.25  # m/s^2
# The still-stopped breakaway ramp gives up after this.
BREAKAWAY_FRAMES = int(3.0 / DT_CTRL)
# Stock's release ramp holds for this long; every stock release pulse falls inside it.
RELEASE_SHAPE_FRAMES = int(0.2 / DT_CTRL)
# leadVisible toggles 2.3 times a minute while moving, half its runs under 0.6 s, mostly on a
# marginal lead past 100 m. Showing it after 0.5 s and dropping it after 1.0 s leaves under one
# change a minute (routes 25e-2c1, 4a3fe2b0e8aec968/0000006e).
LEAD_SHOW_FRAMES = int(0.5 / DT_CTRL)
LEAD_HIDE_FRAMES = int(1.0 / DT_CTRL)


class LeadIcon:
  """The dash lead icon: leadVisible once it has held long enough to stop it flickering."""

  def __init__(self):
    self.shown = False
    self.frames = 0

  def update(self, lead_visible: bool) -> bool:
    self.frames = self.frames + 1 if lead_visible != self.shown else 0
    if self.frames >= (LEAD_SHOW_FRAMES if lead_visible else LEAD_HIDE_FRAMES):
      self.shown = lead_visible
      self.frames = 0
    return self.shown


class StandstillHold:
  """Hold the car until the plan or driver requests movement.

  The plan supplies the hold command. It relaxes when the body ECU takes brake ownership,
  matching the stock stop-and-go sequence.
  """

  def __init__(self):
    self._reset()

  def _reset(self):
    self.holding = False
    self.car_has_hold = False
    self.unlatch_frames = 0
    self.release_frames = 0
    self.latched_release = False
    self.just_released = False
    self.latched_frames = 0  # frames waiting for the body ECU to release its hold
    self.repulsed = False
    self.standstill_frames = 0

  def update(self, long_engaged: bool, stopping: bool, standstill: bool,
             plan_accel: float, body_hold: bool, gas_pressed: bool) -> None:
    self.just_released = False
    if not long_engaged:
      self._reset()
      return

    was_holding = self.holding
    self.standstill_frames = self.standstill_frames + 1 if standstill else 0
    # Debounce plan movement requests: opening a hold just after arriving takes RELEASE_ACCEL,
    # otherwise any positive plan. Driver throttle releases the hold immediately.
    settling = self.standstill_frames <= RELEASE_SETTLE_FRAMES
    release_accel = RELEASE_ACCEL if was_holding and settling else 0.
    self.release_frames = self.release_frames + 1 if plan_accel > release_accel else 0
    plan_wants_go = self.release_frames >= RELEASE_DEBOUNCE_FRAMES
    # Keep STOPPING off once the plan or driver requests acceleration.
    release = gas_pressed or plan_wants_go
    self.holding = not release and (stopping or standstill)

    if self.unlatch_frames > 0:
      self.unlatch_frames -= 1
    # Send one unlatch pulse per plan-driven body-latched release. Throttle releases use none.
    if was_holding and not self.holding and standstill and not gas_pressed and self.unlatch_frames == 0:
      # The previous frame records whether the body owned a hold that must be unlatched.
      self.latched_release = self.car_has_hold
      if self.latched_release:
        self.unlatch_frames = RESUME_UNLATCH_LATCHED_FRAMES
      self.just_released = True
      self.latched_frames = 0
      self.repulsed = False

    # Retry one unanswered body-latched release so a positive plan cannot remain blocked.
    if self.latched_release and not self.holding and standstill and body_hold and not gas_pressed:
      self.latched_frames += 1
      if self.latched_frames >= RESUME_REPULSE_FRAMES and not self.repulsed and self.unlatch_frames == 0:
        self.unlatch_frames = RESUME_UNLATCH_LATCHED_FRAMES
        self.repulsed = True
    else:
      self.latched_frames = 0

    # Body ownership is valid only while the controller still requests a hold.
    self.car_has_hold = self.holding and standstill and body_hold

  @property
  def stop_bits(self) -> bool:
    # Keep STOPPING and RESUME_UNLATCHING mutually exclusive, including during a re-hold.
    return self.holding and not self.car_has_hold and self.unlatch_frames == 0

  @property
  def resume_unlatching(self) -> bool:
    return self.unlatch_frames > 0

  @property
  def acc_active_2(self) -> bool:
    # Stock clears ACC_ACTIVE_2 when the command relaxes.
    return not self.car_has_hold


class AccelShaper:
  """The CRZ_INFO command between the plan and the wire, shaped like stock MRCC's.

  A hold's release ramps up from the relaxed command, with a bounded breakaway while the car is
  still stopped. Otherwise the plan is slew-limited, and positive commands follow stock's
  ceiling and build rate at this speed and lift at stock's rate. The standstill hold's own
  commands come last. accel_last is the command sent the frame before.
  """

  def __init__(self):
    self.release_ramp: float | None = None
    self.breakaway_frames = 0

  def update(self, long_active: bool, plan_accel: float, accel_last: float, sm: StandstillHold,
             standstill: bool, body_hold: bool, v_ego: float) -> float:
    if sm.just_released:
      # Never-latched stops relax in one frame; latched holds ramp from the relaxed command.
      self.release_ramp = CarControllerParams.ACCEL_HOLD_LATCHED if sm.latched_release else \
                          CarControllerParams.ACCEL_RELEASE_BAND
    elif sm.holding or not long_active:
      # Re-holds and driver overrides terminate the release ramp.
      self.release_ramp = None

    accel = 0.
    if long_active:
      accel = float(np.clip(plan_accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      # Continue a bounded release ramp while stopped because the plan may not break static hold.
      if self.release_ramp is None or not standstill:
        self.breakaway_frames = 0
      else:
        self.breakaway_frames += 1
      breakaway = standstill and self.breakaway_frames <= BREAKAWAY_FRAMES
      # Bound breakaway by stock authority and by the plan-relative margin.
      ramp_ceiling = max(accel, min(CarControllerParams.ACCEL_BREAKAWAY_MAX,
                                    accel + CarControllerParams.ACCEL_BREAKAWAY_OVERSHOOT))
      if self.release_ramp is not None and (self.release_ramp < accel or breakaway):
        # The release ramp owns the command until it reaches the plan. Body-latched holds remain
        # at the relaxed command until the body lets go.
        accel = self.release_ramp
        if not (sm.latched_release and body_hold):
          quick = breakaway and self.breakaway_frames > RELEASE_SHAPE_FRAMES
          ramp = CarControllerParams.ACCEL_BREAKAWAY_RAMP if quick else CarControllerParams.ACCEL_RELEASE_RAMP
          if sm.resume_unlatching:
            # Stay under the pulse's clamp so the command cannot step up when it lifts.
            ramp_ceiling = min(ramp_ceiling, CarControllerParams.ACCEL_RESUME_PULSE_MAX)
          # Follow a falling plan ceiling at the winddown limit.
          self.release_ramp = rate_limit(ramp_ceiling, self.release_ramp, CarControllerParams.ACCEL_WINDDOWN_LIMIT, ramp * DT_CTRL)
      else:
        self.release_ramp = None
        # Track overrides in accel_last so control resumes through the slew limiter.
        accel = rate_limit(accel, accel_last, CarControllerParams.ACCEL_WINDDOWN_LIMIT,
                           CarControllerParams.ACCEL_WINDUP_LIMIT)
        if accel > 0.:
          # Shape positive commands to stock MRCC's ceiling and build rate at this speed.
          ceiling = float(np.interp(v_ego, CarControllerParams.ACCEL_CEILING_BP, CarControllerParams.ACCEL_CEILING_V))
          build = float(np.interp(v_ego, CarControllerParams.ACCEL_BUILD_BP, CarControllerParams.ACCEL_BUILD_V)) * DT_CTRL
          accel = min(accel, ceiling, max(accel_last, 0.) + build)
        if accel_last > 0. and plan_accel >= 0.:
          # Lift the throttle at stock's rate; a brake request bypasses this above.
          accel = max(accel, accel_last + CarControllerParams.ACCEL_LIFT_LIMIT * DT_CTRL)
      if sm.car_has_hold:
        # Stop requesting brake hold after the body ECU takes ownership.
        accel = CarControllerParams.ACCEL_HOLD_LATCHED
      elif sm.holding:
        # Freeze the braking command while STOPPING is asserted.
        accel = min(accel, 0.) if plan_accel <= 0. else min(accel_last, 0.)
      if sm.resume_unlatching:
        # Bound the latched release pulse to stock's command range.
        accel = min(max(accel, CarControllerParams.ACCEL_HOLD_LATCHED),
                    CarControllerParams.ACCEL_RESUME_PULSE_MAX)
    return accel
