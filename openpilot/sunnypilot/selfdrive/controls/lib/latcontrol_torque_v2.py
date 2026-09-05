"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import math
import numpy as np
from collections import deque

from openpilot.cereal import log
from opendbc.car.lateral import get_friction
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.swaglog import cloudlog

from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import (
  LatControlTorque as LatControlTorqueV0,
  FRICTION_THRESHOLD,
  LP_FILTER_CUTOFF_HZ,
)

# v2 adds friction shaping, hand-back handling, low-speed damping, and a curvature request
# buffer to v0. See docs/zoompilot/lateral-tune.md for tuning data.

VERSION = 2

# Shape only the friction input with a clipped, filtered jerk signal and center deadzone.
MAX_FRICTION_JERK = 2.5  # m/s^3, clips the friction jerk input only (clip_curvature's 5 m/s^3 bounds the request)
CENTER_CHATTER_JERK_DEADZONE_SPEED_BP = [0.0, 5.0, 12.0, 25.0]  # m/s
CENTER_CHATTER_JERK_DEADZONE_SPEED_V = [0.08, 0.12, 0.18, 0.18]  # m/s^3
CENTER_CHATTER_JERK_DEADZONE_LAT_ACCEL_BP = [0.0, 0.18, 0.35]  # m/s^2
CENTER_CHATTER_JERK_DEADZONE_LAT_ACCEL_V = [1.0, 1.0, 0.0]  # full at lane center, zero in a real turn

# Decay the integrator and ramp error after driver hand-back while retaining feedforward.
STEER_RELEASE_I_DECAY = 0.8
RELEASE_ERROR_RAMP_T = 0.3  # s

# Add low-speed damping for torque retained by the EPS slew after a saturated step. Fade the
# derivative term as measurement noise and natural loop damping dominate.
KD_INTERP_SPEEDS = [7.5, 10.0, 12.0, 14.5]  # m/s
KD_INTERP = [1.65, 1.05, 0.85, 0.0]


def get_center_chatter_jerk_deadzone(v_ego, setpoint):
  """Small-signal jerk deadzone for the friction input: full at lane center, gone above 0.35 m/s^2."""
  center_weight = np.interp(abs(setpoint), CENTER_CHATTER_JERK_DEADZONE_LAT_ACCEL_BP, CENTER_CHATTER_JERK_DEADZONE_LAT_ACCEL_V)
  if center_weight == 0.0:
    return 0.0
  speed_deadzone = np.interp(max(v_ego, 0.0), CENTER_CHATTER_JERK_DEADZONE_SPEED_BP, CENTER_CHATTER_JERK_DEADZONE_SPEED_V)
  return float(speed_deadzone * center_weight)


class LatControlTorque(LatControlTorqueV0):
  # v0 already supplies negative measurement rate to the shared PID.
  KD_SCHEDULE = [KD_INTERP_SPEEDS, KD_INTERP]

  def __init__(self, CP, CP_SP, CI, dt):
    super().__init__(CP, CP_SP, CI, dt)
    # Buffer curvature and apply current v^2 on read to avoid speed-change artifacts in jerk.
    self.curvature_request_buffer = deque([0.] * self.lat_accel_request_buffer_len, maxlen=self.lat_accel_request_buffer_len)
    self.jerk_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt)
    self.prev_steering_pressed = False
    self._release_error_ramp = 1.0
    # Torque-space output overrides bypass v2 friction shaping and damping.
    cloudlog.info("LatControlTorque v2: extension output overrides (jerk-aware/NNLC) disabled")
    self.extension.disable_output_overrides()
    self.update_limits()  # restore lateral-acceleration PID limits

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, calibrated_pose, curvature_limited, lat_delay):
    # Apply live torque parameters before updating the controller limits.
    if self.extension.update_override_torque_params(self.torque_params):
      self.update_limits()

    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = VERSION

    measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
    measurement = measured_curvature * CS.vEgo ** 2
    future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2

    if not active:
      output_torque = 0.0
      pid_log.active = False
      # Prime request and rate state while inactive. Preserve the integrator across frequent
      # MADS cycles; hand-back decay handles the release transient.
      self.curvature_request_buffer.append(desired_curvature)
      self.previous_measurement = measurement
      self.measurement_rate_filter.x = 0.0
      self.jerk_filter.x = 0.0
      self._release_error_ramp = 1.0
    else:
      if self.prev_steering_pressed and not CS.steeringPressed:
        self.pid.i *= STEER_RELEASE_I_DECAY
        self._release_error_ramp = 0.0
      self._release_error_ramp = min(1.0, self._release_error_ramp + self.dt / RELEASE_ERROR_RAMP_T)

      roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
      curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      delay_frames = int(np.clip(lat_delay / self.dt, 1, self.lat_accel_request_buffer_len))
      expected_lateral_accel = self.curvature_request_buffer[-delay_frames] * CS.vEgo ** 2
      self.curvature_request_buffer.append(desired_curvature)
      gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
      desired_lateral_jerk = (future_desired_lateral_accel - expected_lateral_accel) / max(lat_delay, self.dt)

      measurement_rate = self.measurement_rate_filter.update((measurement - self.previous_measurement) / self.dt)
      self.previous_measurement = measurement

      # Reconstruct v0's live setpoint from the delayed request and differencer jerk.
      setpoint = lat_delay * desired_lateral_jerk + expected_lateral_accel
      error = (setpoint - measurement) * self._release_error_ramp

      # Correct error in lateral-acceleration space before applying the nonlinear torque map.
      pid_log.error = float(error)
      ff = gravity_adjusted_future_lateral_accel
      # latAccelOffset corrects device-to-vehicle roll bias.
      ff -= self.torque_params.latAccelOffset
      # Feed shaped jerk to friction without changing PID error.
      shaped_jerk = self.jerk_filter.update(min(max(desired_lateral_jerk, -MAX_FRICTION_JERK), MAX_FRICTION_JERK))
      friction_jerk = math.copysign(max(abs(shaped_jerk) - get_center_chatter_jerk_deadzone(CS.vEgo, setpoint), 0.0), shaped_jerk)
      friction_error = expected_lateral_accel + friction_jerk * lat_delay - measurement
      ff += get_friction(friction_error, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)

      # The shared limit classifier excludes integrator decay and the EPS authority rail.
      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      # Suppress derivative response to driver steering.
      error_rate = 0.0 if CS.steeringPressed else -measurement_rate
      if self.extension.overrides_output:
        # Prevent a future output override from stepping the shared PID twice.
        output_torque = 0.0
      else:
        output_lataccel = self.pid.update(pid_log.error,
                                          error_rate,
                                          feedforward=ff,
                                          speed=CS.vEgo,
                                          freeze_integrator=freeze_integrator)
        output_torque = self.torque_from_lateral_accel(output_lataccel, self.torque_params)

      # The extension may replace pid_log.error and output_torque. Bind by keyword because the
      # signature is shared across controller versions.
      pid_log, output_torque = self.extension.update(CS, VM, self.pid, params, ff, pid_log,
                                                     setpoint=setpoint,
                                                     measurement=measurement,
                                                     calibrated_pose=calibrated_pose,
                                                     roll_compensation=roll_compensation,
                                                     desired_lateral_accel=future_desired_lateral_accel,
                                                     actual_lateral_accel=measurement,
                                                     lateral_accel_deadzone=lateral_accel_deadzone,
                                                     gravity_adjusted_lateral_accel=gravity_adjusted_future_lateral_accel,
                                                     desired_curvature=desired_curvature,
                                                     actual_curvature=measured_curvature,
                                                     steer_limited_by_safety=steer_limited_by_safety,
                                                     output_torque=output_torque)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque)
      pid_log.actualLateralAccel = float(measurement)
      pid_log.desiredLateralAccel = float(setpoint)
      pid_log.desiredLateralJerk = float(desired_lateral_jerk)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    self.prev_steering_pressed = CS.steeringPressed

    # Positive curvature is leftward in this convention.
    return -output_torque, 0.0, pid_log
