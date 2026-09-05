"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np

from openpilot.common.params import Params


class LatControlTorqueExtOverride:
  def __init__(self, CP):
    self.CP = CP
    self.params = Params()
    self.enforce_torque_control_toggle = self.params.get_bool("EnforceTorqueControl")  # only during init
    self.torque_override_enabled = self.params.get_bool("TorqueParamsOverrideEnabled")
    self.frame = -1
    # cached at the 3 s poll below; preloaded so the values are valid from frame 0
    self._override_lat_accel_factor = float(self.params.get("TorqueParamsOverrideLatAccelFactor", return_default=True))
    self._override_friction = float(self.params.get("TorqueParamsOverrideFriction", return_default=True))

    # Speed-dep state (set by LatControlTorqueExt subclass)
    self._speed_dep_active = False
    self._speed_dep_speed_bp = []
    self._speed_dep_lat_accel_factor_bp = []
    self._speed_dep_friction_bp = []
    # Per-count tables for platforms with a speed-dependent STEER_MAX: the schedule is
    # (speed_bp, steer_max_v) from the car's speed-dep config, LAF divided and friction
    # multiplied by it at each bin center. None/empty on flat cars.
    self._speed_dep_steer_max_schedule = None
    self._speed_dep_laf_per_count_bp = []
    self._speed_dep_friction_per_count_bp = []
    self._speed_dep_car_cfg = None
    self._last_vego = 0.0

  def update_override_torque_params(self, torque_params) -> bool:
    changed = False

    # Manual override first: it must own the params on every frame, or the speed-dep interp
    # below out-writes it between the 3 s polls. The cached values apply each frame.
    if self.enforce_torque_control_toggle:
      self.frame += 1
      if self.frame % 300 == 0:
        self.torque_override_enabled = self.params.get_bool("TorqueParamsOverrideEnabled")
        if self.torque_override_enabled:
          self._override_lat_accel_factor = float(self.params.get("TorqueParamsOverrideLatAccelFactor", return_default=True))
          self._override_friction = float(self.params.get("TorqueParamsOverrideFriction", return_default=True))

      if self.torque_override_enabled:
        if torque_params.latAccelFactor != self._override_lat_accel_factor or torque_params.friction != self._override_friction:
          torque_params.latAccelFactor = self._override_lat_accel_factor
          torque_params.friction = self._override_friction
          changed = True
        return changed

    # Speed-dep latAccelFactor and friction, interpolated by speed each frame. On a platform
    # with a speed-dependent STEER_MAX the bins are normalized units learned under one scale
    # each, so both interp in CAN-count space and rescale at the current speed: the scale's
    # step lands where the carcontroller applies it instead of being smeared across the bin
    # span. Friction is inverted (counts = friction * STEER_MAX). See docs/zoompilot/lateral-tune.md.
    if self._speed_dep_active and self._speed_dep_speed_bp:
      if self._speed_dep_steer_max_schedule and self._speed_dep_laf_per_count_bp:
        sm_bp, sm_v = self._speed_dep_steer_max_schedule
        steer_max = float(np.interp(self._last_vego, sm_bp, sm_v))
        new_lat_accel_factor = float(np.interp(self._last_vego, self._speed_dep_speed_bp, self._speed_dep_laf_per_count_bp)) * steer_max
        new_fric = float(np.interp(self._last_vego, self._speed_dep_speed_bp, self._speed_dep_friction_per_count_bp)) / steer_max
      else:
        new_lat_accel_factor = float(np.interp(self._last_vego, self._speed_dep_speed_bp, self._speed_dep_lat_accel_factor_bp))
        new_fric = float(np.interp(self._last_vego, self._speed_dep_speed_bp, self._speed_dep_friction_bp))
      # torque_params is a capnp Float32 builder: compare in float32 or update_limits runs every frame
      new_lat_accel_factor = float(np.float32(new_lat_accel_factor))
      new_fric = float(np.float32(new_fric))
      if new_lat_accel_factor != torque_params.latAccelFactor or new_fric != torque_params.friction:
        torque_params.latAccelFactor = new_lat_accel_factor
        torque_params.friction = new_fric
        changed = True

    return changed
