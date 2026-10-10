"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from opendbc.sunnypilot.car.interfaces import get_steer_rail_schedule, get_tune_scale


class LatControlTorqueExtZP:
  """zoompilot's layer on LatControlTorqueExt, first in its bases: the EPS rail as steer_max,
  speed-dependent torque, the manual override on the car's tune scale, the record the
  steer-limit classifier reads, and the switch a tune uses to turn off the output overrides
  (jerk-aware, NNLC). See docs/zoompilot/lateral-tune.md."""

  def __init__(self, CP):
    # cached at the override's 3 s poll; preloaded so the values are valid from frame 0
    self._override_lat_accel_factor = float(self.params.get("TorqueParamsOverrideLatAccelFactor", return_default=True))
    self._override_friction = float(self.params.get("TorqueParamsOverrideFriction", return_default=True))

    # speed-dep state, set by update_speed_dep_torque
    self._speed_dep_active = False
    self._speed_dep_speed_bp = []
    self._speed_dep_lat_accel_factor_bp = []
    self._speed_dep_friction_bp = []
    self._speed_dep_car_cfg = None
    self._last_vego = 0.0

    # The manual override is typed on the scale upstream's tunes use (TUNE_STEER_MAX), which is
    # 1.5x off the Mazda EPS envelope's STEER_MAX; 1.0 everywhere else.
    self._tune_scale = get_tune_scale(CP)

    self._output_overrides_disabled = False
    # EPS ceiling as a fraction of the carcontroller's scale, by speed (None: full scale
    # everywhere). Applied to the host as steer_max, so every tune's own update_limits() and
    # saturation test land on the rail with no tune changes. See docs/zoompilot/lateral-tune.md.
    self.steer_rail_schedule = get_steer_rail_schedule(CP)
    # this frame's command, for controls_lateral_zp: update() only runs on active frames, so the
    # per-frame update_override_torque_params call clears the mark and update() sets it
    self._commanded = False
    # what the carcontroller reported back, pushed by controls_lateral_zp after its classifier
    self._applied_torque = 0.0
    self._at_rail = False

  def rail_scale_at(self, v_ego: float) -> float:
    if self.steer_rail_schedule is None:
      return 1.0
    return float(np.interp(v_ego, self.steer_rail_schedule[0], self.steer_rail_schedule[1]))

  @property
  def commanded_torque(self) -> float:
    """This frame's CC.actuators.torque, in the actuator's sign convention: the tunes
    return -output_torque and controlsd publishes that; a frame the tune ran inactive
    commanded 0.0."""
    return -self._output_torque if self._commanded else 0.0

  @property
  def last_error(self) -> float:
    """pid_log.error of the frame just computed (the tune sets it before calling update())."""
    return float(self._pid_log.error) if self._pid_log is not None else 0.0

  @property
  def integrator(self) -> float:
    return float(self._pid.i)

  def set_actuator_state(self, applied_torque: float, at_rail: bool) -> None:
    self._applied_torque = applied_torque
    self._at_rail = at_rail

  @staticmethod
  def _write_torque_params(torque_params, prev, lat_accel_factor: float, friction: float) -> bool:
    # torque_params is a capnp Float32 builder: compare in float32 or update_limits runs every frame
    lat_accel_factor = float(np.float32(lat_accel_factor))
    friction = float(np.float32(friction))
    torque_params.latAccelFactor = lat_accel_factor
    torque_params.friction = friction
    return not (lat_accel_factor == prev[0] and friction == prev[1])

  def update_override_torque_params(self, torque_params) -> bool:
    self._commanded = False
    prev = (torque_params.latAccelFactor, torque_params.friction)
    # upstream's override: the frame count, the 3 s poll and its write on the poll frame,
    # which the scaled values below replace
    super().update_override_torque_params(torque_params)
    changed = False

    # Manual override first: it must own the params on every frame, or the speed-dep interp
    # below out-writes it between the 3 s polls. The cached values apply each frame.
    if self.enforce_torque_control_toggle and self.torque_override_enabled:
      if self.frame % 300 == 0:
        self._override_lat_accel_factor = float(self.params.get("TorqueParamsOverrideLatAccelFactor", return_default=True))
        self._override_friction = float(self.params.get("TorqueParamsOverrideFriction", return_default=True))
      changed = self._write_torque_params(torque_params, prev, self._override_lat_accel_factor * self._tune_scale,
                                          self._override_friction / self._tune_scale)

    # Speed-dep latAccelFactor and friction, interpolated by speed each frame.
    elif self._speed_dep_active and self._speed_dep_speed_bp:
      new_lat_accel_factor = float(np.interp(self._last_vego, self._speed_dep_speed_bp, self._speed_dep_lat_accel_factor_bp))
      new_fric = float(np.interp(self._last_vego, self._speed_dep_speed_bp, self._speed_dep_friction_bp))
      changed = self._write_torque_params(torque_params, prev, new_lat_accel_factor, new_fric)

    if self.steer_rail_schedule is not None:
      # _last_vego is the previous active frame's speed, like the speed-dep interp. The host's
      # limits scale linearly in steer_max only for a linear lateral_accel_from_torque; a
      # non-linear interface (NNLC-style torque models) would need its own rail handling.
      rail = self.rail_scale_at(self._last_vego)
      if rail != self.lac_torque.steer_max:
        self.lac_torque.steer_max = rail
        changed = True
    return changed

  def disable_output_overrides(self):
    """Permanently neutralize the override controllers (jerk-aware, NNLC) for a host that owns
    its own friction shaping and integrator policy. Speed-dependent torque is unaffected. The
    caller must re-run the host's update_limits(): an override controller may already have
    retuned the shared PID to torque-space limits at construction."""
    self._output_overrides_disabled = True

  @property
  def overrides_output(self) -> bool:
    return not self._output_overrides_disabled and super().overrides_output

  def update_limits(self):
    # the extension's only limit work is the override controllers' torque-space retune
    if self._output_overrides_disabled:
      return
    super().update_limits()

  def update_calculations(self, CS, VM, desired_lateral_accel):
    # the first of the extension's steps in update(), which runs on active frames only:
    # vEgo for the next frame's interp and rail, and the mark that this frame commanded
    self._last_vego = CS.vEgo
    self._commanded = True
    if self._output_overrides_disabled:
      return
    super().update_calculations(CS, VM, desired_lateral_accel)

  def update_jerk_aware_torque_control(self, CS, roll_compensation, gravity_adjusted_lateral_accel):
    if self._output_overrides_disabled:
      return
    super().update_jerk_aware_torque_control(CS, roll_compensation, gravity_adjusted_lateral_accel)

  def update_neural_network_feedforward(self, CS, params, calibrated_pose) -> None:
    if self._output_overrides_disabled:
      return
    super().update_neural_network_feedforward(CS, params, calibrated_pose)

  def disable_speed_dep_torque(self):
    """The single speed-dep deactivation path. Restores the CP tune so the controller does not
    keep running on the last interpolated values, as upstream does when useParams is false."""
    if not self._speed_dep_active:
      return
    self._speed_dep_active = False
    tune = self.CP.lateralTuning.torque
    self.lac_torque.torque_params.latAccelFactor = tune.latAccelFactor
    self.lac_torque.torque_params.latAccelOffset = tune.latAccelOffset
    self.lac_torque.torque_params.friction = tune.friction
    self.lac_torque.update_limits()

  def update_speed_dep_torque(self, tp, tp_sp):
    """Apply torqued's per-bin values: learned values for valid bins, the car's TOML seeds or
    the global filtered values for the rest. tp is upstream's lateralTorqueParameters (the
    globals and useParams), tp_sp the fork's liveTorqueParametersSP published beside it
    (the bins), or None when that service has not checked out. useParams off, no fork
    message or no bins all mean torqued no longer stands behind the values (the manual
    override flips useParams mid-drive), and each deactivates through
    disable_speed_dep_torque rather than leaving stale tables."""
    if not tp.useParams or tp_sp is None or not tp_sp.speedBinCenters:
      self.disable_speed_dep_torque()
      return
    speed_bp = list(tp_sp.speedBinCenters)

    factors = list(tp_sp.speedBinLatAccelFactors)
    frictions = list(tp_sp.speedBinFrictions)
    valid_bp = list(tp_sp.speedBinValid)

    if self._speed_dep_car_cfg is None:
      from opendbc.sunnypilot.car.interfaces import get_speed_dep_config_for_car
      self._speed_dep_car_cfg = get_speed_dep_config_for_car(self.CP)
    cfg = self._speed_dep_car_cfg
    seed_lafs = cfg.get('laf_bp')
    seed_frictions = cfg.get('friction_bp')
    if (seed_lafs and seed_frictions and
        len(seed_lafs) == len(speed_bp) and len(seed_frictions) == len(speed_bp)):
      fallback_factors = seed_lafs
      fallback_frictions = seed_frictions
    else:
      global_factor = tp.latAccelFactorFiltered
      global_fric = tp.frictionCoefficientFiltered
      fallback_factors = [global_factor] * len(speed_bp)
      fallback_frictions = [global_fric] * len(speed_bp)

    self._speed_dep_active = True
    self._speed_dep_speed_bp = speed_bp
    self._speed_dep_lat_accel_factor_bp = [factors[i] if valid_bp[i] else fallback_factors[i] for i in range(len(speed_bp))]
    self._speed_dep_friction_bp = [frictions[i] if valid_bp[i] else fallback_frictions[i] for i in range(len(speed_bp))]

    # global filtered values as the PID-limits baseline; the per-frame interp overwrites next frame
    self.lac_torque.torque_params.latAccelFactor = tp.latAccelFactorFiltered
    self.lac_torque.torque_params.latAccelOffset = tp.latAccelOffsetFiltered
    self.lac_torque.torque_params.friction = tp.frictionCoefficientFiltered
    self.lac_torque.update_limits()
