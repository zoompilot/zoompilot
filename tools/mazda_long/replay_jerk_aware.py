#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Replay logged drives through LatControlTorqueV0 with the Lateral Jerk Torque Controller and
speed-dependent torque both enabled, comparing the fixed extension against the as-shipped
(2026-08-09, #693) behavior.

The three defects being validated (see the fix commit):
  1. PID limits: the per-frame speed-dep override triggers update_limits(), resetting the
     torque-space PID's clamp to lat-accel-space bounds (~latAccelFactor times too wide).
  2. Double pid.update: the stock lat-accel-space update and the extension's torque-space
     update both hit the same integrator; effective ki becomes (1 + latAccelFactor(v)) times
     the tune, varying with speed through the speed-dep bins.
  3. Feedforward ignores latAccelOffset: torqued fits lat_accel = LAF*torque + offset, so the
     torque-space inversion must subtract the offset (stock does `ff -= latAccelOffset`).

Open-loop caveat: the car in the log was driven by the stock speed-dep controller, so errors
do not converge the way they would closed-loop. The comparison is still valid for windup
bounds, clamp behavior, and FF bias, because both variants see identical inputs.

Usage:
  python tools/mazda_long/replay_jerk_aware.py tools/mazda_long/device_data/000000ab--* [--plots]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from openpilot.common.prefix import OpenpilotPrefix


def load_frames(seg_paths, keep_model=True):
  """Extract per-controlsState-frame inputs from rlogs (100 Hz), forward-filling slower
  services. keep_model=False drops the modelV2 reference from each frame: a run with the
  extension override controllers off never reads it, and retaining the readers pins every
  segment's rlog buffer in memory for the whole run. keep_model='plan' copies just the
  fields the v2 setpoint jerk source reads into a small namespace per model message, so
  the trajectory replays without pinning the rlog buffers."""
  from openpilot.tools.lib.logreader import LogReader
  from speed_bin_log import SpeedBinTracker
  frames = []
  cs_last = None
  lp_last = None
  model_last = None
  ltp_last = None
  ltp_sp_last = None
  bins = SpeedBinTracker()
  cp_reader = None
  for seg in seg_paths:
    for m in LogReader(str(seg / "rlog.zst")):
      w = m.which()
      if bins.feed(w, m):
        continue
      if w == 'carParams' and cp_reader is None:
        cp_reader = m.carParams
      elif w == 'carState':
        cs_last = m.carState
      elif w in ('liveParameters', 'vehicleParameters'):  # renamed upstream 2026-08 (#38601)
        lp_last = getattr(m, w)
      elif w == 'modelV2':
        model_last = m.modelV2
        if keep_model == 'plan':
          model_last = SimpleNamespace(
            frameId=model_last.frameId,
            orientation=SimpleNamespace(x=[0.0] * 33),  # only length-checked (ext-base model_valid)
            orientationRate=SimpleNamespace(z=np.array(model_last.orientationRate.z)),
            velocity=SimpleNamespace(x=np.array(model_last.velocity.x)),
            acceleration=SimpleNamespace(y=np.array(model_last.acceleration.y)),  # v0's ext lookahead reads it
          )
      elif w == 'lateralTorqueParameters':
        ltp_last = m.lateralTorqueParameters
        ltp_sp_last = bins.bins_for(ltp_last)  # the fork message beside it, or the legacy fields
      elif w == 'controlsState':
        st = m.controlsState.lateralControlState
        if st.which() != 'torqueState' or cs_last is None or lp_last is None or model_last is None:
          continue
        ts = st.torqueState
        frames.append(SimpleNamespace(
          v_ego=cs_last.vEgo, a_ego=cs_last.aEgo,
          steering_angle=cs_last.steeringAngleDeg, steering_rate=cs_last.steeringRateDeg,
          steering_pressed=cs_last.steeringPressed,
          roll=lp_last.roll, angle_offset=lp_last.angleOffsetDeg,
          model=model_last if keep_model else None, ltp=ltp_last, ltp_sp=ltp_sp_last,
          active=ts.active, logged_output=ts.output, logged_i=ts.i,
          desired_curvature=m.controlsState.desiredCurvature,
        ))
  return frames, cp_reader


def make_controller(fingerprint, jerk_aware=True):
  from opendbc.car.car_helpers import interfaces
  from openpilot.common.params import Params
  from openpilot.common.realtime import DT_CTRL
  from openpilot.selfdrive.car.helpers import convert_to_capnp
  from openpilot.sunnypilot.selfdrive.car import interfaces as sunnypilot_interfaces
  from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0
  from opendbc.car.vehicle_model import VehicleModel

  params = Params()
  for k in ("EnforceTorqueControl", "LiveTorqueParamsToggle", "SpeedDependentTorqueToggle"):
    params.put_bool(k, True, block=True)
  params.put_bool("LateralJerkTorqueController", jerk_aware, block=True)
  params.put_bool("NeuralNetworkLateralControl", False, block=True)

  CarInterface = interfaces[fingerprint]
  CP = CarInterface.get_non_essential_params(fingerprint)
  CP_SP = CarInterface.get_non_essential_params_sp(CP, fingerprint)
  CI = CarInterface(CP, CP_SP)
  sunnypilot_interfaces.setup_interfaces(CI, params)
  CP_SP = convert_to_capnp(CP_SP)
  VM = VehicleModel(CP)
  controller = LatControlTorqueV0(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)
  return controller, VM, CP


def apply_shipped_behavior(controller):
  """Monkeypatch the fixed tree back to the as-shipped #693 behavior for this instance."""
  from opendbc.car.lateral import FRICTION_THRESHOLD
  from opendbc.sunnypilot.car.interfaces import LatControlInputs
  from opendbc.sunnypilot.car.lateral_ext import get_friction as get_friction_in_torque_space

  # 1. update_limits without the extension re-assert
  def stock_update_limits():
    controller.pid.set_limits(controller.lateral_accel_from_torque(controller.steer_max, controller.torque_params),
                              controller.lateral_accel_from_torque(-controller.steer_max, controller.torque_params))
  controller.update_limits = stock_update_limits

  # 2. overrides_output False -> the stock lat-accel pid.update runs too (double update)
  type(controller.extension).overrides_output = property(lambda self: False)

  # 3. original FF without the latAccelOffset subtraction
  ext = controller.extension
  def shipped_jerk_aware(CS, roll_compensation, gravity_adjusted_lateral_accel):
    if not ext._jerk_aware_enabled:
      return
    torque_from_setpoint = ext.torque_from_lateral_accel_in_torque_space(
      LatControlInputs(ext._setpoint, roll_compensation, CS.vEgo, CS.aEgo), ext.torque_params, gravity_adjusted=False)
    torque_from_measurement = ext.torque_from_lateral_accel_in_torque_space(
      LatControlInputs(ext._measurement, roll_compensation, CS.vEgo, CS.aEgo), ext.torque_params, gravity_adjusted=False)
    ext._pid_log.error = float(torque_from_setpoint - torque_from_measurement)
    ext._ff = ext.torque_from_lateral_accel_in_torque_space(
      LatControlInputs(gravity_adjusted_lateral_accel, roll_compensation, CS.vEgo, CS.aEgo), ext.torque_params, gravity_adjusted=True)
    friction_input = ext.update_friction_input(ext._desired_lateral_accel, ext._actual_lateral_accel)
    ext._ff += get_friction_in_torque_space(friction_input, ext._lateral_accel_deadzone, FRICTION_THRESHOLD, ext.torque_params)
    ext.update_output_torque(CS)
  ext.update_jerk_aware_torque_control = shipped_jerk_aware


def run_variant(frames, fingerprint, mode: str):
  controller, VM, CP = make_controller(fingerprint, jerk_aware=(mode != 'stock'))
  if mode == 'shipped':
    apply_shipped_behavior(controller)

  lat_delay = CP.steerActuatorDelay
  last_ltp = None
  out = {k: [] for k in ('output', 'i', 'ff', 'pos_limit', 'active', 'v_ego', 'logged_output', 'logged_i')}
  for f in frames:
    # controlsd: global filtered params + extension limits, every frame the service is alive
    if f.ltp is not None and f.ltp.useParams:
      controller.update_torque_parameters(f.ltp.latAccelFactorFiltered, f.ltp.latAccelOffsetFiltered, f.ltp.frictionCoefficientFiltered)
      controller.extension.update_limits()
      # controlsd_ext: per-bin values on each new lateralTorqueParameters message
      if f.ltp is not last_ltp:
        controller.extension.update_speed_dep_torque(f.ltp, f.ltp_sp)
        last_ltp = f.ltp
    controller.extension.update_model_v2(f.model)
    controller.extension.update_lateral_lag(lat_delay)

    CS = SimpleNamespace(vEgo=f.v_ego, aEgo=f.a_ego, steeringAngleDeg=f.steering_angle,
                         steeringRateDeg=f.steering_rate, steeringPressed=f.steering_pressed)
    lp = SimpleNamespace(roll=f.roll, angleOffsetDeg=f.angle_offset)
    _, _, pid_log = controller.update(f.active, CS, VM, lp, False, f.desired_curvature, None, False, lat_delay)

    out['output'].append(-pid_log.output if f.active else 0.0)
    out['i'].append(controller.pid.i)
    out['ff'].append(controller.pid.f)
    out['pos_limit'].append(controller.pid.pos_limit)
    out['active'].append(f.active)
    out['v_ego'].append(f.v_ego)
    out['logged_output'].append(f.logged_output)
    out['logged_i'].append(f.logged_i)
  return {k: np.array(v) for k, v in out.items()}


def report(name, r):
  act = r['active'].astype(bool)
  i_a, out_a = np.abs(r['i'][act]), np.abs(r['output'][act])
  print(f"\n[{name}]  active frames: {act.sum()}")
  print(f"  pid pos_limit: min {r['pos_limit'].min():.3f} max {r['pos_limit'].max():.3f}")
  print(f"  |integrator|:  p50 {np.percentile(i_a, 50):.3f}  p99 {np.percentile(i_a, 99):.3f}  max {i_a.max():.3f}")
  print(f"  |output|:      p99 {np.percentile(out_a, 99):.3f}  max {out_a.max():.3f}  frames>1.0: {(out_a > 1.0 + 1e-6).sum()}")
  print(f"  frames |i|>1:  {(i_a > 1.0).sum()}   frames |i|>0.5: {(i_a > 0.5).sum()}")
  logged = np.abs(r['logged_output'][act])
  rms = float(np.sqrt(np.mean((np.abs(r['output'][act]) - logged) ** 2)))
  print(f"  RMS |output| vs logged stock: {rms:.4f}")


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('segments', nargs='+')
  ap.add_argument('--plots', action='store_true')
  args = ap.parse_args()

  segs = sorted(Path(s) for s in args.segments)
  print(f"loading {len(segs)} segments...")
  frames, cp = load_frames(segs)
  fingerprint = cp.carFingerprint
  print(f"{len(frames)} controlsState frames, car: {fingerprint}")

  results = {}
  for name in ('stock', 'shipped', 'fixed'):
    with OpenpilotPrefix():
      results[name] = run_variant(frames, fingerprint, name)
    report(name, results[name])

  if args.plots:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t = np.arange(len(frames)) / 100.0
    fig, axes = plt.subplots(3, 1, figsize=(16, 10), sharex=True)
    axes[0].plot(t, results['shipped']['output'], label='shipped', alpha=0.7, lw=0.7)
    axes[0].plot(t, results['fixed']['output'], label='fixed', alpha=0.7, lw=0.7)
    axes[0].plot(t, results['fixed']['logged_output'], label='logged stock', alpha=0.5, lw=0.7, color='gray')
    axes[0].axhline(1.0, color='r', ls=':', lw=0.8)
    axes[0].axhline(-1.0, color='r', ls=':', lw=0.8)
    axes[0].set_ylabel('torque (norm)')
    axes[0].legend(loc='upper right')
    axes[1].plot(t, results['shipped']['i'], label='shipped i', alpha=0.8, lw=0.7)
    axes[1].plot(t, results['fixed']['i'], label='fixed i', alpha=0.8, lw=0.7)
    axes[1].set_ylabel('integrator')
    axes[1].legend(loc='upper right')
    axes[2].plot(t, results['fixed']['v_ego'], lw=0.7, color='k')
    axes[2].set_ylabel('vEgo (m/s)')
    axes[2].set_xlabel('t (s)')
    fig.suptitle('Jerk-aware torque controller replay: shipped vs fixed (open loop, logged drive)')
    out = REPO_ROOT / 'docs' / 'jerk-aware-replay.png'
    fig.savefig(out, dpi=110, bbox_inches='tight')
    print(f"\nplot: {out}")


if __name__ == '__main__':
  main()
