#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Replay logged drives through the v0 and v2 torque tunes with speed-dependent torque
enabled, reporting what the v2 mechanisms change on real CX-5 inputs.

Variants:
  v0  upstream's algorithm (harness sanity on a v0 drive: must track the logged outputs)
  v2  v0's algebra plus the curvature buffer, the filtered-jerk friction input, KD and the
      release handling (docs/zoompilot/lateral-tune-roadmap.md)

Reports the integrator level, the desired-jerk distribution, the output delta vs v0, the
release-window transients and an entry-lead table (signed v2-vs-v0 output advance while
the request rises into a turn).

Open-loop caveat: the car in the log was driven by whichever tune was on it, so errors do
not converge the way they would closed-loop. The comparison is still valid for friction
activity, integrator behavior and release transients, because all variants see identical
inputs. A large RMS vs logged for the tune that drove the route means the harness is wrong.

Usage:
  python tools/mazda_long/replay_torque_v2.py tools/mazda_long/device_data/000000ab--* [--plots]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from openpilot.common.prefix import OpenpilotPrefix
from replay_jerk_aware import load_frames  # shared rlog-to-controller-input adapter


def make_controller(fingerprint, version: int):
  from opendbc.car.car_helpers import interfaces
  from openpilot.common.params import Params
  from openpilot.common.realtime import DT_CTRL
  from openpilot.selfdrive.car.helpers import convert_to_capnp
  from openpilot.sunnypilot.selfdrive.car import interfaces as sunnypilot_interfaces
  from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0
  from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v2 import LatControlTorque as LatControlTorqueV2
  from opendbc.car.vehicle_model import VehicleModel

  params = Params()
  for k in ("EnforceTorqueControl", "LiveTorqueParamsToggle", "SpeedDependentTorqueToggle"):
    params.put_bool(k, True, block=True)
  params.put_bool("LateralJerkTorqueController", False, block=True)
  params.put_bool("NeuralNetworkLateralControl", False, block=True)

  CarInterface = interfaces[fingerprint]
  CP = CarInterface.get_non_essential_params(fingerprint)
  CP_SP = CarInterface.get_non_essential_params_sp(CP, fingerprint)
  CI = CarInterface(CP, CP_SP)
  sunnypilot_interfaces.setup_interfaces(CI, params)
  CP_SP = convert_to_capnp(CP_SP)
  VM = VehicleModel(CP)
  cls = LatControlTorqueV0 if version == 0 else LatControlTorqueV2
  controller = cls(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)
  return controller, VM, CP


def run_variant(frames, fingerprint, version: int):
  controller, VM, CP = make_controller(fingerprint, version)

  lat_delay = CP.steerActuatorDelay
  last_ltp = None
  keys = ('output', 'i', 'jerk', 'request', 'active', 'pressed', 'v_ego', 'logged_output')
  out = {k: [] for k in keys}
  for f in frames:
    # controlsd: global filtered params + extension limits, every frame the service is alive
    if f.ltp is not None and f.ltp.useParams:
      controller.update_torque_parameters(f.ltp.latAccelFactorFiltered, f.ltp.latAccelOffsetFiltered, f.ltp.frictionCoefficientFiltered)
      controller.extension.update_limits()
      # controlsd_ext: per-bin values on each new lateralTorqueParameters message
      if f.ltp is not last_ltp:
        controller.extension.update_speed_dep_torque(f.ltp, f.ltp_sp)
        last_ltp = f.ltp
    controller.extension.update_lateral_lag(lat_delay)

    CS = SimpleNamespace(vEgo=f.v_ego, aEgo=f.a_ego, steeringAngleDeg=f.steering_angle,
                         steeringRateDeg=f.steering_rate, steeringPressed=f.steering_pressed)
    lp = SimpleNamespace(roll=f.roll, angleOffsetDeg=f.angle_offset)
    _, _, pid_log = controller.update(f.active, CS, VM, lp, False, f.desired_curvature, None, False, lat_delay)

    # inactive frames leave the capnp log fields at their 0.0 defaults
    out['output'].append(-pid_log.output)
    out['i'].append(controller.pid.i)
    out['jerk'].append(pid_log.desiredLateralJerk)
    out['request'].append(f.desired_curvature * f.v_ego ** 2)
    out['active'].append(f.active)
    out['pressed'].append(f.steering_pressed)
    out['v_ego'].append(f.v_ego)
    out['logged_output'].append(f.logged_output)
  return {k: np.array(v) for k, v in out.items()}


def report(name, r, base=None):
  act = r['active'].astype(bool)
  out_a, i_a, jerk_a = r['output'][act], np.abs(r['i'][act]), np.abs(r['jerk'][act])
  logged = -r['logged_output'][act]  # logged field is pid_log.output = -torque; compare in torque convention
  rms_logged = float(np.sqrt(np.mean((out_a - logged) ** 2)))
  print(f"\n[{name}]  active frames: {act.sum()}")
  print(f"  RMS output vs logged v0 drive: {rms_logged:.4f}")
  print(f"  |integrator|: p50 {np.percentile(i_a, 50):.3f}  p99 {np.percentile(i_a, 99):.3f}  max {i_a.max():.3f}")
  print(f"  |desired jerk|: p50 {np.percentile(jerk_a, 50):.3f}  p99 {np.percentile(jerk_a, 99):.3f}  max {jerk_a.max():.3f}")
  if base is not None:
    b_out = base['output'][act]
    diff = out_a - b_out
    print(f"  output delta vs v0: RMS {np.sqrt(np.mean(diff ** 2)):.4f}  p99 |d| {np.percentile(np.abs(diff), 99):.4f}  max |d| {np.abs(diff).max():.4f}")
    # release transients: 1 s window after each steeringPressed falling edge
    pressed = r['pressed'].astype(bool)
    edges = np.flatnonzero(pressed[:-1] & ~pressed[1:]) + 1
    win = []
    for e in edges:
      sl = slice(e, min(e + 100, len(r['output'])))
      if r['active'][sl].all():
        win.append(np.abs(r['output'][sl] - base['output'][sl]).max())
    if win:
      print(f"  release windows ({len(win)}): max |output delta| p50 {np.percentile(win, 50):.4f}  max {max(win):.4f}")
    # entry lead: signed output advance vs v0 while the request rises into a turn
    req = r['request']
    rising = np.abs(req)
    crossings = np.flatnonzero((rising[:-1] < 0.5) & (rising[1:] >= 0.5) & (np.abs(req[1:]) > np.abs(req[:-1]))) + 1
    lead = []
    for c in crossings:
      sl = slice(c, min(c + 50, len(req)))
      if not (r['active'][sl].all() and (r['v_ego'][sl] > 10.0).all()):
        continue
      turn_sign = np.sign(req[c])
      lead.append(((r['output'][sl] - base['output'][sl]) * turn_sign).max())
    if lead:
      print(f"  entry windows ({len(lead)}): signed lead vs v0 p50 {np.percentile(lead, 50):.4f}  p90 {np.percentile(lead, 90):.4f}  min {min(lead):.4f}")


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('segments', nargs='+')
  ap.add_argument('--plots', action='store_true')
  args = ap.parse_args()

  segs = sorted(Path(s) for s in args.segments)
  print(f"loading {len(segs)} segments...")
  # neither tune reads the model with the override controllers off; dropping it keeps long
  # routes from pinning every segment's rlog buffer in memory
  frames, cp = load_frames(segs, keep_model=False)
  fingerprint = cp.carFingerprint
  print(f"{len(frames)} controlsState frames, car: {fingerprint}")

  results = {}
  for name, version in (('v0', 0), ('v2', 2)):
    with OpenpilotPrefix():
      results[name] = run_variant(frames, fingerprint, version)
    report(name, results[name], base=None if name == 'v0' else results['v0'])

  if args.plots:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    t = np.arange(len(frames)) / 100.0
    fig, axes = plt.subplots(4, 1, figsize=(16, 12), sharex=True)
    axes[0].plot(t, results['v0']['output'], label='v0', alpha=0.7, lw=0.7)
    axes[0].plot(t, results['v2']['output'], label='v2', alpha=0.7, lw=0.7)
    axes[0].plot(t, results['v0']['logged_output'], label='logged drive', alpha=0.4, lw=0.7, color='gray')
    axes[0].set_ylabel('torque (norm)')
    axes[0].legend(loc='upper right')
    axes[1].plot(t, results['v0']['jerk'], label='v0 desired jerk', alpha=0.7, lw=0.7)
    axes[1].plot(t, results['v2']['jerk'], label='v2 desired jerk', alpha=0.7, lw=0.7)
    axes[1].set_ylabel('jerk (m/s^3)')
    axes[1].legend(loc='upper right')
    axes[2].plot(t, results['v0']['i'], label='v0 i', alpha=0.8, lw=0.7)
    axes[2].plot(t, results['v2']['i'], label='v2 i', alpha=0.8, lw=0.7)
    axes[2].set_ylabel('integrator')
    axes[2].legend(loc='upper right')
    axes[3].plot(t, results['v0']['v_ego'], lw=0.7, color='k')
    axes[3].set_ylabel('vEgo (m/s)')
    axes[3].set_xlabel('t (s)')
    fig.suptitle('v2 torque tune replay vs v0 (open loop, logged CX-5 drive)')
    out = REPO_ROOT / 'docs' / 'torque-v2-replay.png'
    fig.savefig(out, dpi=110, bbox_inches='tight')
    print(f"\nplot: {out}")


if __name__ == '__main__':
  main()
