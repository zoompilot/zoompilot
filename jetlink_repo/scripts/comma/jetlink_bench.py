#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of jetlink and is licensed under the MIT License.
See the LICENSE file in the root directory for more details.

Isolated, disengaged camera/modeld bench on the comma. Never starts controls or pandad.

Runs openpilot's own camerad and modeld from the checkout this repo is a
submodule of; see jetlink_live_bench.sh.
"""
import argparse
import csv
import importlib
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

# the modeld manager starts: stock for openpilot's own model, modeld_v2 for a
# sunnypilot (tinygrad) bundle; the frame loops differ before model.run
MODELD = {'stock': 'openpilot.selfdrive.modeld.modeld', 'tinygrad': 'openpilot.sunnypilot.modeld_v2.modeld'}


def main():
  from openpilot.cereal import messaging
  from openpilot.common.basedir import BASEDIR
  from openpilot.common.hardware import HARDWARE
  from openpilot.common.params import Params
  from openpilot.common.prefix import OpenpilotPrefix
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--adapter', default='openpilot.sunnypilot.jetlink_adapter',
                      help="the fork's jetlink adapter module; its Stock subclass names the params and services")
  parser.add_argument('--seconds', type=float, default=180)
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('--small', action='store_true')
  parser.add_argument('--engaged-until', type=float, default=0.0,
                      help='fake engaged controls for this many seconds, so the join has to wait for a window')
  parser.add_argument('--write-chunk', type=int, choices=[8192, 16384],
                      help='bench-only FunctionFS AIO request size; it must divide the 16 KB padding')
  parser.add_argument('--modeld', help="which modeld runs: stock, tinygrad or a module; by default the one zoompilot's "
                      "manager runs for the active driving model, else stock")
  parser.add_argument('--record', action='store_true', help='run loggerd, encoderd and driver monitoring; keep recordings under output')
  parser.add_argument('--resources', action='store_true', help='sample CPU, PSS and VM state once per second')
  args = parser.parse_args()
  fork = importlib.import_module(args.adapter)
  adapter = fork.adapter()
  IN_CONTROL, KEYS = adapter.IN_CONTROL_SERVICES, adapter.KEYS
  args.output = args.output.resolve()
  if not np.isfinite(args.seconds) or args.seconds <= 0:
    parser.error('--seconds must be positive')
  live = Params()
  if not live.get_bool('IsOffroad'):
    raise SystemExit('bench requires the real device to remain offroad')
  if args.modeld is None:
    try:
      from openpilot.cereal import custom
      from openpilot.sunnypilot.models.helpers import get_active_model_runner
      tinygrad = get_active_model_runner(live, force_check=True) == custom.ModelManagerSP.Runner.tinygrad
      args.modeld = 'tinygrad' if tinygrad else 'stock'
    except ImportError:   # not a sunnypilot fork
      args.modeld = 'stock'
  modeld_module = MODELD.get(args.modeld, args.modeld)
  # Cameras are a physical resource, even with isolated messaging. jetlinkd is
  # not: it owns the gadget and lends the endpoints, which is what a drive does
  # too, so leaving it up is the arrangement under test rather than a conflict.
  for proc in Path('/proc').glob('[0-9]*/cmdline'):
    try:
      argv = proc.read_bytes().split(b'\0')
    except (OSError, ProcessLookupError):
      continue
    if argv and (Path(os.fsdecode(argv[0])).name == 'camerad' or
                 any(b'openpilot.selfdrive.modeld.modeld' in arg or b'modeld_v2' in arg or
                     b'openpilot.selfdrive.modeld.dmonitoringmodeld' in arg for arg in argv) or
                 Path(os.fsdecode(argv[0])).name in ('encoderd', 'loggerd')):
      raise SystemExit(f'physical resource already owned by {proc.parent.name}: {argv[:3]}')
  # the slot names a ref; without the catalog it names nothing the picker can
  # find, and the join asks for a model that has not been picked
  keys = ('CarParamsPersistent', 'CalibrationParams', 'ModelManager_ActiveBundle', 'ModelManager_ActiveBundleChestnut',
          'ModelManager_ModelsCache_Chestnut', 'RecordFront', 'IsRhdDetected', 'ExperimentalMode',
          KEYS.pointers, KEYS.spec, KEYS.big_model, KEYS.catalog)
  # the adapter's get is None for a key this fork does not have
  saved = {key: adapter.get(key) for key in dict.fromkeys(k for k in keys if k)}
  if saved['CarParamsPersistent'] is None:
    raise SystemExit('no saved CarParams; bench cannot choose a vehicle configuration')
  args.output.mkdir(parents=True, exist_ok=False)
  if args.record and shutil.disk_usage(args.output).free < 2 << 30:
    raise SystemExit('recording bench requires at least 2 GiB free')
  stop = False

  def stop_requested(*_):
    nonlocal stop
    stop = True

  signal.signal(signal.SIGTERM, stop_requested)
  signal.signal(signal.SIGINT, stop_requested)
  children, files, rows = [], [], []
  failure = None
  # Keep artifacts under the explicit output directory; no live Params writes.
  with OpenpilotPrefix() as prefix:
    params = Params()
    for key, value in saved.items():
      if value is not None:
        params.put(key, value, block=True)
    params.put('CarParams', saved['CarParamsPersistent'], block=True)
    params.put(KEYS.link, 0 if args.small else 1, block=True)   # Jetlink off, or USB
    pm = messaging.PubMaster([*IN_CONTROL, 'deviceState', 'extrinsicsCalibration'])
    sm = messaging.SubMaster(['modelV2'])
    hooks = fork.hooks()
    calibration = None
    if saved['CalibrationParams'] is not None:
      calibration = messaging.log_from_bytes(saved['CalibrationParams'])
    child_env = dict(os.environ, LOG_ROOT=str(args.output / 'recordings'))
    if args.record:
      params.put('RecordFront', True, block=True)
    print(f'isolated prefix={prefix.prefix} output={args.output} modeld={args.modeld}', flush=True)
    HARDWARE.set_power_save(False)
    try:
      # -m puts the installed checkout ahead of PYTHONPATH. Always test the
      # package this script belongs to, including a staged candidate in /data/tmp.
      package = str(Path(__file__).resolve().parents[2])
      setup = f'import sys; sys.path.insert(0, {package!r}); '
      if args.write_chunk is not None:
        setup += f'from jetlink.transport.ffs import FfsTransport; FfsTransport.write_chunk={args.write_chunk}; '
      model_command = [sys.executable, '-c', setup +
                       f'import runpy; runpy.run_module({modeld_module!r}, run_name="__main__")']
      commands = [
        ('camerad', [str(Path(BASEDIR) / 'openpilot/system/camerad/camerad')]),
        ('modeld', model_command)]
      if args.record:
        # AGNOS otherwise writes Python logs to the live /data/log even with
        # an isolated prefix. Keep those files with the recording artifacts.
        log_setup = ('from openpilot.common.hardware.hw import Paths; '
                     f'Paths.swaglog_root=staticmethod(lambda: {str(args.output / "swaglog")!r}); '
                     'import runpy; runpy.run_module("openpilot.system.logmessaged", run_name="__main__")')
        commands.extend([
          ('logmessaged', [sys.executable, '-c', log_setup]),
          ('loggerd', [str(Path(BASEDIR) / 'openpilot/system/loggerd/loggerd')]),
          ('encoderd', [str(Path(BASEDIR) / 'openpilot/system/loggerd/encoderd')]),
          ('dmonitoringmodeld', [sys.executable, '-m', 'openpilot.selfdrive.modeld.dmonitoringmodeld']),
        ])
      for name, command in commands:
        log = (args.output / f'{name}.log').open('w')
        files.append(log)
        children.append(subprocess.Popen(command, cwd=BASEDIR, env=child_env, stdout=log, stderr=subprocess.STDOUT))
      if args.resources:
        log = (args.output / 'resources.log').open('w')
        files.append(log)
        sampler = [sys.executable, str(Path(__file__).with_name('bench_resources.py')),
                   '--output', str(args.output / 'resources.jsonl'), *[str(child.pid) for child in children]]
        children.append(subprocess.Popen(sampler, stdout=log, stderr=subprocess.STDOUT))
      start = last = time.monotonic()
      tick = 0
      runtime, runtime_at = None, 0.0
      with (args.output / 'frames.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['elapsed_s', 'frame_id', 'big', 'valid', 'exec_ms', 'drop_pct', 'age_ms', 'accel_state', 'available'])
        while not stop and time.monotonic() - start < args.seconds:
          if any(child.poll() is not None for child in children):
            raise RuntimeError('bench process exited; inspect captured logs')
          if tick % 100 == 0 and args.record and shutil.disk_usage(args.output).free < 2 << 30:
            raise RuntimeError('stopping recording bench: less than 2 GiB disk space free')
          if tick % 100 == 0 and not live.get_bool('IsOffroad'):
            raise RuntimeError('real ignition changed: stopping bench')
          engaged = time.monotonic() - start < args.engaged_until
          for service in IN_CONTROL:
            message = messaging.new_message(service)
            message.valid = True
            # what the adapter's in_control reads before every frame of the
            # joining model; faking it engaged holds the join back
            if service == 'carControl':
              message.carControl.enabled = engaged
            elif service == 'carControlSP':
              message.carControlSP.mads.enabled = engaged
            elif service == 'selfdriveState':
              message.selfdriveState.enabled = message.selfdriveState.active = engaged
            pm.send(service, message)
          # at the car's rates (services.py): modeld recomputes both warp
          # matrices on every calibration, 0.55 ms of its frame
          if tick % 50 == 0:
            device = messaging.new_message('deviceState')
            device.valid = True
            device.deviceState.deviceType = HARDWARE.get_device_type()
            pm.send('deviceState', device)
          if tick % 25 == 0 and calibration is not None:
            message = calibration.as_builder()
            message.logMonoTime = time.monotonic_ns()
            pm.send('extrinsicsCalibration', message)
          sm.update(0)
          if sm.updated['modelV2']:
            model = sm['modelV2']
            if time.monotonic() - runtime_at >= 1.0:   # modeld rewrites its record once a second
              runtime, runtime_at = hooks.runtime(), time.monotonic()
            state = getattr(runtime, 'state', None) or 'none'
            row = (time.monotonic() - start, model.frameId, int(model.big), int(sm.valid['modelV2']),
                   model.modelExecutionTime * 1000, model.frameDropPerc,
                   (sm.logMonoTime['modelV2'] - model.timestampEof) / 1e6,
                   # the available column is the state being ready, as bigModelAvailable was
                   state, int(state == 'ready'))
            rows.append(row)
            writer.writerow(row)
          if time.monotonic() - last >= 10:
            last = time.monotonic()
            stream.flush()
            print(f't={last-start:.0f}s frames={len(rows)} big={sum(r[2] for r in rows)} engaged={engaged}',
                  f'latest={rows[-1] if rows else None}', flush=True)
          tick += 1
          if tick % 100 == 0:
            # parked, power saving comes back on and takes cores 4-7, which
            # slows the comma's share of every frame; a drive keeps it off
            HARDWARE.set_power_save(False)
          time.sleep(max(0, start + tick * 0.01 - time.monotonic()))
    except Exception as error:
      failure = error
    finally:
      stopping = time.monotonic()
      for child in reversed(children):
        if child.poll() is None:
          child.send_signal(signal.SIGINT)
      for child in children:
        name = child.args[-1]
        try:
          child.wait(timeout=5)
        except subprocess.TimeoutExpired:
          child.kill()
          try:
            child.wait(timeout=30)
          except subprocess.TimeoutExpired:
            # stuck in the kernel; the numbers are still worth keeping
            print(f'{name} still running 35 s after SIGINT', flush=True)
            continue
        print(f'{name} exited {time.monotonic() - stopping:.1f} s after SIGINT', flush=True)
      for log in files:
        log.close()
      # Power saving belongs to hardwared if real ignition changed.
      if live.get_bool('IsOffroad'):
        HARDWARE.set_power_save(True)
  summary = {'interrupted': stop}
  if failure is not None:
    summary['error'] = f'{type(failure).__name__}: {failure}'
  for label, subset in [('all', rows), ('big', [r for r in rows if r[2]]), ('small', [r for r in rows if not r[2]])]:
    if subset:
      # the state column is text, so pick the numeric columns before numpy sees the rows
      values = np.asarray([row[:7] for row in subset], dtype=float)
      summary[label] = {'frames': len(subset), 'exec_p50_p99_p999_max_ms': np.percentile(values[:, 4], [50, 99, 99.9, 100]).tolist(),
                        'over_50ms': int((values[:, 4] > 50).sum()), 'max_drop_pct': float(values[:, 5].max()),
                        'lagging_frames': int((values[:, 5] > 1).sum()), 'invalid_frames': int((values[:, 3] == 0).sum()),
                        'states': sorted({row[7] for row in subset})}
  if args.record:
    segments = sorted((args.output / 'recordings').glob('*--*'))
    complete = [p.name for p in segments if all((p / name).exists() and (p / name).stat().st_size > 0
                                               for name in ('rlog.zst', 'fcamera.hevc', 'ecamera.hevc', 'dcamera.hevc'))]
    summary['recording'] = {'segments': len(segments), 'complete_segments': complete}
  (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
  print(json.dumps(summary, indent=2), flush=True)
  if failure is not None:
    raise failure
  if args.record and not summary['recording']['complete_segments']:
    raise SystemExit('recording workload did not produce logs and all three camera files')
  if args.record and args.seconds >= 150 and len(summary['recording']['complete_segments']) < 2:
    raise SystemExit('recording workload did not exercise segment rotation')
  if not rows or (not args.small and not any(row[2] for row in rows)):
    raise SystemExit('bench did not exercise the requested model')


if __name__ == '__main__':
  main()
