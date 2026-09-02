#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Replay a recorded segment through the real modeld, on the accelerator.

The bench tools stop at the link: bench_link measures the round trip and
verify_parity checks the numbers against onnxruntime, but both drive the client
directly. Nothing had ever put a real camera frame through modeld's own warp,
its own ModelState and its own modelV2 parsing while an accelerator ran the
model. This does, using openpilot's process_replay so the code under test is
the shipped modeld and not a copy of it.

Runs entirely on the device against a segment already in /data/media, so it
needs no network and no route download.

    # jetlinkd owns the link offroad, so hand it over first
    kill -TERM $(pgrep -f accelerators.jetlink.jetlinkd)
    PYTHONPATH=/data/openpilot:/data/replaydeps tools/jetlink_replay.py \
        --segment /data/media/0/realdata/00000151--cdb98c775d--9
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

# The camera state each stream is fed from, and the file it is recorded in.
CAMERAS = {
  'narrowRoadCameraState': 'fcamera.hevc',
  'wideRoadCameraState': 'ecamera.hevc',
}


class HevcFrameReader:
  """The slice of tools.lib.FrameReader that process_replay actually uses.

  The real one shells out to ffmpeg, which AGNOS does not ship. PyAV carries
  its own, and decoding on the device beats moving 3.5 MB a frame of raw NV12
  over wifi.
  """

  pix_fmt = 'nv12'

  def __init__(self, path: Path, limit: int):
    import av
    self.frames: list[np.ndarray] = []
    self.base_id: int | None = None
    with av.open(str(path)) as container:
      stream = container.streams.video[0]
      stream.thread_type = 'AUTO'
      for i, frame in enumerate(container.decode(stream)):
        if i >= limit:
          break
        # (h*3//2, w) for nv12: the Y plane then the interleaved UV rows, which
        # is the flat layout process_replay slices back apart.
        self.frames.append(frame.reformat(format='nv12').to_ndarray().reshape(-1))
    if not self.frames:
      raise SystemExit(f"no frames decoded from {path}")
    self.h = stream.height
    self.w = stream.width

  def get(self, frame_id: int) -> np.ndarray:
    # frameId counts from the start of the drive, not the segment, so the
    # first one we are asked for defines the offset into this file.
    if self.base_id is None:
      self.base_id = frame_id
    idx = frame_id - self.base_id
    return self.frames[min(max(idx, 0), len(self.frames) - 1)]


def trim_to_frames(msgs: list, n: int) -> list:
  """Keep only as much of the segment as we decoded frames for.

  Camera states run the whole minute, so replaying all of them against a
  60-frame reader would feed the last decoded frame over and over and report
  1200 results from 60 images.
  """
  out, seen = [], 0
  for m in msgs:
    if m.which() == 'narrowRoadCameraState':
      seen += 1
      if seen > n:
        break
    out.append(m)
  return out


def model_name() -> str:
  """Which large model this run will put on the accelerator."""
  from openpilot.sunnypilot.accelerators.jetlink import helpers
  chosen = helpers.selected_model()
  if chosen is None:
    return 'unknown'
  return f"{chosen['name']} ({chosen['size'] / 1e6:.0f} MB, oid {chosen['oid'][:16]})"


def jetlink_params() -> dict:
  """Carry the accelerator's provisioning into the replay's params.

  process_replay gives the process a private PARAMS_ROOT, so without this
  modeld sees no cached spec, decides no accelerator is ready and quietly runs
  the small model - which looks like a pass everywhere except modelV2.big.
  """
  from openpilot.common.params import Params
  params = Params()
  out: dict = {'JetlinkEnabled': True}
  for key in ('JetlinkEngineReady', 'JetlinkSpec', 'JetlinkModel', 'JetlinkEndpoint'):
    value = params.get(key)
    if value is not None:
      out[key] = value
  return out


ONLINE = Path('/sys/devices/system/cpu/online')


def big_cores_up() -> bool:
  """Bring the big cluster online so modeld can pin itself as it does onroad.

  modeld opens with config_realtime_process(7, 54), and openpilot parks cores
  4-7 while the device is offroad - so on a bench sched_setaffinity raises
  EINVAL and modeld dies before it loads a model. This is the same call
  hardwared makes at ignition rather than a poke at sysfs behind its back, so
  the timings below are measured against the scheduling modeld really gets.
  """
  from openpilot.common.hardware import HARDWARE
  try:
    HARDWARE.set_power_save(False)
  except Exception as e:
    print(f"  could not leave power save: {e}")
  return ONLINE.read_text().strip() == '0-7'


def unpin() -> None:
  """Last resort when the big cores will not come up: run modeld unpinned.

  It still exercises the whole model path, but the frame times are then the
  bench's rather than the car's, so they are reported with that caveat.
  """
  from openpilot.common import realtime
  realtime.set_core_affinity = lambda cores: None


def summarise(msgs) -> int:
  model = [m for m in msgs if m.which() == 'modelV2']
  if not model:
    print("FAIL: modeld produced no modelV2")
    return 1

  big = [bool(getattr(m.modelV2, 'big', False)) for m in model]
  exec_ms = np.array([m.modelV2.modelExecutionTime for m in model]) * 1e3
  drops = np.array([m.modelV2.frameDropPerc for m in model])
  # Frame 0 is the warmup: the engine load and the first queue fill land there.
  steady = exec_ms[1:] if len(exec_ms) > 1 else exec_ms

  p50, p90 = np.percentile(steady, 50), np.percentile(steady, 90)
  print(f"\nmodelV2 messages: {len(model)}")
  print(f"  big (accelerator ran it): {sum(big)}/{len(big)}")
  print(f"  execution time ms: mean {steady.mean():.2f}  p50 {p50:.2f}  p90 {p90:.2f}  max {steady.max():.2f}")
  print(f"  first frame (engine load + queue fill): {exec_ms[0]:.1f} ms")
  print(f"  frame drop: mean {drops.mean():.2f}%  max {drops.max():.2f}%")

  last = model[-1].modelV2
  lanes = np.array(last.laneLines[1].y) if len(last.laneLines) > 1 else np.array([])
  plan_x = np.array(last.position.x) if len(last.position.x) else np.array([])
  print("  last frame sanity:")
  if lanes.size:
    ok = bool(np.all(np.isfinite(lanes)))
    print(f"    laneLines[1].y   n={lanes.size} finite={ok} range [{lanes.min():.2f}, {lanes.max():.2f}]")
  else:
    print("    laneLines empty")
  if plan_x.size:
    ok = bool(np.all(np.isfinite(plan_x)))
    print(f"    position.x       n={plan_x.size} finite={ok} reaches {plan_x.max():.1f} m")
  else:
    print("    position empty")
  print(f"    desiredCurvature {last.action.desiredCurvature:+.5f}")
  if len(last.leadsV3):
    print(f"    leadProb         {last.leadsV3[0].prob:.3f}")

  bad = []
  if not all(big):
    bad.append(f"{len(big) - sum(big)} frames ran on the small model")
  if steady.max() > 50:
    bad.append(f"execution time peaked at {steady.max():.1f} ms, over the 50 ms budget")
  if lanes.size and not np.all(np.isfinite(lanes)):
    bad.append("lane lines contain non-finite values")
  if bad:
    print("\nFAIL: " + "; ".join(bad))
    return 1
  print("\nOK: every frame ran on the accelerator, inside the frame budget, with finite outputs")
  return 0


def main() -> int:
  p = argparse.ArgumentParser()
  p.add_argument('--segment', required=True, help='a directory under /data/media/0/realdata')
  p.add_argument('--frames', type=int, default=60)
  args = p.parse_args()

  seg = Path(args.segment)
  rlog = next((seg / n for n in ('rlog.zst', 'rlog') if (seg / n).exists()), None)
  if rlog is None:
    raise SystemExit(f"no rlog in {seg}")

  from openpilot.selfdrive.test.process_replay.process_replay import (
    get_custom_params_from_lr,
    get_process_config,
    replay_process,
  )
  from openpilot.tools.lib.logreader import LogReader

  full = list(LogReader(str(rlog)))
  # The calibration and live-parameter snapshots process_replay wants are
  # scattered through the whole segment, so read them off the full log and
  # only then cut it down to the frames we decoded.
  custom = get_custom_params_from_lr(full)
  custom.update(jetlink_params())
  lr = trim_to_frames(full, args.frames)
  # Cutting the log can drop the carParams that process_replay would otherwise
  # fingerprint from, so name the car outright instead.
  fingerprint = next((m.carParams.carFingerprint for m in full if m.which() == 'carParams'), None)
  print(f"segment {seg.name}: {len(lr)} of {len(full)} messages, {args.frames} frames")
  print(f"  car: {fingerprint}")
  print(f"  model: {model_name()}")

  frs = {}
  for state, name in CAMERAS.items():
    path = seg / name
    if not path.exists():
      raise SystemExit(f"missing {path}")
    frs[state] = HevcFrameReader(path, args.frames)
    print(f"  {state}: {len(frs[state].frames)} frames at {frs[state].w}x{frs[state].h}")

  pinned = big_cores_up()
  how = 'pinned as onroad' if pinned else 'UNPINNED, timings are not representative'
  print(f"  cores online: {ONLINE.read_text().strip()} ({how})")
  if not pinned:
    unpin()

  out = replay_process(get_process_config('modeld'), lr, frs,
                       fingerprint=fingerprint, custom_params=custom)
  return summarise(out)


if __name__ == '__main__':
  sys.exit(main())
