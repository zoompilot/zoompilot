#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Put Set Speed Assist's lift under a clip, outside the UI: the lift the planner published as a share
of the personality's approach ceiling, the gauge's target alpha, and the last 10 s of it as a trace.
Run with the route and range given to run.py, so the frames line up:

  openpilot/tools/clip/run.py ROUTE -d DIR --qcam -s 435 -e 540 --no-metadata --no-time-overlay -o clip.mp4
  openpilot/tools/clip/speed_assist_strip.py ROUTE -d DIR -s 435 -e 540 clip.mp4 -o clip_strip.mp4
"""
import argparse
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from openpilot.common.basedir import BASEDIR
from openpilot.selfdrive.ui.sunnypilot.mici.onroad.hud_renderer import SPEED_ASSIST_ALPHA_IDLE, SPEED_ASSIST_ALPHA_LIFT
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.approach import approach_ceiling
from openpilot.tools.clip.run import FRAMERATE, load_logs_parallel
from openpilot.tools.lib.route import Route

STRIP_H = 56  # px at the UI's 536x240
HISTORY = 10 * FRAMERATE  # frames of trace
PERSONALITY = {0: "Aggressive", 1: "Standard", 2: "Relaxed"}
FONT = str(Path(BASEDIR) / "openpilot/selfdrive/assets/fonts/Inter-Medium.ttf")
TRACE_COLOR = (0x16, 0xA0, 0x50)


def lift_series(route: Route, start: int, end: int) -> list[tuple[float, float, int]]:
  """(lift m/s^2, ceiling m/s^2, personality) per clip frame, chunked exactly as run.py chunks them."""
  seg_start, seg_end = start // 60, (end - 1) // 60 + 1
  chunks = load_logs_parallel(route.log_paths()[seg_start:seg_end], fps=FRAMERATE)
  frame_start = (start - seg_start * 60) * FRAMERATE
  v_ego, personality, lift = 0., 1, 0.
  series = []
  for chunk in chunks[frame_start:frame_start + (end - start) * FRAMERATE]:
    if "carState" in chunk:
      v_ego = chunk["carState"].carState.vEgo
    if "selfdriveState" in chunk:
      personality = chunk["selfdriveState"].selfdriveState.personality.raw
    if "longitudinalPlanSP" in chunk:
      lift = chunk["longitudinalPlanSP"].longitudinalPlanSP.zoompilot.e2eSetSpeed.boost
    series.append((lift, approach_ceiling(v_ego, personality), personality))
  return series


def draw_strip(width: int, scale: int, series, idx: int, font: ImageFont.FreeTypeFont) -> np.ndarray:
  lift, ceiling, personality = series[idx]
  share = min(lift / ceiling, 1.)
  alpha = SPEED_ASSIST_ALPHA_IDLE + (SPEED_ASSIST_ALPHA_LIFT - SPEED_ASSIST_ALPHA_IDLE) * share
  h, pad = STRIP_H * scale, 8 * scale
  img = Image.new("RGB", (width, h), (16, 16, 16))
  d = ImageDraw.Draw(img)
  d.text((pad, pad), f"Set Speed Assist lift {lift:.2f} m/s²", fill=(235, 235, 235), font=font)
  d.text((pad, h // 2 + scale), f"{share * 100:3.0f}% of {PERSONALITY[personality]} {ceiling:.2f}  ·  gauge {alpha:.2f}",
         fill=(150, 150, 150), font=font)

  # the last HISTORY frames of the share, 0..100%, newest at the right edge
  x0, x1, y0, y1 = width // 2 + pad, width - pad, pad, h - pad
  d.rectangle((x0, y0, x1, y1), outline=(60, 60, 60))
  hist = [min(lf / c, 1.) for lf, c, _ in series[max(0, idx - HISTORY + 1):idx + 1]]
  xs = np.linspace(x1 - (x1 - x0) * (len(hist) - 1) / (HISTORY - 1), x1, len(hist))
  pts = [(x, y1 - (y1 - y0) * s) for x, s in zip(xs, hist, strict=True)]
  if len(pts) > 1:
    d.polygon([(pts[0][0], y1), *pts, (pts[-1][0], y1)], fill=tuple(c // 3 for c in TRACE_COLOR))
    d.line(pts, fill=TRACE_COLOR, width=scale)
  return np.asarray(img)


def main() -> None:
  parser = argparse.ArgumentParser(description="Add Set Speed Assist's lift under a run.py clip")
  parser.add_argument("route")
  parser.add_argument("clip", help="mp4 from run.py for the same route and range")
  parser.add_argument("-d", "--data-dir")
  parser.add_argument("-s", "--start", type=int, required=True)
  parser.add_argument("-e", "--end", type=int, required=True)
  parser.add_argument("-o", "--output", default="clip_strip.mp4")
  parser.add_argument("--scale", type=int, default=2, help="upscale the 536x240 UI for viewing")
  args = parser.parse_args()

  series = lift_series(Route(args.route, data_dir=args.data_dir), args.start, args.end)
  w, h = 536 * args.scale, 240 * args.scale
  font = ImageFont.truetype(FONT, 13 * args.scale)

  # pipe:N rather than '-': the minimal ffmpeg in the venv only knows the pipe protocol by name
  dec = subprocess.Popen(["ffmpeg", "-v", "error", "-i", args.clip, "-vf", f"scale={w}:{h}:flags=lanczos",
                          "-f", "rawvideo", "-pix_fmt", "rgb24", "pipe:1"], stdout=subprocess.PIPE)
  enc = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h + STRIP_H * args.scale}",
                          "-r", str(FRAMERATE), "-i", "pipe:0", "-c:v", "libx264", "-crf", "18", "-pix_fmt", "yuv420p",
                          "-movflags", "+faststart", args.output], stdin=subprocess.PIPE)
  assert dec.stdout is not None and enc.stdin is not None
  frame_bytes = w * h * 3
  for idx in range(len(series)):
    buf = dec.stdout.read(frame_bytes)
    if len(buf) < frame_bytes:
      break
    enc.stdin.write(buf)
    enc.stdin.write(draw_strip(w, args.scale, series, idx, font).tobytes())
  enc.stdin.close()
  enc.wait()
  dec.kill()
  print(f"wrote {args.output}")


if __name__ == "__main__":
  main()
