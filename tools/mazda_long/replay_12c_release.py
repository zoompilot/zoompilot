#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Replay route 12c's three latched stops through the current StandstillHold and check both
halves of the release rework:

1. Timing: the pulse must fire the frame the debounced release lands (the on-car build
   deferred it 2.0 s behind the falsified nudge -- the resume delay the driver reported).
2. Track status: the occupied-slot constants must carry drive_0b's at-release signature
   (byte1 low nibble 0xe, byte4 low bits 0x1c, byte5 0x00), attested in the stock corpus,
   not the retired capture's 1d/c0 -- c0 in byte 5 is the empty-slot signature that was on
   the wire under all 10 SCBS-latching pulses.

Usage: .venv/bin/python3 tools/mazda_long/replay_12c_release.py [route_dir_prefix]
"""
import glob
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from openpilot.tools.lib.logreader import LogReader  # noqa: E402
from opendbc.car.mazda import mazdacan  # noqa: E402
from opendbc.car.mazda.longitudinal import RELEASE_DEBOUNCE_FRAMES, StandstillHold  # noqa: E402
from opendbc.car.structs import CarControl  # noqa: E402

ROUTE = sys.argv[1] if len(sys.argv) > 1 else "tools/mazda_long/device_data/0000012c--55f3ecf13c"


def load_inputs(segs):
  """One row per carControl frame: (t, enabled, stopping, accel, standstill, gas, brake_hold)."""
  rows = []
  standstill = gas = False
  brake_hold = 0
  for p in segs:
    for m in LogReader(p):
      w = m.which()
      if w == "carState":
        standstill, gas = m.carState.standstill, m.carState.gasPressed
      elif w == "can":
        for c in m.can:
          if c.src == 0 and c.address == 0x228 and len(c.dat) >= 3:
            brake_hold = (bytes(c.dat)[2] >> 4) & 1
      elif w == "carControl":
        cc = m.carControl
        stopping = cc.actuators.longControlState == CarControl.Actuators.LongControlState.stopping
        rows.append((m.logMonoTime * 1e-9, cc.enabled, stopping, cc.actuators.accel,
                     standstill, gas, bool(brake_hold)))
  return rows


def main():
  segs = sorted(glob.glob(f"{ROUTE}--*/rlog.zst"),
                key=lambda p: int(p.split("--")[-1].split("/")[0]))
  if not segs:
    raise SystemExit(f"no rlogs under {ROUTE}--*/")
  rows = load_inputs(segs)
  t0 = rows[0][0]

  sm = StandstillHold()
  was_holding = False
  release_frame = None
  latencies = []
  for i, (t, enabled, stopping, accel, standstill, gas, hold) in enumerate(rows):
    sm.update(enabled, stopping, standstill, accel, hold, gas)
    if was_holding and not sm.holding and sm.latched_release:
      release_frame = (i, t)
    if release_frame is not None and sm.resume_unlatching:
      lag = i - release_frame[0]
      latencies.append((release_frame[1] - t0, lag))
      release_frame = None
    was_holding = sm.holding

  print(f"replayed {len(rows)} control frames, {len(latencies)} latched release(s)")
  ok = True
  for trel, lag in latencies:
    print(f"  t+{trel:7.2f}s  release -> pulse lag: {lag} frames "
          f"(on-car build: ~{RELEASE_DEBOUNCE_FRAMES + 200} frames behind the plan)")
    ok &= lag == 0

  tmpl = mazdacan.LEAD_TRACK_TEMPLATE
  sig = (tmpl[1] & 0x0F, tmpl[4] & 0x1F, tmpl[5])
  print(f"track status bits: byte1lo={sig[0]:#x} byte4lo={sig[1]:#x} byte5={sig[2]:#x} "
        f"(stock drive_0b at-release: 0xe/0x1c/0x00; retired capture: 0x0/0x1d/0xc0)")
  ok &= sig == (0x0E, 0x1C, 0x00)

  print("PASS" if ok else "FAIL")
  return 0 if ok else 1


if __name__ == "__main__":
  raise SystemExit(main())
