"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Replay route 115 seg 6 (the 2026-08-27 SCBS latch: body-latched hold release whose command
climbed +25 raw/frame from the first pulse frame, 2 wire frames before the body dropped
GEAR.BRAKE_HOLD) through the CarController and validate the emission against the stock
latched-release grammar.

The validator extends replay_53's census rules with the latched-family pin: in every stock
body-latched release (18 pulses), ACCEL_CMD sits at -1 raw until BRAKE_HOLD drops, and only
then climbs. It is sanity-checked against a stock latched release (drive_04 t=2328.4) before
judging us.
"""
import sys, os, glob
sys.path.insert(0, "/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot")
sys.path.insert(0, "/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot/opendbc_repo")
sys.path.append("/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot/tools/mazda_long")
os.chdir("/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot")
from openpilot.tools.lib.logreader import LogReader
from replay_standstill_hold import build_controller, decode_cmd, mock_inputs
from replay_53 import validate

LOG = "tools/mazda_long/device_data/00000115--f8526a6f47--6/rlog.zst"


def validate_latched(rows):
  """rows: [(t, cmd_raw, stop, unl, hold)] at the wire rate. Returns violations.

  replay_53's census validator carries the shared latched-family rules (band, pulse run
  length, stop/unlatch exclusion); the one rule new here is the pin: stock holds the
  command at -1 raw on every pulse frame the body is still latched for."""
  bad = validate([r[:4] for r in rows], latched=True)
  bad += [(t, f"pulse cmd {cmd:+d} while BRAKE_HOLD still latched (stock pins -1)")
          for t, cmd, stop, unl, hold in rows if unl and hold and not -16 <= cmd <= 0]
  return sorted(bad)


def stock_twin():
  """A stock body-latched release (drive_04 t=2328.4, 9 fr, hold drop @ fr3) for sanity."""
  rows = []
  hold = 0
  paths = sorted(glob.glob("tools/mazda_long/test_data/drive_04/rlog*"),
                 key=lambda p: int(p.split("seg")[1].split(".")[0]))
  for p in paths:
    done = False
    for m in LogReader(p):
      if m.which() != "can":
        continue
      t = m.logMonoTime * 1e-9
      if t > 2331.0:
        done = True
        break
      if t < 2326.0:
        continue
      for c in m.can:
        if c.address == 0x228 and c.src == 0:
          hold = (bytes(c.dat)[2] >> 4) & 1
        elif c.address == 0x21b and c.src == 0:
          dat = bytes(c.dat)
          rows.append((t, decode_cmd(dat), (dat[5] >> 2) & 1, (dat[6] >> 6) & 1, hold))
    if done:
      break
  return rows


def replay_115():
  rows = []
  cc = cs = lead = None
  brake_hold = False
  for m in LogReader(LOG):
    w = m.which()
    if w == "carControl":
      cc = m.carControl
    elif w == "carControlSP":
      lo = m.carControlSP.leadOne
      lead = (lo.dRel, lo.vRel)
    elif w == "can":
      for c in m.can:
        if c.address == 0x228 and c.src == 0:
          brake_hold = bool((bytes(c.dat)[2] >> 4) & 1)
    elif w == "carState":
      cs = m.carState
      if cc is not None:
        rows.append((m.logMonoTime * 1e-9, cc, cs, lead, brake_hold))

  ctrl = build_controller()
  out = []
  for t, cc, cs, lead, brake_hold in rows:
    control, control_sp, carstate = mock_inputs(cc, cs, brake_hold, lead)
    sends = ctrl.update_longitudinal(control, control_sp, carstate)
    ctrl.frame += 1
    info = next((d for a, d, b in sends if a == 0x21b and b == 0), None)
    if info is None:
      continue
    out.append((t, decode_cmd(info), (info[5] >> 2) & 1, (info[6] >> 6) & 1, brake_hold))
  return out, ctrl


if __name__ == "__main__":
  twin = stock_twin()
  twin_bad = validate_latched(twin)
  print(f"stock twin (drive_04 latched release): {len(twin)} frames, violations: {twin_bad or 'NONE'}")

  out, ctrl = replay_115()
  # the faulting release: the segment's long hold window (abs t 1773.8..1800.55), + 3 s around
  win = [r for r in out if 1795.0 <= r[0] <= 1803.5]
  bad = validate_latched(win)
  print(f"route 115 release window: {len(win)} frames, latched={ctrl.stop_and_go.latched_release}, "
        f"violations: {bad or 'NONE'}")
  last = None
  for t, c, s, u, h in win:
    key = (s, u, h, c if u or not s else None)
    if key != last or u:
      print(f"  {t:9.3f}  cmd={c:+5d} stop={s} unl={u} hold={h}")
      last = key
