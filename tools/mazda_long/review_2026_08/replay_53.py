"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Replay route 53 seg 11 (the 2026-08-27 SCBS latch: never-latched release, real departing
lead, command slewing off the hold value under a 13-frame pulse) through the CarController
and validate every emitted CRZ_INFO frame against the stock release grammar.

The validator encodes the 33-pulse stock census:
  - STOPPING and RESUME_UNLATCHING never co-occur
  - a never-latched pulse frame carries a command in the -0.28..-0.10 band, never deeper,
    never positive; a latched pulse frame carries -0.016..+0.35
  - never-latched pulses run <= 3 wire frames and start 2-4 wire frames after the drop;
    latched pulses run <= 11 wire frames
  - after a never-latched drop the command never returns to hold depth (< -300) while stopped
It is sanity-checked against the stock twin (drive_0b t=17487-17491) before judging us.
"""
import sys, os, glob
sys.path.insert(0, "/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot")
sys.path.insert(0, "/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot/opendbc_repo")
sys.path.append("/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot/tools/mazda_long")
os.chdir("/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot")
from openpilot.tools.lib.logreader import LogReader
from replay_standstill_hold import build_controller, decode_cmd, mock_inputs


def validate(rows, latched):
  """rows: [(t, cmd_raw, stop, unl)] at the wire rate. Returns list of violations."""
  bad = []
  for t, cmd, stop, unl in rows:
    if stop and unl:
      bad.append((t, f"STOPPING with RESUME_UNLATCHING (cmd {cmd:+d})"))
    if unl:
      lo, hi = (-16, 350) if latched else (-280, -100)
      if not lo <= cmd <= hi:
        bad.append((t, f"pulse cmd {cmd:+d} outside the {'latched' if latched else 'never-latched'} band [{lo}, {hi}]"))
  # pulse runs
  runs, cur = [], None
  drop_t = None
  prev_stop = None
  for t, cmd, stop, unl in rows:
    if prev_stop and not stop:
      drop_t = t
    prev_stop = stop
    if unl and cur is None:
      cur = [t, 0]
    if cur is not None:
      if unl:
        cur[1] += 1
      else:
        runs.append(tuple(cur)); cur = None
  for t0, n in runs:
    cap = 11 if latched else 3
    if n > cap:
      bad.append((t0, f"pulse ran {n} wire frames (stock max {cap})"))
    if not latched and drop_t is not None:
      gap = round((t0 - drop_t) / 0.02)
      if not 2 <= gap <= 4:
        bad.append((t0, f"pulse started {gap} wire frames after the drop (stock: 3)"))
  if not latched and drop_t is not None:
    deep = [(t, c) for t, c, stop, _ in rows if t > drop_t + 0.021 and not stop and c < -300 and t < drop_t + 1.0]
    if deep:
      bad.append((deep[0][0], f"cmd back at hold depth after the drop: {deep[0][1]:+d}"))
  return bad


def stock_twin():
  """The same tuples off the stock capture (drive_0b non-latched release) for validator sanity."""
  rows = []
  paths = sorted(glob.glob("tools/mazda_long/test_data/drive_0b/*/rlog*") +
                 glob.glob("tools/mazda_long/test_data/drive_0b/rlog*") +
                 glob.glob("tools/mazda_long/test_data/rlog_0b_seg*.zst"))
  for p in paths:
    for m in LogReader(p):
      if m.which() != "can":
        continue
      t = m.logMonoTime * 1e-9
      if not (17487.0 <= t <= 17491.5):
        continue
      for c in m.can:
        if c.address == 0x21b and c.src == 0:
          dat = bytes(c.dat)
          rows.append((t, decode_cmd(dat), (dat[5] >> 2) & 1, (dat[6] >> 6) & 1))
  return rows


def replay_53():
  rows = []
  cc = cs = lead = None
  brake_hold = False
  for m in LogReader("tools/mazda_long/test_data/route_53/rlog_seg11.zst"):
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
  t0 = rows[0][0]
  out = []
  for t, cc, cs, lead, brake_hold in rows:
    control, control_sp, carstate = mock_inputs(cc, cs, brake_hold, lead)
    sends = ctrl.update_longitudinal(control, control_sp, carstate)
    ctrl.frame += 1
    info = next((d for a, d, b in sends if a == 0x21b and b == 0), None)
    if info is None:
      continue
    out.append((t - t0, decode_cmd(info), (info[5] >> 2) & 1, (info[6] >> 6) & 1))
  return out, ctrl


if __name__ == "__main__":
  twin = stock_twin()
  twin_bad = validate(twin, latched=False)
  print(f"stock twin (drive_0b): {len(twin)} frames, violations: {twin_bad or 'NONE'}")

  out, ctrl = replay_53()
  # the faulting stop: the segment's one stop-bit window, plus 4 s either side
  stops = [t for t, _, s, _ in out if s]
  win = [(t, c, s, u) for t, c, s, u in out if stops[0] - 4.0 <= t <= stops[-1] + 4.0]
  bad = validate(win, latched=ctrl.stop_and_go.latched_release)
  print(f"route 53 release window: {len(win)} frames, latched={ctrl.stop_and_go.latched_release}, "
        f"violations: {bad or 'NONE'}")
  last = None
  for t, c, s, u in win:
    key = (s, u, c if u or s == 0 else None)
    if key != last or u:
      print(f"  {t:7.2f}  cmd={c:+5d} stop={s} unl={u}")
      last = key
