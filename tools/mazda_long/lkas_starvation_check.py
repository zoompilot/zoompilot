#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Did the panda drop our LKAS frames, and if so, why?

Every "LKAS Fault: Restart the Car" this port has produced came from the EPS being starved
of CAM_LKAS 0x243. The camera's own copy is relay-blocked while openpilot is controlling, so
a frame the panda refuses to transmit is a frame the EPS never receives from anyone; hold
that up for long enough and the EPS drops out of LKAS entirely, comes back with LKAS_BLOCK
set and LKAS_EFFECTIVE at zero, and the controller ramps into an EPS that is not listening.
Route 00000148 lost 1721 ms that way.

Rejected frames are the giveaway and they are already in every rlog: pandad marks a refused
transmission by adding CAN_REJECTED_BUS_OFFSET to the bus, so our 0x243 shows up as src 192
instead of the usual src 128 (selfdrive/pandad/panda.h). Nothing else in the log says this,
which is why the same failure was diagnosed three times from its downstream symptoms before
anyone looked here.

Two causes are known and they need different fixes, so the tool separates them by the panda's
own controlsAllowedLateral at the moment of the burst:

  lateral NOT allowed -> the two MADS state machines disagree about whether lateral is armed.
    Software MADS engaged, the panda never saw a matching edge (routes 00000116/00000117).
    Fixed by mirroring the radar-silence latch into mazda.h, so a recurrence means that
    latch, not the torque envelope.

  lateral allowed, first rejected frame retreats by exactly STEER_DELTA_DOWN -> the panda's
    max_rate_down was larger than the controller's retreat. Once the driver bound falls below
    the last command, driver_limit_check demands a retreat of at least max_rate_down per frame;
    a smaller one is rejected, the panda resets its last command to zero, and every following
    frame is rejected until |cmd| <= max_rate_up. Route 00000148 seg 10: 1076 -> 1064 against a
    required 1051, then 170 more. Fixed by MazdaSafetyFlags.STEER_TO_ZERO_EPS selecting a
    12/12 envelope in mazda.h, equal to the controller's.

  lateral allowed, any other first frame -> the command was riding the driver-torque ceiling
    and the panda, which computes that ceiling from the min/max of its own last 6
    STEER_TORQUE samples rather than from the one stale sample the controller holds, put it
    over the line. Fixed by STEER_DRIVER_SAMPLES / STEER_DRIVER_MARGIN in values.py.

A fourth pattern is none of these, and that is the point of keeping this around.

Usage:  lkas_starvation_check.py <route-dir> [<route-dir> ...]
        lkas_starvation_check.py tools/mazda_long/device_data/00000148--e00a5dce42--10
"""
import glob
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from openpilot.tools.lib.logreader import LogReader

LKAS_ADDR = 0x243
TX_DELIVERED = 128  # CAN_RETURNED_BUS_OFFSET | bus 0
TX_REJECTED = 192   # CAN_REJECTED_BUS_OFFSET | bus 0
STEER_DELTA_DOWN = 12  # the 2022+ EPS controller winddown (opendbc mazda/values.py)

# A burst only matters once the EPS notices. It re-asserted LKAS_BLOCK 0.6 s into route 148's
# hole, so anything at or past that is long enough to have cost real steering; shorter runs
# are reported but not counted against the drive.
STARVING_T = 0.5
IGNORED_SAFETY = ("silent", "noOutput")


def segments(route_dir):
  segs = sorted(glob.glob(os.path.join(route_dir, "rlog_seg*.zst")),
                key=lambda p: int(p.split("seg")[1].split(".")[0]))
  if segs:
    return segs
  nested = sorted(glob.glob(os.path.join(route_dir, "*", "rlog*")))
  return nested or sorted(glob.glob(os.path.join(route_dir, "rlog*")))


def scan(route_dir):
  """Return (delivered, rejected, gaps) where a gap is a run with nothing delivered."""
  delivered = rejected = 0
  gaps, t0, last_ok = [], None, None
  lat_allowed = None
  driver_torque = 0.0
  last_cmd = 0
  # open gap: [start, end, rejected_frames, lateral_allowed_seen, worst driver torque, peak cmd,
  #            first rejected cmd minus the last delivered one]
  cur = None

  for p in segments(route_dir):
    for m in LogReader(p):
      w = m.which()
      if w == "carState":
        driver_torque = m.carState.steeringTorque
        continue
      if w == "pandaStates":
        for ps in m.pandaStates:
          if str(ps.safetyModel) not in IGNORED_SAFETY:
            lat_allowed = bool(ps.controlsAllowedLateral)
        continue
      if w != "can":
        continue
      t = m.logMonoTime * 1e-9
      if t0 is None:
        t0 = t
      tr = t - t0
      for c in m.can:
        if c.address != LKAS_ADDR or c.src not in (TX_DELIVERED, TX_REJECTED):
          continue
        cmd = (((c.dat[0] & 0x0F) << 8) | c.dat[1]) - 2048
        if c.src == TX_REJECTED:
          rejected += 1
          if cur is None:
            cur = [tr, tr, 0, lat_allowed, driver_torque, cmd, cmd - last_cmd]
          cur[1] = tr
          cur[2] += 1
          # keep whichever driver torque and command best explain the ceiling being hit
          if abs(driver_torque) > abs(cur[4]):
            cur[4] = driver_torque
          if abs(cmd) > abs(cur[5]):
            cur[5] = cmd
          if cur[3] is None:
            cur[3] = lat_allowed
        else:
          delivered += 1
          last_cmd = cmd
          # a lone delivered frame between rejects does not feed the EPS; only close the gap
          # once delivery actually resumes
          if cur is not None and last_ok is not None and tr - last_ok < 0.05:
            gaps.append(cur)
            cur = None
          last_ok = tr
  if cur is not None:
    gaps.append(cur)
  return delivered, rejected, gaps


def main(dirs):
  bad = 0
  for route_dir in dirs:
    delivered, rejected, gaps = scan(route_dir)
    name = os.path.basename(route_dir.rstrip("/"))
    print(f"\n{name}")
    if not delivered and not rejected:
      print("  no openpilot LKAS frames in this route")
      continue
    pct = 100.0 * rejected / (delivered + rejected)
    print(f"  0x243 transmitted: {delivered} delivered, {rejected} rejected ({pct:.2f}%)")
    if not gaps:
      print("  CLEAN - the EPS heard every frame we sent")
      continue
    starving = [g for g in gaps if (g[1] - g[0]) >= STARVING_T]
    for start, end, n, lat, dtq, cmd, step in sorted(gaps, key=lambda g: g[0] - g[1])[:5]:
      dur = end - start
      if lat is None:
        cause = "no panda state in the log; cannot attribute"
      elif lat and abs(step) == STEER_DELTA_DOWN and cmd * step < 0:
        cause = (f"lateral WAS allowed, first frame retreated by {abs(step)} -> panda max_rate_down "
                 + f"above the controller's winddown (peak driver torque {dtq:.0f} against a command "
                 + f"of {cmd}); see MazdaSafetyFlags.STEER_TO_ZERO_EPS")
      elif lat:
        cause = (f"lateral WAS allowed -> driver-torque envelope (peak driver torque {dtq:.0f} "
                 + f"against a command of {cmd}); see STEER_DRIVER_SAMPLES")
      else:
        cause = "lateral NOT allowed -> MADS/panda arming desync; see MAZDA_RADAR_SILENT_FRAMES"
      flag = "STARVED" if dur >= STARVING_T else "brief  "
      print(f"  {flag} t+{start:7.2f} for {dur * 1000:6.0f} ms, {n} frames rejected")
      print(f"          {cause}")
    if len(gaps) > 5:
      print(f"  ... and {len(gaps) - 5} shorter bursts")
    if starving:
      bad += 1
      print(f"  {len(starving)} burst(s) long enough to drop the EPS out of LKAS")
  return 1 if bad else 0


if __name__ == "__main__":
  if len(sys.argv) < 2:
    print(__doc__)
    raise SystemExit(2)
  raise SystemExit(main(sys.argv[1:]))
