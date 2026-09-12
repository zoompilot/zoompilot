#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Replay a recorded Mazda alpha-long route's CAN through the current carstate + radar session
manager, disengaged, and print when the session manager would have requested the radar takeover
and what the stock ECU status said meanwhile. Run once as the recorded configuration (parked
takeover only) and once with the moving takeover capability, side by side.

This tests the software's decisions on recorded inputs only: it cannot say how the radar answers
a programming-session request at speed, which only the car can (docs/zoompilot/
force-offroad-alpha-transition.md, validation section).

Usage:
  PYTHONPATH=. .venv/bin/python tools/mazda_long/offroad_transition/replay_moving_takeover.py <route dir>... [--to SECONDS]
"""
import argparse
import sys
from pathlib import Path

from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.tests.conftest import car_control, car_control_sp, LongCtrlState
from opendbc.car.mazda.values import CAR, MazdaFlags
from openpilot.tools.lib.logreader import LogReader


def build_interface(cp_log, moving_takeover: bool) -> CarInterface:
  fw = list(cp_log.carFw)  # structs.CarParams is the capnp CarParams; _get_params reads ecu and fwVersion only
  candidate = CAR(cp_log.carFingerprint)
  fingerprint = {b: {} for b in range(4)}
  CP = CarInterface.get_params(candidate, fingerprint, fw, alpha_long=True, is_release=False, docs=False)
  if moving_takeover:
    CP.flags |= MazdaFlags.MOVING_TAKEOVER.value
  else:
    CP.flags &= ~MazdaFlags.MOVING_TAKEOVER.value
  CP_SP = CarInterface.get_params_sp(CP, candidate, fingerprint, fw, alpha_long=True, is_release_sp=False, docs=False)
  return CarInterface(CP, CP_SP)


def replay(route_dirs: list[Path], moving_takeover: bool, to_s: float | None):
  lr = LogReader([str(d / "rlog.zst") for d in route_dirs], sort_by_time=True)
  cp_log = None
  CI = None
  t0 = None
  cc = car_control(long_active=False, accel=0., long_state=LongCtrlState.off, lead_visible=False)
  cc_sp = car_control_sp()
  last_state = None
  last_status = None
  events = []
  frames = 0
  for m in lr:
    if t0 is None:
      t0 = m.logMonoTime
    t = (m.logMonoTime - t0) * 1e-9
    if to_s is not None and t > to_s:
      break
    if CI is None:
      if m.which() == "carParams":
        cp_log = m.carParams
        CI = build_interface(cp_log, moving_takeover)
      continue
    if m.which() != "can":
      continue
    can = [(c.address, bytes(c.dat), c.src) for c in m.can if c.src in (0, 2)]
    if not can:
      continue
    CS, _ = CI.update([(m.logMonoTime, can)])
    CI.CC.update_longitudinal(cc, cc_sp, CI.CS)
    CI.CC.frame += 1
    frames += 1
    mgr = CI.CC.radar_session
    status = CI.CC.stock_ecu_state
    if mgr.state != last_state:
      events.append((t, f"session {last_state} -> {mgr.state}  v={CS.vEgo * 3.6:5.1f} km/h  fsc={CI.CS.fsc_settled} " +
                        f"stockCruise={CI.CS.cruise_enabled} radarAlive={CI.CS.stock_radar_alive} programmingSent={mgr.programming_sent}"))
      last_state = mgr.state
    if status != last_status:
      events.append((t, f"status {status}  v={CS.vEgo * 3.6:5.1f} km/h"))
      last_status = status
    if mgr.diagnostic_message is not None and mgr.diagnostic_message.dat[1] == 0x10:
      events.append((t, f"TX 0x764 {mgr.diagnostic_message.dat[:3].hex()}  v={CS.vEgo * 3.6:5.1f} km/h"))
  print(f"  frames={frames} fingerprint={cp_log.carFingerprint if cp_log else None} " +
        f"movingTakeover={bool(CI.CP.flags & MazdaFlags.MOVING_TAKEOVER) if CI else None}")
  for t, text in events:
    print(f"  {t:8.3f}  {text}")


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("routes", nargs="+")
  ap.add_argument("--to", type=float, default=None)
  args = ap.parse_args()
  dirs = [Path(r) for r in args.routes]
  for moving in (False, True):
    print(f"== {'moving takeover capable' if moving else 'recorded configuration (parked takeover only)'}")
    replay(dirs, moving, args.to)
  return 0


if __name__ == "__main__":
  sys.exit(main())
