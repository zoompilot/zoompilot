#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Replay recorded stop-and-go episodes through the real CarController.

Feeds each logged frame's plan output and car state into update_longitudinal and reports the
CRZ_INFO command that would go on the wire, so a hold regression shows up as the command
leaving the plan's brake while the car is still stopped.

Usage: .venv/bin/python3 tools/mazda_long/replay_standstill_hold.py <rlog...>
"""
import os
import sys
from types import SimpleNamespace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR
from openpilot.tools.lib.logreader import LogReader

LongCtrlState = structs.CarControl.Actuators.LongControlState


def build_controller():
  fp = {0: {}, 1: {}, 2: {}}
  CP = CarInterface.get_params(CAR.MAZDA_CX5_2022, fp, [], alpha_long=True, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.MAZDA_CX5_2022, fp, [], True, False, False)
  return CarController({Bus.pt: "mazda_2017"}, CP, CP_SP)


def frames(path):
  """Logged (t, plan, carstate) at the carControl rate, with GEAR.BRAKE_HOLD from raw CAN."""
  cp = CANParser("mazda_2017", [("GEAR", float("nan"))], 0)
  brake_hold = False
  cs = None
  out = []
  for m in LogReader(path):
    t = m.logMonoTime * 1e-9
    w = m.which()
    if w == "can":
      cp.update([(m.logMonoTime, [(c.address, bytes(c.dat), c.src) for c in m.can])])
      brake_hold = cp.vl["GEAR"]["BRAKE_HOLD"] == 1
    elif w == "carState":
      cs = m.carState
    elif w == "carControl" and cs is not None:
      cc = m.carControl
      out.append((t, cc, cs, brake_hold))
  return out


def decode_cmd(dat):
  return (((dat[2] & 0x3) << 11) | (dat[3] << 3) | (dat[4] >> 5)) - 4096


def mock_inputs(cc, cs, brake_hold, lead=None):
  """One logged (carControl, carState) frame as the inputs update_longitudinal reads;
  lead is the (dRel, vRel) for CC_SP.leadOne, if the replay carries one."""
  out = SimpleNamespace(standstill=cs.standstill, gasPressed=cs.gasPressed, brakePressed=cs.brakePressed,
                        cruiseState=SimpleNamespace(available=cs.cruiseState.available,
                                                    enabled=cs.cruiseState.enabled))
  actuators = SimpleNamespace(accel=cc.actuators.accel, longControlState=cc.actuators.longControlState)
  control = SimpleNamespace(enabled=cc.enabled, longActive=cc.longActive, actuators=actuators,
                            cruiseControl=SimpleNamespace(resume=cc.cruiseControl.resume,
                                                          override=cc.cruiseControl.override, cancel=False),
                            hudControl=SimpleNamespace(leadVisible=cc.hudControl.leadVisible,
                                                       leadDistanceBars=cc.hudControl.leadDistanceBars))
  control_sp = SimpleNamespace(stockEcuHandBack=False,
                               leadOne=SimpleNamespace(dRel=lead[0] if lead else 0.0,
                                                       vRel=lead[1] if lead else 0.0))
  carstate = SimpleNamespace(out=out, resume_button=0, brake_hold=brake_hold,
                             stock_radar_alive=False, stock_radar_gone=True, fsc_settled=True, radar_session_refused=False)
  return control, control_sp, carstate


def replay(path):
  cc_ctrl = build_controller()
  t0 = None
  worst = None
  rows = []
  for t, cc, cs, brake_hold in frames(path):
    if t0 is None:
      t0 = t
    control, control_sp, carstate = mock_inputs(cc, cs, brake_hold)
    sends = cc_ctrl.update_longitudinal(control, control_sp, carstate)
    cc_ctrl.frame += 1
    dat = next((d for a, d, b in sends if a == 0x21b and b == 0), None)
    if dat is None:
      continue
    cmd = decode_cmd(dat)
    rows.append((t - t0, cs.standstill, cc.actuators.accel, cmd, brake_hold))
    # a hold regression: still stopped, plan still braking, but our command let go
    if cs.standstill and cc.longActive and cc.actuators.accel < -0.1 and cmd > -100 and not brake_hold:
      if worst is None:
        worst = (t - t0, cc.actuators.accel, cmd)

  held = [r for r in rows if r[1]]
  print(f"\n{os.path.basename(os.path.dirname(path))}: {len(rows)} frames, {len(held)} at standstill")
  if held:
    cmds = sorted({r[3] for r in held})
    print(f"  command while stopped: {cmds[:6]}{' ...' if len(cmds) > 6 else ''}")
    print(f"  plan while stopped:    min={min(r[2] for r in held):+.3f} max={max(r[2] for r in held):+.3f}")
    print(f"  car ever took the hold: {any(r[4] for r in held)}")
  if worst:
    print(f"  REGRESSION at {worst[0]:.2f}s: plan {worst[1]:+.3f} but we sent raw {worst[2]}")
  else:
    print("  OK: never released the brakes while stopped with the plan still braking")
  return worst is None


if __name__ == "__main__":
  ok = all(replay(p) for p in sys.argv[1:])
  sys.exit(0 if ok else 1)
