#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Replay recorded stop-and-go episodes through the real CarController.

Feeds each logged frame's plan output and car state into update_longitudinal, with the body hold
decoded from the log's own 0x79/0x228 frames by the rule carstate uses, and checks the CRZ_INFO
that would go on the wire against stock's standstill grammar:

  - stopped, plan braking, body not holding: STOPPING up and the command still braking
  - body holding: STOPPING down and the command relaxed to raw -1, as stock does 0-40 ms after
    EPB.HOLD_STATE goes HOLDING (43/43 stock holds, acc_hold_census.py)

Open loop: the body's trace is what it did on that drive, so a release it answered to someone
else's pulse shows up here as the body letting go early, and the controller must brake again.

The inputs go through the opendbc test rig (conftest) onto a real CarState, so the replay tracks
the controller instead of mocking it. The review_2026_08 replays share these helpers.

Usage: .venv/bin/python3 tools/mazda_long/replay_standstill_hold.py <rlog> [<rlog> ...]
"""
import functools
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "opendbc_repo"))

from opendbc.can import CANParser
from opendbc.car.mazda.carstate import body_holds
from opendbc.car.mazda.tests.conftest import CRZ_INFO, car_control, car_control_sp, car_controller, car_params, \
  car_params_sp, crz_info, frame, mazda_car_state, set_car_state
from openpilot.tools.lib.logreader import LogReader

EPB, GEAR = 0x79, 0x228

# The alpha-long CX-5 2022 controller the test rig builds.
build_controller = car_controller


def decode_cmd(dat):
  return crz_info(dat)[0]


@functools.cache
def _rig_car_state():
  CP = car_params(alpha_long=True)
  return mazda_car_state(CP, car_params_sp(CP, alpha_long=True))


def frames(path):
  """Logged (t, carControl, carState, body hold) at the carControl rate."""
  cp = CANParser("mazda_2017", [(EPB, float("nan")), (GEAR, float("nan"))], 0)
  body_hold = False
  cs = None
  out = []
  for m in LogReader(path):
    w = m.which()
    if w == "can":
      cp.update([(m.logMonoTime, [(c.address, bytes(c.dat), 0) for c in m.can if c.src == 0 and c.address in (EPB, GEAR)])])
      body_hold = body_holds(cp.vl)
    elif w == "carState":
      cs = m.carState
    elif w == "carControl" and cs is not None:
      out.append((m.logMonoTime * 1e-9, m.carControl, cs, body_hold))
  return out


def mock_inputs(cc, cs, body_hold, lead=None):
  """One logged (carControl, carState) frame as update_longitudinal's inputs, on a real CarState.
  lead is the (dRel, vRel) for CC_SP.leadOne, if the replay carries one."""
  carstate = set_car_state(_rig_car_state(), body_hold=body_hold, standstill=cs.standstill, gas=cs.gasPressed,
                           brake_pressed=cs.brakePressed, v_ego=cs.vEgo, available=cs.cruiseState.available,
                           cruise_engaged=cs.cruiseState.enabled)
  act = cc.actuators
  control = car_control(enabled=cc.enabled, long_active=cc.longActive, accel=act.accel, long_state=act.longControlState,
                        resume=cc.cruiseControl.resume, lead_visible=cc.hudControl.leadVisible,
                        gap=cc.hudControl.leadDistanceBars)
  control_sp = car_control_sp(lead_d_rel=lead[0] if lead else 0.0, lead_v_rel=lead[1] if lead else 0.0)
  return control, control_sp, carstate


def replay(path):
  ctrl = build_controller()
  t0 = None
  stopped = held = 0
  bad = []
  for t, cc, cs, body_hold in frames(path):
    t0 = t if t0 is None else t0
    sends = ctrl.update_longitudinal(*mock_inputs(cc, cs, body_hold))
    ctrl.frame += 1
    dat = frame(sends, CRZ_INFO)
    if dat is None or not cs.standstill or not cc.longActive:
      continue
    cmd, stop, _ = crz_info(dat)
    stopped += 1
    held += body_hold
    if ctrl.stop_and_go.holding:
      if body_hold and (stop or cmd != -1):
        bad.append((t - t0, f"body holds but we send stop={stop:d} cmd={cmd:+d}"))
      elif not body_hold and cc.actuators.accel < -0.1 and (not stop or cmd > -100):
        bad.append((t - t0, f"plan {cc.actuators.accel:+.2f}, body not holding, but we send stop={stop:d} cmd={cmd:+d}"))

  print(f"\n{os.path.relpath(path)}: {stopped} engaged frames stopped, {held} with the body holding")
  for t, why in bad[:10]:
    print(f"  VIOLATION t+{t:.2f}: {why}")
  if not bad:
    print("  OK: braked until the body held, relaxed while it did")
  return not bad


if __name__ == "__main__":
  results = [replay(p) for p in sys.argv[1:]]  # every log reports, not just up to the first failure
  sys.exit(0 if all(results) else 1)
