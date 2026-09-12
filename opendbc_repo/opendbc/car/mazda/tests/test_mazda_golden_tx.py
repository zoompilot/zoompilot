"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Golden transmit capture for the Mazda CarController.

One fixed scripted drive through the real CarController.update(): boot with the stock radar,
the FSC settle gate, the radar teardown, an engaged steer ramp against driver torque, the
highway rail, a body-latched stop with its release pulse, a never-latched stop with its
breakaway ramp, a gas override, the non-delivery latch, a cancel, ICBM taps and the hand-back.
Every tx frame (address, bus, bytes) and the reported actuator outputs are hashed per control
frame and compared against the checked-in fixture. Any change to what goes on the wire, in
any of those states, fails here with the phase and the frame that moved.

Regenerate the fixture only for a deliberate behaviour change:

  python opendbc/car/mazda/tests/test_mazda_golden_tx.py --update [--dump PATH]
"""
import hashlib
import json
import os
import sys

from opendbc.car import DT_CTRL
from opendbc.car.mazda.tests.conftest import (LongCtrlState, SendButtonState, VisualAlert, car_control, car_control_sp,
                                              car_controller, mazda_car_state, set_car_state, split_inputs)

FIXTURE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mazda_golden_tx.json")
HASH_CHARS = 12

# the driver-torque run from route 00000148 seg 10, the one that put every frame over the panda's ceiling
FIGHTING_DRIVER = [-25, -25, -25, -27, -28, -29, -30, -28, -26, -26, -29, -29, -31, -31, -31]


def _phase(name, n, base, per_frame=None):
  return name, n, base, per_frame


def _driver_fight(i, _):
  if i < 100:
    dt = 0.
  elif i < 120:
    dt = -20.
  elif i < 120 + len(FIGHTING_DRIVER):
    dt = FIGHTING_DRIVER[i - 120]
  else:
    dt = 0.
  return {"torque": min(i, 100) / 100, "driver_torque": dt, "steering_pressed": dt != 0.,
          "visual_alert": VisualAlert.steerRequired if 150 <= i < 200 else VisualAlert.none}


def _highway(i, _):
  return {"torque": 1.0 if i < 50 else -0.8, "driver_torque": 0. if i < 50 else 10.}


def _approach(i, _):
  return {"v_ego": max(5.0 - i * 0.05, 0.)}


def _body_answers_pulse(i, cc):
  # the body drops GEAR.BRAKE_HOLD two wire frames into the pulse, as in every capture
  sm = cc.stop_and_go
  if not hasattr(cc, "_golden_pulse_start"):
    cc._golden_pulse_start = None
  if sm.resume_unlatching and cc._golden_pulse_start is None:
    cc._golden_pulse_start = i
  held = cc._golden_pulse_start is None or i < cc._golden_pulse_start + 4
  return {"brake_hold": held}


def _drive_off(i, _):
  return {"v_ego": min(i * 0.03, 3.0)}


def _undelivered(i, _):
  return {"steer_undelivered": i < 20}


def _cancel(i, _):
  return {"brake_pressed": i < 30}


DISENGAGED = dict(enabled=False, long_active=False, lat_active=False, accel=0., long_state=LongCtrlState.off,
                  lead_visible=False, lead_d_rel=0., gap=0)
BOOT = dict(DISENGAGED, available=False, standstill=True, brake_pressed=True, stock_radar_alive=True,
            fsc_settled=False, radar_was_silenced=False)
SILENCED = dict(stock_radar_alive=False, stock_radar_gone=True, radar_was_silenced=True, fsc_settled=True)
ENGAGED = dict(SILENCED, enabled=True, long_active=True, lat_active=True, long_state=LongCtrlState.pid,
               available=True, cruise_engaged=True, gap=2)
LEAD_30 = dict(lead_visible=True, lead_d_rel=30.0, lead_v_rel=-1.0)
LEAD_4 = dict(lead_visible=True, lead_d_rel=4.0, lead_v_rel=0.0)
NO_LEAD = dict(lead_visible=False, lead_d_rel=0.0, lead_v_rel=0.0)

SCENARIO = [
  _phase("boot_stock_radar", 100, BOOT),
  _phase("fsc_settled_silencing", 120, dict(BOOT, fsc_settled=True)),
  _phase("radar_silenced_armed_idle", 100, dict(BOOT, **SILENCED, available=True), lambda i, _: {"brake_pressed": i < 50}),
  _phase("engage_steer_ramp", 220, dict(ENGAGED, **LEAD_30, v_ego=10.0, accel=1.0), _driver_fight),
  _phase("highway_rail", 170, dict(ENGAGED, **NO_LEAD, v_ego=20.0, accel=0.2), _highway),
  _phase("approach_stop", 100, dict(ENGAGED, lead_visible=True, lead_d_rel=6.0, lead_v_rel=-1.0,
                                     long_state=LongCtrlState.stopping, accel=-1.5, torque=0.1), _approach),
  _phase("hold_on_the_plan", 150, dict(ENGAGED, **LEAD_4, long_state=LongCtrlState.stopping, accel=-1.024,
                                        standstill=True, torque=0.1)),
  _phase("hold_body_latched", 100, dict(ENGAGED, **LEAD_4, long_state=LongCtrlState.stopping, accel=-1.024,
                                         standstill=True, brake_hold=True, torque=0.1)),
  _phase("latched_release", 120, dict(ENGAGED, **LEAD_4, accel=1.0, standstill=True, torque=0.1), _body_answers_pulse),
  _phase("drive_off", 100, dict(ENGAGED, **LEAD_4, accel=0.6, torque=0.1), _drive_off),
  _phase("second_approach", 50, dict(ENGAGED, **LEAD_4, long_state=LongCtrlState.stopping, accel=-1.0, v_ego=1.0)),
  _phase("hold_never_latched", 100, dict(ENGAGED, **LEAD_4, long_state=LongCtrlState.stopping, accel=-1.024, standstill=True)),
  _phase("never_latched_release_breakaway", 100, dict(ENGAGED, **LEAD_4, accel=0.45, standstill=True)),
  _phase("moving_again", 50, dict(ENGAGED, **LEAD_4, accel=0.45, v_ego=1.5)),
  _phase("gas_override", 60, dict(ENGAGED, **LEAD_30, long_active=False, long_state=LongCtrlState.off, accel=0., gas=True,
                                   v_ego=5.0, torque=0.2)),
  _phase("steer_undelivered_latch", 40, dict(ENGAGED, **LEAD_30, accel=0.3, v_ego=3.0, torque=0.8), _undelivered),
  _phase("cancel", 60, dict(DISENGAGED, **SILENCED, available=True, cruise_engaged=True, cancel=True, v_ego=5.0), _cancel),
  _phase("icbm_taps", 40, dict(ENGAGED, **LEAD_30, accel=0.3, v_ego=15.0, send_button=SendButtonState.increase)),
  _phase("handback", 100, dict(DISENGAGED, **SILENCED, available=True, standstill=True, handback=True)),
  _phase("stock_radar_back", 60, dict(DISENGAGED, available=False, standstill=True, handback=True,
                                       stock_radar_alive=True, radar_was_silenced=True)),
]


def run_scenario():
  """Drive a fresh alpha-long controller through SCENARIO. Yields one record per control frame."""
  cc = car_controller(alpha_long=True)
  cs = mazda_car_state(cc.CP, cc.CP_SP)
  frame = 0
  for name, n, base, per_frame in SCENARIO:
    for i in range(n):
      kwargs = dict(base)
      if per_frame is not None:
        kwargs.update(per_frame(i, cc))
      cc_kw, cc_sp_kw, cs_kw = split_inputs(kwargs)
      set_car_state(cs, **cs_kw)
      actuators, sends = cc.update(car_control(**cc_kw), car_control_sp(**cc_sp_kw), cs, int(frame * DT_CTRL * 1e9))
      yield {
        "frame": frame,
        "phase": name,
        "tx": [[addr, bus, dat.hex()] for addr, dat, bus in sends],
        "torque_can": int(actuators.torqueOutputCan),
        "accel": round(float(actuators.accel), 4),
      }
      frame += 1


def frame_hash(rec) -> str:
  payload = json.dumps([rec["tx"], rec["torque_can"], rec["accel"]], separators=(",", ":"))
  return hashlib.sha256(payload.encode()).hexdigest()[:HASH_CHARS]


def build_fixture(records):
  phases = []
  keyframes = {}
  for rec in records:
    if not phases or phases[-1]["name"] != rec["phase"]:
      phases.append({"name": rec["phase"], "start": rec["frame"]})
      keyframes[str(rec["frame"])] = {"tx": rec["tx"], "torque_can": rec["torque_can"], "accel": rec["accel"]}
  return {
    "frames": len(records),
    "phases": phases,
    "hashes": [frame_hash(r) for r in records],
    "keyframes": keyframes,
  }


def load_fixture():
  with open(FIXTURE) as f:
    return json.load(f)


def test_scenario_shape():
  fixture = load_fixture()
  assert [p["name"] for p in fixture["phases"]] == [name for name, *_ in SCENARIO]
  assert fixture["frames"] == sum(n for _, n, *_ in SCENARIO) == len(fixture["hashes"])


def test_golden_tx_matches():
  fixture = load_fixture()
  records = list(run_scenario())
  assert len(records) == fixture["frames"]
  for rec, expected in zip(records, fixture["hashes"], strict=True):
    if frame_hash(rec) != expected:
      key = fixture["keyframes"].get(str(rec["frame"]))
      golden = f"\ngolden keyframe: {json.dumps(key)}" if key else ""
      now = f"now: tx={rec['tx']} torque_can={rec['torque_can']} accel={rec['accel']}"
      raise AssertionError(f"tx diverged at frame {rec['frame']} ({rec['phase']}):\n{now}{golden}")


def test_keyframes_match_byte_for_byte():
  # the first frame of every phase is stored in full, so a divergence there reads as a diff
  fixture = load_fixture()
  by_frame = {r["frame"]: r for r in run_scenario()}
  for frame, key in fixture["keyframes"].items():
    rec = by_frame[int(frame)]
    assert rec["tx"] == key["tx"], f"frame {frame} ({rec['phase']})"
    assert rec["torque_can"] == key["torque_can"], f"frame {frame} ({rec['phase']})"
    assert rec["accel"] == key["accel"], f"frame {frame} ({rec['phase']})"


def test_scenario_reaches_every_state():
  # the scenario is only a regression net if it actually visits the states it names
  by_phase = {}
  for rec in run_scenario():
    by_phase.setdefault(rec["phase"], []).append(rec)

  def tx_addrs(phase):
    return {addr for r in by_phase[phase] for addr, _, _ in r["tx"]}

  def crz_info_unlatching(phase):
    return any(int(dat[12:14], 16) & 0x40 for r in by_phase[phase] for addr, bus, dat in r["tx"] if addr == 0x21b and bus == 0)

  assert tx_addrs("boot_stock_radar") == {0x243, 0x440}
  assert 0x764 in tx_addrs("fsc_settled_silencing") and 0x21b not in tx_addrs("fsc_settled_silencing")
  assert {0x21b, 0x21c, 0x499, 0x364} <= tx_addrs("radar_silenced_armed_idle")
  assert max(r["torque_can"] for r in by_phase["engage_steer_ramp"]) > 1000
  # the 10 m/s command winds down at STEER_DELTA_DOWN a frame and settles on the 620 rail
  assert by_phase["highway_rail"][49]["torque_can"] == 620
  assert min(r["torque_can"] for r in by_phase["highway_rail"]) == -620
  assert {r["accel"] for r in by_phase["hold_on_the_plan"][-50:]} == {-1.024}
  assert {r["accel"] for r in by_phase["hold_body_latched"][-10:]} == {-0.001}
  assert crz_info_unlatching("latched_release")
  assert not crz_info_unlatching("never_latched_release_breakaway")
  assert max(r["accel"] for r in by_phase["never_latched_release_breakaway"]) > 0.45
  assert {r["accel"] for r in by_phase["gas_override"]} == {0.}
  assert by_phase["steer_undelivered_latch"][0]["torque_can"] == 0
  assert 0x9d in tx_addrs("cancel")
  assert 0x9d in tx_addrs("icbm_taps")
  assert tx_addrs("stock_radar_back") == {0x243, 0x440}


if __name__ == "__main__":
  if "--update" not in sys.argv:
    print(__doc__)
    sys.exit(1)
  recs = list(run_scenario())
  with open(FIXTURE, "w") as f:
    json.dump(build_fixture(recs), f, separators=(",", ":"))
  print(f"wrote {FIXTURE}: {len(recs)} frames, {os.path.getsize(FIXTURE)} bytes")
  if "--dump" in sys.argv:
    dump = sys.argv[sys.argv.index("--dump") + 1]
    with open(dump, "w") as f:
      json.dump(recs, f)
    print(f"wrote {dump}: {os.path.getsize(dump)} bytes")
