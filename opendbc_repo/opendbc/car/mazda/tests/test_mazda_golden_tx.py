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

That scenario seeds CarState attributes directly. The full-stack scenarios below feed synthetic
CAN through CarInterface.update and apply, and every frame through the panda's safety model, so
carstate, the interface and mazda.h are pinned too, on each configuration the port treats
differently: alpha long, stock long, the legacy-firmware EPS, the G46L radar, the TJA button
with the MADS white wheel.

Regenerate the fixtures only for a deliberate behavior change:

  python opendbc/car/mazda/tests/test_mazda_golden_tx.py --update [--dump PATH]
"""
import functools
import hashlib
import json
import os
import sys

import pytest

from opendbc.can import CANPacker
from opendbc.car import DT_CTRL
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.radar_session import RADAR_UDS_STEP
from opendbc.car.mazda.tests.conftest import (DBC_NAME, SESSION_DFLT_DAT, SESSION_PROG_DAT, TESTER_PRESENT_DAT, LongCtrlState,
                                              SendButtonState, VisualAlert, car_control, car_control_sp, car_controller, car_params,
                                              car_params_sp, eps_fw, mazda_car_state, radar_fw, set_car_state, split_inputs)
from opendbc.car.mazda.values import CAR, G46L_RADAR_FW
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.sunnypilot.car.mazda.values import MazdaFlagsSP, MazdaSafetyFlagsSP

FIXTURE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mazda_golden_tx.json")
STACK_FIXTURE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "mazda_golden_stack.json")
HASH_CHARS = 12
STACK_HASH_CHARS = 10

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
  # the body lets go two wire frames into the pulse, as in every capture
  sm = cc.stop_and_go
  if not hasattr(cc, "_golden_pulse_start"):
    cc._golden_pulse_start = None
  if sm.resume_unlatching and cc._golden_pulse_start is None:
    cc._golden_pulse_start = i
  held = cc._golden_pulse_start is None or i < cc._golden_pulse_start + 4
  return {"body_hold": held}


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
  _phase("radar_silenced_armed_idle", 100, dict(BOOT, **SILENCED, available=True),
         lambda i, _: {"brake_pressed": i < 50, "hbc_request": 25 <= i < 75}),
  _phase("engage_steer_ramp", 220, dict(ENGAGED, **LEAD_30, v_ego=10.0, accel=1.0), _driver_fight),
  _phase("highway_rail", 190, dict(ENGAGED, **NO_LEAD, v_ego=20.0, accel=0.2, hbc_request=True), _highway),
  _phase("approach_stop", 100, dict(ENGAGED, lead_visible=True, lead_d_rel=6.0, lead_v_rel=-1.0,
                                     long_state=LongCtrlState.stopping, accel=-1.5, torque=0.1), _approach),
  _phase("hold_on_the_plan", 150, dict(ENGAGED, **LEAD_4, long_state=LongCtrlState.stopping, accel=-1.0,
                                        standstill=True, torque=0.1)),
  _phase("hold_body_latched", 100, dict(ENGAGED, **LEAD_4, long_state=LongCtrlState.stopping, accel=-1.0,
                                         standstill=True, body_hold=True, torque=0.1)),
  _phase("latched_release", 120, dict(ENGAGED, **LEAD_4, accel=1.0, standstill=True, torque=0.1), _body_answers_pulse),
  _phase("drive_off", 100, dict(ENGAGED, **LEAD_4, accel=0.6, torque=0.1), _drive_off),
  _phase("second_approach", 50, dict(ENGAGED, **LEAD_4, long_state=LongCtrlState.stopping, accel=-1.0, v_ego=1.0)),
  _phase("hold_never_latched", 100, dict(ENGAGED, **LEAD_4, long_state=LongCtrlState.stopping, accel=-1.0, standstill=True)),
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


RECORD_KEYS = ("tx", "torque_can", "accel")
STACK_RECORD_KEYS = ("tx", "fwd", "cs", "torque_can", "accel")


def frame_hash(rec, keys=RECORD_KEYS, chars=HASH_CHARS) -> str:
  payload = json.dumps([rec[k] for k in keys], separators=(",", ":"))
  return hashlib.sha256(payload.encode()).hexdigest()[:chars]


def build_fixture(records, keys=RECORD_KEYS, chars=HASH_CHARS):
  phases = []
  keyframes = {}
  for rec in records:
    if not phases or phases[-1]["name"] != rec["phase"]:
      phases.append({"name": rec["phase"], "start": rec["frame"]})
      keyframes[str(rec["frame"])] = {k: rec[k] for k in keys}
  return {
    "frames": len(records),
    "phases": phases,
    "hashes": [frame_hash(r, keys, chars) for r in records],
    "keyframes": keyframes,
  }


def load_fixture(path=FIXTURE):
  with open(path) as f:
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

  def uds(phase):
    return [(r["frame"], bytes.fromhex(dat)) for r in by_phase[phase] for addr, _, dat in r["tx"] if addr == 0x764]

  assert tx_addrs("boot_stock_radar") == {0x243, 0x440}
  # the teardown: programming-session requests at 2 Hz and no synthetic frame while the radar talks
  assert uds("fsc_settled_silencing") and 0x21b not in tx_addrs("fsc_settled_silencing")
  assert all(f % RADAR_UDS_STEP == 0 and d == SESSION_PROG_DAT for f, d in uds("fsc_settled_silencing"))
  assert {0x21b, 0x21c, 0x499, 0x364} <= tx_addrs("radar_silenced_armed_idle")
  assert TESTER_PRESENT_DAT in {d for _, d in uds("radar_silenced_armed_idle")}
  # the hand-back: default-session requests, never tester present, synthetic frames until the radar is back
  assert SESSION_DFLT_DAT in {d for _, d in uds("handback")} and TESTER_PRESENT_DAT not in {d for _, d in uds("handback")}
  assert 0x21b in tx_addrs("handback")
  assert max(r["torque_can"] for r in by_phase["engage_steer_ramp"]) > 1000
  # the 10 m/s command winds down at STEER_DELTA_DOWN a frame, settles on the 620 rail, then walks
  # to the opposite rail at 10 a frame (190 frames leave room for the 123-frame crossing)
  assert by_phase["highway_rail"][49]["torque_can"] == 620
  assert min(r["torque_can"] for r in by_phase["highway_rail"]) == -620
  assert {r["accel"] for r in by_phase["hold_on_the_plan"][-50:]} == {-1.0}
  assert {r["accel"] for r in by_phase["hold_body_latched"][-10:]} == {-0.001}  # raw -1, recorded to 4 decimals
  # a body-latched hold releases in-protocol: the pulse, and no button frame
  assert crz_info_unlatching("latched_release") and 0x9d not in tx_addrs("latched_release")
  assert not crz_info_unlatching("never_latched_release_breakaway")
  assert max(r["accel"] for r in by_phase["never_latched_release_breakaway"]) > 0.45
  assert {r["accel"] for r in by_phase["gas_override"]} == {0.}
  assert by_phase["steer_undelivered_latch"][0]["torque_can"] == 0
  assert 0x9d in tx_addrs("cancel")
  assert 0x9d in tx_addrs("icbm_taps")
  assert tx_addrs("stock_radar_back") == {0x243, 0x440}


# Full stack: synthetic CAN through CarInterface and the panda

P, D = 1, 4  # GEAR
IDLE_LANEINFO = bytes.fromhex("4201000000001040")  # a settled camera, the white wheel's canonical base
BOOT_LANEINFO = bytes.fromhex("4241000000001040")  # NO_ERR_BIT: the camera still booting
HBC_LANEINFO = bytes.fromhex("4221000000001040")   # BIT2: auto high beam armed
SESSION_REPLIES = {0x01: bytes.fromhex("065001003201f400"), 0x02: bytes.fromhex("065002003201f400")}
SWAPPED_EPS_FW = b'KSD5-3210X-C-00' + b'\x00' * 9
G46L_FW = sorted(G46L_RADAR_FW)[0].ljust(24, b'\x00')

# what the car sends, with each message's period in 100 Hz frames
PT_PERIODS = {"ENGINE_DATA": 1, "PEDALS": 1, "STEER_TORQUE": 1, "WHEEL_SPEEDS": 2, "STEER": 2, "STEER_RATE": 2,
              "CRZ_EVENTS": 2, "GEAR": 5, "EPB": 5, "BLINK_INFO": 10, "BSM": 10, "SEATBELT": 10, "DOORS": 10, "CRZ_BTNS": 10}
RADAR_PERIODS = {"CRZ_CTRL": 2, "CRZ_INFO": 2}
CAM_PERIODS = {"CAM_LKAS": 2, "CAM_EMPTY": 10, "CAM_PEDESTRIAN": 10, "CAM_SETTINGS": 20, "CAM_TRAFFIC_SIGNS": 20}
CAM_LANEINFO_PERIOD = 50

IDLE_BUTTONS = {"CAN_OFF_INV": 1, "SET_P_INV": 1, "RES_INV": 1, "SET_M_INV": 1, "DISTANCE_LESS_INV": 1,
                "DISTANCE_MORE_INV": 1, "MODE_X_INV": 1, "MODE_Y_INV": 1, "BIT1": 1, "BIT2": 1, "BIT3": 1}
BUTTONS = {None: {}, "main": {"MODE_Y": 1, "MODE_Y_INV": 0}, "set": {"SET_M": 1, "SET_M_INV": 0},
           "set_plus": {"SET_P": 1, "SET_P_INV": 0}, "tja": {"TJA_BUTTON": 1}, "mrcc": {"BIT1": 0, "BIT1_INV": 1}}


class SimCar:
  """The car's side of the bus: what carstate parses, at the car's own rates, and the closed loops
  the port relies on. The radar answers its UDS session and falls silent in programming, the EPS
  echoes the torque the panda let through, the body lets go of a hold on the unlatch pulse and
  disarms MRCC on a master press."""

  def __init__(self):
    self.packer = CANPacker(DBC_NAME)
    self.state = dict(v=0., gear=P, brake=True, gas=0, driver_torque=0, armed=False, active=False, crz_speed=0., button=None,
                      laneinfo=IDLE_LANEINFO, lkas_block=False, lkas_track=False, lane_keep=True, hold=False, speed_sign=0)
    self.frame = 0
    self.radar_alive = True
    self.eps_request = 0
    self.btn_counter = 0
    self.replies: list = []
    self.release_at: int | None = None
    self.disarm_at: int | None = None

  def can(self):
    s, f = self.state, self.frame
    kph = s["v"] * CV.MS_TO_KPH
    signals = {
      "ENGINE_DATA": {"SPEED": kph, "PEDAL_GAS": s["gas"]},
      "PEDALS": {"ACC_OFF": s["armed"], "ACC_ACTIVE": s["active"], "BRAKE_ON": s["brake"], "STANDSTILL": kph <= 0.1},
      "STEER_TORQUE": {"STEER_TORQUE_SENSOR": s["driver_torque"]},
      "WHEEL_SPEEDS": {w: kph for w in ("FL", "FR", "RL", "RR")},
      "STEER": {"STEER_ANGLE": 0.},
      "STEER_RATE": {"LKAS_REQUEST": self.eps_request, "LKAS_EFFECTIVE": 0 if s["lkas_block"] else self.eps_request,
                     "LKAS_BLOCK": s["lkas_block"], "LKAS_TRACK_STATE": s["lkas_track"]},
      "CRZ_EVENTS": {"CRZ_SPEED": s["crz_speed"]},
      "GEAR": {"GEAR": s["gear"]},
      "EPB": {"HOLD_STATE": 3 if s["hold"] else 2},
      "BLINK_INFO": {}, "BSM": {}, "DOORS": {},
      "SEATBELT": {"DRIVER_SEATBELT": 1},
      "CRZ_BTNS": {"CTR": self.btn_counter, **IDLE_BUTTONS, **BUTTONS[s["button"]]},
      "CRZ_CTRL": {"CRZ_AVAILABLE": s["armed"] or s["active"], "CRZ_ACTIVE": s["active"]},
      "CRZ_INFO": {"CTR": f // 2 % 16, "ACC_ACTIVE": s["active"]},
      "CAM_LKAS": {"CTR": f // 2 % 16, "BIT_1": 1},
      "CAM_EMPTY": {"STATUS": 0x7f}, "CAM_PEDESTRIAN": {},
      "CAM_SETTINGS": {"LKAS_INERVENTION_ON1": s["lane_keep"]},
      "CAM_TRAFFIC_SIGNS": {"SPEED_SIGN": s["speed_sign"], "SPEED_SIGN_UNIT": 2 if s["speed_sign"] else 0},
    }
    periods = [(PT_PERIODS, 0), (RADAR_PERIODS if self.radar_alive else {}, 0), (CAM_PERIODS, 2)]
    msgs = [self.packer.make_can_msg(name, bus, signals[name]) for table, bus in periods for name, period in table.items() if f % period == 0]
    if f % PT_PERIODS["CRZ_BTNS"] == 0:
      self.btn_counter = (self.btn_counter + 1) % 16
    if f % CAM_LANEINFO_PERIOD == 0:
      msgs.append((0x440, s["laneinfo"], 2))
    msgs += self.replies
    self.replies = []
    return msgs

  def react(self, sends, allowed):
    self.frame += 1
    self.eps_request = 0  # the panda forwards the camera's zero instead of a refused frame
    for (addr, dat, bus), ok in zip(sends, allowed, strict=True):
      if not ok or bus != 0:
        continue
      if addr == 0x764 and dat[:2] == b"\x02\x10":
        self.replies.append((0x76c, SESSION_REPLIES[dat[2]], 0))
        self.radar_alive = dat[2] == 0x01
      elif addr == 0x243:
        self.eps_request = (((dat[0] & 0x0f) << 8) | dat[1]) - 2048
      elif addr == 0x21b and dat[6] & 0x40 and self.state["hold"] and self.release_at is None:
        self.release_at = self.frame + 4
      elif addr == 0x9d and dat[:3] == b"\x00\x81\xfe" and self.disarm_at is None:
        self.disarm_at = self.frame + 8
    if self.release_at == self.frame:
      self.state["hold"], self.release_at = False, None
    if self.disarm_at == self.frame:
      self.state["armed"], self.disarm_at = False, None


def stack_interface(candidate, alpha_long=False, car_fw=None, tja_button=False) -> CarInterface:
  CP = car_params(candidate, alpha_long=alpha_long, car_fw=car_fw)
  CP_SP = car_params_sp(CP, candidate, alpha_long=alpha_long, car_fw=car_fw)
  if tja_button:
    CP_SP.flags |= MazdaFlagsSP.TJA_BUTTON
    CP_SP.safetyParam |= MazdaSafetyFlagsSP.TJA_BUTTON
  return CarInterface(CP, CP_SP)


def carstate_record(ci, cs, cs_sp, safety) -> dict:
  """What card publishes from carstate and the controller, and the panda's view of the same frame."""
  c = cs.cruiseState
  return {
    "vEgoRaw": round(cs.vEgoRaw, 4), "standstill": cs.standstill, "gear": str(cs.gearShifter), "brake": cs.brakePressed,
    "gas": cs.gasPressed, "steeringPressed": cs.steeringPressed, "steeringTorque": round(cs.steeringTorque, 2),
    "available": c.available, "enabled": c.enabled, "speed": round(c.speed, 4), "speedCluster": round(c.speedCluster, 4),
    "cruiseStandstill": c.standstill, "accFaulted": cs.accFaulted, "invalidLkasSetting": cs.invalidLkasSetting,
    "lowSpeedAlert": cs.lowSpeedAlert, "steerFaultTemporary": cs.steerFaultTemporary,
    "steerFaultPermanent": cs.steerFaultPermanent, "stockFcw": cs.stockFcw, "canValid": cs.canValid,
    "canTimeout": cs.canTimeout, "buttonEnable": cs.buttonEnable,
    "buttons": [[str(b.type), b.pressed] for b in cs.buttonEvents], "speedLimit": round(cs_sp.speedLimit, 4),
    "stockEcu": str(ci.CC.stock_ecu_state), "distanceFarther": int(ci.CS.distance_more_button), "lkasArming": ci.CS.lkas_arming,
    "pandaControls": safety.get_controls_allowed(), "pandaLateral": safety.get_controls_allowed_lateral(),
    "pandaMain": safety.get_acc_main_on(),
  }


STACK_CC = dict(enabled=False, long_active=False, lat_active=False, accel=0., long_state=LongCtrlState.off, lead_visible=False, gap=2)


def run_stack(ci, phases, mads=False):
  """One configuration: per control frame, the car's frames reach the panda and carstate, the controller
  answers, and the panda judges every frame it sends. Its refusals come back on src 192 as on the car."""
  sim = SimCar()
  safety = libsafety_py.libsafety
  safety.set_current_safety_param_sp(ci.CP_SP.safetyParam)
  cfg = ci.CP.safetyConfigs[0]
  assert safety.set_safety_hooks(cfg.safetyModel.raw, cfg.safetyParam) == 0
  safety.init_tests()
  safety.set_mads_params(mads, False, False)
  loopback: list = []
  try:
    for name, n, base, per_frame in phases:
      sim.state.update({k: v for k, v in base.items() if k in sim.state})
      for i in range(n):
        kwargs = dict(STACK_CC, **{k: v for k, v in base.items() if k not in sim.state})
        for k, v in (per_frame(i, sim) if per_frame is not None else {}).items():
          if k in sim.state:
            sim.state[k] = v
          else:
            kwargs[k] = v
        cc_kw, cc_sp_kw, cs_kw = split_inputs(kwargs)
        assert not cs_kw, f"not a car or CarControl input: {cs_kw}"
        t = int(sim.frame * DT_CTRL * 1e9)

        safety.set_timer(sim.frame * 10_000)
        msgs = sim.can()
        fwd = []
        for addr, dat, bus in msgs:
          safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, dat))
          if bus == 2 and addr in (0x243, 0x440):
            fwd.append([addr, safety.safety_fwd_hook(bus, addr)])
        safety.safety_tick()

        cs, cs_sp = ci.update([(t, msgs + loopback)])
        actuators, sends = ci.apply(car_control(**cc_kw), car_control_sp(**cc_sp_kw), t)
        allowed = [bool(safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, bus, dat))) for addr, dat, bus in sends]
        loopback = [(addr, dat, bus + 192) for (addr, dat, bus), ok in zip(sends, allowed, strict=True) if not ok]
        yield {
          "frame": sim.frame,
          "phase": name,
          "tx": [[addr, bus, dat.hex(), ok] for (addr, dat, bus), ok in zip(sends, allowed, strict=True)],
          "fwd": fwd,
          "cs": carstate_record(ci, cs, cs_sp, safety),
          "torque_can": int(actuators.torqueOutputCan),
          "accel": round(float(actuators.accel), 4),
        }
        sim.react(sends, allowed)
  finally:
    safety.set_mads_params(False, False, False)
    safety.set_current_safety_param_sp(0)


def _ramp(x, rate, limit):
  return min(x + rate, limit) if rate > 0 else max(x + rate, limit)


def _press(button, i, frames=20):
  return {"button": button if i < frames else None}


ENGAGED_LONG = dict(enabled=True, long_active=True, lat_active=True, long_state=LongCtrlState.pid)
ENGAGED_STOCK = dict(enabled=True, lat_active=True)
MADS_LATERAL = dict(lat_active=True, mads_active=True, torque=0.2)
PARKED = dict(v=0., gear=P, brake=True)


def _set_engages(engaged):
  # SET reaches the body on CRZ_BTNS, the body engages a cycle later, openpilot follows its cruise state
  return lambda i, _: {**_press("set", i), "gear": D, "brake": False, "active": i >= 10, **(engaged if i >= 15 else {})}


def _takeover(laneinfo_frames):
  return [
    ("boot_camera_booting", laneinfo_frames, dict(PARKED, laneinfo=BOOT_LANEINFO), None),
    ("camera_settled", 720, dict(laneinfo=IDLE_LANEINFO), None),
    ("radar_takeover", 200, {}, None),
    ("main_on", 60, {}, lambda i, _: {**_press("main", i), "armed": i >= 10}),
    ("set_engages", 60, {}, _set_engages(ENGAGED_LONG)),
  ]


ALPHA_LONG_DRIVE = _takeover(150) + [
  ("follow", 300, dict(ENGAGED_LONG, lead_visible=True, lead_d_rel=30., lead_v_rel=-1., gap=3),
   lambda i, sim: {"v": _ramp(sim.state["v"], 0.05, 12.), "accel": 1.0 if i < 200 else 0.3, "torque": min(i, 100) / 200,
                   "driver_torque": FIGHTING_DRIVER[i - 120] if 120 <= i < 120 + len(FIGHTING_DRIVER) else 0,
                   "visual_alert": VisualAlert.steerRequired if 220 <= i < 280 else VisualAlert.none,
                   "laneinfo": HBC_LANEINFO if i >= 150 else IDLE_LANEINFO}),
  ("stop", 200, dict(ENGAGED_LONG, lead_visible=True, lead_d_rel=6., torque=0.1, driver_torque=0),
   lambda i, sim: {"v": _ramp(sim.state["v"], -0.12, 0.), "accel": -1.5 if i < 100 else -1.0,
                   "long_state": LongCtrlState.stopping if i >= 80 else LongCtrlState.pid}),
  ("body_holds", 150, dict(ENGAGED_LONG, lead_visible=True, lead_d_rel=4., long_state=LongCtrlState.stopping, accel=-1.0),
   lambda i, _: {"hold": i >= 50}),
  ("unlatch_release", 150, dict(ENGAGED_LONG, lead_visible=True, lead_d_rel=4., accel=1.0),
   lambda i, sim: {"v": 0. if sim.state["hold"] else _ramp(sim.state["v"], 0.03, 3.)}),
  ("gas_override", 60, dict(ENGAGED_LONG, long_active=False, long_state=LongCtrlState.off, gas=30, torque=0.2),
   lambda i, sim: {"v": _ramp(sim.state["v"], 0.02, 5.)}),
  ("brake_cancels", 80, dict(brake=True, gas=0, active=False),
   lambda i, sim: {"cancel": i < 40, "v": _ramp(sim.state["v"], -0.05, 0.)}),
  ("park_hand_back", 300, dict(PARKED, handback=True), None),
]

STOCK_LONG_DRIVE = [
  ("parked_main_off", 50, PARKED, None),
  ("main_on", 50, {}, lambda i, _: {**_press("main", i), "armed": i >= 10}),
  ("lateral_before_the_panda", 40, dict(gear=D, brake=False, v=15., lat_active=True, torque=0.3), None),
  ("set_engages", 50, dict(crz_speed=54., speed_sign=50), _set_engages(ENGAGED_STOCK)),
  ("steer_against_the_driver", 200, ENGAGED_STOCK,
   lambda i, _: {"torque": min(i, 100) / 100, "driver_torque": -20 if 100 <= i < 120 else 0}),
  ("steer_required", 300, dict(ENGAGED_STOCK, torque=0.3),
   lambda i, _: {"visual_alert": VisualAlert.steerRequired if 50 <= i < 250 else VisualAlert.none}),
  ("eps_undelivered", 200, dict(ENGAGED_STOCK, torque=0.8, lkas_block=True), None),
  ("lane_keep_off", 100, dict(ENGAGED_STOCK, torque=0.3, lkas_block=False, lane_keep=False), None),
  ("lane_keep_back", 400, dict(ENGAGED_STOCK, torque=0.3, lane_keep=True), lambda i, _: {"lkas_block": i < 305}),
  ("resume_button", 40, dict(ENGAGED_STOCK, resume=True), None),
  ("icbm_tap", 40, dict(ENGAGED_STOCK, send_button=SendButtonState.increase), None),
  ("brake_cancels", 60, dict(brake=True, active=False), lambda i, _: {"cancel": i < 40}),
]

LEGACY_EPS_DRIVE = [
  ("engaged_below_the_floor", 150, dict(v=40. * CV.KPH_TO_MS, armed=True, lkas_block=True),
   lambda i, sim: {**_set_engages(dict(ENGAGED_STOCK, torque=0.3, visual_alert=VisualAlert.steerRequired))(i, sim)}),
  ("past_the_floor", 200, dict(ENGAGED_STOCK, torque=0.3),
   lambda i, _: {"v": (40. + i * 0.1) * CV.KPH_TO_MS, "lkas_block": i < 150}),
  ("block_above_the_floor", 100, dict(ENGAGED_STOCK, torque=0.3, lkas_block=True), None),
  ("under_the_floor", 150, dict(ENGAGED_STOCK, torque=0.3, lkas_block=False),
   lambda i, _: {"v": (60. - i * 0.2) * CV.KPH_TO_MS}),
]

G46L_DRIVE = _takeover(100) + [
  ("follow", 150, dict(ENGAGED_LONG, lead_visible=True, lead_d_rel=20., accel=0.8, torque=0.2),
   lambda i, sim: {"v": _ramp(sim.state["v"], 0.05, 6.)}),
]

TJA_MADS_DRIVE = [
  ("cruising_mads_off", 50, dict(gear=D, brake=False, v=15.), None),
  # the press arms MRCC in the body; the controller undoes it with the wheel's master press
  ("tja_press_arms_mrcc", 200, {}, lambda i, _: {**_press("tja", i, 15), **({"armed": True} if i == 8 else {}),
                                                  **(MADS_LATERAL if i >= 5 else {})}),
  ("white_wheel", 150, MADS_LATERAL, None),
  ("set_plus_withdraws_it", 60, MADS_LATERAL, lambda i, _: _press("set_plus", i)),
  ("white_wheel_again", 100, MADS_LATERAL, None),
  ("driver_arms_mrcc", 80, MADS_LATERAL, lambda i, _: {**_press("mrcc", i, 15), **({"armed": True} if i == 8 else {})}),
]

STACK_CONFIGS = {
  "alpha_long_cx5_2022": (lambda: stack_interface(CAR.MAZDA_CX5_2022, alpha_long=True), ALPHA_LONG_DRIVE, False),
  "stock_long_cx5_2022": (lambda: stack_interface(CAR.MAZDA_CX5_2022), STOCK_LONG_DRIVE, False),
  "legacy_eps_cx9_2021": (lambda: stack_interface(CAR.MAZDA_CX9_2021), LEGACY_EPS_DRIVE, False),
  "g46l_cx5_ke_swapped_eps": (lambda: stack_interface(CAR.MAZDA_CX5_KE, alpha_long=True,
                                                      car_fw=eps_fw(SWAPPED_EPS_FW) + [radar_fw(G46L_FW)]), G46L_DRIVE, False),
  "tja_mads_cx5_2022": (lambda: stack_interface(CAR.MAZDA_CX5_2022, tja_button=True), TJA_MADS_DRIVE, True),
}


@functools.cache
def stack_records(name) -> list:
  make_ci, phases, mads = STACK_CONFIGS[name]
  return list(run_stack(make_ci(), phases, mads))


@pytest.mark.parametrize("name", STACK_CONFIGS)
def test_stack_matches(name):
  fixture = load_fixture(STACK_FIXTURE)[name]
  _, phases, _ = STACK_CONFIGS[name]
  assert [p["name"] for p in fixture["phases"]] == [p[0] for p in phases]
  records = stack_records(name)
  assert len(records) == fixture["frames"]
  for rec, expected in zip(records, fixture["hashes"], strict=True):
    if frame_hash(rec, STACK_RECORD_KEYS, STACK_HASH_CHARS) != expected:
      key = fixture["keyframes"].get(str(rec["frame"]))
      golden = f"\ngolden keyframe: {json.dumps(key)}" if key else ""
      now = json.dumps({k: rec[k] for k in STACK_RECORD_KEYS})
      raise AssertionError(f"{name} diverged at frame {rec['frame']} ({rec['phase']}):\nnow: {now}{golden}")


def test_stack_reaches_every_state():
  def phase(name, phase_name):
    return [r for r in stack_records(name) if r["phase"] == phase_name]

  def sent(recs, addr, allowed=True, bus=0):
    return [bytes.fromhex(d) for r in recs for a, b, d, ok in r["tx"] if a == addr and b == bus and ok == allowed]

  def cs(recs, field):
    return [r["cs"][field] for r in recs]

  # alpha long: the takeover, an engagement the panda allows, the unlatch pulse, the hand-back
  alpha = "alpha_long_cx5_2022"
  assert sent(phase(alpha, "camera_settled"), 0x764)
  assert cs(phase(alpha, "radar_takeover"), "stockEcu")[-1] == "ready"
  assert any(d[4] & 0x02 for d in sent(phase(alpha, "follow"), 0x21b))  # ACC_ACTIVE
  assert any(d[6] & 0x40 for d in sent(phase(alpha, "unlatch_release"), 0x21b))  # RESUME_UNLATCHING
  assert cs(phase(alpha, "park_hand_back"), "stockEcu")[-1] == "restored"
  # stock long: a refusal fed back on src 192, the non-delivery alert, the lane-keep setting, the buttons
  stock = "stock_long_cx5_2022"
  assert sent(phase(stock, "lateral_before_the_panda"), 0x243, allowed=False)
  assert any(cs(phase(stock, "eps_undelivered"), "steerFaultTemporary"))
  assert all(cs(phase(stock, "lane_keep_off"), "invalidLkasSetting")[10:])
  assert any(cs(phase(stock, "lane_keep_back"), "lkasArming"))
  assert sent(phase(stock, "resume_button"), 0x9d) and sent(phase(stock, "icbm_tap"), 0x9d)
  assert sent(phase(stock, "brake_cancels"), 0x9d) and sent(phase(alpha, "brake_cancels"), 0x9d)
  # legacy firmware: the speed floor's alert and its block fault
  legacy = "legacy_eps_cx9_2021"
  assert any(cs(phase(legacy, "engaged_below_the_floor"), "lowSpeedAlert"))
  assert any(cs(phase(legacy, "block_above_the_floor"), "steerFaultTemporary"))
  # the G46L dialect: its static frame on both buses, no tracks
  g46l = phase("g46l_cx5_ke_swapped_eps", "follow")
  assert sent(g46l, 0x499) and sent(g46l, 0x499, bus=2) and not sent(g46l, 0x361)
  # TJA: the MRCC undo press, the white wheel on the wire, the panda's lateral from the button
  tja = "tja_mads_cx5_2022"
  assert any(d[:3] == b"\x00\x81\xfe" for d in sent(phase(tja, "tja_press_arms_mrcc"), 0x9d))
  assert any(d[4] & 0x20 for d in sent(phase(tja, "white_wheel"), 0x440))
  assert all(cs(phase(tja, "white_wheel"), "pandaLateral"))


if __name__ == "__main__":
  if "--update" not in sys.argv:
    print(__doc__)
    sys.exit(1)
  recs = list(run_scenario())
  with open(FIXTURE, "w") as f:
    json.dump(build_fixture(recs), f, separators=(",", ":"))
  print(f"wrote {FIXTURE}: {len(recs)} frames, {os.path.getsize(FIXTURE)} bytes")
  stack = {name: build_fixture(stack_records(name), STACK_RECORD_KEYS, STACK_HASH_CHARS) for name in STACK_CONFIGS}
  with open(STACK_FIXTURE, "w") as f:
    json.dump(stack, f, separators=(",", ":"))
  print(f"wrote {STACK_FIXTURE}: {sum(c['frames'] for c in stack.values())} frames, {os.path.getsize(STACK_FIXTURE)} bytes")
  if "--dump" in sys.argv:
    dump = sys.argv[sys.argv.index("--dump") + 1]
    with open(dump, "w") as f:
      json.dump(recs, f)
    print(f"wrote {dump}: {os.path.getsize(dump)} bytes")
