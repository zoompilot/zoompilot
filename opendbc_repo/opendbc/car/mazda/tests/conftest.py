"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Shared builders for the Mazda tests. Everything the controller and carstate read is a real
object here: CarParams from CarInterface.get_params, capnp CarState / CarControl messages,
the port's own CarState class seeded through its public attributes, and CANPacker /
CANParser round trips for the frames on the wire.
"""
import inspect

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.carstate import CarState, FSC_SETTLE_FRAMES, STOCK_RADAR_ALIVE_FRAMES, STOCK_RADAR_GUARD_FRAMES
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR, CarControllerParams

DBC_NAME = "mazda_2017"

LongCtrlState = structs.CarControl.Actuators.LongControlState
VisualAlert = structs.CarControl.HUDControl.VisualAlert
SendButtonState = structs.IntelligentCruiseButtonManagement.SendButtonState

# addresses the tests read back off the bus
CRZ_BTNS = 0x9d
CRZ_INFO = 0x21b
CRZ_CTRL = 0x21c
CAM_LKAS = 0x243
CAM_LANEINFO = 0x440
LEAD_TRACK = 0x364
RADAR_STATIC = 0x499
RADAR_UDS = 0x764

MSG_NAMES = {
  CRZ_BTNS: "CRZ_BTNS",
  CRZ_INFO: "CRZ_INFO",
  CRZ_CTRL: "CRZ_CTRL",
  CAM_LKAS: "CAM_LKAS",
  CAM_LANEINFO: "CAM_LANEINFO",
}

SESSION_PROG_DAT = bytes([0x02, 0x10, 0x02, 0, 0, 0, 0, 0])
SESSION_DFLT_DAT = bytes([0x02, 0x10, 0x01, 0, 0, 0, 0, 0])
TESTER_PRESENT_DAT = bytes([0x02, 0x3e, 0x80, 0, 0, 0, 0, 0])


# CarParams and port objects

def car_params(candidate=CAR.MAZDA_CX5_2022, alpha_long=False, car_fw=None) -> structs.CarParams:
  return CarInterface.get_params(candidate, gen_empty_fingerprint(), car_fw or [],
                                 alpha_long=alpha_long, is_release=False, docs=False)


def car_params_sp(CP, candidate=CAR.MAZDA_CX5_2022, alpha_long=False, car_fw=None) -> structs.CarParamsSP:
  return CarInterface.get_params_sp(CP, candidate, gen_empty_fingerprint(), car_fw or [],
                                    alpha_long=alpha_long, is_release_sp=False, docs=False)


def cx5_2022_params(alpha_long=False) -> structs.CarParams:
  return car_params(CAR.MAZDA_CX5_2022, alpha_long=alpha_long)


def controller_params(candidate=CAR.MAZDA_CX5_2022, car_fw=None) -> CarControllerParams:
  """CarControllerParams for a real CarParams: the CX-5 2022 tune, a swapped EPS, or stock."""
  return CarControllerParams(car_params(candidate, car_fw=car_fw))


def car_interface(alpha_long=True, candidate=CAR.MAZDA_CX5_2022, car_fw=None) -> CarInterface:
  CP = car_params(candidate, alpha_long=alpha_long, car_fw=car_fw)
  CP_SP = car_params_sp(CP, candidate, alpha_long=alpha_long, car_fw=car_fw)
  return CarInterface(CP, CP_SP)


def car_controller(alpha_long=True, candidate=CAR.MAZDA_CX5_2022) -> CarController:
  CP = car_params(candidate, alpha_long=alpha_long)
  CP_SP = car_params_sp(CP, candidate, alpha_long=alpha_long)
  assert CP.openpilotLongitudinalControl == alpha_long
  return CarController({Bus.pt: DBC_NAME}, CP, CP_SP)


# capnp messages

def car_state(standstill=False, gas=False, brake_pressed=False, v_ego=0., driver_torque=0.,
              steering_pressed=False, available=True, cruise_engaged=False) -> structs.CarState:
  """structs.CarState with the fields the controller reads off CS.out."""
  ret = structs.CarState()
  ret.standstill = standstill
  ret.gasPressed = gas
  ret.brakePressed = brake_pressed
  ret.vEgoRaw = v_ego
  ret.vEgo = v_ego
  ret.steeringTorque = driver_torque
  ret.steeringPressed = steering_pressed
  ret.cruiseState.available = available
  ret.cruiseState.enabled = cruise_engaged
  return ret


def car_control(enabled=None, long_active=True, lat_active=False, accel=0.5, torque=0.,
                long_state=LongCtrlState.pid, cancel=False, resume=False, lead_visible=True, gap=2,
                visual_alert=VisualAlert.none):
  """structs.CarControl reader, as card hands it to the controller. openpilot is enabled
  whenever it is longitudinally active; a gas override is the case where it stays enabled
  with longActive low."""
  cc = structs.CarControl()
  cc.enabled = long_active if enabled is None else enabled
  cc.longActive = long_active
  cc.latActive = lat_active
  cc.actuators.accel = accel
  cc.actuators.torque = torque
  cc.actuators.longControlState = long_state
  cc.cruiseControl.cancel = cancel
  cc.cruiseControl.resume = resume
  cc.hudControl.leadVisible = lead_visible
  cc.hudControl.leadDistanceBars = gap
  cc.hudControl.visualAlert = visual_alert
  return cc.as_reader()


def car_control_sp(handback=False, lead_d_rel=12.0, lead_v_rel=0.0, send_button=SendButtonState.none) -> structs.CarControlSP:
  cc_sp = structs.CarControlSP()
  cc_sp.stockEcuHandBack = handback
  cc_sp.leadOne.dRel = lead_d_rel
  cc_sp.leadOne.vRel = lead_v_rel
  cc_sp.intelligentCruiseButtonManagement.sendButton = send_button
  return cc_sp


# CarState seeded without a bus

def set_car_state(cs: CarState, out=None, *, brake_hold=False, stock_radar_alive=False, stock_radar_gone=None,
                  fsc_settled=True, radar_was_silenced=False, radar_session_refused=False, steer_undelivered=False,
                  lkas_blocked=False, lkas_effective=0, lkas_allowed_speed=True, crz_btns_counter=0,
                  cancel_button=0, accel_button=0, decel_button=0, **out_kwargs) -> CarState:
  """Put the controller-facing state of a real CarState where a test wants it.

  Every keyword is reset to its default on each call, so a test that drives frame by frame
  gets the same semantics as a fresh state each frame. The stock-radar and FSC properties are
  derived from their frame counters, so those are what get seeded: a silent radar reads as
  gone (guard-long silence) unless a test is about the gap between the two windows.
  """
  cs.out = out if out is not None else car_state(**out_kwargs)
  cs.brake_hold = brake_hold
  if stock_radar_gone is None:
    stock_radar_gone = not stock_radar_alive
  if stock_radar_alive:
    cs.stock_radar_silent_frames = 0
  elif stock_radar_gone:
    cs.stock_radar_silent_frames = STOCK_RADAR_GUARD_FRAMES
  else:
    cs.stock_radar_silent_frames = STOCK_RADAR_ALIVE_FRAMES
  cs.fsc_settled_frames = FSC_SETTLE_FRAMES if fsc_settled else 0
  cs.radar_was_silenced = radar_was_silenced
  cs.radar_session_refused = radar_session_refused
  cs.steer_undelivered = steer_undelivered
  cs.lkas_blocked = lkas_blocked
  cs.lkas_effective = lkas_effective
  cs.lkas_allowed_speed = lkas_allowed_speed
  cs.crz_btns_counter = crz_btns_counter
  cs.cancel_button = cancel_button
  cs.accel_button = accel_button
  cs.decel_button = decel_button
  return cs


def mazda_car_state(CP, CP_SP, **kwargs) -> CarState:
  """The port's own CarState, with the camera pass-through dicts a real parser hands it."""
  cs = CarState(CP, CP_SP)
  parsers = CarState.get_can_parsers(CP, CP_SP)
  cs.cam_lkas = parsers[Bus.cam].vl["CAM_LKAS"]
  cs.cam_laneinfo = parsers[Bus.cam].vl["CAM_LANEINFO"]
  return set_car_state(cs, **kwargs)


_CC_KEYS = set(inspect.signature(car_control).parameters)
_CC_SP_KEYS = set(inspect.signature(car_control_sp).parameters)
_OUT_KEYS = set(inspect.signature(car_state).parameters)
_CS_KEYS = set(inspect.signature(set_car_state).parameters) - {"cs", "out", "out_kwargs"}


def split_inputs(kwargs):
  """Route one flat keyword set to car_control, car_control_sp and set_car_state."""
  cc_kw, cc_sp_kw, cs_kw = {}, {}, {}
  for k, v in kwargs.items():
    if k in _CC_KEYS:
      cc_kw[k] = v
    elif k in _CC_SP_KEYS:
      cc_sp_kw[k] = v
    elif k in _CS_KEYS or k in _OUT_KEYS:
      cs_kw[k] = v
    else:
      raise TypeError(f"unknown input {k!r}")
  return cc_kw, cc_sp_kw, cs_kw


def step(cc: CarController, cs: CarState, **kwargs):
  """One full CarController.update() frame. Returns (actuators, can_sends)."""
  cc_kw, cc_sp_kw, cs_kw = split_inputs(kwargs)
  set_car_state(cs, **cs_kw)
  return cc.update(car_control(**cc_kw), car_control_sp(**cc_sp_kw), cs, 0)


def step_long(cc: CarController, cs: CarState, **kwargs):
  """One update_longitudinal() frame, advancing the controller's frame counter."""
  cc_kw, cc_sp_kw, cs_kw = split_inputs(kwargs)
  set_car_state(cs, **cs_kw)
  sends = cc.update_longitudinal(car_control(**cc_kw), car_control_sp(**cc_sp_kw), cs)
  cc.frame += 1
  return sends


# Wire frames

def packer() -> CANPacker:
  return CANPacker(DBC_NAME)


def parse_frame(addr, dat, bus=0, msg=None) -> dict:
  """The DBC's view of one frame, so a test can assert on signal values."""
  msg = msg or MSG_NAMES.get(addr, addr)  # the radar tracks are addressed by id
  cp = CANParser(DBC_NAME, [(msg, float("nan"))], bus)
  cp.update([(0, [(addr, dat, bus)])])
  return dict(cp.vl[msg])


def frames(sends, addr, bus=0) -> list[bytes]:
  return [d for a, d, b in sends if a == addr and b == bus]


def frame(sends, addr, bus=0) -> bytes | None:
  return next(iter(frames(sends, addr, bus)), None)


def addrs(sends) -> list[int]:
  return [a for a, _, _ in sends]


def crz_info(dat) -> tuple[int, bool, bool]:
  """(ACCEL_CMD raw, STOPPING, RESUME_UNLATCHING) from a CRZ_INFO frame."""
  v = parse_frame(CRZ_INFO, dat)
  return round(v["ACCEL_CMD"] * 1000), bool(v["STOPPING"]), bool(v["RESUME_UNLATCHING"])


def accel_cmd_raw(sends, bus=0) -> int | None:
  dat = frame(sends, CRZ_INFO, bus)
  return None if dat is None else crz_info(dat)[0]


def crz_ctrl_lead(dat) -> tuple[int, int]:
  """(RADAR_HAS_LEAD, RADAR_LEAD_RELATIVE_DISTANCE) from a CRZ_CTRL frame."""
  v = parse_frame(CRZ_CTRL, dat)
  return int(v["RADAR_HAS_LEAD"]), int(v["RADAR_LEAD_RELATIVE_DISTANCE"])


def lead_track(dat) -> tuple[float, float]:
  """(DIST_OBJ, RELV_OBJ) decoded from a 0x364 track frame."""
  v = parse_frame(LEAD_TRACK, dat)
  return v["DIST_OBJ"], v["RELV_OBJ"]


# fixtures

@pytest.fixture(name="packer")
def packer_fixture() -> CANPacker:
  return packer()


@pytest.fixture
def cc() -> CarController:
  """An alpha-long CX-5 2022 controller."""
  return car_controller(alpha_long=True)


@pytest.fixture
def stock_cc() -> CarController:
  """A CX-5 2022 controller with the stock MRCC still in charge of longitudinal."""
  return car_controller(alpha_long=False)


@pytest.fixture
def cs(cc) -> CarState:
  return mazda_car_state(cc.CP, cc.CP_SP)


@pytest.fixture
def stock_cs(stock_cc) -> CarState:
  return mazda_car_state(stock_cc.CP, stock_cc.CP_SP)
