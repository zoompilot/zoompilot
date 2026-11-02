"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The message builders in mazdacan. The synthetic CRZ_INFO / CRZ_CTRL / radar frames must
reproduce stock captures byte for byte; the hex values below come from real radar traffic.
"""
import pytest

from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.values import Buttons
from opendbc.car.mazda.tests.conftest import CAM_LANEINFO, car_params, hands_code, parse_frame


def crz_info_reference_checksum(dat):
  # independent reimplementation of the CRZ_INFO checksum, validated against 1.67M stock
  # frames with zero mismatches: the sum excludes the STOPPING bit (byte 5, 9,681 frames)
  # and the RESUME_UNLATCHING bit (byte 6, 269 frames)
  return (0xFF - ((sum(dat[:7]) - (dat[5] & 0x04) - (dat[6] & 0x40)) & 0xFF)) & 0xFF


def decode_accel_cmd_raw(dat):
  return (((dat[2] & 0x3) << 11) | (dat[3] << 3) | (dat[4] >> 5)) - 4096


def test_alert_command_relays_state_but_not_the_tja_churn(packer):
  # Preserve camera error and lane state, but clear TJA state that does not own the command.
  cam_msg = {"LINE_VISIBLE": 1, "LINE_NOT_VISIBLE": 0, "LANE_LINES": 2, "BIT1": 1,
             "BIT2": 0, "BIT3": 0, "NO_ERR_BIT": 0, "ERR_BIT": 1,
             "TJA": 4, "TJA_TRANSITION": 3, "S1": 1, "S1_HBEAM": 0}
  dat = mazdacan.create_alert_command(packer, cam_msg, ldw=False, steer_required=False)[1]
  out = parse_frame(CAM_LANEINFO, dat)
  assert out["ERR_BIT"] == 1 and out["LINE_VISIBLE"] == 1 and out["LANE_LINES"] == 2 and out["S1"] == 1
  assert out["TJA"] == 0 and out["TJA_TRANSITION"] == 0


@pytest.mark.parametrize("steer_required", [False, True])
def test_alert_command_sends_the_frame_the_cluster_draws(packer, steer_required):
  # On-car map 2026-09-30: the cluster draws the hands-on-wheel text only with the 0b111 code
  # and HANDS_ON_STEER_WARN_2 together; (0,1,0), (0,1,1), (7,1,0) and (0,0,1) draw nothing.
  cam_msg = {"LINE_VISIBLE": 0, "LINE_NOT_VISIBLE": 1, "LANE_LINES": 1, "BIT1": 1,
             "BIT2": 0, "BIT3": 0, "NO_ERR_BIT": 0, "ERR_BIT": 0,
             "TJA": 0, "TJA_TRANSITION": 0, "S1": 0, "S1_HBEAM": 0}
  dat = mazdacan.create_alert_command(packer, cam_msg, ldw=False, steer_required=steer_required)[1]
  assert hands_code(dat) == ((0b111, 1, 1) if steer_required else (0, 0, 0))
  out = parse_frame(CAM_LANEINFO, dat)
  assert out["LDW_WARN_LL"] == 0 and out["LDW_WARN_RL"] == 0


def test_buttons_never_carry_the_tja_bit(packer):
  # never pressed by openpilot on either bus: on the car's side it toggles MADS and arms MRCC,
  # on the camera's side it switches the car's own lane-keep setting off
  for button in (Buttons.CANCEL, Buttons.RESUME, Buttons.SET_PLUS, Buttons.SET_MINUS):
    _, dat, bus = mazdacan.create_button_cmd(packer, car_params(), 3, button)
    assert bus == 0 and not dat[1] & 0x08


@pytest.mark.parametrize("counter", range(16))
def test_crz_info_standby_matches_stock(packer, counter):
  checksum = (0x5d - counter) & 0xff
  expected = f"01ffe3ffc000{counter:02x}{checksum:02x}"
  dat = mazdacan.create_acc_command(packer, 0, counter, 0.0, long_active=False, acc_available=False)[1]
  assert dat.hex() == expected


@pytest.mark.parametrize("brake_pressed, byte4, base", [(False, 0xc4, 0xd9), (True, 0xc0, 0xdd)])
@pytest.mark.parametrize("counter", range(16))
def test_crz_info_armed_idle_matches_stock(packer, brake_pressed, byte4, base, counter):
  # Armed idle uses stock's raw 8190 sentinel and gates ACC_SET_ALLOWED with the brake.
  checksum = (base - counter) & 0xff
  expected = f"01ffe3ff{byte4:02x}80{counter:02x}{checksum:02x}"
  dat = mazdacan.create_acc_command(packer, 0, counter, 0.0, long_active=False, acc_available=True,
                                    brake_pressed=brake_pressed)[1]
  assert dat.hex() == expected


@pytest.mark.parametrize("accel, stopping, unlatching, counter, expected", [
  (0.0, False, False, 0, "01ffe20006800097"),     # engaged, zero command
  (2.0, False, False, 3, "01ffe30006800393"),     # ISO max accel, raw 2048
  (-3.5, False, False, 7, "01ffe04006800752"),    # ISO max brake, raw -3584
  (-1.0, True, False, 5, "01ffe18006841503"),     # standstill hold, raw -1024 + stop bits
  (-1 / 1024, False, False, 9, "01ffe1ffe68009b0"),  # latched hold, raw -1
  (0.0, False, True, 11, "01ffe20006804b8c"),     # resume unlatch pulse
  # The unlatch bit is excluded, so this checksum matches the counter-zero hold frame.
  (-1 / 1024, False, True, 0, "01ffe1ffe68040b9"),
])
def test_crz_info_engaged_golden_bytes(packer, accel, stopping, unlatching, counter, expected):
  dat = mazdacan.create_acc_command(packer, 0, counter, accel, long_active=True, acc_available=False,
                                    stopping=stopping, resume_unlatching=unlatching)[1]
  assert dat.hex() == expected


@pytest.mark.parametrize("stopping, unlatching", [(False, False), (True, False), (False, True)])
def test_crz_info_accel_encoding_and_checksum(packer, stopping, unlatching):
  # the packed command must round-trip at the 1/1024 factor and carry a valid masked-bit
  # checksum over the whole command window, stop bits set or not
  for raw in range(-3584, 2049, 137):
    dat = mazdacan.create_acc_command(packer, 0, raw % 16, raw / 1024.0, long_active=True, acc_available=False,
                                      stopping=stopping, resume_unlatching=unlatching)[1]
    assert decode_accel_cmd_raw(dat) == raw
    assert dat[7] == crz_info_reference_checksum(dat)
    assert bool(dat[5] & 0x04) == stopping
    assert bool(dat[6] & 0x10) == stopping
    assert bool(dat[6] & 0x40) == unlatching
    # the excluded event bits must not move the checksum: stripping them yields the
    # same byte a bare frame carries (stock 0b: 8040b9 vs 8000b9, both chk b9)
    bare = bytearray(dat)
    bare[5] &= ~0x04
    bare[6] &= ~0x40
    assert dat[7] == crz_info_reference_checksum(bytes(bare))


@pytest.mark.parametrize("long_active, acc_available, gap, lead_distance, acc_active_2, expected", [
  (False, False, 0, 0, False, "0201010000000000"),  # standby
  (False, True, 2, 0, False, "02010b0000000000"),   # MRCC armed, SET allowed
  (False, True, 2, 2, False, "02018b4000000000"),   # MRCC armed, lead ahead
  (True, True, 2, 0, True, "0a010b0000001000"),     # engaged, no lead
  (True, True, 2, 2, True, "0a018b4000001000"),     # engaged, following a lead
  (True, True, 2, 3, True, "0a018b6000001000"),     # stopped behind a lead
  (True, True, 2, 3, False, "0a018b6000000000"),    # relaxed hold, ACC_ACTIVE_2 drops
  (True, True, 1, 2, True, "0a01874000001000"),     # wire gap 1 (4 bars on the cluster)
])
def test_crz_ctrl_golden_bytes(packer, long_active, acc_available, gap, lead_distance, acc_active_2, expected):
  dat = mazdacan.create_crz_ctrl(packer, 0, long_active, acc_available, gap, acc_active_2,
                                 lead_distance=lead_distance)[1]
  assert dat.hex() == expected


@pytest.mark.parametrize("long_active, acc_available, gap, acc_active_2, expected", [
  (False, False, 0, False, "0221010000000000"),  # standby: the stock radar's frame
  (False, True, 2, False, "02210b0000000000"),   # MRCC armed
  (True, True, 2, True, "0a210b0000001000"),     # engaged
])
def test_crz_ctrl_relays_hbc_arming(packer, long_active, acc_available, gap, acc_active_2, expected):
  dat = mazdacan.create_crz_ctrl(packer, 0, long_active, acc_available, gap, acc_active_2, hbc_request=True)[1]
  assert dat.hex() == expected
  bare = mazdacan.create_crz_ctrl(packer, 0, long_active, acc_available, gap, acc_active_2)[1]
  assert bytes(a ^ b for a, b in zip(dat, bare, strict=True)) == bytes([0, 0x20, 0, 0, 0, 0, 0, 0])


def test_radar_frames_match_stock():
  expected = [
    (0x499, "0008c00000000000"),
    (0x361, "fff7fefe1fc00080"),
    (0x362, "fff7fefe1fc78c80"),
    (0x363, "fff7fefe1fc00000"),
    (0x364, "fff7fefe1fc00000"),
    (0x365, "fff7fe7ffbff3fc0"),
    (0x366, "fff7fe7ffbff3fc0"),
  ]
  for bus in (0, 2):
    frames = mazdacan.create_radar_frames(bus, 0)
    assert [(f.address, f.dat.hex(), f.src) for f in frames] == [(a, d, bus) for a, d in expected]


def test_radar_frames_stamp_only_the_counter():
  frames = mazdacan.create_radar_frames(2, 15)
  base = mazdacan.create_radar_frames(2, 0)
  assert frames[0] == base[0], "the static frame carries no counter"
  for f, b in zip(frames[1:], base[1:], strict=True):
    assert f.dat[7] & 0x0f == 15
    assert f.dat[:7] + bytes([f.dat[7] & 0xf0]) == b.dat


def test_g46l_radar_frames_are_the_static_capture_alone():
  frames = mazdacan.create_radar_frames(0, 15, g46l=True)
  assert [(f.address, f.dat.hex(), f.src) for f in frames] == [(0x499, "0098400000000000", 0)]
