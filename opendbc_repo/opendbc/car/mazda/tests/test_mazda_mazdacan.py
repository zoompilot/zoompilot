"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The message builders in mazdacan. The synthetic CRZ_INFO / CRZ_CTRL / radar frames must
reproduce stock captures byte for byte; the hex values below come from real radar traffic.
"""
import pytest

from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.tests.conftest import CAM_LANEINFO, LEAD_TRACK, parse_frame


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
  (2.0, False, False, 3, "01ffe2fa0680039a"),     # ISO max accel, raw 2000
  (-3.5, False, False, 7, "01ffe04a868007c8"),    # ISO max brake, raw -3500
  (-1.024, True, False, 5, "01ffe18006841503"),   # standstill hold, raw -1024 + stop bits
  (-0.001, False, False, 9, "01ffe1ffe68009b0"),  # latched hold, raw -1
  (0.0, False, True, 11, "01ffe20006804b8c"),     # resume unlatch pulse
  # The unlatch bit is excluded, so this checksum matches the counter-zero hold frame.
  (-0.001, False, True, 0, "01ffe1ffe68040b9"),
])
def test_crz_info_engaged_golden_bytes(packer, accel, stopping, unlatching, counter, expected):
  dat = mazdacan.create_acc_command(packer, 0, counter, accel, long_active=True, acc_available=False,
                                    stopping=stopping, resume_unlatching=unlatching)[1]
  assert dat.hex() == expected


@pytest.mark.parametrize("stopping, unlatching", [(False, False), (True, False), (False, True)])
def test_crz_info_accel_encoding_and_checksum(packer, stopping, unlatching):
  # the packed command must round-trip at the 0.001 factor and carry a valid masked-bit
  # checksum over the whole command window, stop bits set or not
  for raw in range(-3500, 2001, 137):
    dat = mazdacan.create_acc_command(packer, 0, raw % 16, raw / 1000.0, long_active=True, acc_available=False,
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


@pytest.mark.parametrize("long_active, acc_available, gap, has_lead, phase, acc_active_2, expected", [
  (False, False, 0, False, 0, False, "0201010000000000"),  # standby
  (False, True, 2, False, 0, False, "02010b0000000000"),   # MRCC armed, SET allowed
  (True, True, 2, True, 1, True, "0a018b2000001000"),      # engaged, cruise, no lead
  (True, True, 2, True, 2, True, "0a018b4000001000"),      # engaged, following a lead
  (True, True, 2, True, 3, True, "0a018b6000001000"),      # stop-and-go hold (near phase)
  (True, True, 2, True, 4, True, "0a018b8000001000"),      # stop-and-go hold (far phase)
  (True, True, 2, True, 3, False, "0a018b6000000000"),     # relaxed hold, ACC_ACTIVE_2 drops
  (True, True, 1, True, 2, True, "0a01874000001000"),      # driver gap 1 mirrored to the dash
])
def test_crz_ctrl_golden_bytes(packer, long_active, acc_available, gap, has_lead, phase, acc_active_2, expected):
  dat = mazdacan.create_crz_ctrl(packer, 0, long_active, acc_available, gap, has_lead, phase, acc_active_2)[1]
  assert dat.hex() == expected


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
  frames = mazdacan.create_radar_frames(0, 0, None)
  assert [(f.address, f.dat.hex()) for f in frames] == expected


def test_radar_frames_counter_and_lead_track():
  frames = mazdacan.create_radar_frames(2, 15, (10.25, 0.))
  assert all(f.src == 2 for f in frames)
  # counter stamps the low nibble of the last byte on every track
  assert [f.dat[7] & 0x0f for f in frames[1:]] == [15] * 6
  tracks = {f.address: f.dat.hex() for f in frames}
  assert tracks[0x364] == "0a4e00001c00000f"


def test_lead_track_constant_bytes_match_the_stock_release_capture():
  # the template's measurement fields are zeroed, so a zero-range lead reproduces it exactly
  assert mazdacan.create_lead_track(0., 0.) == mazdacan.LEAD_TRACK_TEMPLATE
  # Preserve the camera's occupied-slot status bytes.
  assert mazdacan.LEAD_TRACK_TEMPLATE[4] & 0x1f == 0x1c
  assert mazdacan.LEAD_TRACK_TEMPLATE[5] == 0x00


@pytest.mark.parametrize("d_rel, v_rel", [
  (0., 0.), (6.5, 1.5), (10.25, -2.0), (29.4, 2.9375), (255.875, 63.9375), (400., 100.), (5., -80.),
])
def test_lead_track_round_trips_through_the_dbc(d_rel, v_rel):
  dat = mazdacan.create_lead_track(d_rel, v_rel)
  vl = parse_frame(LEAD_TRACK, dat)
  assert vl["DIST_OBJ"] == pytest.approx(min(max(d_rel, 0.), 255.875), abs=0.0625)
  assert vl["RELV_OBJ"] == pytest.approx(min(max(v_rel, -64.), 63.9375), abs=0.0625)
  # the bits outside the two fields we drive stay exactly as captured
  assert dat[1] & 0x0f == mazdacan.LEAD_TRACK_TEMPLATE[1] & 0x0f
  assert dat[2] == mazdacan.LEAD_TRACK_TEMPLATE[2]
  assert dat[4] & 0x1f == mazdacan.LEAD_TRACK_TEMPLATE[4] & 0x1f
  assert dat[5:] == mazdacan.LEAD_TRACK_TEMPLATE[5:]
