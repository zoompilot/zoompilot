"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Radar track parsing: empty-slot detection requires ALL THREE sentinel fields.
Each sentinel is also a reachable real value on its own grid (relv -1.0 m/s is
an ordinary closing speed), so any-single-match dropping deletes a live track
and resets radard's Kalman state whenever a lead crosses that value. Measured
over 29k track frames (route 0b): sentinels occur either all-three (empty slot)
or not at all, except real tracks at exactly -1.0 m/s.
"""
import math

import pytest

from opendbc.car.mazda.radar_interface import (RadarInterface, RADAR_TRACK_ADDRS, RADAR_USABLE_ADDRS,
                                               SENTINEL_DIST, SENTINEL_ANG, SENTINEL_RELV)
from opendbc.car.mazda.tests.conftest import car_params, car_params_sp, packer
from opendbc.car.mazda.values import CAR

PACKER = packer()


@pytest.fixture
def ri() -> RadarInterface:
  # stock-long: under alpha long the stock radar is torn down and radarUnavailable is True
  CP = car_params(CAR.MAZDA_CX5_2022, alpha_long=False)
  return RadarInterface(CP, car_params_sp(CP, alpha_long=False))


def track_msg(addr, dist=SENTINEL_DIST, ang=SENTINEL_ANG, relv=SENTINEL_RELV):
  """Pack one track frame by signal name (physical values), defaulting to an empty slot."""
  return PACKER.make_can_msg(addr, 0, {"DIST_OBJ": dist, "ANG_OBJ": ang, "RELV_OBJ": relv})


def burst(ri, t_ns, overrides=None):
  """Feed one full 0x361-0x366 burst (empty slots unless overridden) and return the RadarData."""
  overrides = overrides or {}
  msgs = [overrides.get(addr, track_msg(addr)) for addr in RADAR_TRACK_ADDRS]
  rr = ri.update([(t_ns, msgs)])
  assert rr is not None, "a full burst ending in the trigger msg must produce a RadarData"
  return rr


def test_empty_slots_produce_no_points(ri):
  assert len(burst(ri, 0).points) == 0


def test_real_track_parses(ri):
  rr = burst(ri, 0, {0x361: track_msg(0x361, dist=40.0, ang=2.0, relv=-5.0)})
  assert len(rr.points) == 1
  pt = rr.points[0]
  assert pt.dRel == pytest.approx(math.cos(math.radians(2.0)) * 40.0, rel=1e-6)
  assert pt.vRel == -5.0


def test_track_at_exactly_minus_one_mps_is_kept(ri):
  """-1.0 m/s with a real distance is a live lead, not an empty slot. Dropping it
  would delete and re-create the track each time the closing speed crosses -1.0,
  resetting radard's Kalman state mid-follow."""
  rr = burst(ri, 0, {0x361: track_msg(0x361, dist=40.0, ang=0.0, relv=-1.0625)})
  track_id = rr.points[0].trackId

  # lead decelerates through exactly -1.0 m/s: the track must survive with its identity
  rr = burst(ri, int(0.1e9), {0x361: track_msg(0x361, dist=39.875, ang=0.0, relv=SENTINEL_RELV)})
  assert len(rr.points) == 1
  assert rr.points[0].vRel == SENTINEL_RELV
  assert rr.points[0].trackId == track_id

  rr = burst(ri, int(0.2e9), {0x361: track_msg(0x361, dist=39.75, ang=0.0, relv=-0.9375)})
  assert rr.points[0].trackId == track_id


def test_track_at_max_range_is_kept(ri):
  rr = burst(ri, 0, {0x362: track_msg(0x362, dist=SENTINEL_DIST, ang=0.0, relv=-10.0)})
  assert len(rr.points) == 1
  assert rr.points[0].dRel == SENTINEL_DIST


def test_track_at_sentinel_angle_is_kept(ri):
  rr = burst(ri, 0, {0x363: track_msg(0x363, dist=40.0, ang=SENTINEL_ANG, relv=-5.0)})
  assert len(rr.points) == 1
  assert rr.points[0].yRel == pytest.approx(-math.sin(math.radians(SENTINEL_ANG)) * 40.0, rel=1e-6)


def test_all_sentinel_slot_deletes_a_prior_track(ri):
  burst(ri, 0, {0x361: track_msg(0x361, dist=40.0, ang=0.0, relv=-5.0)})
  rr = burst(ri, int(0.1e9))  # slot empties: all three sentinels
  assert len(rr.points) == 0


def test_undecoded_relv_addrs_never_produce_points(ri):
  overrides = {addr: track_msg(addr, dist=40.0, ang=0.0, relv=-5.0)
               for addr in RADAR_TRACK_ADDRS if addr not in RADAR_USABLE_ADDRS}
  assert len(burst(ri, 0, overrides).points) == 0
