"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.values import CAR, MazdaFlags
from opendbc.car.structs import CarParams
from opendbc.sunnypilot.car.interfaces import (get_speed_dep_config, get_speed_dep_config_for_car, get_steer_max_schedule,
                                             get_steer_rail_schedule, get_steer_slew_schedule)

CX5_2022_SCHEDULE = ([0.0, 14.2, 14.5], [1200.0, 1200.0, 800.0])


def cx5_2022_cp() -> CarParams:
  # the real CarParams: the 2022 EPS flag and minSteerSpeed 0 come from the interface
  cp = CarInterface.get_params(CAR.MAZDA_CX5_2022, gen_empty_fingerprint(), [], alpha_long=False, is_release=False, docs=False)
  assert cp.flags & MazdaFlags.STEER_TO_ZERO_EPS and cp.minSteerSpeed == 0.0
  return cp


def brand_cp(brand: str, fingerprint: str = "", min_steer_speed: float = 0.0) -> CarParams:
  # a minimal CarParams for the brands the schedule lookup has to decline
  cp = CarParams()
  cp.brand = brand
  cp.carFingerprint = fingerprint
  cp.minSteerSpeed = min_steer_speed
  return cp


# stock Mazda EPS: minSteerSpeed above zero, so requires_steer_to_zero suppresses the entry
STOCK_MAZDA = dict(brand="mazda", fingerprint="MAZDA_CX9_2021", min_steer_speed=20.0)


class TestSteerMaxSchedule:
  def test_mazda_steer_to_zero_returns_lookup(self):
    assert get_steer_max_schedule(cx5_2022_cp()) == CX5_2022_SCHEDULE

  @pytest.mark.parametrize("cp_kwargs", [
    dict(brand="toyota", fingerprint="TOYOTA_RAV4_TSS2"),  # flat steer max
    dict(brand="notabrand"),
  ], ids=["flat_steer_max_brand", "unknown_brand"])
  def test_brands_without_a_schedule_return_none(self, cp_kwargs):
    assert get_steer_max_schedule(brand_cp(**cp_kwargs)) is None

  def test_schedule_attached_to_active_entry(self):
    cfg = get_speed_dep_config_for_car(cx5_2022_cp())
    assert cfg['steer_max_schedule'] == CX5_2022_SCHEDULE
    # the schedule's step must sit inside a bin span, or per-count interp cannot place it
    assert cfg['speed_bp'][2] < 14.2
    assert 14.5 < cfg['speed_bp'][3]

  def test_seed_version_is_a_non_negative_int(self):
    """Every entry's seed_version (0 when absent) is an int the cache field can carry."""
    for name, entry in get_speed_dep_config().items():
      v = entry.get('seed_version', 0)
      assert isinstance(v, int) and not isinstance(v, bool) and 0 <= v < 2**31, name

  def test_inactive_entry_stays_empty(self):
    assert get_speed_dep_config_for_car(brand_cp(**STOCK_MAZDA)) == {}

  def test_config_copy_not_cached_dict(self):
    a = get_speed_dep_config_for_car(cx5_2022_cp())
    a['steer_max_schedule'] = 'mutated'
    assert get_speed_dep_config_for_car(cx5_2022_cp())['steer_max_schedule'] != 'mutated'


class TestSteerRailSchedule:
  def test_mazda_steer_to_zero_rail(self):
    bp, rail = get_steer_rail_schedule(cx5_2022_cp())
    assert all(0.0 < r <= 1.0 for r in rail)
    # the rail bottoms out just below the cliff (648/1200) and recovers above it (620/800)
    assert rail[bp.index(14.2)] == min(rail)
    assert rail[bp.index(14.2)] == pytest.approx(0.54, abs=0.01)
    assert rail[bp.index(14.5)] == pytest.approx(620.0 / 800.0, abs=1e-6)

  @pytest.mark.parametrize("cp_kwargs", [
    dict(brand="mazda", min_steer_speed=20.0),  # stock EPS params have no ceiling lookup
    dict(brand="toyota"),
  ], ids=["stock_mazda_eps", "toyota"])
  def test_no_ceiling_returns_none(self, cp_kwargs):
    assert get_steer_rail_schedule(brand_cp(**cp_kwargs)) is None


class TestSteerSlewSchedule:
  def test_mazda_steer_to_zero_slew_steps_at_the_cliff(self):
    bp, up, down = get_steer_slew_schedule(cx5_2022_cp())
    assert bp == [0.0, 14.2, 14.5]
    # 12 counts/frame both ways: 0.01 of scale below the cliff, 0.015 above
    assert up == [12.0 / 1200.0, 12.0 / 1200.0, 12.0 / 800.0]
    assert down == up

  def test_legacy_mazda_flat_scale(self):
    # stock EPS params: 10 up, 25 down over a flat 800
    bp, up, down = get_steer_slew_schedule(brand_cp(brand="mazda", min_steer_speed=20.0))
    assert bp == [0.0]
    assert up == [10.0 / 800.0]
    assert down == [25.0 / 800.0]

  @pytest.mark.parametrize("brand", ["tesla", "notabrand"])  # angle steering has no STEER_DELTA_UP/DOWN
  def test_brand_without_rate_limits_returns_none(self, brand):
    assert get_steer_slew_schedule(brand_cp(brand=brand)) is None
