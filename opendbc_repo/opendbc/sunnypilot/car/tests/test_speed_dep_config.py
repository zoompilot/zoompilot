"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.tests.conftest import eps_fw
from opendbc.car.mazda.values import CAR, MazdaFlags
from opendbc.car.structs import CarParams
from opendbc.sunnypilot.car.lateral_tune import (get_speed_dep_config, get_speed_dep_config_for_car, get_steer_rail_schedule,
                                               get_steer_slew_schedule, get_tune_scale)


def cx5_2022_cp() -> CarParams:
  # the real CarParams: the 2022 EPS flag and minSteerSpeed 0 come from the interface
  cp = CarInterface.get_params(CAR.MAZDA_CX5_2022, gen_empty_fingerprint(), [], alpha_long=False, is_release=False, docs=False)
  assert cp.flags & MazdaFlags.STEER_TO_ZERO_EPS and cp.minSteerSpeed == 0.0
  return cp


def swapped_cp(platform=CAR.MAZDA_CX5_KE) -> CarParams:
  # a chassis behind the 2022 CX-5 EPS the seed bins were learned under (the 2016.5 KE report)
  fw = eps_fw(b'KSD5-3210X-C-00\x00\x00\x00\x00\x00\x00\x00\x00\x00')
  cp = CarInterface.get_params(platform, gen_empty_fingerprint(), fw, alpha_long=False, is_release=False, docs=False)
  assert cp.flags & MazdaFlags.STEER_TO_ZERO_EPS and cp.minSteerSpeed == 0.0
  return cp


def brand_cp(brand: str, fingerprint: str = "", min_steer_speed: float = 0.0) -> CarParams:
  # a minimal CarParams for the brands the schedule lookup has to decline
  cp = CarParams()
  cp.brand = brand
  cp.carFingerprint = fingerprint
  cp.minSteerSpeed = min_steer_speed
  return cp


# stock CX-9 2021 EPS: a 45 kph floor on the same hardware, so the entry keeps only the bins above it
STOCK_MAZDA = dict(brand="mazda", fingerprint="MAZDA_CX9_2021", min_steer_speed=20.0)


def legacy_fw_cp() -> CarParams:
  # the 2022 EPS hardware on firmware with the 45 kph floor: hardware envelope, no steer-to-zero
  fw = eps_fw(b'K319-3210X-B-00' + b'\x00' * 9)
  cp = CarInterface.get_params(CAR.MAZDA_CX5_2022, gen_empty_fingerprint(), fw, alpha_long=False, is_release=False, docs=False)
  assert cp.flags & MazdaFlags.LEGACY_FW_EPS and cp.minSteerSpeed > 0
  return cp


class TestSpeedDepConfig:
  def test_entries_are_consistent(self):
    for name, entry in get_speed_dep_config().items():
      if 'speed_bp' not in entry:
        continue
      assert entry['speed_bp'] == sorted(entry['speed_bp']), name
      assert len(entry['laf_bp']) == len(entry['speed_bp']) == len(entry['friction_bp']), name

  def test_seed_version_is_a_non_negative_int(self):
    """Every entry's seed_version (0 when absent) is an int the cache field can carry."""
    for name, entry in get_speed_dep_config().items():
      v = entry.get('seed_version', 0)
      assert isinstance(v, int) and not isinstance(v, bool) and 0 <= v < 2**31, name

  def test_stock_cx9_keeps_only_the_bins_above_its_floor(self):
    cfg = get_speed_dep_config_for_car(brand_cp(**STOCK_MAZDA))
    full = get_speed_dep_config()['MAZDA_CX9_2021']
    assert cfg['speed_bp'] == [v for v in full['speed_bp'] if v >= 20.0]
    assert len(cfg['laf_bp']) == len(cfg['speed_bp']) == len(cfg['friction_bp'])

  def test_flagged_entry_stays_empty_behind_a_floor(self, monkeypatch):
    import opendbc.sunnypilot.car.lateral_tune as mod
    entry = {'requires_steer_to_zero': True, 'speed_bp': [30.0]}
    monkeypatch.setattr(mod, 'get_speed_dep_config', lambda: {'MAZDA_CX9_2021': entry})
    assert get_speed_dep_config_for_car(brand_cp(**STOCK_MAZDA)) == {}
    assert get_speed_dep_config_for_car(brand_cp(brand="mazda", fingerprint="MAZDA_CX9_2021")) == entry

  def test_entry_with_every_bin_below_the_floor_keeps_its_other_keys(self, monkeypatch):
    # the car falls back to the default bins, still under the entry's seed_version
    import opendbc.sunnypilot.car.lateral_tune as mod
    monkeypatch.setattr(mod, 'get_speed_dep_config', lambda: {'MAZDA_CX9_2021': {'speed_bp': [10.0], 'laf_bp': [1.0], 'seed_version': 3}})
    assert get_speed_dep_config_for_car(brand_cp(**STOCK_MAZDA)) == {'seed_version': 3}

  SWAP_CHASSIS = [CAR.MAZDA_CX5_KE, CAR.MAZDA_CX5, CAR.MAZDA_CX9, CAR.MAZDA_3, CAR.MAZDA_6]

  @pytest.mark.parametrize("platform", SWAP_CHASSIS)
  def test_swapped_chassis_take_the_donor_table(self, platform):
    # the donor EPS is the one the CX-5 2022 bins were learned behind
    cfg = get_speed_dep_config_for_car(swapped_cp(platform))
    cx5_2022 = get_speed_dep_config_for_car(cx5_2022_cp())
    for key in ('speed_bp', 'laf_bp', 'friction_bp', 'seed_version'):
      assert cfg[key] == cx5_2022[key], key

  @pytest.mark.parametrize("platform", SWAP_CHASSIS)
  def test_swapped_chassis_entries_withheld_on_the_stock_eps(self, platform):
    assert get_speed_dep_config_for_car(brand_cp(brand="mazda", fingerprint=str(platform), min_steer_speed=20.0)) == {}

  def test_config_copy_not_cached_dict(self):
    a = get_speed_dep_config_for_car(cx5_2022_cp())
    a['speed_bp'] = 'mutated'
    assert get_speed_dep_config_for_car(cx5_2022_cp())['speed_bp'] != 'mutated'


class TestSteerRailSchedule:
  def test_mazda_steer_to_zero_rail(self):
    bp, rail = get_steer_rail_schedule(cx5_2022_cp())
    assert all(0.0 < r <= 1.0 for r in rail)
    # the ceiling over the one 1200-count scale: monotone, no cliff to interpolate across
    assert rail == sorted(rail, reverse=True)
    assert rail[0] == pytest.approx(1148.0 / 1200.0)
    assert rail[-1] == pytest.approx(620.0 / 1200.0)

  def test_no_ceiling_returns_none(self):
    assert get_steer_rail_schedule(brand_cp(brand="toyota")) is None


class TestSteerSlewSchedule:
  def test_mazda_steer_to_zero_slew(self):
    # 10 counts/frame both ways over the flat 1200
    assert get_steer_slew_schedule(cx5_2022_cp()) == ([0.0], [10.0 / 1200.0], [10.0 / 1200.0])

  def test_legacy_mazda_slew(self):
    # legacy firmware on the same EPS hardware: the same 10 counts/frame over the flat 1200
    assert get_steer_slew_schedule(legacy_fw_cp()) == ([0.0], [10.0 / 1200.0], [10.0 / 1200.0])

  @pytest.mark.parametrize("brand", ["tesla", "notabrand"])  # angle steering has no STEER_DELTA_UP/DOWN
  def test_brand_without_rate_limits_returns_none(self, brand):
    assert get_steer_slew_schedule(brand_cp(brand=brand)) is None


class TestLegacyFirmwareEntry:
  def test_bins_below_the_floor_are_dropped(self):
    full = get_speed_dep_config()['MAZDA_CX5_2022']
    cp = legacy_fw_cp()
    cfg = get_speed_dep_config_for_car(cp)
    keep = [i for i, v in enumerate(full['speed_bp']) if v >= cp.minSteerSpeed]
    assert 0 < len(keep) < len(full['speed_bp'])
    assert cfg['speed_bp'] == [full['speed_bp'][i] for i in keep]
    assert cfg['laf_bp'] == [full['laf_bp'][i] for i in keep]
    assert cfg['friction_bp'] == [full['friction_bp'][i] for i in keep]
    # every surviving bin sits above the firmware's 45-52 kph dead band
    assert min(cfg['speed_bp']) > 52 / 3.6
    assert cfg.get('seed_version', 0) == full.get('seed_version', 0)

  def test_first_kept_bin_keeps_its_lower_edge(self):
    # without it the first kept bin would reach down to the default 5 m/s floor, over the
    # floor and the dead band, where the EPS takes no torque
    full = get_speed_dep_config()['MAZDA_CX5_2022']['speed_bp']
    cfg = get_speed_dep_config_for_car(legacy_fw_cp())
    i = full.index(cfg['speed_bp'][0])
    assert cfg['min_speed'] == pytest.approx((full[i - 1] + full[i]) / 2)
    # the steer-to-zero car keeps the table's own edge, if it sets one
    assert get_speed_dep_config_for_car(cx5_2022_cp()).get('min_speed') == get_speed_dep_config()['MAZDA_CX5_2022'].get('min_speed')

  def test_legacy_firmware_shares_the_rail_and_slew(self):
    cp, stz = legacy_fw_cp(), cx5_2022_cp()
    assert get_steer_rail_schedule(cp) == get_steer_rail_schedule(stz)
    assert get_steer_slew_schedule(cp) == get_steer_slew_schedule(stz)

  def test_steer_to_zero_entry_is_untouched(self):
    cfg = get_speed_dep_config_for_car(cx5_2022_cp())
    assert cfg['speed_bp'] == get_speed_dep_config()['MAZDA_CX5_2022']['speed_bp']


class TestTuneScale:
  @pytest.mark.parametrize("make_cp", [cx5_2022_cp, legacy_fw_cp, swapped_cp], ids=["cx5_2022", "legacy_fw", "eps_swap"])
  def test_mazda_eps_hardware_is_one_and_a_half_upstreams_scale(self, make_cp):
    assert get_tune_scale(make_cp()) == 1.5

  @pytest.mark.parametrize("cp_kwargs", [
    dict(brand="toyota", fingerprint="TOYOTA_RAV4_TSS2"),
    dict(brand="notabrand"),
  ], ids=["no_tune_scale_brand", "unknown_brand"])
  def test_other_scales_are_one(self, cp_kwargs):
    assert get_tune_scale(brand_cp(**cp_kwargs)) == 1.0
