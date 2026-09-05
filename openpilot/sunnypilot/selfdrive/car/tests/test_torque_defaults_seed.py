#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Tests for the one-time steer-to-zero Mazda torque-control default seeding.
"""

from opendbc.car.mazda.values import MazdaFlags
from opendbc.car.structs import car

from openpilot.sunnypilot.selfdrive.car.interfaces import (MAZDA_STEER_TO_ZERO_TORQUE_TUNE, _seed_mazda_torque_defaults,
                                                          seed_car_defaults_offroad)

CarParams = car.CarParams

SEEDED_KEYS = ("EnforceTorqueControl", "LiveTorqueParamsToggle", "SpeedDependentTorqueToggle")


class FakeParams:
  """Minimal dict-backed Params stand-in (avoids the stale on-disk params_pyx for new keys)."""
  def __init__(self, initial=None):
    self._store = dict(initial or {})

  def get_bool(self, key):
    return bool(self._store.get(key, False))

  def put_bool(self, key, val):
    self._store[key] = bool(val)

  def get(self, key):
    return self._store.get(key)

  def put(self, key, val, block=False):
    # TorqueControlTune is a FLOAT param: the real Params.put rejects a str for it
    assert not isinstance(val, str), f"{key}: str put into a FLOAT param"
    self._store[key] = val


def _cx5_eps_cp():
  # the 2022+ CX-5 EPS, flagged by the interface (also set on EPS-swapped cars)
  return CarParams(brand="mazda", flags=int(MazdaFlags.STEER_TO_ZERO_EPS))


def _pre_2022_mazda_cp():
  # stock pre-2022 EPS: no flag, low-speed lockout
  return CarParams(brand="mazda", minSteerSpeed=12.5)


def _non_mazda_cp():
  # the flag bit alone must not seed another brand
  return CarParams(brand="toyota", flags=int(MazdaFlags.STEER_TO_ZERO_EPS))


class TestMazdaTorqueDefaultsSeed:
  def test_steer_to_zero_mazda_gets_defaults(self):
    params = FakeParams()
    _seed_mazda_torque_defaults(_cx5_eps_cp(), params)
    for key in SEEDED_KEYS:
      assert params.get_bool(key) is True
    assert params.get_bool("MazdaTorqueDefaultsApplied") is True

  def test_pre_2022_mazda_not_seeded(self):
    params = FakeParams()
    _seed_mazda_torque_defaults(_pre_2022_mazda_cp(), params)
    for key in SEEDED_KEYS:
      assert params.get_bool(key) is False
    assert params.get_bool("MazdaTorqueDefaultsApplied") is False

  def test_non_mazda_not_seeded(self):
    params = FakeParams()
    _seed_mazda_torque_defaults(_non_mazda_cp(), params)
    for key in SEEDED_KEYS:
      assert params.get_bool(key) is False
    assert params.get_bool("MazdaTorqueDefaultsApplied") is False

  def test_idempotent_respects_user_override(self):
    # Already applied once, and the user has since turned the toggles back off.
    params = FakeParams({"MazdaTorqueDefaultsApplied": True})
    _seed_mazda_torque_defaults(_cx5_eps_cp(), params)
    for key in SEEDED_KEYS:
      assert params.get_bool(key) is False  # not re-seeded


class TestMazdaTorqueTuneSeed:
  """TorqueControlTune keeps upstream's 0.0 default in params_keys.h, and manager_init writes
  that default to disk at boot, so the seed cannot key on "unset". It keys on the
  MazdaTorqueTuneSeeded marker instead, independent of the toggle marker."""

  def test_unset_tune_is_seeded_to_v2(self):
    params = FakeParams()
    _seed_mazda_torque_defaults(_cx5_eps_cp(), params)
    assert params.get("TorqueControlTune") == 2.0
    assert params.get("MazdaTorqueTuneSeeded") == MAZDA_STEER_TO_ZERO_TORQUE_TUNE

  def test_manager_default_on_disk_is_overridden(self):
    # what every device looks like when card first runs: manager_init already wrote the 0.0 default
    params = FakeParams({"TorqueControlTune": 0.0})
    _seed_mazda_torque_defaults(_cx5_eps_cp(), params)
    assert params.get("TorqueControlTune") == 2.0

  def test_already_marked_device_still_gets_the_tune(self):
    # seeded before the tune was part of the seed: the toggle marker is set, the tune is not
    params = FakeParams({"MazdaTorqueDefaultsApplied": True, "TorqueControlTune": 0.0})
    _seed_mazda_torque_defaults(_cx5_eps_cp(), params)
    assert params.get("TorqueControlTune") == 2.0
    for key in SEEDED_KEYS:
      assert params.get_bool(key) is False  # toggles still not re-seeded

  def test_choice_made_after_the_seed_is_kept(self):
    params = FakeParams({"TorqueControlTune": 0.0, "MazdaTorqueTuneSeeded": MAZDA_STEER_TO_ZERO_TORQUE_TUNE})
    _seed_mazda_torque_defaults(_cx5_eps_cp(), params)
    assert params.get("TorqueControlTune") == 0.0

  def test_bumped_default_reseeds_once(self):
    # a marker from an older seeded version moves the device to the new default, once
    params = FakeParams({"TorqueControlTune": 1.0, "MazdaTorqueTuneSeeded": 1.0})
    _seed_mazda_torque_defaults(_cx5_eps_cp(), params)
    assert params.get("TorqueControlTune") == 2.0
    assert params.get("MazdaTorqueTuneSeeded") == 2.0
    params.put("TorqueControlTune", 0.0)
    _seed_mazda_torque_defaults(_cx5_eps_cp(), params)
    assert params.get("TorqueControlTune") == 0.0

  def test_other_cars_are_not_seeded(self):
    for cp in (_pre_2022_mazda_cp(), _non_mazda_cp()):
      params = FakeParams()
      _seed_mazda_torque_defaults(cp, params)
      assert params.get("TorqueControlTune") is None
      assert params.get("MazdaTorqueTuneSeeded") is None


class TestOffroadSeed:
  """manager_init runs the seed from CarParamsPersistent so an offroad update shows v2 at once."""

  def test_seeds_from_persistent_carparams(self):
    params = FakeParams({"CarParamsPersistent": _cx5_eps_cp().to_bytes(), "TorqueControlTune": 0.0})
    seed_car_defaults_offroad(params)
    assert params.get("TorqueControlTune") == 2.0
    for key in SEEDED_KEYS:
      assert params.get_bool(key) is True

  def test_never_driven_device_is_left_to_card(self):
    params = FakeParams({"TorqueControlTune": 0.0})
    seed_car_defaults_offroad(params)
    assert params.get("TorqueControlTune") == 0.0
    assert params.get("MazdaTorqueTuneSeeded") is None

  def test_other_car_persisted_is_not_seeded(self):
    params = FakeParams({"CarParamsPersistent": _non_mazda_cp().to_bytes(), "TorqueControlTune": 0.0})
    seed_car_defaults_offroad(params)
    assert params.get("TorqueControlTune") == 0.0

  def test_unparseable_carparams_does_not_raise(self):
    params = FakeParams({"CarParamsPersistent": b"\x00\x01garbage", "TorqueControlTune": 0.0})
    seed_car_defaults_offroad(params)
    assert params.get("TorqueControlTune") == 0.0
