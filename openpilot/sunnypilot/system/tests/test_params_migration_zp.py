"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from pathlib import Path

import pytest

from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Policy
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.source_auto import AUTO
from openpilot.sunnypilot.system.params_migration_zp import E2E_ASSISTS, E2E_ASSISTS_MIGRATED, LANE_CENTER_MIGRATED, OLD_LANE_CENTER_READOUT, \
  run_migration_zp


class TestE2EAssistsMigration(OpenpilotTestCase):
  """Existing installs hold the old declared default (off) on disk; the migration turns the
  assists on once, then leaves the driver's choice alone."""

  def test_turns_on_the_materialized_old_default(self):
    params = Params()
    for key in E2E_ASSISTS:
      params.put_bool(key, False, block=True)  # what manager_init wrote under the old default
    run_migration_zp(params)
    assert all(params.get_bool(key) for key in E2E_ASSISTS)
    assert params.get_bool(E2E_ASSISTS_MIGRATED)

  def test_runs_once(self):
    params = Params()
    run_migration_zp(params)
    params.put_bool("ExperimentalModeSetSpeed", False, block=True)  # the driver turns it off
    run_migration_zp(params)
    assert not params.get_bool("ExperimentalModeSetSpeed")


class TestLaneCenterMigration(OpenpilotTestCase):
  """Blendful's declared default (off) is already on disk; the migration turns it on once."""

  def test_turns_on_the_materialized_default(self):
    params = Params()
    params.put_bool("Blendful", False, block=True)  # what manager_init wrote
    run_migration_zp(params)
    assert params.get_bool("Blendful")
    assert params.get_bool(LANE_CENTER_MIGRATED)

  def test_runs_once(self):
    params = Params()
    run_migration_zp(params)
    params.put_bool("Blendful", False, block=True)  # the driver turns it off
    run_migration_zp(params)
    assert not params.get_bool("Blendful")


class TestLaneCenterUIMigration(OpenpilotTestCase):
  """BlendfulReadout, no longer a declared key, is left on disk as a file: debug becomes debug,
  anything else the gauge, and the file goes so it runs once."""

  @staticmethod
  def _old_readout(params: Params) -> Path:
    return Path(params.get_param_path(OLD_LANE_CENTER_READOUT))

  def _check(self, old: bytes, expected: int):
    params = Params()
    params.put("LaneCenterUI", 0, block=True)
    self._old_readout(params).write_bytes(old)
    run_migration_zp(params)
    assert params.get("LaneCenterUI") == expected
    assert not self._old_readout(params).exists()

  def test_debug_stays_debug(self):
    self._check(b"1", 2)

  def test_readout_off_gets_the_gauge(self):
    self._check(b"0", 1)

  def test_no_old_file_leaves_the_setting(self):
    params = Params()
    params.put("LaneCenterUI", 0, block=True)  # the driver turned it off after migrating
    run_migration_zp(params)
    assert params.get("LaneCenterUI") == 0

  def test_old_key_is_gone(self):
    with pytest.raises(UnknownKeyName):
      Params().get(OLD_LANE_CENTER_READOUT)


class TestSpeedLimitSourceFold(OpenpilotTestCase):
  """Map-first (the old default, already on disk) and combined become Auto; car and map stand."""

  def test_folds_map_first_and_combined(self):
    params = Params()
    for old in (Policy.map_data_priority, Policy.combined):
      params.put("SpeedLimitPolicy", int(old), block=True)
      run_migration_zp(params)
      assert params.get("SpeedLimitPolicy") == AUTO

  def test_keeps_car_and_map_only(self):
    params = Params()
    for kept in (Policy.car_state_only, Policy.map_data_only, AUTO):
      params.put("SpeedLimitPolicy", int(kept), block=True)
      run_migration_zp(params)
      assert params.get("SpeedLimitPolicy") == kept
