"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

zoompilot's boot hook beside sunnypilot's run_migration: one-time param migrations and the
offroad car-default seeds. Each migration keys on a marker param without a default:
manager_init writes every declared default to disk, so a changed default never reaches a device
that has booted before.
"""
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.car.mazda_torque_seed import seed_car_defaults_offroad

E2E_ASSISTS_MIGRATED = "ExperimentalAssistsDefaultOn"
E2E_ASSISTS = ("ExperimentalModeSetSpeed",)


def _migrate_e2e_assists_on(params: Params) -> None:
  # Set Speed Assist became default on 2026-10-26; turn it on once for every install, after which
  # the driver's choice stands. (Lead Follow Assist was folded into it 2026-10-27.)
  if params.get_bool(E2E_ASSISTS_MIGRATED):
    return
  try:
    for key in E2E_ASSISTS:
      params.put_bool(key, True, block=True)
    params.put_bool(E2E_ASSISTS_MIGRATED, True, block=True)
    cloudlog.info(f"params_migration_zp: turned on {', '.join(E2E_ASSISTS)}")
  except Exception:
    cloudlog.exception("params_migration_zp: e2e assists migration failed, retrying next boot")


def run_migration_zp(params: Params) -> None:
  _migrate_e2e_assists_on(params)
  seed_car_defaults_offroad(params)
