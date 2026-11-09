"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

zoompilot's boot hook beside sunnypilot's run_migration: one-time param migrations and the
offroad car-default seeds. Each migration keys on a marker param without a default:
manager_init writes every declared default to disk, so a changed default never reaches a device
that has booted before.
"""
from pathlib import Path

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.car.mazda_torque_seed import seed_car_defaults_offroad
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Policy
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.source_auto import AUTO

E2E_ASSISTS_MIGRATED = "ExperimentalAssistsDefaultOn"
E2E_ASSISTS = ("ExperimentalModeSetSpeed",)
LANE_CENTER_MIGRATED = "LaneCenterAssistDefaultOn"
OLD_LANE_CENTER_READOUT = "BlendfulReadout"


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


def _migrate_lane_center_on(params: Params) -> None:
  # Lane Center Assist became default on 2026-11-06; turn it on once for every install, after
  # which the driver's choice stands. Upstream's Blendful default stays "0", so new installs get
  # it here too. LaneCenterUI is left as it is.
  if params.get_bool(LANE_CENTER_MIGRATED):
    return
  try:
    params.put_bool("Blendful", True, block=True)
    params.put_bool(LANE_CENTER_MIGRATED, True, block=True)
    cloudlog.info("params_migration_zp: turned on Lane Center Assist")
  except Exception:
    cloudlog.exception("params_migration_zp: lane center migration failed, retrying next boot")


def _migrate_lane_center_ui(params: Params) -> None:
  # The comma 4's "debug ui" lane center option (BlendfulReadout) became LaneCenterUI 2026-11-08:
  # debug stays debug, anything else gets the gauge. BlendfulReadout is no longer a declared key,
  # so its file is read directly and removed after; no file, nothing to migrate.
  path = Path(params.get_param_path(OLD_LANE_CENTER_READOUT))
  if not path.is_file():
    return
  try:
    debug = path.read_bytes().strip() == b"1"
    params.put("LaneCenterUI", 2 if debug else 1, block=True)
    path.unlink()
    cloudlog.info(f"params_migration_zp: BlendfulReadout folded into LaneCenterUI ({'debug' if debug else 'gauge'})")
  except Exception:
    cloudlog.exception("params_migration_zp: LaneCenterUI migration failed, retrying next boot")


def _fold_speed_limit_sources(params: Params) -> None:
  # Car-first, map-first and combined became one Auto source 2026-11-05. No marker: the settings
  # can no longer write 3 or 4, but a restored backup can.
  try:
    if params.get("SpeedLimitPolicy", return_default=True) in (Policy.map_data_priority, Policy.combined):
      params.put("SpeedLimitPolicy", int(AUTO), block=True)
      cloudlog.info("params_migration_zp: SpeedLimitPolicy folded into Auto")
  except Exception:
    cloudlog.exception("params_migration_zp: SpeedLimitPolicy fold failed, retrying next boot")


def run_migration_zp(params: Params) -> None:
  _migrate_e2e_assists_on(params)
  _migrate_lane_center_on(params)
  _migrate_lane_center_ui(params)
  _fold_speed_limit_sources(params)
  seed_car_defaults_offroad(params)
