"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

One-time move from the ICBM toggle to ICBM on demand (demand.py). With the toggle off, the
features that act through ICBM were inert; on demand they would bring ICBM up and start
pressing buttons. Turning them off keeps every install driving as it did.
"""
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import icbm_applicable
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode

LEGACY_TOGGLE = "IntelligentCruiseButtonManagement"
MIGRATED = "IcbmDemandMigrated"


def migrate_icbm_toggle(CP, CP_SP, params: Params) -> None:
  if params.get_bool(MIGRATED):
    return

  if icbm_applicable(CP, CP_SP) and not params.get_bool(LEGACY_TOGGLE):
    inert = ["CustomAccIncrementsEnabled"]
    if not CP.openpilotLongitudinalControl:
      # openpilot long runs the curve speeds itself; stock ACC only had the buttons
      inert += ["SmartCruiseControlVision", "SmartCruiseControlMap"]
      if params.get("SpeedLimitMode", return_default=True) == Mode.assist:
        params.put("SpeedLimitMode", int(Mode.warning), block=True)
    for key in inert:
      if params.get_bool(key):
        cloudlog.warning(f"ICBM migration: {key} was inert without ICBM, turning it off")
        params.put_bool(key, False, block=True)

  params.remove(LEGACY_TOGGLE)
  params.put_bool(MIGRATED, True, block=True)
