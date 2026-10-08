"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

One-time move from the ICBM toggle to ICBM on demand (demand.py). With the toggle off, the
features that act through ICBM were inert; on demand they would bring ICBM up and start
pressing buttons. Turning them off keeps every install driving as it did.

It runs at car init rather than with system/params_migration.py: whether a feature was
inert depends on this drive's longitudinal mode and ICBM capability, which only the
fingerprinted CarParams know.
"""
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.demand import CONSUMERS
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import icbm_applicable

LEGACY_TOGGLE = "IntelligentCruiseButtonManagement"
MIGRATED = "IcbmDemandMigrated"


def migrate_icbm_toggle(CP, CP_SP, params: Params) -> None:
  if params.get_bool(MIGRATED):
    return

  try:
    if icbm_applicable(CP, CP_SP) and not params.get_bool(LEGACY_TOGGLE):
      op_long = CP.openpilotLongitudinalControl
      for consumer in CONSUMERS:
        if (consumer.inert_under_op_long or not op_long) and consumer.is_on(params):
          cloudlog.warning(f"ICBM migration: {consumer.key} was inert without ICBM, turning it off")
          if isinstance(consumer.off, bool):
            params.put_bool(consumer.key, consumer.off, block=True)
          else:
            params.put(consumer.key, consumer.off, block=True)
  except Exception:
    cloudlog.exception("ICBM migration failed, leaving the features as set")

  params.remove(LEGACY_TOGGLE)
  params.put_bool(MIGRATED, True, block=True)
