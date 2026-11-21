"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs
from opendbc.car.mazda.values import MazdaFlags
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

MAZDA_STEER_TO_ZERO_TORQUE_TUNE = 2.0  # FLOAT param; the tune fitted to the 2022+ EPS (latcontrol_torque_v2.py)


def seed_mazda_torque_defaults(CP: structs.CarParams, params: Params | None = None) -> None:
  """One-time: default the torque-control stack ON for Mazdas on the measured EPS hardware.

  Gated on the EPS hardware mask, not the model, so the CX-9 sharing this EPS, EPS swaps and
  legacy firmware on the same hardware are covered.
  Both seeds sit behind markers, because manager_init materializes every declared default at
  boot: TorqueControlTune is already 0.0 on disk by the time card runs, so "unset" never
  survives to here. The two toggles are seeded once behind MazdaTorqueDefaultsApplied. The
  tune is seeded once per value of MAZDA_STEER_TO_ZERO_TORQUE_TUNE, recorded in
  MazdaTorqueTuneSeeded, so a later bump moves everyone again while a choice made after the
  seed is kept. TorqueControlTune's declared default stays 0.0 for every other brand.
  """
  if params is None:
    params = Params()

  if CP.brand != "mazda" or not (CP.flags & MazdaFlags.EPS_HW):
    return
  if params.get("MazdaTorqueTuneSeeded") != MAZDA_STEER_TO_ZERO_TORQUE_TUNE:
    params.put("TorqueControlTune", MAZDA_STEER_TO_ZERO_TORQUE_TUNE, block=True)  # controlsd reads it at startup
    params.put("MazdaTorqueTuneSeeded", MAZDA_STEER_TO_ZERO_TORQUE_TUNE, block=True)
    cloudlog.warning("Seeded steer-to-zero Mazda TorqueControlTune=%s", MAZDA_STEER_TO_ZERO_TORQUE_TUNE)
  if params.get_bool("MazdaTorqueDefaultsApplied"):
    return

  params.put_bool("EnforceTorqueControl", True)     # torque lateral control
  params.put_bool("LiveTorqueParamsToggle", True)   # self-tune (live torque params)
  params.put_bool("MazdaTorqueDefaultsApplied", True)
  cloudlog.warning("Seeded steer-to-zero Mazda torque-control defaults (EnforceTorqueControl, self-tune)")


def seed_car_defaults_offroad(params: Params) -> None:
  """manager_init hook: apply the per-car seeds from the last drive's CarParams, so a device
  that updated offroad shows and runs the seeded defaults without waiting for card to
  fingerprint. A device that has never driven is seeded by card on its first drive."""
  CP_bytes = params.get("CarParamsPersistent")
  if CP_bytes is None:
    return
  try:
    from openpilot.cereal import messaging  # lazy: keep manager_init's import cost down
    from opendbc.car.structs import car
    CP = messaging.log_from_bytes(CP_bytes, car.CarParams)
  except Exception:
    cloudlog.exception("seed_car_defaults_offroad: could not parse CarParamsPersistent")
    return
  seed_mazda_torque_defaults(CP, params)
