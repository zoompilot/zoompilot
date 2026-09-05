"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Shared builders for the speed-dependent torque controller tests: the real torque-tuning
builder the override writes into, a torqued message, a Params stand-in for the override,
and a stub for the extension object update_speed_dep_torque runs against.
"""
from unittest.mock import MagicMock

import openpilot.cereal.messaging as messaging
from opendbc.car.structs import car
from openpilot.sunnypilot.selfdrive.locationd.torqued_ext import LIVE_TORQUE_PARAMETERS_SP_SERVICE

# sample tables, the shape of a speed_dependent.toml entry
SAMPLE_SPEED_BP = [6.5, 10.0, 15.0, 21.0, 26.5, 32.0, 37.5]
SAMPLE_LAT_ACCEL_FACTOR_BP = [2.39, 2.52, 2.71, 2.39, 2.28, 2.22, 2.21]
SAMPLE_FRICTION_BP = [0.177, 0.158, 0.131, 0.118, 0.113, 0.109, 0.108]


class FakeParams:
  """The override's Params: the enforce toggle, the manual override toggle and its two values."""

  def __init__(self, enforce=False, manual_override=False, manual_lat_accel_factor='200', manual_friction='15'):
    self.enforce = enforce
    self.manual_override = manual_override
    self.manual_lat_accel_factor = manual_lat_accel_factor
    self.manual_friction = manual_friction

  def get_bool(self, key):
    return {'EnforceTorqueControl': self.enforce, 'TorqueParamsOverrideEnabled': self.manual_override}.get(key, False)

  def get(self, key, **kwargs):
    return {'TorqueParamsOverrideLatAccelFactor': self.manual_lat_accel_factor,
            'TorqueParamsOverrideFriction': self.manual_friction}.get(key)


def make_cp(fingerprint='TEST_CAR', min_steer_speed=0.0):
  CP = car.CarParams.new_message()
  CP.carFingerprint = fingerprint
  CP.minSteerSpeed = min_steer_speed
  CP.lateralTuning.init('torque')
  return CP


def make_torque_params(latAccelFactor=2.0, latAccelOffset=0.0, friction=0.15):
  """The real CarParams.LateralTorqueTuning builder the controller hands the override. Its
  fields are Float32: a value written in reads back rounded."""
  tp = car.CarParams.new_message().lateralTuning.init('torque')
  tp.latAccelFactor = latAccelFactor
  tp.latAccelOffset = latAccelOffset
  tp.friction = friction
  return tp


def make_torqued_msg(speed_bp, lafs, frictions, valid, global_laf=2.0, global_fric=0.15, use_params=True):
  """The pair torqued publishes each cycle, as update_speed_dep_torque reads them: upstream's
  lateralTorqueParameters (globals, useParams) and the fork's liveTorqueParametersSP (bins)."""
  tp = messaging.new_message('lateralTorqueParameters').lateralTorqueParameters
  tp.useParams = use_params
  tp.latAccelFactorFiltered = global_laf
  tp.frictionCoefficientFiltered = global_fric
  tp.latAccelOffsetFiltered = 0.0
  tp_sp = getattr(messaging.new_message(LIVE_TORQUE_PARAMETERS_SP_SERVICE), LIVE_TORQUE_PARAMETERS_SP_SERVICE)
  tp_sp.speedBinCenters = list(speed_bp)
  tp_sp.speedBinLatAccelFactors = list(lafs)
  tp_sp.speedBinFrictions = list(frictions)
  tp_sp.speedBinValid = list(valid)
  return tp, tp_sp


def make_ext_stub(fingerprint='TEST_CAR'):
  """A stand-in for the LatControlTorqueExt instance update_speed_dep_torque and
  disable_speed_dep_torque are called on, with a real CarParams and the speed-dep state
  the override initializes. lac_torque stays a MagicMock so update_limits calls are countable."""
  stub = MagicMock()
  stub.CP = make_cp(fingerprint)
  stub._speed_dep_active = False
  stub._speed_dep_speed_bp = []
  stub._speed_dep_lat_accel_factor_bp = []
  stub._speed_dep_friction_bp = []
  stub._speed_dep_car_cfg = None
  return stub


def activate_speed_dep(ovr, speed_bp=None, lat_accel_factor_bp=None, friction_bp=None):
  """Sets the tables update_speed_dep_torque would set on the override."""
  ovr._speed_dep_active = True
  ovr._speed_dep_speed_bp = speed_bp or list(SAMPLE_SPEED_BP)
  ovr._speed_dep_lat_accel_factor_bp = lat_accel_factor_bp or list(SAMPLE_LAT_ACCEL_FACTOR_BP)
  ovr._speed_dep_friction_bp = friction_bp or list(SAMPLE_FRICTION_BP)
