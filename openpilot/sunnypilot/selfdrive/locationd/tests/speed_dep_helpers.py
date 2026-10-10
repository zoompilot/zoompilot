"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Shared builders for the speed-dependent torque tests, learner and controller side: a real
CarParams, the two cache messages torqued writes, a PubMaster stand-in that captures the fork
message, the torqued pair the controller reads and the torque-tuning builder its override
writes into. Both sides use car/tests/fakes.FakeParams for Params.
"""
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.structs import car
from opendbc.sunnypilot.car.lateral_tune import get_speed_dep_config
from openpilot.selfdrive.locationd.torqued import VERSION, MIN_FILTER_DECAY
from openpilot.sunnypilot.selfdrive.locationd.speed_bin_learner import (
  DEFAULT_SPEED_BIN_BOUNDS, DEFAULT_SPEED_BIN_CENTERS, LIVE_TORQUE_PARAMETERS_SP_SERVICE, SpeedBinLearner,
)

# configured cars; every test is driven by config, not hardcoded fingerprints
SPEED_DEP_CARS = get_speed_dep_config()
SPEED_DEP_FINGERPRINT = next(iter(SPEED_DEP_CARS)) if SPEED_DEP_CARS else None

# sentinel fingerprint that must not appear in speed_dependent.toml
NON_SPEED_DEP_FINGERPRINT = 'NOT_IN_SPEED_DEP_TOML'
assert NON_SPEED_DEP_FINGERPRINT not in SPEED_DEP_CARS, f"{NON_SPEED_DEP_FINGERPRINT} unexpectedly in speed_dependent.toml"

# sample tables, the shape of a speed_dependent.toml entry
SAMPLE_SPEED_BP = [6.5, 10.0, 15.0, 21.0, 26.5, 32.0, 37.5]
SAMPLE_LAT_ACCEL_FACTOR_BP = [2.39, 2.52, 2.71, 2.39, 2.28, 2.22, 2.21]
SAMPLE_FRICTION_BP = [0.177, 0.158, 0.131, 0.118, 0.113, 0.109, 0.108]


def get_car_bins(fingerprint):
  """Bin centers and bounds for a configured car, or the defaults for an unconfigured one."""
  cfg = SPEED_DEP_CARS.get(fingerprint, {})
  if 'speed_bp' in cfg:
    centers = list(cfg['speed_bp'])
    bounds = SpeedBinLearner._centers_to_bounds(centers)
  else:
    centers = list(DEFAULT_SPEED_BIN_CENTERS)
    bounds = list(DEFAULT_SPEED_BIN_BOUNDS)
  return centers, bounds


def make_cp(fingerprint=None, lat_accel_factor=1.25, friction=0.125, brand='test'):
  """A real CarParams with a torque tune. minSteerSpeed stays 0 (a steer-to-zero EPS), so
  entries flagged requires_steer_to_zero remain valid."""
  if fingerprint is None:
    fingerprint = SPEED_DEP_FINGERPRINT
  CP = car.CarParams.new_message()
  CP.carFingerprint = fingerprint
  CP.brand = brand
  CP.lateralTuning.init('torque')
  CP.lateralTuning.torque.friction = friction
  CP.lateralTuning.torque.latAccelFactor = lat_accel_factor
  return CP


class FakePubMaster:
  """Captures what torqued_ext publishes on the fork service."""

  def __init__(self):
    self.sent = []

  def send(self, service, msg):
    self.sent.append((service, msg))

  def last(self, service=LIVE_TORQUE_PARAMETERS_SP_SERVICE):
    msgs = [m for s, m in self.sent if s == service]
    return getattr(msgs[-1], service) if msgs else None


def make_cache(decay=float(MIN_FILTER_DECAY), valid=True, version=VERSION, global_laf=1.25, global_friction=0.125):
  """Upstream's LiveTorqueParameters cache event as torqued's 60 s write serializes it: the
  restore key, decay and valid flag the fork restore reads off it. The bins are not on it:
  they go on the fork cache (make_cache_sp)."""
  msg = messaging.new_message('lateralTorqueParameters')
  msg.valid = True
  ltp = msg.lateralTorqueParameters
  ltp.version = version
  ltp.valid = valid
  ltp.decay = decay
  ltp.latAccelFactorFiltered = global_laf
  ltp.frictionCoefficientFiltered = global_friction
  return msg


def seed_version_of(fingerprint):
  """The TOML entry's seed_version, as the estimator reads it."""
  return int(SPEED_DEP_CARS.get(fingerprint, {}).get('seed_version', 0))


def make_cache_sp(centers, lafs, frictions, points=None, version=VERSION, seed_version=None):
  """The LiveTorqueParametersSP cache event (the liveTorqueParametersSP message with the
  buckets filled): VERSION, the seed version and centers keying it, the per-bin values, and
  the points. seed_version defaults to the test car's TOML entry."""
  msg = messaging.new_message(LIVE_TORQUE_PARAMETERS_SP_SERVICE)
  msg.valid = True
  sp = getattr(msg, LIVE_TORQUE_PARAMETERS_SP_SERVICE)
  sp.version = version
  sp.seedVersion = seed_version_of(SPEED_DEP_FINGERPRINT) if seed_version is None else seed_version
  sp.speedBinCenters = list(centers)
  sp.speedBinLatAccelFactors = list(lafs)
  sp.speedBinFrictions = list(frictions)
  sp.speedBinValid = [True] * len(centers)
  if points is not None:
    sp.speedBinPoints = points
  return msg


def in_bounds_values(est):
  """Per-bin (lafs, frictions) inside the estimator's sanity bounds and distinct from the
  seeds, pre-rounded to Float32 so a trip through the wire reads back exactly."""
  lafs = [float(np.float32(lo + 0.37 * (hi - lo))) for lo, hi in est.speed_bin_lat_accel_factor_bounds]
  frictions = [float(np.float32(lo + 0.61 * (hi - lo))) for lo, hi in est.speed_bin_friction_bounds]
  return lafs, frictions


def seed_values(est):
  return ([f['latAccelFactor'].x for f in est.speed_bin_filtered],
          [f['frictionCoefficient'].x for f in est.speed_bin_filtered])


def assert_untouched(est, seeds, n_points=0, decay=MIN_FILTER_DECAY):
  seed_lafs, seed_frictions = seeds
  for i in range(len(est.speed_bin_bounds)):
    assert est.speed_bin_filtered[i]['latAccelFactor'].x == seed_lafs[i]
    assert est.speed_bin_filtered[i]['frictionCoefficient'].x == seed_frictions[i]
    assert len(est.speed_bin_points[i]) == n_points
  assert all(d == decay for d in est.speed_bin_decays)


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


def activate_speed_dep(ovr):
  """Sets the sample tables on the override, as update_speed_dep_torque would."""
  ovr._speed_dep_active = True
  ovr._speed_dep_speed_bp = list(SAMPLE_SPEED_BP)
  ovr._speed_dep_lat_accel_factor_bp = list(SAMPLE_LAT_ACCEL_FACTOR_BP)
  ovr._speed_dep_friction_bp = list(SAMPLE_FRICTION_BP)
