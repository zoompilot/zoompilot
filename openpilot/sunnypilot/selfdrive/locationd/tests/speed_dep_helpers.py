"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Shared builders for the speed-binned torqued tests: a real CarParams, the two cache
messages torqued writes, a Params stand-in that serves both and a PubMaster stand-in that
captures the fork message.
"""
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.structs import car
from opendbc.sunnypilot.car.interfaces import get_speed_dep_config
from openpilot.selfdrive.locationd.torqued import VERSION, MIN_FILTER_DECAY
from openpilot.sunnypilot.selfdrive.locationd.torqued_ext import (
  DEFAULT_SPEED_BIN_BOUNDS, DEFAULT_SPEED_BIN_CENTERS, LIVE_TORQUE_PARAMETERS_SP_SERVICE, TorqueEstimatorExt,
)

# configured cars; every test is driven by config, not hardcoded fingerprints
SPEED_DEP_CARS = get_speed_dep_config()
SPEED_DEP_FINGERPRINT = next(iter(SPEED_DEP_CARS)) if SPEED_DEP_CARS else None

# sentinel fingerprint that must not appear in speed_dependent.toml
NON_SPEED_DEP_FINGERPRINT = 'NOT_IN_SPEED_DEP_TOML'
assert NON_SPEED_DEP_FINGERPRINT not in SPEED_DEP_CARS, f"{NON_SPEED_DEP_FINGERPRINT} unexpectedly in speed_dependent.toml"

SPEED_DEP_TOGGLES = ("SpeedDependentTorqueToggle", "EnforceTorqueControl", "LiveTorqueParamsToggle")


def get_car_bins(fingerprint):
  """Bin centers and bounds for a configured car, or the defaults for an unconfigured one."""
  cfg = SPEED_DEP_CARS.get(fingerprint, {})
  if 'speed_bp' in cfg:
    centers = list(cfg['speed_bp'])
    bounds = TorqueEstimatorExt._centers_to_bounds(centers)
  else:
    centers = list(DEFAULT_SPEED_BIN_CENTERS)
    bounds = list(DEFAULT_SPEED_BIN_BOUNDS)
  return centers, bounds


class FakeParams:
  """Stands in for Params in both torqued (caches) and torqued_ext (toggles). Speed-dep
  learning needs the whole activation chain the UI enforces (Enforce Torque Control,
  Self-Tune, the speed-dep toggle), so speed_dep_on drives all three; everything else
  reads off. Caches are served from, and cache writes recorded in, one dict."""

  def __init__(self, store=None, speed_dep_on=True):
    self.store = dict(store or {})
    self.speed_dep_on = speed_dep_on

  def get_bool(self, key):
    return self.speed_dep_on if key in SPEED_DEP_TOGGLES else False

  def get(self, key, **kwargs):
    return self.store.get(key)

  def put(self, key, value, **kwargs):
    self.store[key] = value

  def remove(self, key):
    self.store.pop(key, None)


def make_cp(fingerprint=None, lat_accel_factor=1.25, friction=0.125):
  """A real CarParams with a torque tune. minSteerSpeed stays 0 (a steer-to-zero EPS), so
  entries flagged requires_steer_to_zero remain valid."""
  if fingerprint is None:
    fingerprint = SPEED_DEP_FINGERPRINT
  CP = car.CarParams.new_message()
  CP.carFingerprint = fingerprint
  CP.brand = 'test'
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
