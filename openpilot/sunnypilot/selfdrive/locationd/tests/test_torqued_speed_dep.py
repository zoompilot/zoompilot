"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Speed-binned learning in torqued: bin layout, point routing, the published message, the
toggle gates and the per-bin fit. The cache lives in test_torqued_cache_restore.py.
"""
import numpy as np
import pytest

from unittest.mock import MagicMock, patch
from opendbc.car.mazda.values import MazdaFlags
from openpilot.selfdrive.locationd.torqued import TorqueEstimator, TorqueBuckets, VERSION, MIN_FILTER_DECAY, POINTS_PER_BUCKET, \
  STEER_BUCKET_BOUNDS
from openpilot.sunnypilot.selfdrive.locationd.speed_bin_learner import (
  DEFAULT_SPEED_BIN_BOUNDS as SPEED_BIN_BOUNDS, DEFAULT_SPEED_BIN_CENTERS as SPEED_BIN_CENTERS,
  SpeedBinLearner,
)
from openpilot.sunnypilot.selfdrive.locationd.tests.speed_dep_helpers import (
  SPEED_DEP_CARS, SPEED_DEP_FINGERPRINT, NON_SPEED_DEP_FINGERPRINT, FakePubMaster, get_car_bins, make_cp,
)


def _published(est, **kwargs):
  """get_msg through a captured PubMaster: (lateralTorqueParameters, liveTorqueParametersSP)."""
  est._pm = FakePubMaster()
  msg = est.get_msg(**kwargs)
  return msg.lateralTorqueParameters, est._pm.last()

needs_speed_dep_car = pytest.mark.skipif(SPEED_DEP_FINGERPRINT is None, reason="No cars in speed_dependent.toml")


class TestSpeedDepConfig:
  """Config-level checks that need no estimator."""

  def test_speed_dep_config_has_entries(self):
    assert len(SPEED_DEP_CARS) > 0

  def test_speed_bin_bounds_cover_full_range(self):
    all_bounds = [b for bounds in SPEED_BIN_BOUNDS for b in bounds]
    assert min(all_bounds) == 5
    assert max(all_bounds) >= 35

  def test_speed_bin_centers_match_bounds(self):
    for center, (lo, hi) in zip(SPEED_BIN_CENTERS, SPEED_BIN_BOUNDS, strict=True):
      assert center >= lo
      assert center <= hi


class TestCentersToBounds:
  def test_midpoints_between_centers(self):
    bounds = SpeedBinLearner._centers_to_bounds([10.0, 20.0, 30.0])
    assert bounds[0] == (5, 15.0)   # lo=DEFAULT[0][0], hi=midpoint(10,20)
    assert bounds[1] == (15.0, 25.0)
    assert bounds[2] == (25.0, 40)  # hi=DEFAULT[-1][1]

  def test_single_center(self):
    bounds = SpeedBinLearner._centers_to_bounds([20.0])
    assert bounds == [(5, 40)]


@needs_speed_dep_car
class TestSpeedBinnedLearning:
  """Self-tune on, on a configured car."""

  def test_speed_bins_initialized(self, fake_params):
    for fingerprint in SPEED_DEP_CARS:
      centers, bounds = get_car_bins(fingerprint)
      est = TorqueEstimator(make_cp(fingerprint=fingerprint))
      assert est.speed_binned
      assert len(est.speed_bin_points) == len(bounds)

  def test_speed_bin_routing(self, fake_params):
    centers, bounds = get_car_bins(SPEED_DEP_FINGERPRINT)
    for bin_idx, (lo, hi) in enumerate(bounds):
      est = TorqueEstimator(make_cp())
      vego = (lo + hi) / 2.0
      est._on_torque_point(0.1, 0.3, vego)
      assert len(est.speed_bin_points[bin_idx]) == 1, \
        f"bin {bin_idx} ({lo}-{hi} m/s) should have 1 point at vego={vego}"
      for j in range(len(bounds)):
        if j != bin_idx:
          assert len(est.speed_bin_points[j]) == 0, \
            f"bin {j} should be empty when vego={vego}"

  def test_fork_message_fields(self, fake_params):
    """The bins ride on the fork message published beside the upstream one, which stays
    upstream's own (no speed-bin fields on lateralTorqueParameters)."""
    for fingerprint in SPEED_DEP_CARS:
      centers, bounds = get_car_bins(fingerprint)
      est = TorqueEstimator(make_cp(fingerprint=fingerprint))
      ltp, sp = _published(est)
      assert not hasattr(ltp, 'speedBinCenters')
      assert sp.version == VERSION
      assert len(sp.speedBinCenters) == len(centers)
      assert len(sp.speedBinLatAccelFactors) == len(bounds)
      assert len(sp.speedBinFrictions) == len(bounds)
      assert len(sp.speedBinValid) == len(bounds)
      assert len(sp.speedBinPoints) == 0

  def test_fork_message_once_per_frame_at_upstream_validity(self, fake_params):
    """torqued calls get_msg twice on a cache frame; the fork message goes out once per
    frame, carrying the validity of the upstream message it accompanies."""
    est = TorqueEstimator(make_cp())
    est._pm = FakePubMaster()
    est.get_msg(valid=False)
    est.get_msg(valid=False, with_points=True)
    assert len(est._pm.sent) == 1
    assert est._pm.sent[0][1].valid is False
    est.frame += 1
    est.get_msg(valid=True)
    assert len(est._pm.sent) == 2
    assert est._pm.sent[1][1].valid is True

  def test_global_fit_unchanged(self, fake_params):
    est = TorqueEstimator(make_cp(lat_accel_factor=1.25, friction=0.125))
    ltp = est.get_msg().lateralTorqueParameters
    assert ltp.latAccelFactorFiltered == pytest.approx(1.25, abs=1e-2)
    assert ltp.frictionCoefficientFiltered == pytest.approx(0.125, abs=1e-3)
    assert ltp.calPerc == 0


class TestSelfTuneGate:
  """Speed-dep runs wherever self-tune does, with no toggle of its own."""

  @staticmethod
  def _cp(brand):
    return make_cp(fingerprint=NON_SPEED_DEP_FINGERPRINT, brand=brand)

  @pytest.mark.parametrize("brand, self_tunes_bare", [('toyota', True), ('mazda', False)])
  def test_follows_self_tune(self, fake_params_off, brand, self_tunes_bare):
    # bare: upstream's brand gate; under Enforce Torque Control: the Self-Tune toggle
    assert TorqueEstimator(self._cp(brand)).speed_binned == self_tunes_bare
    fake_params_off.bools.add("EnforceTorqueControl")
    assert not TorqueEstimator(self._cp(brand)).speed_binned
    fake_params_off.bools.add("LiveTorqueParamsToggle")
    assert TorqueEstimator(self._cp(brand)).speed_binned

  def test_manual_override_keeps_bins(self, fake_params):
    # the override pauses the learner's output, not the learner
    fake_params.bools.update({"CustomTorqueParams", "TorqueParamsOverrideEnabled"})
    fake_params.store.update({"TorqueParamsOverrideLatAccelFactor": 2.0, "TorqueParamsOverrideFriction": 0.1})
    est = TorqueEstimator(make_cp(fingerprint=NON_SPEED_DEP_FINGERPRINT))
    assert est.speed_binned
    assert not est.use_params


class TestBackwardCompatibility:
  """Cars without self-tune are unaffected."""

  def test_unconfigured_car_creates_no_bins(self, fake_params_off):
    est = TorqueEstimator(make_cp(fingerprint=NON_SPEED_DEP_FINGERPRINT))
    assert not est.speed_binned
    est._on_torque_point(0.1, 0.3, 10.0)
    assert not hasattr(est, 'speed_bin_points')
    assert not hasattr(est, 'speed_bin_filtered')

  def test_unconfigured_car_publishes_empty_bins(self, fake_params_off):
    """The fork message still goes out (consumers check it alive), with no bins."""
    est = TorqueEstimator(make_cp(fingerprint=NON_SPEED_DEP_FINGERPRINT))
    _, sp = _published(est)
    assert sp.version == VERSION
    assert len(sp.speedBinCenters) == 0
    assert len(sp.speedBinLatAccelFactors) == 0
    assert len(sp.speedBinFrictions) == 0
    assert len(sp.speedBinValid) == 0

  def test_unconfigured_car_global_params_still_work(self, fake_params_off):
    est = TorqueEstimator(make_cp(fingerprint=NON_SPEED_DEP_FINGERPRINT, lat_accel_factor=2.0, friction=0.15))
    ltp = est.get_msg().lateralTorqueParameters
    assert ltp.latAccelFactorFiltered == pytest.approx(2.0, abs=1e-2)
    assert ltp.frictionCoefficientFiltered == pytest.approx(0.15, abs=1e-3)
    assert ltp.calPerc == 0


class TestUnconfiguredCarSelfTuneOn:
  """An unconfigured car with self-tune on gets the default bins and the offline seeds."""

  def test_default_bins_seeded_with_offline_values(self, fake_params):
    est = TorqueEstimator(make_cp(fingerprint=NON_SPEED_DEP_FINGERPRINT, lat_accel_factor=2.5, friction=0.18))
    assert est.speed_binned
    assert len(est.speed_bin_bounds) == len(SPEED_BIN_BOUNDS)
    assert est.speed_bin_centers == list(SPEED_BIN_CENTERS)
    for i in range(len(SPEED_BIN_BOUNDS)):
      assert est.speed_bin_filtered[i]['latAccelFactor'].x == pytest.approx(2.5)
      assert est.speed_bin_filtered[i]['frictionCoefficient'].x == pytest.approx(0.18)


@needs_speed_dep_car
class TestNaNHandling:
  """Bin behavior when the SVD fails. The bucket is a MagicMock here on purpose: it forces
  the failure path without needing thousands of points."""

  @staticmethod
  def _failing_bucket(est, target_bin, valid):
    bucket = MagicMock()
    bucket.is_valid.return_value = valid
    bucket.get_points.return_value = np.zeros((10, 3))
    est.speed_bin_points[target_bin] = bucket
    return bucket

  def test_svd_failure_returns_false(self, fake_params):
    est = TorqueEstimator(make_cp())
    self._failing_bucket(est, 1, valid=True)
    with patch('numpy.linalg.svd', side_effect=np.linalg.LinAlgError):
      results = est._estimate_params_speed_binned()
    assert dict(results)[1] is False

  def test_valid_bin_svd_failure_resets_bin(self, fake_params):
    """A bin with enough data that produces NaN/error is reset."""
    est = TorqueEstimator(make_cp())
    bucket = self._failing_bucket(est, 1, valid=True)
    with patch('numpy.linalg.svd', side_effect=np.linalg.LinAlgError):
      est._estimate_params_speed_binned()
    assert est.speed_bin_points[1] is not bucket
    assert isinstance(est.speed_bin_points[1], TorqueBuckets)
    assert est.speed_bin_decays[1] == MIN_FILTER_DECAY

  def test_invalid_bin_is_not_fit(self, fake_params):
    """A bin short of points is never fit, so an SVD failure cannot reset it."""
    est = TorqueEstimator(make_cp())
    bucket = self._failing_bucket(est, 1, valid=False)
    with patch('numpy.linalg.svd', side_effect=np.linalg.LinAlgError) as svd:
      results = est._estimate_params_speed_binned()
    svd.assert_not_called()
    assert dict(results)[1] is False
    assert est.speed_bin_points[1] is bucket


@needs_speed_dep_car
class TestFullBinKeepsLearning:
  """A bin's buckets are ring buffers: once all eight hold POINTS_PER_BUCKET its length stops
  changing while new points keep replacing old ones. The fit must still rerun on them; keyed on
  the length, a full bin froze at whatever it had learned when it filled."""

  def test_a_full_bin_refits_on_new_points(self, fake_params):
    est = TorqueEstimator(make_cp())
    i = 1
    lo, hi = est.speed_bin_bounds[i]
    rng = np.random.default_rng(0)
    bucket = est.speed_bin_points[i]
    for blo, bhi in STEER_BUCKET_BOUNDS:
      steer = rng.uniform(blo, bhi, POINTS_PER_BUCKET)
      bucket.load_points(np.c_[steer, 2.2 * steer + rng.normal(0.0, 0.05, len(steer))].tolist())
    full = len(STEER_BUCKET_BOUNDS) * POINTS_PER_BUCKET
    assert len(bucket) == full
    est._estimate_params_speed_binned()
    before = est.speed_bin_filtered[i]['latAccelFactor'].x
    for steer in rng.uniform(-0.45, 0.45, 50):
      est._on_torque_point(float(steer), 3.0 * float(steer), (lo + hi) / 2)
    assert len(bucket) == full
    est._estimate_params_speed_binned()
    assert est.speed_bin_filtered[i]['latAccelFactor'].x != before

  def test_no_refit_without_new_points(self, fake_params):
    est = TorqueEstimator(make_cp())
    bucket = est.speed_bin_points[0]
    bucket.load_points([[s, 2.0 * s] for s in np.linspace(-0.45, 0.45, 1000)])
    assert bucket.is_valid()
    est._estimate_params_speed_binned()
    before = est.speed_bin_filtered[0]['latAccelFactor'].x
    est._estimate_params_speed_binned()
    assert est.speed_bin_filtered[0]['latAccelFactor'].x == before


@needs_speed_dep_car
class TestFilteredOnlyOnceValid:
  """As upstream gates its global filter, a bin is fit and filtered only once it is valid; until
  then it publishes its seed and reads not valid."""

  def test_a_calculable_bin_keeps_its_seed_until_valid(self, fake_params):
    est = TorqueEstimator(make_cp())
    bucket = est.speed_bin_points[0]
    seed = est.speed_bin_filtered[0]['latAccelFactor'].x
    bucket.load_points([[s, 3.0 * s] for s in np.linspace(-0.45, 0.45, 80)])  # every bucket, too few points
    assert bucket.is_calculable() and not bucket.is_valid()
    assert dict(est._estimate_params_speed_binned())[0] is False
    assert est.speed_bin_filtered[0]['latAccelFactor'].x == seed

    bucket.load_points([[s, 3.0 * s] for s in np.linspace(-0.45, 0.45, 1000)])
    assert bucket.is_valid()
    assert dict(est._estimate_params_speed_binned())[0] is True
    assert est.speed_bin_filtered[0]['latAccelFactor'].x != seed


class TestLegacyFirmwareBins:
  def test_first_kept_bin_starts_at_its_own_edge(self, fake_params):
    # a legacy-firmware CX-5 2022 keeps the bins above its 45 kph floor; the first of them must
    # not reach down over the floor and the firmware's dead band to the default 5 m/s
    CP = make_cp('MAZDA_CX5_2022')
    CP.minSteerSpeed = 45 / 3.6
    est = TorqueEstimator(CP)
    full = SPEED_DEP_CARS['MAZDA_CX5_2022']['speed_bp']
    first = full.index(est.speed_bin_centers[0])
    assert est.speed_bin_bounds[0][0] == pytest.approx((full[first - 1] + full[first]) / 2)
    assert est.speed_bin_bounds[0][0] > CP.minSteerSpeed


class TestCustomTorqueParamsScale:
  """The custom offline values share the manual override's params, typed on upstream's scale."""

  @pytest.mark.parametrize("brand, laf, friction", [('mazda', 1.8, 0.1), ('toyota', 1.2, 0.15)])
  def test_offline_values_on_steer_max(self, fake_params, brand, laf, friction):
    fake_params.store.update(TorqueParamsOverrideLatAccelFactor='1.2', TorqueParamsOverrideFriction='0.15')
    fake_params.bools.add('CustomTorqueParams')
    CP = make_cp('MAZDA_CX5_2022', brand=brand)
    if brand == 'mazda':
      CP.flags = int(MazdaFlags.GEN1 | MazdaFlags.STEER_TO_ZERO_EPS)
    est = TorqueEstimator(CP)
    assert est.offline_latAccelFactor == pytest.approx(laf)
    assert est.offline_friction == pytest.approx(friction)
