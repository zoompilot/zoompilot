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
from openpilot.selfdrive.locationd.torqued import TorqueEstimator, TorqueBuckets, VERSION, MIN_FILTER_DECAY
from openpilot.sunnypilot.selfdrive.locationd.torqued_ext import (
  DEFAULT_SPEED_BIN_BOUNDS as SPEED_BIN_BOUNDS, DEFAULT_SPEED_BIN_CENTERS as SPEED_BIN_CENTERS,
  TorqueEstimatorExt,
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

  def test_version_exists(self):
    assert VERSION >= 1

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
    bounds = TorqueEstimatorExt._centers_to_bounds([10.0, 20.0, 30.0])
    assert bounds[0] == (5, 15.0)   # lo=DEFAULT[0][0], hi=midpoint(10,20)
    assert bounds[1] == (15.0, 25.0)
    assert bounds[2] == (25.0, 40)  # hi=DEFAULT[-1][1]

  def test_single_center(self):
    bounds = TorqueEstimatorExt._centers_to_bounds([20.0])
    assert bounds == [(5, 40)]

  def test_edges_use_default_bounds(self):
    bounds = TorqueEstimatorExt._centers_to_bounds([7.0, 35.0])
    assert bounds[0][0] == 5    # DEFAULT_SPEED_BIN_BOUNDS[0][0]
    assert bounds[-1][1] == 40  # DEFAULT_SPEED_BIN_BOUNDS[-1][1]
    assert bounds[0][1] == pytest.approx((7.0 + 35.0) / 2)
    assert bounds[1][0] == pytest.approx((7.0 + 35.0) / 2)

  def test_contiguous_coverage(self):
    """Each bin's upper bound must equal the next bin's lower bound."""
    centers = [8.0, 15.0, 22.0, 30.0]
    bounds = TorqueEstimatorExt._centers_to_bounds(centers)
    for i in range(len(bounds) - 1):
      assert bounds[i][1] == pytest.approx(bounds[i + 1][0])


@needs_speed_dep_car
class TestSpeedBinnedLearning:
  """Toggle ON on a configured car."""

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
    msg = est.get_msg()
    ltp = msg.lateralTorqueParameters
    assert ltp.latAccelFactorFiltered == pytest.approx(1.25, abs=1e-2)
    assert ltp.frictionCoefficientFiltered == pytest.approx(0.125, abs=1e-3)

  def test_global_buckets_still_require_min_vel(self, fake_params):
    est = TorqueEstimator(make_cp())
    assert len(est.filtered_points) == 0


class TestToggleGate:
  """Toggle OFF disables speed-binning even for configured cars."""

  def test_toggle_off_no_speed_bins(self, fake_params_off):
    if SPEED_DEP_FINGERPRINT:
      est = TorqueEstimator(make_cp(fingerprint=SPEED_DEP_FINGERPRINT))
      assert not est.speed_binned


class TestBackwardCompatibility:
  """Cars with the toggle OFF are unaffected."""

  def test_unconfigured_car_no_speed_bins(self, fake_params_off):
    est = TorqueEstimator(make_cp(fingerprint=NON_SPEED_DEP_FINGERPRINT))
    assert not est.speed_binned

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
    msg = est.get_msg()
    ltp = msg.lateralTorqueParameters
    assert ltp.latAccelFactorFiltered == pytest.approx(2.0, abs=1e-2)
    assert ltp.frictionCoefficientFiltered == pytest.approx(0.15, abs=1e-3)
    assert not est.speed_binned

  def test_unconfigured_car_no_speed_bin_attributes(self, fake_params_off):
    est = TorqueEstimator(make_cp(fingerprint=NON_SPEED_DEP_FINGERPRINT))
    assert not hasattr(est, 'speed_bin_points')
    assert not hasattr(est, 'speed_bin_filtered')

  def test_cal_percent_works_for_both(self, fake_params):
    fingerprints = [NON_SPEED_DEP_FINGERPRINT]
    if SPEED_DEP_FINGERPRINT:
      fingerprints.append(SPEED_DEP_FINGERPRINT)
    for fp in fingerprints:
      est = TorqueEstimator(make_cp(fingerprint=fp))
      msg = est.get_msg()
      assert msg.lateralTorqueParameters.calPerc == 0


class TestUnconfiguredCarToggleOn:
  """An unconfigured car with speed-dep ON gets the default bins and the offline seeds."""

  def test_default_bins_created(self, fake_params):
    est = TorqueEstimator(make_cp(fingerprint=NON_SPEED_DEP_FINGERPRINT))
    assert est.speed_binned
    est._on_torque_point(0.1, 0.3, 10.0)
    assert len(est.speed_bin_bounds) == len(SPEED_BIN_BOUNDS)
    assert est.speed_bin_centers == list(SPEED_BIN_CENTERS)

  def test_seeded_with_offline_values(self, fake_params):
    est = TorqueEstimator(make_cp(fingerprint=NON_SPEED_DEP_FINGERPRINT, lat_accel_factor=2.5, friction=0.18))
    est._on_torque_point(0.1, 0.3, 10.0)
    for i in range(len(SPEED_BIN_BOUNDS)):
      assert est.speed_bin_filtered[i]['latAccelFactor'].x == pytest.approx(2.5)
      assert est.speed_bin_filtered[i]['frictionCoefficient'].x == pytest.approx(0.18)


class TestOnTorquePointWhenOff:
  def test_no_bins_created_when_off(self, fake_params_off):
    est = TorqueEstimator(make_cp(fingerprint=NON_SPEED_DEP_FINGERPRINT))
    est._on_torque_point(0.1, 0.3, 10.0)
    assert not hasattr(est, 'speed_bin_points')


@needs_speed_dep_car
class TestSpeedBinInitIdempotency:
  """Lazy speed-bin init (triggered by _on_torque_point) must not re-init on later points."""

  def test_second_call_preserves_points(self, fake_params):
    est = TorqueEstimator(make_cp())
    centers, bounds = get_car_bins(SPEED_DEP_FINGERPRINT)
    vego = (bounds[0][0] + bounds[0][1]) / 2
    est._on_torque_point(0.1, 0.3, vego)
    assert len(est.speed_bin_points[0]) == 1

    est._on_torque_point(0.2, 0.4, vego)
    assert len(est.speed_bin_points[0]) == 2


@needs_speed_dep_car
class TestNaNHandling:
  """Bin behavior when the SVD fails. The bucket is a MagicMock here on purpose: it forces
  the failure path without needing thousands of points."""

  @staticmethod
  def _failing_bucket(est, target_bin, valid):
    bucket = MagicMock()
    bucket.is_calculable.return_value = True
    bucket.is_valid.return_value = valid
    bucket.get_points.return_value = np.zeros((10, 3))
    est.speed_bin_points[target_bin] = bucket
    est._speed_bin_last_len[target_bin] = -1  # force recalculation
    return bucket

  def test_svd_failure_returns_false(self, fake_params):
    est = TorqueEstimator(make_cp())
    self._failing_bucket(est, 1, valid=False)
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

  def test_non_valid_bin_svd_failure_preserves_bin(self, fake_params):
    """A bin that is calculable but not valid is NOT reset on SVD failure."""
    est = TorqueEstimator(make_cp())
    bucket = self._failing_bucket(est, 1, valid=False)
    with patch('numpy.linalg.svd', side_effect=np.linalg.LinAlgError):
      est._estimate_params_speed_binned()
    assert est.speed_bin_points[1] is bucket
