"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The speed-bin cache: upstream's LiveTorqueParameters supplies the restore key, decay and
valid flag, the fork's LiveTorqueParametersSP the bin values and points; the guards on
both, and the cache write that feeds them.
"""
import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.locationd.torqued import TorqueEstimator, VERSION, MIN_FILTER_DECAY
from openpilot.sunnypilot.selfdrive.locationd.torqued_ext import LIVE_TORQUE_PARAMETERS_SP_KEY, LIVE_TORQUE_PARAMETERS_SP_SERVICE
from openpilot.sunnypilot.selfdrive.locationd.tests.speed_dep_helpers import (
  SPEED_DEP_FINGERPRINT, NON_SPEED_DEP_FINGERPRINT, FakePubMaster, get_car_bins, make_cp, make_cache, make_cache_sp,
  in_bounds_values, seed_values, assert_untouched, seed_version_of,
)

pytestmark = pytest.mark.skipif(SPEED_DEP_FINGERPRINT is None, reason="No cars in speed_dependent.toml")


def _cache(**kwargs):
  """Upstream's cache as _restore_ext_cache reads it: key, decay, valid."""
  return make_cache(**kwargs).lateralTorqueParameters


def _sp(est, lafs, frictions, points=None, centers=None, version=VERSION, seed_version=None):
  """The fork cache keyed to this estimator's config."""
  msg = make_cache_sp(est.speed_bin_centers if centers is None else centers, lafs, frictions, points=points, version=version,
                      seed_version=est.speed_dep_seed_version if seed_version is None else seed_version)
  return getattr(msg, LIVE_TORQUE_PARAMETERS_SP_SERVICE)


def _read_sp(cache_bytes):
  with log.Event.from_bytes(cache_bytes) as evt:
    sp = getattr(evt, LIVE_TORQUE_PARAMETERS_SP_SERVICE)
    return (list(sp.speedBinLatAccelFactors), list(sp.speedBinFrictions),
            [[list(p) for p in bin_points] for bin_points in sp.speedBinPoints])


def _one_point_per_bin(est):
  return [[[0.11, 0.3]] for _ in est.speed_bin_bounds]


class TestCacheRestore:
  """_restore_ext_cache called directly with the cache objects and the cached CP."""

  def test_successful_restore_updates_filters(self, fake_params):
    est = TorqueEstimator(make_cp())
    lafs, frictions = in_bounds_values(est)
    n_bins = len(est.speed_bin_bounds)
    points = [[[0.11 + 0.01 * i, 0.3]] * 2 for i in range(n_bins)]

    est._restore_ext_cache(_cache(decay=120.0), cache_CP=est.CP, cache_sp=_sp(est, lafs, frictions, points))

    for i in range(n_bins):
      assert est.speed_bin_filtered[i]['latAccelFactor'].x == lafs[i]
      assert est.speed_bin_filtered[i]['frictionCoefficient'].x == frictions[i]
      assert len(est.speed_bin_points[i]) == 2
    assert all(d == 120.0 for d in est.speed_bin_decays)

  def test_same_key_from_a_different_cp_object_accepted(self, fake_params):
    """The key is by value (fingerprint, tuning, seeds, VERSION), not object identity."""
    est = TorqueEstimator(make_cp())
    lafs, frictions = in_bounds_values(est)
    est._restore_ext_cache(_cache(), cache_CP=make_cp(), cache_sp=_sp(est, lafs, frictions))
    assert est.speed_bin_filtered[0]['latAccelFactor'].x == lafs[0]

  def test_mismatched_laf_length_rejected(self, fake_params):
    est = TorqueEstimator(make_cp())
    seeds = seed_values(est)
    _, frictions = in_bounds_values(est)
    est._restore_ext_cache(_cache(), cache_CP=est.CP, cache_sp=_sp(est, [seeds[0][0]], frictions))
    assert_untouched(est, seeds)

  def test_mismatched_friction_length_rejected(self, fake_params):
    """Both LAF and friction must match the bin count; if only one matches, nothing is restored."""
    est = TorqueEstimator(make_cp())
    seeds = seed_values(est)
    lafs, frictions = in_bounds_values(est)
    est._restore_ext_cache(_cache(), cache_CP=est.CP, cache_sp=_sp(est, lafs, [frictions[0]]))
    assert_untouched(est, seeds)

  def test_missing_points_still_restores_filters(self, fake_params):
    est = TorqueEstimator(make_cp())
    lafs, frictions = in_bounds_values(est)
    est._restore_ext_cache(_cache(), cache_CP=est.CP, cache_sp=_sp(est, lafs, frictions))
    for i in range(len(est.speed_bin_bounds)):
      assert est.speed_bin_filtered[i]['latAccelFactor'].x == lafs[i]
      assert est.speed_bin_filtered[i]['frictionCoefficient'].x == frictions[i]
      assert len(est.speed_bin_points[i]) == 0

  def test_fork_cache_read_from_params_when_not_passed(self, fake_params):
    est = TorqueEstimator(make_cp())
    lafs, frictions = in_bounds_values(est)
    fake_params.store[LIVE_TORQUE_PARAMETERS_SP_KEY] = make_cache_sp(est.speed_bin_centers, lafs, frictions,
                                                                     points=_one_point_per_bin(est)).to_bytes()
    est._restore_ext_cache(_cache(), cache_CP=est.CP)
    assert est.speed_bin_filtered[0]['latAccelFactor'].x == lafs[0]
    assert all(len(b) == 1 for b in est.speed_bin_points)

  def test_absent_fork_cache_restores_nothing(self, fake_params):
    """Upstream's cache alone carries no bins; without the fork key learning restarts."""
    est = TorqueEstimator(make_cp())
    seeds = seed_values(est)
    est._restore_ext_cache(_cache(decay=200.0), cache_CP=est.CP)
    assert_untouched(est, seeds)


  def test_decay_comes_from_cache_not_min(self, fake_params):
    """At init self.decay does not exist yet; the cache's decay is what upstream restores, so
    the bins resume there instead of dropping back to MIN_FILTER_DECAY every boot."""
    est = TorqueEstimator(make_cp())
    lafs, frictions = in_bounds_values(est)
    saved_decay = est.decay
    del est.decay

    est._restore_ext_cache(_cache(decay=200.0), cache_CP=est.CP, cache_sp=_sp(est, lafs, frictions))

    assert all(d == 200.0 for d in est.speed_bin_decays)
    assert all(d != MIN_FILTER_DECAY for d in est.speed_bin_decays)
    for f in est.speed_bin_filtered:
      assert f['latAccelFactor'].alpha == pytest.approx(DT_MDL / (200.0 + DT_MDL))
      assert f['frictionCoefficient'].alpha == pytest.approx(DT_MDL / (200.0 + DT_MDL))
    est.decay = saved_decay

  def test_estimator_decay_does_not_override_cache(self, fake_params):
    est = TorqueEstimator(make_cp())
    lafs, frictions = in_bounds_values(est)
    est.decay = 200
    est._restore_ext_cache(_cache(decay=120.0), cache_CP=est.CP, cache_sp=_sp(est, lafs, frictions))
    assert all(d == 120.0 for d in est.speed_bin_decays)

  def test_decay_outside_range_clipped(self, fake_params):
    est = TorqueEstimator(make_cp())
    lafs, frictions = in_bounds_values(est)
    est._restore_ext_cache(_cache(decay=0.0), cache_CP=est.CP, cache_sp=_sp(est, lafs, frictions))
    assert all(d == MIN_FILTER_DECAY for d in est.speed_bin_decays)
    assert est.speed_bin_filtered[0]['latAccelFactor'].x == lafs[0]

  def test_non_finite_decay_rejected(self, fake_params):
    est = TorqueEstimator(make_cp())
    seeds = seed_values(est)
    lafs, frictions = in_bounds_values(est)
    est._restore_ext_cache(_cache(decay=float('nan')), cache_CP=est.CP, cache_sp=_sp(est, lafs, frictions))
    assert_untouched(est, seeds)


  def test_nan_in_one_bin_rejects_whole_cache(self, fake_params):
    """Upstream drops the entire cache on any bad content (one try/except, full reset on NaN).
    The bins are one interpolated tune, so a partial restore would leave a step between a
    cached bin and a re-seeded neighbour; the whole cache goes, points included."""
    est = TorqueEstimator(make_cp())
    seeds = seed_values(est)
    lafs, frictions = in_bounds_values(est)
    lafs[1] = float('nan')

    est._restore_ext_cache(_cache(decay=200.0), cache_CP=est.CP, cache_sp=_sp(est, lafs, frictions, _one_point_per_bin(est)))

    assert_untouched(est, seeds)

  def test_inf_friction_rejects_whole_cache(self, fake_params):
    est = TorqueEstimator(make_cp())
    seeds = seed_values(est)
    lafs, frictions = in_bounds_values(est)
    frictions[-1] = float('inf')
    est._restore_ext_cache(_cache(), cache_CP=est.CP, cache_sp=_sp(est, lafs, frictions))
    assert_untouched(est, seeds)

  def test_value_outside_sanity_bounds_rejected(self, fake_params):
    """Same clip range the learner applies to its SVD output; a cached value past it cannot
    have come from this config."""
    est = TorqueEstimator(make_cp())
    seeds = seed_values(est)
    lafs, frictions = in_bounds_values(est)
    lafs[2] = est.speed_bin_lat_accel_factor_bounds[2][1] * 1.5
    est._restore_ext_cache(_cache(), cache_CP=est.CP, cache_sp=_sp(est, lafs, frictions))
    assert_untouched(est, seeds)

    lafs, frictions = in_bounds_values(est)
    frictions[0] = est.speed_bin_friction_bounds[0][0] - 0.05
    est._restore_ext_cache(_cache(), cache_CP=est.CP, cache_sp=_sp(est, lafs, frictions))
    assert_untouched(est, seeds)

  def test_value_on_clip_bound_accepted(self, fake_params):
    """A filter railed at its clip bound reads back through Float32 a rounding step past it."""
    est = TorqueEstimator(make_cp())
    lafs = [float(np.float32(hi)) for _, hi in est.speed_bin_lat_accel_factor_bounds]
    frictions = [float(np.float32(lo)) for lo, _ in est.speed_bin_friction_bounds]
    est._restore_ext_cache(_cache(), cache_CP=est.CP, cache_sp=_sp(est, lafs, frictions))
    for i in range(len(est.speed_bin_bounds)):
      assert est.speed_bin_filtered[i]['latAccelFactor'].x == lafs[i]
      assert est.speed_bin_filtered[i]['frictionCoefficient'].x == frictions[i]

  def test_wrong_version_rejected(self, fake_params):
    """VERSION is checked on both caches: either one behind rejects the restore."""
    est = TorqueEstimator(make_cp())
    seeds = seed_values(est)
    lafs, frictions = in_bounds_values(est)
    points = _one_point_per_bin(est)
    est._restore_ext_cache(_cache(decay=200.0, version=VERSION + 1), cache_CP=est.CP,
                           cache_sp=_sp(est, lafs, frictions, points))
    assert_untouched(est, seeds)
    est._restore_ext_cache(_cache(decay=200.0), cache_CP=est.CP,
                           cache_sp=_sp(est, lafs, frictions, points, version=VERSION + 1))
    assert_untouched(est, seeds)

  def test_wrong_restore_key_rejected(self, fake_params):
    est = TorqueEstimator(make_cp())
    seeds = seed_values(est)
    lafs, frictions = in_bounds_values(est)
    sp = _sp(est, lafs, frictions, _one_point_per_bin(est))

    # different car
    est._restore_ext_cache(_cache(decay=200.0), cache_CP=make_cp(fingerprint=NON_SPEED_DEP_FINGERPRINT), cache_sp=sp)
    assert_untouched(est, seeds)
    # same car, different offline seed (part of upstream's key)
    est._restore_ext_cache(_cache(decay=200.0), cache_CP=make_cp(lat_accel_factor=2.0), cache_sp=sp)
    assert_untouched(est, seeds)

  def test_other_bin_layout_rejects_whole_cache(self, fake_params):
    """The centers key the fork cache: values and points learned on another layout do not apply."""
    est = TorqueEstimator(make_cp())
    seeds = seed_values(est)
    lafs, frictions = in_bounds_values(est)
    other_centers = [c + 0.5 for c in est.speed_bin_centers]
    est._restore_ext_cache(_cache(decay=200.0), cache_CP=est.CP,
                           cache_sp=_sp(est, lafs, frictions, _one_point_per_bin(est), centers=other_centers))
    assert_untouched(est, seeds)

  def test_seed_version_read_from_config(self, fake_params):
    est = TorqueEstimator(make_cp())
    assert est.speed_dep_seed_version == seed_version_of(est.CP.carFingerprint)

  @pytest.mark.parametrize("delta", [1, -1], ids=["cache_behind", "cache_ahead"])
  def test_other_seed_version_rejects_whole_cache(self, fake_params, delta):
    """The TOML entry's seed_version keys the fork cache: a bump retires the bins and the
    points learned under the old seeds, so the new seeds take over on the next boot. Any
    mismatch counts, since neither side's values were learned under this config."""
    est = TorqueEstimator(make_cp())
    seeds = seed_values(est)
    lafs, frictions = in_bounds_values(est)
    est._restore_ext_cache(_cache(decay=200.0), cache_CP=est.CP,
                           cache_sp=_sp(est, lafs, frictions, _one_point_per_bin(est),
                                        seed_version=est.speed_dep_seed_version + delta))
    assert_untouched(est, seeds)

  def test_legacy_cache_without_seed_version_reads_zero(self, fake_params):
    """A cache written before the field existed reads back 0, so the first seed_version on
    an entry retires it exactly once, and an entry without one keeps restoring it."""
    est = TorqueEstimator(make_cp())
    seeds = seed_values(est)
    lafs, frictions = in_bounds_values(est)
    sp = _sp(est, lafs, frictions, seed_version=0)
    assert sp.seedVersion == 0
    est._restore_ext_cache(_cache(), cache_CP=est.CP, cache_sp=sp)
    if est.speed_dep_seed_version == 0:
      assert est.speed_bin_filtered[0]['latAccelFactor'].x == lafs[0]
    else:
      assert_untouched(est, seeds)

  def test_invalid_cache_keeps_seeds_but_restores_points_and_decay(self, fake_params):
    """Upstream takes filtered values only from a valid cache, and still reloads points and
    decay on a key match so the learner resumes with its data. Mirror both halves."""
    est = TorqueEstimator(make_cp())
    seeds = seed_values(est)
    lafs, frictions = in_bounds_values(est)
    points = [[[0.11, 0.3]] * 2 for _ in est.speed_bin_bounds]

    est._restore_ext_cache(_cache(decay=200.0, valid=False), cache_CP=est.CP, cache_sp=_sp(est, lafs, frictions, points))

    assert_untouched(est, seeds, n_points=2, decay=200.0)


  def _restore_with_points(self, est, points):
    lafs, frictions = in_bounds_values(est)
    est._restore_ext_cache(_cache(decay=200.0), cache_CP=est.CP, cache_sp=_sp(est, lafs, frictions, points))
    for i in range(len(est.speed_bin_bounds)):
      assert est.speed_bin_filtered[i]['latAccelFactor'].x == lafs[i]
    assert all(d == 200.0 for d in est.speed_bin_decays)

  def test_non_finite_point_drops_points_keeps_values(self, fake_params):
    est = TorqueEstimator(make_cp())
    points = _one_point_per_bin(est)
    points[3] = [[0.11, float('nan')]]
    self._restore_with_points(est, points)
    assert all(len(b) == 0 for b in est.speed_bin_points)

  def test_points_bin_count_mismatch_dropped(self, fake_params):
    est = TorqueEstimator(make_cp())
    self._restore_with_points(est, _one_point_per_bin(est)[:-1])
    assert all(len(b) == 0 for b in est.speed_bin_points)


class TestCacheRestoreGolden:
  """End to end through Params bytes: the restore must hand back exactly what a healthy
  cache holds. Guards the test car's tune."""

  @staticmethod
  def _healthy_cache(fake, CP, decay=173.25, **overrides):
    """The two blobs torqued itself would write for this CP: upstream's cache, and the fork
    cache with matching centers, finite values inside the per-bin sanity bounds and three
    points per bin. Returns (cache, cache_sp, seed_est)."""
    seed_est = TorqueEstimator(CP)
    n_bins = len(seed_est.speed_bin_bounds)
    lafs, frictions = in_bounds_values(seed_est)
    points = [[[0.11 + 0.01 * i, 0.3 + 0.02 * i]] * 3 for i in range(n_bins)]
    seed_version = overrides.pop('seed_version', seed_est.speed_dep_seed_version)  # fork cache only
    kwargs = {'decay': decay, 'global_laf': CP.lateralTuning.torque.latAccelFactor,
              'global_friction': CP.lateralTuning.torque.friction, **overrides}
    cache = make_cache(**kwargs).to_bytes()
    cache_sp = make_cache_sp(seed_est.speed_bin_centers, lafs, frictions, points=points,
                             version=overrides.get('version', VERSION), seed_version=seed_version).to_bytes()
    return cache, cache_sp, seed_est

  @staticmethod
  def _restore_from(fake, CP, cache, cache_sp=None, prev_route=None):
    fake.store = {"LiveTorqueParameters": cache, "CarParamsPrevRoute": (CP if prev_route is None else prev_route).to_bytes()}
    if cache_sp is not None:
      fake.store[LIVE_TORQUE_PARAMETERS_SP_KEY] = cache_sp
    return TorqueEstimator(CP)

  def test_valid_cache_restores_values_bit_identical(self, fake_params):
    CP = make_cp()
    cache, cache_sp, _ = self._healthy_cache(fake_params, CP)
    est = self._restore_from(fake_params, CP, cache, cache_sp)

    # expected values are what the wire carries (Float32), read back through the same reader
    exp_lafs, exp_frictions, exp_points = _read_sp(cache_sp)
    n_bins = len(est.speed_bin_bounds)
    assert n_bins == len(exp_lafs) > 0
    for i in range(n_bins):
      # exact equality on purpose: no approx, the restore must not touch the numbers
      assert est.speed_bin_filtered[i]['latAccelFactor'].x == exp_lafs[i]
      assert est.speed_bin_filtered[i]['frictionCoefficient'].x == exp_frictions[i]
      assert len(est.speed_bin_points[i]) == 3
      assert est.speed_bin_points[i].get_points()[:, [0, 2]].tolist() == exp_points[i]

  def test_fork_cache_absent_restores_nothing(self, fake_params):
    CP = make_cp()
    cache, _, seed_est = self._healthy_cache(fake_params, CP)
    est = self._restore_from(fake_params, CP, cache)
    assert_untouched(est, seed_values(seed_est))

  def test_decay_restored_matches_upstream(self, fake_params):
    """No per-bin decay exists on the wire; the bins take the same cached decay upstream
    restores into self.decay, not MIN_FILTER_DECAY."""
    CP = make_cp()
    cache, cache_sp, _ = self._healthy_cache(fake_params, CP, decay=173.25)
    est = self._restore_from(fake_params, CP, cache, cache_sp)
    assert est.decay == 173.25
    assert est.speed_bin_decays == [est.decay] * len(est.speed_bin_bounds)

  def test_wrong_version_rejected(self, fake_params):
    CP = make_cp()
    cache, cache_sp, seed_est = self._healthy_cache(fake_params, CP, version=VERSION + 1)
    est = self._restore_from(fake_params, CP, cache, cache_sp)
    assert_untouched(est, seed_values(seed_est))

  def test_seed_version_bump_rejected(self, fake_params):
    """A release that bumps seed_version in the TOML finds every device's cache one behind."""
    CP = make_cp()
    cache, cache_sp, seed_est = self._healthy_cache(fake_params, CP, seed_version=TorqueEstimator(CP).speed_dep_seed_version - 1)
    est = self._restore_from(fake_params, CP, cache, cache_sp)
    assert_untouched(est, seed_values(seed_est))

  def test_prev_route_carparams_mismatch_rejected(self, fake_params):
    """CarParamsPrevRoute is part of upstream's key: a cache learned under other offline seeds
    or another fingerprint does not apply here, points included."""
    CP = make_cp()
    cache, cache_sp, seed_est = self._healthy_cache(fake_params, CP)
    for prev in (make_cp(friction=0.2), make_cp(fingerprint=NON_SPEED_DEP_FINGERPRINT)):
      est = self._restore_from(fake_params, CP, cache, cache_sp, prev_route=prev)
      assert_untouched(est, seed_values(seed_est))

  def test_missing_prev_route_carparams_rejected(self, fake_params):
    CP = make_cp()
    cache, cache_sp, seed_est = self._healthy_cache(fake_params, CP)
    fake_params.store = {"LiveTorqueParameters": cache, LIVE_TORQUE_PARAMETERS_SP_KEY: cache_sp}
    est = TorqueEstimator(CP)
    assert_untouched(est, seed_values(seed_est))

  def test_invalid_cache_keeps_seeds_restores_points_and_decay(self, fake_params):
    CP = make_cp()
    cache, cache_sp, seed_est = self._healthy_cache(fake_params, CP, valid=False, decay=173.25)
    est = self._restore_from(fake_params, CP, cache, cache_sp)
    assert_untouched(est, seed_values(seed_est), n_points=3, decay=173.25)

  def test_nan_bin_rejects_whole_cache(self, fake_params):
    CP = make_cp()
    seed_est = TorqueEstimator(CP)
    lafs, frictions = in_bounds_values(seed_est)
    lafs[1] = float('nan')
    cache = make_cache(decay=173.25).to_bytes()
    cache_sp = make_cache_sp(seed_est.speed_bin_centers, lafs, frictions, points=_one_point_per_bin(seed_est)).to_bytes()
    est = self._restore_from(fake_params, CP, cache, cache_sp)
    assert_untouched(est, seed_values(seed_est))


class TestPointsCacheWrite:
  """torqued's cache write (get_msg with_points=True) is what feeds the fork cache."""

  @staticmethod
  def _two_points(est):
    centers, bounds = get_car_bins(SPEED_DEP_FINGERPRINT)
    est._on_torque_point(0.1, 0.3, (bounds[0][0] + bounds[0][1]) / 2)
    est._on_torque_point(0.2, 0.4, (bounds[-1][0] + bounds[-1][1]) / 2)
    return bounds

  def test_live_message_never_carries_points(self, fake_params):
    est = TorqueEstimator(make_cp())
    est._pm = FakePubMaster()
    self._two_points(est)
    est.get_msg(with_points=False)
    assert len(est._pm.last().speedBinPoints) == 0
    assert LIVE_TORQUE_PARAMETERS_SP_KEY not in fake_params.store
    est.frame += 1
    est.get_msg(with_points=True)
    assert len(est._pm.last().speedBinPoints) == 0
    assert LIVE_TORQUE_PARAMETERS_SP_KEY in fake_params.store

  def test_cache_write_persists_fork_key(self, fake_params):
    est = TorqueEstimator(make_cp())
    est._pm = FakePubMaster()
    bounds = self._two_points(est)
    est.get_msg(with_points=True)

    with log.Event.from_bytes(fake_params.store[LIVE_TORQUE_PARAMETERS_SP_KEY]) as evt:
      sp = getattr(evt, LIVE_TORQUE_PARAMETERS_SP_SERVICE)
      assert sp.version == VERSION
      assert sp.seedVersion == est.speed_dep_seed_version == seed_version_of(est.CP.carFingerprint)
      assert list(sp.speedBinCenters) == pytest.approx(est.speed_bin_centers)
      assert len(sp.speedBinLatAccelFactors) == len(bounds)
      assert len(sp.speedBinPoints) == len(bounds)
      assert sum(len(bin_pts) for bin_pts in sp.speedBinPoints) >= 2

  def test_written_points_round_trip_into_a_fresh_estimator(self, fake_params):
    CP = make_cp()
    est = TorqueEstimator(CP)
    est._pm = FakePubMaster()
    self._two_points(est)
    cache = est.get_msg(with_points=True).to_bytes()
    written = [b.get_points()[:, [0, 2]].tolist() for b in est.speed_bin_points]

    fake_params.store["LiveTorqueParameters"] = cache
    fake_params.store["CarParamsPrevRoute"] = CP.to_bytes()
    restored = TorqueEstimator(CP)
    for want, bucket in zip(written, restored.speed_bin_points, strict=True):
      got = np.asarray(bucket.get_points()[:, [0, 2]], dtype=float).reshape(-1, 2)
      np.testing.assert_allclose(got, np.asarray(want, dtype=float).reshape(-1, 2), rtol=1e-6)  # one Float32 trip
