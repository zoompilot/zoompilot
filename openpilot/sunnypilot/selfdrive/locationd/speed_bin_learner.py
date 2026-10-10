"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.structs import car
from openpilot.cereal import log, custom

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog

# Speed bins for cars without a speed_dependent.toml entry
DEFAULT_SPEED_BIN_BOUNDS = [(5, 8), (8, 12), (12, 18), (18, 24), (24, 29), (29, 35), (35, 40)]
DEFAULT_SPEED_BIN_CENTERS = [6.5, 10.0, 15.0, 21.0, 26.5, 32.0, 37.5]

# The fork's own message and cache. The per-bin values ride on liveTorqueParametersSP, which
# is customReserved19 on the wire (the last of sunnypilot's reserved Event slots, so log.capnp
# stays upstream's), published beside every lateralTorqueParameters at the same cadence and
# validity. torqued's 60 s cache write serializes the same struct, plus the per-bin point
# buckets, into LiveTorqueParametersSP; the buckets are thousands of points and only the
# restore path reads them, so the wire copy leaves them empty. See docs/zoompilot/lateral-tune.md.
LIVE_TORQUE_PARAMETERS_SP_SERVICE = "customReserved19"
LIVE_TORQUE_PARAMETERS_SP_KEY = "LiveTorqueParametersSP"
LiveTorqueParametersSP = custom.CustomReserved19


def _torqued():
  # torqued imports this module through torqued_ext, so its names resolve at call time
  from openpilot.selfdrive.locationd import torqued
  return torqued


def fit_torque_points(points):
  """Upstream's estimate_params on one point set ([steer, 1, lateral accel] rows): the TLS slope
  (latAccelFactor), its offset and the spread friction. Raises LinAlgError as the SVD does."""
  torqued = _torqued()
  _, _, v = np.linalg.svd(points, full_matrices=False)
  slope, offset = -v.T[0:2, 2] / v.T[2, 2]
  _, spread = np.matmul(points[:, [0, 2]], torqued.slope2rot(slope)).T
  return slope, offset, np.std(spread) * torqued.FRICTION_FACTOR


class SpeedBinLearner:
  """Per-speed-bin torque learning, mixed into TorqueEstimator through TorqueEstimatorExt.

  Each bin runs upstream's total-least-squares fit on the quality-filtered points that fall
  in its speed range and publishes its own latAccelFactor and friction; the controller
  interpolates them by speed. Bins come from speed_dependent.toml, or the defaults above
  seeded with the car's global offline values. Runs wherever self-tune does.
  """
  speed_binned = False  # set by initialize_custom_params, once torqued has set use_params
  # the fork message goes out once per frame get_msg runs, which is upstream's publish cadence
  _pm = None
  _sp_pub_frame = -1

  @staticmethod
  def _centers_to_bounds(centers, min_speed=None):
    """Bin bounds at the midpoints between consecutive centers; the outer edges take min_speed
    (the config's, when it dropped bins below a steering floor) or the default range's 5 m/s,
    and the default range's 40 m/s."""
    bounds = []
    for i, c in enumerate(centers):
      lo = (DEFAULT_SPEED_BIN_BOUNDS[0][0] if min_speed is None else min_speed) if i == 0 else (centers[i - 1] + c) / 2
      hi = DEFAULT_SPEED_BIN_BOUNDS[-1][1] if i == len(centers) - 1 else (c + centers[i + 1]) / 2
      bounds.append((lo, hi))
    return bounds

  def _init_speed_bins(self):
    """Builds the per-bin buckets, filters and sanity bounds, then restores the cache. Runs
    after factor_sanity and the offline values are set, before the first get_msg."""
    if not self.speed_binned:
      return

    from opendbc.sunnypilot.car.lateral_tune import get_speed_dep_config_for_car

    cfg = get_speed_dep_config_for_car(self.CP)
    # the TOML entry's seed_version, bumped with a seed refresh to retire every cache learned
    # under the old seeds; 0 for an entry without one and for the defaults
    self.speed_dep_seed_version = int(cfg.get('seed_version', 0))

    if 'speed_bp' in cfg:
      self.speed_bin_centers = list(cfg['speed_bp'])
      self.speed_bin_bounds = self._centers_to_bounds(self.speed_bin_centers, cfg.get('min_speed'))
    else:
      self.speed_bin_bounds = list(DEFAULT_SPEED_BIN_BOUNDS)
      self.speed_bin_centers = list(DEFAULT_SPEED_BIN_CENTERS)

    n_bins = len(self.speed_bin_bounds)

    self.speed_bin_points = [self._make_speed_bin_bucket() for _ in range(n_bins)]
    # a bin with points since its last fit: a full bin's length stops changing while its ring
    # buffers keep turning over, so the length cannot mark new data
    self._speed_bin_dirty = [True] * n_bins
    self._speed_bin_last_valid = [False] * n_bins

    # seeds from the TOML entry, else the global offline values for every bin
    ref_lafs = cfg.get('laf_bp', [self.offline_latAccelFactor] * n_bins)
    ref_frictions = cfg.get('friction_bp', [self.offline_friction] * n_bins)
    self.speed_bin_decays = [_torqued().MIN_FILTER_DECAY] * n_bins
    self.speed_bin_filtered = [
      {'latAccelFactor': FirstOrderFilter(ref_lafs[i], self.speed_bin_decays[i], DT_MDL),
       'frictionCoefficient': FirstOrderFilter(ref_frictions[i], self.speed_bin_decays[i], DT_MDL)}
      for i in range(n_bins)
    ]
    # the fits are clipped to +-sanity of the seed, as upstream clips its global fit
    self.speed_bin_lat_accel_factor_bounds = [
      ((1.0 - self.factor_sanity) * factor, (1.0 + self.factor_sanity) * factor)
      for factor in ref_lafs
    ]
    self.speed_bin_friction_bounds = [
      ((1.0 - self.friction_sanity) * f, (1.0 + self.friction_sanity) * f)
      for f in ref_frictions
    ]
    self._restore_ext_cache()

  def _make_speed_bin_bucket(self):
    """One speed bin's buckets. Each bin sees a fraction of the data, so the per-bucket
    minimums are the global learner's divided by the bin count."""
    torqued = _torqued()
    # min_bucket_points is a plain list upstream; coerce before the integer divide
    scaled_min = np.maximum(np.asarray(self.min_bucket_points) // len(self.speed_bin_bounds), 1)
    return torqued.TorqueBuckets(x_bounds=torqued.STEER_BUCKET_BOUNDS,
                                 min_points=scaled_min,
                                 min_points_total=int(scaled_min.sum()),
                                 points_per_bucket=torqued.POINTS_PER_BUCKET,
                                 rowsize=3)

  def _on_torque_point(self, steer, lateral_acc, vego):
    """Routes a quality-filtered point from handle_log to its speed bin."""
    if not self.speed_binned:
      return
    for i, (lo, hi) in enumerate(self.speed_bin_bounds):
      if lo <= vego < hi:
        self.speed_bin_points[i].add_point(steer, lateral_acc)
        self._speed_bin_dirty[i] = True
        break

  @staticmethod
  def _within_bounds(vals, bounds):
    """True when every value is finite and inside its bin's (lo, hi) clip range. The wire is
    Float32, so a filter sitting on a bound can read back a rounding step past it."""
    vals = np.asarray(vals, dtype=float)
    lo, hi = np.asarray(bounds, dtype=float).T
    tol = 1e-5 * np.maximum(np.abs(hi), 1.0)
    return bool(np.all(np.isfinite(vals)) and np.all(vals >= lo - tol) and np.all(vals <= hi + tol))

  def _centers_match(self, cached_centers) -> bool:
    # a legacy cache has no centers; length first so allclose never sees a shape mismatch
    cached = list(cached_centers)
    return len(cached) == len(self.speed_bin_centers) and bool(np.allclose(cached, self.speed_bin_centers, atol=0.01))

  def _restore_ext_cache(self, cache_ltp=None, cache_CP=None, cache_sp=None):
    """Restores the per-bin filters, decay and point buckets from the two caches: upstream's
    LiveTorqueParameters supplies the restore key, decay and the valid flag; the fork's
    LiveTorqueParametersSP supplies its own VERSION, the seed version, the bin centers, the
    values and the points. Both must carry this car's restore key (fingerprint, tuning type,
    offline seeds, VERSION) and the fork cache this config's seed_version and bin centers; the
    filtered values are taken only
    when upstream's cache was written valid, and since the bins are one tune a single bad
    value rejects them whole. Points are restored on top when they pass their own checks, and
    are simply skipped when they fail them. Reads from Params for whichever argument is None."""
    if not self.speed_binned:
      return
    try:
      if cache_ltp is None:
        cache = self._params.get("LiveTorqueParameters")
        if not cache:
          return
        with log.Event.from_bytes(cache) as evt:
          cache_ltp = evt.lateralTorqueParameters
      if cache_CP is None:
        params_cache = self._params.get("CarParamsPrevRoute")
        if not params_cache:
          cloudlog.info("speed-dep: no CarParamsPrevRoute, restarting learning")
          return
        with car.CarParams.from_bytes(params_cache) as msg:
          cache_CP = msg
      torqued = _torqued()
      restore_key = torqued.TorqueEstimator.get_restore_key

      if restore_key(cache_CP, cache_ltp.version) != restore_key(self.CP, torqued.VERSION):
        cloudlog.info("speed-dep: cache restore key mismatch, restarting learning")
        return
      if cache_sp is None:
        cache = self._params.get(LIVE_TORQUE_PARAMETERS_SP_KEY)
        if not cache:
          cloudlog.info("speed-dep: no LiveTorqueParametersSP cache, restarting learning")
          return
        with log.Event.from_bytes(cache) as evt:
          cache_sp = getattr(evt, LIVE_TORQUE_PARAMETERS_SP_SERVICE)
      if restore_key(cache_CP, cache_sp.version) != restore_key(self.CP, torqued.VERSION):
        cloudlog.info("speed-dep: fork cache restore key mismatch, restarting learning")
        return
      if cache_sp.seedVersion != self.speed_dep_seed_version:
        cloudlog.info(f"speed-dep: seed version {cache_sp.seedVersion} -> {self.speed_dep_seed_version}, restarting learning")
        return
      n_bins = len(self.speed_bin_bounds)
      if not self._centers_match(cache_sp.speedBinCenters):
        cloudlog.info("speed-dep: config changed, restarting learning")
        return
      cached_lafs = list(cache_sp.speedBinLatAccelFactors)
      cached_frictions = list(cache_sp.speedBinFrictions)
      if len(cached_lafs) != n_bins or len(cached_frictions) != n_bins:
        cloudlog.info("speed-dep: cache bin count mismatch, restarting learning")
        return
      if (not self._within_bounds(cached_lafs, self.speed_bin_lat_accel_factor_bounds)
          or not self._within_bounds(cached_frictions, self.speed_bin_friction_bounds)):
        cloudlog.warning("speed-dep: cached bin values non-finite or outside sanity bounds, restarting learning")
        return
      decay = float(cache_ltp.decay)
      if not np.isfinite(decay):
        cloudlog.warning("speed-dep: cached decay non-finite, restarting learning")
        return
      # torqued only writes a decay inside this range; the clip guards a hand-edited cache
      decay = float(np.clip(decay, torqued.MIN_FILTER_DECAY, torqued.MAX_FILTER_DECAY))

      # values only from a valid cache, as upstream does for its globals
      if cache_ltp.valid:
        for i in range(n_bins):
          self.speed_bin_filtered[i]['latAccelFactor'].x = cached_lafs[i]
          self.speed_bin_filtered[i]['frictionCoefficient'].x = cached_frictions[i]
      else:
        cloudlog.info("speed-dep: cache not valid, keeping seed values")
      cached_points = self._load_points_cache(cache_sp, n_bins)
      if cached_points is not None:
        for i in range(n_bins):
          self.speed_bin_points[i].load_points(cached_points[i])
      # one decay on the wire (upstream's), so every bin resumes at it rather than at MIN
      self.speed_bin_decays = [decay] * n_bins
      for filters in self.speed_bin_filtered:
        filters['latAccelFactor'].update_alpha(decay)
        filters['frictionCoefficient'].update_alpha(decay)
      cloudlog.info("restored speed-bin torque params from cache")
    except Exception:
      cloudlog.exception("speed-dep: failed to restore cache")

  def _load_points_cache(self, cache_sp, n_bins):
    """The per-bin points from the fork cache, or None when they fail the bin-count or
    finiteness checks (the restore key and bin centers were checked on the same struct
    already). A failure here never touches the values restore: the learner just starts its
    buckets empty."""
    try:
      points = [[list(point) for point in bin_points] for bin_points in cache_sp.speedBinPoints]
      if len(points) != n_bins:
        cloudlog.info("speed-dep: points cache bin count mismatch, points not restored")
        return None
      if not all(np.all(np.isfinite(np.asarray(bin_points, dtype=float))) for bin_points in points if bin_points):
        cloudlog.warning("speed-dep: cached bin points non-finite, points not restored")
        return None
      return points
    except Exception:
      cloudlog.exception("speed-dep: failed to read points cache, points not restored")
      return None

  def _sp_msg(self, valid, values, with_points):
    """A liveTorqueParametersSP event: VERSION, the seed version, the bin centers and the
    per-bin values, plus the point buckets for the cache copy. Empty bins on a car that is
    not speed-binned."""
    msg = messaging.new_message(LIVE_TORQUE_PARAMETERS_SP_SERVICE)
    msg.valid = valid
    sp = getattr(msg, LIVE_TORQUE_PARAMETERS_SP_SERVICE)
    sp.version = _torqued().VERSION
    if values is not None:
      sp.seedVersion = self.speed_dep_seed_version
      lat_factors, frictions, valid_flags = values
      sp.speedBinCenters = self.speed_bin_centers
      sp.speedBinLatAccelFactors = lat_factors
      sp.speedBinFrictions = frictions
      sp.speedBinValid = valid_flags
      if with_points:
        sp.speedBinPoints = [bucket.get_points()[:, [0, 2]].tolist() for bucket in self.speed_bin_points]
    return msg

  def _estimate_params_speed_binned(self):
    """Independent total-least-squares fit per bin, upstream's estimate_params() per bucket
    set. A bin that goes NaN with valid data is reset, as upstream resets its global fit."""
    torqued = _torqued()

    results = []
    for i, bucket in enumerate(self.speed_bin_points):
      if not bucket.is_calculable():
        results.append((i, False))
        continue

      # nothing new since the last fit
      if not self._speed_bin_dirty[i]:
        results.append((i, self._speed_bin_last_valid[i]))
        continue

      # self.fit_points honors the decimated (qlog) point count
      points = bucket.get_points(self.fit_points)
      try:
        slope, _, friction_coeff = fit_torque_points(points)  # slope = latAccelFactor
        if not any(np.isnan(val) for val in [slope, friction_coeff]):
          factor_lo, factor_hi = self.speed_bin_lat_accel_factor_bounds[i]
          fric_lo, fric_hi = self.speed_bin_friction_bounds[i]
          self.speed_bin_decays[i] = min(self.speed_bin_decays[i] + DT_MDL, torqued.MAX_FILTER_DECAY)  # slow down filter over time
          self.speed_bin_filtered[i]['latAccelFactor'].update(np.clip(slope, factor_lo, factor_hi))
          self.speed_bin_filtered[i]['latAccelFactor'].update_alpha(self.speed_bin_decays[i])
          self.speed_bin_filtered[i]['frictionCoefficient'].update(np.clip(friction_coeff, fric_lo, fric_hi))
          self.speed_bin_filtered[i]['frictionCoefficient'].update_alpha(self.speed_bin_decays[i])
          self._speed_bin_dirty[i] = False
          self._speed_bin_last_valid[i] = bucket.is_valid()
          results.append((i, self._speed_bin_last_valid[i]))
          continue
      except np.linalg.LinAlgError:
        pass

      if bucket.is_valid():
        cloudlog.warning(f"speed-dep: bin {i} produced NaN with valid data, resetting bin")
        self.speed_bin_points[i] = self._make_speed_bin_bucket()
        self.speed_bin_decays[i] = torqued.MIN_FILTER_DECAY
        self._speed_bin_dirty[i] = True
      self._speed_bin_last_valid[i] = False
      results.append((i, False))
    return results

  def _extend_msg(self, msg, with_points):
    """torqued's get_msg hook. Publishes the fork message beside the upstream one, once per
    frame at the same validity, and on torqued's cache write (with_points) persists the
    same struct with the point buckets under the fork's own key. The wire copy never
    carries points."""
    values = None
    if self.speed_binned:
      bin_results = self._estimate_params_speed_binned()
      n_bins = len(self.speed_bin_bounds)
      lat_factors, frictions, valid_flags = [], [], []
      for i in range(n_bins):
        lat_factors.append(float(self.speed_bin_filtered[i]['latAccelFactor'].x))
        frictions.append(float(self.speed_bin_filtered[i]['frictionCoefficient'].x))
        valid_flags.append(bin_results[i][1])
      values = (lat_factors, frictions, valid_flags)

    if self._sp_pub_frame != self.frame:
      self._sp_pub_frame = self.frame
      if self._pm is None:
        self._pm = messaging.PubMaster([LIVE_TORQUE_PARAMETERS_SP_SERVICE])
      self._pm.send(LIVE_TORQUE_PARAMETERS_SP_SERVICE, self._sp_msg(msg.valid, values, with_points=False))
    if with_points and self.speed_binned:
      self._params.put(LIVE_TORQUE_PARAMETERS_SP_KEY, self._sp_msg(msg.valid, values, with_points=True).to_bytes())
