"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.structs import car
from openpilot.cereal import log, custom

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD

RELAXED_MIN_BUCKET_POINTS = np.array([1, 200, 300, 500, 500, 300, 200, 1])

ALLOWED_CARS = ['toyota', 'hyundai', 'rivian', 'honda']

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


class TorqueEstimatorExt:
  """Per-speed-bin torque learning, mixed into TorqueEstimator.

  Each bin runs upstream's total-least-squares fit on the quality-filtered points that fall
  in its speed range and publishes its own latAccelFactor and friction; the controller
  interpolates them by speed. Bins come from speed_dependent.toml, or the defaults above
  seeded with the car's global offline values. Gated by SpeedDependentTorqueToggle.
  """

  def __init__(self, CP: car.CarParams):
    self.CP = CP
    self._params = Params()
    self.frame = -1

    self.enforce_torque_control_toggle = self._params.get_bool("EnforceTorqueControl")  # only during init
    self.use_params = self.CP.brand in ALLOWED_CARS and self.CP.lateralTuning.which() == 'torque'
    self.use_live_torque_params = self._params.get_bool("LiveTorqueParamsToggle")
    self.custom_torque_params = self._params.get_bool("CustomTorqueParams")
    self.torque_override_enabled = self._params.get_bool("TorqueParamsOverrideEnabled")
    # Speed-dep extends the self-tune learner, so it needs the same toggles the UI requires
    # before its own; without them the per-bin fits would run with no consumer. The
    # ALLOWED_CARS brand gate above is deliberately separate.
    self.speed_binned = (self.CP.lateralTuning.which() == 'torque'
                         and self._params.get_bool("SpeedDependentTorqueToggle")
                         and self.enforce_torque_control_toggle
                         and self.use_live_torque_params)
    # overwritten by TorqueEstimator.__init__ before initialize_custom_params runs
    self.min_bucket_points = RELAXED_MIN_BUCKET_POINTS
    self.factor_sanity = 0.0
    self.friction_sanity = 0.0
    self.offline_latAccelFactor = 0.0
    self.offline_friction = 0.0
    # the fork message goes out once per frame get_msg runs, which is upstream's publish cadence
    self._pm = None
    self._sp_pub_frame = -1

  def initialize_custom_params(self, decimated=False):
    self.update_use_params()

    if self.enforce_torque_control_toggle:
      if self._params.get_bool("LiveTorqueParamsRelaxedToggle"):
        self.min_bucket_points = RELAXED_MIN_BUCKET_POINTS / (10 if decimated else 1)
        self.factor_sanity = 0.5 if decimated else 1.0
        self.friction_sanity = 0.8 if decimated else 1.0

      if self._params.get_bool("CustomTorqueParams"):
        self.offline_latAccelFactor = float(self._params.get("TorqueParamsOverrideLatAccelFactor", return_default=True))
        self.offline_friction = float(self._params.get("TorqueParamsOverrideFriction", return_default=True))

    # bins and their cache must exist before the first get_msg
    if self.speed_binned:
      self._post_reset()
      self._restore_ext_cache()

  def _update_params(self):
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.use_live_torque_params = self._params.get_bool("LiveTorqueParamsToggle")
      self.custom_torque_params = self._params.get_bool("CustomTorqueParams")
      self.torque_override_enabled = self._params.get_bool("TorqueParamsOverrideEnabled")

  def update_use_params(self):
    self._update_params()

    if self.enforce_torque_control_toggle:
      if self.custom_torque_params and self.torque_override_enabled:
        self.use_params = False
      else:
        self.use_params = self.use_live_torque_params

    self.frame += 1


  @staticmethod
  def _centers_to_bounds(centers):
    """Bin bounds at the midpoints between consecutive centers; the outer edges take the
    default range (5 to 40 m/s)."""
    bounds = []
    for i, c in enumerate(centers):
      lo = DEFAULT_SPEED_BIN_BOUNDS[0][0] if i == 0 else (centers[i - 1] + c) / 2
      hi = DEFAULT_SPEED_BIN_BOUNDS[-1][1] if i == len(centers) - 1 else (c + centers[i + 1]) / 2
      bounds.append((lo, hi))
    return bounds

  def _post_reset(self):
    """Builds the per-bin buckets, filters and sanity bounds. Runs after factor_sanity and the
    offline values are set."""
    if not self.speed_binned:
      return

    from openpilot.selfdrive.locationd.torqued import TorqueBuckets, STEER_BUCKET_BOUNDS, \
      POINTS_PER_BUCKET, MIN_FILTER_DECAY
    from opendbc.sunnypilot.car.interfaces import get_speed_dep_config_for_car

    cfg = get_speed_dep_config_for_car(self.CP)
    # the TOML entry's seed_version, bumped with a seed refresh to retire every cache learned
    # under the old seeds; 0 for an entry without one and for the defaults
    self.speed_dep_seed_version = int(cfg.get('seed_version', 0))

    if 'speed_bp' in cfg:
      self.speed_bin_centers = list(cfg['speed_bp'])
      self.speed_bin_bounds = self._centers_to_bounds(self.speed_bin_centers)
    else:
      self.speed_bin_bounds = list(DEFAULT_SPEED_BIN_BOUNDS)
      self.speed_bin_centers = list(DEFAULT_SPEED_BIN_CENTERS)

    n_bins = len(self.speed_bin_bounds)

    self.speed_bin_points = [self._make_speed_bin_bucket(TorqueBuckets, STEER_BUCKET_BOUNDS, POINTS_PER_BUCKET) for _ in range(n_bins)]
    self._speed_bin_last_len = [0] * n_bins
    self._speed_bin_last_valid = [False] * n_bins

    # seeds from the TOML entry, else the global offline values for every bin
    ref_lafs = cfg.get('laf_bp', [self.offline_latAccelFactor] * n_bins)
    ref_frictions = cfg.get('friction_bp', [self.offline_friction] * n_bins)
    self.speed_bin_decays = [MIN_FILTER_DECAY] * n_bins
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

  def _make_speed_bin_bucket(self, TorqueBuckets, STEER_BUCKET_BOUNDS, POINTS_PER_BUCKET):
    """One speed bin's buckets. Each bin sees a fraction of the data, so the per-bucket
    minimums are the global learner's divided by the bin count."""
    # min_bucket_points is a plain list upstream; coerce before the integer divide
    scaled_min = np.maximum(np.asarray(self.min_bucket_points) // len(self.speed_bin_bounds), 1)
    return TorqueBuckets(x_bounds=STEER_BUCKET_BOUNDS,
                         min_points=scaled_min,
                         min_points_total=int(scaled_min.sum()),
                         points_per_bucket=POINTS_PER_BUCKET,
                         rowsize=3)

  def _on_torque_point(self, steer, lateral_acc, vego):
    """Routes a quality-filtered point from handle_log to its speed bin."""
    if not self.speed_binned:
      return
    for i, (lo, hi) in enumerate(self.speed_bin_bounds):
      if lo <= vego < hi:
        self.speed_bin_points[i].add_point(steer, lateral_acc)
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
      from openpilot.selfdrive.locationd.torqued import TorqueEstimator, VERSION, MIN_FILTER_DECAY, MAX_FILTER_DECAY

      if TorqueEstimator.get_restore_key(cache_CP, cache_ltp.version) != TorqueEstimator.get_restore_key(self.CP, VERSION):
        cloudlog.info("speed-dep: cache restore key mismatch, restarting learning")
        return
      if cache_sp is None:
        cache = self._params.get(LIVE_TORQUE_PARAMETERS_SP_KEY)
        if not cache:
          cloudlog.info("speed-dep: no LiveTorqueParametersSP cache, restarting learning")
          return
        with log.Event.from_bytes(cache) as evt:
          cache_sp = getattr(evt, LIVE_TORQUE_PARAMETERS_SP_SERVICE)
      if TorqueEstimator.get_restore_key(cache_CP, cache_sp.version) != TorqueEstimator.get_restore_key(self.CP, VERSION):
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
      decay = float(np.clip(decay, MIN_FILTER_DECAY, MAX_FILTER_DECAY))

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
    from openpilot.selfdrive.locationd.torqued import VERSION
    msg = messaging.new_message(LIVE_TORQUE_PARAMETERS_SP_SERVICE)
    msg.valid = valid
    sp = getattr(msg, LIVE_TORQUE_PARAMETERS_SP_SERVICE)
    sp.version = VERSION
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
    from openpilot.selfdrive.locationd.torqued import TorqueBuckets, STEER_BUCKET_BOUNDS, \
      POINTS_PER_BUCKET, FRICTION_FACTOR, slope2rot, MIN_FILTER_DECAY, MAX_FILTER_DECAY

    results = []
    for i, bucket in enumerate(self.speed_bin_points):
      if not bucket.is_calculable():
        results.append((i, False))
        continue

      # nothing new since the last fit
      cur_len = len(bucket)
      if cur_len == self._speed_bin_last_len[i]:
        results.append((i, self._speed_bin_last_valid[i]))
        continue

      # self.fit_points honors the decimated (qlog) point count
      points = bucket.get_points(self.fit_points)
      try:
        _, _, v = np.linalg.svd(points, full_matrices=False)
        slope, offset = -v.T[0:2, 2] / v.T[2, 2]  # slope = latAccelFactor
        _, spread = np.matmul(points[:, [0, 2]], slope2rot(slope)).T
        friction_coeff = np.std(spread) * FRICTION_FACTOR
        if not any(np.isnan(val) for val in [slope, friction_coeff]):
          factor_lo, factor_hi = self.speed_bin_lat_accel_factor_bounds[i]
          fric_lo, fric_hi = self.speed_bin_friction_bounds[i]
          self.speed_bin_decays[i] = min(self.speed_bin_decays[i] + DT_MDL, MAX_FILTER_DECAY)  # slow down filter over time
          self.speed_bin_filtered[i]['latAccelFactor'].update(np.clip(slope, factor_lo, factor_hi))
          self.speed_bin_filtered[i]['latAccelFactor'].update_alpha(self.speed_bin_decays[i])
          self.speed_bin_filtered[i]['frictionCoefficient'].update(np.clip(friction_coeff, fric_lo, fric_hi))
          self.speed_bin_filtered[i]['frictionCoefficient'].update_alpha(self.speed_bin_decays[i])
          self._speed_bin_last_len[i] = cur_len
          self._speed_bin_last_valid[i] = bucket.is_valid()
          results.append((i, self._speed_bin_last_valid[i]))
          continue
      except np.linalg.LinAlgError:
        pass

      if bucket.is_valid():
        cloudlog.warning(f"speed-dep: bin {i} produced NaN with valid data, resetting bin")
        self.speed_bin_points[i] = self._make_speed_bin_bucket(TorqueBuckets, STEER_BUCKET_BOUNDS, POINTS_PER_BUCKET)
        self.speed_bin_decays[i] = MIN_FILTER_DECAY
        self._speed_bin_last_len[i] = 0
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
