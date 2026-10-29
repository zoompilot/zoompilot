"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Per-car lateral ceiling for the vision curve planner, learned from steering saturation.

The shared curve table is one lateral acceleration for every car, but what a car can hold
through a curve is its EPS torque, and that differs by model. The learner starts each car on
the shared table and moves the ceiling, per speed breakpoint, from what the lateral controller
reports in the curves it drives:

- a curve that saturates the steering (the controller's own flag, which "Turn Exceeds Steering
  Limit" is raised on) is evidence the car cannot hold the lateral acceleration it was
  delivering, so the ceiling drops under it, and the raise step at that speed halves;
- a curve driven near the ceiling without saturating is evidence the car holds it, so the
  ceiling rises by the current step.

The step shrinks with every saturation, so the ceiling closes in on the highest lateral
acceleration the steering holds, from below, and the warning becomes rarer as it converges.
Learned values persist per fingerprint. See docs/zoompilot/scc-curve-planning.md.
"""
import json

import numpy as np

from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog

PARAM_KEY = "CurveLatAccelLimit"
# bump to retire every learned ceiling, e.g. when the saturation test or the steps change
VERSION = 1

# Bounds on the learned ceiling, m/s2, before the planning margin. The top is ISO 11270's
# lateral limit, which openpilot's lateral control enforces anyway; past it a curve saturates
# on curvature_limited however strong the EPS is.
A_LAT_MIN = 1.2
A_LAT_MAX = 3.0

# selfdrived's steerSaturated gates: turning, no recent driver steering. Its third, the car 1.2x
# short of the model's demand, is left out: a ceiling held just under it would leave the EPS at
# full torque and the car running up to 20% wide of its line, so the controller's saturation
# alone counts, and the ceiling converges to the torque limit with the warning clear of it.
_TURNING_LAT_ACC = 1.0  # m/s2 desired
_STEER_PRESSED_HOLD = 2.0  # s; a driver's steering input excludes the next 2 s, as there

# Saturation: the ceiling at that speed drops to the lateral acceleration the car was delivering
# at full torque, less a backoff, by at most _DOWN_MAX per curve (a crosswind or crown can
# saturate a curve the torque alone would hold). A car carried well past the ceiling (the
# driver's gas, a bend the planner did not see) says nothing about it and is skipped.
_SAT_BACKOFF = 0.1  # m/s2
_DOWN_MIN = 0.05  # m/s2; a saturation always lowers the ceiling at least this much
_DOWN_MAX = 0.2  # m/s2
_OVER_CEILING = 1.1  # delivered over this fraction of the ceiling: not the ceiling's curve

# Clean curves: one driven past this fraction of the ceiling (the planner plans the apex at 95%
# of it) for at least _MIN_CURVE_T without saturating raises the ceiling by the step.
_PROBE_FRAC = 0.85
_MIN_CURVE_T = 1.0  # s
_STEP_INIT = 0.05  # m/s2
_STEP_MIN = 0.002  # m/s2; never zero, so a ceiling lowered by a one-off keeps recovering
_STEP_DECAY = 0.5  # the step at a saturated speed is multiplied by this


class LatLimitLearner:
  def __init__(self, bp, prior, fingerprint: str, params=None):
    self.bp = np.asarray(bp, dtype=float)
    self.prior = np.clip(np.asarray(prior, dtype=float), A_LAT_MIN, A_LAT_MAX)
    self.fingerprint = str(fingerprint)
    self.params = params
    self.ceiling = self.prior.copy()
    self.step = np.full(len(self.bp), _STEP_INIT)
    self.learned = False  # moved off the shared table; until then the planner keeps it
    self.saturations = 0
    self.curve_ok = False  # this curve was entered engaged; one the driver started never counts
    self._reset_curve()
    self.steer_pressed_t = _STEER_PRESSED_HOLD
    self._restore()

  def _reset_curve(self) -> None:
    self.curve_t = 0.
    self.peak_lat = 0.
    self.peak_v = 0.
    self.curve_saturated = False

  def weights(self, v: float) -> np.ndarray:
    """Linear-interpolation weights of v over the breakpoints, flat past either end."""
    w = np.zeros(len(self.bp))
    if v <= self.bp[0]:
      w[0] = 1.
    elif v >= self.bp[-1]:
      w[-1] = 1.
    else:
      i = int(np.searchsorted(self.bp, v)) - 1
      f = (v - self.bp[i]) / (self.bp[i + 1] - self.bp[i])
      w[i], w[i + 1] = 1. - f, f
    return w

  def ceiling_at(self, v: float) -> float:
    return float(np.interp(v, self.bp, self.ceiling))

  def update(self, v_ego: float, actual_lat: float, desired_lat: float, lat_active: bool,
             saturated: bool, steering_pressed: bool) -> None:
    """One model frame. actual_lat and desired_lat are v^2 * curvature, m/s2, of the car and the model."""
    self.steer_pressed_t = 0. if steering_pressed else self.steer_pressed_t + DT_MDL
    engaged = lat_active and self.steer_pressed_t >= _STEER_PRESSED_HOLD
    actual_lat, desired_lat = abs(actual_lat), abs(desired_lat)

    if not engaged:
      # the driver took the curve (or lateral dropped out): nothing clean to learn from it
      self.curve_ok = False
      self._reset_curve()
      return

    if desired_lat <= _TURNING_LAT_ACC:
      if self.curve_ok:
        self._end_curve()
      self.curve_ok = True
      return
    if not self.curve_ok:
      return

    self.curve_t += DT_MDL
    if actual_lat > self.peak_lat:
      self.peak_lat, self.peak_v = actual_lat, v_ego

    if saturated and not self.curve_saturated:
      # once per curve: the controller's saturation timer already wants it sustained
      self.curve_saturated = True
      self._on_saturation(v_ego, actual_lat)

  def _end_curve(self) -> None:
    if not self.curve_saturated and self.curve_t >= _MIN_CURVE_T and self.peak_lat >= _PROBE_FRAC * self.ceiling_at(self.peak_v):
      w = self.weights(self.peak_v)
      self._apply(self.ceiling + w * self.step, f"held {self.peak_lat:.2f} m/s2 at {self.peak_v:.1f} m/s")
    self._reset_curve()

  def _on_saturation(self, v: float, delivered: float) -> None:
    self.saturations += 1
    ceiling_v = self.ceiling_at(v)
    if delivered > _OVER_CEILING * ceiling_v:
      return
    w = self.weights(v)
    drop = float(np.clip(ceiling_v - (delivered - _SAT_BACKOFF), _DOWN_MIN, _DOWN_MAX))
    self.step = np.maximum(self.step * (1. - w * (1. - _STEP_DECAY)), _STEP_MIN)
    self._apply(self.ceiling - w * drop, f"saturated at {delivered:.2f} m/s2, {v:.1f} m/s")

  def _apply(self, ceiling, why: str) -> None:
    ceiling = np.clip(ceiling, A_LAT_MIN, A_LAT_MAX)
    if np.allclose(ceiling, self.ceiling):
      return
    self.ceiling = ceiling
    self.learned = True
    cloudlog.info(f"curve lat limit: {why} -> {np.round(self.ceiling, 3).tolist()}")
    self._save()

  def _save(self) -> None:
    if self.params is None:
      return
    try:
      self.params.put(PARAM_KEY, {"version": VERSION, "fingerprint": self.fingerprint, "bp": self.bp.tolist(),
                                  "ceiling": self.ceiling.tolist(), "step": self.step.tolist()})
    except Exception:
      cloudlog.exception("curve lat limit: failed to save")

  def _restore(self) -> None:
    """The car's own learned ceiling, when the cache is this version, this fingerprint and these
    breakpoints and its values are finite; otherwise the car starts on the shared table."""
    if self.params is None:
      return
    try:
      cache = self.params.get(PARAM_KEY)
      if not cache:
        return
      if isinstance(cache, (bytes, str)):
        cache = json.loads(cache)
      if cache.get("version") != VERSION or cache.get("fingerprint") != self.fingerprint:
        cloudlog.info("curve lat limit: cache is another car's or version, starting on the shared table")
        return
      bp = np.asarray(cache["bp"], dtype=float)
      ceiling = np.asarray(cache["ceiling"], dtype=float)
      step = np.asarray(cache["step"], dtype=float)
      if bp.shape != self.bp.shape or not np.allclose(bp, self.bp) or ceiling.shape != bp.shape or step.shape != bp.shape:
        cloudlog.info("curve lat limit: breakpoints changed, starting on the shared table")
        return
      if not (np.all(np.isfinite(ceiling)) and np.all(np.isfinite(step))):
        cloudlog.warning("curve lat limit: cache non-finite, starting on the shared table")
        return
      self.ceiling = np.clip(ceiling, A_LAT_MIN, A_LAT_MAX)
      self.step = np.clip(step, _STEP_MIN, _STEP_INIT)
      self.learned = not np.allclose(self.ceiling, self.prior)
      cloudlog.info(f"curve lat limit: restored {np.round(self.ceiling, 3).tolist()}")
    except Exception:
      cloudlog.exception("curve lat limit: failed to restore")
