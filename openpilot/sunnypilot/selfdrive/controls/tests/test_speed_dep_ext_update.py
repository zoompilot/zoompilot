"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

How LatControlTorqueExt takes a torqued message: learned values for valid bins, TOML seeds
or the global filtered values for the rest, the two deactivation paths, plus the learner's
sanity bounds and the seed-validity gate on speed_dependent.toml entries.
"""
import numpy as np
import pytest

from opendbc.sunnypilot.car.interfaces import get_speed_dep_config, get_speed_dep_config_for_car
from openpilot.selfdrive.locationd import torqued
from openpilot.selfdrive.locationd.torqued import TorqueEstimator
from openpilot.sunnypilot.selfdrive.locationd import torqued_ext
from openpilot.sunnypilot.selfdrive.locationd.tests.speed_dep_helpers import FakeParams as LearnerFakeParams, make_cp as make_learner_cp
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext import LatControlTorqueExt
from openpilot.sunnypilot.selfdrive.controls.tests.speed_dep_helpers import SAMPLE_SPEED_BP, make_cp, make_ext_stub, make_torqued_msg

SPEED_DEP_CARS = get_speed_dep_config()


class TestUpdateSpeedDepTorqueFallback:
  """update_speed_dep_torque fallback logic: TOML seeds vs global filtered."""

  def test_toml_seeds_used_for_invalid_bins(self, set_speed_dep_config):
    """Invalid bins fall back to TOML seed values, not global filtered."""
    seed_lafs = [2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7]
    seed_frictions = [0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17]
    set_speed_dep_config({'TEST_CAR': {'speed_bp': SAMPLE_SPEED_BP, 'laf_bp': seed_lafs, 'friction_bp': seed_frictions}})

    ext = make_ext_stub()
    tp, tp_sp = make_torqued_msg(SAMPLE_SPEED_BP, [999.0] * 7, [999.0] * 7, [False] * 7, global_laf=1.0, global_fric=0.05)
    LatControlTorqueExt.update_speed_dep_torque(ext, tp, tp_sp)

    assert ext._speed_dep_lat_accel_factor_bp == seed_lafs
    assert ext._speed_dep_friction_bp == seed_frictions

  def test_global_filtered_fallback_when_no_config(self, set_speed_dep_config):
    """Unconfigured car: invalid bins use the message's global filtered values."""
    set_speed_dep_config({})

    ext = make_ext_stub(fingerprint='UNKNOWN_CAR')
    tp, tp_sp = make_torqued_msg(SAMPLE_SPEED_BP, [999.0] * 7, [999.0] * 7, [False] * 7, global_laf=2.0, global_fric=0.15)
    LatControlTorqueExt.update_speed_dep_torque(ext, tp, tp_sp)

    assert ext._speed_dep_lat_accel_factor_bp == [tp.latAccelFactorFiltered] * 7
    assert ext._speed_dep_friction_bp == [tp.frictionCoefficientFiltered] * 7

  def test_friction_bp_missing_uses_global_fallback(self, set_speed_dep_config):
    """Config with laf_bp but no friction_bp uses the global fallback (and does not crash)."""
    set_speed_dep_config({'TEST_CAR': {'speed_bp': SAMPLE_SPEED_BP, 'laf_bp': [2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7]}})

    ext = make_ext_stub()
    tp, tp_sp = make_torqued_msg(SAMPLE_SPEED_BP, [999.0] * 7, [999.0] * 7, [False] * 7, global_laf=2.0, global_fric=0.15)
    LatControlTorqueExt.update_speed_dep_torque(ext, tp, tp_sp)

    assert ext._speed_dep_lat_accel_factor_bp == [tp.latAccelFactorFiltered] * 7
    assert ext._speed_dep_friction_bp == [tp.frictionCoefficientFiltered] * 7

  def test_laf_bp_length_mismatch_uses_global_fallback(self, set_speed_dep_config):
    """Config with a wrong-length laf_bp uses the global fallback."""
    set_speed_dep_config({'TEST_CAR': {'speed_bp': SAMPLE_SPEED_BP, 'laf_bp': [2.1, 2.2], 'friction_bp': [0.1, 0.2]}})

    ext = make_ext_stub()
    tp, tp_sp = make_torqued_msg(SAMPLE_SPEED_BP, [999.0] * 7, [999.0] * 7, [False] * 7, global_laf=2.0, global_fric=0.15)
    LatControlTorqueExt.update_speed_dep_torque(ext, tp, tp_sp)

    assert ext._speed_dep_lat_accel_factor_bp == [tp.latAccelFactorFiltered] * 7
    assert ext._speed_dep_friction_bp == [tp.frictionCoefficientFiltered] * 7

  def test_mixed_valid_invalid_bins(self, set_speed_dep_config):
    """Valid bins use the learned values off the message, invalid bins the TOML seeds."""
    seed_lafs = [2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7]
    seed_frictions = [0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17]
    set_speed_dep_config({'TEST_CAR': {'speed_bp': SAMPLE_SPEED_BP, 'laf_bp': seed_lafs, 'friction_bp': seed_frictions}})

    valid = [True, False, True, False, True, False, False]
    tp, tp_sp = make_torqued_msg(SAMPLE_SPEED_BP, [3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6], [0.21, 0.22, 0.23, 0.24, 0.25, 0.26, 0.27], valid)
    learned_lafs, learned_frictions = list(tp_sp.speedBinLatAccelFactors), list(tp_sp.speedBinFrictions)

    ext = make_ext_stub()
    LatControlTorqueExt.update_speed_dep_torque(ext, tp, tp_sp)

    for i in range(7):
      if valid[i]:
        assert ext._speed_dep_lat_accel_factor_bp[i] == learned_lafs[i]
        assert ext._speed_dep_friction_bp[i] == learned_frictions[i]
      else:
        assert ext._speed_dep_lat_accel_factor_bp[i] == seed_lafs[i]
        assert ext._speed_dep_friction_bp[i] == seed_frictions[i]

  def test_empty_bins_deactivate(self, set_speed_dep_config):
    """Empty bins route through the single deactivation path (CP-tune restore included)."""
    set_speed_dep_config({})
    ext = make_ext_stub()
    ext._speed_dep_active = True

    LatControlTorqueExt.update_speed_dep_torque(ext, *make_torqued_msg([], [], [], []))

    ext.disable_speed_dep_torque.assert_called_once()

  def test_use_params_off_deactivates(self, set_speed_dep_config):
    """useParams flipping off mid-drive (manual override enabled) routes through the same
    deactivation path even with bins still present."""
    set_speed_dep_config({})
    ext = make_ext_stub()
    ext._speed_dep_active = True

    LatControlTorqueExt.update_speed_dep_torque(ext, *make_torqued_msg([6.5, 10.0], [2.0, 2.0], [0.1, 0.1], [True, True], use_params=False))

    ext.disable_speed_dep_torque.assert_called_once()

  def test_disable_speed_dep_restores_cp_tune(self):
    """Mid-drive de-assert (useParams flipped off): the controller must return to the CP tune
    instead of keeping the last interpolated values forever."""
    ext = make_ext_stub()
    ext._speed_dep_active = True
    tune = ext.CP.lateralTuning.torque
    tune.latAccelFactor = 2.5
    tune.latAccelOffset = 0.05
    tune.friction = 0.12

    LatControlTorqueExt.disable_speed_dep_torque(ext)

    assert ext._speed_dep_active is False
    assert ext.lac_torque.torque_params.latAccelFactor == tune.latAccelFactor
    assert ext.lac_torque.torque_params.latAccelOffset == tune.latAccelOffset
    assert ext.lac_torque.torque_params.friction == tune.friction
    ext.lac_torque.update_limits.assert_called_once()

  def test_disable_speed_dep_noop_when_inactive(self):
    ext = make_ext_stub()
    LatControlTorqueExt.disable_speed_dep_torque(ext)
    ext.lac_torque.update_limits.assert_not_called()


@pytest.fixture
def learner_params(monkeypatch):
  """Both Params sites of the torqued learner behind one FakeParams (speed-dep chain on,
  LiveTorqueParamsRelaxedToggle off)."""
  fake = LearnerFakeParams()
  monkeypatch.setattr(torqued, "Params", lambda: fake)
  monkeypatch.setattr(torqued_ext, "Params", lambda: fake)
  return fake


class TestLearnerSanityBounds:
  """Speed-bin sanity bounds must allow learning regardless of the 'Less Restrict' toggle."""

  @staticmethod
  def _learner():
    fingerprint = next(iter(SPEED_DEP_CARS)) if SPEED_DEP_CARS else 'FAKE'
    est = TorqueEstimator(make_learner_cp(fingerprint=fingerprint, lat_accel_factor=2.0, friction=0.15))
    est._on_torque_point(0.1, 0.3, 10.0)  # trigger lazy init
    return est

  def test_sanity_bounds_allow_learning_without_relaxed(self, learner_params):
    """With LiveTorqueParamsRelaxedToggle OFF (factor_sanity=0.0), speed bins must still have
    +/-30% bounds, not (seed, seed)."""
    est = self._learner()

    for i, (lo, hi) in enumerate(est.speed_bin_lat_accel_factor_bounds):
      assert hi > lo, f"Bin {i} latAccelFactor bounds ({lo:.3f}, {hi:.3f}) must allow a range"

    for i, (lo, hi) in enumerate(est.speed_bin_friction_bounds):
      assert hi > lo, f"Bin {i} friction bounds ({lo:.3f}, {hi:.3f}) must allow a range"

  def test_clip_allows_10pct_movement(self, learner_params):
    """A learned value 10% above seed passes through np.clip with +/-30% bounds."""
    est = self._learner()

    seed_factor = est.speed_bin_filtered[0]['latAccelFactor'].x
    nudged = seed_factor * 1.10
    lo, hi = est.speed_bin_lat_accel_factor_bounds[0]
    clipped = np.clip(nudged, lo, hi)
    assert clipped == pytest.approx(nudged, abs=1e-6), \
      f"+10% nudge ({nudged:.3f}) should not be clipped by +/-30% bounds ({lo:.3f}, {hi:.3f})"


class TestSeedValidityGate:
  """Entries measured on a steer-to-zero EPS declare requires_steer_to_zero and must not
  apply to the same model with its stock EPS (different STEER_MAX schedule, mis-scaled LAF)."""

  def test_flagged_entry_suppressed_on_stock_eps(self, set_speed_dep_config):
    set_speed_dep_config({'SWAP_CAR': {'requires_steer_to_zero': True, 'speed_bp': [10.0]}})
    assert get_speed_dep_config_for_car(make_cp('SWAP_CAR', 12.5)) == {}

  def test_flagged_entry_applies_with_swap_eps(self, set_speed_dep_config):
    cfg = {'requires_steer_to_zero': True, 'speed_bp': [10.0]}
    set_speed_dep_config({'SWAP_CAR': cfg})
    assert get_speed_dep_config_for_car(make_cp('SWAP_CAR', 0.0)) == cfg

  def test_unflagged_entry_applies_regardless(self, set_speed_dep_config):
    cfg = {'speed_bp': [10.0]}
    set_speed_dep_config({'PLAIN_CAR': cfg})
    assert get_speed_dep_config_for_car(make_cp('PLAIN_CAR', 12.5)) == cfg

  def test_real_toml_flags_are_as_intended(self):
    """CX-9 2021 seeds were measured on an EPS-swapped car and must carry the flag;
    the CX-5 2022 seeds were measured on the stock (steer-to-zero) EPS and must not."""
    cars = get_speed_dep_config()
    assert cars['MAZDA_CX9_2021'].get('requires_steer_to_zero') is True
    assert 'requires_steer_to_zero' not in cars['MAZDA_CX5_2022']
