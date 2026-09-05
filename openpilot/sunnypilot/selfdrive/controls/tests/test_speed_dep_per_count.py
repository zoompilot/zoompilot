"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Speed-dependent torque across a speed-dependent STEER_MAX: LAF and friction interpolate in
per-CAN-count space so the scale's step lands at the cliff, and plain interpolation on a
flat platform. See docs/zoompilot/lateral-tune.md for the CX-5 numbers.
"""
import numpy as np
import pytest

from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext import LatControlTorqueExt
from openpilot.sunnypilot.selfdrive.controls.tests.speed_dep_helpers import (
  activate_speed_dep, make_ext_stub, make_torque_params, make_torqued_msg,
)


class TestPerCountLafInterp:
  """On a platform with a speed-dependent STEER_MAX, LAF interps in per-count space and
  rescales by the schedule at the current speed, so the scale's step lands at the cliff
  instead of being smeared across the cliff-spanning bin pair."""

  # CX-5-shaped fixture: cliff at 14.2-14.5 m/s inside the 12.0-16.4 bin span
  SM_SCHEDULE = ([0.0, 14.2, 14.5], [1200.0, 1200.0, 800.0])
  SPEED_BP = [6.5, 9.5, 12.0, 16.4, 21.0, 28.0, 35.0]
  LAF_BP = [2.43, 2.93, 2.37, 1.21, 1.16, 1.53, 1.76]

  # the CX-5's own friction bins (speed_dependent.toml), which step 149 -> 166 -> 112 -> 122
  # counts through 12.0, 14.2, 14.5, 16.4 m/s under a plain interp
  FRICTION_BP = [0.156, 0.142, 0.124, 0.153, 0.139, 0.122, 0.108]

  def _activate(self, ovr, schedule=None):
    activate_speed_dep(ovr, speed_bp=list(self.SPEED_BP), lat_accel_factor_bp=list(self.LAF_BP),
                       friction_bp=list(self.FRICTION_BP))
    if schedule is not None:
      sm_bp, sm_v = schedule
      ovr._speed_dep_steer_max_schedule = schedule
      ovr._speed_dep_laf_per_count_bp = [laf / float(np.interp(c, sm_bp, sm_v))
                                         for laf, c in zip(self.LAF_BP, self.SPEED_BP, strict=True)]
      ovr._speed_dep_friction_per_count_bp = [fric * float(np.interp(c, sm_bp, sm_v))
                                              for fric, c in zip(self.FRICTION_BP, self.SPEED_BP, strict=True)]

  def _laf_at(self, ovr, v):
    ovr._last_vego = v
    tp = make_torque_params()
    ovr.update_override_torque_params(tp)
    return tp.latAccelFactor

  def test_step_lands_at_the_cliff(self, make_override):
    ovr = make_override()
    self._activate(ovr, schedule=self.SM_SCHEDULE)
    below, above = self._laf_at(ovr, 14.2), self._laf_at(ovr, 14.5)
    # LAF steps down by ~the STEER_MAX ratio across 0.3 m/s (slightly more than 1.5:
    # the smooth per-count decline adds its own slope over the same interval)
    assert below / above == pytest.approx(1200.0 / 800.0, rel=0.03)

  def test_no_smear_below_the_cliff(self, make_override):
    ovr = make_override()
    self._activate(ovr, schedule=self.SM_SCHEDULE)
    # plain interp of normalized bins under-reads LAF here (over-torques ~+15%);
    # per-count interp must sit well above it
    smeared = float(np.interp(14.0, self.SPEED_BP, self.LAF_BP))
    assert self._laf_at(ovr, 14.0) > smeared * 1.10

  def test_round_trips_at_bin_centers(self, make_override):
    ovr = make_override()
    self._activate(ovr, schedule=self.SM_SCHEDULE)
    for c, laf in zip(self.SPEED_BP, self.LAF_BP, strict=True):
      assert self._laf_at(ovr, c) == pytest.approx(laf, rel=1e-6)  # float32-rounded, see the change check

  def test_flat_platform_unchanged(self, make_override):
    ovr = make_override()
    self._activate(ovr, schedule=None)
    for v in [10.0, 13.4, 14.35, 15.0, 25.0]:
      assert self._laf_at(ovr, v) == pytest.approx(float(np.interp(v, self.SPEED_BP, self.LAF_BP)), rel=1e-6)

  def _friction_counts_at(self, ovr, v):
    ovr._last_vego = v
    tp = make_torque_params()
    ovr.update_override_torque_params(tp)
    return tp.friction * float(np.interp(v, *self.SM_SCHEDULE))

  def test_friction_counts_continuous_across_the_cliff(self, make_override):
    """Friction is a normalized torque (get_friction's latAccelFactor cancels against the
    linear torque conversion), so its CAN counts are friction * STEER_MAX(v): a plain interp
    of the CX-5 bins put 149 -> 166 -> 112 -> 122 counts on the wire through the cliff.
    Per-count interp keeps the counts monotonic between the two bin centers."""
    ovr = make_override()
    self._activate(ovr, schedule=self.SM_SCHEDULE)
    speeds = [12.0, 13.0, 14.0, 14.2, 14.3, 14.35, 14.4, 14.5, 15.0, 16.0, 16.4]
    counts = [self._friction_counts_at(ovr, v) for v in speeds]
    assert counts[0] == pytest.approx(0.124 * 1200, abs=0.1)   # 149 at the 12.0 bin center
    assert counts[-1] == pytest.approx(0.153 * 800, abs=0.1)   # 122 at the 16.4 bin center
    assert all(b <= a + 1e-6 for a, b in zip(counts[:-1], counts[1:], strict=True)), counts
    # the excursion the fix removes: plain interp of normalized bins times the scale
    smeared = [float(np.interp(v, self.SPEED_BP, self.FRICTION_BP)) * float(np.interp(v, *self.SM_SCHEDULE)) for v in speeds]
    assert max(smeared) > 165 and min(smeared) < 113

  def test_friction_round_trips_at_bin_centers(self, make_override):
    ovr = make_override()
    self._activate(ovr, schedule=self.SM_SCHEDULE)
    for c, fric in zip(self.SPEED_BP, self.FRICTION_BP, strict=True):
      ovr._last_vego = c
      tp = make_torque_params()
      ovr.update_override_torque_params(tp)
      assert tp.friction == pytest.approx(fric, abs=1e-6)

  def test_update_speed_dep_torque_builds_per_count_table(self, set_speed_dep_config):
    """The per-count table is built from the config's schedule for learned and seed bins alike."""
    set_speed_dep_config({'TEST_CAR': {'speed_bp': self.SPEED_BP, 'laf_bp': self.LAF_BP,
                                       'friction_bp': [0.1] * 7, 'steer_max_schedule': self.SM_SCHEDULE}})
    ext = make_ext_stub()
    LatControlTorqueExt.update_speed_dep_torque(ext, *make_torqued_msg(self.SPEED_BP, self.LAF_BP, [0.1] * 7, [True] * 7))

    assert ext._speed_dep_steer_max_schedule == self.SM_SCHEDULE
    sm_bp, sm_v = self.SM_SCHEDULE
    expected = [laf / float(np.interp(c, sm_bp, sm_v)) for laf, c in zip(self.LAF_BP, self.SPEED_BP, strict=True)]
    assert ext._speed_dep_laf_per_count_bp == pytest.approx(expected)
    expected_fric = [0.1 * float(np.interp(c, sm_bp, sm_v)) for c in self.SPEED_BP]
    assert ext._speed_dep_friction_per_count_bp == pytest.approx(expected_fric)

  def test_update_speed_dep_torque_no_schedule_leaves_table_empty(self, set_speed_dep_config):
    set_speed_dep_config({'TEST_CAR': {'speed_bp': self.SPEED_BP, 'laf_bp': self.LAF_BP, 'friction_bp': [0.1] * 7}})
    ext = make_ext_stub()
    LatControlTorqueExt.update_speed_dep_torque(ext, *make_torqued_msg(self.SPEED_BP, self.LAF_BP, [0.1] * 7, [True] * 7))

    assert ext._speed_dep_steer_max_schedule is None
    assert ext._speed_dep_laf_per_count_bp == []
    assert ext._speed_dep_friction_per_count_bp == []


class TestPlainLafInterpolation:
  """LAF uses plain np.interp between bins on a flat platform: the PID sees a smooth blend."""

  SPEED_BP = [12.0, 16.4]
  LAF_BP = [2.36, 0.89]
  FRICTION_BP = [0.164, 0.164]

  def test_at_bin_centers_matches_values(self, make_override):
    ovr = make_override()
    activate_speed_dep(ovr, speed_bp=self.SPEED_BP, lat_accel_factor_bp=self.LAF_BP, friction_bp=self.FRICTION_BP)
    for i, speed in enumerate(self.SPEED_BP):
      tp = make_torque_params()
      ovr._last_vego = speed
      ovr.update_override_torque_params(tp)
      assert tp.latAccelFactor == pytest.approx(self.LAF_BP[i], abs=1e-4)

  def test_midpoint_is_linear_blend(self, make_override):
    """LAF at the midpoint is plain linear interpolation."""
    ovr = make_override()
    activate_speed_dep(ovr, speed_bp=self.SPEED_BP, lat_accel_factor_bp=self.LAF_BP, friction_bp=self.FRICTION_BP)
    v_mid = (self.SPEED_BP[0] + self.SPEED_BP[1]) / 2
    ovr._last_vego = v_mid
    tp = make_torque_params()
    ovr.update_override_torque_params(tp)
    expected = float(np.interp(v_mid, self.SPEED_BP, self.LAF_BP))
    assert tp.latAccelFactor == pytest.approx(expected, abs=1e-4)

  def test_laf_monotonic_between_bins(self, make_override):
    """LAF decreases monotonically between the high and low bins."""
    ovr = make_override()
    activate_speed_dep(ovr, speed_bp=self.SPEED_BP, lat_accel_factor_bp=self.LAF_BP, friction_bp=self.FRICTION_BP)
    lafs = []
    for v in np.linspace(self.SPEED_BP[0], self.SPEED_BP[1], 20):
      tp = make_torque_params()
      ovr._last_vego = v
      ovr.update_override_torque_params(tp)
      lafs.append(tp.latAccelFactor)
    diffs = np.diff(lafs)
    assert all(d <= 1e-6 for d in diffs)  # monotonically decreasing

  def test_friction_also_plain_interp(self, make_override):
    ovr = make_override()
    activate_speed_dep(ovr, speed_bp=self.SPEED_BP, lat_accel_factor_bp=self.LAF_BP, friction_bp=[0.120, 0.170])
    v_mid = 14.2
    ovr._last_vego = v_mid
    tp = make_torque_params()
    ovr.update_override_torque_params(tp)
    expected = float(np.interp(v_mid, self.SPEED_BP, [0.120, 0.170]))
    assert tp.friction == pytest.approx(expected, abs=1e-4)
