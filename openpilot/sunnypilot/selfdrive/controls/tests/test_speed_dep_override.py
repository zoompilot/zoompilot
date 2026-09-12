"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The per-frame speed-dependent torque interpolation in LatControlTorqueExtOverride: the
latAccelFactor and friction the controller reads, toggle-off behavior, manual override
priority and change detection. Tested on the override directly (the class that owns the
interpolation) rather than LatControlTorqueExt, which inherits from NNLC and needs model
files to init. Per-count interpolation across a STEER_MAX cliff is in
test_speed_dep_per_count.py, the torqued message handling in test_speed_dep_ext_update.py.
"""
import numpy as np
import pytest

from openpilot.sunnypilot.selfdrive.controls.tests.speed_dep_helpers import (
  SAMPLE_SPEED_BP, SAMPLE_LAT_ACCEL_FACTOR_BP, SAMPLE_FRICTION_BP, activate_speed_dep, make_torque_params,
)


class TestLafInterpolatedBySpeed:
  """torque_params.latAccelFactor must be speed-interpolated before torque_from_lateral_accel reads it."""

  def test_lat_accel_factor_set_to_interpolated_value(self, make_override):
    ovr = make_override()
    activate_speed_dep(ovr)
    tp = make_torque_params(latAccelFactor=999.0)  # sentinel

    ovr._last_vego = 10.0
    ovr.update_override_torque_params(tp)

    expected = float(np.interp(10.0, SAMPLE_SPEED_BP, SAMPLE_LAT_ACCEL_FACTOR_BP))
    assert tp.latAccelFactor == pytest.approx(expected, abs=1e-4), \
      f"latAccelFactor should be {expected}, got {tp.latAccelFactor}"

  def test_lat_accel_factor_differs_at_different_speeds(self, make_override):
    ovr = make_override()
    activate_speed_dep(ovr)

    tp = make_torque_params()
    ovr._last_vego = 6.5
    ovr.update_override_torque_params(tp)
    factor_low = tp.latAccelFactor

    tp = make_torque_params()
    ovr._last_vego = 37.5
    ovr.update_override_torque_params(tp)
    factor_high = tp.latAccelFactor

    assert factor_low != pytest.approx(factor_high, abs=0.01), \
      "latAccelFactor must differ between 6.5 m/s and 37.5 m/s"

  def test_lat_accel_factor_not_global_value(self, make_override):
    """latAccelFactor should NOT be the global scalar."""
    ovr = make_override()
    activate_speed_dep(ovr)
    global_factor = 2.0
    tp = make_torque_params(latAccelFactor=global_factor)

    ovr._last_vego = 6.5  # seed latAccelFactor at 6.5 is 2.39, not 2.0
    ovr.update_override_torque_params(tp)

    assert tp.latAccelFactor != pytest.approx(global_factor, abs=0.01), \
      "latAccelFactor should be speed-interpolated, not the global value"


class TestFrictionInterpolatedBySpeed:
  """torque_params.friction must be speed-interpolated before get_friction reads it."""

  def test_friction_set_to_interpolated_value(self, make_override):
    ovr = make_override()
    activate_speed_dep(ovr)
    tp = make_torque_params(friction=999.0)

    ovr._last_vego = 35.0
    ovr.update_override_torque_params(tp)

    expected = float(np.interp(35.0, SAMPLE_SPEED_BP, SAMPLE_FRICTION_BP))
    assert tp.friction == pytest.approx(expected, abs=1e-4)

  def test_friction_differs_at_different_speeds(self, make_override):
    ovr = make_override()
    activate_speed_dep(ovr)

    tp = make_torque_params()
    ovr._last_vego = 6.5
    ovr.update_override_torque_params(tp)
    fric_low = tp.friction

    tp = make_torque_params()
    ovr._last_vego = 37.5
    ovr.update_override_torque_params(tp)
    fric_high = tp.friction

    assert fric_low != pytest.approx(fric_high, abs=0.01)


class TestToggleOffClearsState:
  """_speed_dep_active must be cleared when bins disappear."""

  def test_inactive_by_default(self, make_override):
    ovr = make_override()
    assert not ovr._speed_dep_active

  def test_deactivated_does_not_modify_params(self, make_override):
    ovr = make_override()
    activate_speed_dep(ovr)
    ovr._speed_dep_active = False

    tp = make_torque_params(latAccelFactor=99.0, friction=99.0)
    ovr._last_vego = 15.0
    ovr.update_override_torque_params(tp)

    assert tp.latAccelFactor == 99.0, "Should not modify params when inactive"
    assert tp.friction == 99.0

  def test_empty_speed_bp_does_not_modify_params(self, make_override):
    ovr = make_override()
    activate_speed_dep(ovr)
    ovr._speed_dep_speed_bp = []  # empty

    tp = make_torque_params(latAccelFactor=99.0, friction=99.0)
    ovr._last_vego = 15.0
    ovr.update_override_torque_params(tp)

    assert tp.latAccelFactor == 99.0
    assert tp.friction == 99.0


class TestManualOverridePriority:
  """Manual override must take priority over speed-dep."""

  def test_manual_overwrites_speed_dep(self, make_override):
    ovr = make_override(enforce=True, manual_override=True, manual_lat_accel_factor='350', manual_friction='25')
    activate_speed_dep(ovr)
    ovr._last_vego = 15.0

    tp = make_torque_params()
    # frame = -1, after +1 -> frame=0, 0 % 300 == 0 -> manual fires
    ovr.update_override_torque_params(tp)

    assert tp.latAccelFactor == pytest.approx(350.0, abs=0.1), "Manual latAccelFactor should overwrite speed-dep"
    assert tp.friction == pytest.approx(25.0, abs=0.1), "Manual friction should overwrite speed-dep"

  def test_manual_wins_every_frame(self, make_override):
    """The manual override must own the params on EVERY frame, not just the 3 s poll frame:
    the per-frame speed-dep interpolation used to out-write it 299/300 frames."""
    ovr = make_override(enforce=True, manual_override=True, manual_lat_accel_factor='350', manual_friction='25')
    activate_speed_dep(ovr)
    ovr._last_vego = 15.0

    tp = make_torque_params()
    for frame in range(10):
      ovr.update_override_torque_params(tp)
      assert tp.latAccelFactor == pytest.approx(350.0), f"speed-dep out-wrote manual on frame {frame}"
      assert tp.friction == pytest.approx(25.0), f"speed-dep out-wrote manual friction on frame {frame}"

  def test_manual_toggle_off_mid_drive_returns_to_speed_dep(self, make_override):
    """Flipping the override off mid-drive hands the params back to speed-dep at the next 3 s poll."""
    ovr = make_override(enforce=True, manual_override=True, manual_lat_accel_factor='350', manual_friction='25')
    activate_speed_dep(ovr)
    ovr._last_vego = 15.0

    tp = make_torque_params()
    ovr.update_override_torque_params(tp)
    assert tp.latAccelFactor == pytest.approx(350.0)

    ovr.params.manual_override = False
    for _ in range(301):  # crosses the next poll frame
      ovr.update_override_torque_params(tp)

    expected_factor = float(np.interp(15.0, SAMPLE_SPEED_BP, SAMPLE_LAT_ACCEL_FACTOR_BP))
    assert tp.latAccelFactor == pytest.approx(expected_factor, abs=1e-4)

  def test_speed_dep_used_when_manual_off(self, make_override):
    ovr = make_override(enforce=True, manual_override=False)
    activate_speed_dep(ovr)
    ovr._last_vego = 15.0

    tp = make_torque_params()
    ovr.update_override_torque_params(tp)

    expected_factor = float(np.interp(15.0, SAMPLE_SPEED_BP, SAMPLE_LAT_ACCEL_FACTOR_BP))
    assert tp.latAccelFactor == pytest.approx(expected_factor, abs=1e-4), "Without manual override, speed-dep should be used"


class TestChangeDetection:
  """update_override_torque_params should only return changed=True when values differ."""

  def test_no_change_returns_false(self, make_override):
    ovr = make_override()
    activate_speed_dep(ovr)
    ovr._last_vego = 15.0

    tp = make_torque_params()
    ovr.update_override_torque_params(tp)
    changed = ovr.update_override_torque_params(tp)  # same speed: no change
    assert not changed, "Should return False when values haven't changed"

  def test_speed_change_returns_true(self, make_override):
    ovr = make_override()
    activate_speed_dep(ovr)

    tp = make_torque_params()
    ovr._last_vego = 6.5
    ovr.update_override_torque_params(tp)

    ovr._last_vego = 37.5  # big speed change -> values change
    changed = ovr.update_override_torque_params(tp)
    assert changed, "Should return True when values changed"

  def test_float32_builder_does_not_report_change_every_frame(self, make_override):
    """The real torque_params is a capnp Float32 builder that hands back the rounded value,
    so the comparison must be made in float32 or every frame re-runs update_limits."""
    ovr = make_override()
    activate_speed_dep(ovr)
    ovr._last_vego = 13.7  # between bins: the interp is not float32-exact
    tp = make_torque_params()
    assert ovr.update_override_torque_params(tp)
    assert not ovr.update_override_torque_params(tp)
    assert tp.latAccelFactor == pytest.approx(float(np.interp(13.7, SAMPLE_SPEED_BP, SAMPLE_LAT_ACCEL_FACTOR_BP)), rel=1e-6)
    ovr._last_vego = 13.8
    assert ovr.update_override_torque_params(tp)


class TestToggleOffFallback:
  """When speed-dep is deactivated, the controller must not use stale tables."""

  def test_deactivation_via_empty_bins(self, make_override):
    """Simulates toggle-off: update_speed_dep_torque receives empty bins."""
    ovr = make_override()
    activate_speed_dep(ovr)
    assert ovr._speed_dep_active

    ovr._speed_dep_active = False  # torqued sent empty speedBinCenters

    # Float32-exact literals so the pass-through check can stay an exact equality
    laf, fric = float(np.float32(2.35)), float(np.float32(0.12))
    tp = make_torque_params(latAccelFactor=laf, friction=fric)
    ovr._last_vego = 15.0
    ovr.update_override_torque_params(tp)

    # global values pass through unmodified
    assert tp.latAccelFactor == laf
    assert tp.friction == fric

  def test_reactivation_after_deactivation(self, make_override):
    """Speed-dep can be re-enabled after being disabled."""
    ovr = make_override()
    ovr._speed_dep_active = False

    activate_speed_dep(ovr)
    assert ovr._speed_dep_active

    tp = make_torque_params()
    ovr._last_vego = 15.0
    ovr.update_override_torque_params(tp)

    expected_factor = float(np.interp(15.0, SAMPLE_SPEED_BP, SAMPLE_LAT_ACCEL_FACTOR_BP))
    assert tp.latAccelFactor == pytest.approx(expected_factor, abs=1e-4)


class TestExtrapolationAtBoundaries:
  """np.interp clamps to edge values for speeds outside the bin range."""

  def test_speed_below_first_bin_clamps(self, make_override):
    ovr = make_override()
    activate_speed_dep(ovr)
    tp = make_torque_params()
    ovr._last_vego = 0.0
    ovr.update_override_torque_params(tp)
    assert tp.latAccelFactor == pytest.approx(SAMPLE_LAT_ACCEL_FACTOR_BP[0], abs=1e-4)
    assert tp.friction == pytest.approx(SAMPLE_FRICTION_BP[0], abs=1e-4)

  def test_speed_above_last_bin_clamps(self, make_override):
    ovr = make_override()
    activate_speed_dep(ovr)
    tp = make_torque_params()
    ovr._last_vego = 100.0
    ovr.update_override_torque_params(tp)
    assert tp.latAccelFactor == pytest.approx(SAMPLE_LAT_ACCEL_FACTOR_BP[-1], abs=1e-4)
    assert tp.friction == pytest.approx(SAMPLE_FRICTION_BP[-1], abs=1e-4)

  def test_speed_at_exact_bin_center(self, make_override):
    ovr = make_override()
    activate_speed_dep(ovr)
    for i, speed in enumerate(SAMPLE_SPEED_BP):
      tp = make_torque_params()
      ovr._last_vego = speed
      ovr.update_override_torque_params(tp)
      assert tp.latAccelFactor == pytest.approx(SAMPLE_LAT_ACCEL_FACTOR_BP[i], abs=1e-4)
      assert tp.friction == pytest.approx(SAMPLE_FRICTION_BP[i], abs=1e-4)
