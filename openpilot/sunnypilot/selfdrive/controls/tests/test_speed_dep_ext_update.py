"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

How a real v0 controller's LatControlTorqueExt takes a torqued message: the bins as published,
and the two deactivation paths back to the CarParams tune. The TOML entry gate is tested in
opendbc (test_speed_dep_config.py), the learner's bounds in test_torqued_speed_dep.py.
"""
import pytest

from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0
from openpilot.sunnypilot.selfdrive.controls.lib.tests.torque_harness import LAF, make_lac
from openpilot.sunnypilot.selfdrive.locationd.tests.speed_dep_helpers import SAMPLE_SPEED_BP, make_torqued_msg

CP_FRICTION = 0.12
LAFS = [3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6]
FRICTIONS = [0.21, 0.22, 0.23, 0.24, 0.25, 0.26, 0.27]


def activated(valid=(True,) * 7):
  """A v0 controller whose extension took a torqued message with bins, and globals of 1.0
  latAccelFactor, 0.05 offset and 0.05 friction."""
  lac = make_lac(LatControlTorqueV0, friction=CP_FRICTION)
  tp, tp_sp = make_torqued_msg(SAMPLE_SPEED_BP, LAFS, FRICTIONS, list(valid), global_laf=1.0, global_fric=0.05)
  tp.latAccelOffsetFiltered = 0.05
  lac.extension.update_speed_dep_torque(tp, tp_sp)
  return lac, tp_sp


def assert_on_cp_tune(lac):
  assert not lac.extension._speed_dep_active
  assert lac.torque_params.latAccelFactor == pytest.approx(LAF)
  assert lac.torque_params.latAccelOffset == 0.0
  assert lac.torque_params.friction == pytest.approx(CP_FRICTION)
  assert lac.pid.pos_limit == pytest.approx(lac.steer_max * LAF)  # update_limits ran on the restored tune


class TestUpdateSpeedDepTorque:
  def test_bins_taken_as_published(self, params, set_speed_dep_config):
    """The learner filters a bin only once it is valid, so an invalid bin already carries its
    seed: the controller neither re-reads the TOML nor substitutes the global values. The
    globals are the PID-limits baseline until the next frame's interp."""
    fingerprint = make_lac(LatControlTorqueV0).extension.CP.carFingerprint
    set_speed_dep_config({fingerprint: {'speed_bp': SAMPLE_SPEED_BP, 'laf_bp': [9.0] * 7, 'friction_bp': [9.0] * 7}})
    lac, tp_sp = activated(valid=(True, False, True, False, True, False, False))
    ext = lac.extension

    assert ext._speed_dep_active
    assert ext._speed_dep_speed_bp == list(tp_sp.speedBinCenters)
    assert ext._speed_dep_lat_accel_factor_bp == list(tp_sp.speedBinLatAccelFactors)
    assert ext._speed_dep_friction_bp == list(tp_sp.speedBinFrictions)
    assert lac.torque_params.latAccelFactor == pytest.approx(1.0)
    assert lac.pid.pos_limit == pytest.approx(lac.steer_max * 1.0)

  def test_empty_bins_restore_the_cp_tune(self, params):
    lac, _ = activated()
    lac.extension.update_speed_dep_torque(*make_torqued_msg([], [], [], []))
    assert_on_cp_tune(lac)

  def test_use_params_off_restores_the_cp_tune(self, params):
    """useParams flipping off mid-drive (manual override enabled) takes the same path even
    with bins still present, instead of keeping the last interpolated values."""
    lac, _ = activated()
    lac.extension.update_speed_dep_torque(*make_torqued_msg([6.5, 10.0], [2.0, 2.0], [0.1, 0.1], [True, True], use_params=False))
    assert_on_cp_tune(lac)

  def test_empty_bins_leave_an_inactive_controller_alone(self, params):
    lac = make_lac(LatControlTorqueV0, friction=CP_FRICTION)
    lac.update_torque_parameters(2.0, 0.0, 0.2)  # torqued's globals
    lac.extension.update_speed_dep_torque(*make_torqued_msg([], [], [], []))
    assert lac.torque_params.latAccelFactor == pytest.approx(2.0)
    assert lac.torque_params.friction == pytest.approx(0.2)
