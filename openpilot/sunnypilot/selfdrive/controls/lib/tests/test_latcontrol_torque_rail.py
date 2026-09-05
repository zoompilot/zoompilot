"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# The EPS rail delivered through the shared extension: LatControlTorqueExt sets the host's
# steer_max to the rail fraction, so an unmodified upstream tune (v0 here) puts its PID limits
# at the rail and raises its own saturation alert there. steer_limited_by_safety arrives as
# the classifier's driver_limited (test_steer_limit.py), so a railed EPS no longer suppresses
# the alert; a driver-limited frame still does.

import math
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from opendbc.car.mazda.values import MazdaFlags
from opendbc.car.structs import car
from opendbc.sunnypilot.car.interfaces import get_steer_rail_schedule
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0

DT = 0.01
LAT_DELAY = 0.3
DELAY_FRAMES = int(LAT_DELAY / DT)
LAF = 2.5
CURV_PER_DEG = 2e-4  # toy geometry: curvature = -steeringAngleDeg * CURV_PER_DEG

VM = SimpleNamespace(calc_curvature=lambda angle_rad, v_ego, roll: math.degrees(angle_rad) * CURV_PER_DEG)
LP = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)


def make_cp(mazda=False):
  CP = car.CarParams.new_message(steerControlType="torque", steerLimitTimer=0.4)
  if mazda:
    CP.brand = 'mazda'
    CP.carFingerprint = 'MAZDA_CX5_2022'
    CP.minSteerSpeed = 0.0
    CP.flags = MazdaFlags.STEER_TO_ZERO_EPS.value
  CP.lateralTuning.init('torque')
  CP.lateralTuning.torque.latAccelFactor = LAF
  CP.lateralTuning.torque.friction = 0.0
  return CP.as_reader()


def make_ci():
  CI = MagicMock()
  CI.torque_from_lateral_accel.return_value = lambda lataccel, tp: lataccel / tp.latAccelFactor
  CI.lateral_accel_from_torque.return_value = lambda torque, tp: torque * tp.latAccelFactor
  return CI


def make_lac(mazda=False, rail=None):
  """v0 through the extension; `rail` overrides the platform's schedule the way the CX-5 would
  supply one ((bp, fractions) or None)."""
  lac = LatControlTorqueV0(make_cp(mazda), custom.CarParamsSP.new_message().as_reader(), make_ci(), DT)
  if rail is not None:
    lac.extension.steer_rail_schedule = rail
  return lac


def make_cs(v_ego=15.0, lat_accel=0.0, pressed=False):
  angle = -lat_accel / (CURV_PER_DEG * v_ego ** 2)
  return SimpleNamespace(vEgo=v_ego, aEgo=0.0, steeringAngleDeg=angle, steeringRateDeg=0.0, steeringPressed=pressed)


def step(lac, cs, desired_curvature, sls=False, active=True):
  _, _, pid_log = lac.update(active, cs, VM, LP, sls, desired_curvature, None, False, LAT_DELAY)
  return pid_log


def count_update_limits(lac):
  calls = []
  orig = lac.update_limits

  def wrapped():
    calls.append(1)
    orig()
  lac.update_limits = wrapped
  return calls


@pytest.fixture
def params():
  with OpenpilotPrefix():
    yield Params()


class TestRailFromPlatform:
  """The CX-5 2022 schedule resolves from CP; the extension reads speed from the previous
  frame (the same _last_vego the speed-dep interp uses), so each speed takes two frames."""

  def test_pid_limits_sit_on_the_rail(self, params):
    lac = make_lac(mazda=True)
    bp, rail = get_steer_rail_schedule(make_cp(mazda=True))
    assert lac.extension.steer_rail_schedule == (bp, rail)
    for v_ego in (10.0, 20.0):
      for _ in range(2):
        step(lac, make_cs(v_ego), 0.0)
      expected = float(np.interp(v_ego, bp, rail))
      assert lac.steer_max == pytest.approx(expected)
      assert lac.pid.pos_limit == pytest.approx(expected * LAF)
      assert lac.pid.neg_limit == pytest.approx(-expected * LAF)
    assert lac.steer_max == pytest.approx(620.0 / 800.0)  # the 20 m/s rail

  def test_update_limits_reruns_across_the_cliff_and_not_at_steady_speed(self, params):
    lac = make_lac(mazda=True)
    for _ in range(3):
      step(lac, make_cs(20.0), 0.0)  # settle: rail read at 0, then at 20 m/s
    calls = count_update_limits(lac)
    for _ in range(5):
      step(lac, make_cs(20.0), 0.0)
    assert calls == []
    step(lac, make_cs(14.2), 0.0)  # this frame still reads 20 m/s
    assert calls == []
    step(lac, make_cs(14.2), 0.0)  # now the 14.2 rail (648/1200)
    assert len(calls) == 1
    assert lac.steer_max == pytest.approx(648.0 / 1200.0)
    step(lac, make_cs(14.5), 0.0)
    step(lac, make_cs(14.5), 0.0)
    assert len(calls) == 2
    assert lac.steer_max == pytest.approx(620.0 / 800.0)

  def test_platform_without_a_schedule_keeps_full_scale(self, params):
    lac = make_lac(mazda=False)
    assert lac.extension.steer_rail_schedule is None
    calls = count_update_limits(lac)
    for v_ego in (5.0, 14.2, 14.5, 25.0):
      for _ in range(2):
        step(lac, make_cs(v_ego), 0.0)
    assert calls == []
    assert lac.steer_max == 1.0
    assert lac.pid.pos_limit == pytest.approx(LAF)
    assert lac.extension.rail_scale_at(15.0) == 1.0


class TestRailAwareSaturation:
  """A railed EPS raises the saturation warning through the tune's own steer_max test once
  the classifier keeps the rail out of steer_limited_by_safety; a driver-limited frame keeps
  the suppression, and platforms without a rail schedule keep stock semantics."""

  SAT_FRAMES = int(0.4 / DT) + 20  # steerLimitTimer plus margin
  RAIL = ([0.0, 30.0], [0.6, 0.6])

  def _run(self, lac, lat_accel_demand, sls, frames):
    cs = make_cs(v_ego=15.0, lat_accel=0.0)
    desired_curvature = lat_accel_demand / 15.0 ** 2
    log = None
    for _ in range(frames):
      log = step(lac, cs, desired_curvature, sls=sls)
    return log

  def test_railed_eps_raises_saturation(self, params):
    lac = make_lac(rail=self.RAIL)
    log = self._run(lac, lat_accel_demand=4.5, sls=False, frames=self.SAT_FRAMES)
    assert log.saturated
    assert abs(log.output) == pytest.approx(0.6)

  def test_driver_limited_frames_suppress_it(self, params):
    lac = make_lac(rail=self.RAIL)
    log = self._run(lac, lat_accel_demand=4.5, sls=True, frames=self.SAT_FRAMES * 2)
    assert not log.saturated

  def test_not_saturated_before_timer(self, params):
    lac = make_lac(rail=self.RAIL)
    log = self._run(lac, lat_accel_demand=4.5, sls=False, frames=5)
    assert not log.saturated

  def test_sub_rail_output_does_not_saturate(self, params):
    lac = make_lac(rail=self.RAIL)
    # measurement tracks the demand: no error, output = ff = 0.2 of scale, under the rail
    cs = make_cs(v_ego=15.0, lat_accel=0.5)
    log = None
    for _ in range(self.SAT_FRAMES * 2):
      log = step(lac, cs, 0.5 / 15.0 ** 2, sls=False)
    assert not log.saturated

  def test_no_schedule_keeps_stock_suppression(self, params):
    lac = make_lac()
    assert lac.extension.steer_rail_schedule is None
    log = self._run(lac, lat_accel_demand=6.0, sls=True, frames=self.SAT_FRAMES * 2)
    assert not log.saturated

  def test_no_schedule_full_scale_still_saturates_without_sls(self, params):
    lac = make_lac()
    log = self._run(lac, lat_accel_demand=6.0, sls=False, frames=self.SAT_FRAMES)
    assert log.saturated


class TestRailLimitedPid:
  """The PID is limited to the torque the EPS will actually deliver, not the full steer scale,
  so its own anti-windup engages at the real rail. Delivered counts are unchanged (the
  carcontroller clamps there regardless); what changes is that a railed integrator can
  still unwind instead of only being frozen from outside."""

  RAIL = 0.5

  def _railed(self, rail=RAIL):
    return make_lac(rail=([0.0], [rail]) if rail is not None else None)

  def test_limits_track_the_rail(self, params):
    lac = self._railed()
    step(lac, make_cs(15.0), 0.0)
    assert lac.pid.pos_limit == pytest.approx(self.RAIL * LAF)
    assert lac.pid.neg_limit == pytest.approx(-self.RAIL * LAF)

  def test_no_schedule_keeps_full_scale_limits(self, params):
    lac = self._railed(rail=None)
    step(lac, make_cs(15.0), 0.0)
    assert lac.pid.pos_limit == pytest.approx(LAF)

  def test_limits_follow_speed(self, params):
    """A falling ceiling must tighten the limits as the car speeds up, not stay on whatever
    the schedule read at construction."""
    lac = make_lac(rail=([5.0, 25.0], [1.0, 0.5]))
    for _ in range(2):
      step(lac, make_cs(5.0), 0.0)
    assert lac.pid.pos_limit == pytest.approx(LAF)
    for _ in range(2):
      step(lac, make_cs(25.0), 0.0)
    assert lac.pid.pos_limit == pytest.approx(0.5 * LAF)

  def test_output_never_exceeds_the_rail(self, params):
    lac = self._railed()
    v_ego = 15.0
    demand = 3.0 / v_ego ** 2  # 3.0 m/s^2: well past the 1.25 m/s^2 the rail allows
    for _ in range(DELAY_FRAMES + 10):
      step(lac, make_cs(v_ego), demand, active=False)
    for _ in range(50):
      out, _, _ = lac.update(True, make_cs(v_ego, 0.0), VM, LP, False, demand, None, False, LAT_DELAY)
      assert abs(out) <= self.RAIL + 1e-9

  def _rail_then_reverse(self, sls, seed_i=0.4):
    """Rail the output on a large demand, seed the integrator, then flip the measurement past
    the request so the error reverses. Returns the integrator after the reversal."""
    v_ego = 15.0
    demand = 3.0 / v_ego ** 2
    lac = self._railed()
    for _ in range(DELAY_FRAMES + 10):
      step(lac, make_cs(v_ego), demand, active=False)
    for _ in range(100):
      lac.update(True, make_cs(v_ego, 0.0), VM, LP, sls, demand, None, False, LAT_DELAY)
    lac.pid.i = seed_i
    for _ in range(100):
      lac.update(True, make_cs(v_ego, 4.0), VM, LP, sls, demand, None, False, LAT_DELAY)
    return lac.pid.i

  def test_railed_integrator_decays_toward_a_reversing_error(self, params):
    """With the PID limits on the rail, an integrator facing a reversed error decays back out
    on a rail frame (the classifier reports those as not driver-limited). A driver-limited
    frame still freezes it: that is what the flag now means."""
    free = self._rail_then_reverse(sls=False)
    assert free < 0.4 - 0.05
    assert self._rail_then_reverse(sls=True) == pytest.approx(0.4)

  def test_railed_integrator_still_cannot_wind_into_the_rail(self, params):
    """The other half: a standing error that pushes further into the rail must not wind up."""
    v_ego = 15.0
    demand = 3.0 / v_ego ** 2
    lac = self._railed()
    for _ in range(DELAY_FRAMES + 10):
      step(lac, make_cs(v_ego), demand, active=False)
    lac.pid.i = 0.0
    for _ in range(200):  # measurement stuck at zero: error stays large and positive
      lac.update(True, make_cs(v_ego, 0.0), VM, LP, False, demand, None, False, LAT_DELAY)
    # unconstrained, ki * dt * error * 200 frames would be ~1.8
    assert lac.pid.i < 0.05
