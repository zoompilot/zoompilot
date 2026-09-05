"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# The steer-limit classifier (lib/steer_limit.py): the unit table on both sides of the CX-5's
# STEER_MAX cliff, the controlsd_ext wiring that replaces controlsd's flag with the
# classifier's driver_limited, and the integrator-level consequence run through the real v0
# controller against a simulated slew-limited actuator.

import math
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from opendbc.car.mazda.values import MazdaFlags
from opendbc.car.structs import car
from opendbc.sunnypilot.car.interfaces import get_steer_slew_schedule
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import LatControlTorque as LatControlTorqueV0
from openpilot.sunnypilot.selfdrive.controls.lib.steer_limit import CLEAN, SteerLimit, classify

# CX-5 2022 carcontroller numbers: 12 counts/frame over 1200 below the cliff, over 800 above;
# EPS ceiling 648/1200 at 14.2 m/s and 620/800 from 14.5 m/s up
BELOW_CLIFF = {'slew_up': 12.0 / 1200.0, 'slew_down': 12.0 / 1200.0, 'rail_scale': 648.0 / 1200.0}
ABOVE_CLIFF = {'slew_up': 12.0 / 800.0, 'slew_down': 12.0 / 800.0, 'rail_scale': 620.0 / 800.0}
SIDES = [pytest.param(BELOW_CLIFF, id='below_cliff'), pytest.param(ABOVE_CLIFF, id='above_cliff')]

DT = 0.01
LAT_DELAY = 0.3
LAF = 2.5
CURV_PER_DEG = 2e-4  # toy geometry: curvature = -steeringAngleDeg * CURV_PER_DEG

VM = SimpleNamespace(calc_curvature=lambda angle_rad, v_ego, roll: math.degrees(angle_rad) * CURV_PER_DEG)
LP = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)


def make_cp(mazda=True):
  CP = car.CarParams.new_message(steerControlType="torque", steerLimitTimer=0.4)
  if mazda:
    # the real CX-5 2022 platform, so the slew and rail schedules resolve from opendbc
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


def make_lac(mazda=True):
  return LatControlTorqueV0(make_cp(mazda), custom.CarParamsSP.new_message().as_reader(), make_ci(), DT)


CX5_SLEW = get_steer_slew_schedule(make_cp())


def make_cs(v_ego=15.0, lat_accel=0.0, pressed=False):
  angle = -lat_accel / (CURV_PER_DEG * v_ego ** 2)
  return SimpleNamespace(vEgo=v_ego, aEgo=0.0, steeringAngleDeg=angle, steeringRateDeg=0.0, steeringPressed=pressed)


@pytest.fixture
def params():
  with OpenpilotPrefix():
    yield Params()


DEEPENING = {'error_prev': 0.2, 'integrator': 0.3}   # same sign: integrating would grow |i|
DECAYING = {'error_prev': -0.2, 'integrator': 0.3}   # opposite signs: integrating shrinks |i|


def cl(cmd, applied, applied_prev, side, upstream_flag=True, **kw):
  args = {**side, **DEEPENING, **kw}
  return classify(cmd, applied, applied_prev, upstream_flag=upstream_flag, **args)


class TestClassifyTable:
  @pytest.mark.parametrize('side', SIDES)
  def test_clean_when_command_and_applied_agree(self, side):
    assert cl(0.30, 0.30, 0.29, side) == CLEAN
    assert cl(0.30, 0.305, 0.29, side) == CLEAN  # inside the 0.01 threshold
    assert cl(0.30, 0.30, 0.29, side, **DECAYING) == CLEAN

  @pytest.mark.parametrize('side', SIDES)
  def test_full_slew_step_toward_the_command_is_rate_limited(self, side):
    step = side['slew_up']
    lim = cl(0.40, 0.30 + step, 0.30, side)
    assert lim == SteerLimit(limited=True, driver_limited=False, rate_limited=True, at_rail=False)

  def test_command_walking_under_the_rate_limit_is_lag_not_a_driver(self):
    # above the cliff a step is 0.015: a command moving 0.012 per frame is tracked exactly, one
    # carOutput frame late, and the 0.012 gap clears the 0.01 threshold every frame
    lim = cl(0.312, 0.300, 0.288, ABOVE_CLIFF)
    assert (lim.rate_limited, lim.driver_limited) == (True, False)
    # the same gap with the actuator barely moving is a driver holding it
    lim = cl(0.312, 0.300, 0.299, ABOVE_CLIFF)
    assert (lim.rate_limited, lim.driver_limited) == (False, True)

  @pytest.mark.parametrize('side', SIDES)
  def test_short_step_is_driver_limited(self, side):
    step = side['slew_up']
    lim = cl(0.40, 0.30 + 0.5 * step, 0.30, side)
    assert lim == SteerLimit(limited=True, driver_limited=True, rate_limited=False, at_rail=False)
    # the 0.9 tolerance covers the scale being read at vEgoRaw and rounded by the carcontroller
    assert cl(0.40, 0.30 + 0.92 * step, 0.30, side).rate_limited

  @pytest.mark.parametrize('side', SIDES)
  def test_walking_down_toward_zero_uses_the_down_step(self, side):
    step = side['slew_down']
    assert cl(0.0, -0.30 + step, -0.30, side).rate_limited
    assert cl(0.0, 0.30 - step, 0.30, side).rate_limited

  @pytest.mark.parametrize('side', SIDES)
  def test_pinned_applied_is_driver_limited(self, side):
    lim = cl(0.40, 0.30, 0.30, side)
    assert lim == SteerLimit(limited=True, driver_limited=True, rate_limited=False, at_rail=False)

  @pytest.mark.parametrize('side', SIDES)
  def test_step_away_from_the_command_is_not_rate_limited(self, side):
    step = side['slew_up']
    lim = cl(0.20, 0.30 + step, 0.30, side)
    assert lim.driver_limited and not lim.rate_limited

  @pytest.mark.parametrize('side', SIDES)
  @pytest.mark.parametrize('kind', ['rate', 'driver'])
  def test_deepening_integrator_freezes_decaying_one_bleeds(self, side, kind):
    """The flag handed to the tunes is directional: rate- and driver-limited alike freeze
    only while integrating would grow |i|; toward a reversing error the integrator stays live."""
    applied = 0.30 + (side['slew_up'] if kind == 'rate' else 0.0)
    assert cl(0.40, applied, 0.30, side, error_prev=0.2, integrator=0.3).limited
    assert cl(0.40, applied, 0.30, side, error_prev=-0.2, integrator=-0.3).limited
    assert not cl(0.40, applied, 0.30, side, error_prev=-0.2, integrator=0.3).limited
    assert not cl(0.40, applied, 0.30, side, error_prev=0.2, integrator=-0.3).limited

  @pytest.mark.parametrize('side', SIDES)
  def test_empty_integrator_counts_as_deepening(self, side):
    assert cl(0.40, 0.30, 0.30, side, error_prev=0.2, integrator=0.0).limited
    assert cl(0.40, 0.30, 0.30, side, error_prev=0.0, integrator=0.3).limited

  @pytest.mark.parametrize('side', SIDES)
  def test_applied_on_the_rail_is_never_limited(self, side):
    rail = side['rail_scale']
    for kw in (DEEPENING, DECAYING):
      lim = cl(rail + 0.20, rail, rail, side, **kw)
      assert lim == SteerLimit(limited=False, driver_limited=False, rate_limited=False, at_rail=True)
      lim = cl(-rail - 0.20, -rail, -rail, side, **kw)
      assert lim == SteerLimit(limited=False, driver_limited=False, rate_limited=False, at_rail=True)

  @pytest.mark.parametrize('side', SIDES)
  def test_rail_reported_even_without_a_mismatch(self, side):
    rail = side['rail_scale']
    assert cl(rail, rail, rail, side, upstream_flag=False) == SteerLimit(False, False, False, True)

  def test_full_scale_is_a_rail_on_platforms_without_a_ceiling(self):
    flat = {'slew_up': 0.01, 'slew_down': 0.02, 'rail_scale': 1.0}
    assert cl(1.0, 1.0, 1.0, flat).at_rail
    assert not cl(1.0, 0.99, 0.98, flat).at_rail

  @pytest.mark.parametrize('side', SIDES)
  @pytest.mark.parametrize('flag', [True, False])
  def test_first_frame_falls_back_to_the_upstream_flag(self, side, flag):
    lim = cl(0.40, 0.30, None, side, upstream_flag=flag, **DECAYING)
    assert lim == SteerLimit(limited=flag, driver_limited=flag, rate_limited=False, at_rail=False)
    # no history is only a fallback under a mismatch: an agreeing pair is clean regardless
    assert cl(0.30, 0.30, None, side, upstream_flag=flag) == CLEAN


class TestControlsdWiring:
  """ControlsExt.reclassify_steer_limit runs after publish() computed the upstream flag."""

  @staticmethod
  def _stub(flag=True, lat_active=True, slew=CX5_SLEW, rail=([0.0], [0.6]), commanded=0.40, applied_prev=None,
            with_extension=True, error=0.2, integrator=0.3):
    ext = SimpleNamespace(commanded_torque=commanded, steer_rail_schedule=rail, last_error=error, integrator=integrator,
                          rail_scale_at=lambda v: float(np.interp(v, rail[0], rail[1])) if rail else 1.0,
                          set_actuator_state=MagicMock())
    lac = SimpleNamespace(extension=ext) if with_extension else SimpleNamespace()
    return SimpleNamespace(LaC=lac, _steer_slew_schedule=slew, _lat_active_last=lat_active,
                           _applied_torque_prev=applied_prev, steer_limited_by_safety=flag)

  @staticmethod
  def _sm(v_ego=10.0, applied=0.30):
    return {'carState': SimpleNamespace(vEgo=v_ego),
            'carOutput': SimpleNamespace(actuatorsOutput=SimpleNamespace(torque=applied))}

  def test_deepening_mismatch_keeps_the_flag(self):
    stub = self._stub(commanded=0.40, applied_prev=0.29, error=0.2, integrator=0.3)
    ControlsExt.reclassify_steer_limit(stub, self._sm(applied=0.30))
    assert stub.steer_limited_by_safety is True
    assert stub._applied_torque_prev == 0.30
    stub.LaC.extension.set_actuator_state.assert_called_once_with(0.30, False)

  def test_decaying_integrator_releases_the_flag(self):
    stub = self._stub(commanded=0.40, applied_prev=0.29, error=-0.2, integrator=0.3)
    ControlsExt.reclassify_steer_limit(stub, self._sm(applied=0.30))
    assert stub.steer_limited_by_safety is False

  def test_rail_clears_the_flag_and_reaches_the_extension(self):
    stub = self._stub(commanded=0.90, applied_prev=0.60, error=0.2, integrator=0.3)
    ControlsExt.reclassify_steer_limit(stub, self._sm(applied=0.60))
    assert stub.steer_limited_by_safety is False
    stub.LaC.extension.set_actuator_state.assert_called_once_with(0.60, True)

  def test_first_active_frame_keeps_the_upstream_flag(self):
    for flag in (True, False):
      stub = self._stub(flag=flag, commanded=0.40, applied_prev=None, error=-0.2, integrator=0.3)
      ControlsExt.reclassify_steer_limit(stub, self._sm(applied=0.30))
      assert stub.steer_limited_by_safety is flag
      assert stub._applied_torque_prev == 0.30

  def test_brand_without_slew_schedule_is_a_passthrough(self):
    stub = self._stub(slew=None, commanded=0.40, applied_prev=0.29, error=-0.2, integrator=0.3)
    ControlsExt.reclassify_steer_limit(stub, self._sm(applied=0.30))
    assert stub.steer_limited_by_safety is True
    stub.LaC.extension.set_actuator_state.assert_not_called()

  def test_controller_without_extension_is_a_passthrough(self):
    # angle and PID controllers carry no torque extension
    stub = self._stub(with_extension=False, commanded=0.40, applied_prev=0.29, error=-0.2, integrator=0.3)
    ControlsExt.reclassify_steer_limit(stub, self._sm(applied=0.30))
    assert stub.steer_limited_by_safety is True

  def test_inactive_lateral_leaves_the_flag_and_drops_history(self):
    stub = self._stub(lat_active=False, applied_prev=0.29, error=-0.2, integrator=0.3)
    ControlsExt.reclassify_steer_limit(stub, self._sm(applied=0.30))
    assert stub.steer_limited_by_safety is True
    assert stub._applied_torque_prev is None
    stub.LaC.extension.set_actuator_state.assert_not_called()

  def test_angle_cars_get_no_slew_schedule(self, params):
    params.put('CarParamsSP', custom.CarParamsSP.new_message().to_bytes())
    CP = car.CarParams.new_message(steerControlType='angle', brand='tesla', carFingerprint='TESLA_MODEL_3')
    ext = ControlsExt(CP.as_reader(), params)
    assert ext._steer_slew_schedule is None
    ext = ControlsExt(make_cp(), params)
    assert ext._steer_slew_schedule == CX5_SLEW


class TestCommandedTorque:
  def test_tracks_the_published_actuator_sign_and_inactive_frames(self, params):
    lac = make_lac()
    v_ego = 15.0
    cs = make_cs(v_ego, 0.0)
    steer = None
    for _ in range(40):
      steer, _, _ = lac.update(True, cs, VM, LP, False, 0.3 / v_ego ** 2, None, False, LAT_DELAY)
    assert steer != 0.0
    assert lac.extension.commanded_torque == steer
    assert lac.extension.last_error == pytest.approx(0.3)  # setpoint 0.3 m/s^2, measurement 0
    assert lac.extension.integrator == lac.pid.i != 0.0
    lac.update(False, cs, VM, LP, False, 0.0, None, False, LAT_DELAY)
    assert lac.extension.commanded_torque == 0.0


class TestIntegratorUnderTheNewFlag:
  """v0 against a simulated actuator that walks toward the command at the carcontroller's
  rate limit, one frame behind, on a weaving request with a constant small tracking error of
  one sign. The command moves faster than one slew step, so upstream's |cmd - applied| > 0.01
  is set on most frames. With the integrator seeded on the error's side both flags freeze it
  (|i| stays put); once the error flips sign the old flag still freezes it and the new one
  lets it bleed; at the rail the new flag is False and the tune's saturation alert fires."""

  V_EGO = 14.0        # below the cliff: step 0.01, rail 0.556
  ERROR = 0.1
  FRAMES = 300        # per phase
  SEED_I = 0.3
  SLOPE = 0.03        # m/s^2 per frame: 0.012 of scale per frame at LAF 2.5, above the 0.01 step
  AMPLITUDE = 0.6     # m/s^2, keeps the command under the rail
  KI = 0.3            # v0's integral gain; the expected integrator slope is KI * DT * ERROR

  def _request(self, k):
    phase = (k * self.SLOPE) % (4 * self.AMPLITUDE)
    return phase if phase <= 2 * self.AMPLITUDE else 4 * self.AMPLITUDE - phase  # triangle in [0, 2A]

  def _run(self, use_classifier):
    lac = make_lac()
    bp, up, down = CX5_SLEW
    step_up, step_down = float(np.interp(self.V_EGO, bp, up)), float(np.interp(self.V_EGO, bp, down))
    rail = lac.extension.rail_scale_at(self.V_EGO)
    lac.pid.i = self.SEED_I
    flag = False
    applied, applied_prev, cmd_prev = 0.0, None, 0.0
    phases = {'deepening': self.ERROR, 'decaying': -self.ERROR, 'rail': self.ERROR}
    result = {}
    for name, error in phases.items():
      old_flags, new_flags, rails, sats = [], [], [], []
      for k in range(self.FRAMES):
        desired = 3.0 if name == 'rail' else self._request(k) - self.AMPLITUDE
        cs = make_cs(self.V_EGO, desired - error)
        cmd, _, log = lac.update(True, cs, VM, LP, flag, desired / self.V_EGO ** 2, None, False, LAT_DELAY)
        # carOutput seen at this frame's publish: the carcontroller acting on the previous command
        target = float(np.clip(cmd_prev, -rail, rail))
        step = step_up if abs(target) > abs(applied) else step_down
        applied = float(np.clip(target, applied - step, applied + step))
        old_flag = abs(cmd - applied) > 1e-2
        lim = classify(cmd, applied, applied_prev, step_up, step_down, rail, old_flag,
                       lac.extension.last_error, lac.extension.integrator)
        old_flags.append(old_flag)
        new_flags.append(lim.limited)
        rails.append(lim.at_rail)
        sats.append(log.saturated)
        flag = lim.limited if use_classifier else old_flag
        applied_prev, cmd_prev = applied, cmd
      at_rail = np.array(rails, dtype=bool)
      result[name] = SimpleNamespace(i=lac.pid.i, old_duty=np.mean(old_flags), new_duty=np.mean(new_flags),
                                     rail_duty=at_rail.mean(), saturated=sats[-1],
                                     new_duty_at_rail=np.mean(np.array(new_flags)[at_rail]) if at_rail.any() else np.nan)
    return result

  def test_freeze_is_directional_and_off_at_the_rail(self, params):
    new = self._run(use_classifier=True)
    old = self._run(use_classifier=False)
    assert old['deepening'].old_duty > 0.7, 'the upstream flag must fire on most frames for this test to mean anything'
    # Both classifiers freeze error that would deepen saturation.
    assert new['deepening'].new_duty > 0.7
    assert new['deepening'].i == pytest.approx(self.SEED_I, abs=0.03)
    assert old['deepening'].i == pytest.approx(self.SEED_I, abs=0.03)
    # The directional classifier permits integrator decay away from saturation.
    assert new['decaying'].new_duty < 0.05
    assert old['decaying'].i == pytest.approx(old['deepening'].i, abs=0.03)
    expected_bleed = self.KI * DT * self.ERROR * self.FRAMES
    assert new['deepening'].i - new['decaying'].i == pytest.approx(expected_bleed, rel=0.15)
    # a request past the rail: once the actuator is on the rail the flag is False (the slew up
    # to it is a deepening mismatch and stays limited) and the tune's alert fires
    assert new['rail'].rail_duty > 0.8
    assert new['rail'].new_duty_at_rail == 0.0
    assert new['rail'].saturated
