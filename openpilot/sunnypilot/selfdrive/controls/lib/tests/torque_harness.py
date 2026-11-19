"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Toy plant the torque controller tests drive the real tunes against: steering geometry with
curvature proportional to steering angle, and a mocked car interface where torque ==
lat_accel / latAccelFactor.
"""
import math
from types import SimpleNamespace
from unittest.mock import MagicMock

from opendbc.car.mazda.values import MazdaFlags
from opendbc.car.structs import car
from openpilot.cereal import custom

DT = 0.01
LAT_DELAY = 0.3
DELAY_FRAMES = int(LAT_DELAY / DT)
LAF = 2.5
FRICTION = 0.25
CURV_PER_DEG = 2e-4  # curvature = -steeringAngleDeg * CURV_PER_DEG

VM = SimpleNamespace(calc_curvature=lambda angle_rad, v_ego, roll: math.degrees(angle_rad) * CURV_PER_DEG)
LP = SimpleNamespace(angleOffsetDeg=0.0, roll=0.0)


def make_cp(mazda=False, friction=0.0):
  CP = car.CarParams.new_message(steerControlType="torque", steerLimitTimer=0.4)
  if mazda:
    # the real CX-5 2022 platform, so the slew and rail schedules resolve from opendbc
    CP.brand = 'mazda'
    CP.carFingerprint = 'MAZDA_CX5_2022'
    CP.minSteerSpeed = 0.0
    CP.flags = MazdaFlags.STEER_TO_ZERO_EPS.value
  CP.lateralTuning.init('torque')
  CP.lateralTuning.torque.latAccelFactor = LAF
  CP.lateralTuning.torque.friction = friction
  return CP.as_reader()


def make_ci():
  CI = MagicMock()
  CI.torque_from_lateral_accel.return_value = lambda lataccel, tp: lataccel / tp.latAccelFactor
  CI.lateral_accel_from_torque.return_value = lambda torque, tp: torque * tp.latAccelFactor
  return CI


def make_lac(cls, mazda=False, friction=0.0):
  return cls(make_cp(mazda, friction), custom.CarParamsSP.new_message().as_reader(), make_ci(), DT)


def make_cs(v_ego=15.0, lat_accel=0.0, pressed=False):
  """CarState whose measured lateral accel equals lat_accel at v_ego."""
  angle = -lat_accel / (CURV_PER_DEG * v_ego ** 2)
  return SimpleNamespace(vEgo=v_ego, aEgo=0.0, steeringAngleDeg=angle, steeringRateDeg=0.0, steeringPressed=pressed)


def step(lac, cs, desired_curvature, active=True, sls=False, lp=LP, lat_delay=LAT_DELAY):
  _, _, pid_log = lac.update(active, cs, VM, lp, sls, desired_curvature, None, False, lat_delay)
  return pid_log
