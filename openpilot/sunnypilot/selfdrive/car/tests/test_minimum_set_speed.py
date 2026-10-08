"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

ICBM compares the car's set-speed floor (CarParamsSP.minimumSetSpeed) in whole dash units.
"""
import pytest

from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.car.tests.icbm_servo_harness import make_icbm, run_frames

SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState

MAZDA_FLOOR = 30. * CV.KPH_TO_MS  # MRCC's floor in either unit mode; an imperial dash shows 19


@pytest.mark.parametrize("floor, is_metric, expected", [
  (MAZDA_FLOOR, False, 19),
  (MAZDA_FLOOR, True, 30),
  (0., False, 20),
  (0., True, 30),
])
def test_floor_in_dash_units(floor, is_metric, expected):
  icbm = make_icbm("mazda", min_set_speed=floor)
  run_frames(icbm, target_mph=40, cluster_mph=40, is_metric=is_metric)
  assert icbm.v_cruise_min == expected


@pytest.mark.parametrize("floor, steps_down", [(MAZDA_FLOOR, True), (0., False)])
def test_steps_down_to_the_cars_floor(floor, steps_down):
  # a 15 mph target with the dash at 20: only a car whose ECU reaches 19 is pressed down
  sends = run_frames(make_icbm("mazda", min_set_speed=floor), target_mph=15, cluster_mph=20, n=500)
  assert any(s in (SendButtonState.decrease, SendButtonState.decreaseHold) for s in sends) == steps_down
