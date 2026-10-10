"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest

from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import icbm_moves_speed_limits, minimum_set_speed_ms
from openpilot.sunnypilot.selfdrive.car.tests.icbm_servo_harness import make_icbm, run_frames
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode

SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState

MAZDA_FLOOR = 30. * CV.KPH_TO_MS  # MRCC's floor in either unit mode; an imperial dash shows 19


@pytest.mark.parametrize(("has_long", "has_icbm", "mode", "moves"), [
  (True, True, Mode.assist, True),
  (True, True, Mode.warning, False),  # warn never moves the set speed
  (True, False, Mode.assist, False),  # alpha long alone: the planner prompts
  (False, True, Mode.assist, False),  # stock ACC: ICBM owns everything, no split to show
])
def test_icbm_moves_speed_limits(has_long, has_icbm, mode, moves):
  assert icbm_moves_speed_limits(has_long, has_icbm, mode) is moves


@pytest.mark.parametrize("mode", [3, None])
def test_mode_as_stored_param(mode):
  # the MICI layout passes the raw param int, the TICI one ui_state's value (None until read)
  assert icbm_moves_speed_limits(True, True, mode) is (mode == 3)


@pytest.mark.parametrize("floor, is_metric, expected_ms, dash", [
  (0., False, 20 * CV.MPH_TO_MS, 20),  # sunnypilot's default floor, in the dash unit
  (0., True, 30 * CV.KPH_TO_MS, 30),
  (MAZDA_FLOOR, False, MAZDA_FLOOR, 19),  # a car's own floor holds in either unit
  (MAZDA_FLOOR, True, MAZDA_FLOOR, 30),
])
def test_minimum_set_speed(floor, is_metric, expected_ms, dash):
  assert minimum_set_speed_ms(custom.CarParamsSP(minimumSetSpeed=floor), is_metric) == pytest.approx(expected_ms)
  # the servo compares it (CarParamsSP.minimumSetSpeed) in whole dash units
  icbm = make_icbm("mazda", min_set_speed=floor)
  run_frames(icbm, target_mph=40, cluster_mph=40, is_metric=is_metric)
  assert icbm.v_cruise_min == dash


@pytest.mark.parametrize("floor, steps_down", [(MAZDA_FLOOR, True), (0., False)])
def test_servo_steps_down_to_the_cars_floor(floor, steps_down):
  # a 15 mph target with the dash at 20: only a car whose ECU reaches 19 is pressed down
  sends = run_frames(make_icbm("mazda", min_set_speed=floor), target_mph=15, cluster_mph=20, n=500)
  assert any(s in (SendButtonState.decrease, SendButtonState.decreaseHold) for s in sends) == steps_down
