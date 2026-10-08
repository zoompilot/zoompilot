"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest

from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import icbm_moves_speed_limits, minimum_set_speed_ms
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode


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


@pytest.mark.parametrize("floor, is_metric, expected", [
  (0., False, 20 * CV.MPH_TO_MS),  # sunnypilot's default floor, in the dash unit
  (0., True, 30 * CV.KPH_TO_MS),
  (30 * CV.KPH_TO_MS, False, 30 * CV.KPH_TO_MS),  # a car's own floor holds in either unit
  (30 * CV.KPH_TO_MS, True, 30 * CV.KPH_TO_MS),
])
def test_minimum_set_speed(floor, is_metric, expected):
  assert minimum_set_speed_ms(custom.CarParamsSP(minimumSetSpeed=floor), is_metric) == pytest.approx(expected)
