"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Only measured brands run zoompilot's curve planners; every other brand gets sunnypilot's own.
"""
import pytest

from opendbc.car import structs
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import map_controller as sp_map
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import vision_controller as sp_vision
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import TUNED_BRANDS, make_smart_cruise_control
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import map_controller as zp_map
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import vision_controller as zp_vision


@pytest.mark.parametrize("op_long", [False, True])
@pytest.mark.parametrize("brand", ["mazda", "hyundai", "honda", "toyota", "chrysler", "gm", "ford", "subaru", "body"])
def test_brand_selects_the_planner(brand, op_long):
  scc = make_smart_cruise_control(structs.CarParams(brand=brand, openpilotLongitudinalControl=op_long))
  if brand in TUNED_BRANDS:
    assert type(scc.vision) is zp_vision.SmartCruiseControlVision
    assert type(scc.map) is zp_map.SmartCruiseControlMap
  else:
    assert type(scc.vision) is sp_vision.SmartCruiseControlVision
    assert type(scc.map) is sp_map.SmartCruiseControlMap
    assert not hasattr(scc.vision, 'v_ahead_min')  # the planner publishes 0: no lookahead


def test_only_mazda_is_tuned():
  assert TUNED_BRANDS == ('mazda',)
