"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

zoompilot's curve planners run on openpilot long for every brand, and on stock ACC only for the
measured brands; every other stock ACC brand gets sunnypilot's own, wrapped only for the Curve Speed
profile.
"""
import pytest

from opendbc.car import structs
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import map_controller as sp_map
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import vision_controller as sp_vision
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import TUNED_BRANDS, make_smart_cruise_control
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import curve_speed_profile as zp_profile
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import map_controller as zp_map
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import vision_controller as zp_vision


@pytest.mark.parametrize("op_long", [False, True])
@pytest.mark.parametrize("brand", ["mazda", "hyundai", "honda", "toyota", "chrysler", "gm", "ford", "subaru", "body"])
def test_brand_selects_the_planner(brand, op_long):
  scc = make_smart_cruise_control(structs.CarParams(brand=brand, openpilotLongitudinalControl=op_long))
  if op_long or brand in TUNED_BRANDS:
    assert type(scc.vision) is zp_vision.SmartCruiseControlVision
    assert type(scc.map) is zp_map.SmartCruiseControlMap
    assert scc.vision.limits.op_long == op_long
  else:
    assert type(scc.vision) is zp_profile.SmartCruiseControlVision
    assert type(scc.map) is zp_profile.SmartCruiseControlMap
    assert zp_profile.SmartCruiseControlVision.__bases__[-1] is sp_vision.SmartCruiseControlVision
    assert zp_profile.SmartCruiseControlMap.__bases__[-1] is sp_map.SmartCruiseControlMap
    assert not hasattr(scc.vision, 'v_ahead_min')  # the planner publishes 0: no lookahead


def test_only_mazda_is_tuned():
  assert TUNED_BRANDS == ('mazda',)


def test_op_long_plans_on_the_cars_own_actuator_delay():
  # no stock ACC plant is involved: the budget is the planner's, the lead is the car's
  cp = structs.CarParams(brand="honda", openpilotLongitudinalControl=True, longitudinalActuatorDelay=0.35)
  lim = make_smart_cruise_control(cp).vision.limits
  assert lim.op_long and lim.t_lead == pytest.approx(0.35)
  assert lim.dash_traversal_time(10.) == 0.
