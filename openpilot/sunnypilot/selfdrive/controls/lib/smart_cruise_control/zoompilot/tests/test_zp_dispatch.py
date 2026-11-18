"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Only measured brands run zoompilot's curve planners; every other brand gets sunnypilot's own,
wrapped only for the Curve Speed profile.
"""
import pytest

from opendbc.car import structs
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import map_controller as sp_map
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import vision_controller as sp_vision
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import TUNED_BRANDS, make_smart_cruise_control
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import curve_speed_profile as zp_profile
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import map_controller as zp_map
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import stock_vision
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import vision_controller as zp_vision


@pytest.mark.parametrize("op_long", [False, True])
@pytest.mark.parametrize("brand", ["mazda", "hyundai", "honda", "toyota", "chrysler", "gm", "ford", "subaru", "body"])
def test_brand_selects_the_planner(brand, op_long):
  scc = make_smart_cruise_control(structs.CarParams(brand=brand, openpilotLongitudinalControl=op_long))
  if brand in TUNED_BRANDS:
    assert type(scc.vision) is zp_vision.SmartCruiseControlVision
    assert type(scc.map) is zp_map.SmartCruiseControlMap
  else:
    assert type(scc.vision) is stock_vision.SmartCruiseControlVision
    assert type(scc.map) is zp_profile.SmartCruiseControlMap
    assert stock_vision.SmartCruiseControlVision.__bases__[-1] is zp_profile.SmartCruiseControlVision
    assert zp_profile.SmartCruiseControlVision.__bases__[-1] is sp_vision.SmartCruiseControlVision
    assert zp_profile.SmartCruiseControlMap.__bases__[-1] is sp_map.SmartCruiseControlMap
    assert not hasattr(scc.vision, 'v_ahead_min')  # the planner publishes 0: no lookahead


def test_only_mazda_is_tuned():
  assert TUNED_BRANDS == ('mazda',)


def test_stock_planner_learns_and_caps_on_the_learned_ceiling():
  from types import SimpleNamespace
  from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.lat_limit_learner import LatLimitLearner
  v = stock_vision.SmartCruiseControlVision(structs.CarParams(brand="toyota", carFingerprint="TOYOTA_RAV4"))
  assert isinstance(v.lat_limit, LatLimitLearner)
  assert list(v.lat_limit.ceiling) == [2.0, 2.0, 2.0]
  v.long_enabled, v.v_ego, v.learn_lat_limit = True, 25., True
  v.speed_scale = 1.0
  v.lat_limit.ceiling = v.lat_limit.ceiling * 0 + 1.5
  v.lat_limit.learned = True
  sm = {'modelV2': SimpleNamespace(orientationRate=SimpleNamespace(z=[0.1] * 33), velocity=SimpleNamespace(x=[25.] * 33)),
        'controlsState': SimpleNamespace(curvature=0.)}
  v._update_calculations(sm)
  assert v.v_target == pytest.approx(25. * (1.5 / v.max_pred_lat_acc) ** 0.5)
