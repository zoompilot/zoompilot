"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Curve Speed profiles scale the lateral ceiling on both planner paths. Normal must leave every
output bit-identical to the unscaled planners.
"""
import copy
import json
import math

import numpy as np
import pytest

import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import vision_controller as sp_vision
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.speed_profile import allowed_speed
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import curve_speed_profile as profile
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import map_controller as zp_map
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import vision_controller as zp_vision
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.tests.vision_harness import make_cp, model_for_road

GENTLE, NORMAL, SPORTY = 0, 1, 2
KAPPAS = np.linspace(0.0005, 0.03, 200)
TARGETS = [{"velocity": 20.0, "latitude": 1.0, "longitude": 2.0}, {"velocity": 13.3, "latitude": 1.1, "longitude": 2.1}]


@pytest.fixture
def params():
  p = Params()
  p.put_bool("SmartCruiseControlVision", True, block=True)
  p.put_bool("SmartCruiseControlMap", True, block=True)
  yield p
  for key in ("CurveSpeedProfile", "SmartCruiseControlVision", "SmartCruiseControlMap"):
    p.remove(key)


def sm_for(v, kappa):
  cs = messaging.new_message('controlsState')
  cs.controlsState.curvature = float(kappa)
  return {'modelV2': model_for_road(v, lambda s: kappa).modelV2, 'controlsState': cs.controlsState}


class TestReadScale:
  @pytest.mark.parametrize(("stored", "scale"), [(GENTLE, 0.85), (NORMAL, 1.0), (SPORTY, 1.1), (-3, 0.85), (9, 1.1)])
  def test_stored_value_picks_the_scale(self, params, stored, scale):
    params.put("CurveSpeedProfile", stored, block=True)
    assert profile.read_scale(params) == scale

  def test_unset_is_normal(self, params):
    params.remove("CurveSpeedProfile")
    assert profile.read_scale(params) == 1.0


class TestMazdaTable:
  def test_normal_is_bit_identical(self):
    unscaled = allowed_speed(KAPPAS, np.interp(KAPPAS, zp_vision._A_LAT_KAPPA_BP, zp_vision._A_LAT_KAPPA))
    assert np.array_equal(zp_vision.curve_speed(KAPPAS, zp_vision.curve_table(1.0)), unscaled)
    assert np.array_equal(zp_vision.curve_speed(KAPPAS), unscaled)

  @pytest.mark.parametrize("scale", profile.SCALES)
  def test_ceiling_scales_at_each_breakpoint(self, scale):
    for v, a_lat in zip(zp_vision._A_LAT_CURVE_BP, zp_vision._A_LAT_CURVE, strict=True):
      a = a_lat * scale * zp_vision._PLAN_MARGIN
      kappa = a / v ** 2
      assert float(zp_vision.curve_speed(kappa, zp_vision.curve_table(scale))) == pytest.approx(v)

  def test_profiles_order(self):
    gentle, normal, sporty = (zp_vision.curve_speed(KAPPAS, zp_vision.curve_table(s)) for s in profile.SCALES)
    assert np.all(gentle < normal) and np.all(normal < sporty)


class TestVisionControllers:
  @pytest.mark.parametrize("stored", [GENTLE, NORMAL, SPORTY])
  def test_mazda_near_field_follows_the_profile(self, params, stored):
    params.put("CurveSpeedProfile", stored, block=True)
    scc = zp_vision.SmartCruiseControlVision(make_cp())
    kappa = 0.02
    for _ in range(3):
      scc.update(sm_for(12., kappa), True, False, 12., 0., 20.)
    # the model arrays are float32
    assert scc.v_near_min == pytest.approx(float(zp_vision.curve_speed(kappa, zp_vision.curve_table(profile.SCALES[stored]))), rel=1e-6)

  @pytest.mark.parametrize("stored", [GENTLE, NORMAL, SPORTY])
  def test_sunnypilot_ceiling_scales(self, params, stored):
    params.put("CurveSpeedProfile", stored, block=True)
    scaled, plain = profile.SmartCruiseControlVision(), sp_vision.SmartCruiseControlVision()
    for _ in range(3):
      for scc in (scaled, plain):
        scc.update(sm_for(20., 0.01), True, False, 20., 0., 25.)
    if stored == NORMAL:
      assert scaled.v_target == plain.v_target
      assert scaled.output_v_target == plain.output_v_target
    else:
      assert scaled.v_target == pytest.approx(plain.v_target * math.sqrt(profile.SCALES[stored]))
      # the same as the flat ceiling scaled directly
      max_curve = scaled.max_pred_lat_acc / 20. ** 2
      assert scaled.v_target == pytest.approx((sp_vision._A_LAT_REG_MAX * profile.SCALES[stored] / max_curve) ** 0.5)

  def test_sunnypilot_stale_target_does_not_compound(self, params):
    params.put("CurveSpeedProfile", SPORTY, block=True)
    scc = profile.SmartCruiseControlVision()
    scc.update(sm_for(20., 0.01), True, False, 20., 0., 25.)
    v_target = scc.v_target
    for _ in range(5):
      scc.update(sm_for(20., 0.01), False, False, 20., 0., 25.)
    assert scc.v_target == v_target


class TestMapControllers:
  @pytest.mark.parametrize("make", [profile.SmartCruiseControlMap, lambda: zp_map.SmartCruiseControlMap(make_cp())])
  @pytest.mark.parametrize("stored", [GENTLE, NORMAL, SPORTY])
  def test_targets_scale_by_the_root(self, params, make, stored):
    params.put("CurveSpeedProfile", stored, block=True)
    scc = make()
    scc.target_velocities = copy.deepcopy(TARGETS)
    root = math.sqrt(profile.SCALES[stored])
    assert [p["velocity"] for p in scc.target_velocities] == [p["velocity"] * root for p in TARGETS]
    assert [(p["latitude"], p["longitude"]) for p in scc.target_velocities] == [(p["latitude"], p["longitude"]) for p in TARGETS]
    if stored == NORMAL:
      assert scc.target_velocities == TARGETS

  def test_targets_are_parsed_fresh_each_frame(self, params):
    """The setter scales in place: the controllers' list must be their own each frame."""
    from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.map_controller import velocities_from_param
    params.put("MapTargetVelocities", json.dumps(TARGETS), block=True)
    try:
      first, second = (velocities_from_param("MapTargetVelocities", params) for _ in range(2))
      assert first == second == TARGETS
      assert first is not second and first[0] is not second[0]
    finally:
      params.remove("MapTargetVelocities")


class TestRefresh:
  def test_profile_change_waits_for_the_params_refresh(self, params):
    params.put("CurveSpeedProfile", NORMAL, block=True)
    scc = profile.SmartCruiseControlVision()
    period = int(PARAMS_UPDATE_PERIOD / DT_MDL)
    scc.update(sm_for(20., 0.001), True, False, 20., 0., 25.)  # frame -1
    scc.update(sm_for(20., 0.001), True, False, 20., 0., 25.)  # frame 0 refreshes
    params.put("CurveSpeedProfile", SPORTY, block=True)
    for _ in range(period - 1):
      scc.update(sm_for(20., 0.001), True, False, 20., 0., 25.)
      assert scc.lat_scale == 1.0
    scc.update(sm_for(20., 0.001), True, False, 20., 0., 25.)
    assert scc.lat_scale == 1.1
