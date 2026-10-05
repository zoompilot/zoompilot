"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Shared harness for the vision controller tests. The controller is exercised with
synthetic roads: kappa(s) profiles rendered into the time-indexed model arrays the way
the model would report them at a given speed. The activation distances asserted in the
tests follow from the platform limits in limits.py; if those constants move, the
geometry in the tests moves with them.
"""
import contextlib
from typing import Any
from unittest import mock

import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.selfdrive.modeld.constants import ModelConstants
from opendbc.car import structs
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import vision_controller
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.vision_controller import SmartCruiseControlVision

V_EGO = 20.
SETPOINT = 20.
# the lateral ceiling at SETPOINT, before the planning margin
A_LAT_MAX = float(np.interp(SETPOINT, vision_controller._A_LAT_REG_V_BP, vision_controller._A_LAT_REG_V))
CURVE_KAPPA = 0.02  # r = 50 m
CURVE_V = (A_LAT_MAX / CURVE_KAPPA) ** 0.5  # allowed speed at the ceiling, ~9.9 m/s


def make_cp(op_long: bool = True) -> structs.CarParams:
  return structs.CarParams(brand="mazda", openpilotLongitudinalControl=op_long,
                           longitudinalActuatorDelay=0.36)


# What the model does to curvature with range, measured over 26 apexes on route 135 (see
# vision_controller._KAPPA_BIAS_GAIN). A test road rendered without it is a perfect sensor,
# which the correction over-reads: past 30 m it plans the road up to 1.5x tighter than it is.
ATTENUATION_D = [0., 30., 50., 70., 90., 110., 130., 200.]
ATTENUATION = [1.0, 0.94, 0.88, 0.79, 0.66, 0.55, 0.30, 0.30]


def model_for_road(v: float, kappa_fn, v_model: float | None = None, attenuate: bool = False, big: bool = False):
  """Render kappa(s) into model arrays as the model would report driving it at speed v.

  v_model lets the model's own velocity plan differ from v (a planned slowdown); the yaw
  rate follows the planned velocity, exactly as the model reports it.

  attenuate applies the measured range under-read, so the road reaches the controller the
  way the real model would report it rather than as perfect geometry. big marks the frame as
  the big model's (modelV2.big), which selects its planning horizon.
  """
  t = np.array(ModelConstants.T_IDXS)
  s = v * t
  vm = v if v_model is None else v_model
  if attenuate:
    reported = kappa_fn

    def kappa_fn(si, _true=reported):
      return _true(si) * float(np.interp(si, ATTENUATION_D, ATTENUATION))

  model = messaging.new_message('modelV2')
  position = log.XYZTData.new_message()
  position.x = [float(si) for si in s]
  position.y = [0.0] * len(t)
  model.modelV2.position = position
  velocity = log.XYZTData.new_message()
  velocity.x = [float(vm)] * len(t)
  model.modelV2.velocity = velocity
  orientation_rate = log.XYZTData.new_message()
  orientation_rate.z = [float(kappa_fn(si) * vm) for si in s]
  model.modelV2.orientationRate = orientation_rate
  model.modelV2.big = big
  return model


def curve_at(d_curve: float, kappa: float = CURVE_KAPPA):
  return lambda s: kappa if s >= d_curve else 0.


@contextlib.contextmanager
def patch_gain(gain):
  """Run with a different far-field curvature correction, to isolate what it buys."""
  saved = vision_controller._KAPPA_BIAS_GAIN
  vision_controller._KAPPA_BIAS_GAIN = gain
  try:
    yield
  finally:
    vision_controller._KAPPA_BIAS_GAIN = saved


def flat_ceiling(a_lat: float):
  """A flat lateral ceiling, for tests whose geometry pins numbers to one ceiling."""
  return mock.patch.object(vision_controller, '_A_LAT_REG_V', [a_lat, a_lat])


class VisionCase:
  """Base for the controller tests: a fresh enabled controller per test and road runners.

  A plain class, not OpenpilotTestCase: that is a unittest.TestCase and pytest's
  parametrize cannot be applied to its methods. The only param involved is set in
  every setup_method, so per-test prefix isolation is not needed.
  """

  def setup_method(self):
    self.params = Params()
    self.params.put_bool("SmartCruiseControlVision", True, block=True)
    self.scc_v = SmartCruiseControlVision(make_cp())

  @staticmethod
  def stock() -> SmartCruiseControlVision:
    return SmartCruiseControlVision(make_cp(op_long=False))

  def make_sm(self, v: float, kappa_fn, cur_curvature: float = 0., v_model: float | None = None,
              attenuate: bool = False, big: bool = False) -> Any:
    controls_state = messaging.new_message('controlsState')
    controls_state.controlsState.curvature = float(cur_curvature)
    return {'modelV2': model_for_road(v, kappa_fn, v_model, attenuate, big).modelV2,
            'controlsState': controls_state.controlsState}

  def run_road(self, v: float, kappa_fn, n: int = 3, cur_curvature: float = 0.,
               v_model: float | None = None, setpoint: float = SETPOINT,
               enabled: bool = True, override: bool = False, scc=None, attenuate: bool = False, big: bool = False):
    scc = scc or self.scc_v
    sm = self.make_sm(v, kappa_fn, cur_curvature, v_model, attenuate, big)
    for _ in range(n):
      scc.update(sm, enabled, override, v, 0., setpoint)
    return scc
