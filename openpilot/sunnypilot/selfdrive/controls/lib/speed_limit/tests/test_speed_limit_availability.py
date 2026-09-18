"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.helpers import set_speed_limit_assist_availability


def car_params(brand: str, openpilot_longitudinal_control: bool) -> structs.CarParams:
  CP = structs.CarParams.new_message()
  CP.brand = brand
  CP.pcmCruise = True
  CP.openpilotLongitudinalControl = openpilot_longitudinal_control
  return CP


def car_params_sp(pcm_cruise_speed: bool) -> custom.CarParamsSP:
  CP_SP = custom.CarParamsSP.new_message()
  CP_SP.pcmCruiseSpeed = pcm_cruise_speed
  return CP_SP


class TestSetSpeedLimitAssistAvailability:

  def setup_method(self):
    self.params = Params()

  def test_mazda_op_long_demoted_to_warning(self):
    self.params.put("SpeedLimitMode", int(Mode.assist), block=True)
    allowed = set_speed_limit_assist_availability(car_params("mazda", True), car_params_sp(True), self.params)
    assert not allowed
    assert self.params.get("SpeedLimitMode", return_default=True) == int(Mode.warning)

  def test_mazda_icbm_class_allowed(self):
    self.params.put("SpeedLimitMode", int(Mode.assist), block=True)
    assert set_speed_limit_assist_availability(car_params("mazda", False), car_params_sp(False), self.params)

  def test_mazda_stock_long_denied(self):
    assert not set_speed_limit_assist_availability(car_params("mazda", False), car_params_sp(True), self.params)

  def test_toyota_op_long_allowed(self):
    assert set_speed_limit_assist_availability(car_params("toyota", True), car_params_sp(True), self.params)
