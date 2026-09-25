"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from opendbc.car import structs
from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import icbm_applicable
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.helpers import pcm_machine_owns_sla, set_speed_limit_assist_availability


def car_params(brand: str, openpilot_longitudinal_control: bool, pcm_cruise: bool = True) -> structs.CarParams:
  CP = structs.CarParams.new_message()
  CP.brand = brand
  CP.pcmCruise = pcm_cruise
  CP.openpilotLongitudinalControl = openpilot_longitudinal_control
  return CP


def car_params_sp(pcm_cruise_speed: bool) -> custom.CarParamsSP:
  CP_SP = custom.CarParamsSP.new_message()
  CP_SP.pcmCruiseSpeed = pcm_cruise_speed
  return CP_SP


class TestSetSpeedLimitAssistAvailability:

  def setup_method(self):
    self.params = Params()

  def test_mazda_alpha_long_with_icbm_allowed(self):
    # ICBM clears pcmCruiseSpeed whoever commands acceleration: the arbiter owns the session
    self.params.put("SpeedLimitMode", int(Mode.assist), block=True)
    assert set_speed_limit_assist_availability(car_params("mazda", True), car_params_sp(False), self.params)
    assert self.params.get("SpeedLimitMode", return_default=True) == int(Mode.assist)

  def test_mazda_alpha_long_without_icbm_is_pcm_op_long(self):
    # no brand rule: like every pcm-op-long car, the planner machine and its required-max confirm
    self.params.put("SpeedLimitMode", int(Mode.assist), block=True)
    assert set_speed_limit_assist_availability(car_params("mazda", True), car_params_sp(True), self.params)

  def test_mazda_icbm_class_allowed(self):
    self.params.put("SpeedLimitMode", int(Mode.assist), block=True)
    assert set_speed_limit_assist_availability(car_params("mazda", False), car_params_sp(False), self.params)

  def test_mazda_stock_long_denied(self):
    self.params.put("SpeedLimitMode", int(Mode.assist), block=True)
    assert not set_speed_limit_assist_availability(car_params("mazda", False), car_params_sp(True), self.params)
    assert self.params.get("SpeedLimitMode", return_default=True) == int(Mode.warning)

  def test_mazda_op_long_without_pcm_allowed(self):
    assert set_speed_limit_assist_availability(car_params("mazda", True, pcm_cruise=False), car_params_sp(True), self.params)

  def test_toyota_op_long_allowed(self):
    assert set_speed_limit_assist_availability(car_params("toyota", True), car_params_sp(True), self.params)


class TestPcmMachineOwnership:
  """One predicate decides the machine everywhere (planner, arbiter, alert text)."""

  def test_only_a_driver_only_setpoint_keeps_the_planner_machine(self):
    assert pcm_machine_owns_sla(car_params("toyota", True), car_params_sp(True))
    assert not pcm_machine_owns_sla(car_params("mazda", True), car_params_sp(False))   # alpha long + ICBM
    assert not pcm_machine_owns_sla(car_params("mazda", False), car_params_sp(False))  # stock long + ICBM
    assert not pcm_machine_owns_sla(car_params("hyundai", True, pcm_cruise=False), car_params_sp(True))
    assert not pcm_machine_owns_sla(car_params("mazda", False), car_params_sp(True))

  def test_icbm_applies_wherever_the_ecu_keeps_the_setpoint(self):
    def cp_sp(available):
      CP_SP = car_params_sp(True)
      CP_SP.intelligentCruiseButtonManagementAvailable = available
      return CP_SP
    assert icbm_applicable(car_params("mazda", False), cp_sp(True))                    # stock long
    assert icbm_applicable(car_params("mazda", True), cp_sp(True))                     # alpha long, body cruise kept
    assert not icbm_applicable(car_params("hyundai", True, pcm_cruise=False), cp_sp(True))  # openpilot owns the setpoint
    assert not icbm_applicable(car_params("mazda", False), cp_sp(False))
