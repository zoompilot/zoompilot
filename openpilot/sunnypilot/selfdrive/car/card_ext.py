"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.car.alpha_long_toggle import AlphaLongToggleMonitor


class CardExt:
  """sunnypilot's per-frame hooks into card, one object so card.py carries one-line call sites.

  Ordering contract with card.state_update: update_v_cruise_post runs after update_v_cruise
  and initialize_v_cruise and before CS.vCruise is read, so the reconciled setpoint is the
  one published. sm is card's SubMaster, already updated this frame.
  """

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, params: Params, sm, v_cruise_helper) -> None:
    self.sm = sm
    self.v_cruise_helper = v_cruise_helper
    # onroad AlphaLongitudinalEnabled changes: sequence any ECU hand-back, then cycle
    self.alpha_long_monitor = AlphaLongToggleMonitor(CP, params)

  def update_v_cruise_post(self, CS, CS_SP) -> None:
    helper = self.v_cruise_helper
    # the regime the reconciler gates on, from the same messages this frame saw
    helper.update_plan_regime(self.sm['longitudinalPlanSP'], self.sm['carControlSP'])
    helper.reconcile_setpoint_with_dash(CS)
    # publish the arbiter's session (plannerd mirrors it; the ICBM servo freezes on a prompt)
    helper.cruise_arbiter.fill_msg(CS_SP)

  def controls_update(self, CS, CC, CC_SP: structs.CarControlSP) -> structs.CarControlSP:
    """Runs just before CI.apply on the converted CarControlSP struct, which it may edit."""
    self.v_cruise_helper.cruise_arbiter.gate_send_button(CC_SP)
    self.alpha_long_monitor.update(CS, CC, CC_SP)
    return CC_SP

  def update_params(self) -> None:
    # rides card's params thread, keeping param reads off the 100 Hz path
    self.alpha_long_monitor.update_params()
