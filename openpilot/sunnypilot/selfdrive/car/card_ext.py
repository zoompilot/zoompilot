"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from opendbc.car import structs
from opendbc.sunnypilot.car.stock_ecu import StockEcuState
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.car.stock_ecu_handback import StockEcuHandBackServer


class CardExt:
  """sunnypilot's per-frame hooks into card, one object so card.py carries one-line call sites.

  Ordering contract with card.state_update: update_v_cruise_post runs after update_v_cruise
  and initialize_v_cruise and before CS.vCruise is read, so the reconciled setpoint is the
  one published. sm is card's SubMaster, already updated this frame.
  """

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, params: Params, sm, v_cruise_helper, CI) -> None:
    self.sm = sm
    self.v_cruise_helper = v_cruise_helper
    # The stock ECU transition contract (opendbc/sunnypilot/car/stock_ecu.py): a controller that
    # silences a stock ECU under openpilot longitudinal exposes `stock_ecu_state`; nothing
    # brand-specific is read here. The hand-back server answers the lifecycle's requests off it.
    self.controller = CI.CC
    self.handback = StockEcuHandBackServer(params)

  def update_v_cruise_post(self, CS, CS_SP) -> None:
    helper = self.v_cruise_helper
    # the regime the reconciler gates on, from the same messages this frame saw
    helper.update_plan_regime(self.sm['longitudinalPlanSP'], self.sm['carControlSP'])
    helper.reconcile_setpoint_with_dash(CS)
    # publish the arbiter's session (plannerd mirrors it; the ICBM servo freezes on a prompt)
    helper.cruise_arbiter.fill_msg(CS_SP)
    # the driver's view of the stock ECU, for the engage-press alert (stockEcuNotReady)
    CS_SP.zoompilot.stockEcu = str(self.stock_ecu_state)

  @property
  def stock_ecu_state(self) -> StockEcuState:
    return getattr(self.controller, "stock_ecu_state", StockEcuState.NOT_NEEDED)

  def controls_update(self, CS, CC, CC_SP: structs.CarControlSP) -> structs.CarControlSP:
    """Runs just before CI.apply on the converted CarControlSP struct, which it may edit."""
    self.v_cruise_helper.cruise_arbiter.gate_send_button(CC_SP)
    self.handback.update(CC.enabled, self.stock_ecu_state, CC_SP)
    return CC_SP

  def update_params(self) -> None:
    # rides card's params thread, keeping param reads off the 100 Hz path
    self.handback.update_params()
