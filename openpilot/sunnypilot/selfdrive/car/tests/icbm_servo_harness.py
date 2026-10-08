"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Shared drivers for the ICBM servo tests: build a button-actuated car's servo and run it
against a synthetic dash for n frames, collecting the button it emits each frame.
"""
from openpilot.cereal import custom
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import IntelligentCruiseButtonManagement
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import V_CRUISE_UNSET

SessionState = custom.LongitudinalPlanSP.SpeedLimit.AssistState


def make_icbm(brand="", op_long=False):
  return IntelligentCruiseButtonManagement(car.CarParams(pcmCruise=True, brand=brand, openpilotLongitudinalControl=op_long),
                                           custom.CarParamsSP(pcmCruiseSpeed=False, intelligentCruiseButtonManagementAvailable=True))


def run_frames(icbm, target_mph, cluster_mph, n=1, source='sccVision', is_metric=False,
               v_ego_mph=None, a_target=0., session_state=SessionState.disabled,
               v_ahead_min_mph=0., button_events=None, v_cruise_mph=None, mpc_a_target=None):
  """Run the servo for n frames against a fixed plan target and dash; returns the sends.

  a_target is the active limiter's own request; mpc_a_target, when given, is what the MPC
  puts on LP_SP.aTarget instead (it rails once the target sits a few mph under vEgo)."""
  sends = []
  for i in range(n):
    CS = car.CarState(cruiseState={"speedCluster": cluster_mph * CV.MPH_TO_MS})
    if v_ego_mph is not None:
      CS.vEgo = float(v_ego_mph * CV.MPH_TO_MS)
    if v_cruise_mph is not None:
      CS.vCruise = float(v_cruise_mph * CV.MPH_TO_KPH)
    if button_events and i == 0:
      CS.buttonEvents = button_events
    CC = car.CarControl(enabled=True)
    LP_SP = custom.LongitudinalPlanSP(vTarget=target_mph * CV.MPH_TO_MS)
    LP_SP.longitudinalPlanSource = source
    LP_SP.aTarget = float(a_target if mpc_a_target is None else mpc_a_target)
    # the servo reads each limiter's own request, not the MPC's railed aTarget
    LP_SP.smartCruiseControl.vision.aTarget = float(a_target)
    LP_SP.smartCruiseControl.map.aTarget = float(a_target)
    LP_SP.speedLimit.assist.aTarget = float(a_target)
    LP_SP.smartCruiseControl.vision.vAheadMin = float(v_ahead_min_mph * CV.MPH_TO_MS)
    LP_SP.speedLimit.assist.state = session_state
    LP_SP.speedLimit.assist.vTarget = V_CRUISE_UNSET  # the mirror's idle value
    icbm.run(CS, CC, LP_SP, is_metric=is_metric)
    sends.append(icbm.cruise_button)
  return sends
