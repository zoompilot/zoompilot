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
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import IntelligentCruiseButtonManagement

SessionState = custom.LongitudinalPlanSP.SpeedLimit.AssistState


def make_icbm(brand=""):
  return IntelligentCruiseButtonManagement(car.CarParams(pcmCruise=True, brand=brand),
                                           custom.CarParamsSP(pcmCruiseSpeed=False))


def run_frames(icbm, target_mph, cluster_mph, n=1, source='sccVision', is_metric=False,
               v_ego_mph=None, a_target=0., overshoot=False, session_state=SessionState.disabled,
               v_ahead_min_mph=0., button_events=None):
  """Run the servo for n frames against a fixed plan target and dash; returns the sends."""
  # the toggle is a param the servo re-reads on its own cadence; set both so a flip takes
  # effect on this call's first frame
  Params().put_bool("SmartCruiseDecelOvershoot", overshoot)
  icbm.decel_overshoot_enabled = overshoot
  sends = []
  for i in range(n):
    CS = car.CarState(cruiseState={"speedCluster": cluster_mph * CV.MPH_TO_MS})
    if v_ego_mph is not None:
      CS.vEgo = float(v_ego_mph * CV.MPH_TO_MS)
    if button_events and i == 0:
      CS.buttonEvents = button_events
    CC = car.CarControl(enabled=True)
    LP_SP = custom.LongitudinalPlanSP(vTarget=target_mph * CV.MPH_TO_MS)
    LP_SP.longitudinalPlanSource = source
    LP_SP.aTarget = float(a_target)
    LP_SP.smartCruiseControl.vision.vAheadMin = float(v_ahead_min_mph * CV.MPH_TO_MS)
    LP_SP.speedLimit.assist.state = session_state
    icbm.run(CS, CC, LP_SP, is_metric=is_metric)
    sends.append(icbm.cruise_button)
  return sends
