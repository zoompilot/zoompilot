"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import math

import numpy as np

from openpilot.cereal import messaging, custom
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_lead_gap.controller import E2ELeadGapController
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.controller import E2ESetSpeedController
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper
from openpilot.sunnypilot.selfdrive.controls.lib.lead_forecast.forecast import LeadForecast
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import make_smart_cruise_control
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.assist_mirror import SpeedLimitAssistMirror
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.helpers import pcm_machine_owns_sla
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.icbm_latch import icbm_active
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.models.helpers import get_active_bundle

DecState = custom.LongitudinalPlanSP.DynamicExperimentalControl.DynamicExperimentalControlState
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]


class LongitudinalPlannerSP:
  # set by the host planner before the e2e candidate is built
  CP: structs.CarParams
  allow_throttle: bool
  fcw: bool
  a_cruise: float
  v_desired_trajectory: np.ndarray
  a_desired_trajectory: np.ndarray

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, mpc):
    self.events_sp = EventsSP()
    self.dec = DynamicExperimentalController(CP, mpc)
    self.e2e_set_speed = E2ESetSpeedController()
    self.e2e_lead_gap = E2ELeadGapController()
    self.a_e2e = 0.  # the e2e candidate both lifts produced, for the floor's count of added speed
    self.lead_forecast = LeadForecast()
    self.lead_forecast.install(mpc)
    self.scc = make_smart_cruise_control(CP)
    self.CP = CP
    self.CP_SP = CP_SP
    self.resolver = SpeedLimitResolver(CP)
    # cars whose setpoint only the driver can move run the SLA machine here; everywhere
    # else it runs in card (the cruise arbiter, next to the buttons and the setpoint) and
    # gets mirrored (speed_limit.helpers.pcm_machine_owns_sla). On Mazda alpha long the owner
    # follows card's ICBM decision, which only moves between engagements.
    self.sla_machine = SpeedLimitAssist(CP, CP_SP) if pcm_machine_owns_sla(CP, icbm_active=False) else None
    self.sla_mirror = SpeedLimitAssistMirror(CP, CP_SP)
    self.sla = self._sla_owner(not CP_SP.pcmCruiseSpeed)
    self.generation = int(model_bundle.generation) if (model_bundle := get_active_bundle()) else None
    self.source = LongitudinalPlanSource.cruise
    self.e2e_alerts_helper = E2EAlertsHelper()

    self.output_v_target = 0.
    self.output_a_target = 0.
    self.seed_fault_logged = False

  def is_e2e(self, sm: messaging.SubMaster) -> bool:
    experimental_mode = sm['selfdriveState'].experimentalMode
    if not self.dec.active():
      return experimental_mode

    return experimental_mode and self.dec.mode() == "blended"

  def _sla_owner(self, icbm: bool):
    # the machine exists only on pcm openpilot-long cars, where it owns SLA while ICBM is passive
    return self.sla_machine if self.sla_machine is not None and not icbm else self.sla_mirror

  def update_targets(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float) -> tuple[float, float]:
    CS = sm['carState']
    v_cruise_cluster_kph = min(CS.vCruiseCluster, V_CRUISE_MAX)
    v_cruise_cluster = v_cruise_cluster_kph * CV.KPH_TO_MS

    long_enabled = sm['carControl'].enabled
    long_override = sm['carControl'].cruiseControl.override

    # Smart Cruise Control
    # SCC has nothing to act through without openpilot long or ICBM, the same condition
    # controlsd gates longActive on. Its params outlive that, so gate it here.
    icbm = icbm_active(sm['carStateSP'], self.CP_SP)
    scc_actionable = self.CP.openpilotLongitudinalControl or icbm
    self.scc.update(sm, long_enabled and scc_actionable, long_override, v_ego, a_ego, v_cruise)

    # Speed Limit Resolver
    self.resolver.update(v_ego, sm)

    # Speed Limit Assist
    sla = self._sla_owner(icbm)
    if sla is not self.sla:
      sla.reset()
      self.sla = sla
    if self.sla.pcm_op_long:
      has_speed_limit = self.resolver.speed_limit_valid or self.resolver.speed_limit_last_valid
      self.sla.update(long_enabled, long_override, v_ego, a_ego, v_cruise_cluster, self.resolver.speed_limit,
                      self.resolver.speed_limit_final_last, has_speed_limit, self.resolver.distance, self.events_sp)
    else:
      self.sla.update(sm['carStateSP'].zoompilot.cruiseSession, v_ego, self.resolver.distance, a_ego, self.events_sp)

    targets = {
      LongitudinalPlanSource.cruise: (v_cruise, a_ego),
      LongitudinalPlanSource.sccVision: (self.scc.vision.output_v_target, self.scc.vision.output_a_target),
      LongitudinalPlanSource.sccMap: (self.scc.map.output_v_target, self.scc.map.output_a_target),
      LongitudinalPlanSource.speedLimitAssist: (self.sla.output_v_target, self.sla.output_a_target),
    }

    self.source = min(targets, key=lambda k: targets[k][0])
    v_target, a_target = targets[self.source]

    # The pair returned here becomes the MPC seed (set_cur_state pins stage 0 to it), and a
    # single NaN in the seed poisons HPIPM's memory past acados_reset: every solve after it
    # fails, the plan stays at zero and the car never resumes. No source may reach the
    # solver with a non-finite target; fall back to what the cruise path would have given.
    if not (math.isfinite(v_target) and math.isfinite(a_target)):
      if not self.seed_fault_logged:
        self.seed_fault_logged = True
        cloudlog.error(f"longitudinal_planner: non-finite target from {self.source}: v={v_target} a={a_target}")
      v_target = v_cruise if math.isfinite(v_cruise) else v_ego
      a_target = a_ego if math.isfinite(a_ego) else 0.
      if not math.isfinite(v_target):
        v_target = 0.
      self.source = LongitudinalPlanSource.cruise

    self.output_v_target, self.output_a_target = v_target, a_target
    return self.output_v_target, self.output_a_target

  def update_e2e_target(self, sm: messaging.SubMaster, a_model: float, reset_state: bool, accel_coast: float) -> float:
    is_e2e, dec_active = self.is_e2e(sm), self.dec.active()
    # lateral acceleration from the measured steering, as the host's cruise candidate computes it
    CS = sm['carState']
    steer_deg = CS.steeringAngleDeg - sm['vehicleParameters'].angleOffsetDeg
    steer_lat_accel = CS.vEgo ** 2 * math.radians(steer_deg) / (self.CP.steerRatio * self.CP.wheelbase)
    # output_v_target is this frame's cruise target after SCC and SLA
    a_e2e = self.e2e_set_speed.update(sm, a_model, self.output_v_target, is_e2e, reset_state, dec_active,
                                      self.allow_throttle, self.fcw, accel_coast, steer_lat_accel)
    # the MPC candidate as the host planner takes it from this frame's solution
    a_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                action_t=self.CP.longitudinalActuatorDelay + DT_MDL)
    # a_cruise is last frame's: the host builds this frame's after the e2e candidate
    self.a_e2e = self.e2e_lead_gap.update(sm, a_e2e, float(a_mpc), is_e2e, reset_state, dec_active, self.allow_throttle,
                                          self.fcw, self.a_cruise, steer_lat_accel)
    return self.a_e2e

  def update(self, sm: messaging.SubMaster) -> None:
    self.events_sp.clear()
    self.dec.update(sm)
    self.lead_forecast.update(sm)
    self.e2e_alerts_helper.update(sm, self.events_sp)

  def publish_longitudinal_plan_sp(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    # the host has chosen: tell the set-speed floor how much of the lift reached the car
    self.e2e_set_speed.delivered(float(self.output_a_target), float(self.a_e2e))

    plan_sp_send = messaging.new_message('longitudinalPlanSP')

    plan_sp_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlanSP = plan_sp_send.longitudinalPlanSP
    longitudinalPlanSP.longitudinalPlanSource = self.source
    longitudinalPlanSP.vTarget = float(self.output_v_target)
    longitudinalPlanSP.aTarget = float(self.output_a_target)
    longitudinalPlanSP.events = self.events_sp.to_msg()

    # Dynamic Experimental Control
    dec = longitudinalPlanSP.dec
    dec.state = DecState.blended if self.dec.mode() == 'blended' else DecState.acc
    dec.enabled = self.dec.enabled()
    dec.active = self.dec.active()

    # Smart Cruise Control
    smartCruiseControl = longitudinalPlanSP.smartCruiseControl
    # Vision Control
    sccVision = smartCruiseControl.vision
    sccVision.state = self.scc.vision.state
    sccVision.vTarget = float(self.scc.vision.output_v_target)
    sccVision.aTarget = float(self.scc.vision.output_a_target)
    sccVision.currentLateralAccel = float(self.scc.vision.current_lat_acc)
    sccVision.maxPredictedLateralAccel = float(self.scc.vision.max_pred_lat_acc)
    sccVision.enabled = self.scc.vision.is_enabled
    sccVision.active = self.scc.vision.is_active
    # zoompilot's planner only; 0 tells the ICBM servo there is no lookahead
    sccVision.vAheadMin = float(getattr(self.scc.vision, 'v_ahead_min', 0.))
    # Map Control
    sccMap = smartCruiseControl.map
    sccMap.state = self.scc.map.state
    sccMap.vTarget = float(self.scc.map.output_v_target)
    sccMap.aTarget = float(self.scc.map.output_a_target)
    sccMap.enabled = self.scc.map.is_enabled
    sccMap.active = self.scc.map.is_active

    # Speed Limit
    speedLimit = longitudinalPlanSP.speedLimit
    resolver = speedLimit.resolver
    resolver.speedLimit = float(self.resolver.speed_limit)
    resolver.speedLimitLast = float(self.resolver.speed_limit_last)
    resolver.speedLimitFinal = float(self.resolver.speed_limit_final)
    resolver.speedLimitFinalLast = float(self.resolver.speed_limit_final_last)
    resolver.speedLimitValid = self.resolver.speed_limit_valid
    resolver.speedLimitLastValid = self.resolver.speed_limit_last_valid
    resolver.speedLimitOffset = float(self.resolver.speed_limit_offset)
    resolver.distToSpeedLimit = float(self.resolver.distance)
    resolver.source = self.resolver.source
    assist = speedLimit.assist
    assist.state = self.sla.state
    assist.enabled = self.sla.is_enabled
    assist.active = self.sla.is_active
    assist.vTarget = float(self.sla.output_v_target)
    assist.aTarget = float(self.sla.output_a_target)

    # E2E Alerts
    e2eAlerts = longitudinalPlanSP.e2eAlerts
    e2eAlerts.greenLightAlert = self.e2e_alerts_helper.green_light_alert
    e2eAlerts.leadDepartAlert = self.e2e_alerts_helper.lead_depart_alert

    # zoompilot: experimental mode's set-speed floor
    e2eSetSpeed = longitudinalPlanSP.zoompilot.e2eSetSpeed
    e2eSetSpeed.authority = float(self.e2e_set_speed.authority)
    e2eSetSpeed.gain = float(self.e2e_set_speed.gain)
    e2eSetSpeed.floor = float(self.e2e_set_speed.floor)
    e2eSetSpeed.boost = float(self.e2e_set_speed.boost)
    e2eSetSpeed.inhibit = self.e2e_set_speed.inhibit
    e2eSetSpeed.bound = float(self.e2e_set_speed.bound) if math.isfinite(self.e2e_set_speed.bound) else 0.
    e2eSetSpeed.added = float(self.e2e_set_speed.added)

    # zoompilot: experimental mode's follow-distance assist
    e2eLeadGap = longitudinalPlanSP.zoompilot.e2eLeadGap
    e2eLeadGap.authority = float(self.e2e_lead_gap.authority)
    e2eLeadGap.gain = float(self.e2e_lead_gap.gain)
    e2eLeadGap.weight = float(self.e2e_lead_gap.weight)
    e2eLeadGap.gapExcess = float(self.e2e_lead_gap.gap_excess)
    e2eLeadGap.boost = float(self.e2e_lead_gap.boost)
    e2eLeadGap.inhibit = self.e2e_lead_gap.inhibit

    # zoompilot: the model's lead forecast in the MPC
    leadForecast = longitudinalPlanSP.zoompilot.leadForecast
    for report, weight, inhibit, lead_xv in zip((leadForecast.leadOne, leadForecast.leadTwo), self.lead_forecast.weights,
                                                self.lead_forecast.inhibits, self.lead_forecast.lead_xv, strict=True):
      report.weight = float(weight)
      report.inhibit = inhibit
      if lead_xv is not None:
        report.x = lead_xv[:, 0].tolist()
        report.v = lead_xv[:, 1].tolist()

    pm.send('longitudinalPlanSP', plan_sp_send)
