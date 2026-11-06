"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import math

import numpy as np

from openpilot.cereal import messaging, custom
from opendbc.car import structs
from opendbc.car.interfaces import ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.selfdrive.controls.lib.accel_boost import AccelBoost
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.approach import approach_accel
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.controller import E2ESetSpeedController
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper
from openpilot.sunnypilot.selfdrive.controls.lib.lead_forecast.forecast import LeadForecast
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.limits import OP_LONG_A_BUDGET, op_long_cruise_jerk
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import make_smart_cruise_control
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.assist_mirror import SpeedLimitAssistMirror
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.helpers import pcm_machine_owns_sla
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.icbm_latch import icbm_active
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist_zp import SpeedLimitAssistZP
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.models.helpers import get_active_bundle

LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
SCC_SOURCES = (LongitudinalPlanSource.sccVision, LongitudinalPlanSource.sccMap)
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]


class LongitudinalPlannerSP:
  # set by the host planner before the e2e candidate is built
  CP: structs.CarParams
  accel_boost: AccelBoost
  a_cruise: float
  v_desired_trajectory: np.ndarray
  a_desired_trajectory: np.ndarray

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, mpc):
    self.events_sp = EventsSP()
    self.e2e_set_speed = E2ESetSpeedController()
    self.lead_forecast = LeadForecast()
    self.lead_forecast.install(mpc)
    self.scc = make_smart_cruise_control(CP)
    self.CP = CP
    self.CP_SP = CP_SP
    self.resolver = SpeedLimitResolver(CP, CP_SP)
    # cars whose setpoint only the driver can move run the SLA machine here; everywhere
    # else it runs in card (the cruise arbiter, next to the buttons and the setpoint) and
    # gets mirrored (speed_limit.helpers.pcm_machine_owns_sla). On Mazda alpha long the owner
    # follows card's ICBM decision, which only moves between engagements.
    self.sla_machine = SpeedLimitAssistZP(CP, CP_SP) if pcm_machine_owns_sla(CP, icbm_active=False) else None
    self.sla_mirror = SpeedLimitAssistMirror(CP, CP_SP)
    self.sla = self._sla_owner(not CP_SP.pcmCruiseSpeed)
    self.generation = int(model_bundle.generation) if (model_bundle := get_active_bundle()) else None
    self.source = LongitudinalPlanSource.cruise
    self.e2e_alerts_helper = E2EAlertsHelper()

    self.output_v_target = 0.
    self.output_a_target = 0.
    self.seed_fault_logged = False
    self.a_cruise = 0.
    self.a_cruise_prev = 0.
    self.v_ego = 0.

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
    self.sla_mirror.session = sm['carStateSP'].zoompilot.cruiseSession
    has_speed_limit = self.resolver.speed_limit_valid or self.resolver.speed_limit_last_valid
    self.sla.update(long_enabled, long_override, v_ego, a_ego, v_cruise_cluster, self.resolver.speed_limit,
                    self.resolver.speed_limit_final_last, has_speed_limit, self.resolver.distance, self.events_sp)

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
    # the host's cruise candidate inputs, before it overwrites a_cruise this frame
    self.a_cruise_prev, self.v_ego = self.a_cruise, CS.vEgo
    return self.output_v_target, self.output_a_target

  def update_cruise_accel(self, a_cruise: float) -> float:
    """Upstream caps the cruise candidate at A_CRUISE_MIN, gentle for a set-speed drop, and SLA
    coasts down to a lower limit the same way. SCC brakes for a curve through the same candidate by
    lowering v_cruise, planned on its own budget: while it is the target, rebuild the candidate from
    it with that budget as the floor. Its a_target is not a plan to take (the map passes a_ego through)."""
    if self.source not in SCC_SOURCES:
      return a_cruise
    j = op_long_cruise_jerk(self.v_ego) * DT_MDL
    a_limiter = np.clip(self.output_v_target - self.v_ego, -OP_LONG_A_BUDGET, ACCEL_MAX)
    return min(a_cruise, float(np.clip(a_limiter, self.a_cruise_prev - j, self.a_cruise_prev + j)))

  def update_e2e_target(self, sm: messaging.SubMaster, a_model: float, should_stop_model: bool,
                        reset_state: bool) -> tuple[float, bool]:
    """The e2e candidate the host planner takes the minimum over. Without openpilot longitudinal it
    can never be the minimum (cruise, listed first, wins a tie) and never asks to stop."""
    is_e2e = self.CP.openpilotLongitudinalControl
    # lateral acceleration from the measured steering: controlsd's curvature from the learned steer
    # ratio, angle offset and roll
    steer_lat_accel = sm['controlsState'].curvature * sm['carState'].vEgo ** 2
    # chill mode's answer: the MPC candidate as the host takes it from this frame's solution (a lead,
    # or its synthetic one far ahead), and last frame's cruise candidate (the host builds this
    # frame's after the e2e candidate), capped at the approach pace
    a_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                action_t=self.CP.longitudinalActuatorDelay + DT_MDL)
    a_chill = min(float(a_mpc), self.a_cruise, approach_accel(self.v_ego, self.output_v_target,
                                                                sm['selfdriveState'].personality.raw))
    a_e2e = self.e2e_set_speed.update(sm, a_model, a_chill, is_e2e, reset_state, steer_lat_accel)
    # upstream's gas boost learns past the model from gas presses; the set-speed floor already lifts
    # it, so the boost only runs with the floor off
    if self.e2e_set_speed.enabled:
      self.accel_boost.total_boost = 0.
      self.accel_boost.boost_eligible = False
    if not is_e2e:
      return ACCEL_MAX, False
    return a_e2e, should_stop_model

  def update(self, sm: messaging.SubMaster) -> None:
    self.events_sp.clear()
    self.lead_forecast.update(sm)

    self.e2e_alerts_helper.update(sm, self.events_sp)

  def publish_longitudinal_plan_sp(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    plan_sp_send = messaging.new_message('longitudinalPlanSP')

    plan_sp_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlanSP = plan_sp_send.longitudinalPlanSP
    longitudinalPlanSP.longitudinalPlanSource = self.source
    longitudinalPlanSP.vTarget = float(self.output_v_target)
    longitudinalPlanSP.aTarget = float(self.output_a_target)
    longitudinalPlanSP.events = self.events_sp.to_msg()

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

    # zoompilot: experimental mode's Set Speed Assist, the model's lead forecast
    zoompilot = longitudinalPlanSP.zoompilot
    self.e2e_set_speed.fill(zoompilot.e2eSetSpeed)
    self.lead_forecast.fill(zoompilot.leadForecast)
    # zoompilot's vision planner only; 0 tells the ICBM servo there is no lookahead
    zoompilot.sccVAheadMin = float(getattr(self.scc.vision, 'v_ahead_min', 0.))

    pm.send('longitudinalPlanSP', plan_sp_send)
