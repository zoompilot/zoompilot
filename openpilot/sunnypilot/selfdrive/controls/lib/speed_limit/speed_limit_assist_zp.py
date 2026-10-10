"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

sunnypilot's Speed Limit Assist machine as zoompilot runs it. The planner builds it only on pcm
openpilot-long cars (speed_limit.helpers.pcm_machine_owns_sla); every other car's session runs
in card's cruise arbiter and is mirrored by speed_limit.assist_mirror, so the upstream non-pcm
machine and its button hooks never run here.
"""
from openpilot.cereal import custom
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.limits import A_PUB_MIN, get_planning_limits, publish_ramp
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.helpers import settle_conv
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import ACTIVE_STATES, V_CRUISE_UNSET, SpeedLimitAssist
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

EventNameSP = custom.OnroadEventSP.EventName
SpeedLimitAssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState


class SpeedLimitAssistZP(SpeedLimitAssist):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.limits = get_planning_limits(CP)
    self.prev_settle_conv = 0
    self._a_out = 0.

  def reset(self) -> None:
    """Fresh session state when the planner hands the session back to this machine (card's ICBM
    went passive), which happens only while disengaged."""
    self.state = SpeedLimitAssistState.disabled
    self.is_enabled = self.is_active = False
    self.long_enabled = self.long_enabled_prev = False
    self.long_engaged_timer = 0
    self.pre_active_timer = 0
    self.output_v_target = V_CRUISE_UNSET
    self.output_a_target = 0.
    self._a_out = 0.

  def get_a_target_from_control(self) -> float:
    # active states publish through the shared ramp (the plan aTarget seeds the MPC, so a
    # state change must never step it); idle states track a_ego, the ramp's starting point
    a_des = float(min(max(self.acceleration_solutions[self.state](), A_PUB_MIN), -A_PUB_MIN))
    if self.state in ACTIVE_STATES:
      self._a_out = publish_ramp(a_des, self._a_out, self.limits, self.v_ego)
    else:
      self._a_out = a_des
    return self._a_out

  def update_events(self, events_sp: EventsSP) -> None:
    if self.state == SpeedLimitAssistState.preActive:
      events_sp.add(EventNameSP.speedLimitPreActive)

    # pending fires no alert: announcing "auto adjusting" on every engage reads as SLA acting

    # announce only a move of the speed the car settles at, as the cruise arbiter does: a limit
    # above the set speed, a confirm the prompt's cap already reached, or a limit that drops out
    # and returns changes nothing
    settle = settle_conv(self.get_v_target_from_control(), self.v_cruise_cluster_conv, self.is_metric)
    if self.is_active and settle != self.prev_settle_conv:
      self.update_active_event(events_sp)
    self.prev_settle_conv = settle
