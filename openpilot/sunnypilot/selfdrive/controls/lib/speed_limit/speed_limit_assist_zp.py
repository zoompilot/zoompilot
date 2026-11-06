"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

sunnypilot's Speed Limit Assist machine as zoompilot runs it. The planner builds it only on pcm
openpilot-long cars (speed_limit.helpers.pcm_machine_owns_sla); every other car's session runs
in card's cruise arbiter and is mirrored by speed_limit.assist_mirror, so the upstream non-pcm
machine and its button hooks never run here.

On these cars nothing but the driver moves the cluster setpoint, so upstream makes the driver
confirm by setting a required max (70/80 mph, 120/130 km/h) and drops the session on any other
set speed. Here the set speed is only the ceiling: with the setting on, the limit caps the plan
whatever the setpoint, and the driver adjusts it with the gas instead. Releasing the gas above
the limit holds the speed reached, up to the set speed, until the next sign.
"""
from openpilot.cereal import custom
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.limits import A_PUB_MIN, get_planning_limits, publish_ramp
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.helpers import settle_conv
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import ACTIVE_STATES, DISABLED_GUARD_PERIOD, ENABLED_STATES, \
  V_CRUISE_UNSET, SpeedLimitAssist
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

EventNameSP = custom.OnroadEventSP.EventName
SpeedLimitAssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState


class SpeedLimitAssistZP(SpeedLimitAssist):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.limits = get_planning_limits(CP)
    self.prev_settle_conv = 0
    self._a_out = 0.
    self._override_prev = False
    self.held_v: float | None = None  # m/s, the speed the gas held over the limit
    self._held_for_conv = 0  # display-unit limit the hold was made against
    self._held_now = False

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
    self._override_prev = False
    self.held_v = None

  def _update_hold(self, long_enabled: bool, long_override: bool, v_ego: float, v_cruise_cluster: float,
                   speed_limit_final_last: float, has_speed_limit: bool) -> None:
    conv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH
    limit_conv = round(speed_limit_final_last * conv) if has_speed_limit else 0
    released = self._override_prev and not long_override
    self._override_prev = long_override
    self._held_now = False

    # a new sign, lower or higher, ends the hold; a limit that drops out and returns does not
    if not (long_enabled and self.enabled) or (limit_conv and limit_conv != self._held_for_conv):
      self.held_v = None

    if released and limit_conv and self.state in ACTIVE_STATES:
      target_conv = round(self.held_v * conv) if self.held_v is not None else limit_conv
      v_conv = min(round(v_ego * conv), round(v_cruise_cluster * conv))
      if v_conv > target_conv:
        self.held_v = v_conv / conv
        self._held_for_conv = limit_conv
        self._held_now = True

  def update(self, long_enabled: bool, long_override: bool, v_ego: float, a_ego: float, v_cruise_cluster: float, speed_limit: float,
             speed_limit_final_last: float, has_speed_limit: bool, distance: float, events_sp: EventsSP) -> None:
    self._update_hold(long_enabled, long_override, v_ego, v_cruise_cluster, speed_limit_final_last, has_speed_limit)
    if self.held_v is not None and has_speed_limit:
      # the held speed stands in for the limit, with no sign ahead to brake toward
      speed_limit_final_last, distance = self.held_v, 0.
    super().update(long_enabled, long_override, v_ego, a_ego, v_cruise_cluster, speed_limit,
                   speed_limit_final_last, has_speed_limit, distance, events_sp)

  def update_state_machine_pcm_op_long(self):
    self.long_engaged_timer = max(0, self.long_engaged_timer - 1)

    if not self.long_enabled or not self.enabled:
      self.state = SpeedLimitAssistState.disabled
    elif self.state == SpeedLimitAssistState.disabled:
      # let the cluster settle after engaging before the limit caps the plan
      if not self.long_enabled_prev:
        self.long_engaged_timer = int(DISABLED_GUARD_PERIOD / DT_MDL)
      elif self.long_engaged_timer <= 0:
        self._update_confirmed_state()
    else:
      self._update_confirmed_state()

    return self.state in ENABLED_STATES, self.state in ACTIVE_STATES

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
    # no prompt and no pending alert: the session needs no confirming, and announcing
    # "auto adjusting" on every engage reads as SLA acting

    # announce only a move of the speed the car settles at, as the cruise arbiter does: a limit
    # above the set speed, a limit that drops out and returns, or a speed the driver held with
    # the gas changes nothing worth a chime
    settle = settle_conv(self.get_v_target_from_control(), self.v_cruise_cluster_conv, self.is_metric)
    if self.is_active and settle != self.prev_settle_conv and not self._held_now:
      self.update_active_event(events_sp)
    self.prev_settle_conv = settle
