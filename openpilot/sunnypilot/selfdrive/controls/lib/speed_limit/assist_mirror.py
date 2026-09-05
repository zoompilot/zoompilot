"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Plannerd-side mirror of the card cruise arbiter's SLA session (non-pcm cars).

The session machine runs in card at 100 Hz, next to the buttons and the setpoint.
Plannerd needs three things from it: the plan cap for min() source selection, the
assist state for the UI wire, and the alert events. The mirror reads
carStateSP.zoompilot.cruiseSession and reproduces the surface SpeedLimitAssist used to provide
here, so longitudinalPlanSP consumers are unchanged.

Events: speedLimitPreActive is level-driven (the prompt alert persists for the whole
window); speedLimitActive fires on announce-counter deltas, which card bumps at 100 Hz
and never un-bumps, so a 20 Hz reader cannot miss one.
"""
from openpilot.cereal import custom
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.limits import get_planning_limits, publish_ramp
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.speed_profile import required_decel
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import ACTIVE_STATES, ENABLED_STATES, V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

EventNameSP = custom.OnroadEventSP.EventName
SessionState = custom.LongitudinalPlanSP.SpeedLimit.AssistState

# past the sign (distance 0) the decel degrades to a pull over the control horizon,
# matching SpeedLimitAssist's active-state form
_T_ACTIVE = float(ModelConstants.T_IDXS[CONTROL_N])


class SpeedLimitAssistMirror:
  pcm_op_long = False

  def __init__(self, CP, CP_SP):
    self.limits = get_planning_limits(CP)
    self.state = SessionState.disabled
    self.output_v_target = V_CRUISE_UNSET
    self.output_a_target = 0.
    self._a_out = 0.
    self._announce_seen: int | None = None  # sync on first update (plannerd restarts)

  @property
  def is_enabled(self) -> bool:
    return self.state in ENABLED_STATES

  @property
  def is_active(self) -> bool:
    return self.state in ACTIVE_STATES

  def update_buttons(self, release_toggle: int) -> None:
    # upstream's plannerd hook: presses are classified by the cruise arbiter in card, whose
    # session this class mirrors, so there is nothing to do here
    pass

  def update(self, session, v_ego: float, distance: float, a_ego: float, events_sp: EventsSP) -> None:
    self.state = session.state
    # the arbiter publishes vCap as a target, a frozen hold, or V_CRUISE_UNSET, never 0; a 0
    # is capnp's default from a not-yet-received carStateSP and would win the plan min()
    v_cap = float(session.vCap)
    self.output_v_target = v_cap if v_cap > 0.0 else V_CRUISE_UNSET

    # the decel actually required to arrive at the cap (it keys ICBM's overshoot gap on
    # stock ACC); computed here because the resolver's distance lives in plannerd
    if self.is_active and 0.0 < v_cap < v_ego:
      d_eff = max(distance, v_ego * _T_ACTIVE)
      a_des = -required_decel(v_ego, [v_cap], [d_eff])
      self._a_out = publish_ramp(a_des, self._a_out, self.limits, v_ego)
    else:
      self._a_out = a_ego
    self.output_a_target = self._a_out

    if self.state == SessionState.preActive:
      events_sp.add(EventNameSP.speedLimitPreActive)

    announce = int(session.announceCounter)
    if self._announce_seen is not None and announce != self._announce_seen:
      events_sp.add(EventNameSP.speedLimitActive)
    self._announce_seen = announce
