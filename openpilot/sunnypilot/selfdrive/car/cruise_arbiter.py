"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Cruise-button intent and speed-limit session arbitration for non-pcm cars.

The arbiter classifies each press once from the pre-frame session state and publishes
the result for the setpoint, planner, and ICBM paths. Confirmation prompts freeze the
session cap and block synthesized button output. See docs/zoompilot/cruise-arbiter.md.
"""
from dataclasses import dataclass

import numpy as np

from openpilot.cereal import custom
from opendbc.car import structs
from opendbc.car.interfaces import V_CRUISE_MAX
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import get_minimum_set_speed
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import ACTIVE_STATES, V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.helpers import compare_cluster_target, confirm_needed_for_change

ButtonType = car.CarState.ButtonEvent.Type
SessionState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
CruiseIntent = custom.CarStateZP.CruiseSession.CruiseIntent

# Timers use 100 Hz control frames.
DISABLED_GUARD_PERIOD = 0.5   # s after engagement before the session may form
PRE_ACTIVE_GUARD_PERIOD = 5.  # s a confirm prompt stays open
# Resolve a held prompt before cruise.py emits its first long-press repeat tick.
LONG_PRESS_FRAMES = 50 - 1

PLUS_BUTTONS = (ButtonType.accelCruise, ButtonType.resumeCruise)
MINUS_BUTTONS = (ButtonType.decelCruise, ButtonType.setCruise)

# Press classes are assigned at the press edge from the pre-frame state.
_PRESS_NORMAL = 0   # plain increment/decrement press
_PRESS_DISMISS = 1  # started while the session was active: owned, ends the session
_PRESS_PROMPT = 2   # started while a confirm prompt was open: resolves at release/tick


@dataclass
class _Press:
  cls: int
  frames: int = 0
  resolved: bool = False   # a resolved prompt press remains owned
  released: bool = False   # retain through the release frame for press_owned


class CruiseArbiter:
  def __init__(self, CP, CP_SP):
    # Only pcmCruise platforms with openpilot longitudinal keep the planner-side machine.
    self.applicable = not (CP.openpilotLongitudinalControl and CP.pcmCruise)
    # ICBM platforms adopt the ECU setpoint; non-pcm openpilot longitudinal writes it here.
    self.op_owns_setpoint = not CP.pcmCruise

    self.state = SessionState.disabled
    self.state_prev_frame = SessionState.disabled  # snapshot from before this frame's step
    self.v_cap = V_CRUISE_UNSET  # m/s; session target while active, frozen hold while prompting
    self.last_intent = CruiseIntent.none
    self.announce_counter = 0

    # The card params thread refreshes these outside the real-time path.
    self.enabled = False   # SpeedLimitMode == assist
    self.is_metric = False

    # longitudinalPlanSP updates these at the planning rate.
    self._speed_limit = 0.
    self._speed_limit_prev = 0.
    self._slf = 0.  # speedLimitFinalLast, m/s
    self._has_limit = False

    self.long_enabled_prev = False
    self.long_engaged_timer = 0
    self.pre_active_timer = 0
    self._driver_dismissed = False
    self._cluster_conv = 0
    self._cluster_conv_prev = 0

    # Key by raw integers because capnp enums do not hash-match cruise.py's integers.
    self._press: dict[int, _Press] = {}
    # The cruise helper uses this to close reconciliation before it runs.
    self.adopted_this_frame = False

  def read_params(self, params):
    if not self.applicable:
      return
    self.enabled = params.get("SpeedLimitMode", return_default=True) == Mode.assist
    self.is_metric = params.get_bool("IsMetric")

  def update_limit(self, LP_SP):
    if not self.applicable:
      return
    resolver = LP_SP.speedLimit.resolver
    self._speed_limit = float(resolver.speedLimit)
    self._slf = float(resolver.speedLimitFinalLast)
    self._has_limit = bool(resolver.speedLimitValid or resolver.speedLimitLastValid)

  @property
  def _conv(self) -> float:
    return CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH

  @property
  def prompting(self) -> bool:
    return self.state == SessionState.preActive

  @property
  def session_active(self) -> bool:
    return self.state in ACTIVE_STATES

  def _target_conv(self) -> int:
    return round(self._slf * self._conv)

  @property
  def target_kph(self) -> float:
    # Convert the display-rounded limit back to the setpoint's kph domain.
    conv = 1. if self.is_metric else CV.KPH_TO_MPH
    return round(self._slf * CV.MS_TO_KPH * conv) / conv

  @property
  def slf_kph(self) -> float:
    return self._slf * CV.MS_TO_KPH

  @property
  def _limit_changed(self) -> bool:
    return self._has_limit and self._speed_limit != self._speed_limit_prev

  def _set_state(self, state, announce=False):
    self.state = state
    if announce:
      self.announce_counter += 1

  def _enter_prompt(self):
    # Preserve an active session's cap while prompting. From idle, leave the cap unset to
    # avoid round-trip error that would incorrectly classify cruise as a limiter.
    was_session = self.state in ACTIVE_STATES or self.v_cap < V_CRUISE_UNSET
    hold = self.v_cap if was_session else V_CRUISE_UNSET
    self._set_state(SessionState.preActive)
    self.v_cap = float(hold)
    self.pre_active_timer = int(PRE_ACTIVE_GUARD_PERIOD / DT_CTRL)

  def _activate(self, from_prompt: bool):
    # Announce confirmations and upcoming setpoint movement, not an existing match.
    announce = from_prompt or self._target_conv() != self._cluster_conv
    self._set_state(SessionState.active, announce=announce)

  def _classify_presses(self, CS, v_cruise_kph: float) -> float:
    """Consume button edges; decide intents from the pre-frame session snapshot.

    Returns v_cruise_kph, possibly raised by an upward confirm adoption."""
    self.adopted_this_frame = False
    if self._press:
      # Release ownership lasts through the release frame.
      self._press = {btn: p for btn, p in self._press.items() if not p.released}

    for b in CS.buttonEvents:
      if b.type not in PLUS_BUTTONS and b.type not in MINUS_BUTTONS:
        continue
      btn = b.type.raw

      if b.pressed:
        if self.state_prev_frame in ACTIVE_STATES:
          # A press dismisses an active session. Re-anchor openpilot-owned setpoints to the
          # active cap before applying the press; ICBM platforms adopt the ECU's result.
          if self.op_owns_setpoint and self.v_cap < V_CRUISE_UNSET:
            anchor = min(v_cruise_kph, self.target_kph)
            v_cruise_kph = float(np.clip(round(anchor, 1), get_minimum_set_speed(self.is_metric), V_CRUISE_MAX))
            self.adopted_this_frame = True
            self._press[btn] = _Press(_PRESS_NORMAL)
          else:
            self._press[btn] = _Press(_PRESS_DISMISS)
          self._set_state(SessionState.inactive)
          self._driver_dismissed = True
          self.last_intent = CruiseIntent.dismiss
        elif self.state_prev_frame == SessionState.preActive:
          self._press[btn] = _Press(_PRESS_PROMPT)
        else:
          self._press[btn] = _Press(_PRESS_NORMAL)

      else:  # release
        press = self._press.get(btn)
        if press is None:
          continue
        if press.cls == _PRESS_PROMPT and not press.resolved:
          v_cruise_kph = self._resolve_prompt_press(btn, press, v_cruise_kph)
        if press.cls == _PRESS_NORMAL and self.last_intent == CruiseIntent.none:
          self.last_intent = CruiseIntent.increment if btn in PLUS_BUTTONS else CruiseIntent.decrement
        press.released = True

    # Resolve a held prompt before the first repeated setpoint step.
    for btn, press in self._press.items():
      if press.released:
        continue
      press.frames += 1
      if press.cls == _PRESS_PROMPT and not press.resolved and press.frames >= LONG_PRESS_FRAMES:
        v_cruise_kph = self._resolve_prompt_press(btn, press, v_cruise_kph)

    return v_cruise_kph

  def _resolve_prompt_press(self, button: int, press: _Press, v_cruise_kph: float) -> float:
    if self.state != SessionState.preActive:
      # Treat an in-flight press as normal if another event resolved the prompt.
      press.cls = _PRESS_NORMAL
      return v_cruise_kph

    req_plus, req_minus = compare_cluster_target(self._cluster_conv / self._conv, self._slf, self.is_metric)
    is_plus = button in PLUS_BUTTONS

    if (req_plus and is_plus) or (req_minus and not is_plus):
      # An upward confirmation may raise the baseline; the active cap handles lower limits.
      press.resolved = True
      self.last_intent = CruiseIntent.confirm
      if is_plus and self.target_kph > v_cruise_kph:
        v_cruise_kph = float(np.clip(round(self.target_kph, 1), get_minimum_set_speed(self.is_metric), V_CRUISE_MAX))
        self.adopted_this_frame = True
      self._activate(from_prompt=True)
    else:
      # A press in the opposite direction declines and resumes normal setpoint handling.
      press.cls = _PRESS_NORMAL
      self.last_intent = CruiseIntent.decline
      self._set_state(SessionState.inactive)

    return v_cruise_kph

  def press_owned(self, button_type: int) -> bool:
    """True when this press must not increment v_cruise (confirm- or dismiss-owned).
    Takes the raw enumerant int (as cruise.py's button paths carry); valid for release
    events and long-press repeat ticks in the same frame."""
    press = self._press.get(button_type)
    if press is None:
      return False
    if press.cls == _PRESS_DISMISS:
      return True
    return press.cls == _PRESS_PROMPT and press.resolved

  def step(self, CS, long_enabled: bool, v_cruise_kph: float, v_cruise_cluster_kph: float) -> float:
    """Classify presses, advance the session, and publish its cap in a fixed order."""
    if not self.applicable:
      return v_cruise_kph

    self.state_prev_frame = self.state
    conv = CV.KPH_TO_MS * self._conv
    self._cluster_conv_prev = self._cluster_conv
    self._cluster_conv = round(v_cruise_cluster_kph * conv) if v_cruise_cluster_kph not in (V_CRUISE_UNSET, -1) else 0
    self.last_intent = CruiseIntent.none

    v_cruise_kph = self._classify_presses(CS, v_cruise_kph)

    self.long_engaged_timer = max(0, self.long_engaged_timer - 1)
    self.pre_active_timer = max(0, self.pre_active_timer - 1)

    if self.state != SessionState.disabled:
      if not long_enabled or not self.enabled:
        self._set_state(SessionState.disabled)
        self._driver_dismissed = False

      elif self.state in ACTIVE_STATES:
        if self._limit_changed and confirm_needed_for_change(self._cluster_conv, self._target_conv(), self.is_metric):
          self._enter_prompt()
        elif self._limit_changed and self._target_conv() != self._cluster_conv:
          # Auto-apply target changes that do not require confirmation.
          self.announce_counter += 1

      elif self.state == SessionState.preActive:
        if self._target_conv() == self._cluster_conv:
          self._activate(from_prompt=True)  # dialing to the target confirms it
        elif self.pre_active_timer <= 0:
          self._set_state(SessionState.inactive)

      elif self.state == SessionState.inactive:
        if self._limit_changed:
          self._driver_dismissed = False
          self._enter_prompt()
        elif not self._driver_dismissed and self._has_limit and self._target_conv() == self._cluster_conv \
             and not self._press:
          # Wait for release so a driver can continue through the target.
          self._activate(from_prompt=False)

    else:
      if long_enabled and self.enabled:
        if not self.long_enabled_prev or self._cluster_conv != self._cluster_conv_prev:
          self.long_engaged_timer = int(DISABLED_GUARD_PERIOD / DT_CTRL)
        elif self.long_engaged_timer <= 0:
          if self._has_limit and self._target_conv() == self._cluster_conv:
            self._activate(from_prompt=False)
          elif self._has_limit:
            self._enter_prompt()
          else:
            self._set_state(SessionState.inactive)

    # Prompts retain the previous cap; active sessions publish the current target.
    if self.state in ACTIVE_STATES:
      self.v_cap = float(self._slf) if self._has_limit else V_CRUISE_UNSET
    elif self.state != SessionState.preActive:  # a prompt keeps its frozen hold
      self.v_cap = V_CRUISE_UNSET

    self._speed_limit_prev = self._speed_limit
    self.long_enabled_prev = long_enabled
    return v_cruise_kph

  def fill_msg(self, cs_sp) -> None:
    if not self.applicable:
      return
    session = cs_sp.zoompilot.cruiseSession
    session.state = self.state
    session.vCap = float(self.v_cap)
    session.lastIntent = self.last_intent
    session.announceCounter = self.announce_counter

  def gate_send_button(self, CC_SP) -> None:
    """Block synthesized buttons at prompt onset before CI.apply."""
    if self.applicable and self.prompting:
      CC_SP.intelligentCruiseButtonManagement.sendButton = structs.IntelligentCruiseButtonManagement.SendButtonState.none
