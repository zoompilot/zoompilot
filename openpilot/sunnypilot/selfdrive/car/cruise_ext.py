"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from openpilot.cereal import custom
from opendbc.car.structs import car
from opendbc.car import structs
from opendbc.car.interfaces import V_CRUISE_MAX
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.cruise_arbiter import CruiseArbiter
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import get_minimum_set_speed
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import ACTIVE_STATES as SESSION_ACTIVE_STATES, V_CRUISE_UNSET

ButtonType = car.CarState.ButtonEvent.Type
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
IcbmState = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
CruiseIntent = custom.CarStateZP.CruiseSession.CruiseIntent

CRUISE_BUTTON_TIMER = {ButtonType.decelCruise: 0, ButtonType.accelCruise: 0,
                       ButtonType.setCruise: 0, ButtonType.resumeCruise: 0,
                       ButtonType.cancel: 0, ButtonType.mainCruise: 0}

V_CRUISE_MIN = 8  # kph; selfdrive.car.cruise's V_CRUISE_MIN is not importable from here (cycle)

# Setpoint reconciliation for non-pcmCruiseSpeed (ICBM) cars: the stock ECU keeps the real
# set speed and steps it on wheel presses while openpilot integrates the same presses, so
# the two drift. Around a driver press the dash is the truth of the setpoint, adopted iff
# the plan source is cruise and ICBM is not mid-move. See docs/zoompilot/cruise-arbiter.md.
RECONCILE_SETTLE_TIME = 1.0  # s after the last press; absorbs the ECU's trailing long-press increment
RECONCILE_SETTLE_FRAMES = int(RECONCILE_SETTLE_TIME / DT_CTRL)
RECONCILE_BUTTONS = (ButtonType.accelCruise, ButtonType.decelCruise)
# the dash must have been at rest when the press started (at the setpoint, or at an active
# SLA session's target); a dash in transit matches neither and is never adopted
RECONCILE_AGREE_KPH = 2 * CV.MPH_TO_KPH


def update_manual_button_timers(CS: car.CarState, button_timers: dict[car.CarState.ButtonEvent.Type, int]) -> None:
  # increment timer for buttons still pressed
  for k in button_timers:
    if button_timers[k] > 0:
      button_timers[k] += 1

  for b in CS.buttonEvents:
    if b.type.raw in button_timers:
      # Start/end timer and store current state on change of button pressed
      button_timers[b.type.raw] = 1 if b.pressed else 0


class VCruiseHelperSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP) -> None:
    self.CP = CP
    self.CP_SP = CP_SP
    self.v_cruise_kph = V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_UNSET
    self.params = Params()
    self.v_cruise_min = 0
    self.enabled_prev = False

    self.custom_acc_enabled = self.params.get_bool("CustomAccIncrementsEnabled")
    self.short_increment = self.params.get("CustomAccShortPressIncrement", return_default=True)
    self.long_increment = self.params.get("CustomAccLongPressIncrement", return_default=True)

    self.enable_button_timers = dict(CRUISE_BUTTON_TIMER)

    # Setpoint reconciliation (non-pcmCruiseSpeed cars)
    self.reconcile_frames = 0
    self.reconcile_allowed = False
    self.reconcile_floor = False  # a - press dismissed a session: floor to the dash at settle

    # Plan/actuation regime, updated from longitudinalPlanSP + carControlSP each frame
    self.lp_source = LongitudinalPlanSource.cruise
    self.icbm_state = IcbmState.inactive

    # classifies every +/- press once and owns the SLA session on non-pcm cars; no-op elsewhere
    self.cruise_arbiter = CruiseArbiter(CP, CP_SP)
    self.cruise_arbiter.read_params(self.params)

  def read_custom_set_speed_params(self) -> None:
    self.custom_acc_enabled = self.params.get_bool("CustomAccIncrementsEnabled")
    self.short_increment = self.params.get("CustomAccShortPressIncrement", return_default=True)
    self.long_increment = self.params.get("CustomAccLongPressIncrement", return_default=True)
    # rides card's params thread, keeping param reads off the 100 Hz path
    self.cruise_arbiter.read_params(self.params)

  def update_v_cruise_delta(self, long_press: bool, v_cruise_delta: float) -> tuple[bool, float]:
    if not self.custom_acc_enabled:
      v_cruise_delta = v_cruise_delta * (5 if long_press else 1)
      return long_press, v_cruise_delta

    # Apply user-specified multipliers to the base increment
    short_increment = np.clip(self.short_increment, 1, 10)
    long_increment = np.clip(self.long_increment, 1, 10)

    actual_increment = long_increment if long_press else short_increment
    round_to_nearest = actual_increment in (5, 10)
    v_cruise_delta = v_cruise_delta * actual_increment

    return round_to_nearest, v_cruise_delta

  def get_minimum_set_speed(self, is_metric: bool) -> None:
    if self.CP_SP.pcmCruiseSpeed:
      self.v_cruise_min = V_CRUISE_MIN
      return

    self.v_cruise_min = get_minimum_set_speed(is_metric)

  def update_enabled_state(self, CS: car.CarState, enabled: bool) -> bool:
    # special enabled state for non pcmCruiseSpeed, unchanged for non pcmCruise
    if not self.CP_SP.pcmCruiseSpeed:
      update_manual_button_timers(CS, self.enable_button_timers)
      button_pressed = any(self.enable_button_timers[k] > 0 for k in self.enable_button_timers)

      if enabled and not self.enabled_prev:
        self.enabled_prev = not button_pressed
        enabled = False
      elif not enabled:
        self.enabled_prev = enabled

      enabled = enabled and self.enabled_prev

    # the arbiter runs on the enabled computed above, not the raw flag: on non-pcmCruiseSpeed
    # cars "enabled" is suppressed until the engaging button releases, and the session must
    # not start mid-engage-hold
    self.update_cruise_arbiter(CS, enabled)
    return enabled

  def update_speed_limit_assist_v_cruise_non_pcm(self) -> None:
    # upstream's hook after the non-pcm increments: the cruise arbiter adopts the limit into
    # v_cruise itself (update_cruise_arbiter), so nothing is left to do here
    pass

  def reconcile_setpoint_with_dash(self, CS: car.CarState) -> None:
    if self.CP_SP.pcmCruiseSpeed or not self.CP.pcmCruise:
      return

    if not CS.cruiseState.available or self.v_cruise_kph in (V_CRUISE_UNSET, -1):
      self.reconcile_frames = 0
      self.reconcile_floor = False
      return

    pressed = any(self.enable_button_timers[b] > 0 for b in RECONCILE_BUTTONS)
    if not pressed and self.reconcile_frames <= 0:
      return

    dash_kph = CS.cruiseState.speed * CV.MS_TO_KPH
    if pressed:
      if any(self.enable_button_timers[b] == 1 for b in RECONCILE_BUTTONS):
        # verdict per press start, before the press's own ECU effect lands; the arbiter's
        # pre-frame snapshot is the session state the press was aimed at
        agree_setpoint = abs(dash_kph - self.v_cruise_kph) <= RECONCILE_AGREE_KPH
        sla_session = self.cruise_arbiter.state_prev_frame in SESSION_ACTIVE_STATES
        agree_sla = sla_session and abs(dash_kph - self.cruise_arbiter.slf_kph) <= RECONCILE_AGREE_KPH
        self.reconcile_allowed = agree_setpoint or agree_sla
        if self.cruise_arbiter.last_intent == CruiseIntent.dismiss and self.enable_button_timers[ButtonType.decelCruise] == 1:
          self.reconcile_floor = True
      self.reconcile_frames = RECONCILE_SETTLE_FRAMES
    else:
      self.reconcile_frames -= 1

    if self.reconcile_allowed:
      # never adopt while a limiter drives the plan, ICBM is stepping the dash, or a
      # confirm prompt is pending (the frozen dash is not the driver's answer)
      regime_ok = (self.lp_source == LongitudinalPlanSource.cruise
                   and self.icbm_state not in (IcbmState.increasing, IcbmState.decreasing)
                   and not self.cruise_arbiter.prompting)
      if regime_ok and dash_kph > 1:
        self.v_cruise_kph = float(np.clip(round(dash_kph, 1), self.v_cruise_min, V_CRUISE_MAX))
        self.v_cruise_cluster_kph = self.v_cruise_kph

    if self.reconcile_floor and not pressed and self.reconcile_frames <= 0:
      # After a decrement dismisses a session, only lower the baseline to the settled dash
      # value so the servo cannot undo the driver's request.
      self.reconcile_floor = False
      if not self.cruise_arbiter.prompting and dash_kph > 1:
        self.v_cruise_kph = min(self.v_cruise_kph, float(np.clip(round(dash_kph, 1), self.v_cruise_min, V_CRUISE_MAX)))
        self.v_cruise_cluster_kph = self.v_cruise_kph

  def update_speed_limit_assist(self, is_metric, LP_SP: custom.LongitudinalPlanSP) -> None:
    self.cruise_arbiter.is_metric = is_metric
    # a plain copy of the resolver fields, so repeating it on an unchanged plan is a no-op
    self.cruise_arbiter.update_limit(LP_SP)

  def update_plan_regime(self, LP_SP: custom.LongitudinalPlanSP, CC_SP: custom.CarControlSP) -> None:
    # the plan/actuation regime the reconciler gates on; card feeds it right before the
    # reconciler runs (CardExt.update_v_cruise_post)
    self.lp_source = LP_SP.longitudinalPlanSource
    self.icbm_state = CC_SP.intelligentCruiseButtonManagement.state

  def update_speed_limit_assist_pre_active_confirmed(self, button_type) -> bool:
    # upstream's hook in the increment path: a confirm- or dismiss-owned press carries
    # session semantics, and the ECU's own step comes back via dash reconciliation, so
    # incrementing here would count it twice
    return self.cruise_arbiter.press_owned(button_type)

  def update_cruise_arbiter(self, CS: car.CarState, enabled: bool) -> None:
    if not self.cruise_arbiter.applicable:
      return

    v_cruise_kph = self.cruise_arbiter.step(CS, enabled, self.v_cruise_kph, self.v_cruise_cluster_kph)
    if self.cruise_arbiter.adopted_this_frame:
      # the arbiter wrote the setpoint; on ICBM cars the ECU's own +1 from the confirm
      # press must not be re-adopted over it
      self.v_cruise_kph = v_cruise_kph
      self.v_cruise_cluster_kph = v_cruise_kph
      self.reconcile_frames = 0
      self.reconcile_allowed = False
      self.reconcile_floor = False
