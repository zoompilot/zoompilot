"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

zoompilot's alerts: its own events (OnroadEventZP, EVENTS_ZP, raised on EventsSP.zp), and its
text for some of sunnypilot's (EVENTS_SP_ZP, which events.py merges over EVENTS_SP at import).
"""
import bisect

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log, custom
from opendbc.car.structs import car
from opendbc.sunnypilot.car.stock_ecu import StockEcuState
from openpilot.common.constants import CV
from openpilot.common.hardware import HARDWARE
from openpilot.selfdrive.selfdrived.events import get_display_speed
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import EventsBase, Priority, ET, Alert, NoEntryAlert, \
  EngagementAlert, NormalPermanentAlert, AlertCallbackType
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import PCM_LONG_REQUIRED_MAX_SET_SPEED, CONFIRM_SPEED_THRESHOLD
from openpilot.sunnypilot.selfdrive.selfdrived.selfdrived_zp import SelfdrivedZP

AlertSize = log.SelfdriveState.AlertSize
AlertStatus = log.SelfdriveState.AlertStatus
VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert
AudibleAlertSP = custom.SelfdriveStateSP.AudibleAlert
EventNameSP = custom.OnroadEventSP.EventName
EventNameZP = custom.OnroadEventZP.EventName
AssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState

IS_MICI = HARDWARE.get_device_type() == 'mici'

EVENT_NAME_ZP = {v: k for k, v in EventNameZP.schema.enumerants.items()}


# What the driver has to do, per stock ECU state that does not engage normally, in the shape of
# upstream's no-entry alerts ("Gear not D"). Main off has no entry: the cluster's MRCC
# indicator is the car's own.
STOCK_ECU_ALERT_TEXT = {
  StockEcuState.STARTING: ("Longitudinal Initializing", "Remain stopped"),
  StockEcuState.PARK_TO_TAKE_OVER: ("Stop to Engage Longitudinal", "Longitudinal enables at next stop"),
  StockEcuState.STOCK_CRUISE_ON: ("Turn Off Stock Cruise", "Longitudinal waits for it"),
  StockEcuState.RESTORING: ("Restarting Stock Cruise", "Resumes in a few seconds"),
  StockEcuState.FAILED: ("Longitudinal Failed", "Restart the car and retry"),
}


def road_edge_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  # mici renders text1 above 16 chars at the smallest font and clips; silent mode keeps the
  # alert and drops its chime
  audible = AudibleAlert.none if SelfdrivedZP.road_edge_silent else AudibleAlert.prompt
  return Alert("Road Edge Ahead" if IS_MICI else "Lane Change Unavailable: Road Edge", "",
               AlertStatus.userPrompt, AlertSize.small,
               Priority.LOW, VisualAlert.none, audible, 0.1)


def speed_assist_switched_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  state = "On" if SelfdrivedZP.switched_to else "Off"
  return NormalPermanentAlert(f"Speed Assist {state}", duration=1.5)


def big_model_ready_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  # an accelerator's comes a second after its swap, when the driver can engage;
  # its offer to switch is the one titled "Big Model Ready"
  accelerator = sm['modelDataV2SP'].zoompilot.acceleratorState != custom.ModelDataV2ZP.AcceleratorState.none
  return Alert("Big Model Active" if accelerator else "Big Model Ready", "",
               AlertStatus.normal, AlertSize.small,
               Priority.LOW, VisualAlert.none, AudibleAlert.prompt, 2.)


def stock_ecu_not_ready_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  state = str(sm['carStateSP'].zoompilot.stockEcu)
  if state == StockEcuState.STARTING and not CS.standstill:
    state = StockEcuState.PARK_TO_TAKE_OVER  # the takeover needs a stop either way
  title, line = STOCK_ECU_ALERT_TEXT.get(state, STOCK_ECU_ALERT_TEXT[StockEcuState.STARTING])
  return NoEntryAlert(line, alert_text_1=title)


def _set_speed_ms(CS: car.CarState, sm: messaging.SubMaster) -> float:
  v_cruise_cluster = CS.vCruiseCluster
  set_speed = sm['controlsState'].deprecated.vCruise if v_cruise_cluster == 0.0 else v_cruise_cluster
  # vCruise/vCruiseCluster are kph; the resolver speeds are m/s
  return set_speed * CV.KPH_TO_MS


def speed_limit_adjust_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  speed = sm['longitudinalPlanSP'].speedLimit.assist.vTarget  # the cap that moved, in this event's message
  # the plan never runs above the set speed, so a limit over it settles there
  set_speed = _set_speed_ms(CS, sm)
  if set_speed > 0:
    speed = min(speed, set_speed)
  # mici wraps the unit off its number; break before the speed instead
  sep = "\n" if IS_MICI else " "
  return Alert(
    f'Adjusting to{sep}{get_display_speed(speed, metric)}',
    "",
    AlertStatus.normal, AlertSize.small,
    Priority.LOW, VisualAlert.none, AudibleAlertSP.promptSingleHigh, 5.)


def speed_limit_pre_active_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  speed_conv = CV.MS_TO_KPH if metric else CV.MS_TO_MPH
  set_speed_conv = round(_set_speed_ms(CS, sm) * speed_conv)

  speed_limit_final_last = sm['longitudinalPlanSP'].speedLimit.resolver.speedLimitFinalLast
  speed_limit_final_last_conv = round(speed_limit_final_last * speed_conv)
  alert_1_str = ""
  alert_size = AlertSize.small

  # the arbiter publishes its session only where it owns SLA (never disabled while long is
  # enabled past its guard); the planner machine never publishes it. One hop old, so key on
  # the session existing rather than on its exact state.
  arbiter_owns = sm['carStateSP'].zoompilot.cruiseSession.state != AssistState.disabled
  if CP.openpilotLongitudinalControl and CP.pcmCruise and not arbiter_owns:
    # PCM long: the driver moves the cluster to the required max
    cst_low, cst_high = PCM_LONG_REQUIRED_MAX_SET_SPEED[metric]
    pcm_long_required_max = cst_low if speed_limit_final_last_conv < CONFIRM_SPEED_THRESHOLD[metric] else cst_high
    pcm_long_required_max_set_speed_conv = round(pcm_long_required_max * speed_conv)
    speed_unit = "km/h" if metric else "mph"

    alert_1_str = f"Speed Limit Assist: set to {pcm_long_required_max_set_speed_conv} {speed_unit} to engage"
  else:
    if IS_MICI:
      # the target is the limit plus any offset; the break keeps the unit with its number
      if set_speed_conv < speed_limit_final_last_conv:
        alert_1_str = f"Press + for\n{get_display_speed(speed_limit_final_last, metric)}"
      elif set_speed_conv > speed_limit_final_last_conv:
        alert_1_str = f"Press - for\n{get_display_speed(speed_limit_final_last, metric)}"
    else:
      alert_size = AlertSize.none

  return Alert(
    alert_1_str,
    "",
    AlertStatus.normal, alert_size,
    Priority.LOW, VisualAlert.none, AudibleAlertSP.promptSingleLow, .1)


# zoompilot's text for sunnypilot's events
EVENTS_SP_ZP: dict[int, dict[str, Alert | AlertCallbackType]] = {
  # the speed limit alerts name the speed the car settles at
  EventNameSP.speedLimitActive: {
    ET.WARNING: speed_limit_adjust_alert,
  },

  EventNameSP.speedLimitChanged: {
    ET.WARNING: speed_limit_adjust_alert,
  },

  EventNameSP.speedLimitPreActive: {
    ET.WARNING: speed_limit_pre_active_alert,
  },

  EventNameSP.laneChangeRoadEdge: {
    ET.WARNING: road_edge_alert,
  },

  EventNameSP.bigModelReady: {
    ET.PERMANENT: big_model_ready_alert,
  },
}

EVENTS_ZP: dict[int, dict[str, Alert | AlertCallbackType]] = {
  EventNameZP.stockLkasOff: {
    # Mazda: invalidLkasSetting is swapped for this when MADS is on (CarSpecificEventsSP); MADS
    # holds lateral paused on it (mads.py). mici gets a standing alert, tizi only its border.
    ET.NO_ENTRY: Alert(
      "Lateral Disabled",
      "LKAS is off",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.refuse, 3.),
    **({ET.PERMANENT: NormalPermanentAlert("Lateral Disabled", "LKAS is off", priority=Priority.LOW)} if IS_MICI else {}),
  },

  # LKA back on with lateral resuming, the EPS not delivering yet: still disabled to the driver.
  EventNameZP.stockLkasArming: {
    ET.PERMANENT: NormalPermanentAlert("Lateral Disabled", "Waiting for steering", priority=Priority.LOW),
  } if IS_MICI else {},

  # Sound-only mirrors of the longitudinal selfdrive transitions while a declared MADS
  # button owns lateral. PERMANENT carries no state-machine meaning, so the chime cannot
  # enable or disable anything.
  EventNameZP.longitudinalEnableChime: {
    ET.PERMANENT: EngagementAlert(AudibleAlert.engage),
  },

  EventNameZP.longitudinalDisableChime: {
    ET.PERMANENT: EngagementAlert(AudibleAlert.disengage),
  },

  # pause-on-brake engagement with the brake already down: MADS enters paused and resumes on
  # release, the same moment the panda arms its pending request
  EventNameZP.silentPedalPressed: {
    ET.NO_ENTRY: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, 0.),
  },

  # The panda is rejecting our steering while MADS thinks it is steering: the wheel is
  # unsteered from the first rejected frame, not from the disable 2 s later (Mazda routes
  # 00000116/117: 2 s of rejected 0x243 with the camera relay-blocked latched the EPS fault)
  EventNameZP.controlsMismatchLateralWarning: {
    ET.WARNING: Alert(
      "Take Control",
      "Steering Blocked by Panda Safety",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.steerRequired, AudibleAlert.prompt, .5),
  },

  # The stock ECU alerts are PERMANENT, not the *AlertOnly WARNING convention: WARNING
  # shows only while cruise or MADS lateral is active, and these must reach a driver whose
  # lateral is off or paused (brake held at the stop where the takeover happens; route
  # 0000021b showed nothing for three minutes). Each press is one frame; the alert's own
  # duration keeps it up.
  EventNameZP.stockEcuNotReady: {
    ET.PERMANENT: stock_ecu_not_ready_alert,
  },

  # Stopped with the takeover still starting: unprompted, re-raised every frame it holds. MID so
  # the startup and big model banners (LOWER/LOW) cannot cover it while the driver waits.
  EventNameZP.stockEcuInitializing: {
    ET.PERMANENT: NormalPermanentAlert(*STOCK_ECU_ALERT_TEXT[StockEcuState.STARTING], priority=Priority.MID),
  },

  # the takeover landed: the car's own emergency braking went with the radar
  EventNameZP.stockEcuReady: {
    ET.PERMANENT: NormalPermanentAlert("Longitudinal Ready", "Stock AEB now off", duration=2.),
  },

  # The last session's hand-back failed and its stop went ahead anyway: the radar is left to
  # recover on its own, which blocks stock cruise until the next ignition. Held the whole session.
  EventNameZP.stockCruiseFailed: {
    ET.PERMANENT: Alert(
      "Stock Cruise Failed",
      "Restart the car",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.prompt, .2),
  },

  # Set Speed Assist switched, from the distance button or the UI (selfdrived_zp)
  EventNameZP.speedAssistSwitched: {
    ET.WARNING: speed_assist_switched_alert,
  },

  # an accelerator ready while something is in control: it swaps in only when
  # nothing is, so the next engagement after a full disengage drives it. Raised
  # for 3 s (accelerator_events), and no longer than the wait for the swap
  EventNameZP.bigModelAvailable: {
    ET.PERMANENT: Alert(
      "Big Model Ready",
      "Re-engage to switch",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.prompt, .2),
  },

  # an accelerator lost or too slow while engaged: the small model drives on
  # from a reset history and nothing disengages. As loud as a soft disable for
  # 5 s (accelerator_events), but it says what happened, not TAKE CONTROL:
  # nothing has let go, and a driver told to take control on every drop reads
  # it as a disengage. A disengage ends it
  EventNameZP.bigModelLinkLost: {
    ET.WARNING: Alert(
      "Big Model Lost",
      "Using small model",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.warningSoft, .2),
  },
}


class EventsZP(EventsBase):
  def __init__(self):
    super().__init__()
    self.event_counters = dict.fromkeys(EVENTS_ZP.keys(), 0)

  def get_events_mapping(self) -> dict[int, dict[str, Alert | AlertCallbackType]]:
    return EVENTS_ZP

  def get_event_name(self, event: int):
    return EVENT_NAME_ZP[event]

  def get_event_msg_type(self):
    return custom.OnroadEventZP.Event


class EventsWithZP(EventsBase):
  """EventsSP's base: sunnypilot's events, with zoompilot's beside them in `zp`. The two enums share
  int values, so add/has/remove/contains_in_list take a name of their own list (events_sp.zp.add for
  ours); what selfdrived asks of all of them at once (clear, contains an event type, create_alerts,
  names for its log-on-change check, add_from_msg) covers both, sunnypilot's first. to_msg stays
  sunnypilot's list, as published (onroadEventsSP.events, longitudinalPlanSP.events)."""
  def __init__(self):
    super().__init__()
    self.zp = EventsZP()

  @property
  def names(self) -> list:
    return self.events + [EVENT_NAME_ZP[e] for e in self.zp.events] if self.zp.events else self.events

  def clear(self) -> None:
    super().clear()
    self.zp.clear()

  def contains(self, event_type: str) -> bool:
    return super().contains(event_type) or self.zp.contains(event_type)

  def create_alerts(self, event_types: list[str], callback_args=None):
    return super().create_alerts(event_types, callback_args) + self.zp.create_alerts(event_types, callback_args)

  def add_from_msg(self, events):
    super().add_from_msg(events)
    for e in getattr(events, 'zp', ()):
      bisect.insort(self.zp.events, e)

  def to_msg_with_zp(self):
    # car_specific's events reach selfdrived through to_msg -> add_from_msg (CarEventsSP)
    return EventsMsgZP(super().to_msg(), self.zp.events)


class EventsMsgZP(list):
  """to_msg's list with zoompilot's event names beside it, which add_from_msg takes too."""
  def __init__(self, events, zp: list[int]):
    super().__init__(events)
    self.zp = zp
