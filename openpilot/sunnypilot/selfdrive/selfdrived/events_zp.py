"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

zoompilot's alerts: its own events, and its text for some of sunnypilot's. events.py merges
EVENTS_ZP over EVENTS_SP once at import.
"""
import openpilot.cereal.messaging as messaging
from openpilot.cereal import log, custom
from opendbc.car.structs import car
from opendbc.sunnypilot.car.stock_ecu import StockEcuState
from openpilot.common.constants import CV
from openpilot.common.hardware import HARDWARE
from openpilot.selfdrive.selfdrived.events import get_display_speed
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import Priority, ET, Alert, NoEntryAlert, EngagementAlert, \
  NormalPermanentAlert, AlertCallbackType
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import PCM_LONG_REQUIRED_MAX_SET_SPEED, CONFIRM_SPEED_THRESHOLD

AlertSize = log.SelfdriveState.AlertSize
AlertStatus = log.SelfdriveState.AlertStatus
VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert
AudibleAlertSP = custom.SelfdriveStateSP.AudibleAlert
EventNameSP = custom.OnroadEventSP.EventName
AssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState

IS_MICI = HARDWARE.get_device_type() == 'mici'


# What the driver has to do, per stock ECU state that does not engage normally, in the shape of
# upstream's no-entry alerts ("Gear not D"). Main off has no entry: the cluster's MRCC
# indicator is the car's own.
STOCK_ECU_ALERT_TEXT = {
  StockEcuState.STARTING: ("Longitudinal Initializing", "Remain parked"),
  StockEcuState.PARK_TO_TAKE_OVER: ("Park to Engage Longitudinal", "Alpha longitudinal takes over at the next stop"),
  StockEcuState.STOCK_CRUISE_ON: ("Turn Off Stock Cruise", "Alpha longitudinal waits for it"),
  StockEcuState.RESTORING: ("Longitudinal Handing Back", "Stock cruise returns when it completes"),
  StockEcuState.FAILED: ("Longitudinal Initializing Failed", "Restart the car to retry"),
}


def big_model_ready_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  # an accelerator's comes a second after its swap, when the driver can engage;
  # its offer to switch is the one titled "Big Model Ready"
  accelerator = sm['modelDataV2SP'].acceleratorState != custom.ModelDataV2SP.AcceleratorState.none
  return Alert("Big Model Active" if accelerator else "Big Model Ready", "",
               AlertStatus.normal, AlertSize.small,
               Priority.LOW, VisualAlert.none, AudibleAlert.prompt, 2.)


def stock_ecu_not_ready_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  state = str(sm['carStateSP'].zoompilot.stockEcu)
  title, line = STOCK_ECU_ALERT_TEXT.get(state, STOCK_ECU_ALERT_TEXT[StockEcuState.STARTING])
  if state == StockEcuState.STARTING and not CS.standstill:
    line = "Wait for the radar takeover"  # a moving takeover in flight; parking is not required
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


EVENTS_ZP: dict[int, dict[str, Alert | AlertCallbackType]] = {
  EventNameSP.stockLkasOff: {
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
  EventNameSP.stockLkasArming: {
    ET.PERMANENT: NormalPermanentAlert("Lateral Disabled", "Waiting for steering", priority=Priority.LOW),
  } if IS_MICI else {},

  # Sound-only mirrors of the longitudinal selfdrive transitions while a declared MADS
  # button owns lateral. PERMANENT carries no state-machine meaning, so the chime cannot
  # enable or disable anything.
  EventNameSP.longitudinalEnableChime: {
    ET.PERMANENT: EngagementAlert(AudibleAlert.engage),
  },

  EventNameSP.longitudinalDisableChime: {
    ET.PERMANENT: EngagementAlert(AudibleAlert.disengage),
  },

  # pause-on-brake engagement with the brake already down: MADS enters paused and resumes on
  # release, the same moment the panda arms its pending request
  EventNameSP.silentPedalPressed: {
    ET.NO_ENTRY: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, 0.),
  },

  # The panda is rejecting our steering while MADS thinks it is steering: the wheel is
  # unsteered from the first rejected frame, not from the disable 2 s later (Mazda routes
  # 00000116/117: 2 s of rejected 0x243 with the camera relay-blocked latched the EPS fault)
  EventNameSP.controlsMismatchLateralWarning: {
    ET.WARNING: Alert(
      "Take Control",
      "Steering Blocked by Panda Safety",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.steerRequired, AudibleAlert.prompt, .5),
  },

  # The three stock ECU alerts are PERMANENT, not the *AlertOnly WARNING convention: WARNING
  # shows only while cruise or MADS lateral is active, and these must reach a driver whose
  # lateral is off or paused (brake held at the stop where the takeover happens; route
  # 0000021b showed nothing for three minutes). Each press is one frame; the alert's own
  # duration keeps it up.
  EventNameSP.stockEcuNotReady: {
    ET.PERMANENT: stock_ecu_not_ready_alert,
  },

  # Parked with the takeover still starting: unprompted, re-raised every frame it holds. MID so
  # the startup and big model banners (LOWER/LOW) cannot cover it while the driver waits.
  EventNameSP.stockEcuInitializing: {
    ET.PERMANENT: NormalPermanentAlert(*STOCK_ECU_ALERT_TEXT[StockEcuState.STARTING], priority=Priority.MID),
  },

  EventNameSP.stockEcuReady: {
    ET.PERMANENT: NormalPermanentAlert("Alpha Longitudinal Ready", duration=2.),
  },

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
    ET.WARNING: Alert(
      # mici renders text1 above 16 chars at the smallest font and clips
      "Road Edge Ahead" if IS_MICI else "Lane Change Unavailable: Road Edge",
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.prompt, 0.1),
  },

  # an accelerator ready while something is in control: it swaps in only when
  # nothing is, so the next engagement after a full disengage drives it. Raised
  # for 3 s (accelerator_events), and no longer than the wait for the swap
  EventNameSP.bigModelAvailable: {
    ET.PERMANENT: Alert(
      "Big Model Ready",
      "Re-engage to switch",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.prompt, .2),
  },

  EventNameSP.bigModelReady: {
    ET.PERMANENT: big_model_ready_alert,
  },

  # an accelerator lost or too slow while engaged: the small model drives on
  # from a reset history and nothing disengages. As loud as a soft disable for
  # 5 s (accelerator_events), but it says what happened, not TAKE CONTROL:
  # nothing has let go, and a driver told to take control on every drop read
  # it as a disengage (2026-10-04). A disengage ends it
  EventNameSP.bigModelLinkLost: {
    ET.WARNING: Alert(
      "Big Model Lost",
      "Using small model",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.warningSoft, .2),
  },
}
