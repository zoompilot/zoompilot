"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import Enum

import numpy as np

from openpilot.cereal import messaging, log, custom
from opendbc.car.structs import car
from opendbc.sunnypilot.car.interfaces import get_steer_rail_schedule
from openpilot.common.params import Params
from openpilot.sunnypilot import jetlink_adapter
from openpilot.selfdrive.ui.sunnypilot.layouts.settings.display import OnroadBrightness
from openpilot.sunnypilot.models.helpers import ACTIVE_BUNDLE_KEYS, get_active_source
from openpilot.sunnypilot.sunnylink.sunnylink_state import SunnylinkState
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.sunnypilot.widgets.screen_saver import ScreenSaverSP
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import icbm_applicable

OpenpilotState = log.SelfdriveState.OpenpilotState
MADSState = custom.ModularAssistiveDrivingSystem.ModularAssistiveDrivingSystemState

ONROAD_BRIGHTNESS_TIMER_PAUSED = -1


class OnroadTimerStatus(Enum):
  NONE = 0
  PAUSE = 1
  RESUME = 2


class UIStateSP:
  def __init__(self):
    self.params = Params()
    self.CP_SP: custom.CarParamsSP | None = None
    self.has_icbm: bool = False
    self.is_sp_release: bool = self.params.get_bool("IsReleaseSpBranch")
    self.sm_services_ext = [
      "modelManagerSP", "selfdriveStateSP", "longitudinalPlanSP", "backupManagerSP",
      "gpsLocation", "lateralTorqueParameters", "carStateSP", "liveMapDataSP", "carParamsSP", "lateralDelay",
      "modelDataV2SP"
    ]

    self.sunnylink_state = SunnylinkState()

    self.screensaver = ScreenSaverSP(params=self.params)
    self.screensaver_enabled: bool = False

    self.active_bundle = None
    self.model_runner_tinygrad: bool = False
    # jetlink's snapshot (jetlink.openpilot.Status) from the params pass; None
    # with a chestnut fitted or no jetlink on this device
    self.jetlink = None
    # Jetlink holds the USB port, so ADB is off and its toggle greyed out
    self.adb_blocked: bool = False
    self._accelerator_state_name: str = 'none'
    self.blindspot: bool = False
    self.chevron_metrics = None
    self.custom_interactive_timeout: int = 0
    self.developer_ui = None
    self.hide_v_ego_ui: bool = False
    self.onroad_brightness: int = 0
    self.onroad_brightness_timer: int = 0
    self.onroad_brightness_timer_param: int = 0
    self.rainbow_path: bool = False
    self.road_name_toggle: bool = False
    self.rocket_fuel: bool = False
    self.speed_limit_mode = None
    self.standstill_timer: bool = False
    self.sunnylink_enabled: bool = False
    self.torque_bar: bool = False
    self.enforce_torque_control: bool = False
    self.custom_torque_params: bool = False
    self.torque_override_enabled: bool = False
    # carOutput's applied torque on a scale where the EPS rail is +-1, for the torque bar and
    # lane lines: a torque tune saturates at the rail, below the carcontroller's full scale
    self.torque_utilization: float = 0.0
    self._steer_rail_schedule = None
    self._sp_initialized: bool = False

  def update(self) -> None:
    if self.sunnylink_enabled:
      self.sunnylink_state.start()
    else:
      self.sunnylink_state.stop()
    # read where sm is updated, so the params thread never touches a message
    self._accelerator_state_name = str(self.sm['modelDataV2SP'].acceleratorState)
    self._update_torque_utilization()

  def _update_torque_utilization(self) -> None:
    torque = self.sm['carOutput'].actuatorsOutput.torque
    if self._steer_rail_schedule is not None:
      rail = float(np.interp(self.sm['carState'].vEgo, self._steer_rail_schedule[0], self._steer_rail_schedule[1]))
      torque = min(1.0, max(-1.0, torque / rail))
    self.torque_utilization = torque

  @property
  def jetlink_view(self):
    """jetlink's snapshot when the chestnut icon is the link's: no chestnut
    fitted, and something to show. Presence comes from jetlink: the comma is
    the gadget and enumerates nothing."""
    s = self.jetlink
    return s if s is not None and (s.enabled or s.present or s.progress is not None) else None

  def _jetlink_state(self, view):
    """ChestnutState for the link: progress and the records offroad, modelV2 and acceleratorState onroad"""
    from openpilot.selfdrive.ui.ui_state import ChestnutState  # defined by the class that mixes this in
    model_seen = self.sm.recv_frame["modelV2"] > self.started_frame
    running_big = self.sm.alive["modelV2"] and self.sm["modelV2"].big
    return ChestnutState(view.icon(self.started, model_seen, running_big, self._accelerator_state_name))

  def onroad_brightness_handle_alerts(self, _ui_state, alert):
    if _ui_state.sm.recv_frame["carState"] < _ui_state.started_frame:
      return

    has_alert = _ui_state.started and self.onroad_brightness != OnroadBrightness.AUTO and alert is not None

    self.update_onroad_brightness(has_alert)
    if has_alert:
      self.reset_onroad_sleep_timer()

  def update_onroad_brightness(self, has_alert: bool) -> None:
    if has_alert:
      return

    if self.onroad_brightness_timer > 0:
      self.onroad_brightness_timer -= 1

  def reset_onroad_sleep_timer(self, timer_status: OnroadTimerStatus = OnroadTimerStatus.NONE) -> None:
    # Toggling from active state to inactive
    if timer_status == OnroadTimerStatus.PAUSE and self.onroad_brightness_timer != ONROAD_BRIGHTNESS_TIMER_PAUSED:
      self.onroad_brightness_timer = ONROAD_BRIGHTNESS_TIMER_PAUSED
    # Toggling from a previously inactive state or resetting an active timer
    elif (self.onroad_brightness_timer_param >= 0 and self.onroad_brightness != OnroadBrightness.AUTO and
          self.onroad_brightness_timer != ONROAD_BRIGHTNESS_TIMER_PAUSED) or timer_status == OnroadTimerStatus.RESUME:
      if self.onroad_brightness == OnroadBrightness.AUTO_DARK:
        self.onroad_brightness_timer = 15 * gui_app.target_fps
      else:
        self.onroad_brightness_timer = self.onroad_brightness_timer_param * gui_app.target_fps

  @property
  def onroad_brightness_timer_expired(self) -> bool:
    return self.onroad_brightness != OnroadBrightness.AUTO and self.onroad_brightness_timer == 0

  @property
  def auto_onroad_brightness(self) -> bool:
    return self.onroad_brightness in (OnroadBrightness.AUTO, OnroadBrightness.AUTO_DARK)

  @staticmethod
  def update_status(ss, ss_sp, onroad_evt) -> str:
    state = ss.state
    mads = ss_sp.mads
    mads_state = mads.state
    # held by the car's own lane keep (mads.py update_stock_lkas): reads as lateral off
    mads_enabled = mads.enabled and not mads.lateralHeld

    if state == OpenpilotState.preEnabled:
      return "override"

    if state == OpenpilotState.overriding:
      if not mads.available:
        return "override"

      if any(e.overrideLongitudinal for e in onroad_evt):
        return "override"

    if mads_state in (MADSState.paused, MADSState.overriding) and not mads.lateralHeld:
      return "override"

    # MADS specific statuses
    if not mads.available:
      return "engaged" if ss.enabled else "disengaged"

    if not mads_enabled and not ss.enabled:
      return "disengaged"

    if mads_enabled and ss.enabled:
      return "engaged"

    if mads_enabled:
      return "lat_only"

    if ss.enabled:
      return "long_only"

    return "disengaged"

  def update_params(self) -> None:
    CP_SP_bytes = self.params.get("CarParamsSPPersistent")
    if CP_SP_bytes is not None:
      self.CP_SP = messaging.log_from_bytes(CP_SP_bytes, custom.CarParamsSP)
      self.has_icbm = self.CP_SP.intelligentCruiseButtonManagementAvailable and self.params.get_bool("IntelligentCruiseButtonManagement")
    self._steer_rail_schedule = get_steer_rail_schedule(self.CP) if self.CP is not None else None

    self._enforce_constraints()
    source = get_active_source(chestnut=self.chestnut_present, chestnut_active=self.chestnut_active,
                               chestnut_loading=self.chestnut_loading, offroad=self.is_offroad())
    self.active_bundle = self.params.get(ACTIVE_BUNDLE_KEYS[source])
    self.model_runner_tinygrad = self.active_bundle is not None and self.active_bundle.get("runner") == "tinygrad"
    self.chestnut_compiled = self.chestnut_compiled or self.model_runner_tinygrad
    # on the 5 Hz params pass, not per frame in a layout; a fitted chestnut owns chestnut_state
    self.jetlink = None if self.sm['deviceState'].chestnutPresent else jetlink_adapter.status()
    self._enforce_usb_port()
    # the Jetson configures the gadget ~25 s after a cold boot, after the one-shot
    # usb_unknown decision; recognising it late still clears "unknown"
    if (view := self.jetlink_view) is not None and view.present and self.usb_unknown:
      self.usb_unknown = False
    self.blindspot = self.params.get_bool("BlindSpot")
    self.chevron_metrics = self.params.get("ChevronInfo")
    self.custom_interactive_timeout = self.params.get("InteractivityTimeout", return_default=True)
    self.developer_ui = self.params.get("DevUIInfo")
    self.hide_v_ego_ui = self.params.get_bool("HideVEgoUI")
    self.onroad_brightness = int(float(self.params.get("OnroadScreenOffBrightness", return_default=True)))
    self.onroad_brightness_timer_param = self.params.get("OnroadScreenOffTimer", return_default=True)
    self.rainbow_path = self.params.get_bool("RainbowMode")
    self.road_name_toggle = self.params.get_bool("RoadNameToggle")
    self.rocket_fuel = self.params.get_bool("RocketFuel")
    self.speed_limit_mode = self.params.get("SpeedLimitMode", return_default=True)
    self.standstill_timer = self.params.get_bool("StandstillTimer")
    self.sunnylink_enabled = self.params.get_bool("SunnylinkEnabled")
    self.torque_bar = self.params.get_bool("TorqueBar")
    self.enforce_torque_control = self.params.get_bool("EnforceTorqueControl")
    self.custom_torque_params = self.params.get_bool("CustomTorqueParams")
    self.torque_override_enabled = self.params.get_bool("TorqueParamsOverrideEnabled")
    self.torque_override_lat_accel_factor = float(self.params.get("TorqueParamsOverrideLatAccelFactor", return_default=True))
    self.torque_override_friction = float(self.params.get("TorqueParamsOverrideFriction", return_default=True))
    self.true_v_ego_ui = self.params.get_bool("TrueVEgoUI")
    self.turn_signals = self.params.get_bool("ShowTurnSignals")
    self.boot_offroad_mode = self.params.get("DeviceBootMode", return_default=True)
    self.always_offroad = self.params.get_bool("OffroadMode") or self.params.get_bool("OffroadModeRequested")  # applied or pending
    self.screensaver_enabled = self.params.get_bool("ScreenSaverEnabled")

    if not self._sp_initialized:
      self._sp_initialized = True
      self.reset_onroad_sleep_timer()

  def _enforce_constraints(self) -> None:
    has_long = self.has_longitudinal_control
    CP = self.CP

    if CP is not None:
      if self.params.get_bool("EnforceTorqueControl") and self.params.get_bool("NeuralNetworkLateralControl"):
        self.params.put_bool("EnforceTorqueControl", False, block=True)
        self.params.put_bool("NeuralNetworkLateralControl", False, block=True)

      if self.params.get_bool("LateralJerkTorqueController") and self.params.get_bool("NeuralNetworkLateralControl"):
        self.params.put_bool("LateralJerkTorqueController", False, block=True)
        self.params.put_bool("NeuralNetworkLateralControl", False, block=True)

      # Angle steering: no torque-based lateral controls
      if CP.steerControlType == car.CarParams.SteerControlType.angle:
        self.params.remove("EnforceTorqueControl")
        self.params.remove("NeuralNetworkLateralControl")
        self.params.remove("LateralJerkTorqueController")

      # Alpha longitudinal: clear if not available
      if not CP.alphaLongitudinalAvailable:
        self.params.remove("AlphaLongitudinalEnabled")

      # BSM not available: clear BSM-dependent settings
      if not CP.enableBsm:
        self.params.remove("AutoLaneChangeBsmDelay")
    else:
      # Clear car-dependent params only offroad. During first-drive initialization, card may
      # seed defaults before CarParamsPersistent is available.
      if not self.started:
        self.params.remove("EnforceTorqueControl")
        self.params.remove("NeuralNetworkLateralControl")
        self.params.remove("LateralJerkTorqueController")
        self.params.remove("AlphaLongitudinalEnabled")

    # No longitudinal control: no experimental mode. DEC, speed assist, custom ACC and
    # SCC keep their values; card and the planner leave them idle until long or ICBM returns.
    if not has_long:
      self.params.remove("ExperimentalMode")

    # ICBM: clear where the buttons have no set speed to move (icbm_applicable)
    if self.CP_SP is not None:
      if not icbm_applicable(self.CP, self.CP_SP):
        self.params.remove("IntelligentCruiseButtonManagement")
        self.has_icbm = False
    else:
      self.params.remove("IntelligentCruiseButtonManagement")
      self.has_icbm = False

  def _enforce_usb_port(self) -> None:
    """ADB and Jetlink both need the comma's USB port: the link
    on turns ADB off, and the developer panels grey its toggle out. Here, not
    in the panels, so a link set from sunnylink counts too. jetlink's owner
    retries the port in seconds while ADB's gadget still holds it."""
    self.adb_blocked = self.jetlink is not None and self.jetlink.enabled
    if self.adb_blocked and self.params.get_bool("AdbEnabled"):
      self.params.put_bool("AdbEnabled", False, block=True)


def set_always_offroad(params: Params, enable: bool) -> None:
  """The UI writes the forced-offroad preference only. hardwared applies it: entering waits for
  a silenced stock ECU to be handed back before pandad sees OffroadMode, exiting clears the old
  session's CarParams first so the fresh session sequences like a boot."""
  params.put_bool("OffroadModeRequested", enable)


class DeviceSP:
  def __init__(self):
    self._blocked_by_screensaver: bool = False

  def _set_awake(self, on: bool, _ui_state=None):
    self._blocked_by_screensaver = False

    if not on and _ui_state.screensaver_enabled:
      if _ui_state.screensaver.was_dismissed:
        self.dismiss_screensaver(_ui_state)
      elif _ui_state.screensaver.is_active:
        self._blocked_by_screensaver = True
      else:
        _ui_state.screensaver.initialize()
        gui_app.push_widget(_ui_state.screensaver)
        self._blocked_by_screensaver = True
    else:
      self.dismiss_screensaver(_ui_state)

    # blocked runs every frame, so write only when actually sleeping; ignition is off here,
    # so hardwared applies the preference at once
    if _ui_state.boot_offroad_mode == 1 and not on and not self._blocked_by_screensaver:
      _ui_state.params.put_bool("OffroadModeRequested", True)

  def dismiss_screensaver(self, _ui_state) -> None:
    if gui_app.get_active_widget() == _ui_state.screensaver:
      gui_app.pop_widget()
    self._blocked_by_screensaver = False

  @staticmethod
  def set_onroad_brightness(_ui_state, awake: bool, cur_brightness: float) -> float:
    if not awake or not _ui_state.started:
      return cur_brightness

    if _ui_state.onroad_brightness_timer != 0:
      if _ui_state.onroad_brightness == OnroadBrightness.AUTO_DARK:
        return max(30.0, cur_brightness)
      return cur_brightness

    # 0: Auto (Default), 1: Auto (Dark), 2: Screen Off
    if _ui_state.onroad_brightness == OnroadBrightness.AUTO:
      return cur_brightness
    if _ui_state.onroad_brightness == OnroadBrightness.AUTO_DARK:
      return cur_brightness
    if _ui_state.onroad_brightness == OnroadBrightness.SCREEN_OFF:
      return 0.0

    # 3-22: 5% - 100%
    return float((_ui_state.onroad_brightness - 2) * 5)

  @staticmethod
  def set_min_onroad_brightness(_ui_state, min_brightness: int) -> int:
    if _ui_state.onroad_brightness == OnroadBrightness.AUTO_DARK:
      min_brightness = 10

    return min_brightness

  @staticmethod
  def wake_from_dimmed_onroad_brightness(_ui_state, evs) -> None:
    if _ui_state.started and (_ui_state.onroad_brightness_timer_expired or _ui_state.onroad_brightness == OnroadBrightness.AUTO_DARK):
      if any(ev.left_down for ev in evs):
        if _ui_state.onroad_brightness_timer_expired:
          gui_app.mouse_events.clear()
        _ui_state.reset_onroad_sleep_timer()
