"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

from opendbc.sunnypilot.car.lateral_tune import get_steer_rail_schedule
from openpilot.sunnypilot import jetlink_adapter
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.demand import needs_icbm
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.icbm_latch import icbm_active


def lane_center_mode(params) -> int:
  """Lane center assist: 0 off, 1 on, 2 debug ui. Blendful stays the bool modeld and sunnylink read;
  BlendfulReadout only counts while it is on."""
  return params.get_bool("BlendfulReadout") + 1 if params.get_bool("Blendful") else 0


def set_lane_center_mode(params, mode: int) -> None:
  params.put_bool("Blendful", mode > 0, block=True)
  params.put_bool("BlendfulReadout", mode == 2, block=True)


class UIStateZP:
  """zoompilot's additions to UIStateSP: jetlink's snapshot and the USB port it holds, the
  torque bar's rail scale, the ICBM start lock, the brand the settings gate on and experimental
  mode, which upstream retired, and the home screens' model name."""

  def __init__(self):
    self.icbm_start_locked: bool = False
    self.icbm_running: bool = False
    # jetlink's snapshot (jetlink.openpilot.Status) from the params pass; None
    # with a chestnut fitted or no jetlink on this device
    self.jetlink = None
    # Jetlink holds the USB port, so ADB is off and its toggle greyed out
    self.adb_blocked: bool = False
    self._accelerator_state_name: str = 'none'
    # carOutput's applied torque on a scale where the EPS rail is +-1, for the torque bar and
    # lane lines: a torque tune saturates at the rail, below the carcontroller's full scale
    self.torque_utilization: float = 0.0
    self._steer_rail_schedule = None
    # experimental (end-to-end) longitudinal: as selfdrived runs it onroad, the setting offroad
    self.experimental_mode: bool = False
    self._experimental_mode_setting: bool = False
    # the carrying model's short name for the home screens; None with HomeShowActiveModel off
    self.home_model_name: str | None = None
    # lane center assist's debug readout on the comma 4
    self.blendful_readout: bool = False

  def update(self) -> None:
    # read where sm is updated, so the params thread never touches a message
    self._accelerator_state_name = str(self.sm['modelDataV2SP'].zoompilot.acceleratorState)
    self._update_torque_utilization()
    self._update_icbm_start_lock()
    if self.started and self.sm.recv_frame['selfdriveStateSP'] >= self.started_frame:
      self.experimental_mode = self.sm['selfdriveStateSP'].zoompilot.experimentalMode
    else:
      self.experimental_mode = self._experimental_mode_setting

  def update_params(self) -> None:
    self._experimental_mode_setting = self.params.get_bool("ExperimentalMode")
    self.blendful_readout = lane_center_mode(self.params) == 2
    self._steer_rail_schedule = get_steer_rail_schedule(self.CP) if self.CP is not None else None
    # on the 5 Hz params pass, not per frame in a layout; a fitted chestnut owns chestnut_state
    self.jetlink = None if self.sm['deviceState'].chestnutPresent else jetlink_adapter.status()
    self._enforce_usb_port()
    # the Jetson configures the gadget ~25 s after a cold boot, after the one-shot
    # usb_unknown decision; recognising it late still clears "unknown"
    if (view := self.jetlink_view) is not None and view.present and self.usb_unknown:
      self.usb_unknown = False
    # here rather than per frame: it reads the model slots and, on a list refetch, the lists
    from openpilot.selfdrive.ui.sunnypilot.home_model import carrying_short_name
    self.home_model_name = carrying_short_name() if self.params.get_bool("HomeShowActiveModel") else None

  def set_experimental_mode(self, on: bool) -> None:
    """The setting, from a toggle or a long press; selfdrived picks it up within a second."""
    self.params.put_bool("ExperimentalMode", on, block=True)
    self._experimental_mode_setting = on

  # Upstream retired the experimental/chill switch with chill itself, so the driving model drives the
  # gas and brakes. zoompilot keeps chill as the opt-out, "legacy chill mode": the off of the same
  # ExperimentalMode param, so nobody's setting moved. The settings show it as legacy chill.
  @property
  def legacy_chill(self) -> bool:
    return not self._experimental_mode_setting

  def set_legacy_chill(self, on: bool) -> None:
    self.set_experimental_mode(not on)

  def set_alpha_longitudinal(self, on: bool) -> None:
    """The setting, offroad or disengaged onroad. card reads it at fingerprint, so onroad it
    cycles the onroad state; hardwared holds that cycle for any stock ECU hand-back first."""
    self.params.put_bool("AlphaLongitudinalEnabled", on, block=True)
    if self.started:
      self.params.put_bool("OnroadCycleRequested", True, block=True)

  def _update_icbm_start_lock(self) -> None:
    """card moves ICBM only between engagements (icbm_latch), so a feature that needs it,
    turned on while engaged with ICBM off, would wait for the next engage. Settings lock
    turning those on until the driver disengages; turning them off stays allowed."""
    if self.CP_SP is None or not (self.has_icbm and self.started):
      self.icbm_running = self.CP_SP is not None and not self.CP_SP.pcmCruiseSpeed
      self.icbm_start_locked = False
      return
    self.icbm_running = icbm_active(self.sm['carStateSP'], self.CP_SP)
    engaged = self.sm['selfdriveState'].enabled or self.sm['carState'].cruiseState.enabled
    self.icbm_start_locked = engaged and not self.icbm_running

  def icbm_turn_on_allowed(self, key: str, on: bool) -> bool:
    """Whether a setting may be changed to (or left at) `on` now: blocked only when that would
    bring ICBM up while engaged with it off."""
    return not (on and self.icbm_start_locked and needs_icbm(self.CP, key))

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
    return self.jetlink if self.jetlink is not None and self.jetlink.shown else None

  def _jetlink_state(self, view):
    """ChestnutState for the link: progress and the records offroad, modelV2 and acceleratorState onroad"""
    from openpilot.selfdrive.ui.ui_state import ChestnutState  # defined by the class that mixes this in
    model_seen = self.sm.recv_frame["modelV2"] > self.started_frame
    running_big = self.sm.alive["modelV2"] and self.sm["modelV2"].big
    return ChestnutState(view.icon(self.started, model_seen, running_big, self._accelerator_state_name))

  def offroad_brand(self) -> str:
    """Brand for settings gating: the manual platform selection while offroad, CarParams otherwise."""
    brand = ""
    if self.is_offroad():
      bundle = self.params.get("CarPlatformBundle")
      brand = bundle.get("brand", "") if isinstance(bundle, dict) else ""
    if not brand and self.CP is not None:
      brand = self.CP.brand
    return brand

  def _enforce_usb_port(self) -> None:
    """ADB and Jetlink both need the comma's USB port: the link
    on turns ADB off, and the developer panels grey its toggle out. Here, not
    in the panels, so a link set from sunnylink counts too. jetlink's owner
    retries the port in seconds while ADB's gadget still holds it."""
    self.adb_blocked = self.jetlink is not None and self.jetlink.holds_port
    if self.adb_blocked and self.params.get_bool("AdbEnabled"):
      self.params.put_bool("AdbEnabled", False, block=True)
