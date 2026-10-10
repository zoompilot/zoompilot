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


class UIStateZP:
  """zoompilot's additions to UIStateSP: jetlink's snapshot and the USB port it holds, the
  torque bar's rail scale and the ICBM start lock."""

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
    self.sm_services_ext.append("modelDataV2SP")

  def update(self) -> None:
    # read where sm is updated, so the params thread never touches a message
    self._accelerator_state_name = str(self.sm['modelDataV2SP'].acceleratorState)
    self._update_torque_utilization()
    self._update_icbm_start_lock()

  def update_params(self) -> None:
    self._steer_rail_schedule = get_steer_rail_schedule(self.CP) if self.CP is not None else None
    # on the 5 Hz params pass, not per frame in a layout; a fitted chestnut owns chestnut_state
    self.jetlink = None if self.sm['deviceState'].chestnutPresent else jetlink_adapter.status()
    self._enforce_usb_port()
    # the Jetson configures the gadget ~25 s after a cold boot, after the one-shot
    # usb_unknown decision; recognising it late still clears "unknown"
    if (view := self.jetlink_view) is not None and view.present and self.usb_unknown:
      self.usb_unknown = False

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
    s = self.jetlink
    return s if s is not None and (s.enabled or s.present or s.progress is not None) else None

  def _jetlink_state(self, view):
    """ChestnutState for the link: progress and the records offroad, modelV2 and acceleratorState onroad"""
    from openpilot.selfdrive.ui.ui_state import ChestnutState  # defined by the class that mixes this in
    model_seen = self.sm.recv_frame["modelV2"] > self.started_frame
    running_big = self.sm.alive["modelV2"] and self.sm["modelV2"].big
    return ChestnutState(view.icon(self.started, model_seen, running_big, self._accelerator_state_name))

  def _enforce_usb_port(self) -> None:
    """ADB and Jetlink both need the comma's USB port: the link
    on turns ADB off, and the developer panels grey its toggle out. Here, not
    in the panels, so a link set from sunnylink counts too. jetlink's owner
    retries the port in seconds while ADB's gadget still holds it."""
    self.adb_blocked = self.jetlink is not None and self.jetlink.enabled
    if self.adb_blocked and self.params.get_bool("AdbEnabled"):
      self.params.put_bool("AdbEnabled", False, block=True)
