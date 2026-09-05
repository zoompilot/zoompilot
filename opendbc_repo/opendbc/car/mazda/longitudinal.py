"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from enum import StrEnum

from opendbc.car import DT_CTRL, uds
from opendbc.car.carlog import carlog
from opendbc.car.can_definitions import CanData
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.values import CarControllerParams

RADAR_ADDR = 0x764
RADAR_BUS = 0


def create_radar_session_msg(session_type: int) -> CanData:
  """Build a fire-and-forget UDS DIAGNOSTIC_SESSION_CONTROL frame.

  The radar does not support COMMUNICATION_CONTROL. A programming session disables its
  periodic traffic and AEB until tester-present traffic stops or the S3 timeout expires.
  """
  return CanData(RADAR_ADDR, bytes([0x02, uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL, session_type, 0x00, 0x00, 0x00, 0x00, 0x00]), RADAR_BUS)


class RadarSessionState(StrEnum):
  STOCK = "stock"          # radar broadcasting; nothing transmitted
  SILENCING = "silencing"  # requesting the programming session
  SILENCED = "silenced"    # radar quiet; tester present + synthetic frames
  HANDBACK = "handback"    # requesting the default session; synthetic frames continue


RADAR_SESSION_LIMIT_FRAMES = int(CarControllerParams.RADAR_SESSION_LIMIT_T / DT_CTRL)


class RadarSessionManager:
  """Move the radar into and out of its UDS programming session.

  Takeover waits for the FSC cold-boot check and begins only while stopped because it disables
  stock AEB. Refused or unanswered requests leave the stock radar in control for the drive.
  """

  def __init__(self):
    self.state = RadarSessionState.STOCK
    self.state_frames = 0
    self.silencing_failed = False
    self.handback_completed = False

  def update(self, gate_passed: bool, stock_radar_alive: bool, handback: bool,
             standstill: bool, session_refused: bool, stock_radar_gone: bool) -> RadarSessionState:
    prev_state = self.state
    if handback:
      if self.state == RadarSessionState.SILENCING:
        # No hand-back is needed before takeover begins.
        self.state = RadarSessionState.STOCK
      elif self.state == RadarSessionState.SILENCED:
        self.state = RadarSessionState.HANDBACK
      elif self.state == RadarSessionState.HANDBACK and \
           (stock_radar_alive or self.state_frames >= RADAR_SESSION_LIMIT_FRAMES):
        # Finish hand-back after recovery or timeout and keep radar ownership stock.
        self.state = RadarSessionState.STOCK
        self.handback_completed = True
    else:
      if self.state == RadarSessionState.HANDBACK:
        # Restart takeover if hand-back is withdrawn before the process restarts.
        self.state = RadarSessionState.STOCK
      if self.state == RadarSessionState.STOCK and gate_passed and not self.handback_completed:
        # Begin silencing only before motion. Adopt an already quiet radar only after the full
        # ownership guard, not a normal short gap in stock traffic.
        if stock_radar_gone:
          self.state = RadarSessionState.SILENCED
        elif standstill and not self.silencing_failed:
          self.state = RadarSessionState.SILENCING
      elif self.state == RadarSessionState.SILENCING:
        if not stock_radar_alive:
          self.state = RadarSessionState.SILENCED
        elif session_refused or self.state_frames >= RADAR_SESSION_LIMIT_FRAMES:
          carlog.error(f"radar silencing failed ({'refused' if session_refused else 'no response'}); staying stock")
          self.state = RadarSessionState.STOCK
          self.silencing_failed = True
      elif self.state == RadarSessionState.SILENCED and stock_radar_alive:
        # Stop synthetic traffic if stock traffic returns. While moving, stock keeps the bus
        # until takeover can safely restart at standstill.
        self.state = RadarSessionState.SILENCING if (standstill and not self.silencing_failed) else RadarSessionState.STOCK

    self.state_frames = 0 if self.state != prev_state else self.state_frames + 1
    return self.state


RESUME_UNLATCH_LATCHED_FRAMES = int(CarControllerParams.RESUME_UNLATCH_LATCHED_T / DT_CTRL)
RESUME_REPULSE_FRAMES = int(CarControllerParams.RESUME_REPULSE_T / DT_CTRL)
LEAD_DEBOUNCE_FRAMES = int(CarControllerParams.LEAD_DEBOUNCE_T / DT_CTRL)
RELEASE_DEBOUNCE_FRAMES = int(CarControllerParams.RELEASE_DEBOUNCE_T / DT_CTRL)
BREAKAWAY_FRAMES = int(CarControllerParams.ACCEL_BREAKAWAY_T / DT_CTRL)


class StandstillHold:
  """Hold the car until the plan or driver requests movement.

  The plan supplies the hold command. It relaxes when the body ECU takes brake ownership,
  matching the stock stop-and-go sequence.
  """

  def __init__(self):
    self._reset()

  def _reset(self):
    self.holding = False
    self.car_has_hold = False
    self.unlatch_frames = 0
    self.release_frames = 0
    self.latched_release = False
    self.just_released = False
    self.latched_frames = 0  # frames waiting for the body ECU to release its hold
    self.repulsed = False

  def update(self, long_engaged: bool, stopping: bool, standstill: bool,
             plan_accel: float, brake_hold: bool, gas_pressed: bool) -> None:
    self.just_released = False
    if not long_engaged:
      self._reset()
      return

    was_holding = self.holding
    # Debounce plan movement requests; driver throttle releases the hold immediately.
    self.release_frames = self.release_frames + 1 if plan_accel > 0. else 0
    plan_wants_go = self.release_frames >= RELEASE_DEBOUNCE_FRAMES
    # Keep STOPPING off once the plan or driver requests acceleration.
    release = gas_pressed or plan_wants_go
    self.holding = not release and (stopping or standstill)

    if self.unlatch_frames > 0:
      self.unlatch_frames -= 1
    # Send one unlatch pulse per plan-driven body-latched release. Throttle releases use none.
    if was_holding and not self.holding and standstill and not gas_pressed and self.unlatch_frames == 0:
      # The previous frame records whether the body owned a hold that must be unlatched.
      self.latched_release = self.car_has_hold
      if self.latched_release:
        self.unlatch_frames = RESUME_UNLATCH_LATCHED_FRAMES
      self.just_released = True
      self.latched_frames = 0
      self.repulsed = False

    # Retry one unanswered body-latched release so a positive plan cannot remain blocked.
    if self.latched_release and not self.holding and standstill and brake_hold and not gas_pressed:
      self.latched_frames += 1
      if self.latched_frames >= RESUME_REPULSE_FRAMES and not self.repulsed and self.unlatch_frames == 0:
        self.unlatch_frames = RESUME_UNLATCH_LATCHED_FRAMES
        self.repulsed = True
    else:
      self.latched_frames = 0

    # Body ownership is valid only while the controller still requests a hold.
    self.car_has_hold = self.holding and standstill and brake_hold

  @property
  def stop_bits(self) -> bool:
    # Keep STOPPING and RESUME_UNLATCHING mutually exclusive, including during a re-hold.
    return self.holding and not self.car_has_hold and self.unlatch_frames == 0

  @property
  def resume_unlatching(self) -> bool:
    return self.unlatch_frames > 0

  @property
  def acc_active_2(self) -> bool:
    # Stock clears ACC_ACTIVE_2 when the command relaxes.
    return not self.car_has_hold


class AdvertisedLead:
  """Maintain consistent lead state across CRZ_CTRL and the 0x364 track.

  Advertisement follows perception rather than engagement. Visibility is debounced and the
  last measurement is propagated through short vision gaps like a radar track.
  """

  def __init__(self):
    self.visible = False
    self.flip_frames = 0
    self.holding = False
    self.lead = None
    self.real_lead = None
    self._measured = None

  def update(self, lead_visible: bool, d_rel: float, v_rel: float, holding: bool) -> None:
    if lead_visible != self.visible:
      self.flip_frames += 1
      if self.flip_frames >= LEAD_DEBOUNCE_FRAMES:
        self.visible = lead_visible
        self.flip_frames = 0
    else:
      self.flip_frames = 0

    if 0. < d_rel <= mazdacan.DIST_OBJ_MAX:
      self._measured = (d_rel, v_rel)
    elif not self.visible:
      # Expire stale state after the debounce window.
      self._measured = None
    elif self._measured is not None:
      # Propagate range through the gap instead of repeating a frozen track.
      d, v = self._measured
      self._measured = (d + v * DT_CTRL, v)
    self.real_lead = self._measured if self.visible else None
    self.lead = self.real_lead
    self.holding = holding

  @property
  def has_lead(self) -> bool:
    return self.lead is not None

  @property
  def ctrl_phase(self) -> int:
    # Use stock's relative-distance buckets and keep all lead fields consistent.
    if not self.has_lead:
      return 0
    return 3 if self.holding else 2
