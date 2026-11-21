"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from enum import StrEnum

from opendbc.can.parser import CANParser
from opendbc.car import Bus, DT_CTRL, create_button_events, structs
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from opendbc.sunnypilot.car.mazda.icbm import BUTTONS
from opendbc.sunnypilot.car.mazda.values import MazdaFlagsSP
from opendbc.sunnypilot.mads_base import MadsCarStateBase

VisualAlert = structs.CarControl.HUDControl.VisualAlert
ButtonType = structs.CarState.ButtonEvent.Type

# The ON shortcut becomes a toggle event only after disabled standby has settled.
# The same window rejects feedback that the control loop has stopped refreshing.
MADS_ON_STANDBY_FRAMES = round(0.5 / DT_CTRL)

TJA_MRCC_MAX_TX_FRAMES = 3
TJA_MRCC_RAW_OFF_CONFIRM_FRAMES = 5
# The body ECU drops discrete presses faster than one per 200 ms (measured, docs/zoompilot/icbm.md);
# the arm clears in PEDALS ~80 ms after a press plus the 50 ms confirm.
TJA_MRCC_TX_PERIOD = 0.2  # s between undo presses
# The press-induced arm appears ~80 ms after the press edge (docs/zoompilot/mazda-lateral.md).
TJA_MRCC_ARM_WAIT_FRAMES = int(1.0 / DT_CTRL)
# The white wheel waits this long on a fully-off, quiet cruise before it displays.
MADS_WHITE_HUD_OFF_CONFIRM_FRAMES = int(0.5 / DT_CTRL)


# The MADS white wheel: the dash draws it when the camera's own HUD frame carries TJA=2,
# and the body reads the same frame. The white bit is only ever XORed into an exact
# camera payload audited to be an idle frame, never a frame we composed. Every observed
# FSC idle family on TJA-declared cars is enumerated: the OFF family with its
# counter-nibble twins, the LINE_VISIBLE families and their high-beam variants, and the
# partial-lane LANE_LINES=3/4 encodings. Exact bases only; do not widen to a field-based
# rule until more captures are audited. No base may carry ERR_BIT, NO_ERR_BIT (byte 1 0x40,
# unsettled camera in carstate's takeover gate), LDW or a hands warning.
MADS_HUD_SAFE_BASE_PAYLOADS = frozenset(bytes.fromhex(h) for h in (
  "4201000000001040", "4201000000001060", "4221000000004040", "4221000000001040",
  "4221000000001060", "4201000000004040", "0221000000000040", "4201000000000040",
  "4221000000000040", "0221000000001040", "4102000000001040",
  "4122000000001040", "4102000000004040", "4122000000004040",
  "4221000000004060", "4122000000000040", "4103000000001040", "4104000000001040",
  "4123000000000040", "4124000000000040", "4123000000001040", "4124000000001040",
  "4123000000004040", "4124000000004040", "4102000000001060", "4102000000004060",
  "4122000000001060", "4122000000004060", "0122000000000040", "0122000000004040",
  "4202000000001040", "4102000000000040",
))
# OFF to WHITE is TJA 0 to 2 only: one bit, byte 4 0x20, XORed in, never a frame swap.
MADS_HUD_WHITE_TJA_XOR = bytes.fromhex("0000000020000000")
# 64-bit big-endian keep-mask over the bits an idle camera may still move between
# samples: TJA (byte 4, 0x70), TJA_TRANSITION (byte 3, 0x0C), and the unnamed byte-3
# transition bits 0x03. Do not clear byte-4 0x80 or unrelated byte-0 family bits.
# Every allowlisted base carries zero in the masked bits, so no two bases share a key.
CAM_LANEINFO_TJA_NORMALIZE_MASK = 0xFFFFFFF08FFFFFFF
_MADS_HUD_SAFE_BASE_BY_INT = {
  int.from_bytes(b, "big") & CAM_LANEINFO_TJA_NORMALIZE_MASK: b
  for b in MADS_HUD_SAFE_BASE_PAYLOADS
}


def white_hud_allowlist_base(fsc_raw: bytes | None) -> bytes | None:
  """The allowlisted idle base for the camera's current frame, TJA/transition bits ignored."""
  if fsc_raw is None or len(fsc_raw) != 8:
    return None
  return _MADS_HUD_SAFE_BASE_BY_INT.get(
    int.from_bytes(fsc_raw, "big") & CAM_LANEINFO_TJA_NORMALIZE_MASK
  )


def apply_mads_white_hud(fsc_raw: bytes | None, packed_dat: bytes, enabled: bool) -> bytes:
  """Set TJA=2 on the camera's own allowlisted idle frame, and on nothing else.

  packed_dat must be exactly the base the camera's current frame normalizes to: any other
  payload, or an unknown camera frame, passes through untouched.
  """
  if not enabled or len(packed_dat) != 8:
    return packed_dat
  if packed_dat != white_hud_allowlist_base(fsc_raw):
    return packed_dat
  return bytes(a ^ b for a, b in zip(packed_dat, MADS_HUD_WHITE_TJA_XOR, strict=True))


def create_mrcc_off_cmd(packer, counter):
  # The wheel's MRCC master press, active-low: every button bit 0 with its inversion 1,
  # the master signature in BIT1/BIT1_INV plus BIT2/BIT3, counter plus one. Only the
  # TJA-press cleanup sends it, and the panda pins the exact bytes in mazda_mrcc_off_msg_valid.
  values = {
    "CAN_OFF_INV": 1, "SET_P_INV": 1, "RES_INV": 1, "SET_M_INV": 1,
    "DISTANCE_LESS_INV": 1, "DISTANCE_MORE_INV": 1, "MODE_X_INV": 1, "MODE_Y_INV": 1,
    "BIT1_INV": 1, "BIT2": 1, "BIT3": 1,
    "CTR": (counter + 1) % 16,
  }
  return packer.make_can_msg("CRZ_BTNS", 0, values)


class MadsCarController:
  """The TJA button as the MADS switch, on the CarController (its packer, frame and
  last_button_frame): the press-induced MRCC arm undone, and the white wheel on the camera's
  own HUD frame."""

  def __init__(self):
    self.tja_button_prev = False
    self.mrcc_armed_prev: bool | None = None
    self.mrcc_undo_pending = False
    self.mrcc_undo_saw_armed = False
    self.mrcc_undo_frames = 0
    self.mrcc_raw_off_frames = 0
    self.mrcc_arm_wait_frames = 0
    # The white wheel rides the alert frame; on_bus tracks that the white bit is on the wire.
    self.mads_white_hud_off_frames = 0
    self.mads_white_hud_on_bus = False

  def update_mrcc_cleanup(self, CC, CC_SP, CS, handback_active: bool):
    # Undo the MRCC arm a physical TJA press causes: the wheel's press reaches every bus-0
    # ECU directly, so the button the driver declared as the lateral switch also arms cruise.
    # Answer with the driver's own MRCC master press until raw PEDALS confirms the arm is gone.
    can_sends = []
    if not self.CP_SP.flags & MazdaFlagsSP.TJA_BUTTON:
      return can_sends

    raw_armed = bool(CS.mrcc_armed_raw)
    # PEDALS drops both cruise bits through a brake transition; the filtered state bridges
    # those samples, and only sustained raw-off is authoritative.
    self.mrcc_raw_off_frames = 0 if raw_armed else self.mrcc_raw_off_frames + 1
    raw_off_confirmed = self.mrcc_raw_off_frames >= TJA_MRCC_RAW_OFF_CONFIRM_FRAMES
    # PEDALS holds both cruise bits low under braking: the clock restarts there and runs
    # again once the pedal is off.
    self.mrcc_arm_wait_frames = 0 if CS.out.brakePressed else self.mrcc_arm_wait_frames + 1
    mrcc_armed = raw_armed or (CS.cruise_available and not raw_off_confirmed)

    if CS.tja_button and not self.tja_button_prev:
      # A press before the previous press's arm reconciled sees that arm as its own
      # artifact, not the driver's baseline.
      if self.mrcc_undo_pending:
        self.mrcc_undo_frames = 0
      else:
        # PEDALS can already show the press-induced arm in the same cycle as the edge; the
        # previous stable sample is the state that existed before the press.
        armed_before_press = self.mrcc_armed_prev if self.mrcc_armed_prev is not None else mrcc_armed
        self.mrcc_undo_pending = not armed_before_press
      self.mrcc_undo_saw_armed = False
      self.mrcc_arm_wait_frames = 0
    self.tja_button_prev = bool(CS.tja_button)
    self.mrcc_armed_prev = mrcc_armed

    # The arm is fully reconciled; the budget returns for the next press.
    if not mrcc_armed and not CS.cruise_enabled and not CS.out.cruiseState.enabled:
      self.mrcc_undo_frames = 0

    if not self.mrcc_undo_pending:
      return can_sends

    self.mrcc_undo_saw_armed |= raw_armed

    # The driver's own cruise presses own CRZ_BTNS from this cycle on, and restoring
    # stock ECU ownership changes who owns the arm.
    driver_activity = (CS.cancel_button or CS.resume_button or CS.accel_button or CS.decel_button or
                       CS.mrcc_button or CS.distance_button)
    if driver_activity or handback_active or CC_SP.stockEcuHandBack:
      self.mrcc_undo_pending = False
      return can_sends

    # The arm never appeared: a takeover already disarmed it, or this car does not arm
    # through the press. Past the bounded brake-free wait, stand down.
    if not self.mrcc_undo_saw_armed:
      if self.mrcc_arm_wait_frames >= TJA_MRCC_ARM_WAIT_FRAMES:
        self.mrcc_undo_pending = False
      return can_sends

    if raw_off_confirmed:
      self.mrcc_undo_pending = False
      return can_sends

    # openpilot's own cancel or resume interleaves button frames; wait it out rather
    # than race the counter stream.
    if CC.cruiseControl.cancel or CC.cruiseControl.resume:
      return can_sends

    # Never inside the driver's held press.
    if CS.tja_button:
      return can_sends

    # One press per body-paced slot, within the episode budget; never while PEDALS reads disarmed,
    # where the master press would arm instead of disarm. last_button_frame also paces ICBM.
    if raw_armed and self.mrcc_undo_frames < TJA_MRCC_MAX_TX_FRAMES and \
       (self.frame - self.last_button_frame) * DT_CTRL > TJA_MRCC_TX_PERIOD:
      can_sends.append(create_mrcc_off_cmd(self.packer, CS.crz_btns_counter))
      self.last_button_frame = self.frame
      self.mrcc_undo_frames += 1
      if self.mrcc_undo_frames >= TJA_MRCC_MAX_TX_FRAMES:
        self.mrcc_undo_pending = False

    return can_sends

  def update_white_hud(self, CC, CC_SP, CS, handback_active: bool) -> bool:
    """Whether the alert frame carries the white wheel.

    On a TJA-declared car with MADS active and cruise fully off, the white-wheel TJA=2 state
    is XORed into the camera's current payload, but only onto an exact allowlisted idle base:
    TJA=2 is not display-only, the body reads the same frame. Every cruise interaction fails
    closed, and a white wheel HUD state that became unsafe is withdrawn immediately, outside
    the cadence.
    """
    session_ambiguous = handback_active or CC_SP.stockEcuHandBack
    mrcc_off = (not session_ambiguous and not CS.mrcc_armed_raw and
                not CS.cruise_available and not CS.cruise_enabled)

    # Every button a TJA wheel carries: TJA, MRCC, SET+/-, RES, DISTANCE, plus the
    # synthesized ICBM set presses and openpilot's own cancel/resume.
    button_activity = (CS.tja_button or CS.mrcc_button or CS.cancel_button or CS.resume_button or
                       CS.accel_button or CS.decel_button or CS.distance_button or
                       CC_SP.intelligentCruiseButtonManagement.sendButton in BUTTONS or
                       CC.cruiseControl.cancel or CC.cruiseControl.resume)

    white_allowed = (
      bool(self.CP_SP.flags & MazdaFlagsSP.TJA_BUTTON) and
      CC_SP.mads.active and
      CS.cam_laneinfo_live and
      white_hud_allowlist_base(CS.cam_laneinfo_raw) is not None and
      CC.hudControl.visualAlert == VisualAlert.none and
      not button_activity and
      mrcc_off
    )
    if white_allowed:
      self.mads_white_hud_off_frames = min(self.mads_white_hud_off_frames + 1, MADS_WHITE_HUD_OFF_CONFIRM_FRAMES)
    else:
      self.mads_white_hud_off_frames = 0
    return white_allowed and self.mads_white_hud_off_frames >= MADS_WHITE_HUD_OFF_CONFIRM_FRAMES

  def white_hud_frame(self, CS, alert, white: bool):
    """The alert frame as sent: the camera's own allowlisted idle base with TJA=2 when white."""
    fsc_raw = CS.cam_laneinfo_raw
    payload = white_hud_allowlist_base(fsc_raw) if white else alert[1]
    self.mads_white_hud_on_bus = white
    return alert[0], apply_mads_white_hud(fsc_raw, payload, white), alert[2]


class MadsCarState(MadsCarStateBase):
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    # Unfiltered PEDALS cruise state; the filtered public state bridges brake dropouts.
    self.mrcc_armed_raw = False
    self.tja_button = 0
    self.prev_tja_button = 0
    # Active-low wheel MRCC master (CRZ_BTNS.BIT1), read from the bus-0 parser: a 0 is a press.
    self.mrcc_button = 0
    self.prev_mrcc_button = 0
    self.crz_btns_seen = False
    self.mads_available = False
    self.mads_enabled = False
    self.mads_feedback_age = MADS_ON_STANDBY_FRAMES
    self.mads_standby_frames = 0
    self.main_on_button = False
    self.main_on_lkas_button = False
    self.prev_main_on_lkas_button = False

  def update_mads_feedback(self, CC: structs.CarControl, CC_SP: structs.CarControlSP) -> None:
    self.mads_available = CC_SP.mads.available
    self.mads_enabled = CC_SP.mads.enabled or CC_SP.mads.active or CC.latActive
    self.mads_feedback_age = 0

  def main_on_lkas_events(self, ret: structs.CarState, btns) -> list[structs.CarState.ButtonEvent]:
    self.mads_feedback_age = min(self.mads_feedback_age + 1, MADS_ON_STANDBY_FRAMES)
    on_pressed = btns["MODE_Y"] == 1
    brake_restricted = self.CP.alternativeExperience & (ALTERNATIVE_EXPERIENCE.MADS_DISENGAGE_LATERAL_ON_BRAKE |
                                                       ALTERNATIVE_EXPERIENCE.MADS_PAUSE_LATERAL_ON_BRAKE)
    standby = (not self.CP_SP.flags & MazdaFlagsSP.TJA_BUTTON and self.mads_available and not self.mads_enabled and
               self.mads_feedback_age < MADS_ON_STANDBY_FRAMES and ret.cruiseState.available and self.out.cruiseState.available and
               not self.cruise_enabled and not self.out.cruiseState.enabled and
               not (self.cancel_button or self.resume_button or self.accel_button or self.decel_button) and
               not (brake_restricted and (ret.brakePressed or ret.regenBraking)))
    self.mads_standby_frames = min(self.mads_standby_frames + 1, MADS_ON_STANDBY_FRAMES) if standby else 0

    self.prev_main_on_lkas_button = self.main_on_lkas_button
    if not on_pressed:
      self.main_on_lkas_button = False
    elif not self.main_on_button and self.mads_standby_frames >= MADS_ON_STANDBY_FRAMES:
      # The shared MADS handler toggles on a pressed LKAS event. Emit once, only
      # from settled disabled standby, then wait for feedback before another request.
      self.main_on_lkas_button = True
      self.mads_standby_frames = 0
    self.main_on_button = on_pressed
    return create_button_events(self.main_on_lkas_button, self.prev_main_on_lkas_button, {True: ButtonType.lkas})

  def update_mads(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    cp = can_parsers[Bus.pt]

    # Both longitudinal modes: the TJA-press cleanup and the white-wheel HUD gate read it.
    self.mrcc_armed_raw = cp.vl["PEDALS"]["ACC_OFF"] == 1 or cp.vl["PEDALS"]["ACC_ACTIVE"] == 1

    self.prev_mrcc_button = self.mrcc_button
    self.prev_tja_button = self.tja_button
    btns = cp.vl["CRZ_BTNS"]
    # BIT1 is active-low: a 0 on the bus-0 parser is the wheel's MRCC master press. Gated on
    # the declaration (an undeclared wheel's idle level is unknown) and held unpressed until
    # the wheel's first frame: parser zeros before it would decode as a phantom press.
    if self.CP_SP.flags & MazdaFlagsSP.TJA_BUTTON:
      self.crz_btns_seen = self.crz_btns_seen or len(cp.vl_all["CRZ_BTNS"]["BIT1"]) > 0
      self.mrcc_button = int(btns["BIT1"] == 0) if self.crz_btns_seen else 0
    else:
      self.mrcc_button = 0
    # Only a car declared to have the physical TJA button reports it as the MADS switch.
    self.tja_button = int(btns["TJA_BUTTON"] == 1) if self.CP_SP.flags & MazdaFlagsSP.TJA_BUTTON else 0
