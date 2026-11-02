from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, DT_CTRL, create_button_events, structs, uds
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.mazda.radar_session import RadarSessionManager
from opendbc.car.mazda.values import DBC, LKAS_LIMITS, CarControllerParams, MazdaFlags
from opendbc.sunnypilot.car.mazda.carstate_ext import CarStateExt
from opendbc.sunnypilot.car.mazda.mads import MadsCarState
from opendbc.sunnypilot.car.mazda.values import MazdaFlagsSP

ButtonType = structs.CarState.ButtonEvent.Type

FSC_SETTLE_FRAMES = int(CarControllerParams.FSC_SETTLE_T / DT_CTRL)
STOCK_RADAR_ALIVE_FRAMES = int(CarControllerParams.STOCK_RADAR_ALIVE_T / DT_CTRL)
STOCK_RADAR_GUARD_FRAMES = round(CarControllerParams.STOCK_RADAR_GUARD_T / DT_CTRL)
STOCK_RADAR_ACK_GUARD_FRAMES = round(CarControllerParams.STOCK_RADAR_ACK_GUARD_T / DT_CTRL)
CAM_LANEINFO_FRESH_FRAMES = int(CarControllerParams.CAM_LANEINFO_FRESH_T / DT_CTRL)
LKAS_REARM_FRAMES = round(CarControllerParams.LKAS_REARM_T / DT_CTRL)
LKAS_REARM_FAULT_FRAMES = round(CarControllerParams.LKAS_REARM_FAULT_T / DT_CTRL)
# Bus witnesses: vehicle messages whose silence means the bus is gone, not the radar, over the
# CANParser's own ten-period validity window. {message: (signal, fresh frames)}
MAIN_CAN_WITNESSES = {"PEDALS": ("ACC_ACTIVE", round(0.2 / DT_CTRL)), "ENGINE_DATA": ("SPEED", round(0.1 / DT_CTRL))}
# EPB.HOLD_STATE once the body has taken the cruise standstill hold over.
HOLD_STATE_HOLDING = 3


def body_holds(pt) -> bool:
  """Whether the body ECU owns the standstill hold, from the powertrain parser's values. The body
  takes the hold whether or not Auto Hold is on, and EPB.HOLD_STATE is what stock MRCC relaxes on.
  GEAR.BRAKE_HOLD joins it only with Auto Hold armed; on its own it still means the car is held."""
  return pt["EPB"]["HOLD_STATE"] == HOLD_STATE_HOLDING or pt["GEAR"]["BRAKE_HOLD"] == 1


class CarState(CarStateBase, MadsCarState, CarStateExt):
  def __init__(self, CP, CP_SP):
    CarStateBase.__init__(self, CP, CP_SP)
    MadsCarState.__init__(self, CP, CP_SP)
    CarStateExt.__init__(self, CP, CP_SP)

    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["GEAR"]["GEAR"]

    self.crz_btns_counter = 0
    self.acc_active_last = False
    self.lkas_allowed_speed = False
    self.lkas_blocked = False
    self.lkas_effective = 0
    self.lkas_track_state = False
    # LKAS non-delivery state is used only with the steer-to-zero EPS.
    self.params = CarControllerParams(CP)
    self.steer_undelivered_frames = 0
    self.steer_undelivered = False
    self.steer_undelivered_alert = False
    self.lkas_block_origin_speed: float | None = None
    # The EPS's first engagement of the cycle, under standby at a crawl, can fault while delivering
    # nothing: hold the request until it delivers once, the standby lifts or the car is rolling
    # (docs/zoompilot/mazda-lateral.md, "The first-activation hold").
    self.lkas_delivered = False
    self.steer_first_engage_hold = False
    # Our 0x243 frames the panda refused since the last cycle, reported back on the can stream
    # with src 192 (bus 0 + 0xC0). Zero-torque refusals while disengaged are not counted.
    self.lkas_rejected = 0
    # CAM_SETTINGS intervention bits: the car's own lane-keep setting. The EPS applies nothing while
    # it is off (docs/zoompilot/mazda-lateral.md, "The car's own lane keep switched off").
    self.lkas_setting_on = True
    # Last frame's invalidLkasSetting: with the setting off the EPS applies nothing by design,
    # so the non-delivery latch has nothing to measure and must not hold or alert.
    self.lkas_setting_invalid = False
    # From the setting's return until the EPS has re-armed (update_lkas_arming); card publishes it
    # as CarStateZP.lkasArming, which keeps lateral reading disabled to the driver meanwhile.
    self.lkas_arming = False
    self.lkas_arming_frames = 0
    # The camera's high-beam request (0x440 BIT2), relayed to CRZ_CTRL by the stock radar and, under
    # the takeover, by the controller (docs/zoompilot/mazda-longitudinal.md, "High-beam relay").
    self.hbc_request = False

    self.distance_button = 0
    # The wheel's second distance button (farther). Upstream has one gapAdjustCruise type that
    # cycles the personality one way; card publishes this level so the fork can step the other.
    self.distance_more_button = 0
    self.accel_button = 0
    self.decel_button = 0
    self.cancel_button = 0
    self.resume_button = 0
    self.main_button = 0

    # The car's own cruise state in either mode; the published one adds the radar guard.
    self.cruise_available = False
    self.cruise_enabled = False
    self.cruise_enabled_blocked = True
    self.stock_radar_silent_frames = 0
    self.stock_radar_seen = False
    self.main_can_silent_frames = {name: fresh for name, (_, fresh) in MAIN_CAN_WITNESSES.items()}
    self.radar_bus_healthy = False
    # The controller's radar session, shared by CarInterface: read where the last control frame left it.
    self.radar_session = RadarSessionManager()
    self.radar_owned = False  # the silence guard passed on an owned radar: the engagement gate below
    self.radar_was_silenced = False
    self.main_off_samples = 0
    # The camera's last CAM_LANEINFO payload and its staleness, latched by the interface.
    self.cam_laneinfo_raw: bytes | None = None
    self.cam_laneinfo_stale_frames = CAM_LANEINFO_FRESH_FRAMES
    self.cam_empty_seen = False
    self.radar_session_refused = False
    # The radar acknowledged this episode's programming request: it is in its bootloader.
    self.radar_programming_acked = False
    self.fsc_settled_frames = 0
    # The body ECU owns the standstill brake hold.
    self.body_hold = False

  @property
  def fsc_settled(self) -> bool:
    return self.fsc_settled_frames >= FSC_SETTLE_FRAMES

  @property
  def stock_radar_alive(self) -> bool:
    return self.stock_radar_seen and self.stock_radar_silent_frames < STOCK_RADAR_ALIVE_FRAMES

  @property
  def cam_laneinfo_live(self) -> bool:
    return self.cam_laneinfo_raw is not None and self.cam_laneinfo_stale_frames < CAM_LANEINFO_FRESH_FRAMES

  @property
  def stock_radar_gone(self) -> bool:
    # This silence duration establishes radar ownership rather than a dropped frame. A radar
    # that acknowledged our programming request has left its application, so a shorter one does.
    guard = STOCK_RADAR_ACK_GUARD_FRAMES if self.radar_programming_acked else STOCK_RADAR_GUARD_FRAMES
    return self.radar_bus_healthy and self.stock_radar_silent_frames >= guard

  def update_lkas_arming(self, lkas_setting_invalid: bool) -> None:
    # Armed on the setting's return (self.lkas_setting_invalid is still last frame's) until the EPS
    # lifts its re-arm block LKAS_REARM_T or more from the edge, or delivers torque first
    # (docs/zoompilot/mazda-lateral.md, "The EPS re-arms").
    if lkas_setting_invalid:
      self.lkas_arming = False
    elif self.lkas_setting_invalid:
      self.lkas_arming = True
      self.lkas_arming_frames = 0
    elif self.lkas_arming:
      self.lkas_arming_frames += 1
      rearmed = self.lkas_arming_frames >= LKAS_REARM_FRAMES and not self.lkas_blocked
      if rearmed or self.lkas_effective != 0:
        self.lkas_arming = False

  def update_steer_undelivered(self, v_ego_raw: float, lkas_request: float) -> None:
    lkas_blocked, lkas_track_state = self.lkas_blocked, self.lkas_track_state
    self.lkas_delivered |= self.lkas_effective != 0
    self.steer_first_engage_hold = (not self.lkas_delivered and lkas_blocked and lkas_track_state and
                                    v_ego_raw < self.params.STEER_UNDELIVERED_ALERT_ORIGIN_SPEED)

    # Latch sustained zero LKAS_EFFECTIVE for a real request before the camera faults. It clears
    # with LKAS_BLOCK (a zeroed command shows no delivery) and while the car's lane keep is off.
    if not lkas_blocked or self.lkas_setting_invalid:
      self.steer_undelivered_frames = 0
      self.steer_undelivered = False
      self.steer_undelivered_alert = False
      self.lkas_block_origin_speed = None
    elif self.lkas_block_origin_speed is None:
      self.lkas_block_origin_speed = v_ego_raw

    if lkas_blocked and not self.steer_undelivered:
      if self.lkas_effective == 0 and abs(lkas_request) > self.params.STEER_UNDELIVERED_MIN:
        self.steer_undelivered_frames += 1
        self.steer_undelivered = self.steer_undelivered_frames >= self.params.STEER_UNDELIVERED_FRAMES
      else:
        self.steer_undelivered_frames = 0

    if self.steer_undelivered:
      # Alert only for a sustained road-speed block that began rolling: LKAS_TRACK_STATE and the
      # origin speed both mark the EPS's standby from a stop (docs/zoompilot/mazda-lateral.md).
      self.steer_undelivered_frames += 1
      if (not self.steer_undelivered_alert and not lkas_track_state and
          self.steer_undelivered_frames >= self.params.STEER_UNDELIVERED_FRAMES + self.params.STEER_UNDELIVERED_ALERT_FRAMES and
          v_ego_raw >= self.params.STEER_UNDELIVERED_ALERT_MIN_SPEED and
          self.lkas_block_origin_speed >= self.params.STEER_UNDELIVERED_ALERT_ORIGIN_SPEED):
        self.steer_undelivered_alert = True

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["FL"],
      cp.vl["WHEEL_SPEEDS"]["FR"],
      cp.vl["WHEEL_SPEEDS"]["RL"],
      cp.vl["WHEEL_SPEEDS"]["RR"],
    )

    # Match panda's ENGINE_DATA source for the standstill decision.
    speed_kph = cp.vl["ENGINE_DATA"]["SPEED"]
    ret.standstill = speed_kph <= .1

    can_gear = int(cp.vl["GEAR"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))
    self.body_hold = body_holds(cp.vl)

    ret.genericToggle = bool(cp.vl["BLINK_INFO"]["HIGH_BEAMS"])
    ret.leftBlindspot = cp.vl["BSM"]["LEFT_BS_STATUS"] != 0
    ret.rightBlindspot = cp.vl["BSM"]["RIGHT_BS_STATUS"] != 0
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(40, cp.vl["BLINK_INFO"]["LEFT_BLINK"] == 1,
                                                                      cp.vl["BLINK_INFO"]["RIGHT_BLINK"] == 1)

    ret.steeringAngleDeg = cp.vl["STEER"]["STEER_ANGLE"]
    ret.steeringTorque = cp.vl["STEER_TORQUE"]["STEER_TORQUE_SENSOR"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > LKAS_LIMITS.STEER_THRESHOLD, 5)

    ret.steeringTorqueEps = cp.vl["STEER_TORQUE"]["STEER_TORQUE_MOTOR"]
    ret.steeringRateDeg = cp.vl["STEER_RATE"]["STEER_ANGLE_RATE"]

    ret.brakePressed = cp.vl["PEDALS"]["BRAKE_ON"] == 1

    ret.seatbeltUnlatched = cp.vl["SEATBELT"]["DRIVER_SEATBELT"] == 0
    ret.doorOpen = any([cp.vl["DOORS"]["FL"], cp.vl["DOORS"]["FR"],
                        cp.vl["DOORS"]["BL"], cp.vl["DOORS"]["BR"]])

    # TODO: this should be from 0 - 1.
    ret.gasPressed = cp.vl["ENGINE_DATA"]["PEDAL_GAS"] > 0

    # Either due to low speed or hands off
    lkas_blocked = cp.vl["STEER_RATE"]["LKAS_BLOCK"] == 1

    # LKAS_EFFECTIVE distinguishes partial delivery from a complete block.
    self.lkas_blocked = lkas_blocked
    self.lkas_effective = cp.vl["STEER_RATE"]["LKAS_EFFECTIVE"]
    self.lkas_track_state = cp.vl["STEER_RATE"]["LKAS_TRACK_STATE"] == 1
    # Count the torque requests the panda turned away; a refused zero carries nothing.
    self.lkas_rejected = sum(1 for v in can_parsers[Bus.loopback].vl_all["CAM_LKAS"]["LKAS_REQUEST"] if v != 0)
    if self.CP.flags & MazdaFlags.STEER_TO_ZERO_EPS:
      self.update_steer_undelivered(ret.vEgoRaw, cp.vl["STEER_RATE"]["LKAS_REQUEST"])

    if not self.CP.flags & MazdaFlags.STEER_TO_ZERO_EPS:
      # LKAS is enabled at 52kph going up and disabled at 45kph going down
      # wait for LKAS_BLOCK signal to clear when going up since it lags behind the speed sometimes
      if speed_kph > LKAS_LIMITS.ENABLE_SPEED and not lkas_blocked:
        self.lkas_allowed_speed = True
      elif speed_kph < LKAS_LIMITS.DISABLE_SPEED:
        self.lkas_allowed_speed = False
    else:
      self.lkas_allowed_speed = True

    # Require fresh CAM_LANEINFO because missing and stale parser values can appear settled.
    cam_laneinfo_fresh = self.cam_laneinfo_live

    # 0x21d leaves its idle 0x7f status only while the collision warning is displayed.
    if not self.cam_empty_seen:
      self.cam_empty_seen = len(cp_cam.vl_all["CAM_EMPTY"]["STATUS"]) > 0
    cam_empty = cp_cam.vl["CAM_EMPTY"]
    ped = cp_cam.vl["CAM_PEDESTRIAN"]
    ret.stockFcw = (self.cam_empty_seen and cam_empty["STATUS"] != 0x7F) or \
                   ped["PED_WARNING"] == 1 or ped["BRAKE_WARNING"] == 1

    if self.CP.openpilotLongitudinalControl:
      # After radar teardown cruise state comes from PEDALS. Main follows arming and falls after
      # MAIN_OFF_DEBOUNCE_SAMPLES both-low samples, brake or no brake, counted per sample so the
      # panda's acc_main_on falls on the same one.
      pedals = cp.vl_all["PEDALS"]
      for off, active in zip(pedals["ACC_OFF"], pedals["ACC_ACTIVE"], strict=True):
        if off or active:
          self.cruise_available = True
          self.main_off_samples = 0
        else:
          self.main_off_samples = min(self.main_off_samples + 1, CarControllerParams.MAIN_OFF_DEBOUNCE_SAMPLES)
          if self.main_off_samples >= CarControllerParams.MAIN_OFF_DEBOUNCE_SAMPLES:
            self.cruise_available = False
      self.cruise_enabled = cp.vl["PEDALS"]["ACC_ACTIVE"] == 1

      # Block engagement until stock radar ownership is clear. Radar traffic after a completed
      # teardown is a fault and triggers the alpha-long recovery path.
      self.radar_bus_healthy = True
      for name, (signal, fresh) in MAIN_CAN_WITNESSES.items():
        silent = 0 if len(cp.vl_all[name][signal]) > 0 else min(self.main_can_silent_frames[name] + 1, fresh)
        self.main_can_silent_frames[name] = silent
        self.radar_bus_healthy &= silent < fresh
      if len(cp.vl_all["CRZ_INFO"]["CTR"]) > 0:
        self.stock_radar_seen = True
        self.stock_radar_silent_frames = 0
        # stock traffic after an acknowledgement voids it; the full guard applies again
        self.radar_programming_acked = False
      elif self.radar_bus_healthy:
        self.stock_radar_silent_frames += 1
      else:
        # A missing vehicle bus is not proof of a silenced radar. Restart the observation
        # guard on recovery instead of adopting an outage accumulated while disconnected.
        self.stock_radar_silent_frames = STOCK_RADAR_ALIVE_FRAMES

      # Validate single-frame session refusals; firmware-query ISO-TP fragments and unrelated
      # service replies must not change session state.
      resp = cp.vl_all["RADAR_UDS_RESPONSE"]
      self.radar_session_refused = False
      for pci, sid, sub, nrc in zip(resp["PCI"], resp["SID"], resp["SUB"], resp["NRC"], strict=True):
        if pci == 3 and sid == 0x7F and sub == uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL and nrc != 0x78:
          self.radar_session_refused = True
        elif pci == 6 and sid == uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL + 0x40 and sub == uds.SESSION_TYPE.PROGRAMMING:
          self.radar_programming_acked = True
      # an acknowledgement counts only for the request this episode sent
      self.radar_programming_acked &= self.radar_session.programming_sent
      # Ownership is established by the silence guard and then held on the controller's claim:
      # the radar stays in its diagnostic session through a bus blip, so recovery does not
      # re-run the guard. Stock traffic ends the claim on the alive window either way.
      session = self.radar_session
      silenced = session.control_active and not self.stock_radar_alive and (self.stock_radar_gone or self.radar_was_silenced)
      ret.accFaulted = session.handback_failed or (self.radar_was_silenced and self.stock_radar_alive and not session.handback_active)
      self.radar_was_silenced |= silenced
      self.radar_owned = silenced

      # available follows PEDALS arming from the first frame; the radar guard gates enabled only,
      # and a live stock engagement passes through idle once before it is adopted
      # (docs/zoompilot/mazda-longitudinal.md, "Main is the main switch").
      if not silenced:
        self.cruise_enabled_blocked = True
      elif not self.cruise_enabled:
        self.cruise_enabled_blocked = False

      ret.cruiseState.available = self.cruise_available
      ret.cruiseState.enabled = self.cruise_enabled and not self.cruise_enabled_blocked

      # The FSC teardown gate requires fresh, settled CAM_LANEINFO without ERR_BIT. BIT2 is
      # excluded: it is the auto high-beam arming bit and stays set for as long as HBC is armed.
      laneinfo = cp_cam.vl["CAM_LANEINFO"]
      settled = cam_laneinfo_fresh and not (laneinfo["NO_ERR_BIT"] or laneinfo["ERR_BIT"])
      self.fsc_settled_frames = self.fsc_settled_frames + 1 if settled else 0
    else:
      # CRZ_AVAILABLE represents adaptive-cruise availability, not the main switch.
      self.cruise_available = cp.vl["CRZ_CTRL"]["CRZ_AVAILABLE"] == 1
      self.cruise_enabled = cp.vl["CRZ_CTRL"]["CRZ_ACTIVE"] == 1
      ret.cruiseState.available = self.cruise_available
      ret.cruiseState.enabled = self.cruise_enabled
    # PEDALS.STANDSTILL means wheels stopped, not ACC hold. Reporting it under openpilot
    # longitudinal would prevent LongControl from leaving its stopping state.
    ret.cruiseState.standstill = cp.vl["PEDALS"]["STANDSTILL"] == 1 and not self.CP.openpilotLongitudinalControl
    # CRZ_SPEED is the held speed; an Oceania cluster displays it over-read, so the dash number the
    # buttons step and ICBM reads is published separately (MazdaFlagsSP.OCEANIA_CLUSTER).
    ret.cruiseState.speed = cp.vl["CRZ_EVENTS"]["CRZ_SPEED"] * CV.KPH_TO_MS
    if self.CP_SP.flags & MazdaFlagsSP.OCEANIA_CLUSTER and ret.cruiseState.speed > 0:
      ret.cruiseState.speedCluster = (cp.vl["CRZ_EVENTS"]["CRZ_SPEED"] / 0.98 + 1.) * CV.KPH_TO_MS

    # Stock LKAS must be switched on: the EPS applies no LKAS torque otherwise. Off is LANE_LINES 0
    # (the LAS switch) or both CAM_SETTINGS intervention bits clear; a car that never sends
    # CAM_SETTINGS reads on (docs/zoompilot/mazda-lateral.md). A faulted camera also sends LANE_LINES 0,
    # with its error bits set; that is the camera fault (steerFaultPermanent), not the switch.
    if len(cp_cam.vl_all["CAM_SETTINGS"]["LKAS_INERVENTION_ON1"]) > 0:
      self.lkas_setting_on = any(cp_cam.vl["CAM_SETTINGS"][s] for s in ("LKAS_INERVENTION_ON1", "ILKAS_NTERVENTION_ON2"))
    camera_faulted = cp_cam.vl["CAM_LANEINFO"]["ERR_BIT"] == 1 or cp_cam.vl["CAM_LKAS"]["ERR_BIT_1"] == 1
    las_off = cam_laneinfo_fresh and cp_cam.vl["CAM_LANEINFO"]["LANE_LINES"] == 0 and not camera_faulted
    ret.invalidLkasSetting = las_off or not self.lkas_setting_on
    self.update_lkas_arming(ret.invalidLkasSetting)
    self.lkas_setting_invalid = ret.invalidLkasSetting

    if ret.cruiseState.enabled:
      if not self.lkas_allowed_speed and self.acc_active_last:
        self.low_speed_alert = True
      else:
        self.low_speed_alert = False
    ret.lowSpeedAlert = self.low_speed_alert

    # Check if LKAS is disabled due to lack of driver torque when all other states indicate
    # it should be enabled (steer lockout). Don't warn until we actually get lkas active
    # and lose it again, i.e, after initial lkas activation
    if not self.CP.flags & MazdaFlags.STEER_TO_ZERO_EPS:
      ret.steerFaultTemporary = self.lkas_allowed_speed and lkas_blocked
    else:
      # Report only sustained road-speed zero delivery after the command has been suppressed.
      ret.steerFaultTemporary = self.steer_undelivered_alert
    # The re-arm's block is expected and already shown to the driver; a soft disable over it would
    # cost MADS its lateral on the older EPS just as it comes back.
    if self.lkas_arming and self.lkas_arming_frames < LKAS_REARM_FAULT_FRAMES:
      ret.steerFaultTemporary = False

    self.acc_active_last = ret.cruiseState.enabled

    btns = cp.vl["CRZ_BTNS"]
    self.crz_btns_counter = btns["CTR"]

    # camera signals
    self.cam_lkas = cp_cam.vl["CAM_LKAS"]
    self.cam_laneinfo = cp_cam.vl["CAM_LANEINFO"]
    ret.steerFaultPermanent = cp_cam.vl["CAM_LKAS"]["ERR_BIT_1"] == 1
    self.hbc_request = cam_laneinfo_fresh and self.cam_laneinfo["BIT2"] == 1

    # Decode distance, set-speed, resume, cancel, and main-button events.
    prev_distance_button = self.distance_button
    prev_accel_button = self.accel_button
    prev_decel_button = self.decel_button
    prev_cancel_button = self.cancel_button
    prev_resume_button = self.resume_button
    prev_main_button = self.main_button
    self.distance_button = btns["DISTANCE_LESS"]
    self.distance_more_button = btns["DISTANCE_MORE"]
    # SET_P is the wheel's increase button; RES is a distinct resume button.
    self.accel_button = btns["SET_P"]
    self.decel_button = btns["SET_M"]
    # Publish CAN_OFF so ICBM does not transmit over a physical cancel press.
    self.cancel_button = btns["CAN_OFF"]
    self.resume_button = btns["RES"]
    # The MRCC main button sets MODE_X, MODE_Y or both (main-on and the KE's main-off).
    self.main_button = int(btns["MODE_X"] == 1 or btns["MODE_Y"] == 1)
    MadsCarState.update_mads(self, ret, can_parsers)

    ret.buttonEvents = [
      *create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise}),
      *create_button_events(self.accel_button, prev_accel_button, {1: ButtonType.accelCruise}),
      *create_button_events(self.decel_button, prev_decel_button, {1: ButtonType.decelCruise}),
      *create_button_events(self.cancel_button, prev_cancel_button, {1: ButtonType.cancel}),
      *create_button_events(self.resume_button, prev_resume_button, {1: ButtonType.resumeCruise}),
      *create_button_events(self.main_button, prev_main_button, {1: ButtonType.mainCruise}),
      # A held press of this button must freeze ICBM through the cruise button timers in the openpilot tree.
      *create_button_events(self.mrcc_button, self.prev_mrcc_button, {1: ButtonType.mainCruise}),
      *create_button_events(self.tja_button, self.prev_tja_button, {1: ButtonType.lkas}),
      *self.main_on_lkas_events(ret, btns),
    ]

    CarStateExt.update(self, ret, ret_sp, can_parsers)

    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    pt_messages = [
      # A body without the standstill hold may not send this, so it never gates canValid.
      ("EPB", float("nan")),
      # Only cars with the navigation SD card send the map speed limit.
      ("NAV_SPEED_LIMIT", float("nan")),
    ]
    if CP.openpilotLongitudinalControl:
      # Do not require liveness for frames intentionally absent after radar teardown.
      pt_messages.append(("CRZ_INFO", float("nan")))
      pt_messages.append(("RADAR_UDS_RESPONSE", float("nan")))
    cam_messages = [
      # Read these optional camera messages without making them part of canValid.
      ("CAM_LANEINFO", float("nan")),
      ("CAM_SETTINGS", float("nan")),
      ("CAM_TRAFFIC_SIGNS", float("nan")),
      ("CAM_EMPTY", float("nan")),
      ("CAM_PEDESTRIAN", float("nan")),
    ]
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, 2),
      # Our own 0x243 frames the panda refused, reported back with src = bus + 0xC0. Sporadic by
      # nature, so never part of canValid or canTimeout.
      Bus.loopback: CANParser(DBC[CP.carFingerprint][Bus.pt], [("CAM_LKAS", float("nan"))], 192),
    }
