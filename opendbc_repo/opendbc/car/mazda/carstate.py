from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, DT_CTRL, create_button_events, structs, uds
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.mazda.values import DBC, LKAS_LIMITS, CarControllerParams, MazdaFlags
from opendbc.sunnypilot.car.mazda.carstate_ext import CarStateExt
from opendbc.sunnypilot.car.mazda.values import MazdaFlagsSP

ButtonType = structs.CarState.ButtonEvent.Type

FSC_SETTLE_FRAMES = int(CarControllerParams.FSC_SETTLE_T / DT_CTRL)
STOCK_RADAR_ALIVE_FRAMES = int(CarControllerParams.STOCK_RADAR_ALIVE_T / DT_CTRL)
STOCK_RADAR_GUARD_FRAMES = round(CarControllerParams.STOCK_RADAR_GUARD_T / DT_CTRL)
CANCEL_CONTEXT_FRAMES = int(CarControllerParams.CANCEL_CONTEXT_T / DT_CTRL)
CAM_LANEINFO_FRESH_FRAMES = int(CarControllerParams.CAM_LANEINFO_FRESH_T / DT_CTRL)
STOCK_CTS_ALERT_FRAMES = int(CarControllerParams.STOCK_CTS_ALERT_T / DT_CTRL)
# Bus witnesses: independent vehicle messages whose silence says the bus is gone, not the radar.
# Windows at the CANParser's own validity threshold, ten periods; a stricter window would revoke
# radar ownership on a gap the parser still accepts. {message: (signal, fresh frames)}
MAIN_CAN_WITNESSES = {"PEDALS": ("ACC_ACTIVE", round(0.2 / DT_CTRL)), "ENGINE_DATA": ("SPEED", round(0.1 / DT_CTRL))}


class CarState(CarStateBase, CarStateExt):
  def __init__(self, CP, CP_SP):
    CarStateBase.__init__(self, CP, CP_SP)
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
    # The EPS's first engagement of the ignition cycle, asked for torque under standby (block
    # and track) at a crawl, raised LKAS_FAULT 0.3 s in on three drives; it delivered nothing
    # in that window on any start on record. Hold the request until it has delivered once,
    # the standby lifts, or the car is rolling.
    self.lkas_delivered = False
    self.steer_first_engage_hold = False
    # Our 0x243 frames the panda refused since the last cycle, reported back on the can stream
    # with src 192 (bus 0 + 0xC0). Zero-torque refusals while disengaged are not counted.
    self.lkas_rejected = 0
    self.lkas_fault = False
    # The camera's own TJA/CTS state from its 0x440: 0 off, 2 armed, 3 to 5 steering. Live,
    # never latched; 0 when the camera is stale.
    self.stock_tja = 0
    # Raised by the controller once per arming episode when its camera presses did not clear
    # stock_tja; consumed here into a stockLkas pulse.
    self.stock_cts_stuck = False
    self.stock_cts_alert_frames = 0

    self.distance_button = 0
    self.accel_button = 0
    self.decel_button = 0
    self.cancel_button = 0
    self.resume_button = 0
    self.main_button = 0
    self.tja_button = 0

    self.cruise_available = False
    self.cruise_enabled = False
    self.cruise_enabled_blocked = True
    self.brake_pressed_prev = False
    self.stock_radar_silent_frames = 0
    self.stock_radar_seen = False
    self.main_can_silent_frames = {name: fresh for name, (_, fresh) in MAIN_CAN_WITNESSES.items()}
    self.radar_bus_healthy = False
    self.radar_control_active = False  # controller owns replacement traffic, read on the next update
    self.radar_owned = False  # the silence guard passed on an owned radar: the engagement gate below
    self.radar_restore_failed = False
    self.radar_handback_active = False
    self.radar_was_silenced = False
    self.cancel_context_frames = 0
    self.cam_laneinfo_seen = False
    self.cam_laneinfo_silent_frames = 0
    self.cam_empty_seen = False
    self.radar_session_refused = False
    self.radar_session_response = 0
    self.fsc_settled_frames = 0
    # The body ECU owns the standstill brake hold.
    self.brake_hold = False

  @property
  def fsc_settled(self) -> bool:
    return self.fsc_settled_frames >= FSC_SETTLE_FRAMES

  @property
  def stock_radar_alive(self) -> bool:
    return self.stock_radar_seen and self.stock_radar_silent_frames < STOCK_RADAR_ALIVE_FRAMES

  @property
  def stock_radar_gone(self) -> bool:
    # This silence duration establishes radar ownership rather than a dropped frame.
    return self.radar_bus_healthy and self.stock_radar_silent_frames >= STOCK_RADAR_GUARD_FRAMES

  def update_steer_undelivered(self, v_ego_raw: float, lkas_request: float) -> None:
    lkas_blocked, lkas_track_state = self.lkas_blocked, self.lkas_track_state
    self.lkas_delivered |= self.lkas_effective != 0
    self.steer_first_engage_hold = (not self.lkas_delivered and lkas_blocked and lkas_track_state and
                                    v_ego_raw < self.params.STEER_UNDELIVERED_ALERT_ORIGIN_SPEED)

    # Latch sustained zero LKAS_EFFECTIVE for a real request before the camera faults. Clear
    # with LKAS_BLOCK because a zeroed command provides no delivery signal. Driver torque does
    # not gate entry because torque in the requested direction does not reduce the request.
    if not lkas_blocked:
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
      # Alert only for a sustained road-speed block that began rolling. LKAS_TRACK_STATE
      # identifies normal low-speed standby, which can remain set briefly during a brisk
      # launch; the origin speed catches the standby blocks it does not, the ones carried
      # from a stop through a slow crawl until TRACK_STATE clears with the block still on.
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
    self.brake_hold = cp.vl["GEAR"]["BRAKE_HOLD"] == 1

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
    # The 2022 EPS raises LKAS_FAULT once its 0x243 stream has stopped for about 0.6 s; the
    # camera's own fault follows 5.3 s later and neither clears before the next ignition cycle.
    # Decoded for the log and tooling; the driver-facing fault stays the camera's own.
    self.lkas_fault = cp.vl["STEER_RATE"]["LKAS_FAULT"] == 1
    # The panda refuses every LKA frame while it is not controlling, so a refused zero-torque
    # frame carries nothing the controller needs; count the torque requests it turned away.
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
    if len(cp_cam.vl_all["CAM_LANEINFO"]["LANE_LINES"]) > 0:
      self.cam_laneinfo_seen = True
      self.cam_laneinfo_silent_frames = 0
    else:
      self.cam_laneinfo_silent_frames += 1
    cam_laneinfo_fresh = self.cam_laneinfo_seen and self.cam_laneinfo_silent_frames < CAM_LANEINFO_FRESH_FRAMES

    # 0x21d leaves its idle 0x7f status only while the collision warning is displayed.
    if not self.cam_empty_seen:
      self.cam_empty_seen = len(cp_cam.vl_all["CAM_EMPTY"]["STATUS"]) > 0
    cam_empty = cp_cam.vl["CAM_EMPTY"]
    ped = cp_cam.vl["CAM_PEDESTRIAN"]
    ret.stockFcw = (self.cam_empty_seen and cam_empty["STATUS"] != 0x7F) or \
                   ped["PED_WARNING"] == 1 or ped["BRAKE_WARNING"] == 1

    if self.CP.openpilotLongitudinalControl:
      # After radar teardown, derive cruise state from PEDALS. Hold the previous state through
      # brake-only samples where both cruise bits are transiently low.
      acc_armed = cp.vl["PEDALS"]["ACC_OFF"] == 1
      acc_active = cp.vl["PEDALS"]["ACC_ACTIVE"] == 1
      brake_free = not ret.brakePressed and not self.brake_pressed_prev
      # Retain wheel-cancel context until PEDALS reflects the main-state change.
      if cp.vl["CRZ_BTNS"]["CAN_OFF"] == 1:
        self.cancel_context_frames = CANCEL_CONTEXT_FRAMES
      elif self.cancel_context_frames > 0:
        self.cancel_context_frames -= 1
      if acc_armed or acc_active:
        self.cruise_available = True
      elif brake_free or self.cancel_context_frames > 0:
        self.cruise_available = False
      if acc_armed or acc_active or self.cruise_enabled or brake_free:
        self.cruise_enabled = acc_active

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
      elif self.radar_bus_healthy:
        self.stock_radar_silent_frames += 1
      else:
        # A missing vehicle bus is not proof of a silenced radar. Restart the observation
        # guard on recovery instead of adopting an outage accumulated while disconnected.
        self.stock_radar_silent_frames = STOCK_RADAR_ALIVE_FRAMES

      # Validate single-frame session responses; firmware-query ISO-TP fragments and
      # unrelated service replies must not change session state.
      resp = cp.vl_all["RADAR_UDS_RESPONSE"]
      self.radar_session_refused = False
      self.radar_session_response = 0
      for pci, sid, sub, nrc in zip(resp["PCI"], resp["SID"], resp["SUB"], resp["NRC"], strict=True):
        if pci == 3 and sid == 0x7F and sub == uds.SERVICE_TYPE.DIAGNOSTIC_SESSION_CONTROL and nrc != 0x78:
          self.radar_session_refused = True
        elif pci == 6 and sid == 0x50 and sub in (1, 2):
          self.radar_session_response = int(sub)  # last positive session response this frame
      # Ownership is established by the silence guard and then held on the controller's claim:
      # the radar stays in its diagnostic session through a bus blip, so recovery does not
      # re-run the guard. Stock traffic ends the claim on the alive window either way.
      silenced = self.radar_control_active and not self.stock_radar_alive and (self.stock_radar_gone or self.radar_was_silenced)
      ret.accFaulted = self.radar_restore_failed or (self.radar_was_silenced and self.stock_radar_alive and not self.radar_handback_active)
      self.radar_was_silenced |= silenced
      self.radar_owned = silenced

      # available follows PEDALS arming from the first frame (the panda's acc_main_on reads the
      # same sample); the radar guard gates enabled only, and a live stock engagement is not
      # adopted the instant the guard lifts: it passes through idle once first. See
      # docs/zoompilot/mazda-longitudinal.md, "Main is the main switch".
      if not silenced:
        self.cruise_enabled_blocked = True
      elif not self.cruise_enabled:
        self.cruise_enabled_blocked = False

      ret.cruiseState.available = self.cruise_available
      ret.cruiseState.enabled = self.cruise_enabled and not self.cruise_enabled_blocked

      # The FSC teardown gate requires fresh, settled CAM_LANEINFO without ERR_BIT. BIT2 is
      # excluded because it may remain set for an entire ignition cycle.
      laneinfo = cp_cam.vl["CAM_LANEINFO"]
      settled = cam_laneinfo_fresh and not (laneinfo["NO_ERR_BIT"] or laneinfo["ERR_BIT"])
      self.fsc_settled_frames = self.fsc_settled_frames + 1 if settled else 0
    else:
      # CRZ_AVAILABLE represents adaptive-cruise availability, not the main switch.
      ret.cruiseState.available = cp.vl["CRZ_CTRL"]["CRZ_AVAILABLE"] == 1
      ret.cruiseState.enabled = cp.vl["CRZ_CTRL"]["CRZ_ACTIVE"] == 1
    self.brake_pressed_prev = ret.brakePressed
    # PEDALS.STANDSTILL means wheels stopped, not ACC hold. Reporting it under openpilot
    # longitudinal would prevent LongControl from leaving its stopping state.
    ret.cruiseState.standstill = cp.vl["PEDALS"]["STANDSTILL"] == 1 and not self.CP.openpilotLongitudinalControl
    ret.cruiseState.speed = cp.vl["CRZ_EVENTS"]["CRZ_SPEED"] * CV.KPH_TO_MS

    # Stock LKAS must be active.
    # TODO: is this needed?
    ret.invalidLkasSetting = cam_laneinfo_fresh and cp_cam.vl["CAM_LANEINFO"]["LANE_LINES"] == 0

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

    self.acc_active_last = ret.cruiseState.enabled

    self.crz_btns_counter = cp.vl["CRZ_BTNS"]["CTR"]

    # camera signals
    self.cam_lkas = cp_cam.vl["CAM_LKAS"]
    self.cam_laneinfo = cp_cam.vl["CAM_LANEINFO"]
    ret.steerFaultPermanent = cp_cam.vl["CAM_LKAS"]["ERR_BIT_1"] == 1
    self.stock_tja = int(self.cam_laneinfo["TJA"]) if cam_laneinfo_fresh else 0

    # The camera stayed armed through the controller's presses: one pulse of stockLkas, which
    # the Mazda event hook turns into a one-shot warning. openpilot keeps steering; the panda
    # blocks the camera's own command meanwhile.
    if self.stock_cts_stuck:
      self.stock_cts_stuck = False
      self.stock_cts_alert_frames = STOCK_CTS_ALERT_FRAMES
    ret.stockLkas = self.stock_cts_alert_frames > 0
    self.stock_cts_alert_frames = max(self.stock_cts_alert_frames - 1, 0)

    # Decode distance, set-speed, resume, cancel, and main-button events.
    prev_distance_button = self.distance_button
    prev_accel_button = self.accel_button
    prev_decel_button = self.decel_button
    prev_cancel_button = self.cancel_button
    prev_resume_button = self.resume_button
    prev_main_button = self.main_button
    prev_tja_button = self.tja_button
    self.distance_button = cp.vl["CRZ_BTNS"]["DISTANCE_LESS"]
    # SET_P is the wheel's increase button; RES is a distinct resume button.
    self.accel_button = cp.vl["CRZ_BTNS"]["SET_P"]
    self.decel_button = cp.vl["CRZ_BTNS"]["SET_M"]
    # Publish CAN_OFF so ICBM does not transmit over a physical cancel press.
    self.cancel_button = cp.vl["CRZ_BTNS"]["CAN_OFF"]
    self.resume_button = cp.vl["CRZ_BTNS"]["RES"]
    self.main_button = int(cp.vl["CRZ_BTNS"]["MODE_X"] == 1 and cp.vl["CRZ_BTNS"]["MODE_Y"] == 1)
    # Only a car declared to have the physical TJA button reports it as the MADS switch.
    self.tja_button = int(cp.vl["CRZ_BTNS"]["TJA_BUTTON"] == 1) if self.CP_SP.flags & MazdaFlagsSP.TJA_BUTTON else 0

    ret.buttonEvents = [
      *create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise}),
      *create_button_events(self.accel_button, prev_accel_button, {1: ButtonType.accelCruise}),
      *create_button_events(self.decel_button, prev_decel_button, {1: ButtonType.decelCruise}),
      *create_button_events(self.cancel_button, prev_cancel_button, {1: ButtonType.cancel}),
      *create_button_events(self.resume_button, prev_resume_button, {1: ButtonType.resumeCruise}),
      *create_button_events(self.main_button, prev_main_button, {1: ButtonType.mainCruise}),
      *create_button_events(self.tja_button, prev_tja_button, {1: ButtonType.lkas}),
    ]

    CarStateExt.update(self, ret, ret_sp, can_parsers)

    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    pt_messages = []
    if CP.openpilotLongitudinalControl:
      # Do not require liveness for frames intentionally absent after radar teardown.
      pt_messages.append(("CRZ_INFO", float("nan")))
      pt_messages.append(("RADAR_UDS_RESPONSE", float("nan")))
    cam_messages = [
      # Read these optional camera messages without making them part of canValid.
      ("CAM_LANEINFO", float("nan")),
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
