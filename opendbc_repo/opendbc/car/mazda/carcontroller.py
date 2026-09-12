from collections import deque

import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, rate_limit, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.longitudinal import BREAKAWAY_FRAMES, AdvertisedLead, StandstillHold
from opendbc.car.mazda.radar_session import RadarSessionManager, RadarSessionState
from opendbc.car.mazda.values import CarControllerParams, Buttons, MazdaFlags

from opendbc.sunnypilot.car.mazda.icbm import IntelligentCruiseButtonManagementInterface
from opendbc.sunnypilot.car.stock_ecu import StockEcuState

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

# Send synthetic radar frames to both consumers; panda does not forward locally generated frames.
LONG_BUSES = (0, 2)


class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    if not CP.flags & MazdaFlags.GEN1:
      # mazdacan message builders require GEN1 frame layouts.
      raise NotImplementedError(f"unsupported platform: {CP.carFingerprint}")
    self.params = CarControllerParams(CP)
    # values.py selects the measured EPS envelope from the hardware mask; the speed-dependent
    # scale and the non-delivery latch belong to the steer-to-zero firmware alone.
    self.eps_2022 = bool(CP.flags & MazdaFlags.EPS_HW)
    self.steer_to_zero = bool(CP.flags & MazdaFlags.STEER_TO_ZERO_EPS)
    self.g46l = bool(CP.flags & MazdaFlags.G46L_RADAR)
    self.apply_torque_last = 0
    self.driver_torque_samples: deque[float] = deque(maxlen=self.params.STEER_DRIVER_SAMPLES if self.eps_2022 else 1)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.brake_counter = 0
    self.stop_and_go = StandstillHold()
    self.lead_adv = AdvertisedLead()
    self.long_counter = 0
    self.radar_counter = 0
    self.radar_session = RadarSessionManager(moving_takeover=bool(CP.flags & MazdaFlags.MOVING_TAKEOVER))
    self.accel_last = 0.
    self.release_ramp = None
    self.breakaway_frames = 0
    # The camera's own TJA/CTS is pressed off on its bus whenever it is armed, per arming
    # episode: the camera re-arms on the driver's own TJA press (that press is also the MADS
    # switch on declared cars) and drops its arm by itself at times.
    self.tja_press_count = 0
    self.tja_press_frame: int | None = None
    self.tja_episode_alerted = False

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    apply_torque = 0

    # The measured EPS uses a speed-dependent STEER_MAX.
    if self.eps_2022:
      steer_max = round(float(np.interp(CS.out.vEgoRaw, self.params.STEER_MAX_LOOKUP[0],
                                         self.params.STEER_MAX_LOOKUP[1])))
    else:
      steer_max = self.params.STEER_MAX

    self.driver_torque_samples.append(CS.out.steeringTorque)
    if CS.lkas_rejected:
      # The panda reports every 0x243 it refused back on the can stream (src 192). A rejection
      # zeroes its rate-limit reference, so a controller that keeps ramping is refused on every
      # later frame and the EPS loses its stream: LKAS_FAULT about 0.6 s in, the camera fault
      # 5.3 s after that, neither clearing before the next ignition cycle. Only a command
      # within one step of zero is accepted next, so the ramp restarts there. A nonzero stream
      # refused because the panda's lateral is not armed becomes a one-step sawtooth instead of
      # a blind ramp to the rail; the camera's own 0x243 is forwarded meanwhile. See
      # docs/zoompilot/mazda-lateral.md, "LKAS_FAULT".
      self.apply_torque_last = 0

    if CC.latActive:
      # calculate steer and also set limits due to driver torque
      new_torque = int(round(CC.actuators.torque * steer_max))

      # Clamp to applied EPS authority so controlsd can detect saturation. Keep this separate
      # from steer_max because torque parameter scaling depends on steer_max.
      if self.eps_2022:
        eps_ceiling = round(float(np.interp(CS.out.vEgoRaw, self.params.EPS_CEILING_LOOKUP[0],
                                            self.params.EPS_CEILING_LOOKUP[1])))
        new_torque = int(np.clip(new_torque, -eps_ceiling, eps_ceiling))

      # Use the worst sample plus margin to stay inside panda's fresher driver-torque envelope.
      margin = self.params.STEER_DRIVER_MARGIN if self.eps_2022 else 0
      if new_torque >= 0:
        driver_torque = min(self.driver_torque_samples) - margin
      else:
        driver_torque = max(self.driver_torque_samples) + margin

      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      driver_torque, self.params, steer_max)

    # Stop requesting torque while carstate says the EPS will not take it: after the
    # non-delivery latch, or through its first engagement of the cycle. Recovery ramps from zero.
    if self.steer_to_zero and (CS.steer_undelivered or CS.steer_first_engage_hold):
      apply_torque = 0

    # Do not cancel a stock MRCC engagement while the stock radar still owns the bus.
    stock_mrcc_owns_cruise = self.CP.openpilotLongitudinalControl and not CS.radar_was_silenced
    if CC.cruiseControl.cancel and not stock_mrcc_owns_cruise:
      # If brake is pressed, let us wait >70ms before trying to disable crz to avoid
      # a race condition with the stock system, where the second cancel from openpilot
      # will disable the crz 'main on'. crz ctrl msg runs at 50hz. 70ms allows us to
      # read 3 messages and most likely sync state before we attempt cancel.
      self.brake_counter = self.brake_counter + 1
      if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
        # Cancel Stock ACC if it's enabled while OP is disengaged
        # Send at a rate of 10hz until we sync with stock ACC state
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CANCEL))
    else:
      self.brake_counter = 0
      if self.resume_requested(CC) and self.frame % 5 == 0:
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))

    self.apply_torque_last = apply_torque

    if self.CP.openpilotLongitudinalControl:
      can_sends.extend(self.update_longitudinal(CC, CC_SP, CS))

    can_sends.extend(self.update_camera_tja(CC, CS))

    # send HUD alerts
    if self.frame % 50 == 0:
      ldw = CC.hudControl.visualAlert == VisualAlert.ldw
      steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
      # TODO: find a way to silence audible warnings so we can add more hud alerts
      steer_required = steer_required and CS.lkas_allowed_speed
      can_sends.append(mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, ldw, steer_required))

    # send steering command
    can_sends.append(mazdacan.create_steering_control(self.packer, self.CP,
                                                      self.frame, apply_torque, CS.cam_lkas))

    # Suppress ICBM while cancel or resume is active to avoid competing button frames.
    icbm_suppress = CC.cruiseControl.cancel or CC.cruiseControl.resume or CS.cancel_button == 1
    if not icbm_suppress:
      can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CC_SP, CS, self.packer, self.frame, self.last_button_frame))

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = apply_torque / steer_max
    new_actuators.torqueOutputCan = apply_torque
    # Report the command sent on the wire after clipping, holds, slew, and overrides.
    new_actuators.accel = self.accel_last

    self.frame += 1
    return new_actuators, can_sends

  @property
  def stock_ecu_state(self):
    """The stock ECU transition contract (opendbc/sunnypilot/car/stock_ecu.py): card reads this
    one name and nothing brand-specific. NOT_NEEDED under stock longitudinal, as on any brand
    that silences nothing."""
    return self.radar_session.status if self.CP.openpilotLongitudinalControl else StockEcuState.NOT_NEEDED

  def update_camera_tja(self, CC, CS):
    """Press the camera's own TJA/CTS off, on its bus, whenever it is armed.

    The two lane-centering systems must never run at once, and with a panda fitted the camera
    is never needed: the panda drops the camera's 0x243 while openpilot controls, but the camera
    keeps its state and takes the wheel the moment lateral drops (route 00000018--5655da2c1c seg
    15), and a MADS-off press re-arms it so stock TJA engages a few seconds later. So the press
    is not gated on lateral. One CRZ_BTNS with the TJA bit over the wheel's idle pattern, counter
    plus one, on bus 2; the forwarded real stream supplies the release and the camera acts on the
    press edge (tja_cts_route_29). At least one 0x440 period between presses, three per arming
    episode; the episode resets when the camera reads 0, so a driver re-arming it is handled
    again. Not gated on the button declaration: any Mazda with the camera armed gets the same
    press. The press only reaches the camera, so it cannot arm or disarm MRCC in the body.
    """
    can_sends = []
    if CS.stock_tja == 0:
      self.tja_press_count = 0
      self.tja_press_frame = None
      self.tja_episode_alerted = False
    else:
      interval = int(CarControllerParams.TJA_PRESS_INTERVAL_T / DT_CTRL)
      due = self.tja_press_frame is None or self.frame - self.tja_press_frame >= interval
      if due and self.tja_press_count < CarControllerParams.TJA_PRESS_MAX:
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.TJA, bus=2))
        self.tja_press_count += 1
        self.tja_press_frame = self.frame
      elif due and CC.latActive and not self.tja_episode_alerted:
        # The camera did not clear while openpilot steers: keep steering (its command is
        # blocked) and tell the driver once. carstate turns this into the one-shot stockLkas
        # pulse. With lateral off the camera steering is stock behaviour, nothing to warn about.
        CS.stock_cts_stuck = True
        self.tja_episode_alerted = True
    return can_sends

  def resume_requested(self, CC) -> bool:
    """The resume button belongs to the stock-longitudinal path alone. Under openpilot longitudinal
    the hold is released in-protocol (stop bits drop, RESUME_UNLATCHING pulses, the command ramps),
    which is what stock MRCC does, and ICBM owns CRZ_BTNS. Toyota, Honda and Hyundai gate their
    resume button the same way.
    """
    return not self.CP.openpilotLongitudinalControl and CC.cruiseControl.resume

  def update_longitudinal(self, CC, CC_SP, CS):
    can_sends = []

    # Start takeover only after the FSC boot check and any stock engagement have ended.
    stock_radar_alive = CS.stock_radar_alive
    stock_engaged = stock_radar_alive and CS.cruise_enabled
    bus_healthy = CS.radar_bus_healthy
    # carstate's guard from the previous frame is the ownership the engagement path sees
    session_state = self.radar_session.update(CS.fsc_settled, stock_radar_alive, CC_SP.stockEcuHandBack,
                                              standstill=CS.out.standstill,
                                              session_refused=CS.radar_session_refused,
                                              stock_radar_gone=CS.stock_radar_gone,
                                              bus_healthy=bus_healthy and CS.out.canValid,
                                              session_response=CS.radar_session_response, frame=self.frame,
                                              stock_engaged=stock_engaged, owned=CS.radar_owned)
    # Continue synthetic radar frames through hand-back to avoid a camera-visible gap.
    radar_master = self.radar_session.replacement_active
    CS.radar_control_active = radar_master and session_state == RadarSessionState.SILENCED
    CS.radar_restore_failed = self.radar_session.handback_failed
    CS.radar_handback_active = session_state == RadarSessionState.HANDBACK or self.radar_session.handback_completed

    if self.radar_session.diagnostic_message is not None:
      can_sends.append(self.radar_session.diagnostic_message)

    stopping = CC.actuators.longControlState == LongCtrlState.stopping
    # Engaged bits follow CC.enabled. Gas is an override, not a disengagement.
    control_ready = CS.radar_control_active and bus_healthy and CS.out.canValid
    long_engaged = CC.enabled and control_ready
    long_active = CC.longActive and control_ready
    sm = self.stop_and_go
    sm.update(long_engaged, stopping, CS.out.standstill, CC.actuators.accel, CS.brake_hold,
              gas_pressed=CS.out.gasPressed)
    # Lead advertisement represents perception and is independent of engagement.
    self.lead_adv.update(CC.hudControl.leadVisible, CC_SP.leadOne.dRel,
                         CC_SP.leadOne.vRel, sm.holding)

    if sm.just_released:
      # Never-latched stops relax in one frame; latched holds ramp from the relaxed command.
      self.release_ramp = CarControllerParams.ACCEL_HOLD_LATCHED if sm.latched_release else \
                          CarControllerParams.ACCEL_RELEASE_BAND
    elif sm.holding or not long_active:
      # Re-holds and driver overrides terminate the release ramp.
      self.release_ramp = None

    accel = 0.
    if long_active:
      accel = float(np.clip(CC.actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      # Continue a bounded release ramp while stopped because the plan may not break static hold.
      if self.release_ramp is None or not CS.out.standstill:
        self.breakaway_frames = 0
      else:
        self.breakaway_frames += 1
      breakaway = CS.out.standstill and self.breakaway_frames <= BREAKAWAY_FRAMES
      # Bound breakaway by stock authority and by the plan-relative margin.
      ramp_ceiling = max(accel, min(CarControllerParams.ACCEL_BREAKAWAY_MAX,
                                    accel + CarControllerParams.ACCEL_BREAKAWAY_OVERSHOOT))
      if self.release_ramp is not None and (self.release_ramp < accel or breakaway):
        # The release ramp owns the command until it reaches the plan. Body-latched holds remain
        # at the relaxed command until GEAR.BRAKE_HOLD clears.
        accel = self.release_ramp
        if not (sm.latched_release and CS.brake_hold):
          # Follow a falling plan ceiling at the winddown limit.
          self.release_ramp = max(min(self.release_ramp + CarControllerParams.ACCEL_RELEASE_RAMP * DT_CTRL, ramp_ceiling),
                                  self.release_ramp + CarControllerParams.ACCEL_WINDDOWN_LIMIT)
      else:
        self.release_ramp = None
        # Track overrides in accel_last so control resumes through the slew limiter.
        accel = rate_limit(accel, self.accel_last, CarControllerParams.ACCEL_WINDDOWN_LIMIT,
                           CarControllerParams.ACCEL_WINDUP_LIMIT)
        if accel > 0.:
          # Shape positive commands to stock MRCC's ceiling and build rate at this speed.
          v_ego = CS.out.vEgoRaw
          ceiling = float(np.interp(v_ego, CarControllerParams.ACCEL_CEILING_BP, CarControllerParams.ACCEL_CEILING_V))
          build = float(np.interp(v_ego, CarControllerParams.ACCEL_BUILD_BP, CarControllerParams.ACCEL_BUILD_V)) * DT_CTRL
          accel = min(accel, ceiling, max(self.accel_last, 0.) + build)
        if self.accel_last > 0. and CC.actuators.accel >= 0.:
          # Lift the throttle at stock's rate; a brake request bypasses this above.
          accel = max(accel, self.accel_last + CarControllerParams.ACCEL_LIFT_LIMIT * DT_CTRL)
      if sm.car_has_hold:
        # Stop requesting brake hold after the body ECU takes ownership.
        accel = CarControllerParams.ACCEL_HOLD_LATCHED
      elif sm.holding:
        # Freeze the braking command while STOPPING is asserted.
        accel = min(accel, 0.) if CC.actuators.accel <= 0. else min(self.accel_last, 0.)
      if sm.resume_unlatching:
        # Bound the latched release pulse to stock's command range.
        accel = min(max(accel, CarControllerParams.ACCEL_HOLD_LATCHED),
                    CarControllerParams.ACCEL_RESUME_PULSE_MAX)
    self.accel_last = accel

    if radar_master and self.frame % CarControllerParams.RADAR_STEP == 0:
      for bus in LONG_BUSES:
        can_sends.extend(mazdacan.create_radar_frames(bus, self.radar_counter, self.lead_adv.lead, g46l=self.g46l))
      self.radar_counter += 1

    if radar_master and self.frame % CarControllerParams.LONG_STEP == 0:
      # Preserve the driver's main-switch state through restoration without advertising
      # openpilot engagement. CarState's public availability is already revoked.
      acc_available = CS.cruise_available if session_state == RadarSessionState.HANDBACK and bus_healthy else \
                      CS.out.cruiseState.available and control_ready
      # Mirror the driver's distance setting; stock defaults to gap 2.
      gap = (int(CC.hudControl.leadDistanceBars) or 2) if (long_engaged or acc_available) else 0
      acc_active_2 = sm.acc_active_2 if long_engaged else False
      for bus in LONG_BUSES:
        can_sends.append(mazdacan.create_acc_command(self.packer, bus, self.long_counter, accel,
                                                     long_active=long_engaged, acc_available=acc_available,
                                                     brake_pressed=CS.out.brakePressed,
                                                     stopping=sm.stop_bits, resume_unlatching=sm.resume_unlatching))
        can_sends.append(mazdacan.create_crz_ctrl(self.packer, bus, long_engaged, acc_available, gap,
                                                  self.lead_adv.has_lead, self.lead_adv.ctrl_phase,
                                                  acc_active_2))
      self.long_counter += 1

    return can_sends
