from collections import deque

import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.longitudinal import AccelShaper, LeadIcon, StandstillHold
from opendbc.car.mazda.radar_session import RadarSessionManager, RadarSessionState
from opendbc.car.mazda.values import CarControllerParams, Buttons, MazdaFlags

from opendbc.sunnypilot.car.mazda.icbm import IntelligentCruiseButtonManagementInterface
from opendbc.sunnypilot.car.mazda.mads import MadsCarController
from opendbc.sunnypilot.car.stock_ecu import StockEcuState

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

# Send synthetic radar frames to both consumers; panda does not forward locally generated frames.
LONG_BUSES = (0, 2)
# The alert frame's cadence.
HUD_ALERT_FRAMES = 50
CANCEL_SETTLE_FRAMES = int(CarControllerParams.CANCEL_SETTLE_T / DT_CTRL)
# The dash hands-on-wheel frame waits this long after lateral or cruise comes on: the driver is
# on the button, and the EPS-standby banner (1.8 s) can outlive the block that raised it.
DASH_STEER_WARNING_QUIET_FRAMES = int(2.0 / DT_CTRL)


class DashSteerWarning:
  """When the steerRequired alert also reaches the car's own dash.

  The device alert is the primary channel and is never touched here. The dash frame (the
  hands-on-wheel code in CAM_LANEINFO, see mazdacan.create_alert_command) mirrors the alert
  only while openpilot is steering, which keeps it off the dash at a standstill and through an
  EPS block; never inside the quiet window after lateral or cruise comes on; and, once the
  driver cancels or touches the brake while the alert is up, not again until the alert clears.
  The cluster chimes on the frame, so each rule is one chime less at a moment the driver is
  already acting.
  """

  def __init__(self):
    self.lat_active_prev = False
    self.enabled_prev = False
    self.brake_prev = False
    self.quiet_until = 0
    self.held = False

  def update(self, frame: int, steer_required: bool, lat_active: bool, enabled: bool, brake_pressed: bool) -> bool:
    if (lat_active and not self.lat_active_prev) or (enabled and not self.enabled_prev):
      self.quiet_until = frame + DASH_STEER_WARNING_QUIET_FRAMES
    if steer_required and ((self.enabled_prev and not enabled) or brake_pressed != self.brake_prev):
      self.held = True
    if not steer_required:
      self.held = False
    self.lat_active_prev, self.enabled_prev, self.brake_prev = lat_active, enabled, brake_pressed
    return steer_required and lat_active and not self.held and frame >= self.quiet_until


class CarController(CarControllerBase, IntelligentCruiseButtonManagementInterface, MadsCarController):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    MadsCarController.__init__(self)
    self.params = CarControllerParams(CP)
    # The non-delivery latch belongs to the steer-to-zero firmware alone.
    self.steer_to_zero = bool(CP.flags & MazdaFlags.STEER_TO_ZERO_EPS)
    self.g46l = bool(CP.flags & MazdaFlags.G46L_RADAR)
    self.apply_torque_last = 0
    self.driver_torque_samples: deque[float] = deque(maxlen=self.params.STEER_DRIVER_SAMPLES)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.cancel_counter = 0
    self.stop_and_go = StandstillHold()
    self.long_counter = 0
    self.radar_counter = 0
    self.radar_session = RadarSessionManager()
    self.accel_shaper = AccelShaper()
    self.accel_last = 0.
    self.lead_icon = LeadIcon()
    self.dash_steer_warning = DashSteerWarning()
    self.dash_warning_on_bus = False

  def update(self, CC, CC_SP, CS, now_nanos):
    CS.update_mads_feedback(CC, CC_SP)
    can_sends = []

    apply_torque = 0

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
      new_torque = int(round(CC.actuators.torque * self.params.STEER_MAX))

      # Clamp to applied EPS authority so controlsd can detect saturation. Keep this separate
      # from STEER_MAX because the torque parameters are expressed on STEER_MAX.
      eps_ceiling = round(float(np.interp(CS.out.vEgoRaw, self.params.EPS_CEILING_LOOKUP[0],
                                          self.params.EPS_CEILING_LOOKUP[1])))
      new_torque = int(np.clip(new_torque, -eps_ceiling, eps_ceiling))

      # Yield a share of the command to a driver pushing against it.
      driver_opposing = -np.sign(new_torque) * CS.out.steeringTorque
      new_torque = int(round(new_torque * float(np.interp(driver_opposing, self.params.STEER_OVERRIDE_BP,
                                                          self.params.STEER_OVERRIDE_V))))

      # Use the worst sample plus margin to stay inside panda's fresher driver-torque envelope.
      if new_torque >= 0:
        driver_torque = min(self.driver_torque_samples) - self.params.STEER_DRIVER_MARGIN
      else:
        driver_torque = max(self.driver_torque_samples) + self.params.STEER_DRIVER_MARGIN

      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      driver_torque, self.params)

    # Stop requesting torque while carstate says the EPS will not take it: after the
    # non-delivery latch, or through its first engagement of the cycle. Recovery ramps from zero.
    if self.steer_to_zero and (CS.steer_undelivered or CS.steer_first_engage_hold):
      apply_torque = 0

    # Do not cancel a stock MRCC engagement while the stock radar still owns the bus.
    stock_mrcc_owns_cruise = self.CP.openpilotLongitudinalControl and not CS.radar_was_silenced
    if CC.cruiseControl.cancel and not stock_mrcc_owns_cruise:
      # A CANCEL landing with cruise already off is the stock main-off. The car answers its own
      # cancels (brake, the wheel button) 60 to 90 ms after openpilot disengages on them, and the
      # request falls with cruise; upstream waited 70 ms for the brake only. Wait it out for all.
      self.cancel_counter = self.cancel_counter + 1
      if self.frame % 10 == 0 and self.cancel_counter >= CANCEL_SETTLE_FRAMES:
        # Cancel Stock ACC if it's enabled while OP is disengaged
        # Send at a rate of 10hz until we sync with stock ACC state
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CANCEL))
    else:
      self.cancel_counter = 0
      if self.resume_requested(CC) and self.frame % 5 == 0:
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))

    can_sends.extend(self.update_mrcc_cleanup(CC, CC_SP, CS, self.radar_session.handback_active))

    self.apply_torque_last = apply_torque

    if self.CP.openpilotLongitudinalControl:
      can_sends.extend(self.update_longitudinal(CC, CC_SP, CS))

    can_sends.extend(self.update_hud(CC, CC_SP, CS))

    # send steering command
    can_sends.append(mazdacan.create_steering_control(self.packer, self.CP,
                                                      self.frame, apply_torque, CS.cam_lkas))

    # Suppress ICBM while cancel/resume is active or the MRCC cleanup owns CRZ_BTNS:
    # the wheel's press pattern owns the counter stream until release.
    icbm_suppress = (CC.cruiseControl.cancel or CC.cruiseControl.resume or CS.cancel_button == 1 or
                     self.mrcc_undo_pending or CS.tja_button == 1)
    if not icbm_suppress:
      can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CC_SP, CS, self.packer, self.frame, self.last_button_frame))

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
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

  def update_hud(self, CC, CC_SP, CS):
    """The alert frame at 2 Hz, mirroring the camera's own HUD state.

    The hands-on-wheel warning is the steerRequired alert above the LKAS speed floor, as
    DashSteerWarning lets it through. The MADS white wheel rides the same frame.
    """
    visual_alert = CC.hudControl.visualAlert
    steer_required = visual_alert == VisualAlert.steerRequired
    # TODO: find a way to silence audible warnings so we can add more hud alerts
    steer_required = steer_required and CS.lkas_allowed_speed
    steer_required = self.dash_steer_warning.update(self.frame, steer_required, CC.latActive, CC.enabled,
                                                    CS.out.brakePressed)
    white = self.update_white_hud(CC, CC_SP, CS, self.radar_session.handback_active)

    # Preserve the normal 2 Hz cadence; the exception is the immediate withdraw of anything
    # the last frame asserted.
    withdraw_now = (self.mads_white_hud_on_bus and not white) or (self.dash_warning_on_bus and not steer_required)
    if self.frame % HUD_ALERT_FRAMES == 0 or withdraw_now:
      alert = mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, visual_alert == VisualAlert.ldw, steer_required)
      self.dash_warning_on_bus = steer_required
      return [self.white_hud_frame(CS, alert, white)]
    return []

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
                                              frame=self.frame,
                                              stock_engaged=stock_engaged, owned=CS.radar_owned)
    # Continue synthetic radar frames through hand-back to avoid a camera-visible gap.
    radar_master = self.radar_session.replacement_active

    if self.radar_session.diagnostic_message is not None:
      can_sends.append(self.radar_session.diagnostic_message)

    stopping = CC.actuators.longControlState == LongCtrlState.stopping
    # Engaged bits follow CC.enabled. Gas is an override, not a disengagement.
    control_ready = self.radar_session.control_active and bus_healthy and CS.out.canValid
    long_engaged = CC.enabled and control_ready
    long_active = CC.longActive and control_ready
    sm = self.stop_and_go
    sm.update(long_engaged, stopping, CS.out.standstill, CC.actuators.accel, CS.body_hold,
              gas_pressed=CS.out.gasPressed)

    accel = self.accel_shaper.update(long_active, CC.actuators.accel, self.accel_last, sm, CS.out.standstill,
                                     CS.body_hold, CS.out.vEgoRaw)
    self.accel_last = accel
    lead_shown = self.lead_icon.update(CC.hudControl.leadVisible)

    if radar_master and self.frame % CarControllerParams.RADAR_STEP == 0:
      for bus in LONG_BUSES:
        can_sends.extend(mazdacan.create_radar_frames(bus, self.radar_counter, g46l=self.g46l))
      self.radar_counter += 1

    if radar_master and self.frame % CarControllerParams.LONG_STEP == 0:
      # Preserve the driver's main-switch state through restoration without advertising
      # openpilot engagement. CarState's public availability is already revoked.
      acc_available = CS.cruise_available if session_state == RadarSessionState.HANDBACK and bus_healthy else \
                      CS.out.cruiseState.available and control_ready
      # Mirror the driver's distance setting on the dash; stock shows 2 bars by default.
      # leadDistanceBars counts up, 1 closest to 3 farthest, and DISTANCE_SETTING counts down
      # from 4 bars, so the cluster lights 5 - DISTANCE_SETTING bars (stock: the wheel's
      # DISTANCE_LESS raises the raw, 576 segments).
      bars = int(CC.hudControl.leadDistanceBars) or 2
      gap = (5 - bars) if (long_engaged or acc_available) else 0
      acc_active_2 = sm.acc_active_2 if long_engaged else False
      # Show the vision lead on the dash with stock's distance bucket: 3 behind a stopped lead,
      # 2 while following. The tracks stay empty.
      lead_distance = (3 if CS.out.standstill else 2) if lead_shown else 0
      for bus in LONG_BUSES:
        can_sends.append(mazdacan.create_acc_command(self.packer, bus, self.long_counter, accel,
                                                     long_active=long_engaged, acc_available=acc_available,
                                                     brake_pressed=CS.out.brakePressed,
                                                     stopping=sm.stop_bits, resume_unlatching=sm.resume_unlatching))
        can_sends.append(mazdacan.create_crz_ctrl(self.packer, bus, long_engaged, acc_available, gap, acc_active_2,
                                                  lead_distance=lead_distance, hbc_request=CS.hbc_request))
      self.long_counter += 1

    return can_sends
