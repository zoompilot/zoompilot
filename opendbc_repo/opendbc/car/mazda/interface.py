#!/usr/bin/env python3
from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.carlog import carlog
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.radar_interface import RadarInterface
from opendbc.car.mazda.values import DBC, G46L_RADAR_FW, LKAS_LIMITS, STEER_TO_ZERO_EPS_FW, STEER_TO_ZERO_PLATFORMS, \
  TORQUE_TUNES, CarControllerParams, MazdaFlags, MazdaSafetyFlags, WMI, platform_from_vin
from opendbc.car.vin import Vin, is_valid_vin
from opendbc.sunnypilot.car.mazda.values import MazdaFlagsSP


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  def __init__(self, CP, CP_SP):
    CarInterfaceBase.__init__(self, CP, CP_SP)
    # The controller runs the radar session; carstate reads its outcome on the next update.
    self.CS.radar_session = self.CC.radar_session

  @staticmethod
  def configure_torque_tune(candidate, tune, steering_angle_deadzone_deg=0.0):
    # params.toml's tunes are on upstream's STEER_MAX. An override rather than a conversion in
    # _get_params, so sunnypilot's second call (enforced torque, NNLC) converts as well.
    CarInterfaceBase.configure_torque_tune(candidate, tune, steering_angle_deadzone_deg)
    lat_accel_factor, friction = TORQUE_TUNES.get(candidate, (tune.torque.latAccelFactor, tune.torque.friction))
    tune.torque.latAccelFactor = lat_accel_factor * CarControllerParams.TUNE_SCALE
    tune.torque.friction = friction / CarControllerParams.TUNE_SCALE

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "mazda"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mazda)]

    # The G46L is the one radar known never to publish 0x361-0x366 on bus 0: parsing its
    # claimed bus would starve radarTracks behind a parser that never goes valid, so it runs
    # vision-only. Any other radar keeps the platform's word — an unlisted newer revision of
    # a working radar must not silently lose its tracks.
    g46l_radar = any(fw.ecu == 'fwdRadar' and fw.fwVersion.rstrip(b'\x00') in G46L_RADAR_FW for fw in car_fw)
    if g46l_radar:
      ret.flags |= MazdaFlags.G46L_RADAR.value
    ret.radarUnavailable = Bus.radar not in DBC[candidate] or g46l_radar

    # Every gen1 Mazda EPS is the same hardware; only the firmware differs. Steer-to-zero follows
    # the EPS firmware, so a donor-EPS swap carries it and older firmware in a 2022 body loses it.
    # Only an unread EPS (docs, a failed query) falls back to the platform: a forced CX-5 2022 or CX-8
    # fingerprint on an unlisted older EPS then gets the floor and its banner, not a silent latch.
    eps_fw = {fw.fwVersion for fw in car_fw if fw.ecu == 'eps'}
    steer_to_zero = not eps_fw.isdisjoint(STEER_TO_ZERO_EPS_FW) or (not eps_fw and candidate in STEER_TO_ZERO_PLATFORMS)
    # The panda enforces the measured envelope the controller runs on every gen1 EPS.
    ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.EPS_HW.value
    if steer_to_zero:
      ret.flags |= MazdaFlags.STEER_TO_ZERO_EPS.value
    else:
      # Same envelope and tune; only the firmware's floor, latch semantics and alpha long differ.
      ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS
      ret.flags |= MazdaFlags.LEGACY_FW_EPS.value

    # Alpha-long silences the radar and stands in for it, so it needs the radar's dialect,
    # not its tracks: offer it wherever the platform's radar speaks the 2022 family dialect
    # (its DBC claims a radar bus) or the detected radar is the G46L whose own replay exists.
    # The EPS gate stays: a stock older EPS cuts lateral below 45 kph, so stop-and-go would
    # run unsteered.
    ret.alphaLongitudinalAvailable = steer_to_zero and (Bus.radar in DBC[candidate] or g46l_radar)
    ret.openpilotLongitudinalControl = alpha_long and ret.alphaLongitudinalAvailable
    if ret.openpilotLongitudinalControl:
      ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.LONG.value
      # The car owns engagement and preserves its setpoint through radar teardown.
      ret.pcmCruise = True
      ret.radarUnavailable = True
      ret.stopAccel = -1.0  # stock MRCC standstill command, raw -1024
      # Gas responds in 0-0.2 s, the friction brake in 0.75-0.85 s.
      ret.longitudinalActuatorDelay = 0.5

    carlog.debug({"event": "mazdaRadarVerdict", "radarUnavailable": ret.radarUnavailable,
                  "platformClaim": Bus.radar in DBC[candidate], "g46lRadar": g46l_radar, "steerToZeroEps": steer_to_zero})

    if 0x477 in fingerprint[0]:
      ret.flags |= MazdaFlags.HAS_BSM.value

    # lagd starts from this plus 0.2 s; CX-5 drives learn 0.30-0.34 s.
    ret.steerActuatorDelay = 0.14
    ret.steerLimitTimer = 0.8

    CarInterface.configure_torque_tune(candidate, ret.lateralTuning)

    ret.centerToFront = ret.wheelbase * 0.41

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    ret.intelligentCruiseButtonManagementAvailable = True
    # MRCC holds 30 km/h at its floor in either unit mode: an imperial dash shows 19 (18.64 mph).
    ret.zoompilot.minimumSetSpeed = 30. * CV.KPH_TO_MS

    # A carried-forward CarPlatformBundle can disagree with the physical car after a
    # hardware swap or a branch switch without reinstall.
    # Oceania clusters over-read the held cruise speed (MazdaFlagsSP.OCEANIA_CLUSTER).
    if is_valid_vin(stock_cp.carVin) and Vin(stock_cp.carVin).wmi == WMI.OCEANIA_EXPORT:
      ret.flags |= MazdaFlagsSP.OCEANIA_CLUSTER.value

    vin_platform = platform_from_vin(stock_cp.carVin)
    if vin_platform is not None and vin_platform != str(candidate):
      carlog.warning({"event": "platformBundleVinMismatch", "bundle": str(candidate), "vin_platform": vin_platform,
                      "hint": "the selected platform bundle does not match the VIN's platform"})

    return ret

  def update(self, can_packets):
    """Latch the camera's last CAM_LANEINFO payload and staleness: carstate's freshness and the white-wheel HUD gate."""
    # card sends [(t, frames), ...]; the model tests send one bare (t, frames) tuple. CANParser.update takes both.
    if can_packets and not isinstance(can_packets[0], (list, tuple)):
      can_packets = [can_packets]
    raw, received = self.CS.cam_laneinfo_raw, False
    for _t, frames in can_packets:
      for addr, dat, src in frames:
        if addr == 0x440 and src == 2 and len(dat) == 8:
          raw, received = bytes(dat), True
    self.CS.cam_laneinfo_raw = raw
    self.CS.cam_laneinfo_stale_frames = 0 if received else self.CS.cam_laneinfo_stale_frames + 1
    return super().update(can_packets)
