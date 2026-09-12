#!/usr/bin/env python3
from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.carlog import carlog
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.radar_interface import RadarInterface
from opendbc.car.mazda.values import DBC, G46L_RADAR_FW, LKAS_LIMITS, STEER_TO_ZERO_EPS_FW, STEER_TO_ZERO_PLATFORMS, SUPPORTED_PLATFORMS, MazdaFlags, \
  MazdaSafetyFlags, platform_from_vin


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

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
    if steer_to_zero:
      # Select panda's matching torque envelope from the detected EPS.
      ret.flags |= MazdaFlags.STEER_TO_ZERO_EPS.value
      ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.STEER_TO_ZERO_EPS.value
    else:
      # Same envelope and tune; only the firmware's floor, latch semantics and alpha long differ.
      ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS
      ret.flags |= MazdaFlags.LEGACY_FW_EPS.value
      ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.LEGACY_FW_EPS.value

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
      ret.stopAccel = -1.024  # stock MRCC standstill command
      ret.longitudinalActuatorDelay = 0.36  # measured ~0.3 s dead time + ~0.3 s first-order lag

    # Older EPS firmware enforces hands-off and low-speed steering lockouts.
    # Docs mode carries no real EPS firmware, so leave dashcamOnly at the default.
    if not docs:
      ret.dashcamOnly = candidate not in SUPPORTED_PLATFORMS and not steer_to_zero

    carlog.debug({"event": "mazdaRadarVerdict", "radarUnavailable": ret.radarUnavailable,
                  "platformClaim": Bus.radar in DBC[candidate], "g46lRadar": g46l_radar, "steerToZeroEps": steer_to_zero})

    ret.enableBsm = 0x477 in fingerprint[0]

    # Command-to-torque lag measured on the EPS hardware; lagd learns the remaining delay.
    ret.steerActuatorDelay = 0.14
    ret.steerLimitTimer = 0.8

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.centerToFront = ret.wheelbase * 0.41

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    ret.intelligentCruiseButtonManagementAvailable = True

    # A carried-forward CarPlatformBundle can disagree with the physical car after a
    # hardware swap or a branch switch without reinstall.
    vin_platform = platform_from_vin(stock_cp.carVin)
    if vin_platform is not None and vin_platform != str(candidate):
      carlog.warning({"event": "platformBundleVinMismatch", "bundle": str(candidate), "vin_platform": vin_platform,
                      "hint": "the selected platform bundle does not match the VIN's platform"})

    return ret
