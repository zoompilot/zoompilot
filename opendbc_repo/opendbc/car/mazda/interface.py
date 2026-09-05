#!/usr/bin/env python3
from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.radar_interface import RadarInterface
from opendbc.car.mazda.values import CAR, DBC, LKAS_LIMITS, STEER_TO_ZERO_EPS_FW, MazdaFlags, MazdaSafetyFlags


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "mazda"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mazda)]

    ret.radarUnavailable = Bus.radar not in DBC[candidate]

    # Detect the steer-to-zero EPS from firmware so donor-EPS swaps retain its capabilities.
    steer_to_zero = candidate == CAR.MAZDA_CX5_2022 or \
      any(fw.ecu == 'eps' and fw.fwVersion in STEER_TO_ZERO_EPS_FW for fw in car_fw)
    if steer_to_zero:
      # Select panda's matching torque envelope from the detected EPS.
      ret.flags |= MazdaFlags.STEER_TO_ZERO_EPS.value
      ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.STEER_TO_ZERO_EPS.value
    else:
      ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS

    # Offer alpha longitudinal only with the EPS that retains lateral control through a stop.
    ret.alphaLongitudinalAvailable = steer_to_zero and not ret.radarUnavailable
    ret.openpilotLongitudinalControl = alpha_long and ret.alphaLongitudinalAvailable
    if ret.openpilotLongitudinalControl:
      ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.LONG.value
      # The car owns engagement and preserves its setpoint through radar teardown.
      ret.pcmCruise = True
      ret.radarUnavailable = True
      ret.stopAccel = -1.024  # stock MRCC standstill command
      ret.longitudinalActuatorDelay = 0.36  # measured ~0.3 s dead time + ~0.3 s first-order lag

    # Older EPS firmware enforces hands-off and low-speed steering lockouts.
    ret.dashcamOnly = candidate not in (CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021) and not steer_to_zero

    ret.enableBsm = 0x477 in fingerprint[0]

    # Command-to-torque lag follows EPS firmware; lagd learns the remaining delay.
    ret.steerActuatorDelay = 0.14 if steer_to_zero else 0.1
    ret.steerLimitTimer = 0.8

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.centerToFront = ret.wheelbase * 0.41

    return ret

  @staticmethod
  def _get_params_sp(stock_cp: structs.CarParams, ret: structs.CarParamsSP, candidate, fingerprint: dict[int, dict[int, int]],
                     car_fw: list[structs.CarParams.CarFw], alpha_long: bool, is_release_sp: bool, docs: bool) -> structs.CarParamsSP:
    ret.intelligentCruiseButtonManagementAvailable = True

    return ret
