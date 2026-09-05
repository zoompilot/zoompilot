"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

CarInterface.get_params: what follows the EPS, what stays keyed on the model, and the
platform admission check in the controller.
"""
import pytest

from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.tests.conftest import DBC_NAME, car_params, car_params_sp
from opendbc.car.mazda.values import CAR, DBC, LKAS_LIMITS, STEER_TO_ZERO_EPS_FW, MazdaFlags, MazdaSafetyFlags

Ecu = structs.CarParams.Ecu

# The steer-to-zero EPS a swap donates, and a stock pre-2022 CX-5 EPS for contrast
SWAPPED_EPS_FW = sorted(STEER_TO_ZERO_EPS_FW)[0]
STOCK_CX5_EPS_FW = b'K319-3210X-A-00' + b'\x00' * 9

MIN_STEER_SPEED_STOCK_EPS = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS


def eps_fw(version: bytes) -> list[structs.CarParams.CarFw]:
  fw = structs.CarParams.CarFw()
  fw.ecu = Ecu.eps
  fw.address = 0x730
  fw.subAddress = 0
  fw.fwVersion = version
  return [fw]


class TestMazdaEpsSwap:
  """A 2022+ CX-5 EPS swapped into an older Mazda brings the EPS-derived behavior with it.

  Pre-2022 Mazdas are dashcam only because their EPS locks steering out after ~5 s hands-off
  and below 45 kph. That lockout lives in the EPS, so the swap lifts it. Everything keyed on
  the radar, camera or vehicle dynamics must stay keyed on the model.
  """

  def test_stock_older_mazda_is_dashcam_only(self):
    CP = car_params(CAR.MAZDA_CX5, car_fw=eps_fw(STOCK_CX5_EPS_FW))
    assert CP.dashcamOnly
    assert CP.minSteerSpeed == pytest.approx(MIN_STEER_SPEED_STOCK_EPS, abs=5e-8)
    assert CP.steerActuatorDelay == pytest.approx(0.1, abs=5e-8)

  def test_swapped_eps_lifts_dashcam_and_the_speed_floor(self):
    CP = car_params(CAR.MAZDA_CX5, car_fw=eps_fw(SWAPPED_EPS_FW))
    assert not CP.dashcamOnly
    assert CP.minSteerSpeed == 0
    assert CP.steerActuatorDelay == pytest.approx(0.14, abs=5e-8)

  def test_swapped_eps_unlocks_longitudinal(self):
    # alpha long follows the EPS: the swap is what lets the port hold the wheel through a stop
    CP = car_params(CAR.MAZDA_CX5, car_fw=eps_fw(SWAPPED_EPS_FW), alpha_long=True)
    assert CP.alphaLongitudinalAvailable
    assert CP.openpilotLongitudinalControl
    assert not car_params(CAR.MAZDA_CX5, alpha_long=True).alphaLongitudinalAvailable

  def test_swapped_eps_keeps_the_real_vehicle_specs(self):
    # EPS detection must not replace the chassis-specific physical parameters.
    swapped = car_params(CAR.MAZDA_CX5, car_fw=eps_fw(SWAPPED_EPS_FW))
    cx5_2022 = car_params(CAR.MAZDA_CX5_2022)
    assert swapped.mass != cx5_2022.mass
    assert swapped.steerRatio != cx5_2022.steerRatio
    assert swapped.tireStiffnessFactor != cx5_2022.tireStiffnessFactor

  def test_supported_platforms_are_unchanged(self):
    cx5_2022 = car_params(CAR.MAZDA_CX5_2022)
    assert not cx5_2022.dashcamOnly
    assert cx5_2022.minSteerSpeed == 0
    assert cx5_2022.steerActuatorDelay == pytest.approx(0.14, abs=5e-8)
    assert cx5_2022.alphaLongitudinalAvailable

    # the CX-9 2021 is supported without the CX-5 EPS, so it keeps the 45 kph floor
    cx9_2021 = car_params(CAR.MAZDA_CX9_2021)
    assert not cx9_2021.dashcamOnly
    assert cx9_2021.minSteerSpeed == pytest.approx(MIN_STEER_SPEED_STOCK_EPS, abs=5e-8)
    assert cx9_2021.steerActuatorDelay == pytest.approx(0.1, abs=5e-8)
    assert not cx9_2021.alphaLongitudinalAvailable

  @pytest.mark.parametrize("candidate", list(CAR))
  @pytest.mark.parametrize("swapped", [False, True], ids=["stock", "swapped_eps"])
  def test_alpha_long_follows_the_eps(self, candidate, swapped):
    # alpha long is offered wherever the 2022 CX-5 EPS is: the CX-5 2022 itself and any
    # Mazda with that EPS swapped in. A stock older EPS cuts lateral below 45 kph, so
    # stop-and-go would run unsteered; those cars are not offered it. Neither is a platform
    # whose DBC has no radar bus (the pre-2021 CX-9), since the port has never seen its radar
    car_fw = eps_fw(SWAPPED_EPS_FW) if swapped else None
    CP = car_params(candidate, car_fw=car_fw, alpha_long=True)
    has_radar_dbc = Bus.radar in DBC[candidate]
    expected = (candidate == CAR.MAZDA_CX5_2022 or swapped) and has_radar_dbc
    assert CP.alphaLongitudinalAvailable == expected
    assert CP.openpilotLongitudinalControl == expected
    assert bool(CP.safetyConfigs[0].safetyParam & MazdaSafetyFlags.LONG.value) == expected

  def test_stock_long_still_reads_the_radar_tracks(self):
    assert not car_params(CAR.MAZDA_CX9_2021, alpha_long=True).radarUnavailable

  @pytest.mark.parametrize("candidate, car_fw, alpha_long, expected", [
    (CAR.MAZDA_CX5_2022, None, False, True),
    (CAR.MAZDA_CX5_2022, None, True, True),
    (CAR.MAZDA_CX5, eps_fw(SWAPPED_EPS_FW), False, True),
    (CAR.MAZDA_CX5, eps_fw(STOCK_CX5_EPS_FW), False, False),
    (CAR.MAZDA_CX5, None, False, False),
    (CAR.MAZDA_CX9_2021, None, True, False),
  ])
  def test_safety_param_follows_the_eps(self, candidate, car_fw, alpha_long, expected):
    # the panda's torque envelope is selected by MazdaSafetyFlags.STEER_TO_ZERO_EPS, and it must
    # travel with the same EPS detection that selects the controller's 1200/12/12 tune
    bit = MazdaSafetyFlags.STEER_TO_ZERO_EPS.value
    CP = car_params(candidate, car_fw=car_fw, alpha_long=alpha_long)
    assert bool(CP.safetyConfigs[0].safetyParam & bit) == expected
    assert bool(CP.flags & MazdaFlags.STEER_TO_ZERO_EPS) == expected
    # the same proxy the controller tune keys on
    assert (CP.minSteerSpeed == 0) == expected
    # longitudinal keeps its own bit
    assert bool(CP.safetyConfigs[0].safetyParam & MazdaSafetyFlags.LONG.value) == CP.openpilotLongitudinalControl

  @pytest.mark.parametrize("candidate", [CAR.MAZDA_CX5, CAR.MAZDA_CX9, CAR.MAZDA_3, CAR.MAZDA_6])
  def test_docs_are_generated_without_firmware(self, candidate):
    # car_fw is empty when building CARS.md, so the docs must keep advertising dashcam mode
    from opendbc.car import gen_empty_fingerprint
    from opendbc.car.mazda.interface import CarInterface
    CP = CarInterface.get_params(candidate, gen_empty_fingerprint(), [], alpha_long=False, is_release=False, docs=True)
    assert CP.dashcamOnly


def test_non_gen1_platform_refused_at_admission():
  # one init-time check instead of per-frame guards in the message builders, which every
  # frame layout in mazdacan assumes; the fall-throughs used to emit an all-zero CAM_LKAS
  # and return None from the button builder, straight into can_sends
  CP = car_params(CAR.MAZDA_CX5_2022)
  CP_SP = car_params_sp(CP)
  CP.flags = 0
  with pytest.raises(NotImplementedError):
    CarController({Bus.pt: DBC_NAME}, CP, CP_SP)
