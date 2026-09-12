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
from opendbc.car.mazda.fingerprints import FW_VERSIONS
from opendbc.car.mazda.interface import CarInterface
from opendbc.car.mazda.tests.conftest import DBC_NAME, car_params, car_params_sp
from opendbc.car.mazda.values import CAR, DBC, G46L_RADAR_FW, LKAS_LIMITS, STEER_TO_ZERO_EPS_FW, STEER_TO_ZERO_PLATFORMS, MazdaFlags, MazdaSafetyFlags

Ecu = structs.CarParams.Ecu

# The steer-to-zero EPS a swap donates, and a stock pre-2022 CX-5 EPS for contrast
SWAPPED_EPS_FW = b'KSD5-3210X-C-00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
STOCK_CX5_EPS_FW = b'K319-3210X-A-00' + b'\x00' * 9
# The 2022 EPS hardware on firmware that keeps the 45 kph floor (THACO CX-5 2023), and a revision listed nowhere
LEGACY_FW_EPS = b'K319-3210X-B-00' + b'\x00' * 9
UNLISTED_EPS_FW = b'KSD5-3210X-D-00' + b'\x00' * 9

MIN_STEER_SPEED_STOCK_EPS = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS


def eps_fw(version: bytes) -> list[structs.CarParams.CarFw]:
  fw = structs.CarParams.CarFw()
  fw.ecu = Ecu.eps
  fw.address = 0x730
  fw.subAddress = 0
  fw.fwVersion = version
  return [fw]


def radar_fw(version: bytes) -> structs.CarParams.CarFw:
  fw = structs.CarParams.CarFw()
  fw.ecu = Ecu.fwdRadar
  fw.address = 0x764
  fw.subAddress = 0
  fw.fwVersion = version
  return fw


# padded to the 24-byte fw field the UDS query returns; the padding length is
# load-bearing — a longer test padding once masked an exact-match miss
_g46l_stem = sorted(G46L_RADAR_FW)[0]
G46L_FW = _g46l_stem + b'\x00' * (24 - len(_g46l_stem))


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
    assert CP.steerActuatorDelay == pytest.approx(0.14, abs=5e-8)

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
    # EPS detection must not replace the chassis-specific physical parameters. steerRatio
    # no longer separates the platforms: the pre-2022 racks run the 2022's 18.1 too.
    swapped = car_params(CAR.MAZDA_CX5, car_fw=eps_fw(SWAPPED_EPS_FW))
    cx5_2022 = car_params(CAR.MAZDA_CX5_2022)
    assert swapped.mass != cx5_2022.mass
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
    assert cx9_2021.steerActuatorDelay == pytest.approx(0.14, abs=5e-8)
    assert not cx9_2021.alphaLongitudinalAvailable

  @pytest.mark.parametrize("candidate", list(CAR))
  @pytest.mark.parametrize("swapped", [False, True], ids=["stock", "swapped_eps"])
  def test_alpha_long_follows_the_eps(self, candidate, swapped):
    # alpha long is offered wherever the steer-to-zero EPS is: the platforms that ship it and any
    # Mazda with that EPS swapped in. A stock older EPS cuts lateral below 45 kph, so
    # stop-and-go would run unsteered; those cars are not offered it. Neither is a platform
    # whose DBC has no radar bus (the pre-2021 CX-9), since the port has never seen its radar
    car_fw = eps_fw(SWAPPED_EPS_FW) if swapped else None
    CP = car_params(candidate, car_fw=car_fw, alpha_long=True)
    has_radar_dbc = Bus.radar in DBC[candidate]
    expected = (candidate in STEER_TO_ZERO_PLATFORMS or swapped) and has_radar_dbc
    assert CP.alphaLongitudinalAvailable == expected
    assert CP.openpilotLongitudinalControl == expected
    assert bool(CP.safetyConfigs[0].safetyParam & MazdaSafetyFlags.LONG.value) == expected

  def test_stock_long_still_reads_the_radar_tracks(self):
    assert not car_params(CAR.MAZDA_CX9_2021, alpha_long=True).radarUnavailable

  def test_ke_runs_vision_only_under_a_swapped_eps(self):
    # the first-generation radar speaks no track dialect, so the platform promises no
    # radar bus: the lead comes from the model and no teardown is offered, while the
    # EPS swap still lifts the steering lockouts
    stock = car_params(CAR.MAZDA_CX5_KE)
    assert stock.radarUnavailable
    assert stock.dashcamOnly

    swapped = car_params(CAR.MAZDA_CX5_KE, car_fw=eps_fw(SWAPPED_EPS_FW))
    assert swapped.radarUnavailable
    assert not swapped.dashcamOnly
    assert swapped.minSteerSpeed == 0
    assert swapped.steerActuatorDelay == pytest.approx(0.14, abs=5e-8)
    assert not swapped.alphaLongitudinalAvailable

  @pytest.mark.parametrize("candidate, car_fw, alpha_long, expected", [
    (CAR.MAZDA_CX5_2022, None, False, True),
    (CAR.MAZDA_CX5_2022, None, True, True),
    (CAR.MAZDA_CX5, eps_fw(SWAPPED_EPS_FW), False, True),
    (CAR.MAZDA_CX5, eps_fw(STOCK_CX5_EPS_FW), False, False),
    (CAR.MAZDA_CX5, None, False, False),
    (CAR.MAZDA_CX9_2021, None, True, False),
    (CAR.MAZDA_CX5_2022, eps_fw(LEGACY_FW_EPS), False, False),
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

  @pytest.mark.parametrize("candidate", [CAR.MAZDA_CX5_KE, CAR.MAZDA_CX5, CAR.MAZDA_CX9, CAR.MAZDA_3, CAR.MAZDA_6])
  def test_docs_are_generated_without_firmware(self, candidate):
    # car_fw is empty in docs mode, and the car picker consumes docs mode: a firmware-gated
    # platform must stay selectable there. dashcamOnly is a measured-hardware call, so the
    # on-device EPS check keeps the gate; docs describe the stock car from the platform table.
    from opendbc.car import gen_empty_fingerprint
    from opendbc.car.mazda.interface import CarInterface
    CP = CarInterface.get_params(candidate, gen_empty_fingerprint(), [], alpha_long=False, is_release=False, docs=True)
    assert not CP.dashcamOnly


class TestMazdaLegacyFwEps:
  """The 2022 EPS hardware behind firmware that keeps the 45 kph floor (the THACO-built CX-5
  2023, EPS K319-3210X-B-00). The measured slew, driver window and road-speed ceiling follow
  the hardware; the speed floor, the low-speed scale, the non-delivery latch and alpha long
  stay with the steer-to-zero firmware.
  """

  def test_legacy_firmware_keeps_the_floor_on_the_hardware_envelope(self):
    CP = car_params(CAR.MAZDA_CX5_2022, car_fw=eps_fw(LEGACY_FW_EPS))
    assert CP.flags & MazdaFlags.LEGACY_FW_EPS
    assert not CP.flags & MazdaFlags.STEER_TO_ZERO_EPS
    assert CP.safetyConfigs[0].safetyParam & MazdaSafetyFlags.LEGACY_FW_EPS.value
    assert not CP.safetyConfigs[0].safetyParam & MazdaSafetyFlags.STEER_TO_ZERO_EPS.value
    assert CP.minSteerSpeed == pytest.approx(MIN_STEER_SPEED_STOCK_EPS, abs=5e-8)
    assert not CP.dashcamOnly
    assert CP.steerActuatorDelay == pytest.approx(0.14, abs=5e-8)

  def test_legacy_firmware_is_not_offered_alpha_long(self):
    CP = car_params(CAR.MAZDA_CX5_2022, car_fw=eps_fw(LEGACY_FW_EPS), alpha_long=True)
    assert not CP.alphaLongitudinalAvailable
    assert not CP.openpilotLongitudinalControl
    assert not CP.safetyConfigs[0].safetyParam & MazdaSafetyFlags.LONG.value

  def test_2022_body_without_an_eps_reading_keeps_steer_to_zero(self):
    # docs and a failed firmware query: the platform still answers for its own EPS
    CP = car_params(CAR.MAZDA_CX5_2022)
    assert CP.flags & MazdaFlags.STEER_TO_ZERO_EPS
    assert not CP.flags & MazdaFlags.LEGACY_FW_EPS
    assert CP.minSteerSpeed == 0

  @pytest.mark.parametrize("car_fw", [eps_fw(UNLISTED_EPS_FW), eps_fw(STOCK_CX5_EPS_FW)], ids=["unlisted_eps", "older_platform_eps"])
  def test_2022_body_with_an_unlisted_eps_gets_the_floor_not_a_silent_latch(self, car_fw):
    # a forced CX-5 2022 fingerprint on an EPS the port has not listed: the visible degradation is
    # the 45 kph floor with its banner, and above it the envelope is the same. Listing a genuine
    # new steer-to-zero revision lifts the floor again.
    CP = car_params(CAR.MAZDA_CX5_2022, car_fw=car_fw)
    assert CP.flags & MazdaFlags.LEGACY_FW_EPS
    assert not CP.flags & MazdaFlags.STEER_TO_ZERO_EPS
    assert CP.minSteerSpeed == pytest.approx(MIN_STEER_SPEED_STOCK_EPS, abs=5e-8)
    assert not CP.dashcamOnly

  def test_legacy_firmware_in_an_older_body_stays_dashcam(self):
    CP = car_params(CAR.MAZDA_CX5, car_fw=eps_fw(LEGACY_FW_EPS))
    assert CP.dashcamOnly
    assert CP.flags & MazdaFlags.LEGACY_FW_EPS

  @pytest.mark.parametrize("candidate, car_fw", [
    (CAR.MAZDA_CX9_2021, None),
    (CAR.MAZDA_CX9_2021, eps_fw(b'TC3M-3210X-A-00' + b'\x00' * 9)),
    (CAR.MAZDA_CX5, eps_fw(STOCK_CX5_EPS_FW)),
    (CAR.MAZDA_3, None),
    (CAR.MAZDA_6, None),
    (CAR.MAZDA_CX9, None),
  ], ids=["cx9_2021_no_fw", "cx9_2021_stock_eps", "cx5_stock_eps", "mazda3", "mazda6", "cx9"])
  def test_every_non_steer_to_zero_eps_is_on_the_hardware_envelope(self, candidate, car_fw):
    # one EPS hardware across gen1: the stock CX-9 2021 and the older platforms get the measured
    # slew, driver window and road-speed ceiling, and keep the 45 kph floor of their firmware
    CP = car_params(candidate, car_fw=car_fw)
    assert CP.flags & MazdaFlags.LEGACY_FW_EPS
    assert not CP.flags & MazdaFlags.STEER_TO_ZERO_EPS
    assert CP.safetyConfigs[0].safetyParam & MazdaSafetyFlags.LEGACY_FW_EPS.value
    assert CP.minSteerSpeed == pytest.approx(MIN_STEER_SPEED_STOCK_EPS, abs=5e-8)
    assert CP.steerActuatorDelay == pytest.approx(0.14, abs=5e-8)
    # support itself is still the platform's call: only the CX-9 2021 drives among these
    assert CP.dashcamOnly == (candidate != CAR.MAZDA_CX9_2021)

  def test_no_mazda_is_left_on_the_upstream_envelope(self):
    for candidate in CAR:
      assert car_params(candidate).flags & MazdaFlags.EPS_HW, candidate

  def test_legacy_firmware_is_listed_for_the_2022_body_only(self):
    from opendbc.car.mazda.fingerprints import FW_VERSIONS
    assert LEGACY_FW_EPS in FW_VERSIONS[CAR.MAZDA_CX5_2022][(Ecu.eps, 0x730, None)]
    assert LEGACY_FW_EPS not in STEER_TO_ZERO_EPS_FW
    listed = {fw for c in STEER_TO_ZERO_PLATFORMS for fw in FW_VERSIONS[c][(Ecu.eps, 0x730, None)]}
    assert listed == STEER_TO_ZERO_EPS_FW | {LEGACY_FW_EPS}
    assert LEGACY_FW_EPS not in FW_VERSIONS[CAR.MAZDA_CX8_2023][(Ecu.eps, 0x730, None)]

class TestForeignRadar:
  """The G46L is the one radar known never to publish 0x361-0x366 on bus 0.

  A carried-forward platform bundle can claim a radar bus the physical car cannot fill;
  parsing that bus would starve radarTracks behind a parser that never goes valid. The
  G46L gets the vision-only path, and alpha-long keys on the radar's dialect, not its
  tracks (mazdacan.py replays the G46L's own). Every other radar keeps the platform's
  word, so an unlisted newer revision of a working radar loses nothing.
  """

  def test_g46l_runs_vision_only_behind_a_radar_claim(self):
    # the support-ticket car: a 2016 KE body with the swapped 2022 EPS, forced to the
    # CX-5 2022 platform by a carried-forward bundle. The G46L answers the fw query but
    # never sends 0x361-0x366, so parsing its bus would starve radarTracks forever
    CP = car_params(CAR.MAZDA_CX5_2022, car_fw=[radar_fw(G46L_FW)])
    assert CP.radarUnavailable
    assert CP.alphaLongitudinalAvailable
    assert CP.flags & MazdaFlags.G46L_RADAR

  def test_g46l_unlocks_alpha_long_on_a_platform_without_a_radar_bus(self):
    # the same car on its own platform: no radar bus claimed, but the G46L is reachable
    # and its dialect can be replayed, so the port is offered with the swapped EPS
    fw = eps_fw(SWAPPED_EPS_FW) + [radar_fw(G46L_FW)]
    CP = car_params(CAR.MAZDA_CX5_KE, car_fw=fw, alpha_long=True)
    assert CP.radarUnavailable
    assert not CP.dashcamOnly
    assert CP.alphaLongitudinalAvailable
    assert CP.openpilotLongitudinalControl
    assert bool(CP.safetyConfigs[0].safetyParam & MazdaSafetyFlags.LONG.value)

    # a stock EPS keeps the offer off even with the G46L present
    stock = car_params(CAR.MAZDA_CX5_KE, car_fw=[radar_fw(G46L_FW)], alpha_long=True)
    assert not stock.alphaLongitudinalAvailable

  def test_an_unknown_radar_keeps_the_stock_parse_path(self):
    # an unlisted newer revision of a working radar must not silently lose its tracks:
    # only the G46L is known not to publish them, so everything else keeps the claim
    unknown = [radar_fw(b'KK00-67X00-A' + b'\x00' * 16)]
    claiming = car_params(CAR.MAZDA_CX5_2022, car_fw=unknown)
    assert not claiming.radarUnavailable
    assert claiming.alphaLongitudinalAvailable

    non_claiming = car_params(CAR.MAZDA_CX5_KE, car_fw=eps_fw(SWAPPED_EPS_FW) + unknown, alpha_long=True)
    assert non_claiming.radarUnavailable
    assert not non_claiming.alphaLongitudinalAvailable
    assert not non_claiming.openpilotLongitudinalControl

  def test_silent_radar_keeps_the_platform_claim(self):
    # a fw query that never reached the radar must not flip a claiming platform into
    # vision-only
    CP = car_params(CAR.MAZDA_CX5_2022, car_fw=eps_fw(SWAPPED_EPS_FW))
    assert not CP.radarUnavailable

  def test_g46l_flag_bit_is_unshared(self):
    # single-bit members sharing the G46L bit must be its own flag alone: the rebase onto
    # the danger-unstable promotion once left LEGACY_FW_EPS and G46L_RADAR both on bit 4,
    # and every legacy-firmware car read as carrying the G46L dialect
    sharers = [m.name for m in MazdaFlags if m.value & MazdaFlags.G46L_RADAR and bin(m.value).count('1') == 1]
    assert sharers == ['G46L_RADAR']

  def test_the_platforms_own_radar_fw_stays_parsed(self):
    # only the G46L set disables parsing: the CX-5 2022's own radar firmware must keep
    # the track path
    own_fw = sorted(FW_VERSIONS[CAR.MAZDA_CX5_2022][(Ecu.fwdRadar, 0x764, None)])[0]
    CP = car_params(CAR.MAZDA_CX5_2022, car_fw=[radar_fw(own_fw)])
    assert not CP.radarUnavailable
    assert CP.alphaLongitudinalAvailable
    assert not CP.flags & MazdaFlags.G46L_RADAR


def test_non_gen1_platform_refused_at_admission():
  # one init-time check instead of per-frame guards in the message builders, which every
  # frame layout in mazdacan assumes; the fall-throughs used to emit an all-zero CAM_LKAS
  # and return None from the button builder, straight into can_sends
  CP = car_params(CAR.MAZDA_CX5_2022)
  CP_SP = car_params_sp(CP)
  CP.flags = 0
  with pytest.raises(NotImplementedError):
    CarController({Bus.pt: DBC_NAME}, CP, CP_SP)


class TestMovingTakeoverCapability:
  """The moving takeover is a developer declaration, never a fingerprint rule yet."""

  def test_param_sets_the_flag_under_alpha_long_only(self):
    from opendbc.sunnypilot.car.interfaces import setup_interfaces
    for alpha_long, declared, expect in ((True, "1", True), (True, "0", False), (False, "1", False)):
      CP = car_params(CAR.MAZDA_CX5_2022, alpha_long=alpha_long, car_fw=eps_fw(SWAPPED_EPS_FW))
      CP_SP = car_params_sp(CP, CAR.MAZDA_CX5_2022, alpha_long=alpha_long)
      setup_interfaces(CarInterface, CP, CP_SP, [{"MazdaMovingTakeover": declared}])
      assert bool(CP.flags & MazdaFlags.MOVING_TAKEOVER) == expect
