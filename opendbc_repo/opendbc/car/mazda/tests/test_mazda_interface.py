"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

CarInterface.get_params: what follows the EPS, what stays keyed on the model, the G46L
radar, and the interface's CAM_LANEINFO latch.
"""
import pytest

from opendbc.car import Bus, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.mazda.fingerprints import FW_VERSIONS
from opendbc.car.mazda.tests.conftest import CAM_LKAS, CAM_LANEINFO, car_interface, car_params, eps_fw, radar_fw
from opendbc.car.mazda.values import CAR, DBC, G46L_RADAR_FW, LKAS_LIMITS, STEER_TO_ZERO_EPS_FW, STEER_TO_ZERO_PLATFORMS, \
  MazdaFlags, MazdaSafetyFlags

Ecu = structs.CarParams.Ecu

# The steer-to-zero EPS a swap donates, and a stock pre-2022 CX-5 EPS for contrast
SWAPPED_EPS_FW = b'KSD5-3210X-C-00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
STOCK_CX5_EPS_FW = b'K319-3210X-A-00' + b'\x00' * 9
# The 2022 EPS hardware on firmware that keeps the 45 kph floor (THACO CX-5 2023), and a revision listed nowhere
LEGACY_FW_EPS = b'K319-3210X-B-00' + b'\x00' * 9
UNLISTED_EPS_FW = b'KSD5-3210X-D-00' + b'\x00' * 9

MIN_STEER_SPEED_STOCK_EPS = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS


# padded to the 24-byte fw field the UDS query returns; the padding length is
# load-bearing — a longer test padding once masked an exact-match miss
_g46l_stem = sorted(G46L_RADAR_FW)[0]
G46L_FW = _g46l_stem + b'\x00' * (24 - len(_g46l_stem))


# (candidate, EPS firmware) -> (steer-to-zero, alpha long offered)
EPS_PROJECTIONS = {
  "cx5_2022": (CAR.MAZDA_CX5_2022, None, True, True),
  "cx5_stock_eps": (CAR.MAZDA_CX5, STOCK_CX5_EPS_FW, False, False),
  "cx5_no_fw": (CAR.MAZDA_CX5, None, False, False),
  "cx5_swapped_eps": (CAR.MAZDA_CX5, SWAPPED_EPS_FW, True, True),
  "cx5_legacy_fw": (CAR.MAZDA_CX5, LEGACY_FW_EPS, False, False),
  "cx9_2021": (CAR.MAZDA_CX9_2021, None, False, False),
  "cx9_2021_stock_eps": (CAR.MAZDA_CX9_2021, b'TC3M-3210X-A-00' + b'\x00' * 9, False, False),
  "cx9": (CAR.MAZDA_CX9, None, False, False),
  "mazda3": (CAR.MAZDA_3, None, False, False),
  "mazda6": (CAR.MAZDA_6, None, False, False),
  # the first-generation radar speaks no track dialect: the swap lifts the floor, no teardown is offered
  "ke": (CAR.MAZDA_CX5_KE, None, False, False),
  "ke_swapped_eps": (CAR.MAZDA_CX5_KE, SWAPPED_EPS_FW, True, False),
  # the THACO CX-5 2023, and a forced 2022 fingerprint on an EPS the port has not listed: the
  # visible degradation is the 45 kph floor with its banner, not a silent latch
  "cx5_2022_legacy_fw": (CAR.MAZDA_CX5_2022, LEGACY_FW_EPS, False, False),
  "cx5_2022_unlisted_eps": (CAR.MAZDA_CX5_2022, UNLISTED_EPS_FW, False, False),
  "cx5_2022_older_platform_eps": (CAR.MAZDA_CX5_2022, STOCK_CX5_EPS_FW, False, False),
}


class TestMazdaEps:
  """What follows the EPS. Every gen1 Mazda EPS is the same hardware, so the measured envelope,
  the panda's matching limits, the 0.14 s actuator delay and lateral itself go with every car.
  The firmware decides the rest: the 2022+ steer-to-zero firmware has no 45 kph floor, the
  non-delivery latch and alpha long, and a swap carries it into an older body. An unread EPS
  (docs, a failed query) falls back to the platform. Everything keyed on the radar, camera or
  vehicle dynamics stays keyed on the model.
  """

  @pytest.mark.parametrize("alpha_long", [False, True], ids=["stock_long", "alpha_long"])
  @pytest.mark.parametrize("name", EPS_PROJECTIONS)
  def test_projection(self, name, alpha_long):
    candidate, fw, steer_to_zero, alpha_offered = EPS_PROJECTIONS[name]
    CP = car_params(candidate, car_fw=eps_fw(fw) if fw else None, alpha_long=alpha_long)
    assert not CP.dashcamOnly
    assert CP.steerActuatorDelay == pytest.approx(0.14, abs=5e-8)
    assert CP.safetyConfigs[0].safetyParam & MazdaSafetyFlags.EPS_HW.value
    assert bool(CP.flags & MazdaFlags.STEER_TO_ZERO_EPS) == steer_to_zero
    assert bool(CP.flags & MazdaFlags.LEGACY_FW_EPS) == (not steer_to_zero)
    assert CP.minSteerSpeed == (0 if steer_to_zero else pytest.approx(MIN_STEER_SPEED_STOCK_EPS, abs=5e-8))
    assert CP.alphaLongitudinalAvailable == alpha_offered
    assert CP.openpilotLongitudinalControl == (alpha_long and alpha_offered)
    assert bool(CP.safetyConfigs[0].safetyParam & MazdaSafetyFlags.LONG.value) == CP.openpilotLongitudinalControl
    # stock long reads the radar's tracks wherever the platform has them
    assert CP.radarUnavailable == (Bus.radar not in DBC[candidate] or CP.openpilotLongitudinalControl)

  @pytest.mark.parametrize("candidate", list(CAR))
  @pytest.mark.parametrize("swapped", [False, True], ids=["stock", "swapped_eps"])
  def test_every_platform(self, candidate, swapped):
    # Lateral on every platform's own EPS, the envelope on the hardware. Alpha long is offered
    # wherever the steer-to-zero EPS is, the platforms that ship it and any swap, except a
    # platform whose DBC has no radar bus (the pre-2021 CX-9): the port has never seen its radar.
    CP = car_params(candidate, car_fw=eps_fw(SWAPPED_EPS_FW) if swapped else None, alpha_long=True)
    assert not CP.dashcamOnly
    assert CP.flags & MazdaFlags.EPS_HW
    assert CP.safetyConfigs[0].safetyParam & MazdaSafetyFlags.EPS_HW.value
    expected = (candidate in STEER_TO_ZERO_PLATFORMS or swapped) and Bus.radar in DBC[candidate]
    assert CP.alphaLongitudinalAvailable == expected
    assert bool(CP.safetyConfigs[0].safetyParam & MazdaSafetyFlags.LONG.value) == expected

  def test_swapped_eps_keeps_the_real_vehicle_specs(self):
    # EPS detection must not replace the chassis-specific physical parameters. steerRatio
    # no longer separates the platforms: the pre-2022 racks run the 2022's 18.1 too.
    swapped = car_params(CAR.MAZDA_CX5, car_fw=eps_fw(SWAPPED_EPS_FW))
    stock = car_params(CAR.MAZDA_CX5)
    assert swapped.wheelbase == stock.wheelbase
    assert swapped.tireStiffnessFactor == stock.tireStiffnessFactor

  @pytest.mark.parametrize("candidate", [CAR.MAZDA_CX5_KE, CAR.MAZDA_CX5, CAR.MAZDA_CX9, CAR.MAZDA_3, CAR.MAZDA_6])
  def test_docs_are_generated_without_firmware(self, candidate):
    # car_fw is empty in docs mode, and the car picker consumes docs mode: every platform must
    # stay selectable there.
    from opendbc.car import gen_empty_fingerprint
    from opendbc.car.mazda.interface import CarInterface
    CP = CarInterface.get_params(candidate, gen_empty_fingerprint(), [], alpha_long=False, is_release=False, docs=True)
    assert not CP.dashcamOnly

  def test_legacy_firmware_is_listed_for_the_2022_body_only(self):
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


class TestCamLaneinfoLatch:
  """CI.update() latches the camera's raw CAM_LANEINFO frame, so it must accept every shape callers pass."""

  LANEINFO_IDLE = bytes.fromhex("4201000000001040")

  def test_bare_tuple_from_the_model_tests(self):
    ci = car_interface(alpha_long=False)
    ci.update((0, [CanData(CAM_LANEINFO, self.LANEINFO_IDLE, 2)]))
    assert ci.CS.cam_laneinfo_raw == self.LANEINFO_IDLE
    assert ci.CS.cam_laneinfo_stale_frames == 0

  def test_tuple_list_from_card(self):
    ci = car_interface(alpha_long=False)
    ci.update([(0, [(CAM_LANEINFO, self.LANEINFO_IDLE, 2)])])
    assert ci.CS.cam_laneinfo_raw == self.LANEINFO_IDLE
    assert ci.CS.cam_laneinfo_stale_frames == 0

  def test_silent_cycle_keeps_the_last_payload_and_grows_stale(self):
    ci = car_interface(alpha_long=False)
    ci.update([(0, [(CAM_LANEINFO, self.LANEINFO_IDLE, 2)])])
    ci.update([(1, [(CAM_LKAS, b"\x00" * 8, 2)])])
    assert ci.CS.cam_laneinfo_raw == self.LANEINFO_IDLE
    assert ci.CS.cam_laneinfo_stale_frames == 1
