"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest

from opendbc.car import structs
from opendbc.car.fw_versions import match_fw_to_car
from opendbc.car.mazda.fingerprints import FW_VERSIONS
from opendbc.car.mazda.values import CAR, match_fw_to_car_fuzzy
from opendbc.car.vin import VIN_UNKNOWN

Ecu = structs.CarParams.Ecu


def make_vin(wmi: str, chassis_code: str, year_code: str) -> str:
  # positions 6-9, 11-17 are arbitrary: the decoder reads the WMI, the model
  # line (positions 4-5) and the model year code (position 10)
  return wmi + chassis_code + '2L50' + year_code + '0' + '000042'


# Real VINs from public listings, one per model year code, with the trim from the listing.
# None marks a model line with no supported platform.
REAL_VINS = [
  # KF 2017-21 -> MAZDA_CX5
  ('JM3KFBDL8H0189068', CAR.MAZDA_CX5),        # 2017 Grand Touring
  ('JM3KFBCM8J0391425', CAR.MAZDA_CX5),        # 2018 Touring
  ('JM3KFBDY8K0524140', CAR.MAZDA_CX5),        # 2019 Grand Touring Reserve
  ('JM3KFBBMXL0721103', CAR.MAZDA_CX5),        # 2020 Sport
  ('JM3KFBEY9M0334140', CAR.MAZDA_CX5),        # 2021 Signature
  # KF 2022-25 -> MAZDA_CX5_2022
  ('JM3KFBEM7N0646584', CAR.MAZDA_CX5_2022),   # 2022 Premium Plus
  ('JM3KFBXY2P0142737', CAR.MAZDA_CX5_2022),   # 2023 2.5 Turbo Signature
  ('JM3KFBCL4R0506329', CAR.MAZDA_CX5_2022),   # 2024 Preferred
  ('JM3KFBAY8S0594547', CAR.MAZDA_CX5_2022),   # 2025 Carbon Turbo
  # TC 2016-20 -> MAZDA_CX9, 2021-23 -> MAZDA_CX9_2021
  ('JM3TCBDY1G0107351', CAR.MAZDA_CX9),        # 2016 Grand Touring
  ('JM3TCBDY2K0314968', CAR.MAZDA_CX9),        # 2019 Grand Touring
  ('JM3TCBBY1L0416377', CAR.MAZDA_CX9),        # 2020 Sport
  ('JM3TCBEYXM0534974', CAR.MAZDA_CX9_2021),   # 2021 Signature
  ('JM3TCBAY3N0628864', CAR.MAZDA_CX9_2021),   # 2022 Touring Plus
  ('JM3TCBDY4P0655571', CAR.MAZDA_CX9_2021),   # 2023 Carbon Edition
  # GL 2017-21 -> MAZDA_6, BN 2017-18 -> MAZDA_3 (Salamanca builds use 3MZ)
  ('JM1GL1U58H1108261', CAR.MAZDA_6),          # 2017 Sport
  ('JM1GL1VM0J1336606', CAR.MAZDA_6),          # 2018 Touring
  ('JM1GL1TYXK1503013', CAR.MAZDA_6),          # 2019 Grand Touring
  ('JM1GL1TY7L1523723', CAR.MAZDA_6),          # 2020 Grand Touring
  ('JM1GL1VM4M1613049', CAR.MAZDA_6),          # 2021 Touring
  ('3MZBN1K71HM135634', CAR.MAZDA_3),          # 2017 Sport
  ('3MZBN1V34JM170702', CAR.MAZDA_3),          # 2018 Touring
  # Unsupported model lines stay unmatched on real VINs too
  ('JM1BPALL3N1522302', None),                 # 2022 Mazda 3 (BP)
  ('3MVDMBEM0LM104467', None),                 # 2020 CX-30 (DM)
]


class TestMazdaVinMatch:

  @pytest.mark.parametrize("vin, expected", REAL_VINS)
  def test_real_listing_vins(self, vin, expected):
    expected_platforms = {str(expected)} if expected is not None else set()
    assert match_fw_to_car_fuzzy({}, vin, FW_VERSIONS) == expected_platforms

  def test_wrong_wmi_does_not_match(self):
    assert match_fw_to_car_fuzzy({}, make_vin('JM6', 'TC', 'M'), FW_VERSIONS) == set()

  @pytest.mark.parametrize("wmi, chassis_code, year_code", [
    ('JM1', 'BP', 'K'),  # Mazda 3 2019+
    ('JM3', 'DM', 'N'),  # CX-30
    ('JM3', 'KE', 'H'),  # pre-2017 CX-5
    ('7MM', 'VA', 'P'),  # CX-50
    ('JM3', 'TC', 'T'),  # a CX-9 past the last supported model year
  ])
  def test_unsupported_models_do_not_match(self, wmi, chassis_code, year_code):
    assert match_fw_to_car_fuzzy({}, make_vin(wmi, chassis_code, year_code), FW_VERSIONS) == set()

  def test_invalid_vin_does_not_match(self):
    assert match_fw_to_car_fuzzy({}, 'JM3KF2L50NI000042', FW_VERSIONS) == set()  # banned character
    assert match_fw_to_car_fuzzy({}, 'JM3KF', FW_VERSIONS) == set()  # too short

  def test_vin_unknown_does_not_match(self):
    # all zeros passes the charset; '00' matches no model line
    assert match_fw_to_car_fuzzy({}, VIN_UNKNOWN, FW_VERSIONS) == set()

  def test_every_vin_field_is_required(self):
    # WMI, model line and model year must all name the platform: a right
    # chassis under a wrong WMI, or a right chassis in an unsupported year,
    # is not evidence
    assert match_fw_to_car_fuzzy({}, make_vin('JM1', 'KF', 'N'), FW_VERSIONS) == set()   # KF is a JM3 line
    assert match_fw_to_car_fuzzy({}, make_vin('3MZ', 'GL', 'K'), FW_VERSIONS) == set()   # GL never built in Mexico
    assert match_fw_to_car_fuzzy({}, make_vin('JM3', 'KF', 'G'), FW_VERSIONS) == set()   # 2016 KF predates the port
    assert match_fw_to_car_fuzzy({}, make_vin('JM3', 'KF', 'N'), FW_VERSIONS) == {str(CAR.MAZDA_CX5_2022)}

  def test_engine_firmware_alone_is_not_evidence_without_a_decodable_vin(self):
    # an Oceania export VIN (real report): no model year, no known WMI; the
    # engine is the only responding firmware in the database. One recognised
    # ECU plus junk addresses used to name the chassis here; it must not, the
    # car's ECUs belong in fingerprints.py
    engine = FW_VERSIONS[CAR.MAZDA_CX9_2021][(Ecu.engine, 0x7e0, None)][0]
    live = {
      (0x7e0, None): {engine},
      (0x730, None): {b'DONOR-EPS-XXXX\x00\x00\x00\x00'},
      (0x760, None): {b'EXPORT-ABS-XXXX\x00\x00\x00\x00'},
      (0x7e1, None): {b'EXPORT-TRN-XXXX\x00\x00\x00\x00'},
    }
    assert match_fw_to_car_fuzzy(live, 'JM0TC2WLA00202380', FW_VERSIONS) == set()

  def test_export_vin_matches_on_the_engine_behind_a_known_donor_eps(self):
    # the swap fallback: an export VIN cannot decode, a steer-to-zero donor EPS
    # is recognised, and the engine names exactly one platform
    engine = FW_VERSIONS[CAR.MAZDA_CX9_2021][(Ecu.engine, 0x7e0, None)][0]
    donor = FW_VERSIONS[CAR.MAZDA_CX5_2022][(Ecu.eps, 0x730, None)][0]
    live = {(0x7e0, None): {engine}, (0x730, None): {donor}, (0x760, None): {UNKNOWN_ABS_FW}}
    assert match_fw_to_car_fuzzy(live, 'JM0TC2WLA00202380', FW_VERSIONS) == {str(CAR.MAZDA_CX9_2021)}
    assert match_fw_to_car_fuzzy(live, make_vin('JM0', 'TC', 'A'), FW_VERSIONS) == {str(CAR.MAZDA_CX9_2021)}

  def test_swap_fallback_is_export_only_and_needs_both_ecus(self):
    engine = FW_VERSIONS[CAR.MAZDA_CX9_2021][(Ecu.engine, 0x7e0, None)][0]
    donor = FW_VERSIONS[CAR.MAZDA_CX5_2022][(Ecu.eps, 0x730, None)][0]
    both = {(0x7e0, None): {engine}, (0x730, None): {donor}, (0x760, None): {UNKNOWN_ABS_FW}}
    # an unknown WMI, no VIN or an invalid VIN never reach it
    for vin in (make_vin('7MM', 'VA', 'P'), VIN_UNKNOWN, 'JM0TC2WLA0020238'):
      assert match_fw_to_car_fuzzy(both, vin, FW_VERSIONS) == set(), vin
    # a decodable WMI that names an unsupported model is authoritative
    assert match_fw_to_car_fuzzy(both, make_vin('JM1', 'BP', 'K'), FW_VERSIONS) == set()
    # the donor EPS alone or the engine alone is one recognised ECU
    donor_only = {(0x730, None): {donor}, (0x7e0, None): {UNKNOWN_ENGINE_FW}, (0x760, None): {UNKNOWN_ABS_FW}}
    engine_only = {(0x7e0, None): {engine}, (0x730, None): {b'ZZ99-3210X-Z-99' + b'\x00' * 9}, (0x760, None): {UNKNOWN_ABS_FW}}
    assert match_fw_to_car_fuzzy(donor_only, 'JM0TC2WLA00202380', FW_VERSIONS) == set()
    assert match_fw_to_car_fuzzy(engine_only, 'JM0TC2WLA00202380', FW_VERSIONS) == set()

  def test_unknown_engine_does_not_match(self):
    abs_fw = FW_VERSIONS[CAR.MAZDA_CX9_2021][(Ecu.abs, 0x760, None)][0]
    live = {(0x7e0, None): {b'ZZ99-7777X-Z-99' + b'\x00' * 9}, (0x760, None): {abs_fw}}
    assert match_fw_to_car_fuzzy(live, VIN_UNKNOWN, FW_VERSIONS) == set()

  def test_lone_engine_response_does_not_match(self):
    engine = FW_VERSIONS[CAR.MAZDA_CX9_2021][(Ecu.engine, 0x7e0, None)][0]
    assert match_fw_to_car_fuzzy({(0x7e0, None): {engine}}, VIN_UNKNOWN, FW_VERSIONS) == set()

  def test_vin_names_the_chassis_over_the_engine(self):
    engine = FW_VERSIONS[CAR.MAZDA_CX5][(Ecu.engine, 0x7e0, None)][0]
    assert match_fw_to_car_fuzzy({(0x7e0, None): {engine}}, make_vin('JM3', 'TC', 'M'), FW_VERSIONS) == {str(CAR.MAZDA_CX9_2021)}

  def test_unsupported_chassis_vin_does_not_match_on_the_engine(self):
    # A recognized unsupported chassis must not match through an older PCM calibration.
    engine = FW_VERSIONS[CAR.MAZDA_3][(Ecu.engine, 0x7e0, None)][0]
    live = {(0x7e0, None): {engine}, (0x760, None): {UNKNOWN_ABS_FW}}
    assert match_fw_to_car_fuzzy(live, make_vin('JM1', 'BP', 'K'), FW_VERSIONS) == set()

  def test_unknown_wmi_does_not_match_on_the_engine(self):
    # 7MM (CX-50) is a real WMI outside the table. A matching engine firmware
    # behind it used to name MAZDA_3 off that one ECU; an unknown WMI is
    # rejected outright now, whatever firmware rides along
    engine = FW_VERSIONS[CAR.MAZDA_3][(Ecu.engine, 0x7e0, None)][0]
    for live in ({(0x7e0, None): {engine}},
                 {(0x7e0, None): {engine}, (0x760, None): {UNKNOWN_ABS_FW}},
                 {(0x7e0, None): {engine}, (0x760, None): {UNKNOWN_ABS_FW}, (0x7e1, None): {UNKNOWN_TRANS_FW}}):
      assert match_fw_to_car_fuzzy(live, make_vin('7MM', 'VA', 'P'), FW_VERSIONS) == set()
      assert match_fw_to_car_fuzzy(live, make_vin('JM0', 'TC', 'A'), FW_VERSIONS) == set()
      assert match_fw_to_car_fuzzy(live, VIN_UNKNOWN, FW_VERSIONS) == set()


def _car_fw(ecu, address, version: bytes) -> structs.CarParams.CarFw:
  fw = structs.CarParams.CarFw()
  fw.ecu = ecu
  fw.address = address
  fw.subAddress = 0
  fw.fwVersion = version
  fw.brand = 'mazda'
  fw.bus = 0
  return fw


# Versions that exist in no database, standing in for dealer-updated ECUs
UNKNOWN_ENGINE_FW = b'ZZ99-9999X-Z-99' + b'\x00' * 9
UNKNOWN_ABS_FW = b'ZZ99-8888X-Z-99' + b'\x00' * 9
UNKNOWN_TRANS_FW = b'ZZ99-7777X-Z-99' + b'\x00' * 9


class TestMatchFwToCarVinFallback:
  """The EPS-swap scenario through the real matcher: the donor EPS breaks every
  exact match, unknown engine and ABS versions break generic fuzzy matching, and
  the VIN names the chassis. An export VIN never decodes, so there the engine
  plus a recognised steer-to-zero donor EPS stand in for upstream's two ECUs."""

  def _swapped_mazda6_fw(self) -> list:
    donor_eps = FW_VERSIONS[CAR.MAZDA_CX5_2022][(Ecu.eps, 0x730, None)][0]
    stock_trans = FW_VERSIONS[CAR.MAZDA_6][(Ecu.transmission, 0x7e1, None)][0]
    return [
      _car_fw(Ecu.eps, 0x730, donor_eps),
      _car_fw(Ecu.engine, 0x7e0, UNKNOWN_ENGINE_FW),
      _car_fw(Ecu.abs, 0x760, UNKNOWN_ABS_FW),
      _car_fw(Ecu.transmission, 0x7e1, stock_trans),
    ]

  def test_swapped_eps_matches_the_chassis_by_vin(self):
    vin = make_vin('JM1', 'GL', 'L')
    exact_match, matches = match_fw_to_car(self._swapped_mazda6_fw(), vin)
    assert not exact_match
    assert matches == {str(CAR.MAZDA_6)}

  def test_no_vin_and_unknown_engine_stays_unmatched(self):
    _, matches = match_fw_to_car(self._swapped_mazda6_fw(), VIN_UNKNOWN)
    assert matches == set()

  def test_stock_fw_set_still_exact_matches(self):
    car_fw = [_car_fw(ecu, addr, versions[0])
              for (ecu, addr, _), versions in FW_VERSIONS[CAR.MAZDA_CX9_2021].items()]
    vin = make_vin('JM3', 'TC', 'M')
    exact_match, matches = match_fw_to_car(car_fw, vin)
    assert exact_match
    assert matches == {str(CAR.MAZDA_CX9_2021)}

  def test_oceania_eps_swap_matches_on_the_engine_behind_the_donor_eps(self):
    # the reported car: Oceania VIN (never decodes), donor EPS, and chassis ECUs
    # unknown to the North American database. Two recognised ECUs: the engine
    # names the chassis, the EPS is one this port grants lateral through
    engine = FW_VERSIONS[CAR.MAZDA_CX9_2021][(Ecu.engine, 0x7e0, None)][0]
    donor_eps = FW_VERSIONS[CAR.MAZDA_CX5_2022][(Ecu.eps, 0x730, None)][0]
    car_fw = [
      _car_fw(Ecu.eps, 0x730, donor_eps),
      _car_fw(Ecu.engine, 0x7e0, engine),
      _car_fw(Ecu.abs, 0x760, UNKNOWN_ABS_FW),
      _car_fw(Ecu.transmission, 0x7e1, UNKNOWN_TRANS_FW),
    ]
    exact_match, matches = match_fw_to_car(car_fw, 'JM0TC2WLA00202380')
    assert not exact_match
    assert matches == {str(CAR.MAZDA_CX9_2021)}

  def test_oceania_swap_without_a_recognised_eps_stays_unmatched(self):
    # same car with an EPS this port does not know: one recognised ECU, no platform
    engine = FW_VERSIONS[CAR.MAZDA_CX9_2021][(Ecu.engine, 0x7e0, None)][0]
    car_fw = [
      _car_fw(Ecu.eps, 0x730, b'ZZ99-3210X-Z-99' + b'\x00' * 9),
      _car_fw(Ecu.engine, 0x7e0, engine),
      _car_fw(Ecu.abs, 0x760, UNKNOWN_ABS_FW),
      _car_fw(Ecu.transmission, 0x7e1, UNKNOWN_TRANS_FW),
    ]
    _, matches = match_fw_to_car(car_fw, 'JM0TC2WLA00202380')
    assert matches == set()

  def test_oceania_eps_swap_with_two_known_ecus_fuzzy_matches(self):
    # same car once its transmission firmware is in the database: two uniquely
    # matching non-EPS ECUs is upstream's own fuzzy bar, no VIN needed
    engine = FW_VERSIONS[CAR.MAZDA_CX9_2021][(Ecu.engine, 0x7e0, None)][0]
    trans = FW_VERSIONS[CAR.MAZDA_CX9_2021][(Ecu.transmission, 0x7e1, None)][0]
    donor_eps = FW_VERSIONS[CAR.MAZDA_CX5_2022][(Ecu.eps, 0x730, None)][0]
    car_fw = [
      _car_fw(Ecu.eps, 0x730, donor_eps),
      _car_fw(Ecu.engine, 0x7e0, engine),
      _car_fw(Ecu.abs, 0x760, UNKNOWN_ABS_FW),
      _car_fw(Ecu.transmission, 0x7e1, trans),
    ]
    exact_match, matches = match_fw_to_car(car_fw, 'JM0TC2WLA00202380')
    assert not exact_match
    assert matches == {str(CAR.MAZDA_CX9_2021)}

  def test_unknown_wmi_with_one_known_ecu_stays_unmatched(self):
    # a WMI outside the table (7MM, CX-50) with a Mazda 3 engine calibration and
    # nothing else recognised: no platform, so no lateral
    engine = FW_VERSIONS[CAR.MAZDA_3][(Ecu.engine, 0x7e0, None)][0]
    car_fw = [
      _car_fw(Ecu.engine, 0x7e0, engine),
      _car_fw(Ecu.abs, 0x760, UNKNOWN_ABS_FW),
      _car_fw(Ecu.transmission, 0x7e1, UNKNOWN_TRANS_FW),
    ]
    for vin in (make_vin('7MM', 'VA', 'P'), VIN_UNKNOWN, 'JM3KF2L50NI000042'):
      _, matches = match_fw_to_car(car_fw, vin)
      assert matches == set(), vin

  @pytest.mark.parametrize("vin, expected", REAL_VINS)
  def test_known_vins_still_match_through_the_real_matcher(self, vin, expected):
    # the VIN path is the fork's one addition over upstream: a decodable North
    # American VIN names the chassis through a donor EPS and dealer-updated ECUs
    donor_eps = FW_VERSIONS[CAR.MAZDA_CX5_2022][(Ecu.eps, 0x730, None)][0]
    car_fw = [
      _car_fw(Ecu.eps, 0x730, donor_eps),
      _car_fw(Ecu.engine, 0x7e0, UNKNOWN_ENGINE_FW),
      _car_fw(Ecu.abs, 0x760, UNKNOWN_ABS_FW),
      _car_fw(Ecu.transmission, 0x7e1, UNKNOWN_TRANS_FW),
    ]
    exact_match, matches = match_fw_to_car(car_fw, vin)
    if expected is None:
      assert matches == set(), vin
    else:
      assert not exact_match
      assert matches == {str(expected)}, vin
