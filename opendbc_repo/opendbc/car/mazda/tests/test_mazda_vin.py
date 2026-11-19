"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest

from opendbc.car.fw_versions import match_fw_to_car
from opendbc.car.mazda.fingerprints import FW_VERSIONS
from opendbc.car.mazda.tests.conftest import Ecu, car_fw
from opendbc.car.mazda.values import CAR, match_fw_to_car_fuzzy, platform_from_vin
from opendbc.car.vin import VIN_UNKNOWN

# a steer-to-zero EPS a swap donates; the CX-5 2022 list also carries legacy firmware now
DONOR_EPS_FW = b'KSD5-3210X-C-00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
# Versions that exist in no database, standing in for dealer-updated ECUs
UNKNOWN_EPS_FW = b'ZZ99-3210X-Z-99' + b'\x00' * 9
UNKNOWN_ENGINE_FW = b'ZZ99-9999X-Z-99' + b'\x00' * 9
UNKNOWN_ABS_FW = b'ZZ99-8888X-Z-99' + b'\x00' * 9
UNKNOWN_TRANS_FW = b'ZZ99-7777X-Z-99' + b'\x00' * 9
# an Oceania export VIN (real report): no model year, so it never decodes
OCEANIA_VIN = 'JM0TC2WLA00202380'
CX9_2021_ENGINE = FW_VERSIONS[CAR.MAZDA_CX9_2021][(Ecu.engine, 0x7e0, None)][0]


def swap_fw(eps=DONOR_EPS_FW, engine=UNKNOWN_ENGINE_FW, trans=UNKNOWN_TRANS_FW) -> list:
  """A swapped EPS on dealer-updated body ECUs, as the real matcher sees it."""
  return [car_fw(Ecu.eps, 0x730, eps), car_fw(Ecu.engine, 0x7e0, engine),
          car_fw(Ecu.abs, 0x760, UNKNOWN_ABS_FW), car_fw(Ecu.transmission, 0x7e1, trans)]


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
  # JM7 export crossovers carry the same chassis and year fields as JM3
  (make_vin('JM7', 'KF', 'S'), CAR.MAZDA_CX5_2022),  # 2025 CX-5 AWD, Latin America (zoompilot/opendbc#18)
  (make_vin('JM7', 'KF', 'T'), CAR.MAZDA_CX5_2022),  # 2026 CX-5, GCC; KF outside North America (zoompilot/opendbc#21)
  (make_vin('JM7', 'KF', 'L'), CAR.MAZDA_CX5),
  (make_vin('JM7', 'TC', 'P'), CAR.MAZDA_CX9_2021),
  ('JM7TC4WLAS0483192', CAR.MAZDA_CX9_2021),  # 2025 export CX-9, route 02d6a98624adfd3f/00000029
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
    # the resolver, the fuzzy matcher, and the real matcher behind a donor EPS and dealer-updated
    # ECUs: the VIN path is the fork's one addition over upstream
    expected_platforms = {str(expected)} if expected is not None else set()
    assert platform_from_vin(vin) == (str(expected) if expected is not None else None)
    assert match_fw_to_car_fuzzy({}, vin, FW_VERSIONS) == expected_platforms
    exact_match, matches = match_fw_to_car(swap_fw(), vin)
    assert matches == expected_platforms
    assert expected is None or not exact_match

  def test_the_support_log_vin_resolves(self):
    # the KE body from the alpha-long support log, running behind a carried-forward
    # CX-5 2022 platform bundle: the resolver must name its own platform
    assert platform_from_vin('JM3KE4DYXG0877243') == str(CAR.MAZDA_CX5_KE)

  def test_platform_from_vin_rejects_unknown(self):
    assert platform_from_vin(VIN_UNKNOWN) is None
    assert platform_from_vin('JM3KE4DYXG08772') is None  # short

  def test_wrong_wmi_does_not_match(self):
    assert match_fw_to_car_fuzzy({}, make_vin('JM6', 'TC', 'M'), FW_VERSIONS) == set()

  @pytest.mark.parametrize("wmi, chassis_code, year_code", [
    ('JM1', 'BP', 'K'),  # Mazda 3 2019+
    ('JM3', 'DM', 'N'),  # CX-30
    ('JM3', 'KE', 'H'),  # a CX-5 KE past the last supported model year
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

  @pytest.mark.parametrize("year_code", ['C', 'D', 'E', 'F', 'G'])
  def test_ke_model_years_name_the_ke_platform(self, year_code):
    # the first-generation CX-5, one platform across its whole 2012-16 run
    assert match_fw_to_car_fuzzy({}, make_vin('JM3', 'KE', year_code), FW_VERSIONS) == {str(CAR.MAZDA_CX5_KE)}

  def test_swap_fallback_is_export_only_and_needs_both_ecus(self):
    both = {(0x7e0, None): {CX9_2021_ENGINE}, (0x730, None): {DONOR_EPS_FW}, (0x760, None): {UNKNOWN_ABS_FW}}
    # an unknown WMI, no VIN or an invalid VIN never reach it
    for vin in (make_vin('7MM', 'VA', 'P'), VIN_UNKNOWN, 'JM0TC2WLA0020238'):
      assert match_fw_to_car_fuzzy(both, vin, FW_VERSIONS) == set(), vin
    # a decodable WMI that names an unsupported model is authoritative
    assert match_fw_to_car_fuzzy(both, make_vin('JM1', 'BP', 'K'), FW_VERSIONS) == set()
    # the donor EPS alone or the engine alone is one recognized ECU
    donor_only = {(0x730, None): {DONOR_EPS_FW}, (0x7e0, None): {UNKNOWN_ENGINE_FW}, (0x760, None): {UNKNOWN_ABS_FW}}
    engine_only = {(0x7e0, None): {CX9_2021_ENGINE}, (0x730, None): {UNKNOWN_EPS_FW}, (0x760, None): {UNKNOWN_ABS_FW}}
    assert match_fw_to_car_fuzzy(donor_only, OCEANIA_VIN, FW_VERSIONS) == set()
    assert match_fw_to_car_fuzzy(engine_only, OCEANIA_VIN, FW_VERSIONS) == set()

  def test_vin_names_the_chassis_over_the_engine(self):
    engine = FW_VERSIONS[CAR.MAZDA_CX5][(Ecu.engine, 0x7e0, None)][0]
    assert match_fw_to_car_fuzzy({(0x7e0, None): {engine}}, make_vin('JM3', 'TC', 'M'), FW_VERSIONS) == {str(CAR.MAZDA_CX9_2021)}


class TestMatchFwToCarVinFallback:
  """The EPS-swap scenario through the real matcher: the donor EPS breaks every
  exact match, unknown engine and ABS versions break generic fuzzy matching, and
  the VIN names the chassis. An export VIN never decodes, so there the engine
  plus a recognized steer-to-zero donor EPS stand in for upstream's two ECUs."""

  def test_swapped_eps_matches_the_chassis_by_vin(self):
    stock_trans = FW_VERSIONS[CAR.MAZDA_6][(Ecu.transmission, 0x7e1, None)][0]
    swapped = swap_fw(trans=stock_trans)
    exact_match, matches = match_fw_to_car(swapped, make_vin('JM1', 'GL', 'L'))
    assert not exact_match
    assert matches == {str(CAR.MAZDA_6)}
    # no VIN and an unknown engine: nothing names it
    assert match_fw_to_car(swapped, VIN_UNKNOWN)[1] == set()

  def test_stock_fw_set_still_exact_matches(self):
    stock = [car_fw(ecu, addr, versions[0]) for (ecu, addr, _), versions in FW_VERSIONS[CAR.MAZDA_CX9_2021].items()]
    exact_match, matches = match_fw_to_car(stock, make_vin('JM3', 'TC', 'M'))
    assert exact_match
    assert matches == {str(CAR.MAZDA_CX9_2021)}

  def test_reported_ke_exact_matches_on_its_body_ecus(self):
    # the 2016.5 report: a 2022 CX-5 EPS swap riding on first-generation body ECUs.
    # The swap firmware stays under MAZDA_CX5_2022; the body ECUs alone exact-match
    # the chassis, and no EPS entry exists to contradict the swap
    donor_eps = FW_VERSIONS[CAR.MAZDA_CX5_2022][(Ecu.eps, 0x730, None)][1]  # KSD5, the reported swap
    body = [car_fw(ecu, addr, versions[0]) for (ecu, addr, _), versions in FW_VERSIONS[CAR.MAZDA_CX5_KE].items()]
    exact_match, matches = match_fw_to_car(body + [car_fw(Ecu.eps, 0x730, donor_eps)], make_vin('JM3', 'KE', 'G'))
    assert exact_match
    assert matches == {str(CAR.MAZDA_CX5_KE)}

  def test_ke_names_by_vin_through_a_donor_eps(self):
    # a KE with dealer-updated body ECUs still names by VIN through the swap; the
    # donor EPS is the one ECU the database knows
    donor_eps = FW_VERSIONS[CAR.MAZDA_CX5_2022][(Ecu.eps, 0x730, None)][0]
    exact_match, matches = match_fw_to_car(swap_fw(eps=donor_eps), make_vin('JM3', 'KE', 'G'))
    assert not exact_match
    assert matches == {str(CAR.MAZDA_CX5_KE)}

  @pytest.mark.parametrize("vin", [OCEANIA_VIN, make_vin('JM0', 'TC', 'A')])
  def test_oceania_eps_swap_matches_on_the_engine_behind_the_donor_eps(self, vin):
    # the reported car: Oceania VIN (never decodes), donor EPS, and chassis ECUs
    # unknown to the North American database. Two recognized ECUs: the engine
    # names the chassis, the EPS is one this port grants lateral through
    exact_match, matches = match_fw_to_car(swap_fw(engine=CX9_2021_ENGINE), vin)
    assert not exact_match
    assert matches == {str(CAR.MAZDA_CX9_2021)}

  def test_oceania_swap_without_a_recognized_eps_stays_unmatched(self):
    # same car with an EPS this port does not know: one recognized ECU, no platform
    assert match_fw_to_car(swap_fw(eps=UNKNOWN_EPS_FW, engine=CX9_2021_ENGINE), OCEANIA_VIN)[1] == set()

  def test_oceania_eps_swap_with_two_known_ecus_fuzzy_matches(self):
    # same car once its transmission firmware is in the database: two uniquely
    # matching non-EPS ECUs is upstream's own fuzzy bar, no VIN needed
    trans = FW_VERSIONS[CAR.MAZDA_CX9_2021][(Ecu.transmission, 0x7e1, None)][0]
    exact_match, matches = match_fw_to_car(swap_fw(engine=CX9_2021_ENGINE, trans=trans), OCEANIA_VIN)
    assert not exact_match
    assert matches == {str(CAR.MAZDA_CX9_2021)}

  def test_unknown_wmi_with_one_known_ecu_stays_unmatched(self):
    # a WMI outside the table (7MM, CX-50) with a Mazda 3 engine calibration and
    # nothing else recognized: no platform, so no lateral. An export VIN without a
    # recognized EPS does not reach the swap fallback either
    engine = FW_VERSIONS[CAR.MAZDA_3][(Ecu.engine, 0x7e0, None)][0]
    fw = [car_fw(Ecu.engine, 0x7e0, engine), car_fw(Ecu.abs, 0x760, UNKNOWN_ABS_FW),
          car_fw(Ecu.transmission, 0x7e1, UNKNOWN_TRANS_FW)]
    for vin in (make_vin('7MM', 'VA', 'P'), VIN_UNKNOWN, 'JM3KF2L50NI000042', make_vin('JM0', 'TC', 'A')):
      assert match_fw_to_car(fw, vin)[1] == set(), vin
