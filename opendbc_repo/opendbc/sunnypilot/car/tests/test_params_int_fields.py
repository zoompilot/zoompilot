"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import dataclasses
import itertools
import re
from pathlib import Path

import pytest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.car_helpers import interfaces
from opendbc.car.common.basedir import BASEDIR
from opendbc.car.interfaces import get_interface_attr
from opendbc.car.vin import VIN_UNKNOWN
from opendbc.sunnypilot.car.interfaces import setup_interfaces

# card serializes CarParamsSP with pycapnp, which rejects any int subclass. OR-ing a bare IntFlag
# member into a field (`ret.flags |= XFlags.Y`) leaves an IntFlag there, behaves like an int
# everywhere in Python, and only crashes card after fingerprinting. CarParams is a capnp builder
# already, so it rejects the IntFlag at assignment.

OPENDBC = Path(BASEDIR).parent
VIN_TAIL = "KF4WLA10114918"
WMIS = get_interface_attr("WMI", ignore_none=True)
# The keys setup_interfaces reads; 1-3 covers every LongitudinalTuningType and MadsScreenButtonType.
TOGGLE_KEYS = ("HyundaiLongitudinalTuning", "MazdaTjaButton", "SubaruStopAndGo", "SubaruStopAndGoManualParkingBrake",
               "TeslaCoopSteering", "TeslaMadsScreenButton", "ToyotaEnforceStockLongitudinal", "ToyotaStopAndGoHack")
TOGGLE_SETS = [None] + [[dict.fromkeys(TOGGLE_KEYS, v)] for v in ("1", "2", "3")]


def _int_subclasses(obj, path: str = ""):
  if dataclasses.is_dataclass(obj):
    for f in dataclasses.fields(obj):
      yield from _int_subclasses(getattr(obj, f.name), f"{path}.{f.name}")
  elif isinstance(obj, (list, tuple)):
    for i, v in enumerate(obj):
      yield from _int_subclasses(v, f"{path}[{i}]")
  elif isinstance(obj, int) and type(obj) not in (int, bool):
    yield path, obj


@pytest.mark.parametrize("platform", sorted(p for p in interfaces if p != "MOCK"))
def test_car_params_sp_holds_plain_ints(platform):
  CarInterface = interfaces[platform]
  fingerprint = gen_empty_fingerprint()
  vins = [VIN_UNKNOWN] + [str(wmi) + VIN_TAIL for wmi in WMIS.get(CarInterface.get_non_essential_params(platform).brand, [])]
  for vin, alpha_long, params_list in itertools.product(vins, (False, True), TOGGLE_SETS):
    CP = CarInterface.get_params(platform, fingerprint, [], alpha_long, False, docs=False)
    CP.carVin = vin
    CP_SP = CarInterface.get_params_sp(CP, platform, fingerprint, [], alpha_long, False, docs=False)
    setup_interfaces(CarInterface, CP, CP_SP, params_list)

    bad = ", ".join(f"{p}={v!r}" for p, v in _int_subclasses(CP_SP, "CP_SP"))
    assert not bad, f"vin={vin} alpha_long={alpha_long} toggles={params_list}: {bad}"


def test_no_bare_flag_members_assigned_to_params():
  # Catches the branches the runtime test cannot reach (fw-, CAN- or param-gated). PlatformConfig's
  # own `self.flags |= XFlags.Y` is fine: interfaces.py stores it as int(platform.config.flags).
  sources = {p: p.read_text() for p in OPENDBC.rglob("*.py") if "tests" not in p.parts}
  enum_classes = {m for text in sources.values() for m in re.findall(r"^class (\w+)\((?:IntFlag|IntEnum)\):", text, re.M)}
  member = rf"(?:{'|'.join(sorted(enum_classes))})\.[A-Z][A-Z0-9_]*"
  # any member left without .value, alone or in an | chain (A.X | A.Y.value is still an IntFlag)
  bare = re.compile(rf"^\s*(?!self\.)\w+(?:\.\w+)+\s*\|?=\s*{member}(?:\s*\|\s*{member})*(?![.\w])")

  hits = [f"{p.relative_to(OPENDBC)}:{i}: {line.strip()}"
          for p, text in sources.items() for i, line in enumerate(text.splitlines(), 1) if bare.match(line)]
  assert not hits, "store .value (or int(...)), capnp rejects IntFlag:\n" + "\n".join(hits)
