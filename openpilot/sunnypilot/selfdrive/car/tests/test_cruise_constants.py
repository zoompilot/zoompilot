"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The cruise sentinels have one upstream home, selfdrive.car.cruise. The card-side stack
cannot import it (cruise.py imports cruise_ext before defining them), so the fork keeps
exactly one copy of V_CRUISE_UNSET, in speed_limit, and takes V_CRUISE_MAX from opendbc.
These tests pin both to upstream and fail if a third copy reappears.
"""
import ast
import pathlib

from opendbc.car import interfaces as opendbc_interfaces
from openpilot.common.basedir import BASEDIR
from openpilot.selfdrive.car import cruise
from openpilot.sunnypilot.selfdrive.car import cruise_arbiter, cruise_ext
from openpilot.sunnypilot.selfdrive.controls.lib import speed_limit

SP_ROOT = pathlib.Path(BASEDIR) / "openpilot" / "sunnypilot"
SENTINELS = ("V_CRUISE_UNSET", "V_CRUISE_MAX", "V_CRUISE_MIN")
# the only fork modules allowed to assign a sentinel, and which one
ALLOWED = {
  SP_ROOT / "selfdrive/controls/lib/speed_limit/__init__.py": {"V_CRUISE_UNSET"},
  SP_ROOT / "selfdrive/car/cruise_ext.py": {"V_CRUISE_MIN"},
}


def _assigned_names(path: pathlib.Path) -> set[str]:
  tree = ast.parse(path.read_text())
  names = set()
  for node in ast.walk(tree):
    if isinstance(node, ast.Assign):
      names.update(t.id for t in node.targets if isinstance(t, ast.Name))
    elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
      names.add(node.target.id)
  return names


class TestCruiseSentinels:
  def test_unset_matches_upstream(self):
    assert speed_limit.V_CRUISE_UNSET == cruise.V_CRUISE_UNSET
    assert cruise_arbiter.V_CRUISE_UNSET is speed_limit.V_CRUISE_UNSET
    assert cruise_ext.V_CRUISE_UNSET is speed_limit.V_CRUISE_UNSET

  def test_max_matches_upstream(self):
    assert opendbc_interfaces.V_CRUISE_MAX == cruise.V_CRUISE_MAX
    assert cruise_arbiter.V_CRUISE_MAX is opendbc_interfaces.V_CRUISE_MAX
    assert cruise_ext.V_CRUISE_MAX is opendbc_interfaces.V_CRUISE_MAX

  def test_min_matches_upstream(self):
    assert cruise_ext.V_CRUISE_MIN == cruise.V_CRUISE_MIN

  def test_no_other_copy(self):
    offenders = {}
    for path in sorted(p for p in SP_ROOT.rglob("*.py") if "tests" not in p.parts):
      extra = (_assigned_names(path) & set(SENTINELS)) - ALLOWED.get(path, set())
      if extra:
        offenders[str(path.relative_to(SP_ROOT))] = sorted(extra)
    assert not offenders, f"cruise sentinels redefined: {offenders}"
