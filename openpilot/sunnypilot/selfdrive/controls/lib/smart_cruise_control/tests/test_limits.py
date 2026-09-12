"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

limits.py mirrors the op-long cruise candidate's budget and jerk table from
selfdrive/controls/lib/longitudinal_planner.py, which it cannot import: the upstream
planner imports the SP overlay, which imports this package (and it pulls in acados).
Read the values out of the upstream source instead, so a sync cannot move them silently.
"""
import ast
import pathlib

from openpilot.common.basedir import BASEDIR
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import limits

UPSTREAM_PLANNER = pathlib.Path(BASEDIR) / "openpilot/selfdrive/controls/lib/longitudinal_planner.py"


def _module_constants(path: pathlib.Path) -> dict:
  tree = ast.parse(path.read_text())
  values = {}
  for node in tree.body:
    if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
      try:
        values[node.targets[0].id] = ast.literal_eval(node.value)
      except ValueError:
        pass
  return values


class TestOpLongMirror:
  def test_budget_and_jerk_table_match_upstream(self):
    up = _module_constants(UPSTREAM_PLANNER)
    assert limits._OP_LONG_A_BUDGET == -up["A_CRUISE_MIN"]
    assert limits._OP_LONG_J_BP == up["A_CRUISE_MAX_BP"]
    assert limits._OP_LONG_J_VALS == up["J_CRUISE_VALS"]

  def test_upstream_still_interpolates_jerk_over_the_accel_breakpoints(self):
    # the mirror assumes j_cruise = interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)
    src = UPSTREAM_PLANNER.read_text()
    assert "np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)" in src
