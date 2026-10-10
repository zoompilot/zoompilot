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


from opendbc.sunnypilot.car.icbm_actuation_profile import ICBM_ACTUATION_PROFILES
from openpilot.common.basedir import BASEDIR
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import limits

UPSTREAM_PLANNER = pathlib.Path(BASEDIR) / "openpilot/selfdrive/controls/lib/longitudinal_planner.py"
MEASURED = [p for p in ICBM_ACTUATION_PROFILES.values() if p.decel_overshoot is not None]


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


class TestStockTrackingGap:
  """The planner's stock budget and actuation lead describe what decel overshoot actually does;
  both must agree with the servo's overshoot table in the same actuation profile."""

  def test_budget_is_a_column_of_the_overshoot_table(self):
    for profile in MEASURED:
      assert profile.stock_a_budget in profile.decel_overshoot['decel_bp']

  def test_track_gap_is_the_gap_at_budget(self):
    for profile in MEASURED:
      p = profile.decel_overshoot
      col = p['decel_bp'].index(profile.stock_a_budget)
      at_budget = [row[col] for row in p['gap_v']]
      assert min(at_budget) <= profile.track_gap <= max(at_budget)

  def test_lead_walks_only_the_tracking_gap(self):
    from opendbc.car import structs
    lim = limits.get_planning_limits(structs.CarParams(brand="mazda", openpilotLongitudinalControl=False))
    mph = 1. / limits._MPH_PER_MS
    assert lim.dash_traversal_time(4. * mph) == 4. / lim.walk_rate
    assert lim.dash_traversal_time(30. * mph) == lim.track_gap / lim.walk_rate
