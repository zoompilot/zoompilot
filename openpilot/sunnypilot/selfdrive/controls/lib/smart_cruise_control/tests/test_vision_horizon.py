"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Vision controller: the highway planning horizon, the per-path budgets, the shared
publication ramp and the ICBM lookahead wire. Road rendering and the base case live in
vision_harness.py.
"""
import pytest

import openpilot.cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import MIN_V
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.limits import A_PUB_MIN, PUB_JERK, get_planning_limits, publish_ramp
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.vision_controller import SmartCruiseControlVision
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.tests.vision_harness import (
  SETPOINT, V_EGO, VisionCase, curve_at, make_cp, model_for_road, patch_horizon)

OP_LONG_IDS = ["op_long", "stock"]


class TestHighwayHorizon(VisionCase):

  @pytest.mark.parametrize("op_long", [True, False], ids=OP_LONG_IDS)
  def test_highway_plans_on_the_near_window_only(self, op_long):
    # 70 mph: a real r=350 m bend (25.8 m/s allowed) is not in the plan while it sits beyond
    # the 3 s window, on either path, and commits once it is inside
    v, kappa = 31., 1. / 350.
    scc = SmartCruiseControlVision(make_cp(op_long=op_long))
    self.run_road(v, curve_at(150., kappa), n=5, setpoint=v, scc=scc)
    assert not scc.is_active, op_long
    assert scc.a_required == 0.
    assert scc.v_ahead_min == 255.  # the ICBM lookahead follows the same horizon
    self.run_road(v, curve_at(80., kappa), n=5, setpoint=v, scc=scc)
    assert scc.is_active, op_long
    assert scc.output_v_target < v - 0.5
    assert scc.output_a_target < 0.

  @pytest.mark.parametrize("op_long", [True, False], ids=OP_LONG_IDS)
  def test_highway_far_field_read_on_a_straight_road_never_commits(self, op_long):
    # the replayed failure: at 68 mph the model reported an r=330 m bend between 120 and
    # 200 m on a road that never bent (steering curvature stayed under 1/1900 for 20 s).
    # Both paths braked for it; with the horizon on the near window neither plans on it
    def hallucination(s):
      return 0.003 if 120. <= s <= 200. else 0.
    v = 30.
    scc = SmartCruiseControlVision(make_cp(op_long=op_long))
    self.run_road(v, hallucination, n=10, setpoint=v + 3., scc=scc)
    assert not scc.is_active, op_long
    assert scc.a_required == 0.
    assert scc.output_v_target == V_CRUISE_UNSET
    # the same read commits as soon as the horizon is let back out
    with patch_horizon([1e4, 1e4]):
      pre = SmartCruiseControlVision(make_cp(op_long=op_long))
      self.run_road(v, hallucination, n=10, setpoint=v + 3., scc=pre)
    assert pre.is_active, op_long

  def test_horizon_is_whole_inside_the_band_and_fades_across_it(self):
    # 50 mph: the far corner is planned on, exactly as before (the fitted gain owns it)
    def far_bend(s):
      return 0.003 if 120. <= s <= 200. else 0.
    self.run_road(22.4, far_bend, setpoint=22.4)
    assert self.scc_v.a_required > 0.
    # 55 mph: the horizon is halfway in (150 m); a bend at 130 m is planned, one past 150 m not
    v = 24.6
    self.run_road(v, lambda s: 0.003 if 130. <= s <= 140. else 0., setpoint=v)
    assert self.scc_v.a_required > 0.
    self.run_road(v, lambda s: 0.003 if 170. <= s <= 200. else 0., setpoint=v)
    assert self.scc_v.a_required == 0.

  def test_op_long_published_decel_is_clipped_to_the_budget(self):
    # the plan aTarget seeds mpc.set_cur_state and stage 0 is pinned to the seed, so a
    # -2.0 here would bypass A_CRUISE_MIN through the MPC candidate; the wire is an
    # actuator command on openpilot long and stops at the budget
    road = curve_at(40., kappa=0.03)
    a_min = 0.
    for _ in range(60):
      self.run_road(V_EGO, road, n=1)
      a_min = min(a_min, self.scc_v.output_a_target)
    assert self.scc_v.is_active
    assert a_min == pytest.approx(-self.scc_v.limits.a_budget, abs=1e-6)

    # stock ACC keeps the full lever: the same wire is ICBM's dash gap, not a command
    stock = SmartCruiseControlVision(make_cp(op_long=False))
    a_min = 0.
    for _ in range(60):
      self.run_road(V_EGO, road, n=1, scc=stock)
      a_min = min(a_min, stock.output_a_target)
    assert a_min == pytest.approx(A_PUB_MIN, abs=1e-6)

  def test_published_decel_stays_within_the_clip(self):
    # Constraints inside the actuation lead can produce a large required deceleration, which
    # the publication cap must bound.
    stock = SmartCruiseControlVision(make_cp(op_long=False))
    self.run_road(V_EGO, curve_at(100., kappa=0.012), n=60, scc=stock, attenuate=True)
    assert stock.output_a_target >= -2.0
    assert stock.output_v_target >= MIN_V


class TestPerPathBudgets(VisionCase):

  def test_stock_path_commits_earlier_and_prepositions_the_dip(self):
    # same road: op-long (1.2 budget, 0.36 s lead) still holds; stock ACC (0.75 budget,
    # response + dash traversal lead) is already inside its braking distance
    road = curve_at(140., kappa=0.014)
    self.run_road(V_EGO, road, attenuate=True)
    assert not self.scc_v.is_active

    stock = SmartCruiseControlVision(make_cp(op_long=False))
    self.run_road(V_EGO, road, scc=stock, attenuate=True)
    assert stock.is_active
    # the dash cannot track a profile; it gets sent to the dip itself
    assert abs(stock.output_v_target - stock.v_dip_ahead) < 1.0
    assert stock.v_dip_ahead < V_EGO


class TestPublishRamp:
  """The one publication ramp the limiter sources share."""

  def test_op_long_clips_at_the_budget(self):
    lim = get_planning_limits(make_cp(op_long=True))
    assert lim.a_pub_min == -lim.a_budget
    assert publish_ramp(-2.0, -1.18, lim, V_EGO) == pytest.approx(-lim.a_budget)

  def test_stock_clips_at_the_lever_depth(self):
    lim = get_planning_limits(make_cp(op_long=False))
    assert lim.a_pub_min == A_PUB_MIN
    assert publish_ramp(-3.0, -1.95, lim, V_EGO) == pytest.approx(A_PUB_MIN)

  def test_step_is_jerk_limited_both_ways(self):
    lim = get_planning_limits(make_cp(op_long=True))
    j = lim.jerk(V_EGO)
    assert publish_ramp(-1.0, 0., lim, V_EGO) == pytest.approx(-j * DT_MDL)
    assert publish_ramp(0., -1.0, lim, V_EGO) == pytest.approx(-1.0 + j * DT_MDL)
    assert publish_ramp(-0.01, 0., lim, V_EGO) == pytest.approx(-0.01)

  def test_stock_ramps_at_the_publication_jerk(self):
    lim = get_planning_limits(make_cp(op_long=False))
    assert lim.jerk(V_EGO) == 0.
    assert publish_ramp(-1.0, 0., lim, V_EGO) == pytest.approx(-PUB_JERK * DT_MDL)


class TestLookaheadWire:
  """v_ahead_min feeds the ICBM restore gate: 0 must mean exactly "no lookahead"."""

  def setup_method(self):
    self.params = Params()
    self.params.put_bool("SmartCruiseControlVision", True, block=True)
    self.scc_v = SmartCruiseControlVision(make_cp())

  def step(self, v=V_EGO, kappa_fn=lambda s: 0., long_enabled=True):
    sm = {'modelV2': model_for_road(v, kappa_fn).modelV2,
          'controlsState': messaging.new_message('controlsState').controlsState}
    self.scc_v.update(sm, long_enabled, False, v, 0., SETPOINT)

  def test_clear_road_caps_at_unset(self):
    self.step()
    assert self.scc_v.v_ahead_min == 255.

  def test_dip_passes_through(self):
    self.step(kappa_fn=curve_at(60.))
    assert 0. < self.scc_v.v_ahead_min < SETPOINT

  def test_long_disabled_reports_no_lookahead(self):
    self.step(kappa_fn=curve_at(60.))
    self.step(long_enabled=False)
    assert self.scc_v.v_ahead_min == 0.

  def test_toggle_off_reports_no_lookahead(self):
    self.params.put_bool("SmartCruiseControlVision", False, block=True)
    scc = SmartCruiseControlVision(make_cp())
    sm = {'modelV2': model_for_road(V_EGO, curve_at(60.)).modelV2,
          'controlsState': messaging.new_message('controlsState').controlsState}
    scc.update(sm, True, False, V_EGO, 0., SETPOINT)
    assert scc.v_ahead_min == 0.
