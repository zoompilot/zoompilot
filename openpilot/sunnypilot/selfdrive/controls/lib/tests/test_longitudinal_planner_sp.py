"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The SP planner overlay hands the arbitrated (vTarget, aTarget) to mpc.set_cur_state, which
pins the MPC's stage 0 to it. A single NaN there poisons HPIPM past acados_reset (every
solve after it fails and the car never resumes), so no source may reach the seed with a
non-finite target.
"""
import math
from unittest.mock import MagicMock, patch

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from opendbc.car import structs
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import SCC_SOURCES, LongitudinalPlannerSP
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.limits import OP_LONG_A_BUDGET

LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource


class TestSeedGuard(OpenpilotTestCase):

  def setup_method(self):
    self.make()

  def make(self):
    cp = structs.CarParams(brand="mazda", openpilotLongitudinalControl=True, longitudinalActuatorDelay=0.36)
    self.planner = LongitudinalPlannerSP(cp, structs.CarParamsSP(), MagicMock())
    # the sources are exercised in their own tests; here only their published pair matters
    self.planner.scc.update = MagicMock()
    self.planner.resolver.update = MagicMock()
    self.planner.resolver.distance = 0.
    self.planner.sla.update = MagicMock()
    self.sm = {'carState': messaging.new_message('carState').carState,
               'carControl': messaging.new_message('carControl').carControl,
               'carStateSP': messaging.new_message('carStateSP').carStateSP}

  def test_finite_targets_pass_through(self):
    self.planner.scc.vision.output_v_target = 15.
    self.planner.scc.vision.output_a_target = -0.8
    v, a = self.planner.update_targets(self.sm, 20., 0., 25.)
    assert (v, a) == (15., -0.8)
    assert self.planner.source == LongitudinalPlanSource.sccVision

  def test_non_finite_source_never_reaches_the_seed(self):
    for bad in (float('nan'), float('inf'), -float('inf')):
      self.make()
      self.planner.scc.vision.output_v_target = 10.
      self.planner.scc.vision.output_a_target = bad
      with patch('openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner.cloudlog') as log:
        v, a = self.planner.update_targets(self.sm, 20., -0.3, 25.)
        self.planner.update_targets(self.sm, 20., -0.3, 25.)
      assert math.isfinite(v) and math.isfinite(a), bad
      assert (v, a) == (25., -0.3)
      assert self.planner.source == LongitudinalPlanSource.cruise
      assert log.error.call_count == 1  # logged once, not at 20 Hz

  def test_nan_v_target_falls_back_to_the_cruise_speed(self):
    self.planner.scc.vision.output_v_target = float('nan')
    self.planner.scc.vision.output_a_target = -0.5
    v, a = self.planner.update_targets(self.sm, 20., 0., 25.)
    assert (v, a) == (25., 0.)

  def test_everything_bad_still_seeds_finite(self):
    self.planner.scc.vision.output_v_target = float('nan')
    v, a = self.planner.update_targets(self.sm, float('nan'), float('nan'), float('nan'))
    assert math.isfinite(v) and math.isfinite(a)


class TestSccNeedsLongOrIcbm:
  """SCC's params survive losing openpilot long and ICBM, so the planner keeps it idle there."""

  def _scc_enabled(self, op_long: bool, pcm_cruise_speed: bool) -> bool:
    cp = structs.CarParams(brand="mazda", openpilotLongitudinalControl=op_long, longitudinalActuatorDelay=0.36)
    planner = LongitudinalPlannerSP(cp, structs.CarParamsSP(pcmCruiseSpeed=pcm_cruise_speed), MagicMock())
    planner.scc.update = MagicMock()
    planner.resolver.update = MagicMock()
    planner.resolver.distance = 0.
    planner.sla.update = MagicMock()
    cc = messaging.new_message('carControl').carControl
    cc.enabled = True
    sm = {'carState': messaging.new_message('carState').carState, 'carControl': cc,
          'carStateSP': messaging.new_message('carStateSP').carStateSP}
    planner.update_targets(sm, 20., 0., 25.)
    return planner.scc.update.call_args.args[1]

  def test_stock_acc_without_icbm_keeps_scc_idle(self):
    assert not self._scc_enabled(op_long=False, pcm_cruise_speed=True)

  def test_icbm_or_op_long_lets_scc_run(self):
    assert self._scc_enabled(op_long=False, pcm_cruise_speed=False)
    assert self._scc_enabled(op_long=True, pcm_cruise_speed=True)


class TestLimiterDecel(OpenpilotTestCase):
  """Upstream caps the cruise candidate at A_CRUISE_MIN; SCC keeps its own budget, SLA coasts."""

  def make(self, source, v_target, v_ego=20., a_prev=-OP_LONG_A_BUDGET):
    cp = structs.CarParams(brand="mazda", openpilotLongitudinalControl=True)
    planner = LongitudinalPlannerSP(cp, structs.CarParamsSP(), MagicMock())
    planner.source, planner.output_v_target = source, v_target
    planner.v_ego, planner.a_cruise_prev = v_ego, a_prev
    return planner

  def test_scc_brakes_past_the_cruise_floor_to_its_budget(self):
    for source in SCC_SOURCES:
      assert self.make(source, 10.).update_cruise_accel(-0.5) == -OP_LONG_A_BUDGET

  def test_jerk_limited_from_last_frame(self):
    # a limiter appearing at full budget eases in from last frame's candidate
    assert -0.1 < self.make(LongitudinalPlanSource.sccVision, 10., a_prev=0.).update_cruise_accel(0.) < 0.

  def test_set_speed_drop_and_sla_keep_upstreams_floor(self):
    for source in (LongitudinalPlanSource.cruise, LongitudinalPlanSource.speedLimitAssist):
      assert self.make(source, 10.).update_cruise_accel(-0.5) == -0.5

  def test_limiter_above_the_car_leaves_cruise_alone(self):
    # a curve target above the car: the cruise candidate keeps accelerating toward it
    assert self.make(LongitudinalPlanSource.sccVision, 22., v_ego=18., a_prev=0.5).update_cruise_accel(0.5) == 0.5

  def test_never_raises_the_cruise_candidate(self):
    assert self.make(LongitudinalPlanSource.sccMap, 19.8, a_prev=-0.5).update_cruise_accel(-0.5) == -0.5
