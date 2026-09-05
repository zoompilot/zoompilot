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
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP

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
