"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Vision controller on stock ACC: the commit hold below the bias band and the escalation
ceiling keyed on the set speed. Road rendering and the base case live in vision_harness.py.
"""
import numpy as np
import pytest

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.speed_profile import lead_distance, required_decel
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import vision_controller
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.vision_controller import SmartCruiseControlVision
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot.tests.vision_harness import (
  VisionCase, curve_at, flat_ceiling, make_cp)

V = 15.  # m/s, inside the hold band
BEND_S, BEND_L, BEND_KAPPA = 130., 20., 0.015  # a short r=67 m bend, first read 130 m out
SEEN_FRAMES = round(vision_controller._HOLD_SEEN_T / DT_MDL)


def bend(start: float = BEND_S, kappa: float = BEND_KAPPA, background: float = 0.):
  return lambda s: kappa if start <= s <= start + BEND_L else background


def straight(s):
  return 0.


class TestCommitHold(VisionCase):

  def drive(self, scc, road, frames, x=0., v=V):
    """Drive road (kappa by absolute position) at constant v from x; returns the new x."""
    for _ in range(frames):
      self.run_road(v, lambda s, x=x: road(s + x), n=1, setpoint=v, scc=scc)
      x += v * DT_MDL
    return x

  def test_committed_bend_is_held_until_the_near_window_reaches_it(self):
    scc = self.stock()
    lim = scc.limits
    x = self.drive(scc, bend(), SEEN_FRAMES - 1)
    assert scc.is_active
    assert not np.isfinite(scc.v_dip_held)  # committed, but not bound long enough yet
    x = self.drive(scc, bend(), 40 - SEEN_FRAMES + 1, x)
    v_held = scc.v_dip_held
    held_at = x - V * DT_MDL + scc.d_held  # where the latched dip lies on the road
    assert BEND_S <= held_at <= BEND_S + BEND_L
    assert v_held == scc.v_dip_ahead < V

    # the read drops the bend 100 m out: the plan keeps it until the near window reaches it
    near_d = V * vision_controller._NEAR_T
    for _ in range(200):
      x = self.drive(scc, straight, 1, x)
      if scc.d_held <= near_d:
        break
      assert scc.is_active
      assert scc.output_v_target <= v_held
      assert scc.v_ahead_min == v_held
      a_held = min(required_decel(V, [v_held], [scc.d_held], lead_distance(V, lim.t_lead)), lim.a_budget)
      assert scc.a_needed >= a_held
      # the approach already ramped the wire; a held bend never asks past the budget
      assert -lim.a_budget - 1e-9 <= scc.output_a_target <= -a_held + 1e-9
    else:
      pytest.fail("the hold never ended")
    # handed to the near field, which reads a straight road
    assert x - V * DT_MDL == pytest.approx(held_at - near_d, abs=V * DT_MDL)
    assert not scc.is_active
    assert scc.output_v_target == V_CRUISE_UNSET
    assert not np.isfinite(scc.v_dip_held)

  def test_a_flicker_is_not_held(self):
    # the same bend read for 0.5 s commits, but a phantom binds too briefly to latch
    scc = self.stock()
    x = self.drive(scc, bend(), 10)
    assert scc.is_active
    self.drive(scc, straight, 1, x)
    assert not scc.is_active
    assert scc.output_v_target == V_CRUISE_UNSET
    assert not np.isfinite(scc.v_dip_held)

  @pytest.mark.parametrize("v, op_long, start", [(25., False, 110.), (V, True, 60.)], ids=["stock_56mph", "op_long"])
  def test_no_hold_above_the_band_or_on_op_long(self, v, op_long, start):
    scc = SmartCruiseControlVision(make_cp(op_long=op_long))
    x = self.drive(scc, bend(start), 40, v=v)  # 2 s, past the latch time
    assert scc.is_active
    assert not np.isfinite(scc.v_dip_held)
    self.drive(scc, straight, 1, x, v=v)
    assert not scc.is_active
    assert scc.output_v_target == V_CRUISE_UNSET

  def test_speeding_up_past_the_band_drops_the_hold(self):
    scc = self.stock()
    x = self.drive(scc, bend(), 40)
    assert np.isfinite(scc.v_dip_held)
    self.drive(scc, straight, 2, x, v=vision_controller._HOLD_V_MAX)
    assert not np.isfinite(scc.v_dip_held)
    assert not scc.is_active

  def test_a_deeper_bend_re_latches(self):
    scc = self.stock()
    x = self.drive(scc, bend(), 40)
    x = self.drive(scc, straight, 10, x)
    v_first = scc.v_dip_held
    assert np.isfinite(v_first)
    # a tighter bend comes into view while the first is held: planned at once, latched once it
    # has bound as long as any other
    x0 = x
    deeper = bend(x0 + BEND_S, kappa=2 * BEND_KAPPA)
    x = self.drive(scc, deeper, SEEN_FRAMES - 1, x)
    assert scc.v_dip_held == v_first
    assert scc.v_dip_ahead < v_first - 2.
    assert scc.output_v_target == pytest.approx(scc.v_dip_ahead)
    x = self.drive(scc, deeper, 1, x)
    assert scc.v_dip_held == pytest.approx(scc.v_dip_ahead)
    assert scc.v_dip_held < v_first - 2.
    assert x0 + BEND_S <= x - V * DT_MDL + scc.d_held <= x0 + BEND_S + BEND_L

  def test_hold_survives_a_slightly_curved_path(self):
    # a real model path is never exactly straight: r=10 km under the dropped bend reads a
    # finite allowed speed (131 m/s), which must neither end the hold nor displace the held dip
    bg = 1e-4
    scc = self.stock()
    x = self.drive(scc, bend(background=bg), 40)
    self.drive(scc, lambda s: bg, 15, x)
    assert scc.is_active
    assert scc.v_ahead_min == scc.v_dip_held
    assert scc.output_v_target <= scc.v_dip_held
    assert scc.output_a_target <= -0.5 * scc.limits.a_budget


class TestEscalationCeiling(VisionCase):
  """Only a measured near bend may ask past the budget, and only cruising at or below 50 mph."""

  def run_setpoint(self, setpoint):
    # a measured r=167 m bend 40 m out at 45 mph: late, it needs about four times the budget
    scc = SmartCruiseControlVision(make_cp(op_long=False))
    self.run_road(20., curve_at(40., 0.006), n=40, setpoint=setpoint, scc=scc)
    return scc

  def test_ceiling_follows_the_set_speed(self):
    # a flat lateral ceiling isolates the escalation fade from the set-speed ceiling
    with flat_ceiling(1.8):
      lo, mid, hi = (self.run_setpoint(sp) for sp in (20., 24.6, 27.))
    budget = lo.limits.a_budget
    # cruising at 45 mph arriving hot costs more than braking hard
    assert lo.a_needed > 2 * budget
    assert lo.output_a_target < -2 * budget
    # cruising from 60 mph the same read asks for the budget at most
    assert hi.a_needed == pytest.approx(budget)
    assert hi.output_a_target == pytest.approx(-budget)
    # and between them the ceiling fades
    assert mid.a_needed == pytest.approx((lo.a_needed + budget) / 2)
