"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import json
import math
import platform

import pytest


from openpilot.cereal import custom
from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.limits import COMMIT_FRAC
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.map_controller import R, SmartCruiseControlMap
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.speed_profile import lead_distance, required_decel
from openpilot.common.test import OpenpilotTestCase

MapState = VisionState = custom.LongitudinalPlanSP.SmartCruiseControl.MapState


class TestSmartCruiseControlMap(OpenpilotTestCase):

  def setup_method(self):
    self.params = Params()
    self.mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else self.params
    self.reset_params()
    cp = structs.CarParams(brand="mazda", openpilotLongitudinalControl=True, longitudinalActuatorDelay=0.36)
    self.scc_m = SmartCruiseControlMap(cp)

  def reset_params(self):
    self.params.put_bool("SmartCruiseControlMap", True, block=True)

    # TODO-SP: mock data from gpsLocation
    self.params.put("LastGPSPosition", "{}", block=True)
    self.params.put("MapTargetVelocities", "{}", block=True)

  def test_initial_state(self):
    assert self.scc_m.state == VisionState.disabled
    assert not self.scc_m.is_active
    assert self.scc_m.output_v_target == V_CRUISE_UNSET
    assert self.scc_m.output_a_target == 0.

  def test_system_disabled(self):
    self.params.put_bool("SmartCruiseControlMap", False, block=True)
    self.scc_m.enabled = self.params.get_bool("SmartCruiseControlMap")

    for _ in range(int(10. / DT_MDL)):
      self.scc_m.update(True, False, 0., 0., 0.)
    assert self.scc_m.state == VisionState.disabled
    assert not self.scc_m.is_active

  def test_disabled(self):
    for _ in range(int(10. / DT_MDL)):
      self.scc_m.update(False, False, 0., 0., 0.)
    assert self.scc_m.state == VisionState.disabled

  def test_transition_disabled_to_enabled(self):
    for _ in range(int(10. / DT_MDL)):
      self.scc_m.update(True, False, 0., 0., 0.)
    assert self.scc_m.state == VisionState.enabled

  def test_moderate_curve(self):
    # Regression: `... / 2 * a` parsed as `(.../2)*a` instead of `.../(2*a)`,
    # making max_d ~11x too small so the moderate-curve branch never tripped.
    # v_ego=25, a_ego=0, tv=24: fixed max_d≈45m vs buggy ≈4m at a 40m waypoint.
    waypoint_lon_deg = (40.0 / R) * (180.0 / math.pi)
    self.mem_params.put("LastGPSPosition", json.dumps({"latitude": 0.0, "longitude": 0.0}), block=True)
    self.mem_params.put("MapTargetVelocities",
                        json.dumps([{"latitude": 0.0, "longitude": waypoint_lon_deg, "velocity": 24.0}]), block=True)

    self.scc_m.update(True, False, 25.0, 0.0, 30.0)

    self.assertAlmostEqual(self.scc_m.v_target, 24.0, delta=24.0 * 1e-6)

  # TODO-SP: mock data from modelV2 to test other states

  def test_active_target_publishes_required_decel(self):
    lat0, lon0 = 32.0, -117.0
    dlat = 200.0 / 111194.9  # ~200 m north
    self.mem_params.put("LastGPSPosition", json.dumps({"latitude": lat0, "longitude": lon0}), block=True)
    self.mem_params.put("MapTargetVelocities", json.dumps([
      {"latitude": lat0, "longitude": lon0, "velocity": 30.0},
      {"latitude": lat0 + dlat, "longitude": lon0, "velocity": 15.0},
    ]), block=True)

    v_ego = 25.
    for _ in range(3):
      self.scc_m.update(True, False, v_ego, 0., 25.)
    assert self.scc_m.state == MapState.turning
    assert self.scc_m.output_v_target == pytest.approx(15.)
    assert 150. < self.scc_m.target_distance < 250.

    # required decel to the target, reached through the publication ramp
    expected = (15. ** 2 - v_ego ** 2) / (2. * self.scc_m.target_distance)
    for _ in range(40):
      self.scc_m.update(True, False, v_ego, 0., 25.)
    assert self.scc_m.output_a_target == pytest.approx(expected, abs=1e-3)

  def test_retained_target_tracks_distance(self):
    """A target that slips under the commit gate while still ahead (the car is already
    slowing) is retained. Its distance must keep tracking the car: the published decel
    divides by it every frame, and frozen at the commit-time value it under-requests
    more the closer the car gets."""
    lat0, lon0 = 32.0, -117.0
    m_per_deg = 111194.9
    target_lat = lat0 + 200.0 / m_per_deg
    self.mem_params.put("LastGPSPosition", json.dumps({"latitude": lat0, "longitude": lon0}), block=True)
    self.mem_params.put("MapTargetVelocities", json.dumps([
      {"latitude": lat0, "longitude": lon0, "velocity": 30.0},
      {"latitude": target_lat, "longitude": lon0, "velocity": 15.0},
    ]), block=True)
    for _ in range(3):
      self.scc_m.update(True, False, 25., 0., 25.)
    assert self.scc_m.state == MapState.turning

    # slowing harder than the gate needs while closing: the target leaves valid_velocities
    # after a few frames (a_req falls under COMMIT_FRAC * a_budget) but stays ahead
    v_ego, d = 25., 200.
    for _ in range(12):
      v_ego -= 0.7
      d -= 6.
      self.mem_params.put("LastGPSPosition", json.dumps({"latitude": target_lat - d / m_per_deg, "longitude": lon0}), block=True)
      self.scc_m.update(True, False, v_ego, 0., 25.)
      assert self.scc_m.is_active and self.scc_m.v_target == pytest.approx(15.)
      assert self.scc_m.target_distance == pytest.approx(d, abs=1.), "retained target kept its commit-time distance"

    # hold the state so the ramp settles: the decel must be the one for the current distance
    for _ in range(40):
      self.scc_m.update(True, False, v_ego, 0., 25.)
    expected = (15. ** 2 - v_ego ** 2) / (2. * max(d, v_ego * 2.8))
    assert self.scc_m.output_a_target == pytest.approx(expected, abs=1e-3)

  def test_commit_gate_and_decel_are_the_shared_solver(self):
    # The commit gate and output must share speed_profile.required_decel.
    lat0, lon0 = 32.0, -117.0
    self.mem_params.put("LastGPSPosition", json.dumps({"latitude": lat0, "longitude": lon0}), block=True)
    self.mem_params.put("MapTargetVelocities", json.dumps([
      {"latitude": lat0, "longitude": lon0, "velocity": 30.0},
      {"latitude": lat0 + 200.0 / 111194.9, "longitude": lon0, "velocity": 15.0},
    ]), block=True)
    v_ego = 25.
    for _ in range(3):
      self.scc_m.update(True, False, v_ego, 0., 25.)
    lim = self.scc_m.limits
    d = self.scc_m.target_distance
    d_lead = lead_distance(v_ego, lim.t_lead + lim.dash_traversal_time(v_ego - 15.), lim.a_budget, lim.jerk(v_ego))
    a_req = required_decel(v_ego, [15.], [d], d_lead)
    assert a_req >= COMMIT_FRAC * lim.a_budget
    assert self.scc_m.is_active
    for _ in range(60):
      self.scc_m.update(True, False, v_ego, 0., 25.)
    assert self.scc_m.output_a_target == pytest.approx(-required_decel(v_ego, [15.], [max(d, v_ego * 2.8)]), abs=1e-3)

  def _steep_target(self, scc, v_ego=25.):
    # 5 m/s target ~200 m ahead: raw required decel 1.5 m/s2, past the op-long budget
    lat0, lon0 = 32.0, -117.0
    dlat = 200.0 / 111194.9
    self.mem_params.put("LastGPSPosition", json.dumps({"latitude": lat0, "longitude": lon0}), block=True)
    self.mem_params.put("MapTargetVelocities", json.dumps([
      {"latitude": lat0, "longitude": lon0, "velocity": 30.0},
      {"latitude": lat0 + dlat, "longitude": lon0, "velocity": 5.0},
    ]), block=True)
    for _ in range(60):
      scc.update(True, False, v_ego, 0., v_ego)
    assert scc.is_active
    return (5. ** 2 - v_ego ** 2) / (2. * scc.target_distance)

  def test_op_long_published_decel_is_clipped_to_the_budget(self):
    # the plan aTarget seeds the MPC on openpilot long, and stage 0 is pinned to the seed,
    # so anything under the budget would bypass A_CRUISE_MIN through the MPC candidate
    raw = self._steep_target(self.scc_m)
    assert raw < -1.2
    assert self.scc_m.output_a_target == pytest.approx(-self.scc_m.limits.a_budget, abs=1e-6)

  def test_stock_path_keeps_the_dash_lever_depth(self):
    # on stock ACC the wire is ICBM's overshoot lever, not an actuator command
    cp = structs.CarParams(brand="mazda", openpilotLongitudinalControl=False)
    stock = SmartCruiseControlMap(cp)
    raw = self._steep_target(stock)
    assert stock.output_a_target == pytest.approx(raw, abs=1e-3)
    assert stock.output_a_target < -1.2
