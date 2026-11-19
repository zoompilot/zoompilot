"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The planner's e2e candidate runs whenever openpilot drives the gas and brakes (chill mode went
2026-11). Also the fixture the planner hook tests build on.
"""
import pytest

from openpilot.cereal import custom, messaging
from opendbc.car import structs
from opendbc.car.interfaces import ACCEL_MAX
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner, LongitudinalPlanSource

V_EGO = 20.0
E2E_ACCEL = -3.0  # low enough that e2e wins the min() whenever it is a candidate


class MockSubMaster(dict):
  def __init__(self, services: dict):
    super().__init__(services)
    self.valid = dict.fromkeys(services, True)
    self.logMonoTime = dict.fromkeys(services, 0)
    self.updated = dict.fromkeys(services, True)
    self.recv_frame = dict.fromkeys(services, 1)

  def all_checks(self, service_list=None) -> bool:
    return True


def build_sm() -> MockSubMaster:
  services = {}
  for service in ("radarState", "carStateSP", "liveMapDataSP", "gpsLocationExternal", "gpsLocation"):
    services[service] = getattr(messaging.new_message(service), service)

  # engaged on openpilot long: longcontrol running, so the planner does not reset
  controls_state = messaging.new_message('controlsState')
  controls_state.controlsState.longControlState = LongCtrlState.pid
  services['controlsState'] = controls_state.controlsState.as_reader()

  car_state = messaging.new_message('carState')
  car_state.carState.vEgo = V_EGO
  car_state.carState.vCruise = 100.0
  car_state.carState.vCruiseCluster = 100.0
  services['carState'] = car_state.carState.as_reader()

  selfdrive_state = messaging.new_message('selfdriveState')
  selfdrive_state.selfdriveState.enabled = True
  services['selfdriveState'] = selfdrive_state.selfdriveState.as_reader()

  car_control = messaging.new_message('carControl')
  car_control.carControl.enabled = True
  services['carControl'] = car_control.carControl.as_reader()

  model = messaging.new_message('modelV2')
  model.modelV2.orientationRate.z = [0.01] * 33  # nonzero: a straight path divides by zero in SCC vision
  model.modelV2.velocity.x = [V_EGO] * 33
  model.modelV2.position.x = [float(i) for i in range(33)]
  model.modelV2.action.desiredAcceleration = E2E_ACCEL
  services['modelV2'] = model.modelV2.as_reader()

  return MockSubMaster(services)


def build_planner(op_long: bool = True) -> LongitudinalPlanner:
  CP = structs.CarParams()
  CP.longitudinalActuatorDelay = 0.2
  CP.openpilotLongitudinalControl = op_long
  CP_SP = custom.CarParamsSP.new_message().as_reader()
  return LongitudinalPlanner(CP, CP_SP, init_v=V_EGO)


class TestExperimentalGate(OpenpilotTestCase):
  def _source(self, op_long: bool = True) -> LongitudinalPlanSource:
    planner = build_planner(op_long)
    planner.update(build_sm())
    return planner.mpc.source

  def test_e2e_on_openpilot_long(self):
    assert self._source() == LongitudinalPlanSource.e2e

  def test_no_e2e_without_openpilot_long(self):
    assert self._source(op_long=False) != LongitudinalPlanSource.e2e

  def test_without_openpilot_long_the_candidate_never_wins_or_stops(self):
    planner = build_planner(op_long=False)
    sm = build_sm()
    model = sm['modelV2'].as_builder()
    model.action.shouldStop = True
    sm['modelV2'] = model.as_reader()
    assert planner.update_e2e_target(sm, E2E_ACCEL, True, False) == (ACCEL_MAX, False)
    planner.update(sm)
    assert not planner.output_should_stop

  def test_cornering_estimate_is_controlsds_curvature(self):
    # plannerd no longer subscribes to vehicleParameters: the lifts read controlsState's curvature
    planner = build_planner()
    sm = build_sm()
    controls_state = sm['controlsState'].as_builder()
    controls_state.curvature = 0.002
    sm['controlsState'] = controls_state.as_reader()
    seen = []
    update = planner.e2e_set_speed.update
    planner.e2e_set_speed.update = lambda *args: seen.append(args[-1]) or update(*args)
    planner.update(sm)
    assert seen == [pytest.approx(0.002 * V_EGO ** 2)]
