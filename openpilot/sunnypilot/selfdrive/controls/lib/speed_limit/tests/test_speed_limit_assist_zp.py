"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

What zoompilot's pcm machine changes over sunnypilot's: the ramped decel it publishes, when it
announces a limit, the set speed as a ceiling instead of a required max, and the gas hold.
"""
import pytest

from openpilot.cereal import custom
from opendbc.car.car_helpers import interfaces
from opendbc.car.toyota.values import CAR as TOYOTA
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.selfdrive.car import interfaces as sunnypilot_interfaces
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import PCM_LONG_REQUIRED_MAX_SET_SPEED
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import ACTIVE_STATES, PRE_ACTIVE_GUARD_PERIOD, V_CRUISE_UNSET
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist_zp import SpeedLimitAssistZP
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

SpeedLimitAssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
EventNameSP = custom.OnroadEventSP.EventName

SPEED_LIMITS = {
  'highway': 65 * CV.MPH_TO_MS,
  'freeway': 80 * CV.MPH_TO_MS,
}

CAR = TOYOTA.TOYOTA_RAV4_TSS2


class TestSpeedLimitAssistZP:
  def setup_method(self):
    params = Params()
    params.put("IsReleaseSpBranch", True, block=True)
    params.put("SpeedLimitMode", int(Mode.assist), block=True)
    params.put_bool("IsMetric", False, block=True)
    params.put("SpeedLimitOffsetType", 0, block=True)
    params.put("SpeedLimitValueOffset", 0, block=True)
    CarInterface = interfaces[CAR]
    CP = CarInterface.get_non_essential_params(CAR)
    CI = CarInterface(CP, CarInterface.get_non_essential_params_sp(CP, CAR))
    CI.CP.openpilotLongitudinalControl = True
    sunnypilot_interfaces.setup_interfaces(CI, params)
    self.sla = SpeedLimitAssistZP(CI.CP, CI.CP_SP)
    self.sla.pre_active_timer = int(PRE_ACTIVE_GUARD_PERIOD[self.sla.pcm_op_long] / DT_MDL)
    self.events_sp = EventsSP()
    self.pcm_long_max_set_speed = PCM_LONG_REQUIRED_MAX_SET_SPEED[self.sla.is_metric][1]
    self.speed_conv = CV.MS_TO_KPH if self.sla.is_metric else CV.MS_TO_MPH

  def initialize_active_state(self, v_cruise):
    self.sla.state = SpeedLimitAssistState.active
    self.sla.v_cruise_cluster = v_cruise
    self.sla.v_cruise_cluster_prev = v_cruise
    self.sla.prev_v_cruise_cluster_conv = round(v_cruise * self.speed_conv)

  def adapt(self, distance: float, frames: int = 60) -> float:
    """Adapt from freeway to highway speed over distance; returns the decel the sign requires."""
    self.sla.state = SpeedLimitAssistState.adapting
    self.sla.v_cruise_cluster_prev = self.pcm_long_max_set_speed
    self.sla.prev_v_cruise_cluster_conv = round(self.pcm_long_max_set_speed * self.speed_conv)
    current_speed, target_speed = SPEED_LIMITS['freeway'], SPEED_LIMITS['highway']
    for _ in range(frames):
      self.sla.update(True, False, current_speed, 0, self.pcm_long_max_set_speed, target_speed, target_speed, True, distance, self.events_sp)
    assert self.sla.state == SpeedLimitAssistState.adapting
    return (target_speed ** 2 - current_speed ** 2) / (2. * distance)

  def test_adapting_decel_past_the_budget_stops_at_it(self):
    # pcm-op-long is openpilot long: the wire seeds the MPC, so it ramps to the budget and stops
    expected = self.adapt(100.0)
    assert expected < -self.sla.limits.a_budget
    assert self.sla.output_a_target == pytest.approx(-self.sla.limits.a_budget, abs=1e-3)

  def test_adapting_decel_inside_the_budget_is_published_exactly(self):
    expected = self.adapt(300.0)
    assert -self.sla.limits.a_budget < expected < 0.
    assert self.sla.output_a_target == pytest.approx(expected, abs=1e-3)

  # announcements: "Adjusting to" is raised only when the speed the car settles at,
  # min(target, set speed), moves; the cruise arbiter follows the same rule

  def step(self, cluster_mph, limit_mph, raw_limit_mph=None):
    self.events_sp.clear()
    raw = limit_mph if raw_limit_mph is None else raw_limit_mph
    self.sla.update(True, False, limit_mph * CV.MPH_TO_MS, 0, cluster_mph * CV.MPH_TO_MS, raw * CV.MPH_TO_MS,
                    limit_mph * CV.MPH_TO_MS, True, 0, self.events_sp)

  def announces(self, cluster_mph, limit_mph, raw_limit_mph=None) -> bool:
    self.step(cluster_mph, limit_mph, raw_limit_mph)
    return self.events_sp.has(EventNameSP.speedLimitActive) or self.events_sp.has(EventNameSP.speedLimitChanged)

  def active_at(self, cluster_mph, limit_mph):
    self.initialize_active_state(cluster_mph * CV.MPH_TO_MS)
    self.step(cluster_mph, limit_mph)
    assert self.sla.state in ACTIVE_STATES

  def test_limit_above_the_set_speed_is_silent(self):
    self.active_at(80, 80)
    assert not self.announces(80, 85)
    assert self.sla.state in ACTIVE_STATES

  def test_limit_change_announces(self):
    self.active_at(80, 65)
    assert self.announces(80, 55)

  def test_limit_dropout_and_return_is_silent(self):
    self.active_at(80, 65)
    assert not self.announces(80, 65, raw_limit_mph=0)
    assert not self.announces(80, 65)

  # no required max: with the setting on, the limit caps the plan at any set speed

  def drive(self, cluster_mph, limit_mph, v_mph=None, gas=False, enabled=True, frames=1):
    v = (limit_mph if v_mph is None else v_mph) * CV.MPH_TO_MS
    limit = limit_mph * CV.MPH_TO_MS
    for _ in range(frames):
      self.events_sp.clear()
      self.sla.update(enabled, gas, v, 0, cluster_mph * CV.MPH_TO_MS, limit, limit, limit_mph > 0, 0, self.events_sp)

  def target_mph(self) -> float:
    return self.sla.output_v_target * CV.MS_TO_MPH

  def engage(self, cluster_mph=60, limit_mph=35):
    self.drive(cluster_mph, limit_mph, frames=int(1. / DT_MDL))
    assert self.sla.state in ACTIVE_STATES

  def test_engages_at_any_set_speed(self):
    self.engage(60, 35)
    assert self.target_mph() == pytest.approx(35)
    assert self.sla.state != SpeedLimitAssistState.preActive

  def test_no_limit_waits_pending_then_activates(self):
    self.drive(60, 0, v_mph=30, frames=int(1. / DT_MDL))
    assert self.sla.state == SpeedLimitAssistState.pending
    self.drive(60, 35)
    assert self.sla.state in ACTIVE_STATES

  def test_set_speed_change_keeps_the_session(self):
    self.engage(60, 35)
    self.drive(61, 35)
    assert self.sla.state in ACTIVE_STATES
    assert self.target_mph() == pytest.approx(35)

  def test_needs_the_setting(self):
    self.sla.enabled = False
    Params().put("SpeedLimitMode", int(Mode.warning), block=True)
    self.drive(60, 35, frames=int(1. / DT_MDL))
    assert self.sla.state == SpeedLimitAssistState.disabled
    assert self.sla.output_v_target == V_CRUISE_UNSET

  def test_new_limit_applies_without_a_prompt(self):
    self.engage(60, 35)
    self.drive(60, 25, v_mph=35)
    assert self.sla.state in ACTIVE_STATES
    assert self.events_sp.has(EventNameSP.speedLimitActive) or self.events_sp.has(EventNameSP.speedLimitChanged)
    assert self.target_mph() == pytest.approx(25)

  # gas hold: releasing the gas above the limit holds the speed reached until the next sign

  def gas_to(self, cluster_mph, limit_mph, v_mph):
    self.drive(cluster_mph, limit_mph, v_mph=v_mph, gas=True, frames=5)
    self.drive(cluster_mph, limit_mph, v_mph=v_mph)

  def test_gas_release_holds_the_speed_quietly(self):
    self.engage(60, 35)
    self.gas_to(60, 35, 40)
    assert self.target_mph() == pytest.approx(40)
    assert not self.events_sp.has(EventNameSP.speedLimitActive)
    assert not self.events_sp.has(EventNameSP.speedLimitChanged)
    self.drive(60, 35, v_mph=40, frames=50)
    assert self.target_mph() == pytest.approx(40)

  def test_hold_caps_at_the_set_speed(self):
    self.engage(38, 35)
    self.gas_to(38, 35, 45)
    assert self.target_mph() == pytest.approx(38)

  def test_release_at_or_below_the_limit_holds_nothing(self):
    self.engage(60, 35)
    self.gas_to(60, 35, 35)
    assert self.sla.held_v is None
    assert self.target_mph() == pytest.approx(35)

  def test_tap_at_a_lower_sign_keeps_the_speed(self):
    """School zone: the limit drops while at speed, a tap of the gas declines it."""
    self.engage(60, 35)
    self.drive(60, 20, v_mph=35)
    assert self.target_mph() == pytest.approx(20)
    self.gas_to(60, 20, 35)
    assert self.target_mph() == pytest.approx(35)
    assert self.sla.state == SpeedLimitAssistState.active  # not adapting toward 20

  def test_hold_survives_a_limit_dropout(self):
    self.engage(60, 35)
    self.gas_to(60, 35, 40)
    self.drive(60, 0, v_mph=40, frames=5)
    self.drive(60, 35, v_mph=40)
    assert self.target_mph() == pytest.approx(40)

  @pytest.mark.parametrize("new_limit", [25, 45])
  def test_next_sign_ends_the_hold(self, new_limit):
    self.engage(60, 35)
    self.gas_to(60, 35, 40)
    self.drive(60, new_limit, v_mph=40)
    assert self.sla.held_v is None
    assert self.target_mph() == pytest.approx(new_limit)

  def test_disengage_ends_the_hold(self):
    self.engage(60, 35)
    self.gas_to(60, 35, 40)
    self.drive(60, 35, v_mph=40, enabled=False)
    self.engage(60, 35)
    assert self.target_mph() == pytest.approx(35)
