"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
from unittest.mock import MagicMock

import pytest

import openpilot.cereal.messaging as messaging
from openpilot.cereal import custom
from opendbc.car import structs
from openpilot.common.params import Params
from openpilot.selfdrive.car.cruise import VCruiseHelper
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import IntelligentCruiseButtonManagement
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.demand import CONSUMERS, icbm_demanded
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.icbm_latch import IcbmActivation, IcbmLatch, icbm_active
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.migration import LEGACY_TOGGLE, MIGRATED, migrate_icbm_toggle
from openpilot.sunnypilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlannerSP
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode

ButtonType = structs.CarState.ButtonEvent.Type
CONSUMER_KEYS = tuple(c.key for c in CONSUMERS if isinstance(c.on, bool))


def _cp(op_long=False, pcm_cruise=True):
  return structs.CarParams(brand="mazda", pcmCruise=pcm_cruise, openpilotLongitudinalControl=op_long,
                           longitudinalActuatorDelay=0.36)


def _cp_sp(active=False, available=True):
  return structs.CarParamsSP(pcmCruiseSpeed=not active, intelligentCruiseButtonManagementAvailable=available)


def _cs_sp(activation=None):
  msg = messaging.new_message('carStateSP')
  if activation is not None:
    msg.carStateSP.zoompilot.icbmActivation = activation
  return messaging.log_from_bytes(msg.to_bytes()).carStateSP


def _cs(cruise_enabled=False, buttons=()):
  CS = structs.CarState()
  CS.cruiseState.available = True
  CS.cruiseState.enabled = cruise_enabled
  CS.buttonEvents = [structs.CarState.ButtonEvent(type=b, pressed=p) for b, p in buttons]
  return CS


@pytest.fixture
def params():
  p = Params()
  for key in CONSUMER_KEYS + (LEGACY_TOGGLE, MIGRATED):
    p.remove(key)
  p.put("SpeedLimitMode", int(Mode.warning), block=True)
  return p


class TestLatch:
  @pytest.mark.parametrize("active", [True, False])
  def test_starts_at_the_boot_decision(self, active):
    latch = IcbmLatch(_cp(), _cp_sp(active))
    assert latch.active == active
    assert latch.activation == (IcbmActivation.active if active else IcbmActivation.passive)

  def test_follows_demand_only_while_disengaged(self):
    latch = IcbmLatch(_cp(), _cp_sp(False))
    latch.demanded = True
    assert not latch.update(engaged=True) and not latch.active
    assert latch.update(engaged=False) and latch.active
    latch.demanded = False
    assert not latch.update(engaged=True) and latch.active  # dropped mid-drive: kept to the disengage
    assert latch.update(engaged=False) and not latch.active

  def test_never_on_a_car_without_the_buttons(self):
    for CP, CP_SP in ((_cp(), _cp_sp(available=False)),
                      (_cp(op_long=True, pcm_cruise=False), _cp_sp())):  # openpilot owns the setpoint
      latch = IcbmLatch(CP, CP_SP)
      latch.demanded = True
      assert not latch.update(engaged=False) and not latch.active


class TestPublishedActivation:
  @pytest.mark.parametrize("pcm_cruise_speed", [True, False])
  def test_unset_falls_back_to_the_boot_flag(self, pcm_cruise_speed):
    # a log from before the field, or no carStateSP received yet
    CP_SP = custom.CarParamsSP.new_message(pcmCruiseSpeed=pcm_cruise_speed)
    assert icbm_active(_cs_sp(), CP_SP) == (not pcm_cruise_speed)
    assert icbm_active(_cs_sp(IcbmActivation.active), CP_SP)
    assert not icbm_active(_cs_sp(IcbmActivation.passive), CP_SP)

  def test_servo_follows_card_and_starts_clean(self):
    servo = IntelligentCruiseButtonManagement(_cp(), _cp_sp(False))
    assert not servo.active
    servo.fast_faulted = True
    servo.pre_active_timer = 7
    servo.update_activation(_cs_sp(IcbmActivation.active))
    assert servo.active and servo.pre_active_timer == 0
    assert servo.fast_faulted  # what this drive learned about the dash survives


class TestDemand:
  @pytest.mark.parametrize("key", CONSUMER_KEYS)
  def test_stock_acc_consumers(self, params, key):
    assert not icbm_demanded(_cp(), params)
    params.put_bool(key, True, block=True)
    assert icbm_demanded(_cp(), params)

  def test_sla_assist(self, params):
    params.put("SpeedLimitMode", int(Mode.assist), block=True)
    assert icbm_demanded(_cp(), params) and icbm_demanded(_cp(op_long=True), params)

  def test_alpha_long_runs_scc_without_the_buttons(self, params):
    params.put_bool("SmartCruiseControlVision", True, block=True)
    params.put_bool("SmartCruiseControlMap", True, block=True)
    assert not icbm_demanded(_cp(op_long=True), params)
    params.put_bool("CustomAccIncrementsEnabled", True, block=True)
    assert icbm_demanded(_cp(op_long=True), params)


class TestMigration:
  def _all_on(self, params):
    for key in CONSUMER_KEYS:
      params.put_bool(key, True, block=True)
    params.put("SpeedLimitMode", int(Mode.assist), block=True)

  def test_toggle_off_stock_acc_clears_what_was_inert(self, params):
    self._all_on(params)
    migrate_icbm_toggle(_cp(), _cp_sp(), params)
    assert not any(params.get_bool(k) for k in CONSUMER_KEYS)
    assert params.get("SpeedLimitMode", return_default=True) == Mode.warning
    assert not icbm_demanded(_cp(), params)
    assert params.get_bool(MIGRATED) and params.get(LEGACY_TOGGLE) is None

  def test_toggle_off_alpha_long_keeps_what_the_planner_ran(self, params):
    self._all_on(params)
    migrate_icbm_toggle(_cp(op_long=True), _cp_sp(), params)
    assert params.get_bool("SmartCruiseControlVision") and params.get_bool("SmartCruiseControlMap")
    assert not params.get_bool("CustomAccIncrementsEnabled")
    assert params.get("SpeedLimitMode", return_default=True) == Mode.assist  # brings ICBM up, decided 10-07

  def test_toggle_on_keeps_everything(self, params):
    self._all_on(params)
    params.put_bool(LEGACY_TOGGLE, True, block=True)
    migrate_icbm_toggle(_cp(), _cp_sp(), params)
    assert all(params.get_bool(k) for k in CONSUMER_KEYS)
    assert params.get("SpeedLimitMode", return_default=True) == Mode.assist

  def test_boot_without_the_buttons_waits_for_one_with_them(self, params):
    # a mock fingerprint, or a long mode the buttons can't act in, decides nothing
    self._all_on(params)
    migrate_icbm_toggle(_cp(), _cp_sp(available=False), params)
    migrate_icbm_toggle(_cp(op_long=True, pcm_cruise=False), _cp_sp(), params)
    assert all(params.get_bool(k) for k in CONSUMER_KEYS)
    assert not params.get_bool(MIGRATED)
    migrate_icbm_toggle(_cp(), _cp_sp(), params)
    assert not any(params.get_bool(k) for k in CONSUMER_KEYS)
    assert params.get_bool(MIGRATED)

  def test_a_failed_migration_retries(self, params, monkeypatch):
    self._all_on(params)
    real_put_bool = Params.put_bool

    def failing_put_bool(self, key, *args, **kwargs):
      if key == "SmartCruiseControlMap":
        raise OSError("disk")
      return real_put_bool(self, key, *args, **kwargs)
    monkeypatch.setattr(Params, "put_bool", failing_put_bool)
    migrate_icbm_toggle(_cp(), _cp_sp(), params)
    monkeypatch.undo()
    assert not params.get_bool(MIGRATED)
    migrate_icbm_toggle(_cp(), _cp_sp(), params)
    assert not any(params.get_bool(k) for k in CONSUMER_KEYS) and params.get_bool(MIGRATED)

  def test_runs_once(self, params):
    migrate_icbm_toggle(_cp(), _cp_sp(), params)
    params.put_bool("SmartCruiseControlVision", True, block=True)
    migrate_icbm_toggle(_cp(), _cp_sp(), params)
    assert params.get_bool("SmartCruiseControlVision")


class TestCardHandover:
  def _helper(self, params, active=False, op_long=False):
    helper = VCruiseHelper(_cp(op_long=op_long), _cp_sp(active))
    helper.read_custom_set_speed_params()
    return helper

  def test_turning_a_feature_on_mid_drive_waits_for_the_disengage(self, params):
    helper = self._helper(params)
    params.put_bool("SmartCruiseControlVision", True, block=True)
    helper.read_custom_set_speed_params()
    helper.update_v_cruise(_cs(cruise_enabled=True), True, False)
    assert helper.pcm_cruise_speed  # engaged: unchanged
    helper.update_v_cruise(_cs(cruise_enabled=False), False, False)
    assert not helper.pcm_cruise_speed

  def test_a_held_button_holds_the_switch_until_it_settles(self, params):
    helper = self._helper(params, active=True)
    helper.read_custom_set_speed_params()  # nothing demands ICBM
    helper.update_v_cruise(_cs(buttons=[(ButtonType.accelCruise, True)]), False, False)
    helper.update_v_cruise(_cs(), False, False)
    assert not helper.pcm_cruise_speed  # still held: no switch
    helper.update_v_cruise(_cs(buttons=[(ButtonType.accelCruise, False)]), False, False)
    assert not helper.pcm_cruise_speed  # release frame: cruise.py's timers settle this frame
    helper.update_v_cruise(_cs(), False, False)
    assert helper.pcm_cruise_speed
    assert not any(helper.button_timers.values()) and not any(helper.enable_button_timers.values())

  def test_alpha_long_sla_owner_moves_with_icbm(self, params):
    params.put("SpeedLimitMode", int(Mode.assist), block=True)
    helper = self._helper(params, active=True, op_long=True)
    assert helper.cruise_arbiter.applicable
    params.put("SpeedLimitMode", int(Mode.warning), block=True)
    helper.read_custom_set_speed_params()
    helper.update_v_cruise(_cs(), False, False)
    assert helper.pcm_cruise_speed and not helper.cruise_arbiter.applicable


class TestPlannerHandover:
  def _targets(self, planner, activation):
    cc = messaging.new_message('carControl').carControl
    sm = {'carState': messaging.new_message('carState').carState, 'carControl': cc, 'carStateSP': _cs_sp(activation)}
    planner.update_targets(sm, 20., 0., 25.)

  def test_alpha_long_switches_sla_owner(self, params):
    planner = LongitudinalPlannerSP(_cp(op_long=True), _cp_sp(False), MagicMock())
    planner.resolver.update = MagicMock()
    planner.resolver.distance = 0.
    assert planner.sla is planner.sla_machine
    self._targets(planner, IcbmActivation.active)
    assert planner.sla is planner.sla_mirror
    self._targets(planner, IcbmActivation.passive)
    assert planner.sla is planner.sla_machine

  def test_stock_acc_always_mirrors(self, params):
    planner = LongitudinalPlannerSP(_cp(), _cp_sp(False), MagicMock())
    assert planner.sla_machine is None and planner.sla is planner.sla_mirror
