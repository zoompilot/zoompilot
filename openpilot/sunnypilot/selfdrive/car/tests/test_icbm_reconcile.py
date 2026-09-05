"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Setpoint reconciliation on ICBM (non-pcmCruiseSpeed) cars: around driver presses the dash
is the source of truth, but only when the plan source is cruise and ICBM is not mid-move
(single-writer setpoint); SLA-owned presses never increment v_cruise; the vEgo clip on
SET- while overriding is disabled.
"""
import pytest

from openpilot.cereal import custom
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.selfdrive.car.cruise import VCruiseHelper, IMPERIAL_INCREMENT
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.cruise_ext import RECONCILE_SETTLE_TIME

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type

MPH = CV.MPH_TO_KPH  # dash and v_cruise are tracked in kph; the CX-5 dash steps in whole mph


def make_car_state(dash_kph=0., gas_pressed=False, button_events=None, available=True, v_ego=0.):
  CS = car.CarState(cruiseState={"available": available, "speed": dash_kph * CV.KPH_TO_MS})
  CS.gasPressed = gas_pressed
  CS.vEgo = v_ego
  CS.buttonEvents = button_events or []
  return CS


class TestSetpointReconcile:
  """pcmCruise (stock ACC) car with ICBM enabled (pcmCruiseSpeed=False)."""

  def setup_method(self):
    Params().put_bool("CustomAccIncrementsEnabled", False)
    self.CP = car.CarParams(pcmCruise=True)
    self.CP_SP = custom.CarParamsSP(pcmCruiseSpeed=False)
    self.v_cruise_helper = VCruiseHelper(self.CP, self.CP_SP)
    self.is_metric = False  # the dash unit; kph-valued fixtures below scale by MPH or 1

  def set_regime(self, source='cruise', icbm_state='inactive', sla_state='disabled', limit_kph=0.):
    LP_SP = custom.LongitudinalPlanSP()
    LP_SP.longitudinalPlanSource = source
    LP_SP.speedLimit.assist.state = sla_state
    LP_SP.speedLimit.resolver.speedLimit = limit_kph * CV.KPH_TO_MS
    LP_SP.speedLimit.resolver.speedLimitFinalLast = limit_kph * CV.KPH_TO_MS
    LP_SP.speedLimit.resolver.speedLimitLastValid = limit_kph > 0
    CC_SP = custom.CarControlSP()
    CC_SP.intelligentCruiseButtonManagement.state = icbm_state
    self.v_cruise_helper.update_speed_limit_assist(self.is_metric, LP_SP)
    self.v_cruise_helper.update_plan_regime(LP_SP, CC_SP)
    # the arbiter owns the session now: an "SLA active" regime is arbiter state, not
    # a longitudinalPlanSP echo
    arb = self.v_cruise_helper.cruise_arbiter
    arb.enabled = True
    arb.state = getattr(custom.LongitudinalPlanSP.SpeedLimit.AssistState, sla_state)
    arb._speed_limit_prev = arb._speed_limit  # regime setup is not a limit-change edge

  def run_frames(self, CS, n=1, enabled=True):
    for _ in range(n):
      self.v_cruise_helper.update_v_cruise(CS, enabled=enabled, is_metric=self.is_metric)
      self.v_cruise_helper.reconcile_setpoint_with_dash(CS)  # card runs it right after (CardExt)
      CS.buttonEvents = []

  def engage_at(self, dash_kph):
    # settle the enabled state machine with the dash at a fixed value
    self.run_frames(make_car_state(dash_kph=dash_kph), n=5, enabled=False)
    self.run_frames(make_car_state(dash_kph=dash_kph), n=5, enabled=True)
    assert abs(self.v_cruise_helper.v_cruise_kph - dash_kph) < 0.1

  def press(self, button_type, dash_kph):
    CS = make_car_state(dash_kph=dash_kph, button_events=[ButtonEvent(type=button_type, pressed=True)])
    self.run_frames(CS, n=2)
    CS = make_car_state(dash_kph=dash_kph, button_events=[ButtonEvent(type=button_type, pressed=False)])
    self.run_frames(CS, n=1)

  def test_adopts_trailing_ecu_increment(self):
    """The ECU applies its final long-press step right after release; v_cruise must adopt it."""
    self.engage_at(35 * MPH)

    self.press(ButtonType.accelCruise, dash_kph=35 * MPH)
    # ECU's trailing +5 mph step lands shortly after release
    self.run_frames(make_car_state(dash_kph=40 * MPH), n=20)
    assert abs(self.v_cruise_helper.v_cruise_kph - 40 * MPH) < 0.5

  def test_sync_window_expires(self):
    """1 s after the last press the dash is no longer authoritative."""
    self.engage_at(35 * MPH)

    self.press(ButtonType.accelCruise, dash_kph=35 * MPH)
    self.run_frames(make_car_state(dash_kph=40 * MPH), n=20)
    # window closes 1 s after release; later dash moves (e.g. ICBM pushing it for SCC) don't leak in
    self.run_frames(make_car_state(dash_kph=40 * MPH), n=100)
    self.run_frames(make_car_state(dash_kph=30 * MPH), n=20)
    assert abs(self.v_cruise_helper.v_cruise_kph - 40 * MPH) < 0.5

  def test_no_adoption_while_scc_limited(self):
    """When a limiter drives the plan, the dash is held away from v_cruise by design;
    a press must increment v_cruise, never adopt the limiter-held dash."""
    self.engage_at(45 * MPH)
    self.set_regime(source='sccVision')
    # smart cruise pushed the real dash down to 35 mph while v_cruise stays at 45 mph
    self.run_frames(make_car_state(dash_kph=35 * MPH), n=110)
    assert abs(self.v_cruise_helper.v_cruise_kph - 45 * MPH) < 0.1

    self.press(ButtonType.accelCruise, dash_kph=35 * MPH)
    self.run_frames(make_car_state(dash_kph=36 * MPH), n=20)
    # v_cruise took its own +1 mph increment, not the dash value
    assert abs(self.v_cruise_helper.v_cruise_kph - (45 * MPH + IMPERIAL_INCREMENT)) < 0.5

  def test_no_adoption_while_icbm_mid_move(self):
    """After an SLA abort the servo restores the dash; the press's settle window must not
    adopt the still-low dash while ICBM is stepping it back up."""
    self.engage_at(60 * MPH)
    self.set_regime(source='cruise', icbm_state='increasing')

    self.press(ButtonType.accelCruise, dash_kph=45 * MPH)
    self.run_frames(make_car_state(dash_kph=47 * MPH), n=20)
    assert abs(self.v_cruise_helper.v_cruise_kph - (60 * MPH + IMPERIAL_INCREMENT)) < 0.5

  def test_sla_owns_buttons_no_increment(self):
    """While SLA is active, +/- presses carry SLA semantics; v_cruise must not increment."""
    self.engage_at(60 * MPH)
    self.set_regime(source='speedLimitAssist', sla_state='active', limit_kph=45 * MPH)

    self.press(ButtonType.accelCruise, dash_kph=45 * MPH)
    assert abs(self.v_cruise_helper.v_cruise_kph - 60 * MPH) < 0.1

  def test_sla_settled_press_reanchors_to_dash(self):
    """Settled at a limit, a + press deactivates SLA (plannerd side) and the setpoint
    re-anchors to the ECU's dash response: stock button feel."""
    self.engage_at(60 * MPH)
    self.set_regime(source='speedLimitAssist', sla_state='active', limit_kph=45 * MPH)
    self.run_frames(make_car_state(dash_kph=45 * MPH), n=110)
    assert abs(self.v_cruise_helper.v_cruise_kph - 60 * MPH) < 0.1

    # press lands while SLA is still active: no increment (owned by SLA)
    self.press(ButtonType.accelCruise, dash_kph=45 * MPH)
    # SLA deactivates on the press, plan source returns to cruise, servo is idle;
    # the ECU stepped the dash to 46; adopt it inside the settle window
    self.set_regime(source='cruise', icbm_state='holding', sla_state='inactive')
    self.run_frames(make_car_state(dash_kph=46 * MPH), n=20)
    assert abs(self.v_cruise_helper.v_cruise_kph - 46 * MPH) < 0.5

  def test_sla_abort_press_keeps_baseline(self):
    """Mid-decrease for SLA, a + press aborts: the baseline must survive untouched while
    the servo walks the dash back up."""
    self.engage_at(60 * MPH)
    self.set_regime(source='speedLimitAssist', sla_state='active', limit_kph=45 * MPH)

    # servo is halfway down (dash 52) when the driver presses +
    self.press(ButtonType.accelCruise, dash_kph=52 * MPH)
    # SLA deactivates; servo restores (increasing); dash still low during the settle window
    self.set_regime(source='cruise', icbm_state='increasing', sla_state='inactive')
    self.run_frames(make_car_state(dash_kph=53 * MPH), n=20)
    assert abs(self.v_cruise_helper.v_cruise_kph - 60 * MPH) < 0.1

  @pytest.mark.parametrize("is_metric, unit", [(False, MPH), (True, 1.)], ids=["mph", "kph"])
  def test_sla_minus_abort_floors_to_dash(self, is_metric, unit):
    """Mid-decrease for SLA, a - press asks for slower. The in-transit dash fails both
    agreement checks, so nothing is adopted inside the window; once it settles the
    setpoint must come down to the dash the ECU left, never stay at the baseline for
    the servo's restore to climb back to (the 51 -> 60 walk in a 45 zone)."""
    self.is_metric = is_metric
    self.engage_at(60 * unit)
    self.set_regime(source='speedLimitAssist', sla_state='active', limit_kph=45 * unit)

    # servo is halfway down (dash 52) when the driver presses -; the ECU's own -1 lands
    self.press(ButtonType.decelCruise, dash_kph=52 * unit)
    self.set_regime(source='cruise', icbm_state='holding', sla_state='inactive')
    settle_frames = int(RECONCILE_SETTLE_TIME / DT_CTRL)
    self.run_frames(make_car_state(dash_kph=51 * unit), n=settle_frames - 20)
    assert abs(self.v_cruise_helper.v_cruise_kph - 60 * unit) < 0.1, "in-transit dash adopted early"
    self.run_frames(make_car_state(dash_kph=51 * unit), n=40)
    assert abs(self.v_cruise_helper.v_cruise_kph - 51 * unit) < 0.5, "setpoint left above the dash after a - press"
    assert self.v_cruise_helper.v_cruise_cluster_kph == self.v_cruise_helper.v_cruise_kph
    # one-shot: a later dash move outside any window is not followed
    self.run_frames(make_car_state(dash_kph=40 * unit), n=20)
    assert abs(self.v_cruise_helper.v_cruise_kph - 51 * unit) < 0.5

  def test_sla_minus_abort_floor_never_raises(self):
    """The floor is min(): a dash that ends up above the setpoint (a trailing ECU step
    the other way, a dash held by something else) leaves the setpoint alone."""
    self.engage_at(60 * MPH)
    self.set_regime(source='speedLimitAssist', sla_state='active', limit_kph=45 * MPH)
    self.press(ButtonType.decelCruise, dash_kph=52 * MPH)
    self.set_regime(source='cruise', icbm_state='holding', sla_state='inactive')
    self.run_frames(make_car_state(dash_kph=65 * MPH), n=int(RECONCILE_SETTLE_TIME / DT_CTRL) + 20)
    assert abs(self.v_cruise_helper.v_cruise_kph - 60 * MPH) < 0.1

  def test_second_press_in_window_gets_its_own_verdict(self):
    """The agreement verdict is per press. A refused first press (in-transit dash) must
    not taint a second press inside the same window whose dash does agree: that one
    adopts the ECU's result like any settled press."""
    self.engage_at(60 * MPH)
    self.set_regime(source='speedLimitAssist', sla_state='active', limit_kph=45 * MPH)

    self.press(ButtonType.accelCruise, dash_kph=52 * MPH)  # dismiss mid-walk: refused
    self.set_regime(source='cruise', icbm_state='holding', sla_state='inactive')
    self.run_frames(make_car_state(dash_kph=59 * MPH), n=50)
    assert abs(self.v_cruise_helper.v_cruise_kph - 60 * MPH) < 0.1

    # 0.5 s later, dash back within the band of the setpoint: a plain + press
    self.press(ButtonType.accelCruise, dash_kph=59 * MPH)
    self.run_frames(make_car_state(dash_kph=60 * MPH), n=20)
    assert abs(self.v_cruise_helper.v_cruise_kph - 60 * MPH) < 0.5, "second press kept the first press's refusal"

  def test_vego_clip_disabled_for_icbm(self):
    """SET- while on the gas decrements on the stock ECU; v_cruise must not jump up to vEgo."""
    self.engage_at(35 * MPH)

    CS = make_car_state(dash_kph=35 * MPH, gas_pressed=True, v_ego=30.,
                        button_events=[ButtonEvent(type=ButtonType.decelCruise, pressed=True)])
    self.run_frames(CS, n=2)
    CS = make_car_state(dash_kph=34 * MPH, gas_pressed=True, v_ego=30.,
                        button_events=[ButtonEvent(type=ButtonType.decelCruise, pressed=False)])
    self.run_frames(CS, n=1)
    self.run_frames(make_car_state(dash_kph=34 * MPH, gas_pressed=True, v_ego=30.), n=10)

    # 30 m/s = 108 kph; without the guard v_cruise would have clipped up to vEgo
    assert self.v_cruise_helper.v_cruise_kph < 60.

