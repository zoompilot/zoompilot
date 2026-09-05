"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Cruise arbiter on op-long ports without pcmCruise: openpilot owns the setpoint, so a
dismiss re-anchors v_cruise to the cap instead of leaving it to a dash reconciler.
"""
import pytest

from openpilot.cereal import custom
from opendbc.car.structs import car as car_struct
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET, VCruiseHelper
from openpilot.sunnypilot.selfdrive.car.cruise_arbiter import DISABLED_GUARD_PERIOD as ARBITER_GUARD_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode

SpeedLimitAssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
ButtonType = car_struct.CarState.ButtonEvent.Type


class TestOpLongNonPcmDismiss:
  """Op-long ports without pcmCruise (Hyundai, GM, VW, Tesla, Subaru), driven through
  the full VCruiseHelper so the increment path runs. The buttons and the setpoint are
  openpilot's own: there is no ECU step for a reconciler to adopt after a dismiss, so
  the arbiter re-anchors v_cruise to the cap and the press counts as a plain step from
  there. Before that, a dismiss dropped the cap and left v_cruise at the old baseline:
  one tap in a 45 zone and the car accelerated to 70, whichever way the tap went."""

  LIMIT_MPH = 45          # 72.42 kph: displays 45 mph or 72 kph
  BASELINE_KPH = {False: 70 * CV.MPH_TO_KPH, True: 113.}

  def setup_method(self, method):
    self.params = Params()
    self.params.put("SpeedLimitMode", int(Mode.assist), block=True)
    self.params.put_bool("CustomAccIncrementsEnabled", False)

  def make_helper(self, is_metric):
    self.is_metric = is_metric
    self.params.put_bool("IsMetric", is_metric, block=True)
    CP = car_struct.CarParams(pcmCruise=False, openpilotLongitudinalControl=True, brand="hyundai")
    CP_SP = custom.CarParamsSP(pcmCruiseSpeed=True)
    h = VCruiseHelper(CP, CP_SP)
    assert h.cruise_arbiter.applicable and h.cruise_arbiter.op_owns_setpoint
    h.cruise_arbiter.read_params(self.params)
    lp = custom.LongitudinalPlanSP()
    lp.speedLimit.resolver.speedLimit = self.LIMIT_MPH * CV.MPH_TO_MS
    lp.speedLimit.resolver.speedLimitFinalLast = self.LIMIT_MPH * CV.MPH_TO_MS
    lp.speedLimit.resolver.speedLimitLastValid = True
    h.update_speed_limit_assist(is_metric, lp)
    return h

  def frames(self, h, n, events=None, enabled=True):
    for _ in range(n):
      CS = car_struct.CarState(cruiseState={"available": True})
      CS.buttonEvents = events or []
      h.update_v_cruise(CS, enabled=enabled, is_metric=self.is_metric)
      events = None

  def press(self, h, button, hold_frames=2):
    self.frames(h, 1, [car_struct.CarState.ButtonEvent(type=button, pressed=True)])
    self.frames(h, hold_frames - 1)
    self.frames(h, 1, [car_struct.CarState.ButtonEvent(type=button, pressed=False)])
    self.frames(h, 5)

  def active_session(self, is_metric):
    h = self.make_helper(is_metric)
    self.frames(h, 5, enabled=False)
    h.v_cruise_kph = h.v_cruise_cluster_kph = self.BASELINE_KPH[is_metric]
    self.frames(h, int(ARBITER_GUARD_PERIOD / DT_CTRL) + 10)  # engage guard, then the prompt
    assert h.cruise_arbiter.prompting
    self.press(h, ButtonType.decelCruise)  # confirm down: the baseline survives, the cap binds
    assert h.cruise_arbiter.session_active
    assert h.v_cruise_kph == pytest.approx(self.BASELINE_KPH[is_metric], abs=0.05)
    assert h.cruise_arbiter.v_cap == pytest.approx(self.LIMIT_MPH * CV.MPH_TO_MS, abs=0.01)
    return h

  @pytest.mark.parametrize("is_metric, button, expected_kph", [
    (False, ButtonType.accelCruise, 72.4 + 1.6),   # 45 -> 46 mph
    (False, ButtonType.decelCruise, 72.4 - 1.6),   # 45 -> 44 mph
    (True, ButtonType.accelCruise, 73.),           # 72 -> 73 kph
    (True, ButtonType.decelCruise, 71.),           # 72 -> 71 kph
  ])
  def test_dismiss_tap_steps_from_the_cap(self, is_metric, button, expected_kph):
    h = self.active_session(is_metric)
    self.press(h, button)
    assert h.cruise_arbiter.state == SpeedLimitAssistState.inactive
    assert h.cruise_arbiter.v_cap == V_CRUISE_UNSET
    assert h.v_cruise_kph == pytest.approx(expected_kph, abs=0.05), "one step from the cap, not the baseline"
    assert h.v_cruise_cluster_kph == h.v_cruise_kph

  @pytest.mark.parametrize("is_metric, button, expected_kph", [
    (False, ButtonType.accelCruise, 80.),  # ceil(72.4 / 8.0) * 8.0
    (False, ButtonType.decelCruise, 72.),  # floor(72.4 / 8.0) * 8.0
    (True, ButtonType.accelCruise, 75.),   # ceil(72 / 5) * 5
    (True, ButtonType.decelCruise, 70.),   # floor(72 / 5) * 5
  ])
  def test_dismiss_long_press_snaps_from_the_cap(self, is_metric, button, expected_kph):
    """The long-press tick at CRUISE_LONG_PRESS runs the partial-interval snap from the
    re-anchored value, so a hold climbs or descends the grid from the cap."""
    h = self.active_session(is_metric)
    self.press(h, button, hold_frames=60)
    assert h.cruise_arbiter.state == SpeedLimitAssistState.inactive
    assert h.v_cruise_kph == pytest.approx(expected_kph, abs=0.05)

  def test_dismiss_never_raises_the_baseline(self):
    """The anchor is what the plan was running: min(baseline, cap). A session whose
    target sits above the setpoint (a CST auto-apply after a limit rise) must not lift
    the setpoint to it on a dismiss."""
    h = self.active_session(False)
    h.v_cruise_kph = h.v_cruise_cluster_kph = 40 * CV.MPH_TO_KPH  # driver dialed under the cap
    self.frames(h, 5)
    self.press(h, ButtonType.decelCruise)
    assert h.v_cruise_kph == pytest.approx(40 * CV.MPH_TO_KPH - 1.6, abs=0.05)

  def test_icbm_dismiss_stays_owned(self):
    """The re-anchor is for setpoints openpilot holds. On an ICBM car the ECU steps the
    dash itself, so the dismiss press must stay owned (no increment, no anchor write)
    and leave the re-anchor to the reconciler."""
    self.is_metric = False
    self.params.put_bool("IsMetric", False, block=True)
    CP = car_struct.CarParams(pcmCruise=True, openpilotLongitudinalControl=False, brand="mazda")
    CP_SP = custom.CarParamsSP(pcmCruiseSpeed=False)
    h = VCruiseHelper(CP, CP_SP)
    assert h.cruise_arbiter.applicable and not h.cruise_arbiter.op_owns_setpoint
    h.cruise_arbiter.read_params(self.params)
    for enabled in (False, False, True, True, True):  # settle the engaged state
      h.update_v_cruise(car_struct.CarState(cruiseState={"available": True, "speed": 45 * CV.MPH_TO_MS}),
                        enabled=enabled, is_metric=False)
    arb = h.cruise_arbiter
    arb.state = SpeedLimitAssistState.active
    arb.v_cap = self.LIMIT_MPH * CV.MPH_TO_MS
    h.v_cruise_kph = h.v_cruise_cluster_kph = 60 * CV.MPH_TO_KPH

    CS = car_struct.CarState(cruiseState={"available": True, "speed": 45 * CV.MPH_TO_MS})
    CS.buttonEvents = [car_struct.CarState.ButtonEvent(type=ButtonType.accelCruise, pressed=True)]
    h.update_v_cruise(CS, enabled=True, is_metric=False)
    assert arb.state == SpeedLimitAssistState.inactive
    assert arb.press_owned(ButtonType.accelCruise)
    assert not arb.adopted_this_frame
    assert h.v_cruise_kph == pytest.approx(60 * CV.MPH_TO_KPH, abs=0.05)
