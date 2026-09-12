"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Card-side cruise arbiter on stock-ACC (ICBM) cars: press classification at the edges,
the confirm/dismiss/decline flow, the prompt freeze and its timers.
"""
import pytest

from openpilot.cereal import custom
from opendbc.car.car_helpers import interfaces
from opendbc.car.structs import car as car_struct
from opendbc.car.toyota.values import CAR as TOYOTA
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.cruise_arbiter import CruiseArbiter, \
  PRE_ACTIVE_GUARD_PERIOD as ARBITER_PROMPT_PERIOD, DISABLED_GUARD_PERIOD as ARBITER_GUARD_PERIOD
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode

SpeedLimitAssistState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
ButtonType = car_struct.CarState.ButtonEvent.Type

DEFAULT_CAR = TOYOTA.TOYOTA_RAV4_TSS2


class TestCruiseArbiterNonPcm:
  """Stock-ACC (button-actuated) cars: pcmCruise=True, openpilotLongitudinalControl=False,
  pcmCruiseSpeed=False. The session runs in the card-side cruise arbiter, synchronous
  with the buttons: presses are classified once at their edges (dismiss at press,
  confirm/decline at release), so the confirm press's own dash step and ICBM-injected
  presses can never tear a session down (the seg16 F1 bug class)."""

  MPH = CV.MPH_TO_MS

  def setup_method(self, method):
    self.params = Params()
    self.params.put("IsReleaseSpBranch", True, block=True)
    self.params.put("SpeedLimitMode", int(Mode.assist), block=True)
    self.params.put_bool("IsMetric", False, block=True)

    CarInterface = interfaces[DEFAULT_CAR]
    CP = CarInterface.get_non_essential_params(DEFAULT_CAR)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, DEFAULT_CAR)
    CP.openpilotLongitudinalControl = False
    CP.pcmCruise = True
    CP_SP.pcmCruiseSpeed = False
    self.arb = CruiseArbiter(CP, CP_SP)
    self.arb.read_params(self.params)
    assert self.arb.applicable
    self.v_cruise_kph = 60 * CV.MPH_TO_KPH

  def _lp(self, limit_mph, has_limit=True):
    lp = custom.LongitudinalPlanSP()
    lp.speedLimit.resolver.speedLimit = limit_mph * self.MPH
    lp.speedLimit.resolver.speedLimitFinalLast = limit_mph * self.MPH
    lp.speedLimit.resolver.speedLimitLastValid = has_limit
    return lp

  def frame(self, cluster_mph, limit_mph, has_limit=True, events=None):
    self.arb.update_limit(self._lp(limit_mph, has_limit))
    CS = car_struct.CarState()
    CS.buttonEvents = events or []
    self.v_cruise_kph = self.arb.step(CS, True, self.v_cruise_kph, cluster_mph * CV.MPH_TO_KPH)

  def press(self, button_type, cluster_mph, limit_mph, has_limit=True):
    self.frame(cluster_mph, limit_mph, has_limit,
               [car_struct.CarState.ButtonEvent(type=button_type, pressed=True)])
    self.frame(cluster_mph, limit_mph, has_limit,
               [car_struct.CarState.ButtonEvent(type=button_type, pressed=False)])

  def go_pre_active(self, cluster_mph, limit_mph):
    self.arb.state = SpeedLimitAssistState.preActive
    self.arb.pre_active_timer = int(ARBITER_PROMPT_PERIOD / DT_CTRL)
    self.frame(cluster_mph, limit_mph)
    assert self.arb.state == SpeedLimitAssistState.preActive

  def test_confirm_press_sticks(self):
    """F1 regression: one + press confirms; neither the press's own dash step nor ICBM
    walking the dash afterward may tear the session down."""
    self.go_pre_active(cluster_mph=40, limit_mph=45)

    self.press(ButtonType.accelCruise, 40, 45)
    assert self.arb.state == SpeedLimitAssistState.active

    # the ECU applies the confirm press's own +1 next frame
    self.frame(41, 45)
    assert self.arb.state == SpeedLimitAssistState.active
    # ICBM walks the dash to the target across the following seconds
    for cluster in (42, 43, 44, 45):
      self.frame(cluster, 45)
      assert self.arb.state == SpeedLimitAssistState.active

  def test_up_confirm_adopts_setpoint(self):
    """+ on a prompt above the setpoint raises v_cruise to the limit (never lowers it);
    the confirm press itself must not also increment."""
    self.v_cruise_kph = 40 * CV.MPH_TO_KPH
    self.go_pre_active(cluster_mph=40, limit_mph=45)

    self.press(ButtonType.accelCruise, 40, 45)
    assert self.arb.state == SpeedLimitAssistState.active
    assert round(self.v_cruise_kph * CV.KPH_TO_MPH) == 45

  def test_down_confirm_keeps_baseline(self):
    self.go_pre_active(cluster_mph=60, limit_mph=45)
    self.press(ButtonType.decelCruise, 60, 45)
    assert self.arb.state == SpeedLimitAssistState.active
    assert round(self.v_cruise_kph * CV.KPH_TO_MPH) == 60

  def test_wrong_direction_press_declines(self):
    """A release against the confirm direction is a decline: the session ends right away
    (instead of a prompt lingering over a driver who is dialing the other way), the press
    still increments, and the next limit change re-prompts normally."""
    self.go_pre_active(cluster_mph=40, limit_mph=45)

    self.press(ButtonType.decelCruise, 40, 45)  # limit is above: requires +
    assert self.arb.state == SpeedLimitAssistState.inactive
    assert not self.arb.press_owned(ButtonType.decelCruise)

    # declining is not a dismissal: a new limit re-prompts
    self.frame(40, 35)
    assert self.arb.state == SpeedLimitAssistState.preActive

  def settled_press_deactivates(self):
    self.go_pre_active(cluster_mph=50, limit_mph=45)
    self.press(ButtonType.decelCruise, 50, 45)
    assert self.arb.state == SpeedLimitAssistState.active
    self.frame(45, 45)  # ICBM finished the move

    self.frame(45, 45, events=[car_struct.CarState.ButtonEvent(type=ButtonType.accelCruise, pressed=True)])
    assert self.arb.state == SpeedLimitAssistState.inactive
    assert self.arb.press_owned(ButtonType.accelCruise)  # the ECU step re-anchors, no increment
    self.frame(45, 45, events=[car_struct.CarState.ButtonEvent(type=ButtonType.accelCruise, pressed=False)])
    assert self.arb.press_owned(ButtonType.accelCruise)

  def test_settled_press_deactivates(self):
    """Settled at the limit, a press hands the buttons back at the press edge."""
    self.settled_press_deactivates()

  def mid_move_press_aborts(self):
    self.go_pre_active(cluster_mph=60, limit_mph=45)
    self.press(ButtonType.decelCruise, 60, 45)
    assert self.arb.state == SpeedLimitAssistState.active

    self.press(ButtonType.accelCruise, 52, 45)  # mid-walk
    assert self.arb.state == SpeedLimitAssistState.inactive

  def test_mid_move_press_aborts(self):
    """A + press while ICBM is still walking the dash down aborts the session; the servo
    then restores the driver's setpoint because the plan min releases."""
    self.mid_move_press_aborts()

  def test_rearms_on_next_limit_change(self):
    self.mid_move_press_aborts()

    self.frame(60, 45)  # same limit: stays down
    assert self.arb.state == SpeedLimitAssistState.inactive
    self.frame(60, 35)  # new limit posted -> new session
    assert self.arb.state == SpeedLimitAssistState.preActive

  def test_dial_to_target_confirms(self):
    """Reaching the limit by hand is a confirmation (upstream semantics)."""
    self.go_pre_active(cluster_mph=50, limit_mph=45)
    self.frame(45, 45)
    assert self.arb.state == SpeedLimitAssistState.active

  def test_settled_dismissal_does_not_reactivate(self):
    """After a settled-press dismissal the cluster still equals the limit until the ECU's
    own +-1 step lands; the dial-to-target auto-confirm must not re-arm and fight the
    driver. Dismissal holds until the next limit change."""
    self.settled_press_deactivates()

    for cluster in (45, 45, 46, 46, 46):
      self.frame(cluster, 45)
      assert self.arb.state == SpeedLimitAssistState.inactive

    self.frame(46, 35)  # a genuinely new limit re-arms
    assert self.arb.state == SpeedLimitAssistState.preActive

  def test_limit_dropout_holds_session(self):
    """Map dropout mid-session: the state survives; the cap releases only through the
    resolver's own last-limit semantics."""
    self.go_pre_active(cluster_mph=50, limit_mph=45)
    self.press(ButtonType.decelCruise, 50, 45)
    assert self.arb.state == SpeedLimitAssistState.active

    self.frame(45, 45, has_limit=False)
    assert self.arb.state == SpeedLimitAssistState.active

  def test_prompt_freezes_cap_out_of_session(self):
    """A limit change mid-session prompts and freezes the plan cap at the old session
    target until answered; a decline releases it."""
    self.go_pre_active(cluster_mph=48, limit_mph=40)
    self.press(ButtonType.decelCruise, 48, 40)
    assert self.arb.state == SpeedLimitAssistState.active
    self.frame(40, 40)
    old_cap = self.arb.v_cap
    assert abs(old_cap - 40 * self.MPH) < 0.1

    self.frame(40, 45)  # limit rises: prompt
    assert self.arb.state == SpeedLimitAssistState.preActive
    for _ in range(50):
      self.frame(40, 45)
      assert self.arb.state == SpeedLimitAssistState.preActive
      assert self.arb.v_cap == old_cap, "prompt must freeze the plan cap"

    self.press(ButtonType.accelCruise, 40, 45)  # confirm up
    assert self.arb.state == SpeedLimitAssistState.active
    assert abs(self.arb.v_cap - 45 * self.MPH) < 0.1

  def test_prompt_times_out(self):
    self.go_pre_active(cluster_mph=50, limit_mph=45)
    for _ in range(int(ARBITER_PROMPT_PERIOD / DT_CTRL) + 2):
      self.frame(50, 45)
    assert self.arb.state == SpeedLimitAssistState.inactive

  def test_wall_clock_timers(self):
    """The arbiter runs at 100 Hz: the prompt window and engage guard must still be
    their wall-clock durations."""
    assert int(ARBITER_PROMPT_PERIOD / DT_CTRL) == 500   # 5 s at 100 Hz
    assert int(ARBITER_GUARD_PERIOD / DT_CTRL) == 50     # 0.5 s at 100 Hz

  @pytest.mark.parametrize("op_long, pcm_cruise, pcm_cruise_speed", [
    (False, True, False),  # ICBM (Mazda)
    (True, False, True),   # op-long, no pcmCruise (most ports)
    (True, True, True),    # pcm-op-long (plannerd machine)
  ], ids=["icbm", "op_long_non_pcm", "pcm_op_long"])
  def test_applicability_matches_planner_selection(self, op_long, pcm_cruise, pcm_cruise_speed):
    """The arbiter must cover exactly the cars plannerd mirrors (everything that is not
    pcm-op-long): stock-ACC button cars AND op-long ports without pcmCruise. A mismatch
    leaves a car mirroring a permanently disabled session."""
    CarInterface = interfaces[DEFAULT_CAR]
    CP = CarInterface.get_non_essential_params(DEFAULT_CAR)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, DEFAULT_CAR)

    CP.openpilotLongitudinalControl = op_long
    CP.pcmCruise = pcm_cruise
    CP_SP.pcmCruiseSpeed = pcm_cruise_speed
    arb = CruiseArbiter(CP, CP_SP)
    mirrored_by_plannerd = not (op_long and pcm_cruise)
    assert arb.applicable == mirrored_by_plannerd, (op_long, pcm_cruise, pcm_cruise_speed)

  def test_op_long_non_pcm_car_confirms(self):
    """Op-long without pcmCruise (e.g. most Hyundai/Honda ports): buttons are
    openpilot's own; the arbiter session and confirm flow must work there too."""
    CarInterface = interfaces[DEFAULT_CAR]
    CP = CarInterface.get_non_essential_params(DEFAULT_CAR)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, DEFAULT_CAR)
    CP.openpilotLongitudinalControl = True
    CP.pcmCruise = False
    CP_SP.pcmCruiseSpeed = True
    self.arb = CruiseArbiter(CP, CP_SP)
    self.arb.read_params(self.params)
    assert self.arb.applicable
    self.v_cruise_kph = 40 * CV.MPH_TO_KPH

    self.go_pre_active(cluster_mph=40, limit_mph=45)
    self.press(ButtonType.accelCruise, 40, 45)
    assert self.arb.state == SpeedLimitAssistState.active
    assert round(self.v_cruise_kph * CV.KPH_TO_MPH) == 45
