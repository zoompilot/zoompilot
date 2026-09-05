"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Closed-loop harness for the ICBM + SLA + driver-setpoint stack.

Unit tests validate each layer alone; the bugs this stack has actually shipped were all
INTERACTIONS: the confirm press tearing down its own session through the cluster guard,
the deadband stranding the restore the servo itself created, the dash re-sync adopting a
limiter-held dash. So this harness wires the real production classes together
(VCruiseHelper in card, SpeedLimitAssist in plannerd at 20 Hz, the ICBM servo in
selfdrived) against a simulated Mazda body ECU with the measured imperfections:

- taps register at most every 200 ms, and ~7% are dropped (seeded, deterministic)
- a sustained hold snaps to the next 5 mph multiple after ~0.6 s, then every ~0.55 s,
  with a trailing extra step if released mid-cycle
- a registered press takes ~60 ms to change the dash

It also models the two DIFFERENT cluster views the real system has: SLA and the planner
see openpilot's own vCruiseCluster (= v_cruise on ICBM cars), while the servo and the
reconciler see the car's real dash from CAN. Conflating those two is exactly the class of
bug the tests built on this harness exist to catch.
"""
import random

from openpilot.cereal import custom
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car.cruise import VCruiseHelper
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import IntelligentCruiseButtonManagement
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.assist_mirror import SpeedLimitAssistMirror
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.common import Mode
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP

ButtonEvent = car.CarState.ButtonEvent
ButtonType = car.CarState.ButtonEvent.Type
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState
SlaState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
PlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
EventNameSP = custom.OnroadEventSP.EventName

MPH_MS = CV.MPH_TO_MS
MPH_KPH = CV.MPH_TO_KPH

TAP_REGISTER_S = 0.2
TAP_DROP_RATE = 0.07
TAP_LATENCY_S = 0.06
HOLD_FIRST_STEP_S = 0.6
HOLD_STEP_PERIOD_S = 0.55
HOLD_TRAILING_RATE = 0.15

# press offsets spanning more than one full 20 Hz SLA cycle, for the timing sweeps
PRESS_OFFSETS_S = (0.0, 0.03, 0.07, 0.11, 0.16, 0.21)


class FakeMazdaEcu:
  """The body ECU's cruise set-speed integrator, driven at 100 Hz.

  A genuine physical hold (the wheel) grid-snaps 5 mph. Synthesized streams never do:
  measured across all 116 recorded routes, every servo-driven dash step was 1 mph (the
  wheel's own button-up frames interleave with the forged ones). forged_mode:
    'taps':    the measured reality: forged frames register as paced discrete presses
    'ignored': rejects them outright: zero movement (must trip the tap fallback)
  """

  def __init__(self, dash_mph, seed=0, forged_mode='taps'):
    self.dash = dash_mph
    self.rng = random.Random(seed)
    self.forged_mode = forged_mode
    self.t = 0.
    self.last_tap_t = -1.
    self.hold_dir = 0
    self.hold_t = 0.
    self.hold_steps = 0
    self.pending = []  # (apply_t, delta or 'snap'±1)

  def _register_tap(self, direction):
    if self.t - self.last_tap_t < TAP_REGISTER_S:
      return
    self.last_tap_t = self.t
    if self.rng.random() < TAP_DROP_RATE:
      return
    self.pending.append((self.t + TAP_LATENCY_S, direction))

  def _snap(self, direction):
    # step to the next multiple of 5 strictly in `direction`
    grid = 5 * ((self.dash // 5) + (1 if direction > 0 else 0)) if self.dash % 5 else self.dash + 5 * direction
    return grid - self.dash if self.dash % 5 else 5 * direction

  def tick(self, tap_dir=0, hold_dir=0, forged_hold_dir=0):
    """tap_dir: paced presses (driver taps or servo taps). hold_dir: the driver's
    PHYSICAL hold (grid-snaps). forged_hold_dir: the servo's synthesized stream
    (registers per forged_mode). -1/0/+1 for this 10 ms tick; returns dash (mph)."""
    self.t += DT_CTRL

    if tap_dir != 0:
      self._register_tap(tap_dir)

    if forged_hold_dir != 0 and self.forged_mode == 'taps':
      self._register_tap(forged_hold_dir)

    if hold_dir != 0:
      if self.hold_dir != hold_dir:
        self.hold_dir, self.hold_t, self.hold_steps = hold_dir, 0., 0
      self.hold_t += DT_CTRL
      due = HOLD_FIRST_STEP_S + self.hold_steps * HOLD_STEP_PERIOD_S
      if self.hold_t >= due:
        self.pending.append((self.t + TAP_LATENCY_S, 'snap' if self.hold_steps == 0 else 5 * hold_dir))
        self.hold_steps += 1
    else:
      if self.hold_dir != 0 and self.hold_steps > 0 and self.rng.random() < HOLD_TRAILING_RATE:
        self.pending.append((self.t + TAP_LATENCY_S, 5 * self.hold_dir))
      self.hold_dir, self.hold_t, self.hold_steps = 0, 0., 0

    due = [d for at, d in self.pending if self.t >= at]
    self.pending = [(at, d) for at, d in self.pending if self.t < at]
    for delta in due:
      if delta == 'snap':
        self.dash += self._snap(1 if self.hold_dir >= 0 else -1) if self.hold_dir else 0
      else:
        self.dash += delta
      self.dash = max(20, min(90, self.dash))
    return self.dash


class Loop:
  """100 Hz co-simulation of card (arbiter inside) + plannerd mirror (20 Hz) +
  selfdrived + the fake ECU.

  The SLA session machine now runs inside card's VCruiseHelper (the cruise arbiter),
  synchronous with the buttons: that hop has genuinely zero latency in production, so
  the harness models it that way. The plannerd->card (longitudinalPlanSP) and
  selfdrived->card (carControlSP) hops keep their one-cycle transport delay, and the
  mirror consumes the session snapshot from the previous frame, as plannerd would."""

  def __init__(self, baseline_mph=60, seed=0, forged_mode='taps', is_metric=False):
    # display units throughout: mph, or kph when is_metric (the *_mph names stay; the
    # ECU integrator, the limits and the setpoint all step in whichever unit the dash shows)
    self.is_metric = is_metric
    self.u_ms = CV.KPH_TO_MS if is_metric else MPH_MS
    self.u_kph = 1. if is_metric else MPH_KPH
    params = Params()
    params.put("IsReleaseSpBranch", True, block=True)
    params.put("SpeedLimitMode", int(Mode.assist), block=True)
    params.put_bool("IsMetric", is_metric, block=True)
    params.put_bool("CustomAccIncrementsEnabled", False)

    CP = car.CarParams(pcmCruise=True, brand="mazda")
    CP_SP = custom.CarParamsSP(pcmCruiseSpeed=False)
    self.helper = VCruiseHelper(CP, CP_SP)
    self.sla = self.helper.cruise_arbiter  # 100 Hz session truth; .state as before
    self.mirror = SpeedLimitAssistMirror(CP, CP_SP)  # plannerd side: plan cap + events
    self.servo = IntelligentCruiseButtonManagement(CP, CP_SP)
    self.ecu = FakeMazdaEcu(baseline_mph, seed=seed, forged_mode=forged_mode)
    self.lookahead_mph = None
    self.events_sp = EventsSP()

    self.tick_n = 0
    self.limit_mph = 0.
    self.scc_dip_mph = 0.  # SCC-vision target when active, 0 = inactive
    # vEgo defaults to the set speed (enough for the button-timing scenarios). Override it,
    # plus a_target, to exercise decel overshoot: the servo only builds the gap when the
    # plan is asking for real decel AND the car is above the plan target.
    self.v_ego_mph = None
    self.a_target = 0.
    self.decel_overshoot = False
    self.driver_queue = {}  # tick -> (ButtonType, hold_ticks)
    self._driver_active = None  # (button, remaining_ticks)
    self.sla_events = []  # (tick, event int) emitted by SLA, across the whole run

    # engage: settle disabled then enabled, dash = baseline
    for _ in range(5):
      self._card_tick(enabled=False)
    for _ in range(5):
      self._card_tick(enabled=True)
    assert abs(self.helper.v_cruise_kph - baseline_mph * self.u_kph) < 0.1

  # message construction
  def _cs(self, button_events=None):
    CS = car.CarState(cruiseState={"available": True,
                                   "speed": self.ecu.dash * self.u_ms,
                                   "speedCluster": self.ecu.dash * self.u_ms})
    v_ego_mph = self.v_ego_mph if self.v_ego_mph is not None else self.helper.v_cruise_kph / self.u_kph
    CS.vEgo = float(v_ego_mph * self.u_ms)
    CS.buttonEvents = button_events or []
    return CS

  def _lp_sp(self):
    LP_SP = custom.LongitudinalPlanSP()
    # as longitudinal_planner.update_targets: the mirror's cap always participates
    # (V_CRUISE_UNSET when idle never wins the min; the frozen prompt hold does)
    targets = {PlanSource.cruise: self.helper.v_cruise_kph * CV.KPH_TO_MS,
               PlanSource.speedLimitAssist: self.mirror.output_v_target}
    if self.scc_dip_mph > 0:
      targets[PlanSource.sccVision] = self.scc_dip_mph * self.u_ms
    source = min(targets, key=lambda k: targets[k])
    LP_SP.longitudinalPlanSource = source
    LP_SP.vTarget = float(targets[source])
    # vision lookahead wire: 0 = no lookahead (default for these tests); a test can set
    # lookahead_mph to model the horizon seeing a dip beyond the current source flip
    if self.lookahead_mph is not None:
      LP_SP.smartCruiseControl.vision.vAheadMin = float(self.lookahead_mph * self.u_ms)
    LP_SP.aTarget = float(self.a_target)
    LP_SP.speedLimit.assist.state = self.mirror.state
    LP_SP.speedLimit.resolver.speedLimit = self.limit_mph * self.u_ms
    LP_SP.speedLimit.resolver.speedLimitFinalLast = self.limit_mph * self.u_ms
    LP_SP.speedLimit.resolver.speedLimitLastValid = self.limit_mph > 0
    return LP_SP

  def _cc_sp(self):
    CC_SP = custom.CarControlSP()
    CC_SP.intelligentCruiseButtonManagement.state = self.servo.state
    return CC_SP

  # per-layer ticks
  def _card_tick(self, CS=None, enabled=True, lp_msg=None, cc_msg=None):
    CS = CS if CS is not None else self._cs()
    lp_msg, cc_msg = lp_msg or self._lp_sp(), cc_msg or self._cc_sp()
    self.helper.update_speed_limit_assist(self.is_metric, lp_msg)
    self.helper.update_v_cruise(CS, enabled=enabled, is_metric=self.is_metric)
    # card's post-update hook (CardExt.update_v_cruise_post)
    self.helper.update_plan_regime(lp_msg, cc_msg)
    self.helper.reconcile_setpoint_with_dash(CS)

  def run(self, seconds, assert_each=None):
    Params().put_bool("SmartCruiseDecelOvershoot", self.decel_overshoot)
    self.servo.decel_overshoot_enabled = self.decel_overshoot
    for _ in range(int(seconds / DT_CTRL)):
      self.tick_n += 1

      # Messages consumed this tick reflect the OTHER processes' state as of the previous
      # tick: plannerd/selfdrived output is in flight for at least one cycle before card
      # and each other see it. Zero-latency views would let e.g. card's press-edge
      # ownership latch observe an SLA deactivation that, in reality, cannot have been
      # published yet.
      lp_msg = self._lp_sp()
      cc_msg = self._cc_sp()

      # driver script
      events = []
      if self.tick_n in self.driver_queue:
        button, hold_ticks = self.driver_queue.pop(self.tick_n)
        self._driver_active = [button, hold_ticks, hold_ticks]
        events.append(ButtonEvent(type=button, pressed=True))
      driver_tap_dir = 0
      driver_hold_dir = 0
      if self._driver_active is not None:
        button, remaining, total = self._driver_active
        self._driver_active[1] -= 1
        if total - self._driver_active[1] > 30:
          # a physical hold reaches the ECU's hold integrator (genuine frames carry it)
          driver_hold_dir = 1 if button == ButtonType.accelCruise else -1
        if self._driver_active[1] <= 0:
          events.append(ButtonEvent(type=button, pressed=False))
          self._driver_active = None
          if total <= 30:
            driver_tap_dir = 1 if button == ButtonType.accelCruise else -1  # ECU applies short presses on release

      # one CarState per tick, shared by all three consumers (none mutates it)
      CS = self._cs(events)

      # plannerd: mirrors the session as published at the END of the previous card
      # frame (one transport hop), at 20 Hz
      if self.tick_n % 5 == 0:
        session_msg = custom.CarStateSP.new_message()
        self.helper.cruise_arbiter.fill_msg(session_msg)
        self.events_sp.clear()
        v_ego_mph = self.v_ego_mph if self.v_ego_mph is not None else self.helper.v_cruise_kph / self.u_kph
        self.mirror.update(session_msg.zoompilot.cruiseSession, v_ego_mph * self.u_ms, 0., 0., self.events_sp)
        self.sla_events.extend((self.tick_n, e) for e in self.events_sp.events)

      # selfdrived: servo against the real dash; the session state it sees is the one
      # plannerd mirrored into the plan (lp_msg.speedLimit.assist.state, 20 Hz, one hop old)
      CC = car.CarControl(enabled=True)
      self.servo.run(CS, CC, lp_msg, is_metric=self.is_metric)

      # card: arbiter (classification + session) runs inside update_v_cruise,
      # synchronous with the buttons
      self._card_tick(CS, lp_msg=lp_msg, cc_msg=cc_msg)

      # ECU: driver's physical press + openpilot's emission. Card vetoes emission with
      # same-frame session state (the servo's own freeze is one hop stale), as
      # card.controls_update does before CI.apply.
      tap_dir, forged_hold_dir = driver_tap_dir, 0
      sb = self.servo.cruise_button
      if self.helper.cruise_arbiter.prompting:
        sb = SendButtonState.none
      if sb == SendButtonState.increase:
        tap_dir = tap_dir or 1
      elif sb == SendButtonState.decrease:
        tap_dir = tap_dir or -1
      elif sb == SendButtonState.increaseHold:
        forged_hold_dir = 1
      elif sb == SendButtonState.decreaseHold:
        forged_hold_dir = -1
      self.ecu.tick(tap_dir=tap_dir, hold_dir=driver_hold_dir, forged_hold_dir=forged_hold_dir)

      if assert_each is not None:
        assert_each(self)

  # driver actions
  def driver_press(self, button, in_seconds, hold_s=0.15):
    self.driver_queue[self.tick_n + int(in_seconds / DT_CTRL)] = (button, max(1, int(hold_s / DT_CTRL)))

  @property
  def v_cruise_mph(self):
    return round(self.helper.v_cruise_kph / self.u_kph, 1)
