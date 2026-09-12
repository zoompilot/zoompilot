#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Regression replay: the ignored-confirm-prompt set speed drop (user report 2026-08-29).

Two segments of route 4eaa96ecd3da95e2 (CX-5 2022, stock ACC) dropped the dash 2-4 mph
about five seconds after engaging near a known limit, then walked it back:

  0000006e--acdc83b60f--3   engage t=184.2, prompt t=184.7, timeout t=189.8, 31 -> 27 -> 31
  0000006f--821e28d2fa--12  engage t=759.9, prompt t=760.5, timeout t=765.5, 40 -> 38 -> 40

The prompt froze the plan at the cluster. That hold round-tripped through whole mph and
landed ~7 mm/s under v_cruise, so SLA won the plan min() by rounding error alone and
relabelled longitudinalPlanSource as a limiter. The relabel armed ICBM's decel overshoot
against an ordinary cruise convergence (the car was ~1 mph over its own setpoint), the
servo's prompt freeze banked the resulting gap for the whole 5 s window, and the timeout
released it as a SET- burst.

This replay streams the recorded carState/longitudinalPlanSP through the CURRENT arbiter
AND servo, recomputing the plan source from the arbiter's live cap the way plannerd's
min() does, and asserts the servo emits nothing across the prompt and its timeout.

Replaying open-loop stays valid only until the recorded dash first moves: after that the
log reflects the buggy build's own presses and our build's world has diverged. So the
check runs from the prompt opening to that first recorded dash change -- if nothing is
emitted in that window, the recorded drop could never have happened. (In the logs the
first dash move IS the bug's own SET-, so the window covers the entire failure.) The
recorded aTarget is reused as-is: the fix moves the plan target by 7 mm/s, and with the
source back on `cruise` the overshoot cannot arm at any aTarget.

Run from repo root (venv active):
  python tools/mazda_long/icbm_sla/replay_prompt_overshoot.py <rlog> [<rlog> ...]
"""
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).parent))

from openpilot.cereal import custom
from opendbc.car.structs import car
from openpilot.common.constants import CV
from openpilot.tools.lib.logreader import LogReader
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import (
  IntelligentCruiseButtonManagement)
from replay_common import BUTTON_MAP, make_arbiter

SessionState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState
PlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
BUTTON_NAME = {int(getattr(SendButtonState, n)): n
               for n in ('none', 'increase', 'decrease', 'increaseHold', 'decreaseHold')}
SOURCE_NAME = {int(getattr(PlanSource, n)): n
               for n in ('cruise', 'sccVision', 'sccMap', 'speedLimitAssist')}
V_CRUISE_UNSET = 255.


def replay(path):
  arb = make_arbiter()
  CP = car.CarParams(pcmCruise=True, brand="mazda")
  servo = IntelligentCruiseButtonManagement(CP, custom.CarParamsSP(pcmCruiseSpeed=False))
  # the overshoot toggle is a param the servo re-reads; force it on without touching this
  # machine's params
  servo.params = SimpleNamespace(get_bool=lambda key: key == "SmartCruiseDecelOvershoot")
  servo.decel_overshoot_enabled = True

  enabled = False
  resolver = None
  scc_vision = scc_map = V_CRUISE_UNSET
  a_target = 0.
  session_state_stale = SessionState.disabled
  t0 = None

  prompts = []          # (t_open, t_close)
  sends = []            # (t, button, dash_mph, source) inside the valid window
  max_overshoot = 0.
  open_t = None
  dash_at_open = None   # replay is valid until the recorded dash leaves this
  window_end = None

  for msg in LogReader(str(path)):
    which = msg.which()
    t = msg.logMonoTime * 1e-9
    if t0 is None:
      t0 = t

    if which == 'carControl':
      enabled = msg.carControl.enabled
    elif which == 'longitudinalPlanSP':
      lp = msg.longitudinalPlanSP
      r = lp.speedLimit.resolver
      resolver = (r.speedLimit, r.speedLimitFinalLast, r.speedLimitValid or r.speedLimitLastValid)
      scc_vision = lp.smartCruiseControl.vision.vTarget
      scc_map = lp.smartCruiseControl.map.vTarget
      a_target = lp.aTarget
    elif which == 'carState' and resolver is not None:
      cs = msg.carState
      CS = car.CarState()
      CS.buttonEvents = [car.CarState.ButtonEvent(type=BUTTON_MAP[str(b.type)], pressed=b.pressed)
                         for b in cs.buttonEvents if str(b.type) in BUTTON_MAP]
      CS.vEgo = cs.vEgo
      CS.cruiseState.speedCluster = cs.cruiseState.speedCluster

      # -- card: arbiter, exactly as VCruiseHelper drives it
      lp_in = custom.LongitudinalPlanSP()
      lp_in.speedLimit.resolver.speedLimit = resolver[0]
      lp_in.speedLimit.resolver.speedLimitFinalLast = resolver[1]
      lp_in.speedLimit.resolver.speedLimitLastValid = resolver[2]
      arb.update_limit(lp_in)
      prev_state = arb.state
      arb.step(CS, enabled, cs.vCruise, cs.vCruiseCluster)

      if prev_state != SessionState.preActive and arb.state == SessionState.preActive:
        open_t = t - t0
        if dash_at_open is None:
          dash_at_open = round(cs.cruiseState.speedCluster * CV.MS_TO_MPH)
      if (dash_at_open is not None and window_end is None
          and round(cs.cruiseState.speedCluster * CV.MS_TO_MPH) != dash_at_open):
        window_end = t - t0
      if prev_state == SessionState.preActive and arb.state != SessionState.preActive:
        prompts.append((open_t, t - t0))

      # -- plannerd: the min() over the driver setpoint and every limiter, as
      #    LongitudinalPlannerSP.update_targets does, using the arbiter's LIVE cap
      cap = arb.v_cap if arb.v_cap > 0. else V_CRUISE_UNSET
      targets = {PlanSource.cruise: min(cs.vCruise, 145.) * CV.KPH_TO_MS,
                 PlanSource.sccVision: scc_vision,
                 PlanSource.sccMap: scc_map,
                 PlanSource.speedLimitAssist: cap}
      source = min(targets, key=lambda k: targets[k])

      lp_servo = custom.LongitudinalPlanSP()
      lp_servo.longitudinalPlanSource = source
      lp_servo.vTarget = float(targets[source])
      lp_servo.aTarget = float(a_target)

      # -- selfdrived: the servo, seeing the session one message hop late through the plan
      lp_servo.speedLimit.assist.state = session_state_stale
      CC = car.CarControl(enabled=enabled)
      servo.run(CS, CC, lp_servo, is_metric=False)
      session_state_stale = arb.state
      max_overshoot = max(max_overshoot, servo.overshoot_mph)

      # -- card's same-frame emission veto, as controls_update does before CI.apply
      button = SendButtonState.none if arb.prompting else servo.cruise_button
      if button != SendButtonState.none and dash_at_open is not None and window_end is None:
        sends.append((t - t0, BUTTON_NAME[int(button)],
                      cs.cruiseState.speedCluster * CV.MS_TO_MPH, SOURCE_NAME[int(source)]))

  return prompts, sends, max_overshoot, window_end


def main(paths):
  failures = 0
  for path in paths:
    prompts, sends, max_overshoot, window_end = replay(path)
    name = Path(path).name
    print(f"\n{name}")
    for open_t, close_t in prompts:
      print(f"  prompt  t={open_t:7.2f} -> {close_t:7.2f}s  ({close_t - open_t:.2f}s)")
    print(f"  peak decel-overshoot gap: {max_overshoot:.2f} mph")
    end = f"t={window_end:.2f}s (recorded dash moved)" if window_end else "end of segment"
    print(f"  replay valid until: {end}")
    if sends:
      failures += 1
      print(f"  FAIL: {len(sends)} button frame(s) emitted")
      for t, button, dash, source in sends[:12]:
        print(f"    t={t:7.2f}  {button:<13} dash={dash:4.1f} mph  source={source}")
    else:
      print("  PASS: no button frames emitted")

  print()
  if failures:
    print(f"FAILED on {failures}/{len(paths)} segment(s)")
  else:
    print(f"OK: {len(paths)} segment(s), the prompt never moves the dash")
  return 1 if failures else 0


if __name__ == "__main__":
  if len(sys.argv) < 2:
    print(__doc__)
    sys.exit(2)
  sys.exit(main(sys.argv[1:]))

