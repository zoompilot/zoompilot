#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Regression replay: drive 0000000b--b039e84091 (2026-07-26) against the cruise arbiter.

That drive, recorded on the shipped build, contains every reported SLA jank in one route:

  t≈ 85.9  dialing onto the limit fired "Auto adjusting to speed limit"
  t≈155.1  resume with setpoint already at the limit fired the same alert
  t≈181.1  limit raised mid-session: ICBM restored the dash upward DURING the prompt
  t≈187.1  dial-to-target activation was dismissed one frame later by its own press
  t≈393.6  + during a down-prompt left a lingering prompt while the dash slammed to 50
  t≈415.6  + confirm on a rising limit went active but stayed inert (min() select)

This replays the recorded buttons, cluster, and resolver limits through the CURRENT
CruiseArbiter (card-side session machine, via replay_common) and asserts the fixed
behavior at each documented moment: activation transitions, announce-counter semantics
(the alert channel), declines, and the frozen prompt cap. The arbiter is frame-based,
so the replay is fully deterministic.

Full-loop behavior (setpoint adoption, ICBM walks) is covered closed-loop by
test_icbm_sla_loop.py; here the recorded cluster is replayed as-is, so cluster-coupled
outcomes follow the OLD build's trajectory by construction.

Run from repo root (venv active):
  python tools/mazda_long/icbm_sla/replay_drive_incidents.py [route_glob]
"""
import sys

from openpilot.common.constants import CV
from openpilot.cereal import custom
from replay_common import arbiter_frames, make_arbiter, sorted_segments

SessionState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
DEFAULT_GLOB = "tools/mazda_long/test_data/sla_drive_logs/0000000b--b039e84091--*/rlog.zst"


def main(route_glob):
  arb = make_arbiter()
  paths = sorted_segments(route_glob)
  assert paths, f"no rlogs match {route_glob}"

  cs_count = 0
  transitions = []  # (t, from, to)
  announces = []    # t of each announce-counter bump (the alert channel)
  holds = []        # (t, v_cap) sampled while prompting
  last_announce = 0

  print(f"replaying {len(paths)} segments of {paths[0].split('/')[-2].rsplit('--', 1)[0]}")
  for t, prev_state, _ in arbiter_frames(arb, paths):
    cs_count += 1
    if arb.state != prev_state:
      transitions.append((t, prev_state, arb.state))
      print(f"  t={t:7.2f}s  {str(prev_state):9s} -> {arb.state}  (intent={arb.last_intent})")
    if arb.announce_counter != last_announce:
      announces.append(t)
      last_announce = arb.announce_counter
    if arb.prompting:
      holds.append((t, arb.v_cap))

  def transitions_in(t_a, t_b, frm=None, to=None):
    return [x for x in transitions if t_a <= x[0] <= t_b
            and (frm is None or x[1] == frm) and (to is None or x[2] == to)]

  def announces_in(t_a, t_b):
    return [x for x in announces if t_a <= x <= t_b]

  failures = []

  # 1. silent activations: dialing onto the limit / resuming at it must not announce
  for name, (a, b) in {"dial-to-target t≈85.9": (85.0, 87.0),
                       "resume-at-limit t≈155.1": (154.0, 157.0),
                       "dial-to-target t≈187.1": (186.5, 188.5)}.items():
    if not transitions_in(a, b, to=SessionState.active):
      failures.append(f"{name}: no activation")
    if announces_in(a, b):
      failures.append(f"{name}: spurious announce")

  # 2. the t≈187.1 activation must survive its own press (shipped build: 1-frame blip)
  if transitions_in(187.0, 190.0, frm=SessionState.active, to=SessionState.inactive):
    failures.append("t≈187.1: activation dismissed by its own press again")

  # 3. confirm presses: preActive -> active WITH the announcement
  for name, (a, b) in {"down-confirm t≈174.8": (174.3, 175.5),
                       "down-confirm t≈238.5": (238.2, 239.0),
                       "down-confirm t≈343.4": (343.0, 344.2),
                       "down-confirm t≈358.5": (358.0, 359.2),
                       "up-confirm t≈415.6": (415.2, 416.4),
                       "up-confirm t≈461.7": (461.3, 462.4)}.items():
    if not transitions_in(a, b, frm=SessionState.preActive, to=SessionState.active):
      failures.append(f"{name}: confirm did not activate")
    if not announces_in(a, b):
      failures.append(f"{name}: confirm fired no announcement")

  # 4. + against a down-prompt declines the session instead of lingering
  if not transitions_in(393.5, 394.5, frm=SessionState.preActive, to=SessionState.inactive):
    failures.append("t≈393.6: wrong-direction press did not decline the prompt")

  # 5. the prompt freezes the plan cap: while prompting out of an active session, vCap
  #    holds the old session target (releasing it would let the servo restore the dash)
  for name, (a, b, tgt_mph) in {"limit 45->35 t≈171-175": (171.5, 174.5, 49.5),
                                "limit 35->45 t≈181-186": (181.0, 185.5, 38.5),
                                "limit 35->25 t≈357-358": (357.3, 358.3, 35.0)}.items():
    window = [v for tt, v in holds if a <= tt <= b]
    if not window:
      failures.append(f"hold {name}: no prompting samples")
    elif any(abs(v - tgt_mph * CV.MPH_TO_MS) > 0.7 for v in window):
      seen = sorted({round(v * CV.MS_TO_MPH, 1) for v in window})
      failures.append(f"hold {name}: cap not frozen at ~{tgt_mph} mph: {seen}")

  expected_windows = [(174.3, 175.5), (238.2, 239.0), (343.0, 344.2), (358.0, 359.2), (415.2, 416.4), (461.3, 462.4)]
  spurious = [x for x in announces if not any(a <= x <= b for a, b in expected_windows)]

  print(f"\n  {cs_count} carState frames, {len(transitions)} transitions, {len(announces)} announces ({len(spurious)} outside confirm windows)")
  for s in spurious:
    print(f"    unexpected announce at t={s:.2f}")
  if spurious:
    failures.append(f"{len(spurious)} announce(s) outside the expected confirm windows")

  if failures:
    print("\nFAIL:")
    for f in failures:
      print(f"  - {f}")
    sys.exit(1)
  print("  PASS: silent latches, announced confirms, decline, and frozen prompt caps all verified")


if __name__ == "__main__":
  main(sys.argv[1] if len(sys.argv) > 1 else DEFAULT_GLOB)
