#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Regression replay: the seg16 confirm bug (F1) against the cruise arbiter.

Replays the recorded carState/longitudinalPlanSP stream from the route where +/-
confirmation self-destructed (952c07dea500f4e2/0000004f--fea08aad07/16, 65 mph zone,
dash set 50, SLA target 70) through the CURRENT card-side CruiseArbiter and checks:

  1. the driver's first matching press confirms (preActive -> active), and
  2. the session STAYS active until the next genuine driver press; on the shipped build
     it fell to inactive within one cycle because the confirm press's own cluster change
     tripped the manual-override guard. The arbiter classifies presses at their edges,
     so only a dismiss-classified press may end a session.

Run from repo root (venv active):
  python tools/mazda_long/icbm_sla/replay_sla_guard.py [path-to-rlog]
"""
import sys
from pathlib import Path

from openpilot.cereal import custom
from replay_common import arbiter_frames, make_arbiter

SessionState = custom.LongitudinalPlanSP.SpeedLimit.AssistState
CruiseIntent = custom.CarStateZP.CruiseSession.CruiseIntent
DEFAULT_LOG = Path.home() / "Desktop" / "952c07dea500f4e2_0000004f--fea08aad07--16--rlog.zst"


def main(log_path):
  arb = make_arbiter()

  cs_count = 0
  press_count = 0
  confirm_time = None
  bad_end = None

  print(f"replaying {log_path}")
  for t, prev_state, cs in arbiter_frames(arb, [str(log_path)]):
    cs_count += 1
    press_count += sum(1 for b in cs.buttonEvents if b.pressed and str(b.type) in ('accelCruise', 'decelCruise'))

    if prev_state == SessionState.preActive and arb.state == SessionState.active and confirm_time is None:
      confirm_time = t
      print(f"  t={confirm_time:7.2f}s  CONFIRMED (preActive -> active, intent={arb.last_intent})")
    if prev_state == SessionState.active and arb.state == SessionState.inactive:
      driver = arb.last_intent == CruiseIntent.dismiss
      tag = "driver dismiss press" if driver else "NO driver press  <-- would be the F1 bug"
      print(f"  t={t:7.2f}s  active -> inactive ({tag})")
      if not driver and bad_end is None:
        bad_end = t

  print(f"\n  {cs_count} carState frames, {press_count} driver +/- presses")
  assert confirm_time is not None, "FAIL: confirmation never happened in replay"
  assert bad_end is None, \
    f"FAIL: session ended without a dismiss-classified press at t={bad_end:.2f}s (F1 regressed)"
  print("  PASS: confirm fired at the documented moment; only dismiss-classified presses ended sessions")


if __name__ == "__main__":
  main(Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_LOG)
