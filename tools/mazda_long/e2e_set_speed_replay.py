#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Experimental mode's set-speed floor (sunnypilot/selfdrive/controls/lib/e2e_set_speed) over rlogs.

replay: the real controller, enabled, open loop on every modelV2 frame of alpha-long driving,
        fed the planner's own logged inputs (one frame behind). What it would have added, how
        often, why it stayed off, and how much it would have overridden the model's slowdowns.
logged: what the controller did on the car, from longitudinalPlanSP.zoompilot.e2eSetSpeed.
        Watch for a_model < -0.05 held with full authority: the floor is winning a slowdown
        the model meant, and the cap or tau is too aggressive.

Both report how often the e2e candidate was the one the planner chose while boosting: a floor
under a lower candidate changes nothing. Open loop cannot show the speed it would have reached;
test_closed_loop covers that.

Usage: e2e_set_speed_replay.py [--logged] <rlog> [<rlog> ...]
"""
import argparse
import os
import sys
from collections import Counter
from functools import partial
from multiprocessing import Pool

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)

from opendbc.car.interfaces import ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_planner import get_coast_accel
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.controller import E2ESetSpeedController, Inhibit
from openpilot.tools.lib.logreader import LogReader

SERVICES = ('carState', 'modelV2', 'radarState', 'controlsState', 'selfdriveState', 'carControl', 'longitudinalPlan',
            'longitudinalPlanSP')
INHIBIT_NAMES = {v: k for k, v in Inhibit.schema.enumerants.items()}
IDLE = (Inhibit.disabled, Inhibit.inactive, Inhibit.decActive)


class AlwaysOn:
  def get_bool(self, key):
    return True


def frames(path):
  """(sm, modelV2 time) at every modelV2 once the other services have arrived."""
  sm = {}
  for m in LogReader(path, sort_by_time=True):
    w = m.which()
    if w not in SERVICES:
      continue
    sm[w] = getattr(m, w)
    if w == 'modelV2' and all(s in sm for s in SERVICES):
      yield sm, m.logMonoTime * 1e-9


def planner_inputs(sm):
  """update()'s arguments after a_model, as the planner logged them."""
  CS, plan, sp = sm['carState'], sm['longitudinalPlan'], sm['longitudinalPlanSP']
  ned = sm['carControl'].orientationNED
  return {
    'v_cruise': sp.vTarget,  # the target after SCC and SLA (0 under forceDecel)
    'is_e2e': sm['selfdriveState'].experimentalMode and (not sp.dec.active or sp.dec.state == 'blended'),
    'reset_state': sm['controlsState'].longControlState == LongCtrlState.off or CS.vCruise == V_CRUISE_UNSET,
    'dec_active': sp.dec.active,
    'allow_throttle': plan.allowThrottle,
    'fcw': plan.fcw,
    'accel_coast': get_coast_accel(ned[1]) if len(ned) == 3 else ACCEL_MAX,
  }


def row(sm, t, a_model, out, authority, inhibit):
  return (t, sm['carState'].vEgo, sm['longitudinalPlanSP'].vTarget, a_model, out, authority, inhibit,
          sm['longitudinalPlan'].longitudinalPlanSource == 'e2e')


def replay(path):
  ctl = E2ESetSpeedController(params=AlwaysOn(), dt=DT_MDL)
  rows = []
  for sm, t in frames(path):
    a_model = sm['modelV2'].action.desiredAcceleration
    out = ctl.update(sm, a_model, **planner_inputs(sm))
    rows.append(row(sm, t, a_model, out, ctl.authority, int(ctl.inhibit)))
  return rows


def logged(path):
  rows = []
  for sm, t in frames(path):
    r = sm['longitudinalPlanSP'].zoompilot.e2eSetSpeed
    a_model = sm['modelV2'].action.desiredAcceleration
    rows.append(row(sm, t, a_model, a_model + r.boost, r.authority, r.inhibit.raw))
  return rows


def report(name, rows):
  if not rows:
    print(f"{name}: no frames")
    return
  t, v, vc, am, out, auth, inh, binds = np.asarray(rows, dtype=float).T
  act = ~np.isin(inh, IDLE)
  if not act.any():
    print(f"{name}: never active")
    return
  boost = out - am
  on = act & (boost > 0.01)
  print(f"{name}: {act.sum() * DT_MDL / 60:.1f} min active, boosting {on.sum() / act.sum() * 100:.0f}% of it")
  if on.any():
    gap = np.median((vc - v)[on]) * CV.MS_TO_MPH
    print(f"  boost median {np.median(boost[on]):+.2f}, p90 {np.percentile(boost[on], 90):+.2f} m/s^2; " +
          f"gap to target while boosting median {gap:.1f} mph; e2e chosen in {binds[on].mean() * 100:.0f}% of it")
  reasons = Counter(INHIBIT_NAMES[int(i)] for i in inh[act])
  print("  state:", ", ".join(f"{k} {n * 100 / act.sum():.0f}%" for k, n in reasons.most_common(8)))

  # jerk the floor adds on consecutive active frames
  pair = act[1:] & act[:-1] & (np.diff(t) < 2 * DT_MDL)
  if pair.any():
    j_model = np.abs(np.diff(am)[pair]) / DT_MDL
    j_out = np.abs(np.diff(out)[pair]) / DT_MDL
    print(f"  jerk p99: model {np.percentile(j_model, 99):.2f}, with floor {np.percentile(j_out, 99):.2f} m/s^3")

  # the tuning signal: full authority while the model asks to slow
  pushed = act & (auth > 0.99) & (am < -0.05)
  if pushed.any():
    share = pushed.sum() / max(on.sum(), 1) * 100
    print(f"  full authority while model accel < -0.05: {pushed.sum() * DT_MDL:.1f} s ({share:.0f}% of boosting), " +
          f"model median {np.median(am[pushed]):+.2f}")


def run(fn, path):
  try:
    return fn(path)
  except Exception as e:
    print(f"{path}: {e!r}", file=sys.stderr)
    return []


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('--logged', action='store_true', help="report what the car's controller did instead of replaying it")
  ap.add_argument('rlogs', nargs='+')
  args = ap.parse_args()
  everything = []
  with Pool() as pool:
    for path, rows in zip(args.rlogs, pool.imap(partial(run, logged if args.logged else replay), args.rlogs), strict=True):
      everything += rows
      report(os.path.relpath(path), rows)
  if len(args.rlogs) > 1:
    print()
    report("all", everything)


if __name__ == "__main__":
  main()
