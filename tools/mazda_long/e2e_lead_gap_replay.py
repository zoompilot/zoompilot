#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Experimental mode's follow-distance assist (sunnypilot/selfdrive/controls/lib/e2e_lead_gap) over rlogs.

replay: the real controller, enabled, open loop on every modelV2 frame, fed the planner's logged
        inputs: the model's acceleration, the MPC candidate rebuilt from the logged plan, the
        nudge's inputs. What it would have added, how often, why it stayed off.
logged: what the controller did on the car, from longitudinalPlanSP.zoompilot.e2eLeadGap.
        Watch for a long stretch at full authority with the model braking: the car is being
        held closer than the model wants and the gain band is too wide.

Open loop cannot show the gap it would have reached; test_closed_loop covers that.

Usage: e2e_lead_gap_replay.py [--logged] (--from-index | <rlog> ...)
  --from-index: every experimental-mode segment with a lead in the lead_event_index cache
"""
import argparse
import os
import sys
from collections import Counter
from functools import partial
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))
sys.path.insert(0, HERE)

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_lead_gap.controller import E2ELeadGapController, Inhibit

from e2e_set_speed_replay import AlwaysOn, frames, planner_inputs, run

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ACTION_T = 0.36 + DT_MDL  # Mazda longitudinalActuatorDelay + DT_MDL
INHIBIT_NAMES = {v: k for k, v in Inhibit.schema.enumerants.items()}
IDLE = (Inhibit.disabled, Inhibit.inactive, Inhibit.decActive)
PERSONALITIES = ('aggressive', 'standard', 'relaxed')


def a_mpc(sm):
  plan = sm['longitudinalPlan']
  if len(plan.speeds) != CONTROL_N or len(plan.accels) != CONTROL_N:
    return float('nan')
  return float(get_accel_from_plan(np.asarray(plan.speeds), np.asarray(plan.accels), CONTROL_N_T_IDX, action_t=ACTION_T))


def row(sm, t, a_model, mpc, out, ctl_state):
  authority, inhibit, excess = ctl_state
  return (t, sm['carState'].vEgo, a_model, mpc, out, authority, inhibit, excess,
          sm['longitudinalPlan'].longitudinalPlanSource == 'e2e', sm['selfdriveState'].personality.raw)


def replay(path):
  ctl = E2ELeadGapController(params=AlwaysOn(), dt=DT_MDL)
  rows = []
  for sm, t in frames(path):
    a_model = sm['modelV2'].action.desiredAcceleration
    mpc = a_mpc(sm)
    inputs = planner_inputs(sm)
    out = ctl.update(sm, a_model, mpc, inputs['is_e2e'], inputs['reset_state'], inputs['dec_active'],
                     inputs['allow_throttle'], inputs['fcw'])
    rows.append(row(sm, t, a_model, mpc, out, (ctl.authority, int(ctl.inhibit), ctl.gap_excess)))
  return rows


def logged(path):
  rows = []
  for sm, t in frames(path):
    r = sm['longitudinalPlanSP'].zoompilot.e2eLeadGap
    a_model = sm['modelV2'].action.desiredAcceleration
    rows.append(row(sm, t, a_model, a_mpc(sm), a_model + r.boost, (r.authority, r.inhibit.raw, r.gapExcess)))
  return rows


def report(name, rows):
  if not rows:
    print(f"{name}: no frames")
    return
  t, v, am, mpc, out, auth, inh, excess, binds, pers = np.asarray(rows, dtype=float).T
  act = ~np.isin(inh, IDLE)
  if not act.any():
    print(f"{name}: never active")
    return
  boost = out - am
  on = act & (boost > 0.01)
  print(f"{name}: {act.sum() * DT_MDL / 60:.1f} min active, boosting {on.sum() / act.sum() * 100:.0f}% of it " +
        f"({on.sum() * DT_MDL / 60:.1f} min)")
  if on.any():
    print(f"  boost median {np.median(boost[on]):+.2f}, p90 {np.percentile(boost[on], 90):+.2f} m/s^2; gap excess while " +
          f"boosting median {np.median(excess[on]):.0f} m; e2e chosen {binds[on].mean() * 100:.0f}% of it; boost at the MPC " +
          f"(tied) {np.mean(np.abs(out - mpc)[on] < 0.01) * 100:.0f}%")
    for p, pname in enumerate(PERSONALITIES):
      m = on & (pers == p)
      if m.any():
        print(f"    {pname}: {m.sum() * DT_MDL / 60:.1f} min, boost median {np.median(boost[m]):+.2f}, " +
              f"excess median {np.median(excess[m]):.0f} m")
  reasons = Counter(INHIBIT_NAMES[int(i)] for i in inh[act])
  print("  state:", ", ".join(f"{k} {n * 100 / act.sum():.0f}%" for k, n in reasons.most_common(10)))

  pair = act[1:] & act[:-1] & (np.diff(t) < 2 * DT_MDL)
  if pair.any():
    j_model = np.abs(np.diff(am)[pair]) / DT_MDL
    j_out = np.abs(np.diff(out)[pair]) / DT_MDL
    print(f"  jerk p99: model {np.percentile(j_model, 99):.2f}, with lift {np.percentile(j_out, 99):.2f} m/s^3")

  pushed = act & (auth > 0.99) & (am < -0.2) & (boost > 0.01)
  if pushed.any():
    print(f"  lifting while model accel < -0.2: {pushed.sum() * DT_MDL:.1f} s, model median {np.median(am[pushed]):+.2f}, " +
          f"boost median {np.median(boost[pushed]):+.2f}")


def index_paths():
  import lead_event_index as L
  paths = set()
  for key in L.engaged_keys():
    d = L.load_key(key)
    if ((d['experimental'] > 0.5) & (d['leadPresent'] > 0.5) & (d['longActive'] > 0.5)).sum() > 100:
      paths.add(str(d['path']))
  return sorted(paths)


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('--logged', action='store_true', help="report what the car's controller did instead of replaying it")
  ap.add_argument('--from-index', action='store_true')
  ap.add_argument('--quiet', action='store_true', help='only the total')
  ap.add_argument('rlogs', nargs='*')
  args = ap.parse_args()
  paths = index_paths() if args.from_index else args.rlogs
  everything = []
  with Pool() as pool:
    for path, rows in zip(paths, pool.imap(partial(run, logged if args.logged else replay), paths), strict=True):
      everything += rows
      if not args.quiet:
        report(os.path.relpath(path), rows)
  if len(paths) > 1:
    print()
    report(f"all ({len(paths)} segments)", everything)


if __name__ == "__main__":
  main()
