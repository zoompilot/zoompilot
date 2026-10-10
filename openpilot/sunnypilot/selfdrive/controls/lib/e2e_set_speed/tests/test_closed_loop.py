"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Closed loop against a model fitted from our experimental-mode logs: it pulls toward its own
pace at about -0.02 m/s^2 per m/s (43 ACC set-speed steps, tau ~50 s), with a 0.5 s actuator lag.
The upper bound is the e2e cruise candidate, as in the planner with no lead.
"""
import numpy as np
import pytest

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL as DT
from openpilot.sunnypilot.selfdrive.car.tests.fakes import FakeParams
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed import controller as c
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.controller import E2ESetSpeedController
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.tests.helpers import ENGAGED, T_IDXS, build_sm

V_PREF = 20.  # m/s, the model's own pace
V_CRUISE = V_PREF + 6 * CV.MPH_TO_MS  # set 6 mph above it, the median gap in the logs
ACTUATOR_TAU = 0.5  # s


def simulate(enabled, seconds=60., tau=50., brake_at=None):
  """Rows of (t, v_ego, a_model, e2e candidate). brake_at: the model brakes at -0.5 from then on."""
  ctl = E2ESetSpeedController(params=FakeParams(ExperimentalModeSetSpeed=enabled), dt=DT)
  v, a_act = V_PREF, 0.
  log = []
  for i in range(int(seconds / DT)):
    t = i * DT
    a_model = float(np.clip((V_PREF - v) / tau, -1., 1.))
    plan_v = v + a_model * T_IDXS
    if brake_at is not None and t >= brake_at:
      a_model = -0.5
    e2e = ctl.update(build_sm(v, plan_v=plan_v), a_model, V_CRUISE, **ENGAGED)
    out = min(e2e, float(np.clip(V_CRUISE - v, -1.2, 2.)))
    ctl.delivered(out, e2e)
    a_act += DT / ACTUATOR_TAU * (out - a_act)
    v += a_act * DT
    log.append((t, v, a_model, e2e))
  return np.asarray(log)


def deficit_mph(log, after):
  return (V_CRUISE - log[log[:, 0] >= after, 1]).max() * CV.MS_TO_MPH


def test_model_alone_stays_under():
  assert deficit_mph(simulate(False), after=40.) > 5.


@pytest.mark.parametrize("tau, deficit", [
  (50., 1.), (65., 1.),
  # a bundle pushing back this hard plans a steady slowdown once past its pace, which the envelope
  # cannot tell from a real one: it closes about half the gap and stops short
  (25., 3.5),
])  # the per-bundle spread in the logs
def test_reaches_set_speed(tau, deficit):
  log = simulate(True, tau=tau)
  assert deficit_mph(log, after=40.) < deficit
  assert deficit_mph(log, after=40.) < 0.6 * deficit_mph(simulate(False, tau=tau), after=40.)
  assert log[:, 1].max() < V_CRUISE + 0.05


def test_added_jerk_bounded():
  log = simulate(True)
  added = np.diff(log[:, 3] - log[:, 2]) / DT
  assert np.abs(added).max() <= c.BOOST_RATE + 1e-6


def test_yields_to_a_slowdown_within_a_quarter_second():
  # converged, then the model starts braking for something at t = 45 s
  log = simulate(True, brake_at=45.)
  after = log[log[:, 0] >= 45. + 0.25]
  assert np.allclose(after[:, 3], after[:, 2])
