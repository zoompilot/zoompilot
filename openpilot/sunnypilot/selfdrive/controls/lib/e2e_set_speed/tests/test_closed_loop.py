"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Closed loop against a model fitted from our experimental-mode logs: it pulls toward its own
pace at about -0.02 m/s^2 per m/s (43 ACC set-speed steps, tau ~50 s), with a 0.5 s actuator lag.
The upper bound is upstream's cruise candidate, as in the planner with no lead, and the floor closes
on the set speed at the approach pace.
"""
import numpy as np
import pytest

from openpilot.cereal import log as cereal_log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL as DT
from openpilot.selfdrive.controls.lib.longitudinal_planner import get_cruise_accel
from openpilot.sunnypilot.selfdrive.car.tests.fakes import FakeParams
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.approach import approach_accel
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.controller import LIFT_FALL, E2ESetSpeedController
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.tests.helpers import ENGAGED, build_sm
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.limits import op_long_cruise_jerk

V_PREF = 20.  # m/s, the model's own pace
V_CRUISE = V_PREF + 6 * CV.MPH_TO_MS  # set 6 mph above it, the median gap in the logs
STANDARD = cereal_log.LongitudinalPersonality.standard
ACTUATOR_TAU = 0.5  # s


def simulate(enabled, seconds=60., tau=50., brake_at=None):
  """Rows of (t, v_ego, a_model, e2e candidate). brake_at: the model brakes at -0.5 from then on."""
  ctl = E2ESetSpeedController(params=FakeParams(ExperimentalModeSetSpeed=enabled), dt=DT)
  v, a_act, a_cruise, out = V_PREF, 0., 0., 0.
  log = []
  for i in range(int(seconds / DT)):
    t = i * DT
    a_model = float(np.clip((V_PREF - v) / tau, -1., 1.))
    if brake_at is not None and t >= brake_at:
      a_model = -0.5
    # the host passes last frame's cruise candidate at the approach pace, then builds this frame's
    e2e = ctl.update(build_sm(v), a_model, min(a_cruise, approach_accel(v, V_CRUISE, STANDARD)), **ENGAGED)
    a_cruise = get_cruise_accel(V_CRUISE, v, a_cruise, out, DT)
    out = min(e2e, a_cruise)
    a_act += DT / ACTUATOR_TAU * (out - a_act)
    v += a_act * DT
    log.append((t, v, a_model, e2e))
  return np.asarray(log)


def chill_peak(seconds=60.):
  """Top speed of the cruise candidate alone, from the model's pace."""
  v, a_act, a_cruise = V_PREF, 0., 0.
  peak = v
  for _ in range(int(seconds / DT)):
    a_cruise = get_cruise_accel(V_CRUISE, v, a_cruise, a_cruise, DT)
    a_act += DT / ACTUATOR_TAU * (a_cruise - a_act)
    v += a_act * DT
    peak = max(peak, v)
  return peak


def deficit_mph(log, after):
  return (V_CRUISE - log[log[:, 0] >= after, 1]).max() * CV.MS_TO_MPH


def test_model_alone_stays_under():
  assert deficit_mph(simulate(False), after=40.) > 5.


@pytest.mark.parametrize("tau, deficit", [(25., 1.), (50., 1.), (65., 1.)])  # the per-bundle spread in the logs
def test_reaches_set_speed(tau, deficit):
  log = simulate(True, tau=tau)
  assert deficit_mph(log, after=40.) < deficit
  assert deficit_mph(log, after=40.) < 0.6 * deficit_mph(simulate(False, tau=tau), after=40.)
  # no further past the set speed than chill mode's own cruise goes through the same lag
  assert log[:, 1].max() <= chill_peak() + 1e-3


def test_added_jerk_bounded():
  # the model never brakes in this run, so the lift never takes the gain's faster drop
  log = simulate(True)
  lift = log[:, 3] - log[:, 2]
  jerk = np.diff(lift) / DT
  assert -LIFT_FALL - 1e-6 <= jerk.min() and jerk.max() <= op_long_cruise_jerk(0.) + 1e-6
  assert lift.min() >= 0.


def test_yields_to_a_slowdown_within_half_a_second():
  # converged, then the model starts braking for something at t = 45 s
  log = simulate(True, brake_at=45.)
  after = log[log[:, 0] >= 45. + 0.5]
  assert np.allclose(after[:, 3], after[:, 2])
