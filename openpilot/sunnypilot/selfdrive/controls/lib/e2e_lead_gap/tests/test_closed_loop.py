"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Closed loop behind a steady lead, against a model fitted from our experimental-mode logs (31k
camera-lead frames, vEgo > 8 m/s, steady lead): it matches the lead's speed at 0.11 m/s^2 per
m/s and all but ignores the gap (0.0004 m/s^2 per m), so it holds wherever it starts. The MPC
candidate is upstream's solver on the same lead, as in the planner; 0.5 s actuator lag.
"""
import numpy as np
import pytest

from types import SimpleNamespace as NS

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL as DT
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.car.tests.fakes import FakeParams
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_lead_gap import controller as c
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_lead_gap.controller import E2ELeadGapController
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_lead_gap.tests.helpers import ENGAGED, build_sm, desired_gap

V_LEAD = 20.
STANDARD = log.LongitudinalPersonality.standard
START_EXCESS = 40.  # m, the logged median at 20 m/s is ~+27 to +52
ACTUATOR_TAU = 0.5  # s
K_VREL = 0.11  # 1/s
K_GAP = 0.0004  # 1/s^2
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]


def model_accel(gap, v):
  return float(np.clip(K_VREL * (V_LEAD - v) + K_GAP * (gap - desired_gap(v, V_LEAD, STANDARD)), -2., 2.))


def simulate(enabled, seconds=90., lead_brake_at=None):
  """Rows of (t, gap excess, v, a_model, e2e, a_mpc)."""
  ctl = E2ELeadGapController(params=FakeParams(ExperimentalModeLeadGap=enabled), dt=DT)
  mpc = LongitudinalMpc(dt=DT)
  v, a_act, out = V_LEAD, 0., 0.
  v_lead = V_LEAD
  gap = desired_gap(V_LEAD, V_LEAD, STANDARD) + START_EXCESS
  rows = []
  for i in range(int(seconds / DT)):
    t = i * DT
    a_lead = -1.5 if lead_brake_at is not None and t >= lead_brake_at else 0.
    lead = NS(present=True, dRel=gap, vLead=v_lead, aLeadK=a_lead, aLeadTau=0.3, modelProb=1., radar=False)
    mpc.set_weights(True, personality=STANDARD)
    mpc.set_cur_state(v, out)
    mpc.update(NS(leadOne=lead, leadTwo=NS(present=False)), personality=STANDARD)
    a_mpc = float(get_accel_from_plan(np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, mpc.v_solution),
                                      np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, mpc.a_solution), CONTROL_N_T_IDX, action_t=0.41))
    a_model = model_accel(gap, v)
    lead_v = [v_lead + a_lead * tt for tt in c.LEAD_T_IDXS]
    sm = build_sm(v, gap, v_lead=v_lead, a_lead=a_lead, lead_v=np.maximum(lead_v, 0.))
    e2e = ctl.update(sm, a_model, a_mpc, **ENGAGED)
    out = min(e2e, a_mpc)
    a_act += DT / ACTUATOR_TAU * (out - a_act)
    v = max(0., v + a_act * DT)
    v_lead = max(0., v_lead + a_lead * DT)
    gap += (v_lead - v) * DT
    rows.append((t, gap - desired_gap(v, v_lead, STANDARD), v, a_model, e2e, a_mpc))
  return np.asarray(rows)


def test_model_alone_stays_back():
  # its weak gap term creeps in ~0.1 m/s; after 90 s it is still far outside the band
  log_ = simulate(False)
  assert log_[-1, 1] > c.GAP_BP[1] + 10.


def test_closes_to_the_setting():
  log_ = simulate(True)
  # within the band in under 30 s (from 40 m out at 20 m/s), then held
  assert log_[log_[:, 0] >= 30., 1].max() < c.GAP_BP[1]
  assert c.GAP_BP[0] - 1. < log_[-1, 1] < c.GAP_BP[1]
  # never inside long_mpc's own gap, never faster than the lead by more than the cap allows
  assert log_[:, 1].min() > -1.
  assert (log_[:, 2] - V_LEAD).max() < 3.


def test_never_above_the_mpc():
  log_ = simulate(True)
  assert (np.minimum(log_[:, 4], log_[:, 5]) <= log_[:, 5] + 1e-9).all()
  lifted = log_[:, 4] > log_[:, 3] + 1e-6
  assert (log_[lifted, 4] <= log_[lifted, 5] + 1e-6).all()


def test_added_jerk_bounded():
  log_ = simulate(True)
  added = np.diff(log_[:, 4] - log_[:, 3]) / DT
  assert added.max() <= c.BOOST_RISE + 1e-6


@pytest.mark.parametrize("brake_at", [10., 30.])
def test_drops_out_when_the_lead_brakes(brake_at):
  log_ = simulate(True, seconds=brake_at + 3., lead_brake_at=brake_at)
  after = log_[log_[:, 0] >= brake_at + 0.25]
  assert np.allclose(after[:, 4], after[:, 3])
