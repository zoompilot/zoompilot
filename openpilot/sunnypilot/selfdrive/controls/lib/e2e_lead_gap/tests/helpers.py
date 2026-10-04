"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_lead_gap.controller import LEAD_T_IDXS, desired_gap
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_set_speed.tests.helpers import MockParams, build_sm as build_nudge_sm

__all__ = ['ENGAGED', 'MockParams', 'build_sm', 'desired_gap']

# update() arguments for an engaged e2e frame
ENGAGED = {'is_e2e': True, 'reset_state': False, 'dec_active': False, 'allow_throttle': True, 'fcw': False}
STANDARD = log.LongitudinalPersonality.standard


def build_sm(v_ego: float, d_rel: float, v_lead: float | None = None, a_lead=0., prob=0.95, lead_v=None, lead2=None,
             personality=STANDARD, **kwargs) -> dict:
  """The nudge's fixture plus a lead (v_lead defaults to v_ego) and selfdriveState.personality.
  lead_v: the model's lead speed forecast at LEAD_T_IDXS. lead2: (d_rel, v_lead) for leadTwo."""
  sm = build_nudge_sm(v_ego, **kwargs)
  v_lead = v_ego if v_lead is None else v_lead

  rs = sm['radarState'].as_builder()
  for lead, (d, v) in ((rs.leadOne, (d_rel, v_lead)), (rs.leadTwo, lead2)) if lead2 else ((rs.leadOne, (d_rel, v_lead)),):
    lead.present = True
    lead.dRel = d
    lead.vLead = v
    lead.vRel = v - v_ego
    lead.aLeadK = a_lead
    lead.modelProb = prob
  sm['radarState'] = rs.as_reader()

  md = sm['modelV2'].as_builder()
  leads = md.init('leadsV3', 3)
  forecast = np.full(len(LEAD_T_IDXS), v_lead) if lead_v is None else np.asarray(lead_v, dtype=float)
  for lead in leads:
    lead.prob = prob
    lead.v = forecast.tolist()
  sm['modelV2'] = md.as_reader()

  ss = messaging.new_message('selfdriveState')
  ss.selfdriveState.personality = personality
  sm['selfdriveState'] = ss.selfdriveState.as_reader()
  return sm
