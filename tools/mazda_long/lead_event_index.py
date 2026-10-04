#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Lead-following index over every alpha-long segment: a per-modelV2-frame cache, then events.

cache:  one npz per segment in .lead_event_cache/ (keyed by size + a hash of the file head, so
        the same rlog copied into two drive dirs is read once). Segments without openpilot
        longitudinal engaged get a stub so a rerun skips them. load_segment(path) gives one
        segment's columns; load_routes(engaged_keys()) gives every cached drive stitched
        across segments. FIELDS lists the scalar columns, lead_* are leadsV3[0..1].
events: slowing lead, lead brake tap, cut-in, and experimental-mode following gap vs the
        gap long_mpc steers to (desired_follow_distance), with the binding plan source.

Usage: lead_event_index.py [--out summary.md] [--top 30] [--rebuild] [<rlog or dir> ...]
       (default: test_data/ and device_data/ beside this file)
"""
import argparse
import hashlib
import os
import re
import sys
from collections import Counter, defaultdict
from multiprocessing import Pool

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, ROOT)

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import COMFORT_BRAKE, STOP_DISTANCE, get_T_FOLLOW
from openpilot.tools.lib.logreader import LogReader

CACHE_DIR = os.path.join(HERE, '.lead_event_cache')
CACHE_VERSION = 3
DEFAULT_DIRS = ('test_data', 'device_data')
SERVICES = ('carState', 'controlsState', 'selfdriveState', 'radarState', 'longitudinalPlan', 'carControl')
PLAN_N = 17   # longitudinalPlan speeds/accels (CONTROL_N)
LEAD_N = 6    # leadsV3 trajectory points
PLAN_SOURCES = ('cruise', 'lead0', 'lead1', 'lead2', 'e2e')
PERSONALITIES = ('aggressive', 'standard', 'relaxed')

# scalar columns, one value per modelV2 frame
FIELDS = (
  't', 'vEgo', 'aEgo', 'vCruise', 'longActive', 'ccLongActive', 'experimental', 'personality',
  'gas', 'brake', 'laneChange', 'desiredCurvature',
  'leadPresent', 'dRel', 'vLead', 'aLeadK', 'aLeadTau', 'modelProb', 'leadRadar', 'yRel',
  'modelAccel', 'aTarget', 'planSource', 'hasLead', 'fcw', 'accel',
  'spPresent', 'vTargetSP', 'e2eAuthority', 'e2eGain', 'e2eFloor', 'e2eBoost', 'e2eInhibit',
  'lead2Present', 'dRel2', 'vLead2', 'aLeadK2', 'aLeadTau2', 'modelProb2', 'lead2Radar', 'modelVEgo',
)
LEAD_FIELDS = ('x', 'xStd', 'v', 'vStd', 'a', 'aStd', 't')  # leadsV3[k].<f>, shape (n, 2, LEAD_N)


def fixed(lst, n):
  out = np.full(n, np.nan, dtype=np.float32)
  k = min(len(lst), n)
  if k:
    out[:k] = list(lst)[:k]
  return out


def file_key(path):
  st = os.stat(path)
  with open(path, 'rb') as f:
    head = f.read(1 << 16)
  return hashlib.sha1(str(st.st_size).encode() + head).hexdigest()[:20]


def route_seg(path):
  """(route, segment number) from the dir name (dongle--route--N) or a seg<N> filename."""
  parent = os.path.basename(os.path.dirname(path))
  m = re.match(r'(.+)--(\d+)$', parent)
  if m:
    return m.group(1), int(m.group(2))
  name = os.path.basename(path)
  m = re.search(r'seg_?(\d+)', name)
  seg = int(m.group(1)) if m else 0
  base = re.sub(r'seg_?\d+', 'seg', name)
  return os.path.join(os.path.dirname(path), base), seg


def extract(path):
  """Per-modelV2 columns for one rlog, or None when openpilot long never engaged."""
  sm = {}
  op_long = None
  sp_seen = False
  rows, leads, plans = [], [], []
  for m in LogReader(path, sort_by_time=True):
    w = m.which()
    if w == 'carParams':
      op_long = bool(m.carParams.openpilotLongitudinalControl) if op_long is None else op_long or m.carParams.openpilotLongitudinalControl
      continue
    if w == 'longitudinalPlanSP':
      sm[w] = m.longitudinalPlanSP
      sp_seen = True
      continue
    if w in SERVICES:
      sm[w] = getattr(m, w)
      continue
    if w != 'modelV2' or not all(s in sm for s in SERVICES):
      continue
    md = m.modelV2
    CS, cs, ss, rs, lp, cc = (sm[s] for s in SERVICES)
    lead = rs.leadOne
    if sp_seen:
      sp = sm['longitudinalPlanSP']
      try:
        e = sp.zoompilot.e2eSetSpeed
        e2e = (e.authority, e.gain, e.floor, e.boost, e.inhibit.raw)
      except Exception:
        e2e = (np.nan,) * 5
      sp_row = (1.0, sp.vTarget, *e2e)
    else:
      sp_row = (0.0,) + (np.nan,) * 6
    rows.append((
      m.logMonoTime * 1e-9, CS.vEgo, CS.aEgo, CS.vCruise, cs.longControlState.raw != 0, cc.longActive,
      ss.experimentalMode, ss.personality.raw, CS.gasPressed, CS.brakePressed,
      md.meta.laneChangeState.raw, cs.desiredCurvature,
      lead.present, lead.dRel, lead.vLead, lead.aLeadK, lead.aLeadTau, lead.modelProb, lead.radar, lead.yRel,
      md.action.desiredAcceleration, lp.aTarget, lp.longitudinalPlanSource.raw, lp.hasLead, lp.fcw,
      cc.actuators.accel, *sp_row,
      rs.leadTwo.present, rs.leadTwo.dRel, rs.leadTwo.vLead, rs.leadTwo.aLeadK, rs.leadTwo.aLeadTau,
      rs.leadTwo.modelProb, rs.leadTwo.radar, md.velocity.x[0] if len(md.velocity.x) else np.nan,
    ))
    lv = np.full((2, len(LEAD_FIELDS), LEAD_N), np.nan, dtype=np.float32)
    lp3 = np.zeros((2, 2), dtype=np.float32)
    for k, ld in enumerate(list(md.leadsV3)[:2]):
      lp3[k] = (ld.prob, ld.probTime)
      for j, f in enumerate(LEAD_FIELDS):
        lv[k, j] = fixed(getattr(ld, f), LEAD_N)
    leads.append((lv, lp3))
    plans.append((fixed(lp.speeds, PLAN_N), fixed(lp.accels, PLAN_N)))

  if not rows:
    return None, op_long
  cols = np.asarray(rows, dtype=np.float64)
  # longControlState leaves off only under openpilot long; early alpha builds logged
  # carParams.openpilotLongitudinalControl False (or no carParams) while long was active
  if not cols[:, FIELDS.index('longActive')].any():
    return None, op_long
  out = {f: cols[:, i].astype(np.float64 if f == 't' else np.float32) for i, f in enumerate(FIELDS)}
  lv = np.stack([x[0] for x in leads])
  for j, f in enumerate(LEAD_FIELDS):
    out[f'lead_{f}'] = lv[:, :, j]
  lp3 = np.stack([x[1] for x in leads])
  out['lead_prob'] = lp3[:, :, 0]
  out['lead_probTime'] = lp3[:, :, 1]
  out['planSpeeds'] = np.stack([p[0] for p in plans])
  out['planAccels'] = np.stack([p[1] for p in plans])
  return out, op_long


def cache_path(key):
  return os.path.join(CACHE_DIR, f'{key}.npz')


def build(path, rebuild=False):
  """Extract one rlog into the cache. Returns (path, key, status)."""
  try:
    key = file_key(path)
    cp = cache_path(key)
    if not rebuild and os.path.exists(cp):
      with np.load(cp) as z:
        if int(z['version']) == CACHE_VERSION:
          return path, key, 'hit' if bool(z['engaged']) else 'skip'
    os.makedirs(CACHE_DIR, exist_ok=True)
    route, seg = route_seg(path)
    out, op_long = extract(path)
    meta = {'version': CACHE_VERSION, 'path': os.path.abspath(path), 'route': route, 'seg': seg,
            'opLong': bool(op_long), 'engaged': out is not None}
    tmp = cp + '.tmp.npz'
    np.savez_compressed(tmp, **meta, **(out or {}))
    os.replace(tmp, cp)
    return path, key, 'built' if out is not None else 'skip'
  except Exception as e:
    print(f"{path}: {e!r}", file=sys.stderr)
    return path, None, 'error'


def load_segment(path):
  """Cached columns for one rlog as a dict (empty when it never engaged); builds it if missing."""
  _, key, status = build(path)
  if status in ('skip', 'error'):
    return {}
  with np.load(cache_path(key)) as z:
    return {k: z[k] for k in z.files}


def load_key(key):
  with np.load(cache_path(key)) as z:
    return {k: z[k] for k in z.files}


def stitch(segs):
  """Concatenate a route's cached segments, split wherever time is not continuous."""
  segs = sorted(segs, key=lambda d: (int(d['seg']), float(d['t'][0])))
  pieces, cur = [], []
  for d in segs:
    if cur and not (0 < d['t'][0] - cur[-1]['t'][-1] < 0.5):
      pieces.append(cur)
      cur = []
    cur.append(d)
  if cur:
    pieces.append(cur)
  out = []
  for p in pieces:
    keys = [k for k in p[0] if isinstance(p[0][k], np.ndarray) and p[0][k].ndim >= 1 and len(p[0][k]) == len(p[0]['t'])]
    d = {k: np.concatenate([s[k] for s in p]) for k in keys}
    d['route'] = str(p[0]['route'])
    d['segs'] = [int(s['seg']) for s in p]
    d['paths'] = [str(s['path']) for s in p]
    d['segStart'] = np.concatenate([[int(s['seg'])] * len(s['t']) for s in p])
    d['segT0'] = np.concatenate([[s['t'][0]] * len(s['t']) for s in p])
    out.append(d)
  return out


def engaged_keys():
  """Cache keys of every engaged segment already extracted."""
  keys = []
  for fn in sorted(os.listdir(CACHE_DIR)):
    if fn.endswith('.npz') and '.tmp' not in fn:
      with np.load(os.path.join(CACHE_DIR, fn)) as z:
        if int(z['version']) == CACHE_VERSION and bool(z['engaged']):
          keys.append(fn[:-4])
  return keys


def load_routes(keys):
  """Stitched, time-continuous drives from cache keys of engaged segments."""
  by_route = defaultdict(list)
  for k in keys:
    d = load_key(k)
    if bool(d['engaged']):
      by_route[str(d['route'])].append(d)
  drives = []
  for segs in by_route.values():
    drives += stitch(segs)
  return drives


# ---- events ---------------------------------------------------------------------------------

def smooth(x, n=5):
  if len(x) < n:
    return x.copy()
  k = np.ones(n) / n
  pad = np.pad(x, (n // 2, n - 1 - n // 2), mode='edge')
  return np.convolve(pad, k, mode='valid')


def runs(mask):
  """(start, end) index pairs of True runs, end exclusive."""
  m = np.concatenate([[False], mask.astype(bool), [False]])
  d = np.diff(m.astype(np.int8))
  return list(zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1), strict=True))


def first_below(x, thr, lo, hi):
  idx = np.flatnonzero(x[lo:hi] < thr)
  return lo + idx[0] if len(idx) else None


def where(d, i):
  seg = int(d['segStart'][i])
  return d['route'], seg, float(d['t'][i] - d['segT0'][i])


def path_of(d, i):
  seg = int(d['segStart'][i])
  return d['paths'][d['segs'].index(seg)]


def desired_gap(v_ego, v_lead, personality):
  """The gap long_mpc's obstacle constraint settles at: x_lead + v_lead^2/2b == v_ego^2/2b + T*v + stop."""
  t_follow = np.array([get_T_FOLLOW(int(p)) for p in personality]) if np.ndim(personality) else get_T_FOLLOW(int(personality))
  return t_follow * v_ego + STOP_DISTANCE + (v_ego ** 2 - v_lead ** 2) / (2 * COMFORT_BRAKE)


def slowing_leads(d):
  W = int(8.0 / DT_MDL)
  out = []
  present = d['leadPresent'] > 0.5
  vl = smooth(d['vLead'], 10)
  hold = int(1.0 / DT_MDL)
  for a, b in runs(present):
    i = a
    while i < b - 10:
      hi = min(i + W, b)
      seg = vl[i:hi]
      jmin = i + int(np.argmin(seg))
      drop = vl[i] - vl[jmin]
      if drop < 3.0 or d['vEgo'][i] <= 3.0 or not d['longActive'][i]:
        i += 1
        continue
      # same lead the whole way: no frame-to-frame range jump
      if np.any(np.abs(np.diff(d['dRel'][i:jmin + 1])) > 5.0):
        i += 1
        continue
      onset = i + int(np.flatnonzero(vl[i:jmin + 1] >= vl[i] - 0.5)[-1])
      lead_dt = d['t'][jmin] - d['t'][onset]
      lead_decel = (vl[onset] - vl[jmin]) / max(lead_dt, DT_MDL)
      # a real car: no faster than ~6 m/s^2 smoothed, and still slow a second later. A step in
      # vision vLead (far-range estimate, track swap) fails one or the other.
      if lead_decel > 6.0 or jmin + hold > b or np.mean(vl[jmin:jmin + hold]) > vl[onset] - 2.5:
        i += 1
        continue
      end = min(jmin + int(2.0 / DT_MDL), b)
      pre = max(onset - int(3.0 / DT_MDL), 0)
      t0 = d['t'][onset]
      k3 = first_below(d['aTarget'], -0.3, pre, end)
      k10 = first_below(d['aTarget'], -1.0, pre, end)
      dv = np.maximum(d['vEgo'][onset:end] - d['vLead'][onset:end], 0.0)
      room = np.maximum(d['dRel'][onset:end] - STOP_DISTANCE / 2, 1.0)
      a_req = float(np.max(dv ** 2 / (2 * room))) if len(dv) else 0.0
      route, segn, ts = where(d, onset)
      out.append({
        'i': int(onset), 'iMin': int(jmin), 'route': route, 'seg': segn, 't_seg': ts, 'path': path_of(d, onset),
        'vEgo': float(d['vEgo'][onset]), 'vLead0': float(vl[onset]), 'vLead1': float(vl[jmin]),
        'leadDt': float(lead_dt), 'leadDecel': float(lead_decel),
        'dRel0': float(d['dRel'][onset]), 'minGap': float(np.min(d['dRel'][onset:end])),
        't_m03': None if k3 is None else float(d['t'][k3] - t0),
        't_m10': None if k10 is None else float(d['t'][k10] - t0),
        'minATarget': float(np.min(d['aTarget'][onset:end])), 'minAEgo': float(np.min(d['aEgo'][onset:end])),
        'aReq': a_req, 'exp': bool(d['experimental'][onset] > 0.5), 'fcw': bool(np.any(d['fcw'][pre:end] > 0.5)),
        'personality': PERSONALITIES[int(d['personality'][onset])] if int(d['personality'][onset]) < 3 else '?',
        'radar': bool(d['leadRadar'][onset] > 0.5),
        'brake': bool(np.any(d['brake'][onset:end] > 0.5)), 'gas': bool(np.any(d['gas'][onset:end] > 0.5)),
        'severity': float(lead_decel + a_req),
      })
      i = jmin + 1
  return out


def brake_taps(d):
  out = []
  present = d['leadPresent'] > 0.5
  n = len(d['t'])
  for a, b in runs(present & (d['aLeadK'] < -1.0)):
    dur = (b - a) * DT_MDL
    if dur >= 1.5 or not d['longActive'][a] or d['vEgo'][a] <= 3.0:
      continue
    after = min(b + int(3.0 / DT_MDL), n)
    if not np.all(present[a:after]) or np.any(np.abs(np.diff(d['dRel'][a:after])) > 5.0):
      continue
    if not np.any(d['aLeadK'][b:after] > -0.3):
      continue
    drop = d['vLead'][a] - np.min(d['vLead'][a:after])
    if drop > 2.0:
      continue
    route, segn, ts = where(d, a)
    out.append({'i': int(a), 'route': route, 'seg': segn, 't_seg': ts, 'path': path_of(d, a), 'dur': dur,
                'minALead': float(np.min(d['aLeadK'][a:b])), 'vDrop': float(drop), 'vEgo': float(d['vEgo'][a]),
                'dRel': float(d['dRel'][a]), 'minATarget': float(np.min(d['aTarget'][a:after])),
                'minAEgo': float(np.min(d['aEgo'][a:after])), 'exp': bool(d['experimental'][a] > 0.5)})
  return out


def cut_ins(d):
  out = []
  present = d['leadPresent'] > 0.5
  n = len(d['t'])
  jump = np.diff(d['dRel']) < -8.0
  for i in np.flatnonzero(jump & present[1:] & present[:-1]) + 1:
    if not d['longActive'][i] or d['vEgo'][i] <= 3.0:
      continue
    after = min(i + int(5.0 / DT_MDL), n)
    if after - i < 10 or not np.all(present[i:after]) or np.any(np.abs(np.diff(d['dRel'][i:after])) > 5.0):
      continue
    rise = float(np.max(smooth(d['vLead'][i:after])) - d['vLead'][i])
    if rise <= 1.0:
      continue
    route, segn, ts = where(d, i)
    a3 = min(i + int(3.0 / DT_MDL), n)
    out.append({'i': int(i), 'route': route, 'seg': segn, 't_seg': ts, 'path': path_of(d, i), 'vEgo': float(d['vEgo'][i]),
                'dBefore': float(d['dRel'][i - 1]), 'dAfter': float(d['dRel'][i]), 'vLead': float(d['vLead'][i]),
                'vLeadRise': rise, 'minATarget': float(np.min(d['aTarget'][i:a3])),
                'minAEgo': float(np.min(d['aEgo'][i:a3])), 'exp': bool(d['experimental'][i] > 0.5),
                'brake': bool(np.any(d['brake'][i:a3] > 0.5))})
  return out


def following(d):
  """Steady-state following frames: lead steady, ego matched to it, lead slower than the set speed."""
  m = ((d['longActive'] > 0.5) & (d['leadPresent'] > 0.5) & (np.abs(d['aLeadK']) < 0.3) & (d['vEgo'] > 5.0) &
       (np.abs(d['vEgo'] - d['vLead']) < 1.5) & (np.abs(d['aEgo']) < 0.5) & (d['vLead'] < d['vCruise'] / 3.6 - 1.0) &
       (d['gas'] < 0.5) & (d['brake'] < 0.5) & (d['personality'] < 3))
  if not m.any():
    return None
  des = desired_gap(d['vEgo'][m], d['vLead'][m], d['personality'][m])
  return {'exp': d['experimental'][m] > 0.5, 'pers': d['personality'][m].astype(int), 'gap': d['dRel'][m],
          'desired': des, 'src': d['planSource'][m].astype(int), 'vEgo': d['vEgo'][m], 'vLead': d['vLead'][m]}


def analyse(key_group):
  drives = load_routes(key_group)
  res = {'slow': [], 'tap': [], 'cut': [], 'follow': [], 'alpha_s': 0.0, 'exp_lead_s': 0.0, 'exp_s': 0.0}
  for d in drives:
    act = d['longActive'] > 0.5
    res['alpha_s'] += act.sum() * DT_MDL
    res['exp_s'] += (act & (d['experimental'] > 0.5)).sum() * DT_MDL
    res['exp_lead_s'] += (act & (d['experimental'] > 0.5) & (d['leadPresent'] > 0.5)).sum() * DT_MDL
    res['slow'] += slowing_leads(d)
    res['tap'] += brake_taps(d)
    res['cut'] += cut_ins(d)
    f = following(d)
    if f is not None:
      res['follow'].append(f)
  return res


# ---- report ---------------------------------------------------------------------------------

def pct(x, q):
  return f"{np.percentile(x, q):+.1f}" if len(x) else '-'


def fmt_t(x):
  return '-' if x is None else f"{x:+.1f}"


def summary(res, n_seg, n_engaged, top):
  L = ["# Lead events over alpha-long drives", ""]
  L.append(f"Segments scanned: {n_seg} unique, {n_engaged} with openpilot longitudinal engaged.")
  L.append(f"Alpha long engaged: {res['alpha_s'] / 3600:.2f} h; experimental mode {res['exp_s'] / 3600:.2f} h, " +
           f"of which with a lead {res['exp_lead_s'] / 3600:.2f} h.")
  L.append("")
  L.append("| event | count | exp mode |")
  L.append("|---|---|---|")
  for k, name in (('slow', 'slowing lead'), ('tap', 'lead brake tap'), ('cut', 'cut-in then accelerates')):
    L.append(f"| {name} | {len(res[k])} | {sum(e['exp'] for e in res[k])} |")
  L.append("")

  slow = res['slow']
  if slow:
    L.append("## Slowing lead: plan response")
    L.append("")
    L.append("Times are relative to the lead's decel onset (negative = plan was already braking).")
    L.append("")
    L.append("chill = openpilot long without experimental mode (long_mpc ACC). never = aTarget did not cross")
    L.append("the threshold from 3 s before onset to 2 s after the lead's minimum speed.")
    L.append("")
    L.append("| mode | n | lead decel p50 | aTarget<-0.3 median | p90 | never | aTarget<-1.0 median | p90 | never | min aTarget p50 | min aEgo p50 | fcw |")
    L.append("|---|---|---|---|---|---|---|---|---|---|---|---|")
    for lab, sel in (('exp', True), ('chill', False)):
      es = [e for e in slow if e['exp'] == sel]
      if not es:
        continue
      t3 = np.array([e['t_m03'] for e in es if e['t_m03'] is not None])
      t10 = np.array([e['t_m10'] for e in es if e['t_m10'] is not None])
      med = lambda k, es=es: np.median([e[k] for e in es])  # noqa: E731
      L.append(f"| {lab} | {len(es)} | {med('leadDecel'):.2f} | {pct(t3, 50)} | {pct(t3, 90)} | {sum(e['t_m03'] is None for e in es)} | " +
               f"{pct(t10, 50)} | {pct(t10, 90)} | {sum(e['t_m10'] is None for e in es)} | {med('minATarget'):+.2f} | " +
               f"{med('minAEgo'):+.2f} | {sum(e['fcw'] for e in es)} |")
    L.append("")

  follow = res['follow']
  if follow:
    cat = {k: np.concatenate([f[k] for f in follow]) for k in follow[0]}
    excess = cat['gap'] - cat['desired']
    L.append("## Following gap, steady state (|aLeadK| < 0.3, |vEgo - vLead| < 1.5, |aEgo| < 0.5, vLead < set - 1, vEgo > 5, no pedals)")
    L.append("")
    L.append("excess = dRel - (T_FOLLOW*vEgo + 6 + (vEgo^2 - vLead^2)/5), the gap long_mpc settles at.")
    L.append("")
    L.append("| mode | personality | min | excess p10 | p50 | p90 | ratio p50 | e2e | lead0 | cruise |")
    L.append("|---|---|---|---|---|---|---|---|---|---|")
    for lab, sel in (('exp', True), ('chill', False)):
      for p, pname in enumerate(PERSONALITIES):
        m = (cat['exp'] == sel) & (cat['pers'] == p)
        if m.sum() < 20:
          continue
        src = Counter(cat['src'][m])
        tot = m.sum()
        ratio = np.median(cat['gap'][m] / cat['desired'][m])
        L.append(f"| {lab} | {pname} | {tot * DT_MDL / 60:.0f} | {pct(excess[m], 10)} | {pct(excess[m], 50)} | " +
                 f"{pct(excess[m], 90)} | {ratio:.2f} | {src[4] / tot * 100:.0f}% | {src[1] / tot * 100:.0f}% | " +
                 f"{src[0] / tot * 100:.0f}% |")
    L.append("")
    m = cat['exp']
    if m.any():
      L.append("Exp mode, excess by speed (all personalities):")
      L.append("")
      L.append("| vEgo m/s | min | excess p10 | p50 | p90 | e2e binding |")
      L.append("|---|---|---|---|---|---|")
      for lo, hi in ((5, 10), (10, 15), (15, 20), (20, 25), (25, 40)):
        mm = m & (cat['vEgo'] >= lo) & (cat['vEgo'] < hi)
        if mm.sum() < 20:
          continue
        L.append(f"| {lo}-{hi} | {mm.sum() * DT_MDL / 60:.0f} | {pct(excess[mm], 10)} | {pct(excess[mm], 50)} | " +
                 f"{pct(excess[mm], 90)} | {(cat['src'][mm] == 4).mean() * 100:.0f}% |")
      L.append("")

  if slow:
    L.append(f"## Top {top} slowing-lead events by severity")
    L.append("")
    L.append("severity = lead decel (m/s^2, onset to min, 0.5 s smoothed) + peak constant decel ego needed to stop closing before dRel = 3 m.")
    L.append("aTgt columns: s from lead decel onset until aTarget first crosses; -3.0 means already below at the window start. " +
             "pedal: B brake, G gas pressed in the event.")
    L.append("")
    L.append("| sev | mode | route--seg @ s | vEgo | vLead start->end (s) | dRel0 / min | aTgt<-0.3 | <-1.0 | min aTarget | min aEgo | fcw | pedal |")
    L.append("|---|---|---|---|---|---|---|---|---|---|---|---|")
    for e in sorted(slow, key=lambda e: -e['severity'])[:top]:
      mode = ('exp' if e['exp'] else 'chill') + ('/' + e['personality'][:3])
      ped = ('B' if e['brake'] else '') + ('G' if e['gas'] else '')
      rname = os.path.relpath(e['route'], HERE) if os.sep in e['route'] else e['route']
      L.append(f"| {e['severity']:.2f} | {mode} | {rname}--{e['seg']} @ {e['t_seg']:.1f} | " +
               f"{e['vEgo']:.1f} | {e['vLead0']:.1f}->{e['vLead1']:.1f} ({e['leadDt']:.1f}) | {e['dRel0']:.0f} / {e['minGap']:.0f} | " +
               f"{fmt_t(e['t_m03'])} | {fmt_t(e['t_m10'])} | {e['minATarget']:+.2f} | {e['minAEgo']:+.2f} | " +
               f"{'Y' if e['fcw'] else ''} | {ped} |")
    L.append("")
    L.append("Paths:")
    L.append("")
    for e in sorted(slow, key=lambda e: -e['severity'])[:top]:
      L.append(f"- {os.path.relpath(e['path'], HERE)} @ {e['t_seg']:.1f} s")
    L.append("")
  return "\n".join(L)


def find_rlogs(targets):
  out = []
  for t in targets:
    if os.path.isfile(t):
      out.append(t)
      continue
    for dp, _, fns in os.walk(t + os.sep, followlinks=True):
      for fn in fns:
        if fn.startswith('._') or 'qlog' in fn or fn.endswith('.npz'):
          continue
        if fn in ('rlog', 'rlog.zst', 'rlog.bz2') or re.match(r'.*rlog.*\.(zst|bz2)$', fn):
          out.append(os.path.join(dp, fn))
  return sorted(set(out))


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('--out', help='write the markdown summary here as well as stdout')
  ap.add_argument('--top', type=int, default=30)
  ap.add_argument('--rebuild', action='store_true', help='re-extract even when cached')
  ap.add_argument('targets', nargs='*', default=[os.path.join(HERE, d) for d in DEFAULT_DIRS])
  args = ap.parse_args()
  os.makedirs(CACHE_DIR, exist_ok=True)

  paths = find_rlogs(args.targets)
  print(f"{len(paths)} rlogs", file=sys.stderr)
  status = Counter()
  keys = {}
  with Pool() as pool:
    for i, (_path, key, st) in enumerate(pool.imap_unordered(build if not args.rebuild else _rebuild, paths, chunksize=4)):
      status[st] += 1
      if key and st in ('hit', 'built'):
        keys[key] = True
      elif key:
        keys.setdefault(key, False)
      if (i + 1) % 200 == 0:
        print(f"  {i + 1}/{len(paths)} {dict(status)}", file=sys.stderr)
  print(f"cache: {dict(status)}", file=sys.stderr)

  engaged = [k for k, v in keys.items() if v]
  routes = defaultdict(list)
  for k in engaged:
    with np.load(cache_path(k)) as z:
      routes[str(z['route'])].append(k)
  res = {'slow': [], 'tap': [], 'cut': [], 'follow': [], 'alpha_s': 0.0, 'exp_lead_s': 0.0, 'exp_s': 0.0}
  with Pool() as pool:
    for r in pool.imap_unordered(analyse, list(routes.values())):
      for k in res:
        res[k] += r[k]
  text = summary(res, len(keys), len(engaged), args.top)
  print(text)
  if args.out:
    with open(args.out, 'w') as f:
      f.write(text + "\n")


def _rebuild(path):
  return build(path, rebuild=True)


if __name__ == "__main__":
  main()
