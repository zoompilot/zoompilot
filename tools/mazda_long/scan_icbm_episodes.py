#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Scan rlogs for ICBM (Intelligent Cruise Button Management) misbehavior episodes.

Flags:
  E_OVERSHOOT    dash cluster speed rises above openpilot's internal vCruise while engaged
  E_RESUME_GAS   cruiseControl.resume overlapping with driver gas press
  E_VCRUISE_JUMP internal vCruise jumps by more than one increment in a single frame
  E_FIGHT        driver wheel button press while ICBM is actively sending buttons

For each flagged segment, writes a full-timeline CSV (100 Hz, one row per carState)
so episodes can be inspected in detail. Also writes an episodes index JSON.

Usage: scan_icbm_episodes.py <rlog_root_dir> <out_dir> [workers]
"""
import csv
import json
import os
import sys
import traceback
from functools import partial
from multiprocessing import Pool

import zstandard as zstd
from cereal import log

ICBM_STATES = ['inactive', 'preActive', 'increasing', 'decreasing', 'holding']

CRZ_BTNS = 0x9D  # 157

COLS = ['t', 'vego', 'gas', 'brake', 'standstill', 'crz_standstill', 'enabled', 'override', 'resume', 'cancel_cc',
        'vcruise_kph', 'cluster_kph', 'icbm_state', 'icbm_send', 'icbm_vtarget_kph', 'lp_vtarget_kph', 'lp_source',
        'btn_accel', 'btn_decel', 'btn_cancel',
        'wheel_setp', 'wheel_setm', 'wheel_res', 'wheel_canoff',
        'sent_setp', 'sent_setm', 'sent_res', 'sent_cancel',
        'wsp_lvl', 'wsm_lvl', 'wres_lvl', 'vcruise_cluster_kph']
I = {c: i for i, c in enumerate(COLS)}  # column name -> row index; single source of truth


def find_runs(rows, pred, min_len):
  """Yield maximal slices of consecutive rows satisfying pred, at least min_len long."""
  start = None
  for i, r in enumerate(rows):
    if pred(r):
      if start is None:
        start = i
    elif start is not None:
      if i - start >= min_len:
        yield rows[start:i]
      start = None
  if start is not None and len(rows) - start >= min_len:
    yield rows[start:]


def decode_crz_btns(dat):
  b0 = dat[0]
  return {
    'canoff': b0 & 0x01,
    'res': (b0 >> 2) & 1,
    'setp': (b0 >> 4) & 1,
    'setm': (b0 >> 5) & 1,
  }


def process_segment(seg_dir, out_dir=None):
  rlog = os.path.join(seg_dir, 'rlog.zst')
  seg = os.path.basename(seg_dir.rstrip('/'))
  try:
    with open(rlog, 'rb') as f:
      data = zstd.ZstdDecompressor().stream_reader(f).read()
  except Exception as e:
    return {'segment': seg, 'error': f'read: {e}'}

  rows = []
  # current values
  enabled = override = resume = cancel_cc = 0
  icbm_state = icbm_send = 0
  icbm_vtarget = 0.0
  lp_vtarget = 0.0
  lp_source = 0
  # CAN accumulators since last carState row
  wheel = {'setp': 0, 'setm': 0, 'res': 0, 'canoff': 0}
  sent = {'setp': 0, 'setm': 0, 'res': 0, 'canoff': 0}
  wheel_last = {'setp': 0, 'setm': 0, 'res': 0, 'canoff': 0}
  t0 = None

  try:
    for evt in log.Event.read_multiple_bytes(data):
      w = evt.which()
      t = evt.logMonoTime / 1e9
      if t0 is None:
        t0 = t

      if w == 'carControl':
        cc = evt.carControl
        enabled = int(cc.enabled)
        override = int(cc.cruiseControl.override)
        resume = int(cc.cruiseControl.resume)
        cancel_cc = int(cc.cruiseControl.cancel)
      elif w == 'selfdriveStateSP':
        icbm = evt.selfdriveStateSP.intelligentCruiseButtonManagement
        icbm_state = icbm.state.raw
        icbm_send = icbm.sendButton.raw
        icbm_vtarget = icbm.vTarget
      elif w == 'longitudinalPlanSP':
        lp = evt.longitudinalPlanSP
        lp_vtarget = lp.vTarget
        lp_source = lp.longitudinalPlanSource.raw
      elif w == 'can':
        for m in evt.can:
          if m.address == CRZ_BTNS and m.src == 0:
            bits = decode_crz_btns(m.dat)
            for k, v in bits.items():
              if v and not wheel_last[k]:
                wheel[k] += 1  # rising edges only
              wheel_last[k] = v
      elif w == 'sendcan':
        for m in evt.sendcan:
          if m.address == CRZ_BTNS:
            bits = decode_crz_btns(m.dat)
            for k, v in bits.items():
              if v:
                sent[k] += 1  # every asserted frame counts
      elif w == 'carState':
        cs = evt.carState
        btn_accel = btn_decel = btn_cancel = 0
        for b in cs.buttonEvents:
          bt = str(b.type)
          if b.pressed:
            if bt == 'accelCruise':
              btn_accel = 1
            elif bt == 'decelCruise':
              btn_decel = 1
            elif bt == 'cancel':
              btn_cancel = 1
        rows.append((
          round(t - t0, 3), round(cs.vEgo, 2), int(cs.gasPressed), int(cs.brakePressed),
          int(cs.standstill), int(cs.cruiseState.standstill),
          enabled, override, resume, cancel_cc,
          round(cs.vCruise, 2), round(cs.cruiseState.speed * 3.6, 2),
          icbm_state, icbm_send, round(icbm_vtarget * 3.6, 2), round(lp_vtarget * 3.6, 2), lp_source,
          btn_accel, btn_decel, btn_cancel,
          wheel['setp'], wheel['setm'], wheel['res'], wheel['canoff'],
          sent['setp'], sent['setm'], sent['res'], sent['canoff'],
          wheel_last['setp'], wheel_last['setm'], wheel_last['res'],
          round(cs.vCruiseCluster, 2),
        ))
        wheel = dict.fromkeys(wheel, 0)
        sent = dict.fromkeys(sent, 0)
  except Exception as e:
    return {'segment': seg, 'error': f'parse: {e} {traceback.format_exc(limit=1)}'}

  if not rows:
    return {'segment': seg, 'episodes': [], 'n_rows': 0}

  episodes = []
  i_t, i_vego, i_gas, i_en, i_res = (I[c] for c in ('t', 'vego', 'gas', 'enabled', 'resume'))
  i_vcr, i_clu, i_opclu, i_state = (I[c] for c in ('vcruise_kph', 'cluster_kph', 'vcruise_cluster_kph', 'icbm_state'))
  i_ba, i_bd = I['btn_accel'], I['btn_decel']
  i_wsp, i_wsm, i_wres = I['wheel_setp'], I['wheel_setm'], I['wheel_res']
  i_wsp_lvl, i_wsm_lvl = I['wsp_lvl'], I['wsm_lvl']

  def add(kind, ts, te, detail):
    episodes.append({'kind': kind, 't_start': ts, 't_end': te, **detail})

  # E_OVERSHOOT: cluster > vcruise + 2.5 kph while enabled, sustained >= 0.3s (30 rows)
  def overshooting(r):
    return r[i_en] and r[i_vcr] < 250 and r[i_clu] > 1 and (r[i_clu] - r[i_vcr]) > 2.5
  for seg_rows in find_runs(rows, overshooting, 30):
    add('E_OVERSHOOT', seg_rows[0][i_t], seg_rows[-1][i_t],
        {'max_gap_kph': round(max(x[i_clu] - x[i_vcr] for x in seg_rows), 1),
         'vcruise_kph': seg_rows[0][i_vcr], 'max_cluster_kph': max(x[i_clu] for x in seg_rows)})

  # E_RESUME_GAS: resume asserted while gas pressed
  for seg_rows in find_runs(rows, lambda r: r[i_res] and r[i_gas], 5):
    add('E_RESUME_GAS', seg_rows[0][i_t], seg_rows[-1][i_t], {'frames': len(seg_rows)})

  # E_VCRUISE_JUMP: vCruise changes > 2 kph in one frame while enabled (not init/unset transitions)
  jumps = []
  for i in range(1, len(rows)):
    a, b = rows[i - 1], rows[i]
    if a[i_en] and b[i_en] and a[i_vcr] < 250 and b[i_vcr] < 250 and abs(b[i_vcr] - a[i_vcr]) > 2.0:
      jumps.append(i)
      add('E_VCRUISE_JUMP', b[i_t], b[i_t],
          {'from_kph': a[i_vcr], 'to_kph': b[i_vcr], 'gas': b[i_gas],
           'btn_accel': b[i_ba], 'btn_decel': b[i_bd],
           'held_p': b[i_wsp_lvl], 'held_m': b[i_wsm_lvl], 'vego_kph': round(b[i_vego] * 3.6, 1)})

  # E_RUNAWAY: >= 3 vCruise jumps within 2s of each other
  grp = []
  for j in [*jumps, None]:
    if grp and (j is None or rows[j][i_t] - rows[grp[-1]][i_t] >= 2.0):
      if len(grp) >= 3:
        add('E_RUNAWAY', rows[grp[0]][i_t], rows[grp[-1]][i_t],
            {'n_jumps': len(grp), 'from_kph': rows[grp[0] - 1][i_vcr], 'to_kph': rows[grp[-1]][i_vcr],
             'held_p_any': int(any(rows[k][i_wsp_lvl] for k in grp)),
             'held_m_any': int(any(rows[k][i_wsm_lvl] for k in grp))})
      grp = []
    if j is not None:
      grp.append(j)

  # E_DASH_DIVERGE: dash (cluster_kph) differs from openpilot's cluster belief (vCruiseCluster)
  # by > 3 kph sustained >= 0.5s while enabled, with no wheel button held
  def diverged(r):
    return (r[i_en] and r[i_opclu] < 250 and r[i_clu] > 1 and not r[i_wsp_lvl] and not r[i_wsm_lvl]
            and abs(r[i_clu] - r[i_opclu]) > 3.0)
  for seg_rows in find_runs(rows, diverged, 50):
    add('E_DASH_DIVERGE', seg_rows[0][i_t], seg_rows[-1][i_t],
        {'max_diff_kph': round(max(abs(x[i_clu] - x[i_opclu]) for x in seg_rows), 1),
         'dash_kph': seg_rows[-1][i_clu], 'op_cluster_kph': seg_rows[-1][i_opclu]})

  # E_FIGHT: real wheel button press within 1s of ICBM actively sending (state increasing/decreasing)
  send_active = [1 if r[i_state] in (2, 3) else 0 for r in rows]
  for i, r in enumerate(rows):
    if r[i_wsp] or r[i_wsm]:
      lo, hi = max(0, i - 100), min(len(rows), i + 100)
      if any(send_active[lo:hi]):
        add('E_FIGHT', r[i_t], r[i_t],
            {'wheel_setp': r[i_wsp], 'wheel_setm': r[i_wsm],
             'icbm_state': ICBM_STATES[r[i_state]] if r[i_state] < 5 else r[i_state],
             'vcruise_kph': r[i_vcr], 'cluster_kph': r[i_clu], 'gas': r[i_gas]})

  # stats
  stats = {
    'engaged_s': round(sum(1 for r in rows if r[i_en]) / 100., 1),
    'resume_s': round(sum(1 for r in rows if r[i_res]) / 100., 1),
    'gas_s': round(sum(1 for r in rows if r[i_gas]) / 100., 1),
    'sent_setp': sum(r[I['sent_setp']] for r in rows), 'sent_setm': sum(r[I['sent_setm']] for r in rows),
    'sent_res': sum(r[I['sent_res']] for r in rows),
    'wheel_setp': sum(r[i_wsp] for r in rows), 'wheel_setm': sum(r[i_wsm] for r in rows),
    'wheel_res': sum(r[i_wres] for r in rows),
  }

  res = {'segment': seg, 'episodes': episodes, 'n_rows': len(rows), 'stats': stats}
  if out_dir is not None:
    # write the CSV here in the worker so the big row list never crosses the Pool pipe
    if episodes:
      csv_path = os.path.join(out_dir, seg + '.csv')
      with open(csv_path, 'w', newline='') as f:
        wr = csv.writer(f)
        wr.writerow(COLS)
        wr.writerows(rows)
      res['csv'] = csv_path
  else:
    res['_rows'] = rows  # ad-hoc/library use: hand the caller the full timeline
  return res


def main():
  root, out_dir = sys.argv[1], sys.argv[2]
  workers = int(sys.argv[3]) if len(sys.argv) > 3 else 8
  os.makedirs(out_dir, exist_ok=True)

  segs = sorted(d for d in (os.path.join(root, x) for x in os.listdir(root))
                if os.path.isfile(os.path.join(d, 'rlog.zst')))
  print(f'{len(segs)} segments')

  index = []
  with Pool(workers) as pool:
    for res in pool.imap_unordered(partial(process_segment, out_dir=out_dir), segs):
      index.append(res)
      eps = res.get('episodes')
      if eps:
        kinds = {}
        for e in eps:
          kinds[e['kind']] = kinds.get(e['kind'], 0) + 1
        print(f"{res['segment']}: {kinds}")
      elif res.get('error'):
        print(f"{res['segment']}: ERROR {res['error']}")

  index.sort(key=lambda x: x['segment'])
  with open(os.path.join(out_dir, 'index.json'), 'w') as f:
    json.dump(index, f, indent=1)

  n_ep = sum(len(x.get('episodes', [])) for x in index)
  print(f'\ntotal episodes: {n_ep}, index: {os.path.join(out_dir, "index.json")}')


if __name__ == '__main__':
  main()
