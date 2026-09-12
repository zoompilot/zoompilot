#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Compact SLA/ICBM timeline from a drive's rlogs.

Usage: python tools/mazda_long/icbm_sla/drive_timeline.py <route_dir_glob> [t_start] [t_end]
Prints one line per "interesting" frame: SLA state changes, ICBM state/button changes,
set speed (dash cluster + op setpoint) changes, speed limit changes, button events,
displayed alerts, engagement changes.
"""
import sys
import glob
import warnings

warnings.filterwarnings("ignore")
import collections

from openpilot.tools.lib.logreader import LogReader
from openpilot.common.constants import CV

MS_TO_MPH = CV.MS_TO_MPH


def run(seg_glob, t_start=None, t_end=None):
  paths = sorted(glob.glob(seg_glob), key=lambda p: int(p.split('--')[-1].split('/')[0]))
  rows = collections.defaultdict(dict)
  t0 = None
  for path in paths:
    for msg in LogReader(path):
      if t0 is None:
        t0 = msg.logMonoTime
      t = round((msg.logMonoTime - t0) / 1e9, 2)
      w = msg.which()
      if w == 'longitudinalPlanSP':
        a = msg.longitudinalPlanSP
        r = rows[t]
        r['sla'] = str(a.speedLimit.assist.state)
        r['sl'] = round(a.speedLimit.resolver.speedLimit * MS_TO_MPH, 1)
        r['slF'] = round(a.speedLimit.resolver.speedLimitFinalLast * MS_TO_MPH, 1)
        r['lpsrc'] = str(a.longitudinalPlanSource)
        r['vT'] = round(a.vTarget * MS_TO_MPH, 1)
      elif w == 'carState':
        cs = msg.carState
        r = rows[t]
        r['v'] = round(cs.vEgo * MS_TO_MPH, 1)
        r['eng'] = int(cs.cruiseState.enabled)
        r['dash'] = round(cs.cruiseState.speedCluster * MS_TO_MPH, 1)
        r['vCr'] = round(cs.vCruise * CV.KPH_TO_MPH, 1)
        for b in cs.buttonEvents:
          r.setdefault('btn', []).append(f"{str(b.type).replace('Cruise','')}{'v' if b.pressed else '^'}")
      elif w == 'carControlSP':
        try:
          i = msg.carControlSP.intelligentCruiseButtonManagement
          r = rows[t]
          r['icbm'] = str(i.state)
          r['iBtn'] = str(i.sendButton)
        except Exception:
          pass
      elif w == 'selfdriveState':
        ss = msg.selfdriveState
        if ss.alertText1:
          rows[t]['alert'] = ss.alertText1[:44]
      elif w == 'onroadEventsSP':
        names = [str(e.name) for e in msg.onroadEventsSP.events]
        if names:
          rows[t]['evSP'] = ','.join(n for n in names if 'speedLimit' in n or 'Limit' in n)

  ts = sorted(rows)
  last = {}
  keys_watch = ('sla', 'iBtn', 'icbm', 'lpsrc', 'dash', 'vCr', 'eng', 'sl', 'alert')
  prev_watch = {}
  for t in ts:
    if t_start is not None and t < t_start:
      for k, v in rows[t].items():
        last[k] = v
      prev_watch = {k: last.get(k) for k in keys_watch}
      continue
    if t_end is not None and t > t_end:
      break
    r = rows[t]
    for k, v in r.items():
      last[k] = v
    cur_watch = {k: last.get(k) for k in keys_watch}
    change = cur_watch != prev_watch or 'btn' in r or (r.get('evSP'))
    if change:
      btn = ','.join(r.get('btn', [])) if 'btn' in r else ''
      ev = r.get('evSP', '')
      alert = last.get('alert') if last.get('alert') != prev_watch.get('alert') else ''
      line = f"t={t:8.2f} v={str(last.get('v')):5} dash={str(last.get('dash')):5} vCr={str(last.get('vCr')):5} eng={str(last.get('eng')):2}"
      line += f" sla={str(last.get('sla')):9.9s} sl={str(last.get('sl')):5} slF={str(last.get('slF')):5} lps={str(last.get('lpsrc'))[:14]:14s}"
      line += f" vT={str(last.get('vT')):5} icbm={str(last.get('icbm')):10.10s} iBtn={str(last.get('iBtn')):12.12s}"
      print(line
            + (f" BTN={btn}" if btn else '')
            + (f" EV={ev}" if ev else '')
            + (f" ALERT[{alert}]" if alert else ''))
    prev_watch = cur_watch


if __name__ == '__main__':
  g = sys.argv[1]
  ts_ = float(sys.argv[2]) if len(sys.argv) > 2 else None
  te_ = float(sys.argv[3]) if len(sys.argv) > 3 else None
  run(g, ts_, te_)
