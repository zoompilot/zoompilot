#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

3d: run the real AdvertisedLead over route_4d vision-lead stream.
- disengage-with-lead events: how long does leadVisible persist after
- coast episodes (visible but leadOne zeroed): length + advertised d trajectory
- any advertised d <= 0 or > 200 m
"""
import sys, os
sys.path.insert(0, "/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot")
sys.path.insert(0, "/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot/opendbc_repo")
os.chdir("/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot")
from openpilot.tools.lib.logreader import LogReader
from opendbc.car.mazda.longitudinal import AdvertisedLead
from opendbc.car.mazda import mazdacan

BASE = "tools/mazda_long/test_data/route_4d_cx9/0000004d--3242c750a7--"
rows = []
cc = lead = None
for seg in range(10):
  path = BASE + f"{seg}/rlog.zst"
  if not os.path.exists(path):
    continue
  for m in LogReader(path):
    w = m.which()
    if w == "carControl":
      cc = m.carControl
    elif w == "carControlSP":
      lo = m.carControlSP.leadOne
      lead = (lo.dRel, lo.vRel)
    elif w == "carState":
      if cc is not None and lead is not None:
        rows.append((m.logMonoTime * 1e-9, cc.enabled, cc.longActive,
                     cc.hudControl.leadVisible, lead[0], lead[1],
                     m.carState.cruiseState.enabled, m.carState.vEgo))

print(f"rows={len(rows)}")
adv = AdvertisedLead()
t0 = rows[0][0]
prev_en = False
diseng = []            # [t, had_lead(d,v), leadVisible frames after, still-advertised frames after]
coast = None           # active coast episode [t_start, d0, v0, frames, dmin, dmax, dlast]
coasts = []
bad_adv = []           # advertised d <=0 or >200
adv_min, adv_max = 1e9, -1e9
pending = []           # disengage events being measured

for i, (t, en, lact, vis, d, v, cs_en, vego) in enumerate(rows):
  adv.update(vis, d, v, holding=False, escort=None)
  a = adv.lead
  if a is not None:
    adv_min = min(adv_min, a[0]); adv_max = max(adv_max, a[0])
    if a[0] <= 0. or a[0] > 200.:
      if len(bad_adv) < 20:
        bad_adv.append((round(t - t0, 2), round(a[0], 2), round(a[1], 2), vis, d))
  # coast episode: visible latched but measurement gone (leadOne zeroed)
  measuring = 0. < d <= mazdacan.DIST_OBJ_MAX
  if adv.visible and not measuring and adv._measured is not None:
    if coast is None:
      coast = [t - t0, adv._measured[0], adv._measured[1], 1, adv._measured[0], adv._measured[0]]
    else:
      coast[3] += 1
      coast[4] = min(coast[4], adv._measured[0])
      coast[5] = max(coast[5], adv._measured[0])
  elif coast is not None:
    coasts.append(coast)
    coast = None
  # disengage detection
  if prev_en and not cs_en:
    diseng.append({"t": round(t - t0, 2), "lead": (round(d, 1), round(v, 1)) if d > 0 else None,
                   "vis": vis, "vis_after": 0, "adv_after": 0, "i": i, "vego": round(vego, 1)})
  prev_en = cs_en
  for ev in diseng:
    if ev["i"] < i:
      if vis and (i - ev["i"]) == ev["vis_after"] + 1:
        ev["vis_after"] = i - ev["i"]
      if adv.has_lead and (i - ev["i"]) == ev["adv_after"] + 1:
        ev["adv_after"] = i - ev["i"]
if coast is not None:
  coasts.append(coast)

print(f"advertised d range over route: [{adv_min:.2f}, {adv_max:.2f}] m")
print(f"advertised d <=0 or >200 frames: {len(bad_adv)}")
for b in bad_adv:
  print("  ", b)
print(f"disengage events: {len(diseng)}")
for ev in diseng:
  print(f"  t+{ev['t']:8.2f} v={ev['vego']:5.1f} lead_at_diseng={ev['lead']} vis={ev['vis']} "
        f"leadVisible_persists={ev['vis_after']*0.01:.2f}s advertised_persists={ev['adv_after']*0.01:.2f}s")
print(f"coast episodes (visible, measurement gone): {len(coasts)}")
long_c = sorted(coasts, key=lambda c: -c[3])[:15]
for c in long_c:
  print(f"  t+{c[0]-t0 if c[0]>1e6 else c[0]:8.2f} d0={c[1]:7.2f} v0={c[2]:+6.2f} frames={c[3]} dmin={c[4]:7.2f} dmax={c[5]:7.2f}")
if coasts:
  print(f"max coast length: {max(c[3] for c in coasts)} frames; min coasted d: {min(c[4] for c in coasts):.2f} m; max coasted d: {max(c[5] for c in coasts):.2f} m")
