"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

How far ahead does each driving model read curvature well enough to plan on? For every modelV2
frame at 25 mph or more, compare the model's curvature at each path point (|orientationRate.z| /
velocity.x) with the curvature the car actually drove there (yaw rate / speed, smoothed 1 s, placed
by integrated distance). Split by modelV2.big and speed band (25-50, 50+ mph). Frames within 1 s of
a big/small switch are dropped, since they belong to neither model.

Tables:
- by distance ahead: read ratio (model / driven) on bends that need slowing from the current speed
  at A_LAT. How many metres of path the far planner can take at face value.
- by seconds ahead (arc length / v_ego, the unit near_d is set in): the same read ratio, the
  distinct driven bends behind it (b=), and the share of model-flagged points (the model's bend
  needs slowing at A_LAT) that are real: strict = the driven road within 15 m needs slowing at
  A_LAT, lenient = at A_LENIENT (a milder bend is there). Where the model stops reading bends it
  should and starts flagging bends that are not there.
- false-bend episodes per hour above 50 mph: the model flags a bend in the first T s and none of
  its flagged points there is real (runs merged across < 1 s gaps); past budget = the false point
  asks more than BUDGET of decel after a T_RESP lead. What a T s near window costs on the highway.

The vision planner's per-model near window (_NEAR_T / _NEAR_T_BIG in zoompilot/vision_controller.py)
comes from the by-time table: the bands from 0 s where the read holds (median >= 0.85, p25 >= 0.60,
>= 90% of flags lenient-real, >= 20 bends), stopping where a wider window adds false bends past the
budget above 50 mph. 2026-09 corpus: small held to 3 s, and 4 s took those from 0.3 to 1.2/h; big
read its 3-4 s band like small's 2-3 s. Re-run on a new model's drives before moving either. The
distance table uses the same frames and points (all frames, point 0 excluded).

Usage: model_reach.py [rlog_root] [cache_dir] [workers]
  Each rlog_root/<route>--<seg>/rlog.zst is cached as cache_dir/<route>--<seg>.npz (t, v, yaw, big,
  rz, vx, px, py; other keys are ignored and empty files skipped). A rlog_root that is not a
  directory ('-') reads cache_dir as it is.
"""
import os
import sys
from functools import partial
from multiprocessing import Pool

import numpy as np

from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import limits
try:
  from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.zoompilot import vision_controller as vc
except ImportError:  # a checkout from before the planner moved (a baseline worktree)
  from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control import vision_controller as vc

from extract import MPH, route_files, routes

HERE = os.path.dirname(os.path.abspath(__file__))
KEYS = ('t', 'v', 'yaw', 'big', 'rz', 'vx', 'px', 'py')
# m/s2; a bend needs slowing when sqrt(A_LAT / kappa) < v. Fixed at the planner's highway ceiling
# (the set-speed table's 1.8 end) so runs against other ceilings, and curve_sim's scoring, compare
A_LAT = 1.8 * vc._PLAN_MARGIN
A_LENIENT = 1.5  # m/s2; lenient real-bend test
V_LO, V_HI = 25 / MPH, 22.4  # m/s; 25 and 50 mph
DIST_BP = [0, 20, 40, 60, 80, 100, 120, 140, 170, 200, 250]
TIME_BP = [0, 1, 2, 3, 4, 5, 6, 8]
ND, NT = len(DIST_BP) - 1, len(TIME_BP) - 1
EP_T = (3, 4, 8)  # s; near windows the false-bend rate is counted for
BUDGET, T_RESP = limits._STOCK_A_BUDGET['mazda'], limits._STOCK_RESPONSE_T  # m/s2 a false bend may ask, s response lead
OFFS = np.array([-15, -7.5, 0, 7.5, 15.])  # m; driven road around a flagged point
RB, NR = 0.005, 801  # read ratio histogram; last bin holds >= 4
STAB = 20  # frames each side the big flag must hold
MERGE = 20  # frames; false-bend runs closer than 1 s are one episode
BEND_GAP = 30.  # m; bend samples further apart are separate bends
MIN_CHUNK = 400  # frames; shorter continuous stretches are skipped
BLK = 20000  # frames per vectorized block
NMIN = 200  # samples or flags to print a cell


def extract(seg, root, cache):
  out = os.path.join(cache, seg + '.npz')
  if os.path.exists(out):
    return
  from openpilot.tools.lib.logreader import LogReader
  v, yaw, rows = np.nan, np.nan, []
  try:
    for m in LogReader(os.path.join(root, seg, 'rlog.zst')):
      w = m.which()
      if w == 'carState':
        v = m.carState.vEgo
      elif w == 'liveLocationKalman':
        av = m.liveLocationKalman.angularVelocityCalibrated.value
        if len(av) >= 3:
          yaw = av[2]
      elif w == 'livePose':
        yaw = m.livePose.angularVelocityDevice.z
      elif w == 'modelV2':
        mm = m.modelV2
        if len(mm.orientationRate.z) == 33 and len(mm.velocity.x) == 33:
          rows.append((m.logMonoTime * 1e-9, v, yaw, int(mm.big), list(mm.orientationRate.z), list(mm.velocity.x),
                       list(mm.position.x), list(mm.position.y)))
  except Exception:
    pass
  if rows:
    t, vv, yy, big, rz, vx, px, py = zip(*rows, strict=True)
    np.savez_compressed(out, t=np.array(t), v=np.array(vv), yaw=np.array(yy), big=np.array(big, np.int8),
                        rz=np.array(rz, np.float32), vx=np.array(vx, np.float32), px=np.array(px, np.float32),
                        py=np.array(py, np.float32))


def load_chunks(route, cache):
  """A route's cached frames as continuous stretches: segments joined, sorted on time and split
  at gaps over 1 s. Empty when the route has no frames."""
  parts = [p for p in (np.load(f) for f in route_files(route, cache)) if len(p['t'])]
  if not parts:
    return []
  d = {k: np.concatenate([p[k] for p in parts]) for k in KEYS}
  o = np.argsort(d['t'], kind='stable')
  d = {k: x[o] for k, x in d.items()}
  brk = np.flatnonzero(np.diff(d['t']) > 1.0) + 1
  return [{k: x[a:b] for k, x in d.items()} for a, b in zip(np.r_[0, brk], np.r_[brk, len(d['t'])], strict=True)]


def driven(c):
  """A stretch's speed, integrated distance and driven curvature (|yaw rate| / speed, smoothed 1 s)."""
  v = np.nan_to_num(c['v'].astype(float))
  s = np.cumsum(v * np.r_[0, np.diff(c['t'])])
  kt = np.convolve(np.abs(np.nan_to_num(c['yaw'].astype(float))) / np.maximum(v, 5.), np.ones(20) / 20, 'same')
  return v, s, kt


def count(key, shape):
  return np.bincount(key, minlength=int(np.prod(shape))).reshape(shape)


def episodes(mask):
  idx = np.flatnonzero(mask)
  return int(np.sum(np.r_[True, np.diff(idx) > MERGE])) if len(idx) else 0


def n_bends(pos):
  p = np.unique(np.round(pos))
  return int(np.sum(np.r_[True, np.diff(p) > BEND_GAP])) if len(p) else 0


def any_at(fr, mask, n):
  out = np.zeros(n, bool)
  out[fr[mask]] = True
  return out


def new_acc():
  # axes: model (small, big), speed band (25-50, 50+), ...
  return {'ratio': np.zeros((2, 2, ND + NT, NR), np.int64),  # distance bands, then time bands
          'flags': np.zeros((2, 2, NT, 3), np.int64),  # flagged points, real strict, real lenient
          'bends': np.zeros((2, 2, NT), np.int64),
          'ep': np.zeros((4, 2, 2, len(EP_T)), np.int64),  # false strict, lenient, lenient past budget; real strict
          'hours': np.zeros((2, 2)), 'switch_h': np.zeros(1), 'routes': np.zeros(1, np.int64)}


def chunk_stats(c, acc):
  t, n = c['t'], len(c['t'])
  v, s, kt = driven(c)
  dt = np.minimum(np.r_[0, np.diff(t)], 0.1)
  big = c['big'].astype(int)
  win = np.ones(2 * STAB + 1)
  nbig = np.convolve(big, win, 'same')
  stable = (nbig == 0) | (nbig == np.convolve(np.ones(n), win, 'same'))
  kap = np.abs(c['rz'].astype(float)) / np.maximum(c['vx'].astype(float), 0.5)
  px, py = c['px'].astype(float), c['py'].astype(float)
  dist = np.c_[np.zeros(n), np.cumsum(np.hypot(np.diff(px, axis=1), np.diff(py, axis=1)), axis=1)]
  valid = (v >= V_LO) & (s + dist[:, -1] <= s[-1]) & np.isfinite(kap).all(1) & np.isfinite(dist).all(1)
  acc['switch_h'] += dt[valid & ~stable].sum() / 3600
  ok = valid & stable
  hwy = (v >= V_HI).astype(int)
  for b in (0, 1):
    for sb in (0, 1):
      acc['hours'][b, sb] += dt[ok & (big == b) & (hwy == sb)].sum() / 3600
  false = np.zeros((4, len(EP_T), n), bool)
  bend_pos = {}
  pt = np.arange(33) >= 1  # point 0 is the car
  idx = np.flatnonzero(ok)
  for lo in range(0, len(idx), BLK):
    ii = idx[lo:lo + BLK]
    K, D, V = kap[ii], dist[ii], v[ii][:, None]
    g = (big[ii] * 2 + hwy[ii])[:, None]
    pos = s[ii][:, None] + D
    TT = D / V
    tb = np.searchsorted(TIME_BP, TT, 'right') - 1
    db = np.searchsorted(DIST_BP, D, 'right') - 1
    tin, din = pt & (tb < NT), pt & (db < ND)

    # read ratio where the driven road needs slowing
    ktd = np.interp(pos, s, kt)
    need = pt & (ktd > 1e-4) & (np.sqrt(A_LAT / np.maximum(ktd, 1e-12)) < V)
    r = np.minimum((K / np.maximum(ktd, 1e-12) / RB).astype(np.int64), NR - 1)
    for m, band in ((need & din, db), (need & tin, tb + ND)):
      acc['ratio'] += count(((g * (ND + NT) + band)[m]) * NR + r[m], acc['ratio'].shape)
    sel = need & tin
    gk, ps = np.broadcast_to(g, sel.shape)[sel] * NT + tb[sel], pos[sel]
    for key in np.unique(gk):
      bend_pos.setdefault(key, []).append(ps[gk == key])

    # model-flagged points, real if the driven road around them needs slowing
    fr, fc = np.nonzero(pt & (K > 1e-5) & (np.sqrt(A_LAT / np.maximum(K, 1e-12)) < V - 0.5))
    vf, kf, df, tf = V[fr, 0], K[fr, fc], D[fr, fc], TT[fr, fc]
    kk = np.max([np.interp(pos[fr, fc] + o, s, kt) for o in OFFS], axis=0)
    real_s = np.sqrt(A_LAT / np.maximum(kk, 1e-12)) < vf - 0.5
    real_l = np.sqrt(A_LENIENT / np.maximum(kk, 1e-12)) < vf - 0.5
    ft = tb[fr, fc]
    m = ft < NT
    k = (g[fr, 0] * NT + ft)[m] * 3
    sh = acc['flags'].shape
    acc['flags'] += count(k, sh) + count(k[real_s[m]] + 1, sh) + count(k[real_l[m]] + 2, sh)

    # per frame: any flag / any real flag inside the first T s
    past = (vf ** 2 - A_LAT / kf) / (2 * np.maximum(df - vf * T_RESP, 0.5)) > BUDGET
    for wi, T in enumerate(EP_T):
      inT = tf < T
      flagged, any_s, any_l = (any_at(fr, inT & x, len(ii)) for x in (True, real_s, real_l))
      false[0, wi, ii] = flagged & ~any_s
      false[1, wi, ii] = flagged & ~any_l
      false[2, wi, ii] = any_at(fr, inT & ~real_l & past, len(ii)) & ~any_l
      false[3, wi, ii] = any_s

  for key, lst in bend_pos.items():
    acc['bends'].flat[key] += n_bends(np.concatenate(lst))
  for kind in range(4):
    for wi in range(len(EP_T)):
      for b in (0, 1):
        for sb in (0, 1):
          acc['ep'][kind, b, sb, wi] += episodes(false[kind, wi] & (big == b) & (hwy == sb))


def route_stats(route, cache):
  acc = new_acc()
  chunks = load_chunks(route, cache)
  acc['routes'] += bool(chunks)
  for c in chunks:
    if len(c['t']) >= MIN_CHUNK:
      chunk_stats(c, acc)
  return acc


def quantiles(h, qs=(0.5, 0.25, 0.75)):
  n, cum, out = h.sum(), np.cumsum(h), []
  for q in qs:
    k = int(np.searchsorted(cum, q * n))
    out.append((k + (q * n - (cum[k - 1] if k else 0)) / max(h[k], 1)) * RB)
  return out


def read_cell(h):
  n = int(h.sum())
  return f"{'.':>4s}{'':13s} n={n:7d}" if n < NMIN else "{:4.2f} ({:4.2f}-{:4.2f})".format(*quantiles(h)) + f" n={n:7d}"


def pct(num, den):
  return f"{100 * num / den:3.0f}%" if den >= NMIN else "  . "


def report(acc):
  H = acc['hours']
  hours = f"small {H[0, 0]:.2f} / {H[0, 1]:.2f}, big {H[1, 0]:.2f} / {H[1, 1]:.2f}"
  print(f"{int(acc['routes'][0])} routes; valid hours 25-50 / 50+ mph: {hours}; dropped near a model switch {acc['switch_h'][0]:.3f} h")
  bands = (('25-50 mph', 0), ('50+ mph', 1))
  for label, sb in bands:
    print(f"\nmodel read / driven curvature on bends that need slowing, {label}, by distance: median (p25-p75) n")
    print(" dist_m    small                            big")
    for k in range(ND):
      print(f"{DIST_BP[k]:4d}-{DIST_BP[k + 1]:<4d}" + ''.join("  " + read_cell(acc['ratio'][b, sb, k]) for b in (0, 1)))
  for label, sb in bands:
    print(f"\n{label}, by seconds ahead: read median (p25-p75) n on bends that need slowing, distinct bends b; flags real strict / lenient (flags)")
    print(" t_s   small" + ' ' * 58 + "big")
    for k in range(NT):
      row = f"{TIME_BP[k]}-{TIME_BP[k + 1]} s"
      for b in (0, 1):
        tot, rs, rl = acc['flags'][b, sb, k]
        row += f"  {read_cell(acc['ratio'][b, sb, ND + k])} b={acc['bends'][b, sb, k]:4d}  {pct(rs, tot)} / {pct(rl, tot)} ({tot:6d})"
      print(row)
  print(f"\nfalse-bend episodes per hour above 50 mph (small {H[0, 1]:.2f} h, big {H[1, 1]:.2f} h): a bend flagged in the first T s, none of it real")
  print(f"past budget: a lenient false bend asking > {BUDGET} m/s2 after a {T_RESP:.0f} s lead; real/h: flagged bends that are real (strict)")
  print("   T   small: strict lenient past-budget real/h   big: strict lenient past-budget real/h")
  for wi, T in enumerate(EP_T):
    row = f"< {T} s"
    for b in (0, 1):
      e = acc['ep'][:, b, 1, wi] / max(H[b, 1], 1e-9)
      row += f"  {e[0]:12.1f} {e[1]:7.1f} {e[2]:11.1f} {e[3]:6.1f}"
    print(row)


def main():
  root = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, '..', 'device_data')
  cache = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '..', 'test_data', 'model_reach')
  workers = int(sys.argv[3]) if len(sys.argv) > 3 else 8
  os.makedirs(cache, exist_ok=True)
  segs = sorted(d for d in os.listdir(root) if d.count('--') == 2) if os.path.isdir(root) else []
  with Pool(workers) as p:
    p.map(partial(extract, root=root, cache=cache), segs, chunksize=2)
    acc = new_acc()
    for st in p.imap_unordered(partial(route_stats, cache=cache), routes(cache)):
      acc = {k: acc[k] + st[k] for k in acc}
  report(acc)


if __name__ == '__main__':
  main()
