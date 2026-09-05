#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Derive the EPS effective-torque ceiling as a function of speed, corpus-wide.

Everything comes out of 0x241 STEER_RATE on bus 0, which the EPS itself transmits:
LKAS_REQUEST is the request it received, LKAS_EFFECTIVE is what it actually applied.
That makes this independent of who commanded (stock camera or openpilot) and of the
harness relay, which only affects bus 2.

The ceiling is only observable where the request exceeded delivery. A bin whose max
request never cleared its max effective proves only a lower bound, so saturated frames
(|req| - |eff| >= SAT_MARGIN) are scored separately and the rail is the modal |eff|
among them.

Because a lookup indexed on instantaneous speed is only valid if the ceiling IS a
function of instantaneous speed, the rail is also split by longitudinal acceleration
(is it the same rail accelerating through 30 mph as decelerating through it?) and by
steering direction. A split that disagrees means hysteresis and a plain lookup is the
wrong shape.
"""

import glob
import os
import sys
from collections import Counter
from multiprocessing import Pool

import numpy as np
import zstandard as zstd
from openpilot.cereal import log

SAT_MARGIN = 40
ACC_BAND = 0.3   # m/s2; |aEgo| below this counts as steady speed
CACHE = os.environ.get('CACHE', '/tmp/eps_ceiling_corpus.npz')


def be(d, start, size, off):
    raw = int.from_bytes(d, 'big')
    msb = (start // 8) * 8 + (7 - start % 8)
    return ((raw >> (64 - msb - size)) & ((1 << size) - 1)) - off


def worker(fp):
    """-> (v*10, a*100, angle*10, req, eff) int16 arrays for clean frames."""
    try:
        data = zstd.ZstdDecompressor().stream_reader(open(fp, 'rb')).read()
    except Exception:
        return None
    v, a, g, r, e = [], [], [], [], []
    vego = 0.0
    aego = 0.0
    angle = 0.0
    pressed = True
    try:
        for evt in log.Event.read_multiple_bytes(data):
            w = evt.which()
            if w == 'carState':
                cs = evt.carState
                vego, aego, angle = cs.vEgo, cs.aEgo, cs.steeringAngleDeg
                pressed = cs.steeringPressed
            elif w == 'can':
                for m in evt.can:
                    if m.src == 0 and m.address == 0x241:
                        d = bytes(m.dat)
                        if (d[6] >> 2) & 1:      # LKAS_BLOCK 50|1@1+ -> byte6 bit2
                            continue
                        if pressed or vego <= 2.0:
                            continue
                        v.append(int(vego * 10))
                        a.append(int(np.clip(aego * 100, -32000, 32000)))
                        g.append(int(np.clip(angle * 10, -32000, 32000)))
                        r.append(be(d, 3, 12, 2048))
                        e.append(be(d, 39, 12, 2048))
    except Exception:
        pass
    if not v:
        return None
    return tuple(np.array(x, np.int16) for x in (v, a, g, r, e))


def collect():
    files = sorted({os.path.realpath(p)
                    for p in glob.glob('tools/mazda_long/**/*.zst', recursive=True)})
    print(f'{len(files)} unique segments', flush=True)
    acc = [[] for _ in range(5)]
    done = ok = 0
    with Pool(int(os.environ.get('JOBS', 9))) as pool:
        for res in pool.imap_unordered(worker, files, chunksize=4):
            done += 1
            if res is not None:
                ok += 1
                for i in range(5):
                    acc[i].append(res[i])
            if done % 250 == 0:
                print(f'  {done}/{len(files)} segs, {ok} with data, '
                      f'{sum(len(x) for x in acc[0]):,} frames', flush=True)
    out = [np.concatenate(x) for x in acc]
    np.savez_compressed(CACHE, v=out[0], a=out[1], g=out[2], r=out[3], e=out[4])
    print(f'cached {len(out[0]):,} frames -> {CACHE}', flush=True)
    return out


def rail_of(eff_sat):
    """Modal |eff| in the top decile of a saturated population = the rail."""
    if len(eff_sat) < 50:
        return None, 0, 0.0
    thr = np.percentile(eff_sat, 90)
    hi = Counter(eff_sat[eff_sat >= thr].tolist())
    if not hi:
        return None, 0, 0.0
    val, cnt = hi.most_common(1)[0]
    return val, cnt, cnt / len(eff_sat)


def analyse(V, A, G, R, E):
    mph = V.astype(np.float64) / 10.0 * 2.237
    aeg = A.astype(np.float64) / 100.0
    ang = G.astype(np.float64) / 10.0
    req, eff = np.abs(R.astype(np.int32)), np.abs(E.astype(np.int32))
    sat = (req - eff) >= SAT_MARGIN

    print(f'\ncorpus: {len(req):,} clean frames, {sat.sum():,} saturated '
          f'({100 * sat.mean():.1f}%)   max|eff|={eff.max()}  max|req|={req.max()}')

    print(f'\n{"mph":>7} {"n":>9} {"nSat":>8} {"maxREQ":>7} {"maxEFF":>7} {"rail":>6} '
          f'{"share":>7}  verdict')
    print('-' * 78)
    curve = []
    for lo in range(2, 80, 2):
        m = (mph >= lo) & (mph < lo + 2)
        if m.sum() < 300:
            continue
        ms = m & sat
        rail, cnt, shr = rail_of(eff[ms])
        if rail is None:
            print(f'{lo:>3}-{lo+2:<3} {m.sum():>9,} {ms.sum():>8,} {req[m].max():>7} '
                  f'{eff[m].max():>7} {"-":>6} {"-":>7}  not pushed (lower bound only)')
            continue
        verdict = 'RAIL' if shr > 0.02 else 'soft rolloff'
        curve.append((lo + 1, eff[m].max(), rail, ms.sum()))
        print(f'{lo:>3}-{lo+2:<3} {m.sum():>9,} {ms.sum():>8,} {req[m].max():>7} '
              f'{eff[m].max():>7} {rail:>6} {shr:>6.1%}  {verdict}')

    # does the rail depend on instantaneous speed only?
    print(f'\nrail vs longitudinal acceleration (|a|<{ACC_BAND} = steady). '
          f'Disagreement here means hysteresis, and a plain speed lookup is wrong.')
    print(f'{"mph":>7} {"decel rail":>12} {"n":>8} {"steady rail":>12} {"n":>8} '
          f'{"accel rail":>12} {"n":>8}  spread')
    print('-' * 86)
    for lo in range(2, 80, 2):
        m = (mph >= lo) & (mph < lo + 2) & sat
        if m.sum() < 300:
            continue
        cells = []
        for cond in (aeg < -ACC_BAND, np.abs(aeg) <= ACC_BAND, aeg > ACC_BAND):
            mm = m & cond
            rl, _, _ = rail_of(eff[mm])
            cells.append((rl, mm.sum()))
        vals = [c[0] for c in cells if c[0] is not None]
        spread = (max(vals) - min(vals)) if len(vals) > 1 else 0
        s = ''.join(f'{(c[0] if c[0] is not None else "-"):>12} {c[1]:>8,} ' for c in cells)
        flag = '  <-- HYSTERESIS' if spread > 40 else ''
        print(f'{lo:>3}-{lo+2:<3} {s} {spread:>6}{flag}')

    # direction asymmetry
    print('\nrail vs steering direction (sign of request):')
    print(f'{"mph":>7} {"left rail":>11} {"n":>8} {"right rail":>11} {"n":>8}  delta')
    print('-' * 60)
    for lo in range(2, 80, 2):
        m = (mph >= lo) & (mph < lo + 2) & sat
        if m.sum() < 300:
            continue
        cells = []
        for cond in (R > 0, R < 0):
            mm = m & cond
            rl, _, _ = rail_of(eff[mm])
            cells.append((rl, mm.sum()))
        vals = [c[0] for c in cells if c[0] is not None]
        d = (max(vals) - min(vals)) if len(vals) > 1 else 0
        s = ''.join(f'{(c[0] if c[0] is not None else "-"):>11} {c[1]:>8,} ' for c in cells)
        print(f'{lo:>3}-{lo+2:<3} {s} {d:>6}')

    print('\nproposed STEER_MAX_LOOKUP support (m/s -> observed ceiling):')
    for c, mx, rail, n in curve:
        print(f'  {c/2.237:>5.1f} m/s ({c:>4.0f} mph): rail {rail:>5}  max {mx:>5}  nSat {n:>7,}')


if __name__ == '__main__':
    if os.path.exists(CACHE) and '--recollect' not in sys.argv:
        z = np.load(CACHE)
        d = [z[k] for k in ('v', 'a', 'g', 'r', 'e')]
        print(f'loaded {len(d[0]):,} cached frames from {CACHE}')
    else:
        d = collect()
    analyse(*d)
