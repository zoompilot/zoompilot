#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Validate the camera's 0x244 CAM_DISTANCE against radar, and dump the
0x21d CAM_EMPTY state payloads.

0x244 looks like four (u8, u8) slots that idle at (0x64, 0xf0). If the second
byte of a slot really is a distance it should track the radar lead range.
"""

import glob
import os
import sys

import numpy as np
import zstandard as zstd
from openpilot.cereal import log

RADAR_TRACKS = list(range(0x361, 0x367))


def scan(files, max_segs, relay_open):
    rows = []          # t, b0..b7 of 0x244
    lead = []          # t, radar lead dRel from radarState
    lkas = []          # t, LKAS_REQUEST, vego
    empty = {}         # payload -> count for 0x21d
    dctx = zstd.ZstdDecompressor()
    used = 0
    for fp in files:
        if used >= max_segs:
            break
        try:
            with open(fp, 'rb') as f:
                data = dctx.stream_reader(f).read()
        except Exception as e:
            print(f'  skip {os.path.basename(fp)}: {e}', file=sys.stderr)
            continue
        used += 1
        pend244, pend21d, pend243 = [], [], []
        t_relay = None
        vego = 0.0
        try:
            for evt in log.Event.read_multiple_bytes(data):
                w = evt.which()
                if w == 'carState':
                    vego = evt.carState.vEgo
                elif w == 'radarState':
                    l0 = evt.radarState.leadOne
                    lead.append((evt.logMonoTime * 1e-9,
                                 l0.dRel if l0.present else np.nan))
                elif w == 'can':
                    t = evt.logMonoTime * 1e-9
                    for m in evt.can:
                        if m.src == 130 and t_relay is None:
                            t_relay = t
                        if m.src != 2:
                            continue
                        if m.address == 0x244:
                            pend244.append((t, bytes(m.dat)))
                        elif m.address == 0x21d:
                            pend21d.append((t, bytes(m.dat)))
                        elif m.address == 0x243:
                            pend243.append((t, bytes(m.dat), vego))
        except Exception as e:
            print(f'  partial {os.path.basename(fp)}: {e}', file=sys.stderr)
        if t_relay is not None:
            cut = t_relay + 1.0
        elif relay_open:
            cut = -1e18
        else:
            continue
        for t, d in pend244:
            if t >= cut:
                rows.append((t,) + tuple(d))
        for t, d in pend21d:
            if t >= cut:
                empty[d] = empty.get(d, 0) + 1
        for t, d, v in pend243:
            if t >= cut:
                raw = int.from_bytes(d, 'big')
                # LKAS_REQUEST is 3|12@0+ big-endian: msb is bit 4, so shift 64-4-12
                req = ((raw >> 48) & 0xFFF) - 2048
                lkas.append((t, req, v))
    return np.array(rows), np.array(lead), np.array(lkas), empty, used


def main():
    relay_open = bool(os.environ.get('RELAY_OPEN'))
    max_segs = int(os.environ.get('MAX_SEGS', 6))
    files = []
    for pat in sys.argv[1:]:
        files.extend(sorted(glob.glob(pat)))
    R, L, K, empty, used = scan(files, max_segs, relay_open)
    print(f'{used} segments; 0x244 frames={len(R)}  radarState={len(L)}  0x243={len(K)}')

    print('\n--- 0x21d CAM_EMPTY distinct payloads ---')
    for pay, cnt in sorted(empty.items(), key=lambda kv: -kv[1]):
        print(f'  {pay.hex(" ")}  x{cnt:,}')

    print('\n--- 0x244 CAM_DISTANCE slot structure ---')
    t = R[:, 0]
    B = R[:, 1:].astype(int)
    for s in range(4):
        a, b = B[:, 2 * s], B[:, 2 * s + 1]
        idle = (a == 0x64) & (b == 0xF0)
        print(f'  slot{s} (byte{2 * s},byte{2 * s + 1}): idle (100,240) '
              f'{100.0 * idle.mean():5.1f}%   '
              f'active byteA {a[~idle].min() if (~idle).any() else "-"}..'
              f'{a[~idle].max() if (~idle).any() else "-"}  '
              f'byteB {b[~idle].min() if (~idle).any() else "-"}..'
              f'{b[~idle].max() if (~idle).any() else "-"}')

    if len(L) and len(R):
        Lt, Ld = L[:, 0], L[:, 1]
        print('\n--- slot byteB vs radar leadOne.dRel ---')
        for s in range(4):
            b = B[:, 2 * s + 1].astype(float)
            act = b != 0xF0
            if act.sum() < 50:
                print(f'  slot{s}: only {act.sum()} active frames, skipping')
                continue
            idx = np.searchsorted(Lt, t[act])
            idx = np.clip(idx, 0, len(Ld) - 1)
            dr = Ld[idx]
            ok = np.isfinite(dr)
            if ok.sum() < 50:
                print(f'  slot{s}: no radar lead overlap ({ok.sum()})')
                continue
            x, y = b[act][ok], dr[ok]
            cc = float(np.corrcoef(x, y)[0, 1])
            fit = np.polyfit(x, y, 1)
            resid = y - np.polyval(fit, x)
            print(f'  slot{s}: n={ok.sum()} corr={cc:+.3f}  '
                  f'dRel ~ {fit[0]:.3f}*raw + {fit[1]:.2f} m  '
                  f'resid sd={resid.std():.2f} m  raw {x.min():.0f}..{x.max():.0f}')
        # radar-lead-present vs slot activity agreement
        idx = np.clip(np.searchsorted(Lt, t), 0, len(Ld) - 1)
        radar_lead = np.isfinite(Ld[idx])
        for s in range(4):
            act = B[:, 2 * s + 1] != 0xF0
            agree = (act == radar_lead).mean()
            print(f'  slot{s} active vs radar-lead-present agreement: {100 * agree:.1f}%  '
                  f'(cam {100 * act.mean():.1f}% / radar {100 * radar_lead.mean():.1f}%)')

    if len(K):
        req = K[:, 1]
        v = K[:, 2]
        nz = req != 0
        print(f'\n--- 0x243 CAM_LKAS LKAS_REQUEST (stock camera) ---')
        print(f'  frames={len(req)}  nonzero={100 * nz.mean():.1f}%  '
              f'range {req.min():.0f}..{req.max():.0f}  '
              f'|max|={np.abs(req).max():.0f}  p99|.|={np.percentile(np.abs(req[nz]), 99):.0f}')
        d = np.abs(np.diff(req))
        print(f'  step |delta| max={d.max():.0f} p99={np.percentile(d, 99):.0f} '
              f'median={np.median(d):.0f}  (at {np.median(np.diff(K[:, 0])) * 1e3:.0f} ms spacing)')
        for lo, hi in ((0, 5), (5, 10), (10, 15), (15, 20), (20, 25), (25, 31)):
            msk = (v >= lo) & (v < hi) & nz
            if msk.sum() > 30:
                print(f'  {lo:>2}-{hi:>2} m/s: n={msk.sum():>6} '
                      f'|req| p50={np.percentile(np.abs(req[msk]), 50):>5.0f} '
                      f'p99={np.percentile(np.abs(req[msk]), 99):>5.0f} '
                      f'max={np.abs(req[msk]).max():>5.0f}')


if __name__ == '__main__':
    main()
