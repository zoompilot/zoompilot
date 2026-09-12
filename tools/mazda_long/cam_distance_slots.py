#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Is 0x244 CAM_DISTANCE an object list?

The payload is four (byteA, byteB) slots idling at (100, 240). Test whether an
active slot's byteA behaves like a lateral offset (centred on 100) and byteB like
a longitudinal range, by comparing against the radar lead when both are present.
"""

import glob
import os
import sys

import numpy as np
import zstandard as zstd
from openpilot.cereal import log


def scan(files, max_segs, relay_open):
    d244, lead, laneinfo = [], [], []
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
        pend, pend440 = [], []
        t_relay = None
        try:
            for evt in log.Event.read_multiple_bytes(data):
                w = evt.which()
                if w == 'radarState':
                    l0 = evt.radarState.leadOne
                    lead.append((evt.logMonoTime * 1e-9,
                                 l0.dRel if l0.present else np.nan,
                                 l0.yRel if l0.present else np.nan))
                elif w == 'can':
                    t = evt.logMonoTime * 1e-9
                    for m in evt.can:
                        if m.src == 130 and t_relay is None:
                            t_relay = t
                        if m.src == 2 and m.address == 0x244:
                            pend.append((t, bytes(m.dat)))
                        elif m.src == 2 and m.address == 0x440:
                            pend440.append((t, bytes(m.dat)))
        except Exception as e:
            print(f'  partial {os.path.basename(fp)}: {e}', file=sys.stderr)
        cut = (t_relay + 1.0) if t_relay is not None else (-1e18 if relay_open else None)
        if cut is None:
            continue
        for t, d in pend:
            if t >= cut:
                d244.append((t,) + tuple(d))
        for t, d in pend440:
            if t >= cut:
                # LANE_LINES is 10|3@0+ big-endian -> msb bit 8*1+(7-2)=13, shift 64-13-3=48
                laneinfo.append((t, (int.from_bytes(d, 'big') >> 48) & 0x7))
    return np.array(d244), np.array(lead), np.array(laneinfo)


def main():
    relay_open = bool(os.environ.get('RELAY_OPEN'))
    max_segs = int(os.environ.get('MAX_SEGS', 10))
    files = []
    for pat in sys.argv[1:]:
        files.extend(sorted(glob.glob(pat)))
    R, L, LI = scan(files, max_segs, relay_open)
    print(f'0x244 frames={len(R)}  radarState={len(L)}  0x440={len(LI)}')
    t = R[:, 0]
    B = R[:, 1:].astype(int)
    Lt, Ld, Ly = L[:, 0], L[:, 1], L[:, 2]
    idx = np.clip(np.searchsorted(Lt, t), 0, len(Ld) - 1)
    dRel, yRel = Ld[idx], Ly[idx]

    for s in range(4):
        a, b = B[:, 2 * s].astype(float), B[:, 2 * s + 1].astype(float)
        act = ~((a == 100) & (b == 240))
        both = act & np.isfinite(dRel)
        print(f'\nslot{s}: active {100 * act.mean():.1f}%  '
              f'active & radar lead {both.sum()} frames')
        if both.sum() < 100:
            continue
        print(f'  byteA: corr(dRel)={np.corrcoef(a[both], dRel[both])[0, 1]:+.3f}  '
              f'corr(yRel)={np.corrcoef(a[both], yRel[both])[0, 1]:+.3f}  '
              f'range {a[both].min():.0f}..{a[both].max():.0f} median {np.median(a[both]):.0f}')
        print(f'  byteB: corr(dRel)={np.corrcoef(b[both], dRel[both])[0, 1]:+.3f}  '
              f'corr(yRel)={np.corrcoef(b[both], yRel[both])[0, 1]:+.3f}  '
              f'range {b[both].min():.0f}..{b[both].max():.0f} median {np.median(b[both]):.0f}')
        print('  byteB bin -> radar dRel:')
        for lo in range(0, 160, 20):
            msk = both & (b >= lo) & (b < lo + 20)
            if msk.sum() > 20:
                print(f'    raw {lo:>3}-{lo + 19:>3}: n={msk.sum():>5} '
                      f'dRel mean={dRel[msk].mean():>6.1f} sd={dRel[msk].std():>5.1f} '
                      f'p10={np.percentile(dRel[msk], 10):>5.1f} '
                      f'p90={np.percentile(dRel[msk], 90):>5.1f}')

    # does slot activity track the camera's own lane-line count instead?
    if len(LI):
        li_idx = np.clip(np.searchsorted(LI[:, 0], t), 0, len(LI) - 1)
        lanes = LI[li_idx, 1]
        nact = sum(~((B[:, 2 * s] == 100) & (B[:, 2 * s + 1] == 240)) for s in range(4))
        print('\nactive slot count vs CAM_LANEINFO LANE_LINES:')
        for lv in sorted(set(lanes.tolist())):
            msk = lanes == lv
            if msk.sum() > 50:
                print(f'  LANE_LINES={int(lv)}: n={msk.sum():>6} '
                      f'mean active slots={nact[msk].mean():.2f}')
        print(f'  corr(active slots, LANE_LINES) = '
              f'{np.corrcoef(nact, lanes)[0, 1]:+.3f}')


if __name__ == '__main__':
    main()
