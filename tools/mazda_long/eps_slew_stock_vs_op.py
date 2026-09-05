#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Is the EPS steer-rate limit per-FRAME or per-unit-TIME?

docs/eps-rate-summary.md concluded "12 units per frame" from data captured entirely
at openpilot's 100 Hz CAM_LKAS cadence, where per-frame and per-time are
indistinguishable. The stock camera commands at 16.6 Hz, so a stock drive separates
them:

  per-frame limit -> stock EPS moves <=12 units per 60 ms  =  200 units/s
  per-time limit  -> stock EPS moves up to 72 units per 60 ms = 1200 units/s

LKAS_EFFECTIVE (0x241 STEER_RATE, bus 0, ~83 Hz) is the EPS's own applied torque, so
its slope is the EPS behaviour regardless of who is commanding.
"""
import glob, os, sys
import numpy as np, zstandard as zstd
from openpilot.cereal import log

def bits(dat, start, size, signed_off):
    raw = int.from_bytes(dat, 'big')
    byte, bit = start // 8, start % 8
    msb = byte * 8 + (7 - bit)
    return ((raw >> (64 - msb - size)) & ((1 << size) - 1)) - signed_off

def scan(files, max_segs, relay_open):
    rows = []   # t, eff, req_eps, block, vego
    cam = []    # t, camera LKAS_REQUEST on bus 2
    op  = []    # t, openpilot LKAS_REQUEST TX on bus 0
    dctx = zstd.ZstdDecompressor(); used = 0
    for fp in files:
        if used >= max_segs: break
        try: data = dctx.stream_reader(open(fp, 'rb')).read()
        except Exception: continue
        used += 1
        pend, pcam, pop = [], [], []
        t_relay = None; vego = 0.0; lat = False
        try:
            for evt in log.Event.read_multiple_bytes(data):
                w = evt.which()
                if w == 'carState':
                    vego = evt.carState.vEgo
                elif w == 'carControl':
                    lat = evt.carControl.latActive
                elif w == 'can':
                    t = evt.logMonoTime * 1e-9
                    for m in evt.can:
                        if m.src == 130 and t_relay is None: t_relay = t
                        d = bytes(m.dat)
                        if m.src == 0 and m.address == 0x241:
                            pend.append((t, bits(d, 39, 12, 2048), bits(d, 3, 12, 2048),
                                         (d[6] >> 2) & 1, vego, lat))
                        elif m.src == 2 and m.address == 0x243:
                            pcam.append((t, bits(d, 3, 12, 2048)))
                        elif m.src == 128 and m.address == 0x243:
                            pop.append((t, bits(d, 3, 12, 2048)))
        except Exception as e:
            print('  partial', os.path.basename(fp), e, file=sys.stderr)
        cut = (t_relay + 1.0) if t_relay is not None else (-1e18 if relay_open else None)
        if cut is None: continue
        rows += [r for r in pend if r[0] >= cut]
        cam  += [r for r in pcam if r[0] >= cut]
        op   += [r for r in pop  if r[0] >= cut]
    return np.array(rows), np.array(cam), np.array(op), used

def slew(R, label, moving_only=True):
    if not len(R):
        print(f'{label}: no data'); return
    t, eff, block, vego = R[:, 0], R[:, 1], R[:, 3], R[:, 4]
    ok = (block == 0) & (vego > 2.0)
    t, eff = t[ok], eff[ok]
    dt = np.diff(t); de = np.abs(np.diff(eff))
    good = (dt > 0.005) & (dt < 0.05)
    dt, de = dt[good], de[good]
    if moving_only:
        m = de > 0
        dtm, dem = dt[m], de[m]
    else:
        dtm, dem = dt, de
    rate = dem / dtm
    print(f'\n{label}   n={len(dt):,} samples, {100*np.mean(de>0):.1f}% moving, '
          f'dt median {np.median(dt)*1e3:.1f} ms')
    print(f'  |delta| per sample: max={de.max():.0f} p99={np.percentile(de,99):.0f} '
          f'p95={np.percentile(de,95):.0f} distinct={sorted(set(de.tolist()))[:8]}')
    print(f'  slew units/s (moving samples): p50={np.percentile(rate,50):.0f} '
          f'p95={np.percentile(rate,95):.0f} p99={np.percentile(rate,99):.0f} '
          f'max={rate.max():.0f}')
    # windowed: how far can EFF move in one stock command period (60 ms)?
    for win in (0.010, 0.060):
        idx = np.searchsorted(t, t + win)
        idx = np.clip(idx, 0, len(eff) - 1)
        d = np.abs(eff[idx] - eff)
        valid = (t[idx] - t) > win * 0.5
        if valid.sum() > 100:
            print(f'  max |Δeff| over {win*1e3:.0f} ms window: '
                  f'p99={np.percentile(d[valid],99):.0f} max={d[valid].max():.0f} '
                  f'-> {np.percentile(d[valid],99)/win:.0f} units/s at p99')

def gap(R, label):
    """|LKAS_REQUEST - LKAS_EFFECTIVE| as seen inside 0x241 itself: how far the EPS
    lags whoever is commanding it."""
    if not len(R): return
    t, eff, req, block, vego = R[:,0], R[:,1], R[:,2], R[:,3], R[:,4]
    ok = (block == 0) & (vego > 2.0) & (np.abs(req) > 20)
    g = np.abs(req[ok] - eff[ok])
    print(f'\n{label}: n={ok.sum():,} frames with |req|>20')
    print(f'  |req-eff|: p50={np.percentile(g,50):.0f} p90={np.percentile(g,90):.0f} '
          f'p99={np.percentile(g,99):.0f} max={g.max():.0f} mean={g.mean():.1f}')
    print(f'  frames lagging >50 units: {100*np.mean(g>50):.1f}%   '
          f'>100 units: {100*np.mean(g>100):.1f}%')

if __name__ == '__main__':
    relay_open = bool(os.environ.get('RELAY_OPEN'))
    max_segs = int(os.environ.get('MAX_SEGS', 8))
    files = []
    for p in sys.argv[1:]: files.extend(sorted(glob.glob(p)))
    R, C, O, used = scan(files, max_segs, relay_open)
    print(f'{used} segs  0x241={len(R)}  cam 0x243={len(C)}  op 0x243 TX={len(O)}')
    for arr, nm in ((C, 'camera cmd (bus2)'), (O, 'openpilot cmd TX (bus0)')):
        if len(arr) > 10:
            dt = np.diff(arr[:, 0]); dt = dt[dt > 0]
            d = np.abs(np.diff(arr[:, 1]))
            print(f'  {nm}: {len(arr):,} frames, dt median {np.median(dt)*1e3:.1f} ms '
                  f'({1/np.median(dt):.1f} Hz), |Δcmd| p99={np.percentile(d,99):.0f} '
                  f'max={d.max():.0f}')
    slew(R, 'EPS LKAS_EFFECTIVE')
    gap(R, 'EPS tracking lag')
