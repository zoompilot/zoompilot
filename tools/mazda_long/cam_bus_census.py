#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Census of everything the stock LKAS camera transmits.

On an intercepted Mazda harness bus 0 is the car side and bus 2 the camera side.
In the rlog, src 0/2 are frames *received* on those buses; src 128/130 are the
panda's TX echoes (128 = onto the car bus, 130 = onto the camera bus).

The harness relay starts OPEN, which shorts the two buses together, so early in
every log bus 2 sees the entire car. Once the relay closes (first src==130 echo)
bus 2 carries only the camera. Everything here is gated on that.

  camera-originated  ->  src 2 high, src 130 zero, forwarded out as src 128
  car-originated     ->  src 0 high, src 130 high, only pre-relay leakage on src 2
"""

import glob
import os
import sys
from collections import defaultdict

import zstandard as zstd
from openpilot.cereal import log

DBC_NAMES = {}
OP_TX = {0x243, 0x249}  # openpilot's own Mazda TX


def load_dbc_names(path='opendbc_repo/opendbc/dbc/mazda_2017.dbc'):
    for line in open(path):
        if line.startswith('BO_ '):
            parts = line.split()
            DBC_NAMES[int(parts[1])] = parts[2].rstrip(':')


class AddrStat:
    __slots__ = ('n', 'dlc', 'byte_vals', 'first', 'last')

    def __init__(self):
        self.n = 0
        self.dlc = set()
        self.byte_vals = [set() for _ in range(8)]
        self.first = None
        self.last = None

    def add(self, dat, t):
        self.n += 1
        self.dlc.add(len(dat))
        for i, b in enumerate(dat[:8]):
            s = self.byte_vals[i]
            if len(s) < 300:
                s.add(b)
        if self.first is None:
            self.first = t
        self.last = t


def scan(files, max_segs=None):
    """Two passes over each segment is wasteful; instead buffer per segment and
    gate on the relay time found within it."""
    stats = defaultdict(lambda: defaultdict(AddrStat))
    eng = tot = 0
    dctx = zstd.ZstdDecompressor()
    used = 0
    relay_seen = False
    for fp in files:
        if max_segs and used >= max_segs:
            break
        try:
            with open(fp, 'rb') as f:
                data = dctx.stream_reader(f).read()
        except Exception as e:
            print(f'  skip {os.path.basename(fp)}: {e}', file=sys.stderr)
            continue
        used += 1
        pending = []
        t_relay = None
        try:
            for evt in log.Event.read_multiple_bytes(data):
                w = evt.which()
                if w == 'carState':
                    tot += 1
                    if evt.carState.cruiseState.enabled:
                        eng += 1
                elif w == 'can':
                    t = evt.logMonoTime * 1e-9
                    for m in evt.can:
                        if m.src == 130 and t_relay is None:
                            t_relay = t
                        pending.append((m.src, m.address, bytes(m.dat), t))
        except Exception as e:
            print(f'  partial {os.path.basename(fp)}: {e}', file=sys.stderr)
        if t_relay is not None:
            relay_seen = True
        cutoff = (t_relay + 1.0) if t_relay is not None else -1e18
        for src, addr, dat, t in pending:
            if t >= cutoff:
                stats[src][addr].add(dat, t)
    return stats, used, eng, tot, relay_seen


def report(stats, used, eng, tot, relay_seen, label):
    print(f'\n{"=" * 112}\n{label}   segments={used}   '
          f'cruise engaged {100.0 * eng / max(tot, 1):.0f}% of {tot} carState frames')
    if not relay_seen:
        print('  !! no src=130 echoes anywhere: bus 2 is not an intercepted camera '
              'bus on this route, skipping')
        return set()

    rx0, rx2 = stats.get(0, {}), stats.get(2, {})
    tx0, tx2 = stats.get(128, {}), stats.get(130, {})
    span = max((s.last - s.first) for s in rx2.values() if s.first is not None)
    print(f'  relay-closed span {span:.0f} s   bus0 rx={len(rx0)} bus2 rx={len(rx2)} '
          f'tx->car={len(tx0)} tx->cam={len(tx2)}\n')

    print(f'{"addr":>6} {"name":<18} {"rx bus2":>8} {"Hz":>6} {"rx bus0":>8} '
          f'{"tx->car":>8} {"tx->cam":>8} {"varying bytes":<24} payload template')
    print('-' * 118)
    cam_addrs = set()
    for addr in sorted(rx2):
        st = rx2[addr]
        dur = (st.last - st.first) if st.first is not None else 0
        hz = st.n / dur if dur > 0.5 else float('nan')
        nb = max(st.dlc)
        active = [i for i in range(nb) if len(st.byte_vals[i]) > 1]
        tmpl = ''.join(f'{next(iter(st.byte_vals[i])):02x}' if len(st.byte_vals[i]) == 1 else '..'
                       for i in range(nb))
        n0 = rx0[addr].n if addr in rx0 else 0
        nt0 = tx0[addr].n if addr in tx0 else 0
        nt2 = tx2[addr].n if addr in tx2 else 0
        if nt2 == 0:
            cam_addrs.add(addr)
        print(f'{addr:>#6x} {DBC_NAMES.get(addr, ""):<18} {st.n:>8,} {hz:>6.1f} {n0:>8,} '
              f'{nt0:>8,} {nt2:>8,} {str(active):<24} {tmpl}')

    print(f'\n  CAMERA-ORIGINATED ({len(cam_addrs)}): '
          + ', '.join(f'{a:#x} {DBC_NAMES.get(a, "?")}' for a in sorted(cam_addrs)))
    print(f'  panda forwards to car ({len(tx0)}): '
          + ', '.join(f'{a:#x}{"[OP]" if a in OP_TX else ""}' for a in sorted(tx0)))
    return cam_addrs


if __name__ == '__main__':
    load_dbc_names()
    max_segs = int(os.environ.get('MAX_SEGS', 4))
    sets = {}
    for pat in sys.argv[1:]:
        files = sorted(glob.glob(pat))
        if not files:
            print(f'no files for {pat}', file=sys.stderr)
            continue
        stats, used, eng, tot, relay = scan(files, max_segs=max_segs)
        s = report(stats, used, eng, tot, relay, pat)
        if s:
            sets[pat] = s
    if len(sets) > 1:
        common = set.intersection(*sets.values())
        union = set.union(*sets.values())
        print(f'\n{"=" * 112}\nagreement across {len(sets)} routes')
        print(f'  in every route ({len(common)}): '
              + ', '.join(f'{a:#x} {DBC_NAMES.get(a, "?")}' for a in sorted(common)))
        print(f'  in some but not all ({len(union - common)}): '
              + ', '.join(f'{a:#x} {DBC_NAMES.get(a, "?")}' for a in sorted(union - common)))
