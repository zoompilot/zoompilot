#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

What the stock Mazda LKAS camera actually puts on the wire.

For each camera-originated address (see cam_bus_census.py) this characterises the
payload from real drives: true frame rate, which bits ever move, counter and
checksum fields, the DBC signals' observed ranges, and -- the interesting part --
which active bits no DBC signal covers.

Bus 2 is gated on the harness relay being closed, and frames are tagged with
whether openpilot was steering at the time so stock behaviour can be isolated.
"""

import glob
import os
import sys
from collections import Counter, defaultdict

import numpy as np
import zstandard as zstd
from openpilot.cereal import log

CAM_ADDRS = [0x21d, 0x242, 0x243, 0x244, 0x245, 0x246, 0x25d, 0x35f, 0x440, 0x485, 0x488]

DBC_PATH = 'opendbc_repo/opendbc/dbc/mazda_2017.dbc'


def parse_dbc():
    """address -> {name, signals: [(name, start_bit, size, is_le, signed, factor, offset)]}"""
    msgs = {}
    cur = None
    for line in open(DBC_PATH):
        if line.startswith('BO_ '):
            p = line.split()
            cur = int(p[1])
            msgs[cur] = {'name': p[2].rstrip(':'), 'signals': []}
        elif line.strip().startswith('SG_ ') and cur is not None:
            body = line.strip()[4:]
            name, rest = body.split(':', 1)
            rest = rest.strip()
            layout, rest2 = rest.split('@', 1)
            start, size = layout.split('|')
            order_sign = rest2[:2]
            is_le = order_sign[0] == '1'
            signed = order_sign[1] == '-'
            factor_off = rest2[rest2.index('(') + 1:rest2.index(')')]
            factor, offset = factor_off.split(',')
            msgs[cur]['signals'].append(
                (name.strip(), int(start), int(size), is_le, signed, float(factor), float(offset)))
        elif not line.strip():
            cur = cur if line.startswith(' ') else cur
    return msgs


def bit_index_be(start_bit, i):
    """i-th bit of a big-endian (Motorola) signal that starts at start_bit,
    expressed as an absolute bit number in the same numbering the DBC uses."""
    byte = start_bit // 8
    bit = start_bit % 8
    pos = byte * 8 + (7 - bit) + i
    return pos


def signal_bits(start, size, is_le):
    """Absolute 'MSB-first within byte' bit positions covered by a signal."""
    out = set()
    if is_le:
        for i in range(size):
            b = start + i
            out.add((b // 8) * 8 + (7 - (b % 8)))
    else:
        for i in range(size):
            out.add(bit_index_be(start, i))
    return out


def extract(dat, start, size, is_le, signed, factor, offset):
    val = 0
    if is_le:
        raw = int.from_bytes(dat, 'little')
        val = (raw >> start) & ((1 << size) - 1)
    else:
        raw = int.from_bytes(dat, 'big')
        byte = start // 8
        bit = start % 8
        msb = byte * 8 + (7 - bit)
        shift = 64 - msb - size
        val = (raw >> shift) & ((1 << size) - 1)
    if signed and val >= (1 << (size - 1)):
        val -= (1 << size)
    return val * factor + offset


class MsgAcc:
    def __init__(self):
        self.payloads = []
        self.times = []
        self.ctx = []  # (vego, angle, op_engaged, stock_lkas)

    def add(self, dat, t, ctx):
        self.payloads.append(dat)
        self.times.append(t)
        self.ctx.append(ctx)


def scan(files, max_segs):
    acc = defaultdict(MsgAcc)
    rates = defaultdict(list)
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
        pending = []
        t_relay = None
        vego = 0.0
        angle = 0.0
        op_eng = False
        seg_t = defaultdict(list)
        try:
            for evt in log.Event.read_multiple_bytes(data):
                w = evt.which()
                if w == 'carState':
                    vego = evt.carState.vEgo
                    angle = evt.carState.steeringAngleDeg
                elif w == 'selfdriveState':
                    op_eng = evt.selfdriveState.enabled
                elif w == 'can':
                    t = evt.logMonoTime * 1e-9
                    for m in evt.can:
                        if m.src == 130 and t_relay is None:
                            t_relay = t
                        if m.src == 2 and m.address in CAM_ADDRS:
                            pending.append((m.address, bytes(m.dat), t, (vego, angle, op_eng)))
        except Exception as e:
            print(f'  partial {os.path.basename(fp)}: {e}', file=sys.stderr)
        if t_relay is not None:
            cutoff = t_relay + 1.0
        elif os.environ.get('RELAY_OPEN'):
            # Route logged with the harness relay never closing: bus 2 is wired
            # straight through to bus 0, so every frame is a genuine bus frame and
            # the camera addresses on src 2 are still the camera's own output.
            cutoff = -1e18
        else:
            print(f'  {os.path.basename(fp)}: relay never closed, skipping '
                  f'(set RELAY_OPEN=1 to accept)', file=sys.stderr)
            continue
        for addr, dat, t, ctx in pending:
            if t >= cutoff:
                acc[addr].add(dat, t, ctx)
                seg_t[addr].append(t)
        for addr, tl in seg_t.items():
            if len(tl) > 20:
                rates[addr].append(len(tl) / (tl[-1] - tl[0]))
    return acc, rates, used


def analyse(addr, m, rates, dbc):
    P = np.frombuffer(b''.join(m.payloads), dtype=np.uint8).reshape(-1, 8)
    n = len(P)
    ctx = np.array(m.ctx, dtype=object)
    vego = np.array([c[0] for c in m.ctx])
    op_eng = np.array([c[2] for c in m.ctx])
    info = dbc.get(addr, {'name': f'{addr:#x}', 'signals': []})

    hz = np.median(rates.get(addr, [float('nan')]))
    print(f'\n{"=" * 100}')
    print(f'{addr:#5x}  {info["name"]:<20} {n:,} frames   {hz:.1f} Hz   '
          f'openpilot steering during {100.0 * op_eng.mean():.0f}%')
    print('=' * 100)

    # bit activity, MSB-first-within-byte numbering to match the DBC
    bits = np.unpackbits(P, axis=1)  # column 8*b + (7 - bitpos) == DBC-ish msb-first
    changed = []
    for i in range(64):
        col = bits[:, i]
        if col.min() != col.max():
            trans = int(np.sum(col[1:] != col[:-1]))
            changed.append((i, float(col.mean()), trans))
    static_bits = [i for i in range(64) if i not in {c[0] for c in changed}]

    tmpl = ' '.join(f'{P[0][b]:02x}' if P[:, b].min() == P[:, b].max() else '..'
                    for b in range(8))
    uniq_payloads = Counter(bytes(p) for p in P)
    print(f'  payload template: {tmpl}      {len(uniq_payloads)} distinct payloads')
    if len(uniq_payloads) <= 12:
        for pay, cnt in uniq_payloads.most_common():
            print(f'    {pay.hex(" ")}  x{cnt:,} ({100.0 * cnt / n:.1f}%)')
    print(f'  bits that never move: {len(static_bits)}/64')

    # counter / checksum detection
    for b in range(8):
        col = P[:, b].astype(np.int64)
        d = np.diff(col)
        for width, mask, shift in ((4, 0x0F, 0), (4, 0xF0, 4), (8, 0xFF, 0)):
            v = (col & mask) >> shift
            dv = (np.diff(v) % (1 << width))
            if len(dv) and np.mean(dv == 1) > 0.9:
                print(f'  byte{b} bits{shift}..{shift + width - 1}: '
                      f'COUNTER +1 mod {1 << width} ({100 * np.mean(dv == 1):.1f}% of steps)')
        # checksum heuristic: near-uniform byte with high entropy and no autocorrelation
        h = Counter(col.tolist())
        if len(h) > 200 and n > 500:
            print(f'  byte{b}: {len(h)} distinct values, flat distribution -> checksum-like')

    # DBC signals
    if info['signals']:
        print(f'\n  {"DBC signal":<26} {"bits":>16} {"min":>10} {"max":>10} {"distinct":>9}  '
              f'{"corr(vEgo)":>10}  top values')
        covered = set()
        for (sname, start, size, is_le, signed, factor, offset) in info['signals']:
            covered |= signal_bits(start, size, is_le)
            vals = np.array([extract(bytes(p), start, size, is_le, signed, factor, offset)
                             for p in P])
            uq = Counter(vals.tolist())
            top = ', '.join(f'{k:g}x{v}' for k, v in uq.most_common(4))
            if vals.std() > 0 and vego.std() > 0:
                cc = float(np.corrcoef(vals, vego)[0, 1])
            else:
                cc = float('nan')
            print(f'  {sname:<26} {f"{start}|{size}@{int(is_le)}{"-" if signed else "+"}":>16} '
                  f'{vals.min():>10.4g} {vals.max():>10.4g} {len(uq):>9} {cc:>10.2f}  {top[:46]}')
        active = {c[0] for c in changed}
        uncovered = sorted(active - covered)
        if uncovered:
            print(f'\n  ACTIVE BUT UNMAPPED bits ({len(uncovered)}): ')
            for i in uncovered:
                mean, trans = next((c[1], c[2]) for c in changed if c[0] == i)
                print(f'    bit {i:>2} (byte{i // 8} bit{7 - i % 8}): high {100 * mean:5.1f}% '
                      f'of frames, {trans} transitions')
        dead = sorted(covered - active)
        if dead:
            print(f'  mapped but constant bits: {len(dead)}')
    else:
        print(f'\n  no DBC signals; active bits: {[c[0] for c in changed]}')


if __name__ == '__main__':
    dbc = parse_dbc()
    max_segs = int(os.environ.get('MAX_SEGS', 6))
    files = []
    for pat in sys.argv[1:]:
        files.extend(sorted(glob.glob(pat)))
    acc, rates, used = scan(files, max_segs)
    print(f'read {used} segments, {sum(len(m.payloads) for m in acc.values()):,} camera frames')
    for addr in CAM_ADDRS:
        if addr in acc:
            analyse(addr, acc[addr], rates, dbc)
