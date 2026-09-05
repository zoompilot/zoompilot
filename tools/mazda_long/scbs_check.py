#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Gate a drive before it is used as SCBS evidence, and say what it actually showed.

The camera's SCBS fault survives short ignition cycles. Routes 00000119, 0000011a and
0000011b all opened with the full trio already set, inherited from 00000118's latch ~90 s
earlier across two key cycles of 23 s and 12 s. A drive that starts faulted can only ever
report "still broken", whatever the code does, and four rounds of validation went that way
without anyone checking. So: t=0 clean or the drive is void.

Usage:  scbs_check.py <route-dir> [<route-dir> ...]
        scbs_check.py tools/mazda_long/test_data/route_118
"""
import glob
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from openpilot.tools.lib.logreader import LogReader  # noqa: E402

# camera fault trio, all on bus 2
BITS = {
  0x21d: ("21d.b1.7", lambda d: (d[1] >> 7) & 1),
  0x25d: ("25d.b1.0", lambda d: d[1] & 1),
  0x440: ("440.b7.5", lambda d: (d[7] >> 5) & 1),
}
# the camera blips these while it boots; ignore anything before the trace settles
SETTLE_T = 8.0
# 0x440 b7.5 also asserts ALONE for seconds at a time mid-drive and clears again (route 53
# t+666 for 16 s, route 11d t+47.8 for 3.1 s) -- a camera state, not the fault. The SCBS latch
# is the whole trio going high together and never clearing, so that is what we test for.
LATCH_BITS = ("21d.b1.7", "25d.b1.0")


def segments(route_dir):
  segs = sorted(glob.glob(os.path.join(route_dir, "rlog_seg*.zst")),
                key=lambda p: int(p.split("seg")[1].split(".")[0]))
  if segs:
    return segs
  return sorted(glob.glob(os.path.join(route_dir, "*", "rlog*")))


def scan(route_dir):
  state, t0, first_set, at_settle, pulses, prev_unl = {}, None, None, None, 0, 0
  for p in segments(route_dir):
    for m in LogReader(p):
      if m.which() != "can":
        continue
      t = m.logMonoTime * 1e-9
      if t0 is None:
        t0 = t
      tr = t - t0
      for c in m.can:
        d = bytes(c.dat)
        if c.src == 2 and c.address in BITS:
          name, get = BITS[c.address]
          v = get(d)
          state[name] = v
          if tr > SETTLE_T and first_set is None and all(state.get(b) for b in LATCH_BITS):
            first_set = (tr, "+".join(LATCH_BITS))
        elif c.src == 128 and c.address == 0x21b:
          unl = (d[6] >> 6) & 1
          pulses += int(unl and not prev_unl)
          prev_unl = unl
      if at_settle is None and tr >= SETTLE_T:
        at_settle = dict(state)
  return at_settle or {}, first_set, pulses


def main(dirs):
  bad = 0
  for route_dir in dirs:
    settle, first_set, pulses = scan(route_dir)
    name = os.path.basename(route_dir.rstrip("/"))
    started_dirty = any(settle.get(b) for b in LATCH_BITS)
    print(f"\n{name}")
    print(f"  fault bits after settle: {settle or 'no camera frames'}")
    print(f"  unlatch pulses emitted:  {pulses}")
    if started_dirty:
      bad += 1
      print("  VOID - camera was already faulted at the start; this drive proves nothing "
            "about the fix. Power the car down properly and drive again.")
    elif first_set:
      bad += 1
      print(f"  LATCHED at t={first_set[0]:.2f} ({first_set[1]}) - a real, attributable fault")
    else:
      print("  CLEAN start to finish"
            + (" - and it never pulsed" if not pulses else " - despite emitting a pulse"))
  return 1 if bad else 0


if __name__ == "__main__":
  if len(sys.argv) < 2:
    print(__doc__)
    raise SystemExit(2)
  raise SystemExit(main(sys.argv[1:]))
