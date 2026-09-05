#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

One-pass evidence scan for the three corrections:
  #2: 0x25d (CAM_PEDESTRIAN) signal activation census + 0x21d activations
  #1: button edges vs PEDALS.ACC_ACTIVE engagement/disengagement edges
  #3: 0x764/0x76C UDS request/response traffic in our alpha-long drives
"""
import sys, glob, os
sys.path.insert(0, "/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot")
from openpilot.tools.lib.logreader import LogReader

BASE = "/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot/tools/mazda_long/test_data"

def drive_paths():
  out = []
  for d in sorted(os.listdir(BASE)):
    full = os.path.join(BASE, d)
    if d.startswith("drive_") and d not in ("drive_0825", "drive_0810_ab") and os.path.isdir(full):
      ps = sorted(glob.glob(f"{full}/rlog*") + glob.glob(f"{full}/*/rlog*"))
      if ps: out.append((d, "stock", ps))
  for route in ("000000fa--6b21bd7e7e", "000000fc--7e8c9faa2d", "000000fe--757df8e60f", "000000ff--c4396c44f5"):
    ps = sorted(glob.glob(f"{BASE}/drive_0825/{route}--*/rlog*"), key=lambda p: int(p.split("--")[-1].split("/")[0]))
    if ps: out.append((route[:8], "ours", ps))
  ps = sorted(glob.glob(f"{BASE}/route_4d_cx9/*/rlog.zst"))
  if ps: out.append(("route_4d", "ours", ps))
  ps = sorted(glob.glob(f"{BASE}/drive_0810_ab/*/rlog*"))
  if ps: out.append(("route_ab", "ours", ps))
  return out

ped_vals = {}          # (kind, signal) -> {value: count}
ped_eps = []           # transition episodes (drive, t, signal, old, new, v, brake)
engage_edges = []      # (drive, kind, t, buttons_in_window, brake)
diseng_edges = []      # (drive, kind, t, cause_flags)
uds = []               # (drive, t, addr, src, payload)

BTN_BITS = dict(SET_P=(0, 4), SET_M=(0, 5), RES=(0, 2), CAN_OFF=(0, 0),
                DIST_MORE=(0, 6), DIST_LESS=(0, 7), MODE_X=(1, 6), MODE_Y=(1, 5))

for name, kind, paths in drive_paths():
  acc = None
  btn_hist = []        # (t, frozenset of pressed buttons)
  brake_hist = []      # (t, brake)
  ped_last = {}
  v = 0.
  brake = 0
  t0 = None
  n_eng = 0
  for p in paths:
    try:
      lr = LogReader(p)
    except Exception:
      continue
    for m in lr:
      w = m.which()
      if w == "carState":
        v = m.carState.vEgo
      elif w != "can":
        continue
      else:
        t = m.logMonoTime * 1e-9
        if t0 is None: t0 = t
        for c in m.can:
          d = bytes(c.dat)
          if c.src == 0 and c.address == 0x9d and len(d) == 8:
            pressed = frozenset(k for k, (byte, bit) in BTN_BITS.items() if (d[byte] >> bit) & 1)
            if pressed:
              btn_hist.append((t, pressed))
              if len(btn_hist) > 400: btn_hist.pop(0)
          elif c.src == 0 and c.address == 0x165 and len(d) == 8:
            brake = (d[0] >> 4) & 1
            a = (d[0] >> 3) & 1
            if acc is not None and a != acc:
              if a == 1:
                btns = set()
                for bt, bp in btn_hist:
                  if t - bt <= 1.0: btns |= bp
                engage_edges.append((name, kind, round(t - t0, 1), sorted(btns), brake))
              else:
                recent_cancel = any("CAN_OFF" in bp for bt, bp in btn_hist if t - bt <= 1.0)
                diseng_edges.append((name, kind, round(t - t0, 1),
                                     ("brake" if brake else "") + ("+cancel" if recent_cancel else "")))
            acc = a
          elif c.src == 2 and c.address == 0x25d and len(d) == 8:
            sigs = dict(PED_BRAKE=(d[0] >> 1) & 7, PED_WARN=(d[1] >> 1) & 1,
                        AEB_NOT_ENG=(d[1] >> 5) & 1, BRAKE_WARN=(d[3] >> 1) & 1,
                        b1_7=(d[1] >> 7) & 1, b1_0=d[1] & 1, b4_34=(d[4] >> 3) & 3)
            for k2, val in sigs.items():
              ped_vals.setdefault((kind, k2), {}).setdefault(val, 0)
              ped_vals[(kind, k2)][val] += 1
              if ped_last.get(k2) is not None and ped_last[k2] != val:
                if len(ped_eps) < 400:
                  ped_eps.append((name, round(t - t0, 1), k2, ped_last[k2], val, round(v, 1), brake))
              ped_last[k2] = val
          elif c.address in (0x764, 0x76c):
            if len(uds) < 200:
              uds.append((name, round(t - (t0 or t), 1), hex(c.address), c.src, d.hex()))

print("=== #2: 0x25d signal value census ===")
for (kind, sig), vals in sorted(ped_vals.items()):
  print(f"{kind:5s} {sig:12s}: {dict(sorted(vals.items()))}")
print(f"\n=== #2: 0x25d transition episodes ({len(ped_eps)} shown, cap 400) ===")
for e in ped_eps[:60]:
  print(f"  {e[0]:12s} t+{e[1]:8.1f} {e[2]:12s} {e[3]}->{e[4]} v={e[5]} brk={e[6]}")

print(f"\n=== #1: engagement edges ({len(engage_edges)}) ===")
no_btn = [e for e in engage_edges if not e[3]]
print(f"with button in prior 1.0s: {len(engage_edges) - len(no_btn)}; WITHOUT: {len(no_btn)}")
from collections import Counter
print("button combos:", Counter(tuple(e[3]) for e in engage_edges).most_common(8))
for e in no_btn[:15]:
  print(f"  NO-BUTTON engage: {e}")
print(f"\n=== #1: disengagement edges ({len(diseng_edges)}) ===")
print(Counter(e[3] for e in diseng_edges).most_common(8))
bare = [e for e in diseng_edges if e[3] == ""][:15]
for e in bare:
  print(f"  bare disengage: {e}")

print(f"\n=== #3: UDS traffic ({len(uds)}) ===")
for u in uds[:40]:
  print(f"  {u}")
