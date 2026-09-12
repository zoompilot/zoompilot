#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Timeline of a single alpha-long gas override: command, engaged bits, and what the car did.

Usage: .venv/bin/python3 tools/mazda_long/plot_gas_override.py <route> [t_center] [out.png]
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from openpilot.tools.lib.logreader import LogReader

CRZ_INFO, CRZ_CTRL, PEDALS, ENGINE_DATA = 0x21B, 0x21C, 0x165, 0x202


def accel_cmd(dat):
  return ((((dat[2] & 0x3) << 11) | (dat[3] << 3) | (dat[4] >> 5)) - 4096) * 0.001


def load(route):
  rows, can_rows = [], []
  cur = dict(v=np.nan, a=np.nan, gas=0, enabled=0, longActive=0, accel=np.nan, aTarget=np.nan)
  t0 = None
  for m in LogReader(route):
    w = m.which()
    t = m.logMonoTime * 1e-9
    if t0 is None:
      t0 = t
    if w == "carState":
      cs = m.carState
      cur.update(v=cs.vEgo, a=cs.aEgo, gas=int(cs.gasPressed), enabled=int(cs.cruiseState.enabled))
      rows.append((t - t0, cur["v"], cur["a"], cur["gas"], cur["enabled"],
                   cur["longActive"], cur["accel"], cur["aTarget"]))
    elif w == "carControl":
      cur.update(longActive=int(m.carControl.longActive), accel=m.carControl.actuators.accel)
    elif w == "longitudinalPlan":
      cur.update(aTarget=m.longitudinalPlan.aTarget)
    elif w in ("can", "sendcan"):
      for c in (m.can if w == "can" else m.sendcan):
        if len(c.dat) != 8:
          continue
        d = c.dat
        if w == "sendcan" and c.address == CRZ_INFO and c.src == 0:
          can_rows.append((t - t0, "cmd", accel_cmd(d), (d[4] >> 1) & 1))
        elif w == "sendcan" and c.address == CRZ_CTRL and c.src == 0:
          can_rows.append((t - t0, "crz", (d[0] >> 3) & 1, 0))
        elif w == "can" and c.src == 0 and c.address == ENGINE_DATA:
          can_rows.append((t - t0, "eng", (d[4] << 4) | (d[5] >> 4), ((d[0] << 8) | d[1]) * 0.25))
        elif w == "can" and c.src == 0 and c.address == PEDALS:
          can_rows.append((t - t0, "ped", (d[0] >> 3) & 1, 0))
  return np.array(rows, dtype=float), can_rows


def series(can_rows, kind):
  sel = [(r[0], r[2], r[3]) for r in can_rows if r[1] == kind]
  if not sel:
    return np.zeros(0), np.zeros(0), np.zeros(0)
  a = np.array(sel, dtype=float)
  return a[:, 0], a[:, 1], a[:, 2]


def main():
  route = sys.argv[1]
  center = float(sys.argv[2]) if len(sys.argv) > 2 else None
  out = sys.argv[3] if len(sys.argv) > 3 else "docs/mazda-gas-override-timeline.png"

  rows, can_rows = load(route)
  t, v, a, gas, enabled, longActive, accel, aTarget = rows.T

  if center is None:
    # the press with the biggest accel swing while openpilot was braking
    pressed = gas > 0
    rise = np.flatnonzero(pressed[1:] & ~pressed[:-1]) + 1
    best, best_swing = None, 0
    for i in rise:
      pre = slice(max(0, i - 50), i)
      if pre.stop - pre.start < 25 or not longActive[pre].all():
        continue
      j = i
      while j < len(t) - 1 and pressed[j]:
        j += 1
      swing = np.nanmax(a[i:j + 1]) - np.nanmean(a[pre]) if j > i else 0
      if accel[pre].mean() < -0.3 and swing > best_swing:
        best, best_swing = t[i], swing
    center = best
    print(f"worst press at t={center:.1f}s (accel swing {best_swing:+.2f} m/s2)")

  w = (center - 4, center + 10)
  m = (t >= w[0]) & (t <= w[1])

  ct, cv, cacc = series(can_rows, "cmd")
  crt, crv, _ = series(can_rows, "crz")
  et, ev, erpm = series(can_rows, "eng")
  pt, pv, _ = series(can_rows, "ped")

  fig, ax = plt.subplots(5, 1, figsize=(11, 11), sharex=True)

  ax[0].plot(t[m], v[m] * 3.6, "k", lw=1.6)
  ax[0].set_ylabel("speed (kph)")

  ax[1].plot(t[m], a[m], "k", lw=1.6, label="measured (aEgo)")
  ax[1].plot(t[m], accel[m], "C3", lw=1.4, label="openpilot command")
  cm = (ct >= w[0]) & (ct <= w[1])
  ax[1].plot(ct[cm], cv[cm], "C0", lw=1.0, alpha=0.8, label="ACCEL_CMD on the wire")
  ax[1].axhline(0, color="0.7", lw=0.8)
  ax[1].set_ylabel("accel (m/s$^2$)")
  ax[1].legend(fontsize=8, loc="upper left")

  em = (et >= w[0]) & (et <= w[1])
  ax[2].plot(et[em], ev[em], "C2", lw=1.4)
  ax[2].set_ylabel("PEDAL_GAS (raw)")

  ax[3].plot(et[em], erpm[em], "C4", lw=1.4)
  ax[3].set_ylabel("engine (rpm)")

  ax[4].plot(ct[cm], cacc[cm], lw=2.0, label="CRZ_INFO.ACC_ACTIVE (we send)")
  crm = (crt >= w[0]) & (crt <= w[1])
  ax[4].plot(crt[crm], crv[crm] - 0.05, lw=2.0, label="CRZ_CTRL.CRZ_ACTIVE (we send)")
  pm = (pt >= w[0]) & (pt <= w[1])
  ax[4].plot(pt[pm], pv[pm] + 0.05, lw=2.0, label="PEDALS.ACC_ACTIVE (car says)")
  ax[4].plot(t[m], longActive[m] + 0.1, "k--", lw=1.2, label="CC.longActive")
  ax[4].set_ylim(-0.3, 1.4)
  ax[4].set_ylabel("engaged bits")
  ax[4].legend(fontsize=8, loc="center left")
  ax[4].set_xlabel("time (s)")

  for x in ax:
    x.grid(alpha=0.25)
    x.axvspan(center, center + 0.02, color="C2", alpha=0.5)

  fig.suptitle("Alpha long: driver taps the gas mid-decel", y=0.995)
  fig.tight_layout()
  fig.savefig(out, dpi=110)
  print("wrote", out)


if __name__ == "__main__":
  main()
