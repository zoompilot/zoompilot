#!/usr/bin/env python3
"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Per body standstill-hold episode (EPB.HOLD_STATE == HOLDING): when the stock radar (bus 0)
dropped STOPPING and relaxed to raw -1, whether GEAR.BRAKE_HOLD ever followed, and whether
RESUME_UNLATCHING preceded the exit. Stock rows only: under alpha long our frames are not on
bus 0, so OPLONG rows show the body side alone.

2026-09-30: stock relaxes 0-40 ms after HOLDING in 43/43 holds, 15 of them with no BRAKE_HOLD
at all, and pulses before every exit that is not a gas drive-off. So HOLD_STATE is the hold
handshake and BRAKE_HOLD only joins it with Auto Hold armed.

HOLD_STATE (0x79 byte 2, low nibble): 1 boot, 2 idle, 3 holding, 5 releasing. The high nibble
reads 0x3 on hold-capable bodies (CX-5 2017+, CX-9) and 0x0 on the 2016.5 CX-5 KE.

Usage:  acc_hold_census.py <rlog> [<rlog> ...]
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "opendbc_repo"))

from opendbc.can.dbc import DBC
from opendbc.can.parser import get_raw_value
from opendbc.car.mazda.carstate import HOLD_STATE_HOLDING
from opendbc.car.mazda.values import CarControllerParams
from openpilot.tools.lib.logreader import LogReader

CRZ_INFO, EPB, GEAR = 0x21b, 0x79, 0x228
MSGS = DBC("mazda_2017").addr_to_msg
RELAXED = CarControllerParams.ACCEL_HOLD_LATCHED


def signal(addr, dat, name):
  """One DBC signal from a raw frame, in order within the log (a CANParser batch would lose it)."""
  sig = MSGS[addr].sigs[name]
  raw = get_raw_value(dat, sig)
  if sig.is_signed:
    raw -= ((raw >> (sig.size - 1)) & 1) << sig.size
  return raw * sig.factor + sig.offset


def episodes(path):
  """Yield one dict per HOLDING episode that ends inside the log."""
  t0 = None
  brake_hold = False
  stop = cmd = None
  gas = False
  sent = False
  ep = None
  for m in LogReader(path):
    t = m.logMonoTime * 1e-9
    t0 = t if t0 is None else t0
    w = m.which()
    if w == "sendcan":
      if not sent:
        sent = any(c.address == CRZ_INFO for c in m.sendcan)
    elif w == "carState":
      gas = m.carState.gasPressed
    elif w == "can":
      for c in m.can:
        if c.src != 0 or c.address not in (CRZ_INFO, EPB, GEAR):
          continue
        dat = bytes(c.dat)
        if c.address == CRZ_INFO:
          stop, cmd = signal(CRZ_INFO, dat, "STOPPING"), signal(CRZ_INFO, dat, "ACCEL_CMD")
          if ep is not None:
            if ep["relax"] is None and not stop and abs(cmd - RELAXED) < 5e-4:
              ep["relax"] = t - ep["t"]
            if signal(CRZ_INFO, dat, "RESUME_UNLATCHING") and ep["unlatch"] is None:
              ep["unlatch"] = t
        elif c.address == GEAR:
          brake_hold = bool(signal(GEAR, dat, "BRAKE_HOLD"))
          if brake_hold and ep is not None and ep["brake_hold"] is None:
            ep["brake_hold"] = t - ep["t"]
        else:
          holding = signal(EPB, dat, "HOLD_STATE") == HOLD_STATE_HOLDING
          if holding and ep is None:
            ep = {"t": t, "stop0": stop, "cmd0": cmd, "relax": None, "unlatch": None,
                  "brake_hold": 0.0 if brake_hold else None}
          elif not holding and ep is not None:
            ep.update(t_rel=ep["t"] - t0, dur=t - ep["t"], exit=int(signal(EPB, dat, "HOLD_STATE")), gas=gas,
                      oplong=sent, unlatch_lead=None if ep["unlatch"] is None else t - ep["unlatch"])
            yield ep
            ep = None


def fmt(x, spec=".2f"):
  return "-" if x is None else f"{x:{spec}}"


if __name__ == "__main__":
  for path in map(os.path.abspath, sys.argv[1:]):
    name = os.path.basename(os.path.dirname(path)) if os.path.basename(path) == "rlog.zst" else os.path.basename(path)
    for ep in episodes(path):
      who = "OPLONG" if ep["oplong"] else "stock"
      onset = f"onset stop={fmt(ep['stop0'], '.0f')} cmd={fmt(ep['cmd0'], '+.3f')}"
      body = f"relax_after={fmt(ep['relax'])} brake_hold_after={fmt(ep['brake_hold'])}"
      end = f"exit->{ep['exit']} unlatch_lead={fmt(ep['unlatch_lead'])} gas={int(ep['gas'])}"
      print(f"{name} {who} t={ep['t_rel']:7.2f} dur={ep['dur']:5.1f} {onset} | {body} | {end}")
