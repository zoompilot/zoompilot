"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Replay route 4d segs 3-4 (t+180-240) through the NEW CarController: does the lead survive
the driver-gas drive-off and the disengage at t+212.4?
"""
import sys, os, glob
sys.path.insert(0, "/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot")
sys.path.insert(0, "/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot/opendbc_repo")
sys.path.append("/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot/tools/mazda_long")
os.chdir("/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot")
from openpilot.tools.lib.logreader import LogReader
from replay_standstill_hold import build_controller, decode_cmd, mock_inputs

BASE = "tools/mazda_long/test_data/route_4d_cx9/0000004d--3242c750a7--"
rows = []
cc = cs = lead = None
brake_hold = False
for seg in (3, 4):
  for m in LogReader(BASE + f"{seg}/rlog.zst"):
    w = m.which()
    if w == "carControl":
      cc = m.carControl
    elif w == "carControlSP":
      lo = m.carControlSP.leadOne
      lead = (lo.dRel, lo.vRel)
    elif w == "can":
      for c in m.can:
        if c.address == 0x228 and c.src == 0:
          brake_hold = bool((bytes(c.dat)[2] >> 4) & 1)
    elif w == "carState":
      cs = m.carState
      if cc is not None:
        rows.append((m.logMonoTime * 1e-9, cc, cs, lead, brake_hold))

ctrl = build_controller()
t0 = rows[0][0]
last = None
for t, cc, cs, lead, hold in rows:
  control, control_sp, carstate = mock_inputs(cc, cs, hold, lead)
  sends = ctrl.update_longitudinal(control, control_sp, carstate)
  ctrl.frame += 1
  info = next((d for a, d, b in sends if a == 0x21b and b == 0), None)
  ctl = next((d for a, d, b in sends if a == 0x21c and b == 0), None)
  trk = next((d for a, d, b in sends if a == 0x364 and b == 0), None)
  if ctl is None:
    continue
  hl, ph = ctl[2] >> 7, (ctl[3] >> 5) & 7
  d = None
  if trk is not None:
    dist = ((trk[0] << 4) | (trk[1] >> 4)) * 0.0625
    d = round(dist, 2)
  stop = (info[5] >> 2) & 1 if info is not None else "?"
  unl = (info[6] >> 6) & 1 if info is not None else "?"
  key = (cc.longActive, cs.cruiseState.enabled, hl, ph, stop, unl, int(cs.standstill), int(cs.gasPressed))
  tr = t - t0
  if key != last or (195 <= tr <= 222 and d is not None):
    print(f"t+{tr:7.2f} lact={int(cc.longActive)} en={int(cs.cruiseState.enabled)} gas={int(cs.gasPressed)} "
          f"ss={int(cs.standstill)} stop={stop} unl={unl} hl={hl} ph={ph} trk={d} lead={None if not lead else (round(lead[0],1), round(lead[1],1))}")
    last = key
