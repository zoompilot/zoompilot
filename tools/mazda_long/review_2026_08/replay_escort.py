"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Replay route fe segs 6-7 through the NEW CarController; show the release escort.
"""
import sys, os
sys.path.insert(0, "/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot")
sys.path.insert(0, "/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot/opendbc_repo")
sys.path.append("/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot/tools/mazda_long")
from replay_standstill_hold import build_controller, frames, decode_cmd, mock_inputs

base = "tools/mazda_long/device_data/000000fe--757df8e60f--"
os.chdir("/Users/zeph/Developer/experiments/sunnypilot_proj/sunnypilot")

rows = []
for seg in (6, 7):
    rows.extend(frames(base + f"{seg}/rlog.zst"))

ctrl = build_controller()
t0 = rows[0][0]
last = None
for t, cc, cs, brake_hold in rows:
    control, control_sp, carstate = mock_inputs(cc, cs, brake_hold)
    sends = ctrl.update_longitudinal(control, control_sp, carstate)
    ctrl.frame += 1
    info = next((d for a, d, b in sends if a == 0x21b and b == 0), None)
    ctl = next((d for a, d, b in sends if a == 0x21c and b == 0), None)
    trk = next((d for a, d, b in sends if a == 0x364 and b == 0), None)
    if info is None:
        continue
    cmd = decode_cmd(info)
    unl = (info[6] >> 6) & 1
    stop = (info[5] >> 2) & 1
    hl = ctl[2] >> 7
    ph = (ctl[3] >> 5) & 7
    d = None
    if trk is not None:
        dist = ((trk[0] << 4) | (trk[1] >> 4)) * 0.0625
        relv_raw = ((trk[3] << 3) | (trk[4] >> 5)) & 0x7ff
        relv = (relv_raw - 2048 if relv_raw > 1023 else relv_raw) * 0.0625
        d = (round(dist, 2), round(relv, 2))
    state = (cmd > 0, stop, unl, hl, ph, brake_hold, cs.standstill)
    key = state
    if key != last or (d is not None and hl and (t - t0) % 0.5 < 0.02):
        print(f"{t-t0:8.2f}  cmd={cmd:+5d} stop={stop} unl={unl} hl={hl} ph={ph} "
              f"hold={int(brake_hold)} ss={int(cs.standstill)} trk={d}")
        last = key
