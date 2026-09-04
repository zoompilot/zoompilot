#!/bin/bash
# Copyright (c) 2026-, Zeph Leggett.
#
# This file is part of zoompilot and is licensed under the MIT License.
# See the LICENSE.md file in the root directory for more details.
#
# The closest thing to a drive without a car: live camerad and the real modeld
# on the bench, with a fake selfdriveState saying "disengaged" so the joining
# state swaps to the accelerator. The replay tool cannot stand in for this:
# it feeds frames as fast as modeld takes them and nothing else is running, so
# it never showed the FunctionFS write being interrupted by msgq's SIGUSR2
# wakeups from camerad, which is what lost the link on the car. This did,
# within 30 s, every time.
#
#   kill -TERM $(pgrep -f accelerators.jetlink.jetlinkd)   # jetlinkd owns the link offroad
#   tools/jetlink_live_bench.sh 120
#
# Writes mon.log (message rates, big-model frames, exec time), modeld.log and
# camerad.log to /data/tmp. Seeds CarParams from a recorded segment because
# modeld blocks on it before its first frame, and removes it afterwards.
set -u
N=${1:-60}
SEG=${SEG:-$(ls -d /data/media/0/realdata/*--* 2>/dev/null | tail -1)}
PY=/usr/local/venv/bin/python3
cd "$(dirname "$0")/.."
export PYTHONPATH=$PWD
mkdir -p /data/tmp

$PY -c "from openpilot.common.hardware import HARDWARE; HARDWARE.set_power_save(False)"
echo "cores online: $(cat /sys/devices/system/cpu/online)"
$PY - "$SEG" <<'PYEOF'
import sys, os
from openpilot.tools.lib.logreader import LogReader
from openpilot.common.params import Params
seg = sys.argv[1]
rlog = next((os.path.join(seg, n) for n in ('rlog.zst', 'rlog') if os.path.exists(os.path.join(seg, n))), None)
if rlog is None:
  raise SystemExit(f"no rlog in {seg}; set SEG=<segment dir>")
for m in LogReader(rlog):
  if m.which() == 'carParams':
    Params().put('CarParams', m.carParams.as_builder().to_bytes())
    print('CarParams seeded from', seg, m.carParams.carFingerprint)
    break
PYEOF

$PY - <<'PYEOF' > /dev/null 2>&1 &
import openpilot.cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
pm = messaging.PubMaster(['selfdriveState'])
rk = Ratekeeper(100)
while True:
  m = messaging.new_message('selfdriveState')
  m.selfdriveState.enabled = False
  pm.send('selfdriveState', m)
  rk.keep_time()
PYEOF
SDS=$!

(cd openpilot/system/camerad && exec ./camerad) > /data/tmp/camerad.log 2>&1 &
CAM=$!
sleep 4

$PY - "$((N + 5))" <<'PYEOF' > /data/tmp/mon.log 2>&1 &
import sys, time
import openpilot.cereal.messaging as messaging
sm = messaging.SubMaster(['modelV2', 'narrowRoadCameraState', 'chestnutState'])
t0 = time.monotonic(); last = t0
cnt = dict.fromkeys(sm.services, 0); big = 0; exec_ms = []
while time.monotonic() - t0 < float(sys.argv[1]):
  sm.update(100)
  for k in sm.services:
    cnt[k] += sm.updated[k]
  if sm.updated['modelV2']:
    big += sm['modelV2'].big
    exec_ms.append(sm['modelV2'].modelExecutionTime * 1000)
  if time.monotonic() - last >= 5:
    last = time.monotonic()
    e = exec_ms[-100:] or [0]
    print('t=%3.0f cam=%d modelV2=%d big=%d chestnutState=%d exec_last100 mean/max=%.1f/%.1f ms' % (
      last - t0, cnt['narrowRoadCameraState'], cnt['modelV2'], big, cnt['chestnutState'],
      sum(e) / len(e), max(e)), flush=True)
PYEOF
MON=$!

$PY -m openpilot.selfdrive.modeld.modeld > /data/tmp/modeld.log 2>&1 &
MOD=$!
echo "running $N s: camerad=$CAM modeld=$MOD"
sleep "$N"
kill -INT $MOD; sleep 3; kill -9 $MOD 2>/dev/null
kill -TERM $CAM $SDS $MON 2>/dev/null; sleep 1; kill -9 $CAM $SDS $MON 2>/dev/null
$PY -c "from openpilot.common.params import Params; Params().remove('CarParams')"

echo "big-model joins: $(grep -c 'large model joined' /data/tmp/modeld.log)  failures: $(grep -c 'large model failed' /data/tmp/modeld.log)"
grep "LinkError:" /data/tmp/modeld.log | sort | uniq -c
tail -3 /data/tmp/mon.log
