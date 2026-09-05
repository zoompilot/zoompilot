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
else:
  cp = Params().get('CarParamsPersistent')
  if cp is None:
    raise SystemExit('segment has no CarParams and no saved configuration is available')
  Params().put('CarParams', cp)
  print('CarParams seeded from the saved device configuration')
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
import signal, sys, time
import openpilot.cereal.messaging as messaging

# The script kills this at N seconds and the loop runs to N+5, so without this
# the summary below never prints - which is the only part worth reading.
_stop = False


def _bye(*_a):
  global _stop
  _stop = True


signal.signal(signal.SIGTERM, _bye)

# The budget is what matters, not the average. A frame past 50 ms is a dropped
# camera frame, and modeld's filter turning that into frameDropPerc > 1 is
# ET.SOFT_DISABLE in selfdrived. A rolling window hides exactly the frames
# that do it, so keep every sample and report the tail.
BUDGET = 50.0
sm = messaging.SubMaster(['modelV2', 'narrowRoadCameraState', 'chestnutState'])
t0 = time.monotonic(); last = t0
cnt = dict.fromkeys(sm.services, 0); big = 0
ex = []; over = []; worst_drop = 0.0; lagging = 0
while not _stop and time.monotonic() - t0 < float(sys.argv[1]):
  sm.update(100)
  for k in sm.services:
    cnt[k] += sm.updated[k]
  if sm.updated['modelV2']:
    m = sm['modelV2']
    big += m.big
    e = m.modelExecutionTime * 1000
    ex.append(e)
    if e > BUDGET:
      over.append((round(time.monotonic() - t0, 1), round(e, 1)))
    worst_drop = max(worst_drop, m.frameDropPerc)
    lagging += m.frameDropPerc > 1.0
  if time.monotonic() - last >= 5:
    last = time.monotonic()
    w = ex[-100:] or [0]
    print('t=%3.0f cam=%d modelV2=%d big=%d exec_last100 mean/max=%.1f/%.1f  worst_all=%.1f over_budget=%d' % (
      last - t0, cnt['narrowRoadCameraState'], cnt['modelV2'], big,
      sum(w) / len(w), max(w), max(ex or [0]), len(over)), flush=True)

def q(xs, p):
  xs = sorted(xs)
  return xs[min(len(xs) - 1, int(len(xs) * p))] if xs else 0.0

if ex:
  print('', flush=True)
  print('=== tail over %d frames' % len(ex), flush=True)
  print('  mean %.1f  p50 %.1f  p99 %.1f  p99.9 %.1f  max %.1f ms' % (
    sum(ex) / len(ex), q(ex, .50), q(ex, .99), q(ex, .999), max(ex)), flush=True)
  print('  frames over %.0f ms: %d (%.3f%%)' % (BUDGET, len(over), 100.0 * len(over) / len(ex)), flush=True)
  if over:
    print('  the offenders (t=s, ms): %s' % (over[:20],), flush=True)
  print('  worst frameDropPerc %.2f%%  frames with >1%% (modeldLagging): %d' % (worst_drop, lagging), flush=True)
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
tail -8 /data/tmp/mon.log
