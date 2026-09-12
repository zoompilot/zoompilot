# Force Offroad and longitudinal mode transitions

Implementation record, 2026-09-10, for the developer workflow while moving: disengage assistance
and stock cruise, enter Force Offroad, flip Alpha Longitudinal, exit Force Offroad, engage
normally, with no stop and no ignition cycle. Companion to `mazda-longitudinal.md` (the radar
protocol) and the handoff plan `force-offroad-alpha-transition-plan.md`. Route ids are the
device's `00000xxx--<id>` names; times are seconds from the route's first logged message.

Status in one line: every part of the workflow is implemented and unit-tested, the moving
hand-back (Force Offroad entry at speed) is on record as working, and the moving takeover
(Force Offroad exit into alpha at speed) is implemented behind a capability rule but **not yet
validated on the car**. Until the driver-coordinated drive below is done, moving alpha
initialization on the test vehicle is a software claim, not a demonstrated one.

## Evidence

Revisions inspected: device `comma@192.168.1.144` on `jetson-trt` 309f10f708 (opendbc
fcdbf15861), which contains all of local `danger-unstable` 14a67b9368 (same opendbc pin); the
device build already carried the 2026-09-09 hand-back broker. Raw-CAN forensics are in
`tools/mazda_long/offroad_transition/` (`route_forensics.py`, `sendcan_census.py`, the
per-route event tables and `evidence-2026-09-10.md`, VINs stripped). Panda uptime is continuous
from route 208 through 20d; only 20d to 20e is a device reboot. Real ignition-offs end 20a, 20b,
20e, 20f, 210 and 211; 208, 209, 20c and 20d ended in software.

| Route | What happened |
| --- | --- |
| 20c | Force Offroad entered at 117.7 km/h. 569.127 hand-back requested (hardwared's gate), 569.228 session `silenced -> handback`, 569.566 TX `02 10 01`, 569.572 RX `06 50 01`, 569.652 stock CRZ_INFO/CRZ_CTRL resume, 569.756 `stock traffic restored`. 0.63 s end to end. 569.943 pandad power save with ignition CAN still high, the `OffroadMode` signature; the alpha toggle path was not involved (CarParams stayed alpha, and that path waits for a stop). No camera or EPS fault bit anywhere in 20b to 20f. The stock 0x21c came back as the radar's re-init payloads for 0.4 s, then normal; the degraded-radar signature from the 2026-08-01 unattended S3 recovery (`1a 01 01 00 02 00 06 00`, ACC_OFF cycling) is absent from 208 to 211. |
| 20d | Force Offroad exited 34 s later at 113.6 km/h, alpha configured. The car never stopped, so the session manager never left STOCK (takeover was parked-only); `gate_passed` was also false 15.8 to 23.9, 61.5 to 66.2 and 93.6 to 99.6 s because the driver engaged stock MRCC (six MAIN and thirteen CAN_OFF presses, all from the wheel). Cruise stayed unavailable with nothing on screen to say why. Ended by `DoReboot` at 115.4 s; manager's gate answered at once, nothing was silenced. |
| 20e | Reboot at 100.7 km/h. 12 to 77 s: six MAIN presses, each answered by PEDALS ACC_OFF within 50 ms (once stock MRCC engaged), each cancelled with CAN_OFF; the last button of the route is a CAN_OFF at 77.5 s. 137.228 first standstill, 137.235 `parked takeover`, 137.462 TX `02 10 02`, 137.480 RX `06 50 02`, 137.538 silenced, synthetic frames from 137.543 on both buses, nothing rejected. **137.5 to 198.0 s: no MAIN, SET or CAN_OFF edge at all.** The driver never pressed main after the takeover. Under a completed takeover the body arms on MAIN in 20a 14.5 s, 20c 67.9 s and 20f 110.1 / 168.5 s (ACC_OFF up in 50 to 80 ms, `cruiseState.available` 2 ms later, openpilot's CRZ_CTRL AVAIL=1 the next frame), so "owned but main off" is an ordinary state that needs an instruction, not a defect. |
| 20f | Booted parked: takeover at 16.3 s after the FSC settle, driver pressed MAIN under the takeover, lateral and alpha long engaged. |

What the record does not establish: how the radar answers a programming-session request while
moving, whether the camera and body accept the handover at speed, and what AEB does in the
seconds around it. Those are on-car questions.

## Replay of route 20d through the new code

`tools/mazda_long/offroad_transition/replay_moving_takeover.py` drives 20d's CAN through the
current carstate and session manager, disengaged, once as recorded (parked takeover only) and
once with the moving capability. Recorded configuration: `parkToTakeOver` from 14.0 s for the
whole route, `stockCruiseOn` in the three windows above. Moving capable:
`02 10 02` at 14.117 s (115.7 km/h) after the FSC settle, undone at 15.813 s with a `02 10 01`
when the driver engaged stock MRCC (prerequisites lost), requested again at 23.948 s once the
driver disengaged. The replay cannot answer the request, so it times out at 34 s and the
manager falls back to parked-only for the session, which is the designed failure path. On the
car the radar answers within 10 ms (route fe).

## Design

One lifecycle owner (hardwared, with manager for mandatory stops), one vehicle ownership
authority (`RadarSessionManager`), one status, one line of UI. The alpha switch is the saved
preference and is editable offroad only, forced offroad included: a change applies at the next
start, natural or Force Offroad exit. There is no onroad toggle path any more (the old toggle
monitor, its standstill gate and `AlphaLongCycleAttempted` are gone), so the only moving change
of mode is the explicit developer workflow.

### Vehicle transition contract

`opendbc/sunnypilot/car/stock_ecu.py`: one `StockEcuState`, the driver's view, kept by the
session manager every control frame: notNeeded, starting, parkToTakeOver, stockCruiseOn,
ready, restoring, restored, failed. `ready` is carstate's own silence guard on the owned radar
(`radar_owned`), never the session acknowledgement. A controller that silences a stock ECU
under openpilot longitudinal exposes `stock_ecu_state`; card reads that one name and nothing
brand-specific (`card_ext.py`) and publishes it on `carStateSP.zoompilot.stockEcu`. The
detailed reasons (camera settle, CAN, which UDS reply) stay in carlog.

### Moving takeover (Mazda)

`RadarSessionManager(moving_takeover=...)`: the first takeover of a session may run while moving
when `MazdaFlags.MOVING_TAKEOVER` is set. The flag comes from the developer's
`MazdaMovingTakeover` param, read at fingerprint next to `MazdaTjaButton`
(`opendbc/sunnypilot/car/interfaces.py`), off by default. A radar-firmware rule was
considered and rejected for now: the validation vehicle's radar firmware is in the fingerprints
of three platforms, so it would have enabled an unvalidated moving takeover for every car with
that radar. Once a moving handover is on record, the firmware rule can replace the param. Rules:

- A refusal or timeout of a **moving** request, or a radar heard again under our frames (S3
  recovery), closes moving attempts for the session (`moving_open`, status `parkToTakeOver`)
  and leaves the parked attempt open; a refusal at a stop is definitive for the drive. One
  moving attempt per session.
- A parked attempt carries on if the car pulls away on a capable radar; on any other radar
  motion undoes the queued request with a default-session request, as before.
- Only the lifecycle's ordered hand-back keeps the radar stock, and only while the request
  stands; a withdrawn request is a fresh start under the normal takeover gate. Undoing our
  own unanswered or refused request latches nothing. Before this, a takeover aborted by
  motion blocked every later attempt in the session.

Upstream precedent: comma's `disable_ecu()` runs at `CI.init()` at whatever speed the car is at
after a manager restart; no upstream port gates the radar disable on standstill. The reason this
port did was the FSC's boot-time radar-presence check, which the FSC settle timer already
covers, and the unattended S3 recovery on the way out, which the ordered hand-back covers.

### Lifecycle records

`StockEcuHandBackRequest` `{id}` and `StockEcuHandBackResult` `{id, outcome}` (JSON params,
cleared on the offroad transition and at manager start; the outcome is the StockEcuState name
restored, failed or notNeeded) replace the two sticky booleans.
Consumers (`StockEcuHandBackGate` in hardwared for cycle/offroad, in manager for
reboot/shutdown/uninstall) open a request with a monotonic id and wait for the result that
carries it; a second consumer joins an open request. Card's `StockEcuHandBackServer` asserts the
brand's hand-back off the control loop while the request stands (never starting on an engaged
car, running to the end once started) and answers restored, failed or notNeeded (the
`StockEcuState` names). Outcomes:

- Force Offroad: proceed on restored/notNeeded; on failed stay open, the vehicle keeps
  neutral replacement traffic, a late recovery completes it, the exit button withdraws.
  Stopping on a failed hand-back would leave the camera without radar frames, the fault the
  hand-back exists to prevent.
- cycle, reboot, shutdown, uninstall (no cancel control): proceed on any answer.
- no answer at all within 15 s: proceed, no card is alive to hold anything.
- withdrawn (the record removed): the assert drops, the session manager finishes any in-flight
  restoration and treats the next takeover as a first one.

### Force Offroad

`OffroadModeRequested` is the preference (UI, remote settings, boot mode, screen sleep);
`OffroadMode` is written by hardwared alone. Entering while onroad waits on the hand-back, then
pandad drops the panda's ignition. Exiting clears `CLEAR_ON_ONROAD_TRANSITION` (CarParams,
ControlsReady, FirmwareQueryDone) **before** clearing `OffroadMode`, so pandad sequences the
fresh session like a boot, ELM327 until the new CarParams is ready, instead of applying the old
safety and opening the relay seconds before controls, the race that latched the camera fault on
2026-08-01 for the cycle path. Remote `OffroadMode` writes are blocked; the sunnylink toggle is
bound to the request param. The UI's offroad buttons are upstream's, unchanged: the exit button
shows while a request is pending and withdraws it.

### Presentation

No status line. The alpha switch is the saved preference, offroad-only (forced offroad
included), through upstream's developer layouts on both families (one line each: the enable
predicate). The running session's state reaches the driver the way upstream's no-entry
conditions do ("Gear not D", "Seatbelt Unlatched"): a SET/RES press before the radar is owned
raises `stockEcuNotReady` (`car_specific.py`, brand-independent, fed from `carStateSP`), an
alert in the no-entry shape (title, one line, refuse chime, 3 s) keyed on the published state:

| State | Title | Line |
| --- | --- | --- |
| starting, parked | Longitudinal Initializing | Remain parked |
| starting, moving (moving takeover in flight) | Longitudinal Initializing | Wait for the radar takeover |
| parkToTakeOver | Park to Engage Longitudinal | Alpha longitudinal takes over at the next stop |
| stockCruiseOn | Turn Off Stock Cruise | Alpha longitudinal waits for it |
| restoring | Longitudinal Handing Back | Stock cruise returns when it completes |
| failed | Longitudinal Initializing Failed | Restart the car to retry |
| ready, restored, notNeeded | none | the press engages, as on a stock car |

Two lines need no press. Parked in `starting` (the takeover not yet landed) shows
"Longitudinal Initializing / Remain parked" unprompted, re-raised every frame it holds and
cleared by motion or readiness: the one window where waiting changes the outcome (21b pulled
away 5 s short of it). The edge into `ready` shows "Alpha Longitudinal Ready" for 2 s. Both are
PERMANENT, no chime, lowest priority; neither exists under stock longitudinal (`notNeeded`),
and nothing stands once the car is rolling: the press alert carries "park to engage".

The event is PERMANENT-typed, not the `*AlertOnly` WARNING convention: WARNING shows only
while cruise or MADS lateral is active, and this has to reach a driver whose lateral is off or
paused (brake held at the stop where the takeover happens). On route 0000021b (2026-09-11, the first
drive on this build) the driver pulled away at 11.6 s, `parkToTakeOver` came at 16.0 s, and
three minutes of MAIN/SET presses (stock MRCC engaged twice, cancelled twice) showed nothing
until the first stop at 181 s took the radar over. Cruise main off is the car's own state, on
its own cluster, as on a stock Mazda; openpilot adds nothing there (route 0000021d: SET at a
standstill with main off, 7 s after `ready`).

Lateral does not wait for any of this. `cruiseState.available` and the panda's `acc_main_on`
follow the MRCC main switch from the first frame (2026-09-11; see mazda-longitudinal.md, "Main
is the main switch"), so MAIN gives MADS lateral while the radar is still stock, and the body
keeps its arming across the takeover. A SET press before ownership engages stock MRCC in the
body (the radar is still stock) and shows the alert; at the stop the takeover waits for that
stock engagement to be cancelled ("Turn Off Stock Cruise").

The 2026-09-10 cut carried one line under the switch (`initializing`, `ready`, `failed`,
`ui_state.alpha_long_status`, a mici subtitle and a tizi description). Removed the next day:
a line on a settings page, on a switch that is disabled onroad, says nothing at the moment
of the press, and the alert already carried the reason.

## Behaviour as built

| State | Shown | Done |
| --- | --- | --- |
| Offroad, natural or forced | switch editable | preference only; applies at the next start |
| Onroad | switch disabled (upstream idiom for offroad-only toggles) | nothing; the moving change of mode is Force Offroad and back |
| Force Offroad requested, disengaged, moving or stopped | exit button (the cancel) | hand-back (0.63 s on record at 117 km/h), then OffroadMode |
| Force Offroad requested while engaged | upstream's "disengage" dialog | nothing starts; the server never starts a hand-back on an engaged car |
| Exit into stock | nothing | old CarParams cleared first; stock CarParams and safety; cruise from CRZ_CTRL |
| Exit into alpha, capable radar | nothing; SET before ready: "Longitudinal Initializing" | FSC settle, moving request, silence guard, fresh driver engagement |
| Exit into alpha, other radars | SET press: "Park to Engage Longitudinal" | parked takeover at the next stop |
| MAIN before the takeover | MADS lateral engages | main is not gated; SET engages stock MRCC in the body until the takeover |
| Parked, takeover starting | "Longitudinal Initializing / Remain parked", unprompted | clears on motion or `ready` |
| Takeover lands | "Alpha Longitudinal Ready", 2 s | once per edge into `ready` |
| Alpha to offroad to alpha, no edit | same as a fresh initialization | the old session's hand-back died with its process; reacquired |
| Stock cruise engaged at exit | SET press: "Turn Off Stock Cruise" | gate holds; a queued request is undone |
| Owned, main off | the cluster's own MRCC indicator | no synthetic arming; the body arms on the driver's MAIN (20a/20c/20f) |
| Hand-back fails | exit button withdraws; SET press: "Longitudinal Initializing Failed" | request held open, neutral traffic continues, late recovery completes |
| Request withdrawn | nothing | a fresh takeover under the normal gate |
| Card absent / CAN lost | nothing | status unknown; no readiness |
| Ignition off, power loss, thermal offroad | normal shutdown | not held; the radar recovers through S3 as before |

## Validation

Software, run 2026-09-10 with the local `.venv` (after the review cut):

```
PYTHONPATH=. .venv/bin/python -m pytest -q opendbc_repo/opendbc/car/mazda/tests opendbc_repo/opendbc/safety/tests/test_mazda.py
  # 821 passed, 44 skipped, 212 subtests passed
PYTHONPATH=. .venv/bin/python -m pytest -q openpilot/sunnypilot/selfdrive/car/tests openpilot/sunnypilot/system/hardware/tests \
  openpilot/sunnypilot/selfdrive/selfdrived/tests openpilot/selfdrive/selfdrived/tests/test_alerts.py \
  openpilot/system/manager/test
  # 215 passed, 31 skipped
PYTHONPATH=. .venv/bin/python -m pytest -q openpilot/selfdrive/ui/sunnypilot/mici/tests/test_mici_settings.py   # 57 passed
PYTHONPATH=. .venv/bin/python -m pytest -q openpilot/selfdrive/ui/sunnypilot/tests/test_tizi_settings.py        # 1 passed
ruff check <every touched file>                                                                                   # clean
```

The golden TX fixture is byte-identical. One pre-existing failure was repaired, not hidden:
`test_panda_arms_lateral_before_the_carstate_guard_lifts` drove carstate without the
controller's ownership claim and without the ENGINE_DATA bus witness, both added by the
2026-09-09 refactor; it now feeds both and passes with the ordering it was written to check.

Limits of the software validation: a replay cannot answer a UDS request; the tests use fakes for
the params transport; the UI tests monkeypatch the presentation inputs. The
`params_keys.h` change means a device `scons` rebuild.

### On-car, driver-coordinated (not done)

On the test vehicle (CX-5 2022, radar K131-67XK2-F, FSC GSH7-67XK2-U, EPS KSD5-3210X-C-00),
one drive with the device on this build and `MazdaMovingTakeover` set to 1 (`Params().put_bool`
over ssh; there is no UI for it), an rlog kept for each step, the driver ready to take over and
the road empty ahead:

1. Cold start in stock mode, drive, disengage everything, Force Offroad at speed, flip alpha on,
   exit. Expect "initializing", "taking over the radar", then "ready, turn on cruise main"
   within about 12 s of exit without stopping; press MAIN, SET; engage. Watch for: NRC on 0x76c
   to the `02 10 02` (then the status must say movingRefused and the parked path must still
   work at the next stop), CAM_LANEINFO ERR_BIT, CAM_LKAS ERR_BIT_1, STEER_RATE LKAS_FAULT, any
   cluster warning, PEDALS ACC_OFF behaviour on MAIN.
2. Alpha, moving: Force Offroad, no edit, exit. Expect the same sequence (hand-back 0.6 s, fresh
   takeover).
3. Alpha, moving: Force Offroad, flip alpha off, exit. Expect "stock cruise", stock MRCC arms and
   engages normally, no `1a 01 ...` CRZ_CTRL payloads, no ACC_OFF cycling.
4. Comma restart onroad (manager restart) in alpha: expect adoption of the quiet radar or a
   moving takeover, no camera gap longer than a stock frame gap.
5. Cancelled entry: request Force Offroad, withdraw within a second, then again after the
   restore. Expect no session alternation; after a withdrawn-after-restore the cycle at the next
   stop.
6. A later clean ignition: stock cruise works, no i-ACTIVSENSE warning.

AEB: the programming session disables it from the moment it lands, as for every alpha-long
drive; the moving takeover moves that moment from the next stop to the exit from Force Offroad.
Whether the FSC shows anything at that instant is part of step 1. Nothing here clears DTCs,
resets an ECU or presses a button on the driver's behalf.

## Supported configurations

- Moving takeover: only with `MazdaMovingTakeover` set by the developer (validation pending).
- Moving hand-back (Force Offroad entry, cycle, reboot at speed): every alpha-long Mazda;
  on record at 117 km/h on the validation vehicle.
- Every other Mazda alpha-long configuration: parked takeover at the next stop, with the
  status saying so; behaviour otherwise as before.
- Non-Mazda and stock-longitudinal sessions: the hand-back answers notNeeded at once; nothing
  else changes.
