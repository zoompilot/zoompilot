# Mazda longitudinal (alpha long): evidence and design notes

This is the measurement record behind the Mazda longitudinal port in `opendbc/car/mazda/`
(`carcontroller.py`, `carstate.py`, `longitudinal.py`, `mazdacan.py`, `radar_interface.py`,
`values.py`) and the panda side in `opendbc/safety/modes/mazda.h`. The source comments say what
the code does; the route ids, frame counts and distributions that justify each constant live
here. Route ids are the dongle-side segment names used in `tools/mazda_long/test_data/`; "wire
frame" means one 50 Hz CRZ_INFO frame, "frame" alone means one 100 Hz control frame. Every
engaged-mode number was measured on a 2022 Mazda CX-5 unless a route says otherwise.

## Bus topology

Openpilot silences the stock radar (UDS address 0x764) and stands in for it. The radar's frames
are CRZ_INFO (0x21b, the accel command), CRZ_CTRL (0x21c, cruise state), a static frame 0x499
and six track slots 0x361 to 0x366. The synthetic copies are sent on bus 0 (the car) and bus 2
(the camera) because the panda only forwards received frames between those buses, never its
own transmissions. CRZ_EVENTS (the dash setpoint) and PEDALS (the body ECU's cruise state) are
owned by other ECUs and keep transmitting through the takeover, which is what lets engagement
stay with the car (`pcmCruise = True`).

## Radar takeover: the UDS session latch

The radar does not implement COMMUNICATION_CONTROL (0x28 replies NRC 0x11), so upstream's
`disable_ecu()` cannot be used. A DIAGNOSTIC_SESSION_CONTROL request for the programming
session (`02 10 02`) stops all of its periodic frames. The radar stays silent as long as tester
present (`02 3e 80`) keeps arriving at 2 Hz and falls back to the default session on its S3
timeout (about 5 s) otherwise. The programming session disables AEB while it is in effect, so,
like every `disable_ecu` caller, the port only starts a silencing episode pre-motion.

The radar answers every session request within about 10 ms. Route 000000fe t+15.0: request
`02 10 02`, positive response `06 50 02` carrying P2* = 5.0 s, which is the S3 timeout. Because
the session manager consumes the response on the same control frame it arrives, no freshness
window is needed on RADAR_UDS_RESPONSE. NRC 0x78 (response pending) is the one negative response
a UDS client waits through rather than fails on, and `radar_session_refused` excludes it. Any
other negative response fails the silencing episode immediately; a radar that answers nothing at
all is given `RADAR_SESSION_LIMIT_T` (10 s) and then the episode gives up for the drive, and stock
keeps the bus.

The panda allows exactly two UDS transmissions to 0x764: tester present and session control for
sessions 0x01 and 0x02. Flashing services stay blocked.

### FSC settle gate

Silencing the radar too early after a cold boot latches an i-ACTIVSENSE fault that only a
roughly 15 min power-down clears. The camera (FSC) runs a radar-presence check in the seconds
after its boot settles. On the setup/teardown drives the check faulted when the radar went quiet
1.9 s after the camera's boot-settle broadcast and passed from 5.8 s; waiting about 8 s was
proven clean, and `FSC_SETTLE_T` is 10 s. The check's verdict is invisible until first motion, so
the gate is a timer on the camera's settle signal, not any fault bit.

The settle signal is CAM_LANEINFO.NO_ERR_BIT, a pure boot marker that clears at 2.8 to 6.0 s
after boot and is never set again while driving. A latched fault (ERR_BIT) also shows the boot
marker clear, so ERR_BIT holds the timer at zero as well. Before the first CAM_LANEINFO frame
the parser reads all-zero, which would count as settled, hence the `cam_laneinfo_seen` latch.

CAM_LANEINFO is a roughly 2 Hz message. The longest period measured across 26 or more segments on
two cars is 0.563 s (`CAM_LANEINFO_PERIOD_T`). A freshness window shorter than one period reads
every inter-frame gap as a dropout, zeroes the settle timer each time, and the teardown gate never
opens. `CAM_LANEINFO_FRESH_T` is 1.5 s, 2.7x the longest observed period, and still catches a real
camera dropout.

### Tried and rejected: BIT2 as a settle input

CAM_LANEINFO.BIT2 used to gate the settle timer together with NO_ERR_BIT. It is byte-identical to
NO_ERR_BIT on every frame of the 40 alpha-long routes the gate was developed against, so it
carried no information there. Another CX-5 2022 with identical camera firmware (GSH7-67XK2-U)
then cold-booted with BIT2 latched high and NO_ERR_BIT clear for a whole ignition cycle. That
pinned the timer at zero, the radar was never silenced, and the two-master guard held accFaulted
for the entire drive with nothing to tell the driver why. BIT2 was removed from the gate.

### Session state machine

`RadarSessionManager` has four states: STOCK (radar broadcasting, nothing transmitted),
SILENCING (requesting the programming session), SILENCED (radar quiet, tester present plus
synthetic frames) and HANDBACK (requesting the default session, synthetic frames continue).

Setup waits for the settle gate and never pulls the radar out from under an active stock MRCC
engagement (a driver who pressed SET before the gate passed on a warm boot); the driver has to
disengage first. Adopting a radar that is already quiet without having silenced it (a process
restart after a takeover) disables nothing and may proceed anywhere, but "quiet" there is the
full guard window (`STOCK_RADAR_GUARD_T`), not the 50 ms alive window; see the gap census below.

A radar heard again while SILENCED is an S3 recovery (tester present not landing for 5 s) or a
radar that was never really silenced. Two masters is the hazard, so the synthetic frames stop
either way. The session request is gated exactly like the first teardown, because it disables
AEB when it lands and a broadcasting radar is a working one: at a stop it is re-requested at
once; moving, stock keeps the bus (carstate's accFaulted takes longitudinal down) until the next
stop. This has never been observed on the car. The only stock CRZ_INFO ever heard after a
takeover, in 10 of 25 alpha-long routes, was the ordered hand-back's own answer, 80 ms after the
`10 01` request.

### Teardown and hand-back

The hand-back to stock has to complete while the openpilot processes are still running: pandad
blocks TX within about 100 ms of an onroad cycle starting. So the hand-back is driven from the
control loop off `CC_SP.stockEcuHandBack`, and the process restart is requested only once the
stock radar is heard again. A hand-back the radar never answers stops waiting after
`RADAR_SESSION_LIMIT_T` so the restart can proceed.

Once a hand-back has run to completion the radar stays stock for the rest of the process
(`handback_completed`). The producer's contract is to hold the assert until the process exits;
the latch is the backstop for one that does not, because a dropped assert would otherwise read as
a withdrawal and, parked with the gate still passed, re-silence the radar right before shutdown.
That is the unattended S3 recovery the hand-back exists to prevent. A hand-back withdrawn before
the restart (toggle flipped back) returns to STOCK and re-runs the normal takeover.

Synthetic radar frames keep flowing through the hand-back so the camera never sees a radar gap.

In `mazda.h` the replaced-radar addresses are deliberately `check_relay = false`. That mechanism
exists for harness-blocked ECUs that are silent from ignition on, and any RX after 1 s latches a
permanent relay_malfunction. This radar is software-silenced mid-session: alive for the first
10 s or so by design and deliberately overlapped during the ordered hand-back, so the relay check
would fault every boot. The two-master guard lives in carstate instead.

## Stock radar gap census and the two-master guard

Stock CRZ_INFO runs at 50 Hz. Over 7.25M stock frames across 166 routes the inter-arrival gap is
p99.9 30.6 ms and p99.99 31.0 ms, but 9 gaps ran past 50 ms and 2 past 100 ms, the longest
105.7 ms, one of them mid-drive at speed (route 0000002d seg 28). A stock radar therefore drops
2 to 5 frames in a row a handful of times per 40 h.

`STOCK_RADAR_ALIVE_T` (0.05 s) is only the SILENCING to SILENCED handover (we asked for the
session and the radar stopped) and the accFaulted watch. It is not enough to adopt a quiet radar
we never silenced: adopting on one of those gaps put synthetic CRZ_INFO on the bus under a live
radar, two masters, and then, when the radar was heard again 2 to 3 frames later, sent the
programming-session request at speed. Adoption waits out `STOCK_RADAR_GUARD_T` instead, about 12x
the longest gap ever observed.

### Guard ordering between the software and the panda

Engagement, and with it MADS lateral, stays blocked until the stock radar has been silent for
the guard window. The panda runs the same guard (`MAZDA_RADAR_SILENT_FRAMES`, 50 PEDALS frames
at 50 Hz = 1.0 s), but its rx hook cannot see the stock CRZ_INFO (it is deliberately not an rx
check, since it goes stale at the teardown), so it clocks from our first synthetic CRZ_INFO
transmission instead, which lands at the latest `STOCK_RADAR_ALIVE_T` plus one `LONG_STEP` after
the last stock frame.

Both machines arm MADS off their own guard completing, and the order matters. If the software
completes first, lateral engages and the controller ramps torque from zero at `STEER_DELTA_UP`
per frame while the panda still rejects every 0x243. When the panda's guard then completes, its
rate limiter (`desired_torque_last = 0`) allows at most one `STEER_DELTA_UP` step, rejects the
36 to 84 counts being commanded by then, resets, and rejects every following frame until the
command falls back under one step: seconds of zero 0x243 to an EPS whose camera copy is
relay-blocked. That is the ERR_BIT_1 starvation of routes 00000116 and 00000117 (2026-08-27), and
route 00000148 reached the same starvation by another door (see mazda-lateral.md). So the software
guard is derived to complete strictly after the panda's:

    STOCK_RADAR_GUARD_T = STOCK_RADAR_ALIVE_T + LONG_STEP * DT_CTRL + PANDA_RADAR_SILENT_T + margin
                        = 0.05 + 0.02 + 1.0 + 0.2 = 1.27 s

The 0.2 s margin covers PEDALS period jitter over 50 frames plus the CAN to carstate to controller
to panda latency. Arming late is harmless: the panda holds its armed edge until pandad's 1 Hz MADS
heartbeat has disagreed three times.

Without the panda-side latch, the panda's acc_main_on edge fired at boot (MRCC main persists
over ignition), was consumed and exited long before the software engaged, and the software's
whole MADS window then transmitted into rejections.

### The block wears two hats

Before the first teardown of the drive the block is the expected boot phase (FSC settle plus UDS
handover, 10 to 15 s), not a fault. Holding availability low keeps engagement out with at most a
wrongCarMode no-entry toast. Raising accFaulted here showed a permanent "Cruise Fault: Restart
the Car" on every start for a condition that clears by itself. After the radar has been silenced
once, hearing it again is a real two-master conflict (dropped tester present, S3 recovery, or the
ordered hand-back) and is a real accFaulted. The alpha-long toggle monitor relies on exactly this
edge as its "stock radar heard" acknowledgment.

### Tried and rejected: gating availability alone

The guard used to ride on `cruiseState.available` alone. MADS engages off the enabled edge but
only ever releases off an availability falling edge, so pinning availability low held the engage
path open while shutting every off-switch: a stock MRCC engage inside the guard window latched
lateral on with no way out short of ignition off (route 00000057 t+13.7 to 37.7; a cancel at
t+28.9 did nothing). Both halves are gated together now, and adopting a live engagement the
instant the guard lifts would be an engage the driver never asked for, so the stock state has to
pass through idle once before `cruiseState.enabled` may follow it (`cruise_enabled_blocked`).

## MRCC state semantics under openpilot longitudinal

### PEDALS as the cruise state source

The radar teardown silences the radar-owned CRZ_CTRL frame, so cruise state comes from PEDALS
(0x165): ACC_OFF (bit 2) means MRCC is armed but idle, ACC_ACTIVE (bit 3) means engaged.
Brake-only PEDALS samples can arrive with both bits low mid-press; carstate and the panda rx
hook both hold the previous state through them, otherwise MADS sees a false availability drop
and force-disengages lateral.

A wheel CANCEL is different: it turns the MRCC main state off for real and has to land even with
the brake down. Holding through it kept lateral engaged against a cancel mashed under braking
until the brake was released 4 s later (route 7f9e3ff336 t+484 to 488). The PEDALS reaction runs
a few frames behind the button, so `CANCEL_CONTEXT_T` (0.5 s) lets availability drops land for
that long after a CAN_OFF press.

### Panda engagement qualifier

The panda arms `controls_allowed` only on an ACC_ACTIVE rising edge backed by a recent RES, SET_P
or SET_M press, the hyundai_common form: ACC_ACTIVE alone can be the body answering frames we
fabricate. Across a 104-engagement census every logged engagement shows the press 30 to 70 ms
before PEDALS.ACC_ACTIVE rises, with zero genuine button-less engagements, so
`MAZDA_ENGAGE_BTN_WINDOW` of 10 CRZ_BTNS frames (1 s at 10 Hz) is generous. The tx hooks already
drop engaged-claiming frames while controls are not allowed, so this is defense in depth.

The CRZ_INFO ACC_ACTIVE bit (bit 33) is gated on `controls_allowed` the way CRZ_CTRL's CRZ_ACTIVE
is. There is no deadlock: the body raises PEDALS.ACC_ACTIVE off the SET press 10 to 20 ms before
the first ACC_ACTIVE = 1 frame in every logged engagement, so `controls_allowed` leads the bit.

### cruiseState.standstill

PEDALS.STANDSTILL is the PCM's "wheels are stopped" bit, not a stock-ACC hold state, so it stays
set for exactly as long as the car is not moving. LongControl gates its starting condition on
`cruiseState.standstill`, so reporting it under openpilot longitudinal deadlocked every stop:
long control could not leave stopping until the car moved, and the car could not move until long
control left stopping. The stock MRCC is not in the loop under alpha long, so there is no stock
standstill to report; Hyundai and Tesla report False for the same reason.

The standstill signal itself comes off ENGINE_DATA.SPEED (<= 0.1 kph) while vEgo comes off
WHEEL_SPEEDS, because the panda's `vehicle_moving` reads the same ENGINE_DATA field and the two
must agree on when the car counts as stopped. The roughly 0.03 m/s disagreement with vEgo at the
stop is the price of that parity.

### Cancel while the stock radar still owns the bus

Under op-long, controlsd raises `cruiseControl.cancel` whenever `cruiseState.enabled` has no
matching `CC.enabled`. While the stock radar still owns the bus (the pre-teardown settle window,
and the silencing-failed stay-stock fallback) that engagement is the driver's own stock MRCC,
and the 10 Hz CANCEL would turn its main off within about 100 ms. The controller leaves it alone
until the radar has been silenced once; the teardown gate already waits out a stock engagement.
The deeper fix is carstate not reporting a stock engagement as `cruiseState.enabled` under op-long
at all, which needs an audit of every enabled consumer first.

### Gas override

The CRZ_INFO engaged bits follow `CC.enabled` the way Honda drives ACC_CONTROL's CONTROL_ON: a gas
press is an override, not a disengagement, so enabled holds while controlsd drops `longActive`
and the command goes to zero. Clearing the bits mid-decel takes the PCM out of ACC mode as the
driver adds throttle, so a light pedal input lands as a lurch and a rev flare. Stock MRCC holds
the bits through 9 of 11 decel overrides (`analyze_gas_override.py`, 576 stock segments).

### Stock forward collision display

CAM_EMPTY (0x21d) leaves its idle 0x7f status only while the camera is actively showing the
collision display (route 0000004d t+213; zero episodes in 50 h of stock cruising), so
`stockFcw` is derived from it together with the DBC-named CAM_PEDESTRIAN warning bits. Upstream
maps no alert to stockFcw for Mazda. `stockAeb` stays unmapped because no candidate signal has
ever activated.

## CRZ_INFO checksum

CRZ_INFO.CHKSUM is the inverted sum of the first seven bytes, with the two event bits left out of
the sum: STOPPING (byte 5, mask 0x04) and RESUME_UNLATCHING (byte 6, mask 0x40). This was
verified against 1.67M stock frames with zero mismatches, including all 9,681 stop-bit frames
and all 269 unlatch-pulse frames.

Before the fix the port summed the unlatch bit, which made every pulse frame it ever sent
checksum-invalid by exactly 0x40. The camera latched the SCBS trio about 3 frames into each
pulse, 11 of 11 releases across every radar-content variant tried (route 132 closed the case,
falsifying the 0x364 template theory), while the body, which does not validate the sum, still
answered the pulse. Route 00000139 was the first drive with the corrected sum: one pulse, camera
clean. The fix shipped 2026-08-30.

### Standby patterns

Only an engaged CRZ_INFO frame carries a live command. Armed-idle pegs the command field exactly
like the main-off standby: 47,752 of 47,752 stock armed-idle frames carry raw 8190 (4.094 m/s2
after the 4096 offset). Armed-idle also sets bit 47 and advertises ACC_SET_ALLOWED whenever the
brake is up so the dash accepts SET; the brake is stock's one observed gate on it (99.9% of
armed-idle frames follow BRAKE_ON). The panda allows the two standby shapes byte-exactly
(checksum included) instead of decoding them as a huge accel command; ACC_ACTIVE (data[4] bit 1)
stays required-low on that allowance so no engaged frame can ride it.

ACCEL_CMD is 13 bits in raw units of 0.001 m/s2 with a 4096 offset. The panda window is the ISO
one, +2.0 / -3.5 m/s2 (`ACCEL_MAX` / `ACCEL_MIN`); stock MRCC itself commands down to raw -3891
in lead stops.

## Stop-and-go

### Design

`StandstillHold` holds the car stopped until the plan asks to move, the way Toyota and Honda do
it: both upstream ports drive the standstill request straight off the plan and off car feedback
with no timers in the path. Toyota clears its request on `actuators.accel > 0` and re-asserts
it whenever the plan is not asking to move; Honda asserts STANDSTILL for exactly as long as long
control is in its stopping state. Neither substitutes a canned command for the plan's own.
LongControl already parks at `CP.stopAccel` while stopping, which for this car is the stock hold
value: stock MRCC holds raw -1024 at a stop, so `stopAccel = -1.024` and the plan's value is sent
as-is. Nothing in the machine latches; `holding` is recomputed every frame.

### Body brake hold and the relax

GEAR.BRAKE_HOLD is the body ECU taking the standstill hold over and holding the brakes itself.
Stock relaxes its standstill command the instant that happens, not on any schedule: across 13
stock holds of 4.5 s or longer, the relax and GEAR.BRAKE_HOLD agreed to within +-0.02 s in all 9
where both were visible, and the latch itself landed anywhere from 0.01 s to 7.6 s after
standstill. `ACCEL_HOLD_LATCHED` (-0.001 m/s2, raw -1) is the relaxed value sent once the car has
the brakes. Stock drops CRZ_CTRL.ACC_ACTIVE_2 together with the relax. If the latch never comes
the port simply keeps braking at the plan's value. BRAKE_HOLD is the driver's Auto Hold feature;
with Auto Hold off the latched family below is structurally unreachable, and the CX-9 body never
sets it.

### Release grammar

RESUME_UNLATCHING at the release comes in two families (33-pulse census over the stock corpus,
release grammar scan of 2026-08-27):

- a body-latched hold pulses 6 to 11 wire frames (0.12 to 0.20 s, mode 9) while the command
  ramps off the relaxed -0.001; nothing stock ever pulsed longer than 0.20 s
- a never-latched stop only blips 1 to 6 wire frames (mostly 2 to 3), starting about 3 wire
  frames after the stop bits drop, once the command has relax-jumped into its release band

`RESUME_UNLATCH_LATCHED_T` is 0.18 s, 9 wire frames, the latched-family mode. The never-latched
blip is not emitted: nothing is latched there, so it unlatches nothing. Dropping it was originally
an SCBS workaround from when every pulse the port emitted latched the camera; that is fixed at the
source (see the checksum section), so restoring stock's blip is a free choice now, gated on a drive
rather than on the fault.

Stock never puts STOPPING and RESUME_UNLATCHING on the wire together: its stop bits are already
dropped when the pulse fires, every release. A re-hold while a pulse is still playing therefore
waits the pulse out, and a pulse is never restarted while one is playing (stock pulses exactly
once per release).

### The pulse is immediate

The pulse is the release protocol; the body answers nothing else. Deferring it behind silence
(route 0000011d, 0.3 s) and behind a +0.15 m/s2 nudge (route 0000012c, 2.0 s, three latched
stops) both left GEAR.BRAKE_HOLD untouched for the whole window, and the body then dropped it
within 2 to 3 wire frames of the fallback pulse every time. So a latched release pulses
immediately; waiting only added dead time to every resume.

The body has answered every latched pulse the port has sent: GEAR.BRAKE_HOLD dropped 30 to 51 ms
into all 10 (routes 103, 115, 118, 11d, 12c x3, 132, 139, fe). If it ever misses one, the command
would sit pinned at the relaxed hold under a positive plan until the driver's pedal, because the
carcontroller never lets a latched release climb while the body holds. So a latched release whose
body is still holding `RESUME_REPULSE_T` (1.0 s, about 20x the slowest answer ever seen) after the
pulse gets exactly one more, the same tuple as the first (stop bits down, command at raw -1,
unlatch set), the only shape stock has for a latched release. Stock itself never pulses twice, so
a second retry has no attested shape behind it. The window restarts from zero whenever the body
lets go, so it counts only an unbroken run of the body ignoring us.

### Release debounce

The plan flapping across zero at a held standstill (a lead inches forward and stops) used to fire
a fresh RESUME_UNLATCHING pulse per flap and re-assert the stop bits mid-pulse, a combination
stock never emits. The plan must ask to move for `RELEASE_DEBOUNCE_T` (0.2 s) before the hold
releases. Stock's own releases lag the lead's departure by at least this much: all 23 latched
releases show the lead already opening at >= +0.31 m/s at the pulse, about 0.2 s into a typical
drive-off. The driver's pedal is not debounced; it outranks the hold immediately.

### Gas-pedal release

The plan asking for acceleration releases the hold, and so does the driver's pedal, the way
Toyota's PCM lets the pedal outrank its standstill request. Holding the stop bits against the
throttle until the car physically moved put an out-of-protocol release on the bus, stop bits
dropping at speed with no unlatch pulse (route 0000004d t+210.9). Stock keeps STOPPING strictly
to the final creep: 2,078 rolling STOPPING frames in the corpus, all below 0.55 m/s.

A gas-ended hold emits no pulse: stock's one captured gas-ended hold drops the stop bits with no
pulse at all, the pedal being the resume authority. Suppressing it was also an SCBS workaround
(route 00000103 t+163.8 latched on a gas release), and that half is obsolete now the checksum is
fixed. What is left is one stock capture, so if a gassed-out latched hold is ever slow to let go,
this is the first thing to revisit.

### Release command shape

The release command follows stock's shape, not a slew off the hold value. At a never-latched
release stock relax-jumps the command in one frame from the hold value into a -0.27 to -0.18
start (pulse-frame commands span -0.269 to -0.111 across the whole corpus) and then ramps about
+25 raw per 50 Hz frame straight through the drive-off; a latched release ramps at the same rate
off the relaxed -0.001. Hence `ACCEL_RELEASE_BAND` = -0.26 m/s2 and `ACCEL_RELEASE_RAMP` =
1.25 m/s3.

Slewing up from -1.024 instead kept hold-grade braking on the wire beneath the release pulse
(route 00000053 t+714.8), and pre-ramping toward the plan crossed zero inside the pulse
(route 00000100 t+353). The band between those two edges is what the camera accepts. While the
plan is braking the hold command is the plan's own, but the moment it turns positive the hold
freezes where it is: stock never lets ACCEL_CMD climb while STOPPING is asserted.

A latched release does not start climbing until the body lets go: stock pins the command at
raw -1 until GEAR.BRAKE_HOLD drops in every latched release of the corpus.

`ACCEL_RESUME_PULSE_MAX` (0.25 m/s2) is the ACCEL_CMD ceiling while a latched release's pulse
plays: stock's latched releases peak at +0.24 to +0.25 m/s2 (raw +182 / +195) in the pulse tail,
+0.34 worst case. Never-latched blips would be capped at zero, since stock's command is still
negative in every never-latched pulse frame of the corpus. The floor of -0.001 during the pulse
does real work on a re-hold that lands while the pulse is still playing: the pulse runs out and
the re-hold's braking stays off the pulse frames.

### Breakaway

The ramp hands the command back the moment it catches the plan, which assumes the plan's own
value is enough to get the car rolling. It is not always. On the EPS-swapped CX-9 the plan parked
at +0.42 to +0.47 behind a lead 2.5 m ahead and the car sat dead still for the whole 1.5 s the
command was held there, until the driver used the pedal (route 00000009--ad9e22f986 t+452.9).
The CX-5 releases in the corpus only ever broke away once the command had climbed past +1.0
(route_118 t+581.6, the only clean corpus breakaway: last still frame at +1.17, first rolling
frame at +1.07). It was not the grade: the accelerometer puts that stop at +0.41 deg nose-up,
0.07 m/s2 of gravity, effectively flat (the three stops the driver gassed out of were on up to
+2.4 deg). So the ramp keeps climbing past the plan for as long as the car is still stopped.
LongControl cannot do this itself: Mazda runs the default ki of 0, so its pid state emits the
plan's a_target verbatim with no integrator to wind up against a car that is not moving.

The ceiling (`ACCEL_BREAKAWAY_MAX` = 1.45 m/s2) is stock's own. Over all 31 stock stop-to-go
episodes in the corpus (the whole population; stock MRCC is rarely still engaged at a
standstill), the command on the last still frame, i.e. what the car actually broke away at:

    body-latched  (n=21): min +0.405  p25 +0.810  median +0.958  max +1.425
    never-latched (n=10): min -0.001  p25 +0.213  median +0.665  max +1.416

Stock then carries on to a median +1.38 / max +1.94 through the drive-off. Pulling away from a
stop is a firm request on this car, not a creep: a ceiling below about +1.0 sits under stock's
own median and would leave the CX-9 exactly where it was. +1.45 clears every breakaway stock has
ever needed here. The override still only climbs until the car moves, and one never-latched stock
release moved at -0.001, pure creep. The roughly 0.3 s actuator dead time carries the command
about 0.38 past the value that actually broke the car free before standstill clears, which is
why the cap is what bounds the worst case.

The climb gives up after `ACCEL_BREAKAWAY_T` (3.0 s), so a car held by something we cannot see
(a kerb, a steep grade, a foot on the brake) settles back onto the plan instead of being leaned
on indefinitely.

The climb is also bounded relative to the plan (`ACCEL_BREAKAWAY_OVERSHOOT` = 0.75 m/s2), so a
small plan (a lead 4 m ahead barely rolling) is not turned into a full-authority launch at it.
Our own engaged breakaways (14 across routes 53, 100, 115, 118, 12c, 132, 139, fe, plan vs wire
on the last still frame) break away at a wire command of +0.79 to +1.16 whatever the plan asked,
0.11 to 1.17: route 00000132 t+207 (lead 3.9 m, plan +0.11) moved with the ramp at +1.16, route
0000012c t+461 (plan +0.35) at +1.14, and the plan >= +0.9 releases with the ramp still below the
plan. Corrected for the 0.3 s dead time, the command the body actually answered in the 132 case
was about +0.78, i.e. about +0.67 above its plan; stock's own latched breakaways sit at p25
+0.744 / median +0.958 (34 episodes, re-scanned 2026-09-01). +0.75 keeps the smallest plan seen
reaching stock's p25 (+0.11 to a cap of +0.86) and the CX-9's +0.47 reaching +1.22, while a plan
of +0.11 no longer climbs to +1.45.

What the corpus does not settle is the CX-9 itself: its qlog carries no CAN, so GEAR.BRAKE_HOLD
and the stop bits are unobservable there. A body brake latch invisible to us is still on the
table, and would not be cured by asking harder. An rlog would settle it.

### Tried and rejected: a lead-distance cap on the breakaway

0x364 was occupied at only 4 of 34 stock breakaways (one close lead, 4.6 m, released at +0.455),
too thin to size a second knob the plan already encodes.

### Command slew limits and the stock acceleration envelope

The plan-following command is slew limited, asymmetrically on purpose. The windup limit
(`ACCEL_WINDUP_LIMIT`, 4.0 m/s3) is what keeps the command from dumping the brake in one frame,
the driver-felt problem; a tight winddown limit would delay real braking for no measured
benefit, so `ACCEL_WINDDOWN_LIMIT` is -10.0 m/s3. 4.0 sits above the p99 of the plan's own
up-slew on the reporter's route (p99 +3.2, p99.9 +6.3, max +34 m/s3), so it only clips
state-transition steps. Toyota uses 4.0 both ways. `accel_last` is tracked through overrides too,
so taking control back when the driver lifts off ramps in instead of stepping, and the reported
`actuators.accel` is what went on the wire (clip, hold values, slew and the override zero), not
the plan.

Above zero the command is additionally shaped to what stock MRCC does
(`tools/mazda_long/accel_profile.py`, 158 stock routes, 1298 stock accelerating episodes against
302 of ours). Two things separate the two systems, and neither is the plant: the car answers the
command with a gain of 1.07 and about 0.6 s of lag whoever sends it.

- **Build rate.** Stock raises a positive command by at most +12 raw per 50 Hz frame, 0.6 m/s3,
  in 99.3% of rising frames at every speed above about 5 m/s; the only faster ramp is its 1.25
  m/s3 pull-away from a stop. Our plan reached the same peaks with p90 rising jerk of 1.2 to 1.7
  m/s3, and the driver reads that as aggressive throttle. `ACCEL_BUILD_V` over `ACCEL_BUILD_BP`
  applies that shape to the positive part of the command: 1.25 m/s3 below 3 m/s (stock's own
  pull-away ramp) and 0.8 m/s3 from 6 m/s up, a third quicker than stock on purpose. Stock's
  slowness on a lead pulling away is mostly its dash walk and radar lag, which the plan does
  not have, and 0.8 is the largest rate that still sits inside stock's rising-jerk
  distribution (p99 0.65 to 0.93 by bin); every other openpilot port builds at 2 m/s3 or more.
  It governs only the region above zero: brake release below zero keeps the 4.0 m/s3 windup,
  because stock's own brake release has a p99 tail near 2 m/s3 and holding the brake longer
  than the plan asks is the wrong failure.
- **Lift-off.** The other half of the harshness is set-speed capture: the plan dropped from
  +1.1 to +0.45 in half a second on the reporter's route and the car overshot. Stock lifts the
  throttle at no more than 40 raw per 50 Hz frame, 2.0 m/s3, in 99.98% of falling positive
  frames (p99.9 by bin 1.2 to 2.5 over a 0.2 s window). `ACCEL_LIFT_LIMIT` applies that rate
  while the plan itself is still at or above zero, i.e. throttle modulation. A plan asking for
  brake bypasses it and drops at the winddown limit, so braking is never delayed.
- **Ceiling.** Stock's accelerating command (p99, no lead) peaks at 1.77 m/s2 around 4 m/s and
  falls to 1.45 at 9, 1.05 at 14, 0.83 at 18 and 0.65 to 0.71 from 22 m/s up. Upstream's cruise
  profile sits close to that curve, but the MPC and e2e candidates are only clipped at
  `ACCEL_MAX` (2.0) and our wire reached 2.0 below 5 m/s and 1.06 at 15 to 20 m/s.
  `ACCEL_CEILING_V` over `ACCEL_CEILING_BP` caps the wire at stock's envelope. It is a tuning
  cap inside the panda's +2000 raw safety limit, not a replacement for it.

Replaying all three over the logged plans of our 17 alpha-long routes moves the p90 rising jerk
from 1.0 to 1.7 m/s3 onto the 0.8 line in every bin from 5 m/s up and trims the peaks by 0.05 to
0.1 m/s2; the no-lead median peak is unchanged. The stop-and-go paths are untouched: the breakaway ramp and
the latched release pulse keep their own ceilings, and the golden capture differs only in the
engaged-and-accelerating phases.

Honda Nidec caps accel by speed in its carcontroller the same way (`NIDEC_MAX_ACCEL_V`), Toyota
rate limits the PCM command, and sunnypilot's Hyundai tune runs a jerk-limited integrator in
the carcontroller; shaping at the wire keeps the shared planner byte-identical to upstream.

### The resume button

Under openpilot longitudinal the hold is released in-protocol (stop bits drop, RESUME_UNLATCHING
pulses, the command ramps positive), which is what the car's own MRCC does: across 23 stock
body-latched-hold releases with cruise engaged, 0 put a RES press on the bus and all 23 pulsed
RESUME_UNLATCHING (`tools/mazda_long/scan_stock_release.py`). Toyota, Honda and Hyundai gate
their resume button off `openpilotLongitudinalControl` the same way. Pressing it under op-long
would also put a second writer on CRZ_BTNS at the release, which ICBM owns.

## Advertised lead

The lead we tell the camera about lives in three places: CRZ_CTRL.RADAR_HAS_LEAD,
CRZ_CTRL.RADAR_LEAD_RELATIVE_DISTANCE (stock's 1 to 5 closeness bucket) and the 0x364 track
slot. Stock pairs them absolutely: RADAR_HAS_LEAD = 1 never came with all six slots empty, and
has_lead = 0 always came with phase = 0. The port reads all three off one piece of state.

The state is perception, not control. A stock radar reports its objects ignition to ignition, and
stock shows RADAR_HAS_LEAD = 1 with cruise disengaged in 19.5% of all frames. Tying the
advertisement to engagement made a real car 4.5 m ahead vanish from the bus in one frame when the
driver braked out of a creep (route 0000004d t+212); the camera, still watching the car close in
its own vision, ran its SCBS display for six seconds, a pattern absent from 50 h of stock driving.
So the advertisement updates every control frame, engaged or not, for as long as we stand in for
the radar. For the same reason the panda's 0x364 check is not gated on `controls_allowed`: doing
so silently killed 0x364 at every disengagement while CRZ_CTRL still said has_lead = 1, the exact
track/ctrl disagreement the camera faults on.

A marginal vision lead flickers `leadVisible` faster than any real radar ever would (route
6bb2dc61c4 t+400: 6 toggles in 1.4 s on a 120 m lead), so visibility is adopted only once it has
held steady for `LEAD_DEBOUNCE_T` (0.5 s), the way Hyundai debounces its lead bit for 50 frames.
`leadOne` drops to zero the instant vision loses the lead, well before that debounce expires;
advertising a fabricated stand-in over the gap put a stationary object 10.25 m dead ahead on the
bus at 22 m/s, so the last real measurement is coasted across the gap (propagated by vRel each
frame, since a frozen range with the car moving is content no radar ever emits) and expired when
visibility drops, which bounds the coast to the debounce window.

The phase emitted is 2 while following and 3 near a hold, both in-distribution (3 is stock's
dominant standstill value); no fault has ever keyed on the bucket value, only on the triple
disagreeing.

### Track frame content

`RADAR_STATIC_MSG` and the six empty `RADAR_TRACK_MSGS` are byte-exact captures from a 0x764
radar with no objects in view; only the counter nibble in the last byte changes.
`LEAD_TRACK_TEMPLATE` (the occupied-slot constant bytes) comes from stock drive_0b's latched hold
release, the one stock release with exactly our bus topology (lead in 0x364 alone, five slots
empty). Its status pair byte4/byte5 reads 1c/00 through the whole stop, unlatch and drive-off,
which keeps it clear of the empty-slot signature c0 in byte 5. The measurement fields are zeroed
in the template because `create_lead_track` rewrites DIST_OBJ and RELV_OBJ every frame; byte 2
wanders on a live radar but parks at zero in clean stock releases too (drive_0e), so it stays
fixed. DIST_OBJ and RELV_OBJ share a 0.0625 scale; DIST_OBJ full scale is 255.875 m.

The panda's 0x364 check leaves the DIST_OBJ and RELV_OBJ fields free and requires every bit the
template owns to match. A byte-exact check silently dropped every real-lead frame and starved
the camera of the track (route 6bb2dc61c4: 982 asked, 0 transmitted).

## Radar gap guard on the stock radar interface

When openpilot is not running longitudinal, `radar_interface.py` feeds the stock radar's tracks
to radard. The six tracks 0x361 to 0x366 arrive at 10 Hz on bus 0. 0x361 to 0x364 carry
stationary and moving objects with a reliable RELV_OBJ. 0x365 and 0x366 appear to be
moving-vehicle-only slots whose RELV_OBJ uses a different, undecoded encoding, so they are
excluded: without a valid vRel, radard's Kalman filter and lead matching cannot track them and a
NaN velocity would poison downstream consumers. An empty slot sets all three of DIST_OBJ (raw
4095), ANG_OBJ (raw 2046) and RELV_OBJ (raw -16) to their sentinels; the full triple is required
because each sentinel alone is a reachable real value (relv raw -16 is -1.0 m/s). The measured
distribution is in the radar test file.

The 2016-20 CX-9 is the one Mazda whose radar does not put the 0x361 to 0x366 tracks on bus 0,
so its platform config claims no radar bus; claiming one would leave radard waiting on a parser
that never goes valid.

## Alpha-long availability rule

Alpha long follows the EPS, not the model: it needs an EPS that can hold the wheel through a
stop, so any Mazda carrying the 2022 CX-5 EPS (`MazdaFlags.STEER_TO_ZERO_EPS`) with a radar bus
may try it. A stock older EPS cuts lateral below 45 kph, so stop-and-go would run unsteered. The
CX-5 2022 is the car every engaged-mode constant was measured on. The CX-9 2021 was checked to
share the wire format (route 00000004: identical CRZ_INFO checksum and rates over 54k frames,
radar UDS at 0x764, same FSC camera firmware GSH7-67XK2-U); the other GEN1 platforms share the
DBC and are assumed to. Note that a stock CX-9 2021 carries EPS firmware TC3M-3210X-A-00, which is
not in `STEER_TO_ZERO_EPS_FW`, so under the EPS-keyed rule only a CX-9 with the swapped 2022 CX-5
EPS is offered alpha long (see mazda-fingerprinting.md).

`longitudinalActuatorDelay` is 0.36 s: about 0.3 s dead time plus about 0.3 s first-order lag.

## Cruise button management (ICBM)

The body ECU registers at most about one discrete press per 200 ms; a tighter cadence makes it
drop presses (measured about 0.93 mph per press at 5 Hz vs about 0.47 at 9 Hz, so faster sending
gives a slower dash). CRZ_BTNS runs at 10 Hz on the wire (99 to 101 ms for about 95% of 7.2k
inter-frame gaps, route 0b), with extra event frames only on press edges, and the wheel's CTR
increments by 1 on every frame including those, never repeating a value. A genuinely held button
just keeps its bit set on the regular 10 Hz cadence, so hold frames are paced at `HOLD_PERIOD`
(0.1 s), which mimics a real hold and guarantees the +1 counter offset is unique (a fresh genuine
counter lands between consecutive sends).

ICBM frames are suppressed while cancel or resume are in flight or while the driver is holding
the wheel cancel button: its interleaved cancel = 0 frames otherwise race the driver's cancel = 1
frames and the body ECU drops the cancel intent. CAN_OFF has to raise a cancel button event for
the same reason, or ICBM's readiness gate never learns the driver is cancelling. On the CX-5 2022
the wheel "+" button toggles SET_P, not RES; RES is the resume button (route
0000019c--84a5408a38 seg 2/3: holding "+" emits SET_P = 1 and the body ECU increments CRZ_SPEED).

## Camera alert passthrough

`create_alert_command` passes the camera's own CAM_LANEINFO state through untouched, the way
Toyota's `create_ui_command` preserves every stock signal it does not own; letting the packer zero
ERR_BIT hid camera-asserted error state from the car. The TJA mode fields are the exception:
under openpilot the camera's own TJA/CTS state machine churns against steering it did not
command (TJA_TRANSITION toggled 442 times in 22 min on route 0000010b) and relaying that flapped
the dash lane indicators, so those two stay zeroed.

## Constants

| Constant | Value | Measurement | Routes |
| --- | --- | --- | --- |
| `LONG_STEP` | 2 frames (50 Hz) | stock CRZ_INFO / CRZ_CTRL rate | corpus |
| `RADAR_STEP` | 10 frames (10 Hz) | stock radar static and track rate | corpus |
| `RADAR_UDS_STEP` | 50 frames (2 Hz) | tester present cadence, well inside S3 = 5 s | 000000fe |
| `FSC_SETTLE_T` | 10.0 s | radar-presence check faulted at 1.9 s, passed from 5.8 s | setup/teardown drives |
| `CAM_LANEINFO_PERIOD_T` | 0.563 s | longest CAM_LANEINFO period, 26+ segments, two cars | corpus |
| `CAM_LANEINFO_FRESH_T` | 1.5 s | 2.7x the longest period | derived |
| `STOCK_RADAR_ALIVE_T` | 0.05 s | stock gap p99.99 31.0 ms | 7.25M frames, 166 routes |
| `PANDA_RADAR_SILENT_T` | 1.0 s | `MAZDA_RADAR_SILENT_FRAMES` 50 / 50 Hz PEDALS | derived |
| `STOCK_RADAR_GUARD_MARGIN_T` | 0.2 s | PEDALS jitter over 50 frames plus pipeline latency | derived |
| `STOCK_RADAR_GUARD_T` | 1.27 s | sum above; about 12x the longest stock gap (105.7 ms) | 0000002d seg 28 |
| `RADAR_SESSION_LIMIT_T` | 10.0 s | per-episode UDS budget | design |
| `MAZDA_ENGAGE_BTN_WINDOW` | 10 CRZ_BTNS frames | press 30 to 70 ms before ACC_ACTIVE, 104 engagements | corpus |
| `CANCEL_CONTEXT_T` | 0.5 s | PEDALS lags the CAN_OFF press by a few frames | 7f9e3ff336 |
| `RESUME_UNLATCH_LATCHED_T` | 0.18 s (9 wire frames) | latched pulses 6 to 11 wire frames, mode 9 | 33-pulse census |
| `RESUME_REPULSE_T` | 1.0 s | body answered all 10 pulses in 30 to 51 ms | 103, 115, 118, 11d, 12c, 132, 139, fe |
| `RELEASE_DEBOUNCE_T` | 0.2 s | lead opening >= +0.31 m/s at all 23 stock latched pulses | corpus |
| `LEAD_DEBOUNCE_T` | 0.5 s | 6 leadVisible toggles in 1.4 s on a 120 m lead | 6bb2dc61c4 |
| `ACCEL_HOLD_LATCHED` | -0.001 m/s2 | relax and BRAKE_HOLD within +-0.02 s in 9 of 9 visible | 13 stock holds |
| `ACCEL_RESUME_PULSE_MAX` | 0.25 m/s2 | stock latched pulse tail +0.24 to +0.25, +0.34 worst | corpus |
| `ACCEL_RELEASE_BAND` | -0.26 m/s2 | stock never-latched relax target -0.27 to -0.18 | corpus |
| `ACCEL_RELEASE_RAMP` | 1.25 m/s3 | +25 raw per 50 Hz frame | corpus |
| `ACCEL_BREAKAWAY_MAX` | 1.45 m/s2 | stock last-still-frame command max +1.425 (n=31) | corpus |
| `ACCEL_BREAKAWAY_T` | 3.0 s | give-up bound on an unseen holder | design |
| `ACCEL_BREAKAWAY_OVERSHOOT` | 0.75 m/s2 | 132 case +0.67 above plan; stock p25 +0.744 | 132, 12c, 34 stock episodes |
| `ACCEL_WINDUP_LIMIT` | 4.0 m/s3 | plan up-slew p99 +3.2, p99.9 +6.3; brake region only | reporter's route |
| `ACCEL_BUILD_V` | 1.25 to 0.8 m/s3 over 3 to 6 m/s | stock +12 raw per 50 Hz frame (0.6) in 99.3% of rising frames, taken a third quicker; 1.25 pulling away | 158 stock routes |
| `ACCEL_LIFT_LIMIT` | -2.0 m/s3 | stock throttle lift <= 40 raw per frame in 99.98% of falling positive frames | 158 stock routes |
| `ACCEL_CEILING_V` | 1.5, 1.75, 1.45, 1.05, 0.85, 0.65 m/s2 at 0, 4, 9, 14, 18, 25 m/s | stock accelerating command p99 by speed, no lead | 158 stock routes |
| `ACCEL_WINDDOWN_LIMIT` | -10.0 m/s3 | clips only p99.9+ steps | reporter's route |
| `stopAccel` | -1.024 m/s2 | stock hold raw -1024 | corpus |
| `longitudinalActuatorDelay` | 0.36 s | 0.3 s dead time + 0.3 s lag | corpus |
| `HOLD_PERIOD` | 0.1 s | CRZ_BTNS 10 Hz, 99 to 101 ms for 95% of 7.2k gaps | 0b |
| `MAZDA_LONG_LIMITS` | +2000 / -3500 raw | ISO window; stock reaches raw -3891 | corpus |

## Tried and rejected

- Adopting a quiet radar on the 50 ms alive window: two masters on a stock frame gap, then a
  session request at speed. Adoption waits the full guard.
- Gating the two-master block on availability alone: lateral latched on with no exit
  (route 00000057).
- Raising accFaulted during the boot-phase block: a permanent cruise fault toast on every start.
- BIT2 as a settle input: pinned the timer for a whole ignition on a second CX-5 2022.
- `check_relay` on the replaced radar addresses: relay_malfunction every boot.
- Deferring the latched unlatch pulse behind silence (0000011d) or a positive nudge (0000012c):
  no body response until the pulse.
- A second repulse: stock never pulses twice, no attested shape.
- Emitting a pulse on a gas-pedal release: stock does not, and it once latched the camera
  (00000103 t+163.8, under the old checksum).
- Holding the stop bits against the throttle until motion: out-of-protocol release
  (0000004d t+210.9).
- Slewing the release command up from -1.024 (00000053 t+714.8) or pre-ramping toward the plan
  (00000100 t+353): outside the band the camera accepts.
- A lead-distance cap on the breakaway: 4 of 34 stock breakaways had 0x364 occupied.
- Tying the advertised lead to engagement (0000004d t+212) or advertising a fabricated stand-in
  during a vision gap (10.25 m ghost at 22 m/s).
- A byte-exact panda check on 0x364: dropped every real-lead frame (6bb2dc61c4).
- Reporting `cruiseState.standstill` under op-long: deadlocked every stop.
- Relaying the camera's TJA mode fields: dash indicators flapped (0000010b).
- Pressing RES to release a hold under op-long: stock never does (0 of 23), and it double-writes
  CRZ_BTNS.
