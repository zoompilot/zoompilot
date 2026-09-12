# Mazda lateral on the 2022 EPS: evidence and design notes

This is the measurement record behind the steering side of the Mazda port: the 2022 CX-5 EPS
detection, the 1200/12/12 torque envelope, the speed-dependent STEER_MAX, the EPS ceiling clamp,
the LKAS_BLOCK non-delivery latch and the history of the camera's ERR_BIT_1 fault. The code is in
`opendbc/car/mazda/{values,interface,carcontroller,carstate}.py` and the panda envelope in
`opendbc/safety/modes/mazda.h`. Route ids are the dongle-side segment names in
`tools/mazda_long/test_data/`. 0x241 is STEER_RATE, the EPS's own 100 Hz report, which carries
LKAS_REQUEST (the request it received), LKAS_EFFECTIVE (what it applied) and LKAS_BLOCK.

## 2022 EPS detection and the STEER_TO_ZERO_EPS flag

Older Mazdas are dashcam only for one reason: their EPS locks steering out after about 5 s of
hands-off and below 45 kph (LKAS_LIMITS: disabled going down through 45 kph, re-enabled going up
through 52 kph, and the LKAS_BLOCK bit lags the speed on the way up). That is a property of the
EPS, not of the car. The 2022 CX-5 EPS can steer to zero and has no hands-off lockout, so a car
with that EPS swapped in is controllable and lifts out of dashcam with it.

`MazdaFlags.STEER_TO_ZERO_EPS` is therefore set from the EPS firmware (`STEER_TO_ZERO_EPS_FW`:
KBST-3210X-A-00 and KSD5-3210X-C-00, the CX-5 2022 EPS block in fingerprints.py) or from the
CX-5 2022 platform itself, and it keys the whole 2022 EPS block together: the higher-authority
`CarControllerParams` tune, carstate's fault handling, `minSteerSpeed = 0`, the alpha-long
availability rule (see mazda-longitudinal.md) and the panda's torque envelope through
`MazdaSafetyFlags.STEER_TO_ZERO_EPS`. Without the panda bit the panda enforces upstream's
800/10/25; with it the 1200/12/12 the controller commands.

`steerActuatorDelay` follows the EPS as well, because command-to-torque lag is EPS firmware:
0.14 s on the 2022 EPS, 0.1 s otherwise. lagd learns the rest (0.338 s total on a CX-5 2022;
its initial value is this plus 0.2). The CX-5 2022 `steerRatio` of 18.1 comes from the paramsd
learner over 2.9M samples; 15.5 is the factory spec.

## The torque envelope: 1200 / 12 / 12

### STEER_STEP and the rate limit are per unit time

The stock camera commands CAM_LKAS at 16.6 Hz (60 ms). Openpilot sends at 100 Hz (`STEER_STEP`
= 1). The EPS rate limit is per unit time, about 1200 counts/s, not per received frame: on a
stock drive where the camera commands at 16.6 Hz the EPS still steps 12 counts per 10 ms. So the
cadence buys no extra authority, but `STEER_DELTA_UP` / `STEER_DELTA_DOWN` are per frame, which
makes `STEER_STEP` load-bearing: 12 counts x 100 frames/s = 1200 counts/s matches the EPS
exactly. Changing `STEER_STEP` without rescaling `STEER_DELTA_UP` by the same factor silently cuts
the commanded slew rate (`STEER_STEP` = 6 would give 200 counts/s, 6x slower than the hardware).

### The slew is symmetric

Over 11.7M clean 0x241 frames the delivered step |dLKAS_EFFECTIVE| has p99 and p99.9 of 12 at
every speed, for the stock camera and for openpilot alike, and stays there when the request
jumps 40 to 100 counts in a frame (mean delivery 8.2). On a CX-9 2021 the stock camera's requests
never moved LKAS_EFFECTIVE more than 12 over 62k samples and 750 request-drop events; openpilot
on a CX-5 2022 shows p99.9 of 12 over 318k samples.

A winddown above 12 therefore buys no faster release at the wheel; the EPS still walks at 12. It
only lets the command run ahead of where the wheel actually is (p99 of that gap was 700 to 800
counts below 20 mph, max 1400), so the command can cross zero while the wheel is still turned and
the P term keeps building against a measurement that has not responded yet. Hence
`STEER_DELTA_UP` = `STEER_DELTA_DOWN` = 12 and, in the panda, `max_rate_up` = `max_rate_down` =
12 with `max_rt_delta` = 384.

### The controller's rate-down must equal the panda's

`driver_limit_check` in the panda demands a retreat of at least `max_rate_down` per frame once
the driver bound is below the last command, so a controller retreating by less has every frame
rejected until |cmd| <= `max_rate_up`. With the panda at 25 and the controller at 12, route
00000148 seg 10 lost 171 consecutive frames that way (the first rejection exactly last - 12),
a 1.7 s hole in LKAS delivery, and the camera latched its LKAS fault. A panda `max_rate_down`
above `STEER_DELTA_DOWN` is not a looser backstop; it rejects every frame of a driver-override
winddown. The fix (2026-09-01, opendbc 95d5ff56c4) set both to 12, param-gated on the EPS bit.

### Driver-torque headroom

The command may not exceed `STEER_MAX + (STEER_DRIVER_ALLOWANCE + driver_torque) *
STEER_DRIVER_MULTIPLIER`, with `STEER_DRIVER_MULTIPLIER` = 15 on this EPS (upstream's value is 1).
The panda enforces the same envelope, but from the min/max of its own last 6 samples
(`MAX_SAMPLE_VALS` in safety/declarations.h), while the controller sees one sample that is already
a control cycle old. At a multiplier of 15 one count of driver torque moves the ceiling by 15, so
when the driver fights hard enough to pin the command against it, a few counts of staleness put
every frame over the panda's line: route 00000148 seg 10 lost 171 consecutive frames (t+648.59 to
t+650.31, a 1721 ms hole in LKAS delivery) and the starved EPS dropped out of LKAS entirely.

So the controller bounds the ceiling with the most adverse driver torque in a window that spans
the panda's (`STEER_DRIVER_SAMPLES` = 10), instead of the newest single sample. The panda's max
over its window is at least the oldest sample in it, which this window also contains, so the
controller's ceiling can only sit at or below the panda's. It costs authority solely while the
driver is actively fighting, which is where yielding is the intent anyway. Only the bound on the
side being commanded can bind (`apply_driver_steer_torque_limits` clamps the opposite one to
zero), so the adverse extreme is the low end of the window when pushing positive and the high end
when pushing negative.

The overlap argument runs out once the controller falls a full panda window behind, and a longer
window cannot rescue that: replayed over the two fault routes, 70 ms of staleness breaks the
invariant for every window from 6 to 24 samples. So a small fixed margin in driver-torque counts
is carried as well (`STEER_DRIVER_MARGIN` = 2). Replay says 2 is what it takes: over 36,195
commanded frames, a margin of 0 leaves 7 frames above the panda's ceiling at 50 ms of staleness
and 1 at a margin of 1, while 2 leaves none at either 30 or 50 ms. It costs 30 counts of ceiling,
which widens the frames the ceiling actually trims from 2.1% to 3.0% on those two
driver-fighting routes.

## Speed-dependent STEER_MAX

`STEER_MAX` is the scale from the controller's normalized output to CAN counts
(`new_torque = actuators.torque * steer_max`), not just a ceiling, and latAccelFactor is
proportional to it. Changing it rescales every sub-saturation command and invalidates every
`speed_dependent.toml` LAF seed at once, so it is left alone and the EPS's real ceiling is
enforced separately (next section).

`STEER_MAX_LOOKUP` is 1200 up to 14.2 m/s and 800 from 14.5 m/s (about 32 mph): 1200 below for
full low-speed authority and feedforward overshoot, 800 above for smoother highway steering.
Speed-dependent STEER_MAX is only applied on the 2022 EPS; the pre-2022 path uses upstream's
constant 800.

## EPS ceiling clamp

The torque the EPS will actually apply is a function of speed. Measured over 11,408,748 clean
frames (4798 segments, not LKAS_BLOCK, not steeringPressed, vEgo > 2 m/s) from 0x241
LKAS_EFFECTIVE: above 32.5 mph zero of 7,490,617 frames exceeded 620 counts; below 18 mph none
exceeded 1148. The rail is a function of instantaneous speed with no memory (decel, steady and
accel rails are identical, spread 0, from 32 to 60 mph) and is left/right symmetric. The
derivation is `tools/mazda_long/eps_ceiling_curve.py`.

    speed m/s : 8.0   8.5   9.4   10.3  11.2  12.1  13.0  13.9  14.5
    counts    : 1148  1132  1092  1048  1012  920   808   676   620

Commanding above this delivers no extra torque at the wheel; it only hides actuator saturation
from the controller. controlsd derives `steer_limited_by_safety` from `actuators.torque` vs
`actuatorsOutput.torque`, so without the clamp the request and the report agree at 1.0 while the
EPS sits railed, the integrator never freezes, and it winds up to be paid back as overshoot on
release. The clamp is applied before `apply_driver_steer_torque_limits`, whose driver-torque term
only ever narrows the window further, and the clamped value is what `new_actuators.torque`
reports. It is deliberately separate from `steer_max` for the LAF-seed reason above.

## LKAS_BLOCK and the non-delivery latch

### LKAS_BLOCK is not all-or-nothing on this EPS

On the pre-2022 EPS LKAS_BLOCK means low speed or hands off, and `steerFaultTemporary` follows it
once LKAS has been allowed. On the 2022 EPS the block is graded: above about 4 m/s a blocked EPS
still delivers a third to a half of the request (median LKAS_EFFECTIVE / LKAS_REQUEST 0.35 to
0.45), and under about 4 m/s applying nothing is simply what the car does while it is being
rolled around. LKAS_BLOCK alone cannot tell those apart; LKAS_EFFECTIVE can. Zero delivery
separates cleanly from normal operation: across 96k unblocked frames with |request| > 200 the
longest run of LKAS_EFFECTIVE == 0 is 2 frames, while blocked runs reach 183 frames.

### The latch

The camera latches CAM_LKAS.ERR_BIT_1 ("LKAS Fault: Restart the Car", `steerFaultPermanent`)
when it watches a request go nowhere for long enough. Both captured faults sat at the top of the
non-delivery distribution: routes 00000139 seg 14 and 00000148 seg 10 rank 1 and 2 of 92 segments
by non-delivery budget (1755 and 3552 count-seconds, against zero for 78 of them), so that is the
right quantity to bound even though the camera's exact predicate is still unknown.

`update_steer_undelivered` counts frames where the EPS reports LKAS_EFFECTIVE == 0 against a
request above `STEER_UNDELIVERED_MIN` (200 counts; below that the EPS rounds to zero anyway).
After `STEER_UNDELIVERED_FRAMES` (20 frames, 200 ms, an order of magnitude clear of both the
2-frame benign tail and the 183-frame blocked runs) the carcontroller zeroes the command so the
camera stops accumulating whatever it accumulates. `apply_torque_last` follows the zero, so
delivery coming back ramps from zero at `STEER_DELTA_UP` (about 1 s to full) instead of stepping
into a request the EPS has not seen. That is the cost of the latch, and it is paid in the speed
range where the EPS was applying nothing anyway. Derivation:
`tools/mazda_long/analyze_lkas_nondelivery.py`.

The latch releases on LKAS_BLOCK clearing rather than on delivery returning, because once the
command is zero there is no request left to observe delivery of; keying the exit off our own
output would ramp back into the block and limit-cycle. The block bit is the EPS's own statement
that it is accepting LKAS again, and the entry condition already established that this particular
block is total.

`steeringPressed` does not gate entry. On route 00000148 seg 10 the driver steered with the
request through a whole 203-frame blocked run, so same-direction driver torque never unwound the
command (it walked to saturation) and the run spent the budget the latch was written to protect.
A driver fighting the request unwinds it below `STEER_UNDELIVERED_MIN` through
`apply_driver_steer_torque_limits`, so that case already cannot latch.

### Scope: defense in depth, not the primary protection

In both captures the EPS was applying nothing because it had been starved of 0x243 entirely: the
panda rejected every frame for 1.7 s (route 148, the rate-down mismatch and driver-torque
staleness above) or lateral was never armed panda-side (routes 116/117, the radar-silent guard
ordering in mazda-longitudinal.md). The controller, blind to that, ramped to the rail and held it
into a dead EPS. The starvation is the cause, and as of 2026-09-06 it is known to be sufficient
on its own: the EPS raises `LKAS_FAULT` about 0.6 s into the break and the camera faults 5.3 s
later whatever is requested afterwards (see below). The latch does not prevent that; it bounds
what the EPS is asked for while blocked. The stream itself is now kept alive by the panda's own
rejection report, read in carstate and acted on in the controller.

### Tried and rejected: a per-ignition cumulative budget

An earlier reading of route 139 blamed a per-ignition cumulative budget and normal MADS wind-up.
That is falsified in both directions: route 00000031 reached 6705 non-delivery frames with no
fault, and 148 faulted on roughly a third of 139's spend. Do not restore it.

### Telling the driver

Telling the driver is a slower decision than protecting the camera. The latch fires in 200 ms
because every frame past it spends the camera's budget, but a banner and a chime that fast turn
every rolling manoeuvre into noise. On the 2022 EPS `steerFaultTemporary` is
`steer_undelivered_alert`, which arms only when the latch has held for a further
`STEER_UNDELIVERED_ALERT_FRAMES` (80 frames, one second all-in) and the car is at or above
`STEER_UNDELIVERED_ALERT_MIN_SPEED` (12 mph) and the EPS is not flagging the block as its own
low-speed standby (`LKAS_TRACK_STATE`, below). Upstream turns `steerFaultTemporary` into
"Steering Assist Temporarily Unavailable" as a warning while the driver is on the wheel and a
soft disable hands-off: "TAKE CONTROL IMMEDIATELY", the loud chime, and under MADS the loss of
lateral 3 s later. On this EPS the sunnypilot car-specific hook (`car_specific.py`) swaps the
event for its `steerTempUnavailableSilent` form before the state machine sees it, so the driver
gets the small banner and one prompt chime and keeps whatever control state they were in. The
latch has already zeroed the command, so a soft disable would protect nothing the latch does not;
on the launches that armed it the block released within 0.7 s, well inside the 3 s soft-disable
timer, so all the escalation ever did was shout. The trade is that with ACC engaged the car
keeps its longitudinal through a road-speed steering dropout on the strength of the banner; on
the corpus that case is the three real blocks below, two of them followed by the camera's own
permanent fault. The banner clears with the latch when the block ends.

Below manoeuvring speed nothing is reported: an EPS applying nothing there is normal (77% of all
LKAS_BLOCK duty sits under 2 m/s, and 91.5% of the standstill and creep block under 1 m/s), the
driver's hands are on the wheel, and there is no lane keeping to have lost. This follows Honda,
which suppresses the same `steerFaultTemporary` for an expected LOW_SPEED_LOCKOUT below its own
minimum. `minSteerSpeed` cannot carry the threshold because this EPS has no lockout and it stays
0, so the speed lives in `CarControllerParams`. 12 mph sits inside the 2 to 15 mph range Honda's
per-car minimums span, and it is under the 4.6 m/s where route 148's block began.

The speed gate alone is not enough, and it was first drawn as if it were. A block the EPS carries
from standstill does not release on speed: over 605 launches in the corpus it releases about a
second after the car passes 3 m/s (median 1.09 s, p90 1.71 s), so the release speed is roughly
3 m/s plus a second of whatever the launch is pulling. 12% of launches release above 12 mph
overall, 22% at 1.5 to 2 m/s2 and 57% at 2 to 3 m/s2. Routes 0000014f (segs 14, 16) and 00000150
(seg 9) each armed the alert that way at 5.4 to 5.9 m/s on an ordinary launch, 0.4 to 0.7 s
before the block released at 6.3 to 6.5 m/s, with the driver on the gas and not one rejected
0x243 frame. The earlier "release band max 5.39 m/s" was a 38-episode subset.

The EPS's own `LKAS_TRACK_STATE` separates most of them. It is set through a block that began
at standstill (the low-speed standby; `TRACK_STATE = 1` implies `LKAS_BLOCK = 1` over 16k frames
with no exception) and clear for a block that began at speed, which is an EPS that dropped LKAS
mid-delivery: route 148's fault block (began at 4.6 m/s, `TRACK_STATE = 0` throughout), route
139's (set on its first frame only), the permanent fault on 00000013 seg 0 and an EPS dropout at
17 m/s on 00000102 seg 0. The alert therefore requires `TRACK_STATE` clear, read at arming like
the speed. The latch itself is unchanged: the command is still zeroed through a launch block,
only the banner is withheld.

`TRACK_STATE` is not enough on its own, and the first replay that said it was (51 firings over
3980 segments, 48 launches all with the bit set throughout) was wrong. Replaying the shipped
state machine itself over every captured drive (`tools/mazda_long/replay_undelivered_alert.py`,
4005 CX-5 2022 segments, 64 h, the mirror checked frame for frame against `carstate.py`) gives
12 armings with the `TRACK_STATE` gate: the 3 real blocks above and 9 more that are not. Those
nine all began at 0.0 to 0.1 m/s and lasted 13 to 44 s while the car crawled at up to 5.5 to
7.2 m/s, a standby the EPS carried from a stop through slow traffic, and in each the bit
cleared with the block still on. None has a rejected 0x243 frame or a camera fault anywhere near
it. What separates them from the three real blocks without exception is where the block began:
the real ones at 4.6, 17.3 and 20.3 m/s, the standbys at a stop. Of the 1915 `LKAS_BLOCK`
episodes in the corpus, 1660 begin below 0.5 m/s (a stop, read through wheel-speed
quantisation), 57 between 0.5 and 3 m/s (none of which ever latched) and every latched block
that began above 3 m/s and reached the alert was a fault. The alert therefore also requires the
block to have begun at or above `STEER_UNDELIVERED_ALERT_ORIGIN_SPEED` (1 m/s), the speed read
on the block's first frame and held until it releases. With it, the corpus arms 3 times in 64 h,
each one a real dropout, and the same 3 whether the hold is 1, 2 or 3 s or the `TRACK_STATE`
gate is kept or dropped (it is kept). Longer holds or higher speed gates alone do not get there:
3 s still lets one crawl through and 20 mph loses route 148. The cost is a block that began at a
stop and then really did die mid-drive, which stays a silent latch until the camera's ERR_BIT_1
reports it.

Above that speed a total block is already abnormal, so the hold only has to clear the transients:
over the corpus, non-delivery runs above 10 mph reach 30 frames (0.35 s) on every route that never
faulted, against 315 frames (3.71 s) on the two that did. One second all-in sits about 3x clear
of the benign tail and about 4x inside the fault, the same separation the 20-frame latch is drawn
from. The speed is read once, when the alert arms, so a car accelerating out of a block that
never lets go still gets told (route 00000148 crossed 5.9 to 7.6 m/s with zero delivery), while
one hovering at the threshold does not flicker.

## LKAS_FAULT and the camera's ERR_BIT_1

The camera's ERR_BIT_1 ("LKAS Fault: Restart the Car", the cluster's "Front Camera Sensor System
Malfunction" once the HUD frame forwards ERR_BIT) follows a bit the EPS raises first:
`STEER_RATE.LKAS_FAULT`, byte 6 bit 5, decoded since 2026-09-06. Over 64 h and 16 camera faults
it is set before 15 of them, the camera follows 5.25 to 5.55 s after every known onset, and it
never clears (57 runs). The camera is reacting to the EPS, not to our request: `000001bb` had
commanded zero for 5.25 s when it faulted, and the March lateral-only drive_02 faulted at 16 m/s
on zero requests. The bit is decoded for the log and tooling; the driver-facing fault stays the
camera's own.

What raises it, on the eleven onsets captured: a break in the 0x243 stream of about 0.6 s (ten,
counting three comma restarts), and twice a ramping request into a zero-delivery standby at a
crawl from a stop, which 406 near-identical stretches in the corpus did not reproduce and which
stays open. The breaks were all panda rejection bursts (routes 00000116/00000117: MADS armed
before the panda's radar-silent guard; 00000139 and 00000148: the 25-versus-12 rate-down and
driver-torque staleness; drive_02 and 00000013 in March and July: commands past the 800-count
limit of the time). Each cause was fixed in turn, but the mechanism behind all of them is the
panda's own: a rejection resets its rate-limit reference to zero, so a controller that keeps
ramping is rejected on every later frame. The panda reports the fact directly: every transmit
its tx hook refuses goes back on the `can` stream with `src = bus + 0xC0` (192 for bus 0), and a
third parser in `get_can_parsers` (`Bus.loopback`, `CAM_LKAS` at nan frequency so it never
touches `canValid`) counts our refused torque requests each cycle as `lkas_rejected`. A nonzero
count restarts the controller's ramp from zero, which is the one command the panda accepts after
a reset; the report lands one or two card cycles behind the refusal. Refused zero-torque frames
are not counted: the tx hook refuses every LKA frame while the panda is not controlling, and a
zero the controller already sent carries nothing it needs. The closed-loop test in
`opendbc/safety/tests/test_mazda.py` runs the real controller through the compiled safety model
with the report delayed one, two and three cycles and bounds the outage at the driver-sample
staleness plus that delay (route 148's stale sample) and at exactly the delay for a lone
reference reset, against the EPS's 60 frames; the same scenario starves the EPS for the rest of
the run without the report. Measurements, the state table for byte 6 and the
open items: [mazda-camera-fault-2026-09-06.md](mazda-camera-fault-2026-09-06.md).

## The camera's own TJA/CTS state

Some gen1 trims carry a physical TJA button on the wheel, CRZ_BTNS bit 11 (byte 1, bit 3), and
the camera runs its own lane-centering state machine behind it: 0x440 `TJA` reads 0 off, 2
armed, 3/4/5 active, and 0x243 from the camera carries a real torque request whenever it is
active. Upstream never sees any of this: it statically blocks the camera's 0x243/0x440 for the
life of the safety mode, so stock LKAS, TJA and CTS are dead with a panda fitted and the dash
only ever shows openpilot's HUD frame. We pass the camera through instead (`mazda_fwd_hook`),
so disengaged driving is stock, and that makes the handover matter.

Ownership rule as of 2026-09-08: the camera owns 0x243/0x440 whenever openpilot is not steering.
`mazda_openpilot_controlling` is `controls_allowed_lateral`, or `controls_allowed` only when the
panda's MADS is off (then lateral follows cruise). Before that it was either axis, and under
stock long cruise alone claimed the addresses: route 00000018--5655da2c1c seg 15 had MADS off,
MRCC on, the camera's CTS active and requesting up to 640 counts, our idle zeros at the EPS and
no icon on the dash, for 35 s at 27 to 40 mph.

What the logs show when the camera is armed while openpilot steers (routes
f0ffadc70bb6477d/0000007b, 06d95d85cae87470/00000018): the camera requests torque on most
frames, the panda drops every one, the EPS executes ours, and the camera's byte 3 churns
(`TJA_TRANSITION`, dropped 4 to 3 to 2 around lane loss and overrides). No fault bit, no nag,
no rejection. The actuation is safe; the risks are the camera's internal state and stock CTS
taking the wheel the instant MADS lateral drops with cruise off. One owner reports the cluster's
camera warning at that cadence; it is not in the CAN log.

### The button on this car (route 00000018 seg 12, frame level)

A TJA press with MRCC off arms MRCC (CRZ_AVAILABLE 0 to 1, PEDALS ACC_OFF 1) 80 ms later and the
camera reads TJA 2. A press with MRCC armed only toggles the camera (seg 11: 2 to 0, MRCC stays
armed). The wheel's MRCC master is CRZ_BTNS `BIT1`, active low (0 = pressed, seg 12 t=21.88): it
disarms MRCC and the camera's TJA drops to 0 with it. The camera's lane-centering state is a
sub-state of MRCC armed. On tja_cts_route_29 a press is 2 or 3 wire frames with bit 11 set, the
camera's 0x440 `TJA` changing 21 ms after the FIRST set frame, from any state: it acts on the
press edge and toggles.

### The TJA button as the MADS switch, second attempt (built 2026-09-09, on-car pending)

The first attempt (2026-09-03 to 09-08) failed because every press also toggled the camera, so
MADS and the camera's TJA were in phase or out of phase by luck. The fix is to own the camera's
state ourselves: whenever we steer, the camera is pressed off on its own bus. Then the button has
one meaning, it alternates the two lane-centering systems:

| Press from | Result |
| --- | --- |
| MADS off, camera off (or armed) | MADS on; the press arms the camera (and MRCC, stock); we press the camera off within 1 s |
| MADS on, camera off | MADS off; the press arms the camera; we press it off within 1 s, so nobody steers until the driver presses again |
| any, MRCC button (BIT1 low) | stock: MRCC and the camera both off; MADS untouched |

The second row changed on 2026-09-09 after a user report on the first build: TJA press, MRCC
button, TJA press left MADS off with the camera armed, and stock TJA engaged a few seconds later
because the camera owns 0x243 when we do not steer. The camera is never needed with a panda
fitted, so the press is no longer gated on lateral: any time 0x440 `TJA` reads nonzero the
camera is pressed off, steering or not. The panda accepts the byte-exact bus-2 frame in every
state (it only reaches the camera and can only toggle its lane centering). The stockLkas
warning still fires only while openpilot steers.

MRCC becoming armed on the first press is the button's stock behaviour and is left alone. It is
armed, not engaged; SET still has to be pressed. No MRCC-off spoof on bus 0.

**1. Camera press on bus 2** (`CarController.update_camera_tja`, opendbc b697d69be6). Carstate
reads `stock_tja` live off the parsed 0x440 (0 when the camera is stale, never latched: the
camera drops its own arm, seg 9 +470 ms). Whenever `stock_tja != 0`, ONE CRZ_BTNS
on bus 2 with `TJA_BUTTON` set, `CTR = crz_btns_counter + 1`, every other byte the wheel's idle
pattern (`00 09 ff Cx 00 00 00 00`), `create_button_cmd(bus=2, Buttons.TJA)`. The forwarded
real stream supplies the release. `TJA_PRESS_INTERVAL_T` 1 s between presses (0x440 period
0.563 s plus parser latency), `TJA_PRESS_MAX` 3 per arming episode, reset when the field reads
0, so a driver re-arming it under us is handled again. Not gated on the declaration: any Mazda
that ends up steering with the camera armed (route 7b, feature off) gets the same press.
Panda: `{MAZDA_CRZ_BTNS, MAZDA_CAM, 8, .check_relay = false}` in both TX lists (skipped by the
relay watchdog and static blocking, so the wheel's 0x09d keeps forwarding); the tx hook accepts
the bus-2 frame only while `mazda_openpilot_controlling()` and byte-exact
(`mazda_cam_tja_press_msg_valid`: bit 11 plus the idle pattern, any counter, no other button);
bit 11 on bus 0 is refused outright. Fwd hook untouched.

**2. If the camera does not clear.** One interval after the third press with `stock_tja` still
nonzero: keep steering (the panda blocks the camera, actuation is safe) and set
`CS.stock_cts_stuck` once per episode. Carstate consumes it into a `STOCK_CTS_ALERT_T` (0.1 s)
`stockLkas` pulse and the Mazda hook in `car_specific.py` swaps upstream's no-entry for
`mazdaStockCtsActive`, a 4 s WARNING ("Stock CTS Is Still On / Press the TJA button to switch
it off"), no NO_ENTRY, no disable. It fires only on this failure, never on a healthy
engagement, which was the 09-08 objection to the permanent banner.

**3. Declaration** (restored: main 10c7367925 + d2242a1365 via the revert of 43223ce41b,
opendbc caf74ae8bb + 68bf13d185 by hand). `MazdaTjaButton` toggle, `MazdaFlagsSP.TJA_BUTTON`,
the safety param bit, `mazda_tja_button` driving `mads_button_press` from bit 11,
`button_owns_lateral` in mads.py (`allow_always`, `no_main_cruise`, UEM blocked), carstate's
lkas button event. The MRCC-main guard is KEPT in the panda, against the earlier draft of this
plan: `acc_main_on` is never written when declared. The panda's MADS
(`mads.h m_update_control_state`) exits lateral on the acc_main falling edge unconditionally,
so with MRCC still on the main edge the MRCC master button would kill the panda's lateral while
the software's MADS stayed on (a controls mismatch, our torque refused), and the table's third
row would be false. Undeclared cars stay byte-identical (`TestMazdaTjaMads`, the mads.py A/B
tests).

**4. White HUD.** Not built (2026-09-09 decision). Our 0x440 keeps TJA 0. If revisited: pack
`TJA = 2` under `mads.enabled`, `stock_tja == 0`, no visual alert, never 3 to 5, and bench
first that TJA 2 from us with MRCC off does not arm MRCC (chinna244's finding that the body
consumes the field).

**5. Handover checks on the car.** MADS off by the button with cruise engaged at speed: our
0x243 is vetoed the frame lateral falls and the camera's flows; measure the EPS echo across the
switch for a torque step (the camera starts from its own ramp). MADS on with the camera armed:
up to 1 s of blocked camera requests before the press lands; nothing reaches the EPS but ours.

**Order of work.** Bench on the CX-9 (engine running, panda controlling): (a) one bus-2 frame
flips 0x440 `TJA`, wheel-side counter untouched; (b) the duplicated counter (ours at n+1, the
next real frame at n+1) is accepted, else two frames 30 to 70 ms apart at n+1, n+2; (c) toggling
active CTS at speed shows nothing on the cluster. Then drive with the camera armed at start:
`stock_tja` reads 0 within 1 s of every engagement, `mazdaStockCtsActive` never shows. The
declaration is a toggle, default off, so it lands with the press; declare it only after the
press is proven. Fallback if the camera ignores a panda-originated frame: 2 becomes a NO_ENTRY
and the toggle stays off.

Tests: `[0x09d, 2]` in `TX_MSGS`; `test_cam_tja_press` over the eight MADS/cruise/lateral states
of `test_stock_passthrough`, exact payload accepted only while steering, every other bus-2
button bit refused in every state; `test_tja_button_never_pressed_on_the_car_side`;
`TestMazdaTjaMads` (both main edges guarded when declared, incl. the PEDALS path under alpha
long); carstate `TestStockTja` (live, stale, the one-shot pulse) and `TestTjaButtonEvents`;
`test_mazda_tja_press.py` (press cadence, cap, reset, a lateral pause not resetting, alpha
long); `test_camera_tja_press_bytes`; the golden tx unchanged; mads `TestMadsTjaButton`;
`TestMazdaStockCtsEvents` (WARNING only).

## Constants

| Constant | Value | Measurement | Routes |
| --- | --- | --- | --- |
| `STEER_STEP` | 1 (100 Hz) | EPS slews 12 counts per 10 ms whatever the command cadence | stock camera drive census |
| `STEER_MAX` | 1200 | scale of normalized torque to counts; LAF seeds depend on it | design |
| `STEER_MAX_LOOKUP` | 1200 to 14.2 m/s, 800 from 14.5 m/s | low-speed authority vs highway smoothness | on-car tuning |
| `STEER_DELTA_UP` / `STEER_DELTA_DOWN` | 12 / 12 | delivered step p99 and p99.9 of 12, 11.7M frames | corpus; CX-9 2021 62k; CX-5 2022 318k |
| `max_rt_delta` (panda) | 384 | 32 x max_rate_up, upstream's ratio | derived |
| `STEER_DRIVER_MULTIPLIER` | 15 | tuned for the 2022 EPS (upstream 1) | on-car tuning |
| `STEER_DRIVER_ALLOWANCE` | 15 | upstream | upstream |
| `STEER_DRIVER_SAMPLES` | 10 | must span the panda's 6-sample window plus one control cycle | 00000148 replay |
| `STEER_DRIVER_MARGIN` | 2 counts | 0 frames over the panda ceiling at 30 or 50 ms staleness, 36,195 frames | 00000139, 00000148 |
| `EPS_CEILING_LOOKUP` | 1148 at 8.0 m/s to 620 at 14.5 m/s | 11,408,748 frames, 4798 segments; 0 of 7,490,617 above 620 over 32.5 mph | corpus |
| `STEER_UNDELIVERED_MIN` | 200 counts | EPS rounds smaller requests to zero | corpus |
| `STEER_UNDELIVERED_FRAMES` | 20 (200 ms) | benign zero-delivery runs max 2 frames, blocked runs 183 | 96k unblocked frames |
| `STEER_UNDELIVERED_ALERT_FRAMES` | 80 (0.8 s on top of the latch) | benign runs above 10 mph max 30 frames, faults 315 | corpus |
| `STEER_UNDELIVERED_ALERT_MIN_SPEED` | 12 mph (5.36 m/s) | block release median 3.13, p90 4.97, max 5.39 m/s (38 episodes); fault began at 5.9 | 00000148 |
| `STEER_UNDELIVERED_ALERT_ORIGIN_SPEED` | 1.0 m/s | 1660 of 1915 blocks begin below 0.5 m/s; every latched block that began above and armed was a fault, slowest 4.6 | corpus, replay_undelivered_alert.py |
| `steerActuatorDelay` | 0.14 s (2022 EPS) / 0.1 s | lagd 0.338 total on a CX-5 2022 | corpus |
| `steerRatio` (CX-5 2022) | 18.1 | paramsd learner, 2.9M samples | corpus |
| `LKAS_LIMITS.DISABLE_SPEED` / `ENABLE_SPEED` | 45 / 52 kph | pre-2022 EPS lockout hysteresis | upstream |

## Tried and rejected

- A winddown faster than 12 counts per frame: the EPS still walks at 12, the command just runs
  ahead of the wheel (gap p99 700 to 800 counts below 20 mph, max 1400).
- A panda `max_rate_down` looser than the controller's: rejects every frame of a driver-override
  winddown (route 00000148).
- Scaling `STEER_MAX` down at speed to model the EPS ceiling: rescales every sub-saturation
  command and invalidates the LAF seeds; the ceiling is a separate clamp instead.
- Gating non-delivery on the LKAS_BLOCK bit alone: throws away the third to a half the EPS still
  delivers above 4 m/s.
- Releasing the latch on delivery returning: limit-cycles, because a zeroed command has no
  delivery to observe.
- Gating latch entry on `steeringPressed`: route 00000148 seg 10 spent 203 frames blocked with
  the driver steering along with the request.
- A per-ignition cumulative non-delivery budget: falsified by route 00000031 (6705 frames, no
  fault) and route 148 (faulted on a third of 139's spend).
- Alerting the moment the latch fires: every rolling manoeuvre becomes a chime.
- The physical TJA button as the MADS lateral switch without owning the camera's state
  (`MazdaTjaButton`, first attempt, 2026-09-03 to 09-08): the same CRZ_BTNS frame reached the
  camera, so every press toggled our MADS and the camera's TJA together and their parity was
  luck (route 00000018: in phase on segs 7-9 and 15, out of phase after seg 12). A forwarded
  frame cannot be rewritten and dropping it opens a counter gap the camera validates. Rebuilt
  2026-09-09 on top of the bus-2 camera press (above).
- Keeping `acc_main_on` on MRCC with the button declared (the 09-09 draft plan): the panda's
  MADS exits lateral on the acc_main falling edge, so the MRCC master button would have killed
  the panda's lateral under a still-engaged software MADS. The guard stays.
- Inferring a panda rejection from the EPS's echo of the last request (`STEER_RATE.LKAS_REQUEST`
  matching none of the recent commands, shipped 2026-09-06 as `recover_from_rejection`): correct
  at the logged timing but latency-bound. The echo lands 10 ms after the command at the median
  and 20 ms at p90; a four-command history misfired 158 times over the 64 h corpus with 20 ms of
  extra latency and 25,000 times with 30 ms, each a drop to zero and a re-ramp. It also assumed
  every Mazda EPS echoes the request, unverified on the legacy units. The panda's own rejection
  report (above) is ground truth with no history window and no EPS assumption.
