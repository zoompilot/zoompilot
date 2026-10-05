# Lateral tune: v2 torque controller, shared layer and speed-bin learner

This is where the measurements live. The source files carry one to three lines of what and
why per mechanism; every number, route id, attribution study and rejected alternative that
justified them is here. Routes are the CX-5 2022 test car's unless stated. The acceptance
plan for the next on-car pass is in `lateral-tune-roadmap.md`.

Files: `openpilot/sunnypilot/selfdrive/controls/lib/latcontrol_torque_v2.py` (the tune),
`latcontrol_torque_ext.py` and `latcontrol_torque_ext_override.py` (the shared extension:
EPS rail, speed-dependent torque), `steer_limit.py` (the classifier),
`controlsd_ext.py` (wiring), `openpilot/sunnypilot/selfdrive/locationd/torqued_ext.py` (the
speed-bin learner and its cache), `openpilot/sunnypilot/selfdrive/car/interfaces.py` (the
Mazda seed).

## Lineage

| version | what it is | selected by |
|---|---|---|
| v0 | sunnypilot's `latcontrol_torque_v0.py`: setpoint == the live request, error corrected in lateral-accel space, the extension owning the feedforward params. Byte-identical to sunnypilot's; the only change it sees is the corrected `steer_limited_by_safety` flag from the classifier. | `TorqueControlTune = 0.0`, and any torque car with Enforce Torque Control off (`torque_tune.resolved_tune_versions`) |
| v1 | sunnypilot's current `LatControlTorque` (the `lac` controlsd built), untouched | `TorqueControlTune = 1.0` |
| v2 | v0 plus the four mechanisms below | `TorqueControlTune = 2.0`; seeded on steer-to-zero Mazdas by `_seed_mazda_torque_defaults` (`MAZDA_STEER_TO_ZERO_TORQUE_TUNE = 2.0`) |

`TorqueControlTune` is the small-model tune, declared default v0 (v2 on the seeded Mazdas).
`TorqueControlTuneBig` picks the tune for a big model (chestnut or jetlink), declared default
v1 for every brand (2026-09-11: v1 drove the big models better, v2 the small ones); with
Enforce Torque Control off both run v0. `controlsd_ext.initialize_lateral_control` builds one
controller per size at startup and `select_lateral_control` swaps `self.LaC` at the end of
any frame whose `modelV2.big` differs from the running controller's, resetting the incoming
one. That is safe by construction: a promotion only happens disengaged, where controlsd resets
the controller every frame anyway, and a demotion arrives with a soft disable latched until
disengagement. The per-frame `lateralControlState.torqueState.version` records which tune ran.

v2 was rewritten on the v0 base on 2026-09-01 (commit `ede6d8bb81`) after a leave-one-out
open-loop replay of the previous v2 (routes 132 and 139 = v2 with KD, 12d and 12f = the same
build without KD, 123 to 126 = the 08-29 v2, 125 = v0). The replay attributed each felt
improvement to a mechanism; everything the study could not tie to one was dropped. The
platform-layer bugs the old tune had been compensating for (the raw steer-limit flag, the
EPS rail, friction across the STEER_MAX cliff) were fixed in the shared layer first, so v0
and v1 benefit too.

Replay harness caveat: the fit only reproduces the logged v2 (0.030 RMS) with lagd's
`lat_delay` (0.34 s, not CP's 0.14) and `steer_limited_by_safety` reconstructed from the
previous frame's `carControl` and `carOutput` torques (51% duty). Earlier replay numbers in
older notes used CP delay and no flag.

## The four v2 mechanisms

### 1. Filtered-jerk friction input with a center deadzone

The friction term sees the request differencer through a 1.2 Hz first-order low-pass
(`LP_FILTER_CUTOFF_HZ`, shared with v0's measurement-rate filter), clipped at
`MAX_FRICTION_JERK = 2.5 m/s^3`, with a small-signal deadzone inside |setpoint| < 0.35 m/s^2.
The PID error is untouched.

Attribution: on the old v2 the shaping was the whole -19% of sub-1 Hz output motion at lane
center vs v0, most of it through a setpoint lead that was removed as a turn-in lag. On the
rewritten structure (WP5 gate, routes 139 + 132 + 12d + 12f) it is -12% on straights
(hf_rms 0.0372 -> 0.0327). The deadzone is the larger part (0.0354 without it); the filter
adds 0.0329 -> 0.0327 and trims the release windows (p50 0.126 -> 0.099). The deadzone
speed curve is about 20% of the straight-line gain and is zero in a real turn.

### 2. Hand-back: release integrator decay and error ramp-in

At the `steeringPressed` falling edge `pid.i *= STEER_RELEASE_I_DECAY` once, and the PID
error ramps in over `RELEASE_ERROR_RAMP_T = 0.3 s`. The feedforward is not ramped, so a curve
hold is immediate.

Attribution: release-edge autopsy on the same routes found the tracking error the driver
leaves at release is p90 0.67 m/s^2 and P landed all of it within one frame (P swing p90
2.24 within 100 ms), which the user felt as "better but abrupt at first". Removing the release
handling is the largest release-window change of any mechanism (max delta 0.19 / 0.88 p50 /
p90). The ramp cut replay release slew p90 from 29.4 to 14.7 /s.

### 3. Low-speed D term, gated while pressed

`KD_SCHEDULE = [7.5, 10, 12, 14.5] m/s -> [1.65, 1.05, 0.85, 0.0]`, i.e. `kd = 0.3 s * KP(v)`,
capped below 7.5 m/s and zero by 14.5 m/s. The D input is `-measurement_rate` (v0 already
feeds it; v0 keeps KD = 0) and is zeroed while `steeringPressed`, because the measured rate
is then the driver's.

Mechanism (route 12e, lateral maneuver mode at 9 m/s, 2026-08-30): every 0.5 m/s^2 step
overshoots 35 to 100% (peak 1.35 to 2.04 of target, t90 about 0.85 s) with the integrator
frozen near zero. Not windup: the EPS slews apply torque at 12 counts per frame (rail to rail
about 2 s), the command rails while apply walks about 1 s behind, and when the measurement
crosses the setpoint the EPS still carries about 0.5 units of stale torque that bleeds at the
same slew. The counter-rail correction that follows is the felt swing-back after sharp
turns. Plant fit at 9 m/s: first order, K 2.56 lat accel per unit apply, tau 0.86 s, delay
150 ms (step regime only; it fails the 0.5 Hz sine gain check, 0.28 vs 0.68). Closed-loop
sim with kd = 1.3 at 9 m/s: entry overshoot 1.63 -> 1.21, reversal 1.42 -> 1.09, rise
unchanged; the command leaves the rail a median 0.13 s earlier on the logged steps.

Attribution: removing KD returns about 85% of the low-speed exit counter-swing toward v0
(applied 0.313 -> 0.415, v0 0.433); the logged KD vs no-KD pair (132/139 vs 12d/12f) halves
the exit-tail tracking error (0.188 vs 0.489 RMS). Replay A/B: p50 delta 0.02 to 0.035 below
11 m/s, p95 0.44 to 0.57 (corner transients only), <= 0.004 above the fade. Below 7.5 m/s a
naive 0.3 * KP tracks KP to 250 and amplifies noise at parking speeds; above 14.5 m/s highway
transients would see d > p. The pressed gate was a P1 in the 2026-09-01 review: KD on the
driver's own wheel motion is on the EPS starvation path.

### 4. Curvature request buffer and inactive priming

The delayed-request buffer stores curvature and is rescaled by the live v^2 on read. v0's
buffered lateral accel keeps the old speed's v^2 and reads as phantom jerk whenever speed
changes inside the delay window; in v2 that would reach the friction input (measured at
decelerating low-speed exits: applied counter-swing +0.01 to 0.05 without it). While
inactive, the buffer and both rate filters track the live command, so a re-engage with a
wound wheel does not read the hold as jerk or spike the D term on the first frame. The
integrator is not cleared on inactive frames: MADS cycles lateral often, and the release
decay covers hand-back.

### Invariants pinned by tests

`controls/tests/test_latcontrol_torque_v2.py`: with KD = 0, the deadzone zeroed and the jerk
filter bypassed, v2 equals v0 frame for frame on a moving request and a moving measurement,
friction included; the KP schedule is v0's at every speed; the extension's output overrides
(jerk-aware, NNLC) are disabled regardless of params.

## What left v2, and why

| dropped | reason |
|---|---|
| plan-secant setpoint jerk, divergence blend, stale-model fade, lead speed fade | no benefit in any regime; the plan secant is a 4 to 9 frame lag at turn-in and adds 2 reversals/s on straights |
| unwind freeze, 0.3 m/s^2 low-speed integrator threshold, roll/offset fade | inert (<= 0.002 output RMS) |
| rail PID limits inside the tune, `_rail_limit_scale`, rail-aware saturation block | inert on applied torque once the integrator is clean; moved to the shared layer as `steer_max` |
| directional integrator freeze inside the tune (`_integrator_deepened_while_limited`) | generalized into the classifier so v0 and v1 get it |
| error boost (2026-08-29) | a disguised KP + KI raise (+21%) that railed the 25 to 32 mph EPS ceiling; reverted |
| setpoint rail clamp / reference governor | rejected by replay before implementation: in every high-rail corner window (12a, 12c, 126; 58 to 100% rail duty) the setpoint exceeds the deliverable bound on 0% of frames. The railed-frame error (mean 0.45 to 0.72) is genuine plant shortfall |
| rail lead taper (`RAIL_LEAD_TAPER_MARGIN`) | flapped at the rail edge |
| budget clip | worse p99 |
| command clamp to apply | a no-op in sim: apply's rate limiter makes its trajectory identical however far the command leads |

## Shared layer

### Steer-limit classifier (`steer_limit.py`)

controlsd sets `steer_limited_by_safety` when |CC.actuators.torque - carOutput.torque| >
0.01. One carcontroller slew step is 12/1200 = 0.010 of scale on the CX-5 (0.015 on the
800-count scale the logged drive still ran above 32 mph), so any command walking faster than
the rate limit reads as limited: 51% of active frames on a logged v2 drive, and the bidirectional freeze blocked integrator
decay toward a reversing error on about 13% of frames. That standing bias is where v0's
|i| p50 of 0.275 came from (v2 0.016). An EPS pinned at its ceiling also never raised the
saturation alert, because the ceiling clamp itself kept the flag high.

`classify()` splits the mismatch into rate-limited (moved toward the command by
`RATE_STEP_FRACTION = 0.9` of a full step, or the remaining gap is no wider than the move,
i.e. a command walking slower than the rate limit seen one carOutput frame late), at the EPS
rail (`|applied| >= rail_scale - RAIL_EPS`), and driver-limited (neither). The tunes receive

    limited = mismatch and not at_rail and deepening,  deepening = error_prev * pid.i >= 0

Rate-limited frames stay frozen on purpose. WP2 originally dropped them from the flag;
that wound the integrator up against the EPS slew lag, because while the command outruns
the slew the plant is not following it and integrating that error is actuator-rate windup,
which is what upstream's flag exists for. The freeze is directional so decay stays live, and
the rail is carved out because the PID limits already sit on it (below) and a False flag
lets the tune's own saturation test raise the alert. The 0.9 step fraction covers a
speed-dependent scale (Rivian's) that the carcontroller reads at `vEgoRaw` while the
classifier interpolates at `vEgo`; on a flat scale a full step is exact.
`error_prev` is the previous frame's `pid_log.error`; the one-frame lag is accepted at 100 Hz.

Caveat: a driver-limited frame with a decaying integrator also hands False to the alert
path; the alert still needs the output on the PID limit for `steerLimitTimer` seconds with
the wheel untouched. Blind spot: a panda-rejected frame is reported as delivered in
carOutput, so a starved EPS looks clean here; the carcontroller's non-delivery latch and
`tools/mazda_long/lkas_starvation_check.py` cover that path (route 148, `src == 192`).

Replay A/B of the directional freeze (routes 132/139): corner |i| falls 4 to 6x,
stale-integrator-vs-error frames 59% -> 5%, open-loop output delta < 0.014.

### EPS rail via `steer_max` (`latcontrol_torque_ext.py`)

`get_steer_rail_schedule(CP)` gives EPS_CEILING / STEER_MAX: on the CX-5 the rail falls
monotonically from 1148/1200 = 0.96 below 8 m/s to 620/1200 = 0.52 from 14.5 m/s up. The
extension writes it to
the host tune as `steer_max` in `update_override_torque_params`, so every tune's own
`update_limits()` puts the PID limits on the rail and its saturation test
(`steer_max - |output| < 1e-3`) fires there, with no tune code. The limits scale linearly in
`steer_max` only for a linear `lateral_accel_from_torque`; an NNLC-style interface would need
its own handling. Corner windows on routes 12a, 12c and 126 ran 58 to 100% rail duty with the
integrator frozen (no windup); exit ringing there (about 1 m/s^2 pk-pk) is P-driven loop
gain at 4 to 11 m/s, which is what KD addresses.

### One STEER_MAX, and the tune scale (2026-09-30)

Every steering Mazda runs the EPS envelope's flat `STEER_MAX` of 1200 counts, the panda's
`max_torque` for it. Until 2026-09-30 it stepped 1200 -> 800 between 14.2 and 14.5 m/s
(`STEER_MAX_LOOKUP`), which put every learned and typed torque value in two units: bins learned
one scale each, a plain interp smeared the step across the 12.0-16.4 m/s span (+18% / -19%
torque), and friction inverts (its counts are friction * STEER_MAX), so the override carried a
per-count interp for the bins, a per-frame rescale for flat tunes, and the rail interpolated
ceiling/scale across the cliff (0.024 high at 31.9 to 32.3 mph, which kept `at_rail` false
and suppressed the saturation alert there). The EPS is linear in counts (latAccelFactor spread
1.47x across speeds in counts, 2.59x as a fraction of the ceiling), so one scale loses nothing:
all of that is gone, and the ceiling clamp and the rail stay because they are the hardware.

Values fitted on upstream's 800 (`TUNE_STEER_MAX`) are converted once where they enter,
latAccelFactor x 1.5 and friction / 1.5 (`get_tune_scale`), so they put the same counts per
m/s^2 on the wire as a stock build:
- params.toml's tune, in the Mazda interface's `configure_torque_tune` override (an override
  rather than a step in `_get_params`, so sunnypilot's second call converts too). The CX-5 2022
  borrows the CX-9 2021's (1.76 at 800 counts); it runs its own global learner's value
  instead, 1.222 / 0.154 at 800 counts (`TORQUE_TUNES`, converted with the rest).
- the manual override and the custom offline values (`TorqueParamsOverride*`), typed against
  stock behaviour. The developer UI's live latAccelFactor is on STEER_MAX, 1.5x a stock value.
- NNLC's model torque (and the friction override summed with it): the models were trained on
  800. Until this change NNLC ran 1.5x torque below 32 mph on the stepped scale.

Behaviour on the wire is unchanged in normal driving (same counts per m/s^2). What moved, over
3.8M highway frames of the CX-5 corpus:
- The driver envelope is built from 1200 above 32 mph as well: at the 620-count ceiling the
  command yields from about 54 counts of opposing driver torque, not 27, the same continuous
  envelope the panda checks. The 800-based envelope had trimmed 0.25% of highway frames, by a
  median 60 counts, with a hand on the wheel.
- controlsd's 0.01 mismatch is 12 counts above 32 mph (was 8): 30.7% of highway frames exceed
  it rather than 39.5%, as below 32 mph already.
- torqued's fits are not scale-invariant (TLS on a wide cloud, and a spread-based friction):
  on the same points, learning on 1200 moves highway latAccelFactor within about 5% and friction
  counts up 13 to 21%. The global learner's outer buckets fill in about two highway drives
  instead of one.

Rejected: a flat 800 with |torque| up to 1.435 at crawl. It needs no conversions, but
car.capnp documents `actuators.torque` as bounded at 1.0, and any consumer that clips there
would silently cap low-speed authority at 800 counts.

The manual override writes the params on every frame, ahead of the speed bins' interp, which
would otherwise out-write it between its 3 s polls. Writes compare in float32: `torque_params`
is a capnp Float32 builder, and a float64 compare re-ran `update_limits` at 100 Hz.

### Swapped chassis (`speed_dependent.toml`)

The swap fallback names a CX-5 KE or KF, CX-9 2016-20, Mazda3 or Mazda6 behind the 2022 CX-5
EPS. Each substitutes the CX-5 2022 table under `requires_steer_to_zero`: the table was learned
behind that EPS firmware. On its stock EPS the entry is withheld (a steering floor and the
firmware's dead band) and the car learns default bins from its global seed.

## Speed-bin learner and cache (`torqued_ext.py`)

Each bin is a `TorqueBuckets` with per-bucket minimums = the global learner's / n_bins,
fed by `_on_torque_point` after upstream's quality filters. `_estimate_params_speed_binned`
runs upstream's total-least-squares fit per bin, clips to +-sanity of the seed (upstream's
FACTOR_SANITY 0.3 / FRICTION_SANITY 0.5; 1.0 / 1.0 with the relaxed toggle), advances the
bin's filter decay from MIN_FILTER_DECAY 50 toward MAX 250 as upstream does, and resets a bin
that goes NaN with valid data. Bins come from `speed_dependent.toml` or the defaults seeded
with the global offline values. A bin refits whenever a point has been routed to it since its
last fit. Until 2026-09-30 the test was the bucket length, which stops changing once all eight
ring buffers are full (12000 points) while new points keep replacing old ones, so a full bin
froze at whatever it had learned when it filled: on the test car every bin above 16 m/s was
full, and the device's values sat within 5% of the seeds while a refit of its own cached
points read up to 21% lower. A car with a steering floor keeps the bins centered above it,
and the first of them keeps the lower edge the full table gives it (`min_speed`) instead of
reaching down over the floor and the dead band to 5 m/s.

Wire. The per-bin values do not ride on `lateralTorqueParameters` (comma's struct, which an
upstream sync would collide on). torqued_ext publishes its own `liveTorqueParametersSP`
message beside every upstream one, at the same 4 Hz cadence and validity: `version`,
`speedBinCenters`, `speedBinLatAccelFactors`, `speedBinFrictions`, `speedBinValid`,
`speedBinPoints` (empty on the wire). On the wire the service is `customReserved19`, the
last of sunnypilot's reserved Event slots, so `log.capnp` stays byte-identical to upstream;
`torqued_ext.LIVE_TORQUE_PARAMETERS_SP_SERVICE` names it and `LiveTorqueParametersSP`
aliases the struct (`custom.CustomReserved19`).

Cache. torqued writes `LiveTorqueParameters` every 240 frames (60 s) with `with_points=True`;
that same call is the extension's hook, which writes the same fork struct with the point
buckets filled to `LiveTorqueParametersSP`. Upstream's cache keeps the restore key, decay
and the valid flag; the fork's carries its own `version`, the centers, the values and the
points.

Restore (`_restore_ext_cache`, guards added 2026-09-02 in `fbabecf35c`): both caches
must carry upstream's restore key (fingerprint, tuning type, offline seeds, VERSION, via
`CarParamsPrevRoute`) and the fork cache this config's `seed_version` and bin centers; filtered values are
taken only when upstream's cache was written valid; anything non-finite or outside a bin's
clip range rejects the whole cache, because the bins are one interpolated tune and a partial
restore leaves a step between a cached bin and a re-seeded neighbour. The decay is restored
from upstream's cache rather than reset to MIN each boot (which had made the bins re-learn
five times faster after every restart). The points are checked on their own (bin count,
finiteness); when they fail, the values still restore and the buckets start empty. A fork
cache that is absent restores nothing. A valid pair restores bit-identically
(`test_torqued_cache_restore.py`).

Replacing learned values on a release. The per-bin seeds are not in upstream's restore key,
so editing `laf_bp` / `friction_bp` alone leaves every device on its learned values (only a
value outside the new +-30% band rejects the cache, and the points refit it straight back).
Each TOML entry carries a `seed_version` (0 when absent); the fork cache records the one it
was learned under (`seedVersion`, 0 for a cache written before the field) and any mismatch
restarts learning from the seeds, values and points both. Bump it with the seed refresh in
the same commit; a device picks it up on the next boot after the update.

## Seeds (2026-09-30)

The seeds keep the feel of the stepped scale: the CX-5 2022's are the values the test car
drives on, the CX-9 2021's its existing seeds, both converted exactly to 1200 counts (bins
from 16.4 m/s up were learned at 800: LAF x 1.5, friction / 1.5). The CX-5's new 34.5 and 37
m/s centers take the old table's values there, within 1% of it in counts at every speed.

`tools/mazda_long/speed_bin_seeds.py` gives the reference to check them against: it replays
torqued's point filter over rlogs with the steer axis in applied counts / 1200
(`torqueOutputCan`, so every build lands on one scale) and runs the learner's estimator per bin
(at most 1500 points per steer bucket, |steer| < 0.5, TLS slope, spread friction), which
matches a direct refit of the device's cached bin points. Over the CX-5's builds from
2026-09-01 on (511 segments), in counts against the seeds: feedforward within 0 to 9% up to 55
mph and 16 to 18% more torque at 63 to 78 mph; friction 30 to 58% higher. About half of that is
the frozen bins (a refit of the device's own cached points already reads up to 27% more torque
below 30 mph and 21% more friction), the rest learning on 1200 above 32 mph. The fixed learner
starts from the seeds and moves toward the data; the test car runs the relaxed sanity band
(+-100%), stock is +-30% LAF and +-50% friction.

The fit depends on the controller as well as the car (5 to 8 m/s read 1.76 to 2.88 across
earlier builds, 13 to 24 m/s stayed within 10%). Bootstrap over segments gives +-0.19 LAF at
6.5 m/s and +-0.04 to 0.10 elsewhere; a second owner's CX-5 2022 (49 segments) sits within 25%
of each bin. Split-half cross-validation of layouts against 1 m/s fits: a crawl bin (3 to 5
m/s) and a split at 13 m/s did not beat the existing centers, noise-limited below 14 m/s; a bin
from 35.75 to 40 m/s (80 to 89 mph) did, learning 2.76 against 2.20 at 34.5 in both this era and
the whole corpus. The CX-9 2021 has no drive above 32 mph in the corpus (its one route reads
2.29 at 16.4 m/s against the converted 2.30).

Friction refit (2026-10-01). After a day on these seeds with the learner unfrozen (16 routes,
about 36,000 points in the six bins up to 28 m/s, none above 31.25 m/s), a refit of the
device's cached bin points reads friction 8 to 55% above them, and the older and newer half of
each bin agree within 5%. The CX-5 2022's friction through 28 m/s now takes that fit: 0.232,
0.155, 0.145, 0.147, 0.094, 0.091 (was 0.188, 0.143, 0.126, 0.095, 0.084, 0.080). The old
0.095 at 16.4 m/s was a dip the data does not have, and stock's +-50% band around it stopped
short of the car's 0.147. LAF stays: its halves land up to 20% either side of the seed (2.42 vs
2.10 at 6.5 m/s, 2.11 vs 2.53 at 12), and 34.5 and 37 m/s have no new points. seed_version
stays 2: the scale is the same, and a bump would drop every learned cache.

Learned values (2026-10-04). The six bins through 28 m/s take the test car's filtered values
from its cache (develop e1255e6a7, seed_version 2, upstream decay at its 250 cap):

| center (m/s) | 6.5 | 9.5 | 12.0 | 16.4 | 21.0 | 28.0 |
|---|---|---|---|---|---|---|
| LAF | 2.37 (was 2.41) | 2.63 (2.72) | 2.13 (2.29) | 1.74 (1.72) | 1.85 (1.75) | 2.27 (2.28) |
| friction | 0.201 (0.232) | 0.145 (0.155) | 0.135 (0.145) | 0.134 (0.147) | 0.104 (0.094) | 0.084 (0.091) |

A refit of the cached points (7,400 to 12,000 per bin) leads the filter the same way: LAF 12 to
14% under the old seeds at 9.5 and 12 m/s, friction within 15% everywhere. The 10-01 friction
refit overshot below 16.4 m/s. 34.5 and 37 m/s still have no points and keep their values;
seed_version stays 2.

## Constants

| name | value | measurement | route |
|---|---|---|---|
| `MAX_FRICTION_JERK` | 2.5 m/s^3 | clips the friction jerk input only; clip_curvature's 5 m/s^3 bounds the request | 132, 139, 12d, 12f |
| `CENTER_CHATTER_JERK_DEADZONE_SPEED_BP/V` | [0, 5, 12, 25] m/s -> [0.08, 0.12, 0.18, 0.18] m/s^3 | about 20% of the straight-line gain; deadzone = larger part of the -12% hf_rms (0.0354 without it) | 132, 139, 12d, 12f |
| `CENTER_CHATTER_JERK_DEADZONE_LAT_ACCEL_BP/V` | [0, 0.18, 0.35] m/s^2 -> [1, 1, 0] | zero in a real turn | same |
| jerk filter cutoff (`LP_FILTER_CUTOFF_HZ`) | 1.2 Hz | hf_rms 0.0329 -> 0.0327; release p50 0.126 -> 0.099 | same |
| `STEER_RELEASE_I_DECAY` | 0.8 | release-edge error p90 0.67 m/s^2, P swing p90 2.24 in 100 ms; largest release delta when removed (0.19 / 0.88 p50 / p90) | 132, 139, 12d, 12f |
| `RELEASE_ERROR_RAMP_T` | 0.3 s | release slew p90 29.4 -> 14.7 /s | same |
| `KD_INTERP_SPEEDS` / `KD_INTERP` | [7.5, 10, 12, 14.5] m/s -> [1.65, 1.05, 0.85, 0] | 0.3 s * KP(v); plant K 2.56, tau 0.86 s, delay 150 ms; sim overshoot 1.63 -> 1.21; removal returns 85% of exit swing (0.313 -> 0.415, v0 0.433); logged pair 0.188 vs 0.489 RMS | 12e (system-ID); 132/139 vs 12d/12f |
| curvature buffer length | `LAT_ACCEL_REQUEST_BUFFER_SECONDS` 1.0 s (v0's) | applied counter-swing +0.01 to 0.05 at decelerating low-speed exits without it | 12d, 12f |
| `MISMATCH_THRESHOLD` | 1e-2 | controlsd's own threshold; one slew step is 0.010 of the 1200 scale (0.015 on 800) | logged v2 drive (51% of active frames flagged) |
| `RAIL_EPS` | 1e-3 | the tunes' own saturation test | |
| `RATE_STEP_FRACTION` | 0.9 | a speed-dependent scale read at vEgoRaw vs vEgo; exact on a flat one | |
| EPS rail (`get_steer_rail_schedule`) | 1148/1200 = 0.96 below 8 m/s to 620/1200 = 0.52 from 14.5 m/s | 58 to 100% rail duty in tight corners, integrator frozen | 12a, 12c, 126 |
| `STEER_MAX` / `TUNE_STEER_MAX` | 1200 at every speed / 800 | tune scale 1.5; LAF spread 1.47x in counts vs 2.59x as a fraction of the ceiling | CX-5 corpus |
| CX-5 speed bins | 6.5, 9.5, 12.0, 16.4, 21.0, 28.0, 34.5, 37.0 m/s | device values converted, new centers interpolated; 37.0 learns 2.76 vs 2.20 at 34.5; crawl bin and 13 m/s split not supported by CV | device cache 2026-09-29; builds from 2026-09-01, 511 segments |
| bin sanity | +-0.3 LAF, +-0.5 friction (relaxed 1.0 / 1.0) | upstream's FACTOR_SANITY / FRICTION_SANITY | |
| filter decay | 50 to 250 (MIN / MAX_FILTER_DECAY) | restored from cache; resetting to MIN re-learned 5x faster per boot | |
| cache cadence | every 240 sm frames (60 s) | upstream's `LiveTorqueParameters` write | |
| `MAZDA_STEER_TO_ZERO_TORQUE_TUNE` | 2.0 | seeded when `TorqueControlTune` is unset on a Mazda with `MazdaFlags.STEER_TO_ZERO_EPS` | |
