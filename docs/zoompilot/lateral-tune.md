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
| v0 | sunnypilot's `latcontrol_torque_v0.py`: setpoint == the live request, error corrected in lateral-accel space, the extension owning the feedforward params. Byte-identical to sunnypilot's; the only change it sees is the corrected `steer_limited_by_safety` flag from the classifier. | `TorqueControlTune = 0.0`, and any torque car with Enforce Torque Control off (`torque_tune.resolved_tune_version`) |
| v1 | sunnypilot's current `LatControlTorque` (the `lac` controlsd built), untouched | `TorqueControlTune = 1.0` |
| v2 | v0 plus the four mechanisms below | `TorqueControlTune = 2.0`; seeded on steer-to-zero Mazdas by `_seed_mazda_torque_defaults` (`MAZDA_STEER_TO_ZERO_TORQUE_TUNE = 2.0`) |

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
0.01. One carcontroller slew step is 12/1200 = 0.010 of scale below the CX-5's cliff and
12/800 = 0.015 above it, so any command walking faster than the rate limit reads as limited:
51% of active frames on a logged v2 drive, and the bidirectional freeze blocked integrator
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
lets the tune's own saturation test raise the alert. The 0.9 step fraction covers the
carcontroller reading the scale at `vEgoRaw` while the classifier interpolates at `vEgo`.
`error_prev` is the previous frame's `pid_log.error`; the one-frame lag is accepted at 100 Hz.

Caveat: a driver-limited frame with a decaying integrator also hands False to the alert
path; the alert still needs the output on the PID limit for `steerLimitTimer` seconds with
the wheel untouched. Blind spot: a panda-rejected frame is reported as delivered in
carOutput, so a starved EPS looks clean here; the carcontroller's non-delivery latch and
`tools/mazda_long/lkas_starvation_check.py` cover that path (route 148, `src == 192`).

Replay A/B of the directional freeze (routes 132/139): corner |i| falls 4 to 6x,
stale-integrator-vs-error frames 59% -> 5%, open-loop output delta < 0.014.

### EPS rail via `steer_max` (`latcontrol_torque_ext.py`)

`get_steer_rail_schedule(CP)` gives EPS_CEILING / STEER_MAX(v): on the CX-5 the rail is
648/1200 = 0.54 at 14.2 m/s and 620/800 = 0.775 from 14.5 m/s up. The extension writes it to
the host tune as `steer_max` in `update_override_torque_params`, so every tune's own
`update_limits()` puts the PID limits on the rail and its saturation test
(`steer_max - |output| < 1e-3`) fires there, with no tune code. The limits scale linearly in
`steer_max` only for a linear `lateral_accel_from_torque`; an NNLC-style interface would need
its own handling. Corner windows on routes 12a, 12c and 126 ran 58 to 100% rail duty with the
integrator frozen (no windup); exit ringing there (about 1 m/s^2 pk-pk) is P-driven loop
gain at 4 to 11 m/s, which is what KD addresses.

### Friction and LAF per-count across the STEER_MAX cliff (`latcontrol_torque_ext_override.py`)

The CX-5's STEER_MAX steps 1200 -> 800 between 14.2 and 14.5 m/s. Bins learn normalized
values under one scale each (the bin boundary is aligned to the cliff), so a plain interp
between the 12.0 and 16.4 m/s bin centers smears the scale's step across the whole span:
about +18% torque below the cliff (the 27 to 32 mph wobble band) and -19% above. Both
tables therefore interpolate in CAN-count space and rescale by `steer_max_schedule` at the
current speed, exact at bin centers, unchanged on flat platforms (schedule None).

Friction is inverted. `get_friction` returns +-friction * latAccelFactor in lat-accel space
and the linear torque function divides by latAccelFactor again, so the normalized friction
torque is the bin value and its counts are friction * STEER_MAX(v). A plain interp of the
CX-5 bins put 149 -> 166 -> 112 -> 122 counts on the wire through 12.0, 14.2, 14.5 and
16.4 m/s. The 2026-08-29 note that friction "cancels the cliff on its own" (79.5 vs 80.2
counts) used the wrong formula; corrected 2026-09-01.

The manual override owns the params on every frame, or the per-frame interp out-writes it
between its 3 s polls (299 of 300 frames before the fix). Comparisons are made in float32
because `torque_params` is a capnp Float32 builder; a float64 compare re-ran `update_limits`
at 100 Hz.

## Speed-bin learner and cache (`torqued_ext.py`)

Each bin is a `TorqueBuckets` with per-bucket minimums = the global learner's / n_bins,
fed by `_on_torque_point` after upstream's quality filters. `_estimate_params_speed_binned`
runs upstream's total-least-squares fit per bin, clips to +-sanity of the seed (upstream's
FACTOR_SANITY 0.3 / FRICTION_SANITY 0.5; 1.0 / 1.0 with the relaxed toggle), advances the
bin's filter decay from MIN_FILTER_DECAY 50 toward MAX 250 as upstream does, and resets a bin
that goes NaN with valid data. Bins come from `speed_dependent.toml` (CX-5:
6.5, 9.5, 12.0, 16.4, 21.0, 28.0, 35.0 m/s, refreshed 2026-08-19 from the device cache) or
the defaults seeded with the global offline values.

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
| `MISMATCH_THRESHOLD` | 1e-2 | controlsd's own threshold; one slew step is 0.010 / 0.015 of scale below / above the cliff | logged v2 drive (51% of active frames flagged) |
| `RAIL_EPS` | 1e-3 | the tunes' own saturation test | |
| `RATE_STEP_FRACTION` | 0.9 | carcontroller rounds the scale at vEgoRaw, classifier interpolates at vEgo | |
| EPS rail (`get_steer_rail_schedule`) | 648/1200 = 0.54 at 14.2 m/s, 620/800 = 0.775 above | 58 to 100% rail duty in tight corners, integrator frozen | 12a, 12c, 126 |
| STEER_MAX cliff | 1200 -> 800 at 14.2 to 14.5 m/s | plain interp smear +18% below / -19% above; friction counts 149 -> 166 -> 112 -> 122 | speed_dependent.toml bins |
| CX-5 speed bins | 6.5, 9.5, 12.0, 16.4, 21.0, 28.0, 35.0 m/s | boundary at 14.2 m/s aligned to the cliff | device cache 2026-08-19 |
| bin sanity | +-0.3 LAF, +-0.5 friction (relaxed 1.0 / 1.0) | upstream's FACTOR_SANITY / FRICTION_SANITY | |
| filter decay | 50 to 250 (MIN / MAX_FILTER_DECAY) | restored from cache; resetting to MIN re-learned 5x faster per boot | |
| cache cadence | every 240 sm frames (60 s) | upstream's `LiveTorqueParameters` write | |
| `MAZDA_STEER_TO_ZERO_TORQUE_TUNE` | 2.0 | seeded when `TorqueControlTune` is unset on a Mazda with `MazdaFlags.STEER_TO_ZERO_EPS` | |
