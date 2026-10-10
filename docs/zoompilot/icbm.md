# Intelligent Cruise Button Management (ICBM)

Code: `openpilot/sunnypilot/selfdrive/car/intelligent_cruise_button_management/controller.py`
(the servo), `opendbc/sunnypilot/car/icbm_actuation_profile.py` (per-brand ECU
characteristics). Tests: `car/tests/test_icbm_servo.py`, `test_icbm_overshoot.py`,
`test_icbm_sla_*.py` (closed loop against a simulated Mazda body ECU). Tools:
`tools/mazda_long/decel_overshoot/` (Tools, below).

On button-actuated (non-pcmCruiseSpeed) cars openpilot cannot command acceleration. The
stock ACC integrates cruise button presses into a dash set speed and decelerates
according to that. ICBM is a servo that walks the dash onto the plan target
(`longitudinalPlanSP.vTarget`) with synthesized presses, and hands the dash back to the
driver's setpoint when the limiter releases. Measurements below are from the Mazda
CX-5 2022 unless stated.

## Activation

ICBM has no toggle. It runs on a car with the buttons (`helpers.icbm_applicable`: the opendbc
`intelligentCruiseButtonManagementAvailable` flag, and a setpoint the ECU keeps) while a
feature that acts through it is on. The features are one table, `demand.CONSUMERS`, which the
boot decision, the latch, the migration and the settings locks all read: SCC vision and map
need ICBM on stock ACC only (under alpha long the planner executes them), SLA assist and custom
increments in both modes. A new feature is one entry. There is no escape hatch: the features
are the switches.

card owns the decision (`icbm_latch.IcbmLatch`) and publishes it as
`carStateSP.zoompilot.icbmActivation`; controlsd, selfdrived, plannerd and the UI read that,
never `CarParamsSP.pcmCruiseSpeed`, which only records the boot decision. The field is an enum
with `unset` first, so logs from before it (old routes, process replay refs) fall back to the
boot flag. Upstream-owned files change by one token on four lines: `cruise.py` x2 and
`controlsd.py` x2 read `self.pcm_cruise_speed` (a property over the latch in `VCruiseHelperSP`,
a read of `carStateSP` in `ControlsExt`) instead of `self.CP_SP.pcmCruiseSpeed`.

The decision moves only while neither openpilot nor the stock cruise is engaged and no cruise
button is down or changing. There the two modes agree on everything (no `longActive`, the
dash's setpoint, an idle servo, no SLA session), so a feature switched on while driving starts
at the next engage and one switched off keeps ICBM until the next disengage. The latch reads
demand on card's 10 Hz params thread and `engaged = CC.enabled or CS.cruiseState.enabled` in
`VCruiseHelperSP.update_enabled_state`; card runs the button timers in both modes, so none
carries across a change. On a change card clears the reconciler, recomputes the minimum set
speed and moves the SLA owner (`pcm_machine_owns_sla(CP, icbm_active)`: the arbiter's
`set_icbm_active`; plannerd keeps its machine and the mirror, picks per frame and resets on a
handover), and selfdrived re-initialises the servo, keeping `fast_faulted`.

Panda safety needs nothing: on every ICBM brand the +/- frames pass only while
`controls_allowed`, with no ICBM flag. opendbc reads `pcmCruiseSpeed` only to default it, and
`longActive` / `override` only on openpilot-long paths (Mazda, Chrysler, Honda Bosch, Hyundai
audited). The audit found Chrysler CUSW (Jeep Cherokee 5th gen) claiming ICBM although
`chrysler_cusw.h` passes only cancel/resume; fixed in opendbc 957a8279c5.

### Settings and migration

The toggle is gone from mici, TICI and sunnylink. The features are offered wherever
`has_long or icbm_applicable`; `ui_state.has_icbm`, sunnylink's `has_icbm` and the SLA assist
demotion (`set_speed_limit_assist_availability`) all key on the capability, or assist could never
bring ICBM up. While engaged with ICBM off the device will not turn on a feature that needs it
(`ui_state.icbm_start_locked`). sunnylink's validator cannot gate a toggle on its own value, so
it locks those features both ways while engaged on an ICBM car. On an upstream sync, a
sunnypilot change to the toggle UI takes ours.

`migration.py` ran once when the toggle went (2026-10), in `setup_interfaces`, marked by
`IcbmDemandMigrated` (`PERSISTENT | BACKUP`: restoring a pre-migration backup runs it again).
On a capable car whose toggle was off it turned off the features that toggle had left inert,
so no install started pressing buttons: SCC vision, SCC map and custom increments on stock ACC
(SLA assist to warning), custom increments only under alpha long, where SLA assist stays on
and now moves the dash to the limit by itself. It removes `IntelligentCruiseButtonManagement`;
the key stays in `params_keys.h` (upstream owns it). Replay against danger-unstable
(2026-10-08, four CX-5 drives, ICBM on and off) was identical; on-car pending.

## The servo

States: `inactive`, `preActive` (react timer running), `holding`, `increasing`,
`decreasing`. Readiness requires `CC.enabled` and no override/cancel/resume and no
physical button held; a driver press drops the servo to `inactive` and it re-enters
through `preActive` after `INACTIVE_TIMER`.

Two error bands. Against a limiter-sourced target (SCC vision, SCC map, SLA), whose
value jitters 1 to 2 display units frame to frame, the servo reacts only past
`REACT_DEADBAND = 2`. Against the cruise source the target is the driver setpoint, a
stable integer, so it is tracked exactly (deadband 1): a dash residual from a dropped
press self-heals instead of stranding the dash 1 mph low (the "F2 ratchet"). Any error
must persist for `REACT_TIMER` before acting, so a one-frame target glitch (a bad map
sample) cannot trigger a button burst. Once moving, the servo runs to the exact target,
not just inside the deadband.

### Down moves

A limiter's decel is urgent, so down moves skip the quiet window while the limiter is
live. Two guards: a residual overshoot gap left after the source flips back to cruise
must not start a fresh descent (the lever is not a destination), and a genuine driver
SET+ parks down moves to the plan target it overrode (`press_target`) for
`DRIVER_PRESS_GRACE_T`; a lower target that appears afterwards is new information and goes
through (route 260 t=1059: cruise set to 40, a 34 mph curve 1.5 s later, and the car
accelerated into it for the whole window). Without overshoot in play a down move is a plain
setpoint correction and stays unconditional.

### Up moves (restore)

On cars whose profile sets `decel_needs_stable_setpoint` the ECU will not commit to
decelerating while the set speed is moving, and limiter dips arrive in trains. Restoring
between dips churns the dash and delays the next decel, so an up move waits for the
plan target to hold still for `RESTORE_QUIET_TIME`, on every entry path. The quiet timer
is keyed on the raw plan target, not the overshoot-adjusted command: the lever's slow
release moved the command every few frames and pinned the timer at zero until the decay
finished (route 126: 4.1 s of extra post-curve braking). It is held at zero through a
confirm prompt so a decline or timeout still waits a full window.

With a valid vision lookahead (`smartCruiseControl.vision.vAheadMin > 0`) the profile
replaces the stillness heuristic outright: restore immediately when nothing ahead binds
below the target, walk up no further than a dip that is coming however quiet the target
is, and abort a restore in progress when a dip appears below the dash. Route 126: 3 of 8
over-ceiling apexes were restore-fed, the car accelerating between bends into the next
apex. The dip is a ceiling, not a stop: `vAheadMin` is the braking-feasible profile
minimum, so walking up to it cannot feed an apex, while freezing below it held route 128
at 20 mph for 6.8 s behind a 41 mph dip and the car braked on to 19.7. Only zoompilot's
vision planner publishes `vAheadMin` (`scc-curve-planning.md`, which planner runs); on every
other brand it is 0 and the stillness heuristic stays in charge.

A genuine driver SET- parks up moves for the grace window (a refused re-anchor would
otherwise restore the baseline right over a fresh -5); a press in the other direction
cancels the other grace. `buttonEvents` carry only the wheel's own presses (forged
frames echo on src 128+ and never reach carState), so the grace cannot latch on the
servo's own sends.

## Actuation profile

`ICBMActuationProfile` carries what the servo needs to know about a body ECU.
`DEFAULT_PROFILE` is discrete taps only, no hold, no stable-setpoint requirement: the
long-standing ICBM behaviour. A brand changes behaviour only by adding a measured entry.

Mazda CX-5 2022, from a 52-episode driver long-press corpus and an injected-press
efficiency analysis over 674 rlog segments:

- Taps register reliably at 5 Hz and move 1 mph. Pushing to ~9 Hz makes the ECU drop
  presses: ~0.47 steps per press versus ~0.93 at 5 Hz, so faster is slower.
- A physical hold snaps the set speed to the next multiple of 5 mph about 0.6 s into the
  hold, then steps 5 mph every ~0.55 s, sometimes with a trailing step after release.
  The grid is confirmed in imperial display units only; metric users plan with taps.
- MRCC will not start decelerating until the set speed stops changing.

### The hold fold on other brands

The servo's sustained sends (`increaseHold` / `decreaseHold`) are a stream of presses,
valid wherever taps are. A brand whose button interface has no native hold cadence
(everything but Mazda so far) folds them onto the discrete tap of the same direction
(`tap_equivalent`) instead of rejecting an unknown state, so enabling ICBM on a new
brand needs no servo change.

## Fast mode

The servo's 10 Hz hold stream never registers on the Mazda ECU as a held button: the
wheel keeps broadcasting its genuine button-up frames, which interleave with the forged
ones, so the ECU sees paced discrete presses. Across all recorded routes 149 of 149
stream-driven dash steps were 1 mph; route 126 measured 294 of 294 steps at 1 mph, zero
grid snaps, 4.1 mph/s under hold frames and 3.8 mph/s under taps. It is still the
fastest walk available, so the stream takes any move with real distance
(`FAST_MODE_MIN = 3` units remaining) and taps take the remainder, where the stream's
in-flight frames would overshoot and ping-pong. If the dash does not move for
`FAST_STALL_T` under the stream this ECU is not registering it at all; the servo faults
to taps for the rest of the drive and logs `icbm_fast_mode_fallback`. The stream carries
no grid or metric assumption, so metric users get it too.

## Decel overshoot

A stock ACC's deceleration scales with the gap between the dash set speed and the
*actual* speed, not the target. Commanding dash = target produces almost nothing until
the car is already several mph over it, so it arrives at curves hot; walking the dash
straight to a deep target does the opposite and brakes as hard as the dip is deep. When
a limiter source demands decel (its own `aTarget < -min_decel` and `vEgo > vTarget`),
the servo holds the dash the gap below `vEgo` that yields that decel and tracks `vEgo`
down through the manoeuvre, even while that sits above a deeper plan target. Two bounds
shape the end: near the target the dash sits no further below it than the car is above
it, so the ECU's lag lands the car on it rather than under it, and during a descent the
dash only rises toward the target, never above it (a stale command still fail-safes to
the car slowing). In `update_calculations`, with `gap` from `gap_v` below:

```python
v_command = max(vEgo - gap, vT - max(vEgo - vT, 0.))  # track at the gap, landing floor
dash_cmd = min(v_command, max(dash, vT))              # down-only: rises only to land on vT
```

The request is the active limiter's own `aTarget` (`source_a_target()`:
`smartCruiseControl.vision`, `.map`, `speedLimit.assist`), not `LP_SP.aTarget`. On stock
ACC the latter is the MPC output, which rails at `A_CRUISE_MIN` (-1.2) whenever the target
sits a few mph below `vEgo`, so keyed on it the lever pulled its full gap for every real
dip and the car finished each curve 4-5 mph under the speed it needed.

The lever is only valid while the servo can pull it and while the limiter that asked
for it is live. It never integrates behind a block (driver press, confirm prompt, SET+
grace): winding up there only banks a stale gap to dump when the block lifts, and a
limiter still asking rebuilds a full gap in ~0.5 s at the rise rate. It releases
slowly while the limiter is live (aTarget flaps between the ECU's coast, downshift and
brake stages) and at the build rate once the plan is back on cruise, where a residual
only holds the dash down and stalls the restore.

Decel overshoot has no user toggle. It runs on every brand with an entry in
`DECEL_OVERSHOOT_PARAMS` (a measured plant; Mazda today) and on no other, since the only
alternative is walking the dash to the whole dip, which brakes as hard as the dip is deep.
The former `SmartCruiseDecelOvershoot` param and its one-time Mazda seed are gone; a stale
key on an updated device is ignored.

### The Mazda plant

`fit_plant.py` models realized decel as a 0.3 s delay and a 0.8 s first-order lag of
f(gap), piecewise linear in the gap with a linear speed term, fitted by least squares on
clean stock-ACC stretches (engaged, no pedals, no lead inside 3 s, no openpilot-long route;
decel samples weighted 5x). The 2026-09-30 refit over the 74.7 h corpus has 115,878
samples (1.6 h) from 50 routes, about 970 s of them with more than 2 mph of gap; the
previous fit, which the table came from, had 65,981 samples from 32 routes. A 0.5 s lag
fits as well (decel-regime rms 0.169 against 0.168 m/s^2).

| gap (mph) | 2.5 | 4 | 6 | 8 | 10 | 14 |
|---|---|---|---|---|---|---|
| decel at 45 mph (m/s^2) | -0.18 | -0.57 | -0.57 | -0.70 | -0.82 | -0.84 |
| decel at 65 mph | -0.15 | -0.62 | -0.59 | -0.78 | -0.89 | -0.85 |

It coasts below ~2.5 mph of gap, holds a first brake stage near -0.57 from 4 to 6 mph and
reaches -0.82 at 10 mph (45 mph). It does not saturate near -0.75 by 9 mph as the original
422k-sample hands-off fit read, but at 45 mph it is nearly flat from 10 to 14 mph, and past
14 mph there are about 10 s of data, so how far it keeps growing is not measured. Where the
data is thin:

- Above 60 mph. 1.8 h of engaged stock ACC at 60-70 mph holds 59 s with more than 2 mph of
  gap, 10 s of it inside the clean stretches, so the 65 mph row above a 6 mph gap is the
  speed term extrapolated.
- CX-9. The corpus has no engaged stock-ACC sample from its 1.8 h of CX-9 drives, so the
  CX-9 runs the CX-5 table. 49 s of CX-9 stock-ACC stretches with openpilot disengaged match
  the CX-5 plant within 0.03 m/s^2 at 2-6 mph of gap.
- Grade and dash motion, which the fit does not model. The residual moves 0.26-0.30 m/s^2
  per m/s^2 of along-road gravity (braking weaker downhill). With grade as a covariate,
  samples with the dash falling brake 0.11-0.14 m/s^2 harder than samples with it flat for
  3 s; the tracking servo spends a manoeuvre with the dash falling, and on flat road those
  samples brake 0.08-0.11 harder than the fit.

The old table assumed saturation, so `max_gap` did nothing once the plan target itself was
deeper: routes 24c, 24d, 128 and 25c walked the dash 15-35 mph under speed from 50-55 mph
and the car braked at -0.9 to -1.15 m/s^2 for 10-20 s, reaching the curve speed well before
the curve; the driver stepped in on each.

`gap_v` is the inverse of the plant at 45 and 65 mph, interpolated on `vEgo` between the
rows:

| request (m/s^2) | 0.15 | 0.30 | 0.50 | 0.60 | 0.70 | 0.75 | 0.90 | 1.05 |
|---|---|---|---|---|---|---|---|---|
| gap at 45 mph | 2.5 | 3.0 | 3.75 | 6.5 | 8.5 | 10.0 | 16.25 | 20.25 |
| refit inverse | 2.25 | 2.97 | 3.75 | 6.50 | 8.07 | 8.90 | 15.36 | 18.85 |
| gap at 65 mph | 2.5 | 3.0 | 3.5 | 6.5 | 7.25 | 7.75 | 12.5 | 17.25 |
| refit inverse | 2.52 | 2.99 | 3.62 | 3.94 | 7.17 | 7.68 | 14.25 | 15.03 |

The refit moved 0.30 to 3.0 mph on both rows (from 3.25) and 0.50 to 3.75 at 45 mph (from
4.5) and 3.5 at 65 (from 4.0). The other columns sit inside the refit's 90% bootstrap
interval except two: 0.60 at 65 mph, on the 4-6 mph stage plateau where the inverse jumps,
and 0.75 at 45 mph, kept at 10 although the refit puts it at 8.9 (interval 8.4-9.7), so at
45 mph a budget request brakes nearer -0.82 than -0.75. The 0.90 and 1.05 columns are
unconstrained (their intervals run to 29 mph) and stay on the previous fit's extrapolation.

Planners budget 0.75 m/s^2 (`limits._STOCK_A_BUDGET`, 10 / 7.75 mph of gap), so an on-time
manoeuvre never asks for more. The columns past it are the ECU's remaining range, reached
only when the vision planner's measured near field says the car is late, and less of it
from a 50 mph set speed, none from 60 (`scc-curve-planning.md`, what the planner asks for).
The planners size the stock actuation lead from the same gap:
`limits._SERVO_TRACK_GAP['mazda'] = 8 mph` is all of a dip the servo walks before the ECU
brakes at budget, and the rest is tracked down rather than waited out.

## Restore quiet window

Sized from an 11-route, 57k-frame sweep of the recorded target streams, scoring
"regret" (restores that a following dip made pointless) against speed lost to waiting.
The churn suppression is all bought in the first second: regret 67.7% with no wait,
27.0% at 1.0 s; 3.0 s only reaches 26.2% while nearly doubling the speed given up.

## Interaction with the cruise arbiter

The servo reads the SLA session state one message hop late. A pending confirm prompt
(`session_state == preActive`) caps the target at the dash value when it opened
(`prompt_ceiling`) and holds the quiet timer at zero, so a restore cannot raise the dash
past the limit and be adopted as a confirm; card additionally vetoes up moves with
same-frame state (`cruise-arbiter.md`). Down moves go through: a curve is not waiting for
the driver's answer (route 269 the old full freeze parked a -1.2 m/s^2 vision request 4 s
before the apex and let the gap bleed off; route 26b, a whole approach), and only a press
resolves a prompt, never the dash the servo moves.

## Tools

`tools/mazda_long/decel_overshoot/`, run with the repo venv. The simulators import the
checked-out stack, so a baseline is the same command with `PYTHONPATH=<worktree>`.

| tool | does | usage |
|---|---|---|
| `extract.py` | 20 Hz table per rlog segment: speed, dash, plan source and each limiter's own aTarget, ICBM command, MRCC ACCEL_CMD, grade, lead, bookmarks; skips cached segments | `extract.py [rlog_root] [out_dir] [workers]`, default `device_data` -> `test_data/decel_overshoot` |
| `fit_plant.py` | the plant above over a tau x delay grid; writes `plant_fit.pkl` | `fit_plant.py [cache_dir] [--fp MAZDA_CX9_2021] [--out plant_fit.pkl]`; `--fp` needs a cache that records fingerprints, which `extract.py` caches do not |
| `curve_sim.py` | straight-then-curve sweep (11 set / curve pairs, vision then a one-waypoint map mirror without the confirmation time or budget cap) of the real vision planner and ICBM servo against the plant | `curve_sim.py [--model small\|big] [--perfect] [--plant-scale 1.0] [--cache DIR]`, reads `DIR/plant_fit.pkl` |
| `model_reach.py` | model curvature read against driven curvature by distance ahead, per model and speed band; writes the model-path cache `route_sim.py` replays | `model_reach.py [rlog_root] [cache_dir] [workers]` |
| `route_sim.py` | closed-loop replay of logged roads (`scc-curve-planning.md`, route sim) | `route_sim.py <model_cache> <table_cache> <plant_fit.pkl> <out.pkl> [workers] [--routes R1,R2] [--every N [--offset K]]` |

## Constants

| name | value | measurement | route |
|---|---|---|---|
| `INACTIVE_TIMER` | 0.4 s | upstream settle after readiness | n/a |
| `REACT_TIMER` | 0.3 s | glitch filter, upstream | n/a |
| `REACT_DEADBAND` | 2 units (limiter) / 1 (cruise) | limiter jitter 1 to 2 units/frame | ICBM corpus |
| `RESTORE_QUIET_TIME` | 1.0 s | regret 67.7% -> 27.0%; 3.0 s reaches 26.2% at twice the speed cost | 11 routes, 57k frames |
| `DRIVER_PRESS_GRACE_T` | 3.0 s | +5 reverted within 1.4 s; parks only moves to the target the press overrode | route 126 t=341, route 260 t=1059 |
| `FAST_MODE_MIN` | 3 units | stream in-flight overshoot below this | route 126 |
| `FAST_STALL_T` | 1.5 s | dash never moved under the stream | n/a |
| `decel_bp` / `gap_v` (mazda) | [0.15..1.05] m/s^2 -> [2.5..20.25] mph at 45, [2.5..17.25] at 65; budget 0.75 at 10 / 7.75 | plant inverse: coast below 2.5 mph, -0.57 at 4-6, -0.82 at 10 (45 mph); unmeasured past 14 | 115,878 samples, 50 routes |
| `min_decel` (mazda) | 0.15 m/s^2 | gentle coast-downs left to stock | same |
| `DECEL_OVERSHOOT_RISE` | 10 mph/s | full gap in ~0.5 s, inside `REACT_TIMER` | n/a |
| `DECEL_OVERSHOOT_RELEASE` | 3 mph/s | no pumping between ECU decel stages | route 126 |
| `tap_rate_hz` (mazda) | 5 Hz | 0.93 steps/press at 5 Hz vs 0.47 at ~9 Hz | 674 segments |
| servo walk rate (mazda) | 4 mph/s | 294/294 steps at 1 mph; 4.1 hold, 3.8 taps | route 126 |

## Tried and rejected

- Walking the dash to the plan target and gapping only below it (`min(target, vEgo - gap)`).
  A deep target opened a 15-35 mph gap and braked at -0.9 to -1.15 m/s^2 from 50-55 mph,
  reaching curve speed hundreds of metres early (routes 24c, 24d, 128, 25c).
- Keying the gap on `LP_SP.aTarget`. It is the MPC output, railed at -1.2 during any real
  dip, so the lever always pulled full depth and the car finished curves 4-5 mph under.
- A restore gate that froze the dash while any dip was on the horizon. Route 128 t=9880
  held the dash at 20 for 6.8 s behind a 41 mph dip; the restore now walks up to the dip.
- Taps at ~9 Hz. The ECU drops presses; net progress is half that of 5 Hz.
- Planning synthesized holds on the 5 mph grid. Forged holds never snap; 149/149
  stream-driven steps were 1 mph. The native grid timing only applies to a physical
  hold and must not size the actuation lead either (`scc-curve-planning.md`).
- A 3.0 s restore quiet window. Regret improves by 0.8 points over 1.0 s while the speed
  lost to waiting nearly doubles.
- A deadband against the cruise-source target. Stranded the dash 1 mph under the
  setpoint after a dropped press.
- The `preActive` route bypassing the quiet window. The servo chased a stale target
  before card had settled the press's own effects.
- Integrating the overshoot behind a blocked emission. A confirm prompt banked a gap for
  its whole 5 s window and the timeout dumped it as a SET- burst (user report 2026-08-29).
- Releasing the lever slowly after the source is back on cruise. The residual held the
  dash down and stalled the restore.
- Keying the quiet timer on the overshoot-adjusted command. Pinned at zero by the lever's
  own decay (route 126, 4.1 s extra braking).
- Restoring on target stillness when a vision lookahead is available. Restored between
  bends and fed the next apex (route 126, 3 of 8 over-ceiling apexes).
- An immediate walk-back after a genuine driver press. Reads as a fight (route 126 t=341).
- Switching ICBM by mutating each process's `CP_SP.pcmCruiseSpeed` at runtime. `carParamsSP`
  (logged, persisted) stops being constant and the init-time caches still need edits.
- A separate "cruise buttons" setting for SLA assist: on stock ACC it has one valid value.

## Under openpilot longitudinal (Mazda alpha long)

The servo runs wherever the car's ECU keeps the setpoint (`icbm_applicable`), including alpha
long, where the body still owns the cluster set speed. There it follows speed limit sessions
only: curve targets are executed by the planner directly, so the dash holds the arbiter's
setpoint through them (`OP_LONG_PLANNER_SOURCES`), and decel overshoot is off. See
mazda-longitudinal.md, "Speed Limit Assist and ICBM under alpha long".
