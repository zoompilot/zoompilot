# Intelligent Cruise Button Management (ICBM)

Code: `openpilot/sunnypilot/selfdrive/car/intelligent_cruise_button_management/controller.py`
(the servo), `opendbc/sunnypilot/car/icbm_actuation_profile.py` (per-brand ECU
characteristics). Tests: `car/tests/test_icbm_servo.py`, `test_icbm_overshoot.py`,
`test_icbm_sla_*.py` (closed loop against a simulated Mazda body ECU).

On button-actuated (non-pcmCruiseSpeed) cars openpilot cannot command acceleration. The
stock ACC integrates cruise button presses into a dash set speed and decelerates
according to that. ICBM is a servo that walks the dash onto the plan target
(`longitudinalPlanSP.vTarget`) with synthesized presses, and hands the dash back to the
driver's setpoint when the limiter releases. Measurements below are from the Mazda
CX-5 2022 unless stated.

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
SET+ parks all down moves for `DRIVER_PRESS_GRACE_T`. Without overshoot in play a down
move is a plain setpoint correction and stays unconditional.

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
below the target, hold while a dip is coming however quiet the target is, and abort a
restore in progress when a dip appears. Route 126: 3 of 8 over-ceiling apexes were
restore-fed, the car accelerating between bends into the next apex.

A genuine driver SET- parks up moves for the grace window (a refused re-anchor would
otherwise restore the baseline right over a fresh -5); a press in the other direction
cancels the other grace. `buttonEvents` carry only the wheel's own presses (forged
frames echo on src 128+ and never reach carState), so the grace cannot latch on the
servo's own sends.

## Actuation profile

`ICBMActuationProfile` carries what the servo needs to know about a body ECU.
`DEFAULT_PROFILE` is discrete taps only, no grid, no hold, no stable-setpoint
requirement: the long-standing ICBM behaviour. A brand changes behaviour only by adding
a measured entry.

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
the car is already several mph over it, so it arrives at curves hot. When a limiter
source demands decel (`aTarget < -min_decel` and `vEgo > vTarget`), the servo commands
the dash below `vEgo` by the gap that yields the requested decel, capped at the plan
target from above (down-only: a stale command fail-safes to the car slowing). The
command tracks `vEgo` down through the manoeuvre and rises back on its own as the car
converges and `aTarget` relaxes.

Mazda inverse map, from 422k hands-off cruise samples across 447 rlog segments: about
0.09 m/s^2 per mph of gap, dead below ~2 mph, saturating near -0.75 m/s^2 by ~9 mph.
`gap_v` carries a lead over the steady-state inverse because the gap the ECU actually
sees lags the command: the lever's rise is limited by the dash walk (~4 mph/s), not by
`DECEL_OVERSHOOT_RISE`, so a manoeuvre spends its first seconds short of the request.
Route 135 measured a 2.65 s median from a limiter taking the plan source to the car
pulling -0.5 m/s^2; commanding the deeper gap up front pays that walk back, and the
request falls as the car converges so the lever still lets go on its own.

The lever is only valid while the servo can pull it and while the limiter that asked
for it is live. It never integrates behind a block (driver press, confirm prompt, SET+
grace): winding up there only banks a stale gap to dump when the block lifts, and a
limiter still asking rebuilds a full gap in ~0.5 s at the rise rate. It releases
slowly while the limiter is live (aTarget flaps between the ECU's coast, downshift and
brake stages) and at the build rate once the plan is back on cruise, where a residual
only holds the dash down and stalls the restore.

## Restore quiet window

Sized from an 11-route, 57k-frame sweep of the recorded target streams, scoring
"regret" (restores that a following dip made pointless) against speed lost to waiting.
The churn suppression is all bought in the first second: regret 67.7% with no wait,
27.0% at 1.0 s; 3.0 s only reaches 26.2% while nearly doubling the speed given up.

## Interaction with the cruise arbiter

The servo reads the SLA session state one message hop late. A pending confirm prompt
(`session_state == preActive`) parks any move and holds the quiet timer at zero; card
additionally vetoes emission with same-frame state (`cruise-arbiter.md`).

## Constants

| name | value | measurement | route |
|---|---|---|---|
| `INACTIVE_TIMER` | 0.4 s | upstream settle after readiness | n/a |
| `REACT_TIMER` | 0.3 s | glitch filter, upstream | n/a |
| `REACT_DEADBAND` | 2 units (limiter) / 1 (cruise) | limiter jitter 1 to 2 units/frame | ICBM corpus |
| `RESTORE_QUIET_TIME` | 1.0 s | regret 67.7% -> 27.0%; 3.0 s reaches 26.2% at twice the speed cost | 11 routes, 57k frames |
| `DRIVER_PRESS_GRACE_T` | 3.0 s | +5 reverted within 1.4 s | route 126 t=341 |
| `FAST_MODE_MIN` | 3 units | stream in-flight overshoot below this | route 126 |
| `FAST_STALL_T` | 1.5 s | dash never moved under the stream | n/a |
| `decel_bp` / `gap_v` (mazda) | [0.02..0.73] m/s^2 -> [2..10] mph | ~0.09 m/s^2 per mph, dead < 2 mph, saturates ~9 mph | 422k samples, 447 segments |
| `max_gap` (mazda) | 10 mph | response saturated | same |
| `min_decel` (mazda) | 0.15 m/s^2 | gentle coast-downs left to stock | same |
| `DECEL_OVERSHOOT_RISE` | 10 mph/s | full gap in ~0.5 s, inside `REACT_TIMER` | n/a |
| `DECEL_OVERSHOOT_RELEASE` | 3 mph/s | no pumping between ECU decel stages | route 126 |
| `tap_rate_hz` (mazda) | 5 Hz | 0.93 steps/press at 5 Hz vs 0.47 at ~9 Hz | 674 segments |
| `longpress_step` / first / period (mazda) | 5 mph / 0.6 s / 0.55 s | physical hold grid | 52 episodes |
| servo walk rate (mazda) | 4 mph/s | 294/294 steps at 1 mph; 4.1 hold, 3.8 taps | route 126 |

## Tried and rejected

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
