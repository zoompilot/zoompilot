# Curve and limit speed planning (SCC vision, SCC map, SLA publication)

Code: `openpilot/sunnypilot/selfdrive/controls/lib/smart_cruise_control/`: the shared
`limits.py` and `speed_profile.py`, zoompilot's planners in `zoompilot/`
(`vision_controller.py`, `map_controller.py`), sunnypilot's own
planners beside them, and the two SLA publishers in `controls/lib/speed_limit/`
(`speed_limit_assist_zp.py`, `assist_mirror.py`). Tests: `smart_cruise_control/tests/`,
`smart_cruise_control/zoompilot/tests/`, `speed_limit/tests/`. Tools:
`tools/mazda_long/decel_overshoot/` (`icbm.md`, Tools).

## Which planner runs

`zoompilot.make_smart_cruise_control(CP)` picks the curve planners once per drive. Brands in
`zoompilot.TUNED_BRANDS` (`('mazda',)`, the brands whose stock ACC response and planning limits
have been measured from their logs) run the zoompilot vision and map planners this document
describes. Every other brand runs sunnypilot's `vision_controller.py`, `map_controller.py` and
`smart_cruise_control.py`, kept byte-identical to sunnypilot master together with their tests,
so an unmeasured brand is not tuned on Mazda data and a sync never conflicts there. Only the
zoompilot vision planner publishes `vAheadMin`; on every other brand it is 0, which leaves
ICBM's restore on its stillness heuristic (`icbm.md`). `limits.py`, `speed_profile.py` and
`publish_ramp` stay shared with the SLA publishers on every brand.

## The shape of the problem

A curve is a region, not a point, and the path ahead can hold several. `speed_profile`
is a pure backward pass: `allowed_speed` turns curvature into the speed at the lateral
acceleration ceiling, `backward_pass` propagates each constraint back at the budget the
platform can deliver, `required_decel` reports the decel needed now to meet the tightest
constraint past the actuation lead, `lead_distance` is the distance eaten before braking
is fully effective, and `min_profile_speed` is the dip a slow consumer must pre-position
for. zoompilot's vision and map controllers, the pcm SLA machine and the SLA mirror all
call these; none of them carries its own copy of the formula.

Curvature comes from geometry (`orientationRate.z / velocity.x`), so slowing down does
not lower the prediction and talk the controller out of the slowdown it just started
(the old lateral-acceleration form used the model's velocity plan and did exactly that).

## Per-path limits (`limits.py`)

A car is either openpilot-long or stock ACC for the whole drive, so the budget and the
actuation lead are picked once.

- **openpilot long**: budget 1.2 m/s^2 (`A_CRUISE_MIN`), jerk from `J_CRUISE_VALS`
  over `A_CRUISE_MAX_BP`, lead `longitudinalActuatorDelay`. Mirrored from the upstream
  planner, which cannot be imported (it imports the SP overlay); `test_limits.py` reads
  the upstream source and pins the mirror.
- **stock ACC**: budget is the decel the overshoot lever holds the ECU to (mazda 0.75,
  the budget column of `DECEL_OVERSHOOT_PARAMS`; unmeasured brands 0.5, where being wrong
  only means braking earlier), response 1.0 s, plus the time to walk the dash down by
  the tracking gap at the measured servo walk rate (mazda 8 mph at 4 mph/s). The ECU
  brakes on the gap as soon as it opens and the lever tracks `vEgo` down from there, so
  the rest of a deep dip is not dead time; leading by the whole dip committed a 30 mph
  drop 5 s early. The native 5 mph hold grid must not size the lead: synthesized holds
  register as discrete presses (route 126: 294/294 steps at 1 mph).

`COMMIT_FRAC = 0.7` is the shared gate: a constraint binds once the decel it requires
reaches 70% of the budget, leaving headroom for slope and curvature error. It was swept
against the corpus together with `_PLAN_MARGIN`.

## Model curvature range bias

The model reads path curvature low at range, so the profile binds late and no budget
can make the distance back up. Measured against the curvature the car actually pulled
at 26 apexes on route 135, the ratio of predicted to realized kappa runs 1.00 inside
30 m, 0.79 at 80 m and 0.30 past 130 m. It is a distance effect, not a horizon-fraction
one: the same shape holds in the 18 to 31, 31 to 42 and 42+ mph bands, and on two
unrelated routes, where it is stronger still. Nothing better is available from the
message: geometric curvature off `position.x/y` carries the same bias and is worse near
the car, because the far end of the path is an 8 to 10 s prediction that regresses
toward straight under its own uncertainty.

`_KAPPA_BIAS_GAIN` undoes it before the profile solve, as the reciprocal of the measured
ratio, capped at 1.5. Uncapped it reaches 2.07 past 130 m, but the per-apex spread is
wide there (IQR 0.50 to 0.82 at 80 to 120 m, and 3 of 26 apexes over-read), and 1.5 is
where the closed-loop replay stops buying apexes and starts adding straight-road
limiter activity. The gain returns to 1.0 as the curve closes, so it moves *when* the
car brakes, not how hard: an over-read at range is walked back by the same solver a
second later, and on a straight road it multiplies a kappa of zero.

The near field (the near window below) is measured, not predicted, and keeps raw
geometry for what it decides (escalation, the hold, the UI). It no longer floors the
target: a `_near_floor` that refused to command below the raw near requirement was
what released the planner on route 260 (big model): the bend held its 30 mph target from
140 m down to 50 m, then the raw read inside the 4 s window (0.68-0.77 of the true
curvature at 40-60 m, the corpus lower quartile) floored the target at 34-36 mph and the
car entered at 2.5 m/s^2. Without the floor the corpus route sim takes hot road curves
from 64 to 59 of 138 on the small model (old servo 61) and 4 to 3 of 11 on the big one,
for 4 more small-model curves slightly under speed; the constant-radius over-slowing it
guarded against did not show.

### The fitted band

The table was fitted on a 30 to 50 mph road (radii 50 to 150 m) and fades out from
50 to 60 mph (`_KAPPA_BIAS_V_BP`). Nothing shows the model under-reads a 500 to 650 m
highway bend, and multiplied by 1.4 such a bend reads as a corner the raw path says can
be taken at the set speed: at 70 mph a perfectly reported r=645 m bend 100 m out (1.49
m/s^2 at the set speed, under the ceiling) commits on both paths and walks the dash down
6 mph for nothing. Inside the band the gain is measured; above it the raw path decides,
which is the pre-gain behaviour.

## The highway horizon

The vision planner plans on the whole predicted path, because of what a misread costs the
servo that carries it out:

- **Tracking servo (decel overshoot, stock ACC) and openpilot long: the whole model path,
  at every speed.** The tracking servo holds a gap sized to the request, and a request from
  beyond the near window is capped at the budget (next sections), so a misread holds a
  budget-sized gap for as long as it lasts. Replayed over 11.4 h above 50 mph (7.1 h small
  model, 4.3 h big), planning past the near window (up to 300 m) raised false commits from
  7.2 to 8.9 an hour on the small model and from 0.7 to 1.2 on the big one, at a median cost
  of 0.8 and 0.1 mph with the far request capped. openpilot long commits at 0.84 m/s^2,
  which a far misread rarely reaches: a 250 m horizon against the near window raised its
  false commits from 5.2 to 6.2 an hour on the small model (median cost 1.5 mph) and left the
  big model at 0.5.
- **No walking-servo mode.** A servo that walks the dash to the whole dip would make a far
  misread a full dip and need the near window above the band; the tracking servo is the only
  stock servo on a tuned brand (`icbm.md`, decel overshoot), so there is no opt-out to plan for.

The near window alone above 60 mph made the planner blind to a sharp curve until 3 s out.
Closed-loop sim (`curve_sim.py`, refit plant, each model's measured read, vision only, stock
ACC), before (fb8e3f5c0b: near window above 60 mph, dash walked to the dip; small model, the
big model's entry is within 0.2 mph) and now; entry speed, apex lateral accel, peak decel:

| set -> curve | before | now, small | now, big |
|---|---|---|---|
| 55 -> 35 mph | 46.7 mph, 3.38, -0.84 | 42.3 mph, 2.78, -0.88 | 39.4 mph, 2.41, -0.80 |
| 65 -> 45 mph | 62.7 mph, 3.69, -0.93 | 57.9 mph, 3.15, -0.75 | 54.0 mph, 2.73, -0.74 |
| 75 -> 45 mph | 72.6 mph, 4.94, -1.52 | 68.0 mph, 4.34, -0.68 | 61.8 mph, 3.58, -0.74 |

A sharp curve at 65+ is still beyond what the camera can see in time to slow gently: at 70
mph the whole path (10 s, ~310 m) covers about a 13 mph drop at the budget once the 3 s stock
actuation lead is taken out. Map data is what covers those.

## The near window

The first `_NEAR_T = 3 s` of path on the small model, `_NEAR_T_BIG = 4 s` on the big one
(`modelV2.big`), is treated as measured rather than predicted: it holds the planner through a
curve, floors the far correction, and is the only part of the path allowed to ask past the
budget. What bounds it is the model flagging a bend that is not there. Measured on the 74 h
corpus by seconds ahead: model curvature over the curvature the car then drove (yaw rate over
speed, smoothed 1 s) on bends that need slowing at 1.9 m/s^2, median (p25), and the share of
model-flagged bends that are real at 1.5 m/s^2:

| ahead | small, 25-50 mph | big, 25-50 mph | small, 50+ mph | big, 50+ mph |
|---|---|---|---|---|
| 0-1 s | 1.06 (1.00), 99% | 1.01 (0.99), 100% | 1.06 (1.00), 99% | 0.92 (0.89), 100% |
| 1-2 s | 0.98 (0.87), 98% | 1.00 (0.95), 99% | 1.03 (0.95), 99% | 0.89 (0.85), - |
| 2-3 s | 0.84 (0.59), 94% | 0.92 (0.77), 96% | 1.00 (0.85), 99% | 0.90 (0.83), - |
| 3-4 s | 0.70 (0.28), 90% | 0.81 (0.51), 93% | 0.98 (0.84), 95% | 0.91 (0.82), 95% |
| 4-5 s | 0.43 (0.07), 90% | 0.69 (0.14), 92% | 0.91 (0.67), 85% | 0.84 (0.78), 85% |
| data | 18.6 h, 1019 bends | 2.1 h, 116 bends | 8.7 h, 107 bends | 4.3 h, 9 bends |

Small model: the read holds to 3 s, and above 50 mph a 4 s window quadruples the phantoms that
ask past the budget (flagged inside the window, nothing there needing even 1.5 m/s^2, asking
over 0.75 with a 1 s lead): 0.3 to 1.2 an hour. Big model: its 3-4 s band reads and flags as
well as the small model's 2-3 s band, and in 4.3 h above 50 mph it raised no past-budget
phantom inside 4 s. Its read at 4 s above 50 mph is not settled: 9 bends, 86% of those hours
on two interstate drives. At 3 s the window stays inside ~100 m up to 75 mph, so it needs no
distance cap.

## What the planner asks for (`a_needed`)

On stock ACC the published vision `aTarget`, which keys ICBM's gap, is not `a_required`. The
commit gate budgets the whole stock lead (response plus dash walk); once committed the servo is
already walking, so the request only leads by the ECU's response (`lead_distance(v, t_lead)`,
1.0 s). A request past the budget says the car is late, and only measured geometry may say so:

- the far field, beyond the near window, asks for the budget at most;
- the near field may ask past it into the ECU's remaining range (the 0.90 and 1.05 m/s^2
  columns of `gap_v`), but that ceiling fades to the budget as the set speed goes from 50 to
  60 mph (`_ESCALATION_V_BP`). It is keyed on the set speed so it does not creep back as the
  car slows into the curve. In the route sim, cruising from 60 mph the escalation bought one
  hot curve in 13 for a p10 peak decel of -0.99 instead of -0.73 m/s^2, which is the braking
  that was reported as too hard.

openpilot long publishes `a_required` itself: there the wire is an actuator command, clipped
to the budget (`publish_ramp`).

## Planning margin

`_PLAN_MARGIN = 0.95`: plan to 95% of the lateral ceiling so actuation lag lands the
apex on it instead of over it. Swept against the corpus: at 1.0 the sim leaves 13% of
fair apexes above 2.2 m/s^2; at 0.95 that drops to 5% for 1.4% of speed given up.

## Commit, hold, release

Commit when `a_required >= COMMIT_FRAC * a_budget`. Once braking, hold while the near
path limits the car below `vEgo + _IN_CURVE_MARGIN` (1 m/s; the car is in the curve) or while
the required decel is above `_RELEASE_FRAC = 0.3` of the budget, so the gate does not chatter
on noise. Until 10-04 the near path was held against the set speed, and on a winding road any
gentle bend kept the plan (and the dash) down through the whole exit. After release the ICBM
restore is still capped at `vAheadMin`, so the dash only climbs to the next dip. The
state machine (`entering`, `turning`, `leaving`) is display-only.

Near convergence a bumper-distance constraint makes `required_decel` scream through its
distance floor (`D_FLOOR = 0.5 m`), so the published request is capped at the unit-gain
pull to the lowest profile speed ahead, the same bound the v target uses.

### Commit hold below 50 mph

Below the bias band the small model's far read flickers: a bend seen at 150 m can vanish from
the path at 100 m and come back at 50 m. In the route sim the planner released ~100 m short of
the apex on both the old and the new code. The old servo arrived slow anyway, because its gap
decayed slowly after the release; the tracking servo with no hold arrived hot (road curves
over 2.2 m/s^2 at <= 45 mph: 65 of 149 with the old servo, 72 with no hold).

So on stock ACC below `_HOLD_V_MAX` (22.4 m/s, 50 mph) a bend that has bound (`a_required` at
30% of the budget or more) for `_HOLD_SEEN_T = 2.0 s` is latched with its profile speed and
remaining distance, and held until the near window reaches it; a deeper bend replaces it. The
held bend keeps the solver active, caps `vAheadMin`, and asks for the budget at most, like any
prediction. The latch clears when the near window reaches the bend, on release, at 50 mph, and
never runs on openpilot long.

A phantom binds for a frame or two, a real bend for seconds before the read drops it. Latch
time, route sim on the refit plant, 149 road curves and 228 straights at a set speed of 45 mph
or less (columns as in the route sim table below):

| latch | 1.5 s | 2.0 s | 2.5 s | 3.0 s | old servo |
|---|---|---|---|---|---|
| over 2.2 m/s^2 | 67 | 68 | 68 | 68 | 65 |
| > 1 mph over | 77 | 81 | 83 | 84 | 76 |
| > 2 mph under | 52 | 50 | 48 | 48 | 64 |
| straights losing > 2 / > 5 mph | 11 / 8 | 7 / 3 | 6 / 1 | 6 / 1 | 15 / 5 |

2.0 s is the shortest latch that brings straight-road slowdowns under the old servo's; the
longer ones buy two more clean straights for more arrivals over the curve speed.

## `publish_ramp` and the op-long budget

The plan `aTarget` wire means two different things. On stock ACC it is not an actuator
command: ICBM's decel-overshoot lever keys the dash gap on each limiter's own request and
the ECU does its own easing, ramped at `PUB_JERK = 2.0 m/s^3`. Every publisher clips at
the budget (`PlanningLimits.a_pub_min`); only the vision planner, whose near field is
measured, passes `a_floor=A_PUB_MIN` (-2.0, beyond which no path can follow) to
`publish_ramp` so a late measured bend reaches the ECU's range past the budget. A map
target or a speed limit is a prediction and never does. On
openpilot long the same wire seeds `mpc.set_cur_state`, and the MPC pins stage 0 to the
seed, so the published value comes straight back out of the MPC candidate and
`min(candidates)` prefers it over the cruise candidate that `A_CRUISE_MIN` clips. There
it is an actuator command: clipped to the budget (`PlanningLimits.a_pub_min = -1.2`) and
ramped at the consumer's own jerk. A one-frame step would otherwise reach the actuators
as a snap, because the seed is not jerk-limited the way the cruise candidate is.

`publish_ramp` is the one implementation, used by the vision controller, the map
controller, the pcm SLA machine and the SLA mirror. Idle states track `a_ego` (wire
parity, and the ramp's starting point on activation).

## Map controller

The map path binds a waypoint with the same gate and solver as vision:
`required_decel` past a lead that includes the dash traversal on stock ACC, compared to
`COMMIT_FRAC * a_budget`. A target that slips back under the commit gate while still
ahead (the car is already slowing harder than the gate needs) is retained, and its
distance keeps tracking the car: the published decel divides by that distance every
frame, and frozen at the commit-time value it under-requested more the closer the car
got. When the distance is degenerate the decel falls back to a `_T_FALLBACK = 2.8 s`
horizon. The published `aTarget` is the required decel to the target, not `a_ego`:
`a_ego` there meant map curves never braked the real car, since the overshoot lever
keys on `aTarget`. A map target is a prediction, so that request is capped at the budget;
only the vision planner's near field may ask past it.

Map targets also flicker on over road that never curves. On route 25c (09-29) mapd matched
the car onto crossing side streets for 1-2 s at a time, and seven targets of 19-49 mph bound
at 47-56 mph. A new or deeper target must therefore be the selection for `T_confirm(v)` before
it binds: none up to 45 mph, rising to 2.5 s at 50 mph (`_CONFIRM_V_BP`, `_CONFIRM_T`). Release
is immediate. The corpus has the map on for 6.2 h (2.3 h above 45 mph): 65 episodes on 22
routes, classed by the curvature the car then drove. At 40 mph and up:

| class | episodes | how long the target lasts | appears before the apex |
|---|---|---|---|
| real (driven curve speed within 1.1x of the target) | 9 | 5.0-12.6 s from 45 mph | median 10.5 s, min 5.3 s |
| phantom (driven road allows 2.3x the target or more) | 10, all from 47 mph | up to 2.1 s, one 6 s | n/a |

The phantoms come from two routes (25c, and route 53, where one target held 6 s: a persistent
map error no time gate catches); the other 48 routes with the map on had none above 45 mph.
The shipped ramp suppresses 7 of the 10 and takes phantom onsets above 45 mph from 4.3 to 2.2
an hour with the map on; the two that pass besides route 53 start at 47 mph, where the ramp
asks 1 s or less. A real curve is delayed up to 2.5 s from 50 mph and 0.3 s at 45.6 mph, and
after the delay and the stock actuation lead the tightest keeps 0.58 s of slack. A ramp from
40 mph left that 45.6 mph curve 0.37 s late; a 45 to 47 mph ramp catches 9 of the 10 (1.3 an
hour) but leaves it 0.29 s.

## The route 135 decel chain (stock ACC)

What limits deceleration on the stock path is the ECU's response to the dash gap.
Route 135: a median 2.65 s from a limiter taking the plan source to the car pulling
-0.5 m/s^2. The response does not saturate at 0.75 to 0.8 m/s^2 as first read: the
refit (50 routes) reaches -0.82 at a 10 mph gap at 45 mph and more at highway speed, with
too little data past 14 mph to say where it stops (`icbm.md`). The lever therefore sizes the
gap to the limiter's request and caps it at the budget, so the budget is what the car
actually does. On stock ACC the vision target is pre-positioned at the deepest dip on the
horizon (a dash servo cannot track a continuous profile in 1 mph taps) and the decel gap
does the shaping, tracking `vEgo` down rather than walking the dash to the dip. With
nothing left to brake for (`a_required` 0) the stock target is the profile itself, so inside
a long curve and on its exit the dash climbs to the allowed speed; until 10-04 it was capped
at `vEgo` and the car could only lose speed while the plan held. On
openpilot long the target leads `v_ego` by the required decel and never goes below the
slowest point of the plan, past which the P candidate is already railed at the budget.

## Route sim, before and after

### 10-04: target follows the profile, release on vEgo, set-speed ceiling

Rebuilt on device_data (189 routes, refit plant), vision only, stock ACC, road curves. Base is
the 09-30 planner; "fixes" is the target and release changes at the old flat 1.8; "all" adds the
2.0 -> 1.8 set-speed ceiling. Time lost is against driving the window at the set speed; "back"
counts curves where the car is within 2 mph of the set speed by 150 m past the apex.

| set speed | curves | apex lat p50 | over 2.2 | time lost p50 / p90 s | back |
|---|---|---|---|---|---|
| <= 45 mph | 132 | 1.70 / 1.74 / 1.92 | 33 / 34 / 42 | 4.8/8.3, 4.4/7.6, 3.6/6.6 | 37 / 42 / 56 |
| 50-55 mph | 38 | 1.78 / 1.77 / 1.80 | 10 / 10 / 10 | 2.6/5.4, 2.3/4.8, 2.0/4.5 | 4 / 4 / 8 |
| 60+ mph | 15 | 1.74 / 1.74 / 1.74 | 4 / 4 / 4 | 1.3/4.5, 1.3/4.2, 1.3/4.2 | 0 / 0 / 0 |

The fixes alone cost nothing in hot curves; the ceiling buys most of the time back at 45 mph and
below for 8 more of 132 over 2.2 m/s^2 (path-reach caveat below applies; p90 apex is 3.2 in all
three). Straight-road slowdowns over 2 mph: 8 -> 5 of 306 below 60 mph.


`route_sim.py` replays real roads closed loop: the checked-out vision planner and ICBM servo
drive the fitted plant along 399 logged routes (177 yield a scored window), and at each
position the planner sees the model path recorded nearest that spot while the curvature the
car actually drove is the truth. Each curve that needs slowing from the set speed (the speed
driven in the 20 s before, rounded to 5 mph) is approached from 25 s out; each 800 m straight
with nothing needing slowing within 300 m scores the speed given up. Before is fb8e3f5c0b,
after is this tree, both on the refit plant, vision only, stock ACC:

| set speed | curves | over 2.2 m/s^2 | > 1 mph over | > 2 mph under | p10 peak decel | straights | lose > 2 mph | lose > 5 mph |
|---|---|---|---|---|---|---|---|---|
| <= 45 mph | 149 | 65 -> 68 | 76 -> 81 | 64 -> 50 | -0.82 -> -0.83 | 228 | 15 -> 7 | 5 -> 3 |
| 50-55 mph | 39 | 15 -> 15 | 20 -> 23 | 13 -> 10 | -0.84 -> -0.89 | 257 | 6 -> 5 | 4 -> 4 |
| 60+ mph | 13 | 5 -> 6 | 9 -> 9 | 2 -> 1 | -0.87 -> -0.74 | 743 | 0 -> 1 | 0 -> 0 |

Columns: apex lateral accel over 2.2 m/s^2; apex speed more than 1 mph over the curve's speed
at 1.9 m/s^2; the car slowed more than 2 mph under that speed before the apex; the 10th
percentile of each approach's hardest decel; straights by speed given up. At 45 mph and below
the tracking servo arrives near the curve speed instead of under it (64 -> 50 under) for more
arrivals just over it (76 -> 81) and 3 more hot curves, with half the straight-road slowdowns.
At 50-55 mph fewer curves end slow at the same hot count, braking harder at p10 (-0.84 ->
-0.89). From 60 mph the hard braking that was reported drops (p10 -0.87 -> -0.74) for one more
hot curve in 13 and one straight-road trim (a 70 mph far-field phantom, 2.7 mph).

Caveats:

- Path reach. A replayed path is as long as the recorded car was fast; on 26 of 149, 6 of 39
  and 4 of 13 curves it did not reach the point where a budget brake from the set speed has to
  start, so the absolute hot counts run high. Both runs see the same paths.
- Intersection turns are excluded: 345 of the 546 detected curves need less than 20 mph, the
  sim's dash floor. The table counts the 201 road curves; 16 of them are on the big model.
- The old servo keyed its gap on `LP_SP.aTarget`, which the sim emulates as the MPC does on
  stock ACC (a 0.5 s lag railing at -1.2). No leads (windows with a close lead are skipped), no
  map, and 60+ has 13 curves.

## On-car validation (pending)

Stock ACC, vision on, map on where there is data. Decel overshoot has no toggle: it is on
for every brand with a measured plant (`icbm.md`).

1. 50-70 mph, curves needing a 10-25 mph drop: peak decel near the 0.75 budget rather than
   -0.9 to -1.2, the dash tracking at most ~10 mph under `vEgo` instead of stepping to the
   curve target, apex lateral accel at or under ~2.2 m/s^2.
2. Twisty roads under 45 mph: the planner stays on the bend to the apex (`sccVision` holding
   the plan source) instead of releasing ~100 m short; note any straight-road slowdown over
   5 mph (sim: 3 in 228 straights).
3. Map at 50+ mph: a real target binds about 2.5 s after it appears and the car still reaches
   it at the target speed; no binds from side-street matches (the 25c stretch of Howells Ferry
   Road is the known case).
4. Big model above 50 mph: false brakes from the 4 s near window, whose read there rests on 9
   bends.
5. After each curve the dash walks up to the next dip rather than freezing below it.
6. Extract the drives and refit the plant (`icbm.md`, Tools): the corpus has under a minute of
   gap data at 60-70 mph and about 10 s past a 14 mph gap.

## Constants

| name | value | measurement | route |
|---|---|---|---|
| `TUNED_BRANDS` | `('mazda',)` | brands with a measured stock ACC response | n/a |
| `_A_LAT_REG_V_BP` / `_V` | 17.9 to 26.8 m/s set speed -> 2.0 to 1.8 m/s^2 | lateral ceiling, keyed on the set speed so it does not rise as the car slows. 2.0 until 09-30, when route 260's entries felt late and hot (apexes 2.1-2.4, with the near floor that released bends late, since removed); 1.8 everywhere until 10-04, when drivers called back roads too slow; the highway end keeps 1.8 | route 260, user reports |
| `_IN_CURVE_MARGIN` | 1.0 m/s | the plan holds while the near field limits the car below `vEgo` plus this | user reports |
| `_PLAN_MARGIN` | 0.95 | 13% -> 5% of fair apexes above 2.2 for 1.4% speed | corpus sim |
| `COMMIT_FRAC` | 0.7 | swept with the margin | corpus sim |
| `_RELEASE_FRAC` | 0.3 | hysteresis against gate chatter | n/a |
| `_NEAR_T` / `_NEAR_T_BIG` | 3.0 s / 4.0 s | small read holds to 3 s, a 4 s window takes past-budget phantoms 0.3 -> 1.2/h above 50 mph; big's 3-4 s band reads and flags like small's 2-3 s | 74 h corpus: small 27 h, big 6.4 h |
| `_KAPPA_BIAS_D` / `_GAIN` | 0..110 m -> 1.0..1.5 | ratio 1.00 / 0.79 / 0.30 at 30 / 80 / 130 m, cap where replay stops buying apexes | route 135, 26 apexes |
| `_KAPPA_BIAS_V_BP` | 22.4 to 26.8 m/s | fitted on a 30 to 50 mph road | route 135 |
| `_ESCALATION_V_BP` | 22.4 to 26.8 m/s set speed | near-field ceiling fades to the budget; at 60+ escalation bought 1 hot curve in 13 for p10 -0.99 vs -0.73 | route sim |
| `_HOLD_V_MAX` / `_HOLD_SEEN_T` | 22.4 m/s / 2.0 s | far read flickers below 50 mph; latch sweep above | route sim, 149 curves, 228 straights |
| `_CONFIRM_V_BP` / `_CONFIRM_T` (map) | 20.1 to 22.4 m/s -> 0 to 2.5 s | phantoms up to 2.1 s (one 6 s), real 5.0-12.6 s; drops 7 of 10 phantoms, tightest real slack 0.58 s | 6.2 h map on, 65 episodes |
| `_OP_LONG_A_BUDGET` | 1.2 m/s^2 | `A_CRUISE_MIN` | upstream |
| `_STOCK_A_BUDGET['mazda']` | 0.75 m/s^2 | the overshoot table's budget column (10 / 7.75 mph gap at 45 / 65) | 115,878 samples, 50 routes |
| `_SERVO_TRACK_GAP['mazda']` | 8 mph | the overshoot gap at budget, 7.75 to 10 mph by speed | same |
| `_STOCK_A_BUDGET_DEFAULT` | 0.5 m/s^2 | estimate, safe direction | n/a |
| `_STOCK_RESPONSE_T` | 1.0 s | estimate, erring large | n/a |
| `_SERVO_WALK_RATE['mazda']` | 4.0 mph/s | 4.1 hold frames, 3.8 taps | route 126 |
| `A_PUB_MIN` / `PUB_JERK` | -2.0 m/s^2 / 2.0 m/s^3 | stock publication depth for measured geometry, and ramp | n/a |
| `D_FLOOR` | 0.5 m | division floor for a constraint at the bumper | n/a |
| `_T_FALLBACK` (map) | 2.8 s | decel horizon for a degenerate distance | n/a |
| MRCC response | 2.65 s median to -0.5; -0.57 at 4-6, -0.82 at 10, -0.84 at 14 mph gap (45 mph) | stock ACC response chain | route 135; 50 routes |

## Tried and rejected

- The lateral-acceleration-percentile heuristic. Used the model's velocity plan, so a
  planned slowdown lowered the prediction below the abort threshold mid-braking. It is
  still what the brands outside `TUNED_BRANDS` run.
- Geometric curvature from `position.x/y`. Same range bias, worse near the car.
- An uncapped bias gain (up to 2.07). Wide per-apex spread past 80 m; the replay bought
  no more apexes and added straight-road limiter activity.
- Applying the gain above the fitted band. A perfectly reported r=645 m highway bend
  read as a corner and walked the dash down 6 mph for nothing.
- Requiring the raw profile to bind before the corrected one may commit. Replayed on
  route 135 it gives back a third of what the gain bought (apex max 2.47 -> 2.68,
  median 1.56 -> 1.66): at 20 m/s a real corner enters the 200 m horizon reading 30% of
  its curvature and corroboration only arrives inside 120 m.
- A trust discount on far kappa, persistence, and a 120 m horizon for the highway false
  commits (all measured against the near window when it was chosen).
- The near window alone above 60 mph under the tracking servo. It only bought ~1.7 fewer
  cheap false commits an hour, for sharp curves entered 10-15 mph hot.
- A 4 s near window on the small model. Past-budget phantoms above 50 mph 0.3 -> 1.2/h.
- Escalating past the budget at any set speed. From 60 mph it bought one hot curve in 13
  for a p10 peak decel of -0.99 instead of -0.73 m/s^2.
- A budget floor on committed requests below 50 mph. Hot road curves stayed at 72 of 149:
  the planner had already released, so there was nothing to floor.
- A 1.5 s hold latch. 8 of 228 straights lost over 5 mph, against 3 at 2.0 s and the old
  servo's 5.
- The near floor itself. It voided the commit hold on any curved near path, and on the
  big model it released a correctly seen bend at 45 m (above).
- A big-model gain fitted by seconds ahead (1.0 to 1.40 at 5 s, faded out over 40-50 mph).
  The fit is sound on the corpus but on route 260 it entered the curves 1-2 mph hotter than
  the shared distance table, because the fade removed most of the correction at 40-45 mph;
  the by-range tables are in the session's bigbias_report.txt.
- Commit fraction 0.6 and 0.5: one curve 1 mph better, the rest identical. The bend is seen
  with the required decel already past the gate, so the gate is not what is late.
- Latching the hold at the raw apex distance, or ending it at 2 s instead of the near window:
  nothing on the corpus or on route 260.
- A vision veto in place of the map confirmation time. It would drop every phantom seen,
  but on real curves at 40+ mph vision agreed with the map a median 3.7 s (up to 12.2 s)
  after the target appeared.
- Ramping the map confirmation from 40 mph. Left a 45.6 mph real curve 0.37 s late.
- Publishing -2.0 on openpilot long. Bypassed `A_CRUISE_MIN` through the MPC seed.
- Publishing `a_ego` from the map and SLA sources. Map curves and map limits never
  braked the real car on stock ACC.
- Freezing the retained map target's distance at commit time. Under-requested more the
  closer the car got.
- Sizing the stock actuation lead from the 5 mph hold grid. Forged holds never snap.
