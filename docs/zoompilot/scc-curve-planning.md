# Curve and limit speed planning (SCC vision, SCC map, SLA publication)

Code: `openpilot/sunnypilot/selfdrive/controls/lib/smart_cruise_control/` (`limits.py`,
`speed_profile.py`, `vision_controller.py`, `map_controller.py`) and the two SLA
publishers in `controls/lib/speed_limit/` (`speed_limit_assist.py`, `assist_mirror.py`).
Tests: `smart_cruise_control/tests/`, `speed_limit/tests/`.

## The shape of the problem

A curve is a region, not a point, and the path ahead can hold several. `speed_profile`
is a pure backward pass: `allowed_speed` turns curvature into the speed at the lateral
acceleration ceiling, `backward_pass` propagates each constraint back at the budget the
platform can deliver, `required_decel` reports the decel needed now to meet the tightest
constraint past the actuation lead, `lead_distance` is the distance eaten before braking
is fully effective, and `min_profile_speed` is the dip a slow consumer must pre-position
for. The vision controller, the map controller, the pcm SLA machine and the SLA mirror
all call these; none of them carries its own copy of the formula.

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
- **stock ACC**: budget is the ECU's saturation under the overshoot lever (mazda 0.75,
  from the same 422k-sample fit as `DECEL_OVERSHOOT_PARAMS`; unmeasured brands 0.5,
  where being wrong only means braking earlier), response 1.0 s, plus the time to walk
  the dash down to the target at the measured servo walk rate (mazda 4 mph/s). The
  native 5 mph hold grid must not size the lead: synthesized holds register as
  discrete presses (route 126: 294/294 steps at 1 mph).

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

The near field (`_NEAR_T = 3 s` of path) is measured, not predicted, and keeps raw
geometry. `_near_floor` stops the correction outvoting it: going under the near
requirement is allowed only when the raw path genuinely reports something tighter
further out (`_NEAR_FLOOR_FRAC = 0.98` against curvature noise). Without it a
constant-radius curve inflates its own far half while the car is inside it and the car
settles below the speed the road requires for the length of the curve.

### The fitted band

The table was fitted on a 30 to 50 mph road (radii 50 to 150 m) and fades out from
50 to 60 mph (`_KAPPA_BIAS_V_BP`). Nothing shows the model under-reads a 500 to 650 m
highway bend, and multiplied by 1.4 such a bend reads as a corner the raw path says can
be taken at the set speed: at 70 mph a perfectly reported r=645 m bend 100 m out (1.49
m/s^2 at the set speed, under the ceiling) commits on both paths and walks the dash down
6 mph for nothing. Inside the band the gain is measured; above it the raw path decides,
which is the pre-gain behaviour.

## The highway near-window horizon

Above the band the predicted far field is not trusted to plan on either. At highway
speed the curvature that binds is tiny (r < 475 m at 70 mph), and at that level the
model's range error is a tail, not a bias. Over 275 highway minutes (350 segments,
v > 25 m/s) the median read of a real r=285 to 670 m bend is within 5% of the truth out
to 150 m, but of the far-field samples that actually bind at the car's speed the road
needs slowing in only 42% at 90 to 120 m, 33% at 120 to 150 m and 20% at 180 to 210 m
(78% inside 30 m). The rest are 1.3 to 2x over-reads of a gentle bend that hold for
seconds, and once a bend that was never there (r=340 m read at 100 to 190 m on a
straight road for 2.5 s). Replayed above 60 mph they were 7 of the 13 commit episodes
on openpilot long and 9 of 15 on stock.

The study that chose the near window over the alternatives:

| candidate | result |
|---|---|
| trust discount on far kappa (0.8) | also drops 90% of the true bindings: both sit at the allowed speed by construction |
| persistence (0.5 s) | a real bend at 100 to 130 m swings 0.4 to 1.4 m/s^2 frame to frame, so it costs 1 to 2 s on every real bend and the straight-road read still passes |
| 120 m horizon | still let the straight-road read through at 100 to 130 m |
| near window only (`_PLAN_HORIZON_D` -> 0 above the band) | false episodes drop to 3 and 4, all single-frame bumper spikes or a 0.2 m/s^2 trim on a bend at 1.78 m/s^2; all six real bends still commit |

The cost lands on the tightest of the six: r=330 m and r=366 m now commit at 94 m and
112 m instead of 127 m and 159 m and arrive at 2.3 and 2.4 m/s^2 instead of 2.05 and
2.04 (the driver pulled 2.5 and 2.7). Inside the band the whole path plans as before:
the 300 m horizon is past the path length at 50 mph. The ICBM lookahead wire
(`v_ahead_min`) follows the same horizon.

## Planning margin

`_PLAN_MARGIN = 0.95`: plan to 95% of the 2.0 m/s^2 ceiling so actuation lag lands the
apex on it instead of over it. Swept against the corpus: at 1.0 the sim leaves 13% of
fair apexes above 2.2 m/s^2; at 0.95 that drops to 5% for 1.4% of speed given up.

## Commit, hold, release

Commit when `a_required >= COMMIT_FRAC * a_budget`. Once braking, hold while the near
path stays below the setpoint (the car is in the curve) or while the required decel is
above `_RELEASE_FRAC = 0.3` of the budget, so the gate does not chatter on noise. The
state machine (`entering`, `turning`, `leaving`) is display-only.

Near convergence a bumper-distance constraint makes `required_decel` scream through its
distance floor (`D_FLOOR = 0.5 m`), so the published request is capped at the unit-gain
pull to the lowest profile speed ahead, and at the pull to the near floor so the two
channels agree.

## `publish_ramp` and the op-long budget

The plan `aTarget` wire means two different things. On stock ACC it is not an actuator
command: ICBM's decel-overshoot lever keys the dash gap on it and the ECU does its own
easing, so a source may ask deeper than any path can deliver, down to `A_PUB_MIN =
-2.0` (beyond that no path can follow anyway), ramped at `PUB_JERK = 2.0 m/s^3`. On
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
keys on `aTarget`.

## The route 135 decel chain (stock ACC)

What limits deceleration on the stock path is the ECU, not the planner. Route 135: a
median 2.65 s from a limiter taking the plan source to the car pulling -0.5 m/s^2, and
an MRCC ceiling around 0.75 to 0.8 m/s^2 under the overshoot lever, which is where the
stock budget comes from. Downstream of the lever, the budget and `COMMIT_FRAC` change
where the braking point goes, not how hard the car brakes; the `gap_v` lead in
`DECEL_OVERSHOOT_PARAMS` pays back the dash walk (`icbm.md`). On stock ACC the vision
target is therefore pre-positioned at the deepest dip on the horizon (a dash servo
cannot track a continuous profile in 1 mph taps) and the decel gap does the shaping; on
openpilot long the target leads `v_ego` by the required decel and never goes below the
slowest point of the plan, past which the P candidate is already railed at the budget.

## Constants

| name | value | measurement | route |
|---|---|---|---|
| `_A_LAT_REG_MAX` | 2.0 m/s^2 | lateral acceleration ceiling | n/a |
| `_PLAN_MARGIN` | 0.95 | 13% -> 5% of fair apexes above 2.2 for 1.4% speed | corpus sim |
| `COMMIT_FRAC` | 0.7 | swept with the margin | corpus sim |
| `_RELEASE_FRAC` | 0.3 | hysteresis against gate chatter | n/a |
| `_NEAR_T` | 3.0 s | measured near field | n/a |
| `_NEAR_FLOOR_FRAC` | 0.98 | curvature noise band | n/a |
| `_KAPPA_BIAS_D` / `_GAIN` | 0..110 m -> 1.0..1.5 | ratio 1.00 / 0.79 / 0.30 at 30 / 80 / 130 m, cap where replay stops buying apexes | route 135, 26 apexes |
| `_KAPPA_BIAS_V_BP` | 22.4 to 26.8 m/s | fitted on a 30 to 50 mph road | route 135 |
| `_PLAN_HORIZON_D` | 300 m -> 0 (near window) above the band | 7/13 and 9/15 false commits above 60 mph; near window leaves 3 and 4 | 275 highway minutes, 350 segments |
| `_OP_LONG_A_BUDGET` | 1.2 m/s^2 | `A_CRUISE_MIN` | upstream |
| `_STOCK_A_BUDGET['mazda']` | 0.75 m/s^2 | overshoot saturation | 422k samples, 447 segments |
| `_STOCK_A_BUDGET_DEFAULT` | 0.5 m/s^2 | estimate, safe direction | n/a |
| `_STOCK_RESPONSE_T` | 1.0 s | estimate, erring large | n/a |
| `_SERVO_WALK_RATE['mazda']` | 4.0 mph/s | 4.1 hold frames, 3.8 taps | route 126 |
| `A_PUB_MIN` / `PUB_JERK` | -2.0 m/s^2 / 2.0 m/s^3 | stock publication depth and ramp | n/a |
| `D_FLOOR` | 0.5 m | division floor for a constraint at the bumper | n/a |
| `_T_FALLBACK` (map) | 2.8 s | decel horizon for a degenerate distance | n/a |
| MRCC decel ceiling | ~0.8 m/s^2; 2.65 s median to -0.5 | stock ACC response chain | route 135 |

## Tried and rejected

- The lateral-acceleration-percentile heuristic. Used the model's velocity plan, so a
  planned slowdown lowered the prediction below the abort threshold mid-braking.
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
  commits (table above).
- Publishing -2.0 on openpilot long. Bypassed `A_CRUISE_MIN` through the MPC seed.
- Publishing `a_ego` from the map and SLA sources. Map curves and map limits never
  braked the real car on stock ACC.
- Freezing the retained map target's distance at commit time. Under-requested more the
  closer the car got.
- Sizing the stock actuation lead from the 5 mph hold grid. Forged holds never snap.
