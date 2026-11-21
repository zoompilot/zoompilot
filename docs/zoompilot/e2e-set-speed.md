# Set Speed Assist in experimental mode

Code: `openpilot/sunnypilot/selfdrive/controls/lib/e2e_set_speed/controller.py`, called from
one hook after the e2e candidate in `openpilot/selfdrive/controls/lib/longitudinal_planner.py`
through `LongitudinalPlannerSP.update_e2e_target`. Toggle "Set Speed Assist" ("set speed assist"
on mici, badge "speed") (`ExperimentalModeSetSpeed`), on by default since 2026-10-26, under Cruise >
Alpha Longitudinal (tizi, mici, sunnylink), locked outside experimental mode. Lead Follow Assist
(`docs/zoompilot/e2e-lead-gap.md`) was folded into it on 2026-10-27. Logged in
`longitudinalPlanSP.zoompilot.e2eSetSpeed`. Tests: `e2e_set_speed/tests/`. Tool:
`tools/mazda_long/e2e_set_speed_replay.py`.

## The problem

In experimental mode the planner takes `min(mpc, cruise, model.action.desiredAcceleration)`.
The driving model has no set-speed input, so with no lead the car runs at the model's pace.
Across 189 experimental-mode alpha-long segments (July to September 2026, RDFM, CTV3M, the
stock model, FM and CGWM; mostly 35-50 mph arterials), lead-free straight cruise above 15 m/s
was model-limited 90% of the time, a median 5.5-6 mph under the set speed and more than 3 mph
under for 77% of it. The model's acceleration there sat at a median +0.02 m/s^2.

The model does not defend its pace hard. After 43 set-speed steps in ACC mode (the car's speed
set by ACC, not the model) its acceleration moved -0.0196 per m/s of extra speed (95% CI -0.031
to -0.008), a ~50 s time constant, holding 1-40 s after the step; per bundle DTRV6 ~25 s, CTMV2
~65 s. Gas-override releases that left the car 5-8 m/s above its pace drew no more than -0.05.
It accelerates lazily and then holds. So the fix is to supply the missing pull, not to fight a
controller.

Nothing above the e2e candidate bounds it without a lead: the MPC follows a synthetic lead 50 m
ahead running 10 m/s faster (logged candidate p50 +1.98 m/s^2), and the e2e cruise candidate has
no lateral or coast cap. Whatever the floor adds is the output, so its trips are the whole safety
case. With a lead the MPC only binds near its own ~2 s gap, while the model paces leads at p50
2.3 s, so a boost under a lead mostly shortens the gap.

## The controller

```
e2e   = a_model + lift
lift -> gain * fade * gap,  gap = max(0, floor - a_model)    e2e rises at most 0.2 m/s^3 (or as a_model does), lift falls max(2.4 m/s^3, (1 - gain) * lift per frame)
floor = min(a_mpc, a_cruise, approach_accel(v_ego, v_target, personality))
gain  = min(interp(a_model, GAIN_BP, [0, 1]), planFade) * fade-in over 5-8 m/s,  GAIN_BP = [-0.3, -0.1] m/s^2
planFade = interp(-slowdown / 10 s, GAIN_BP, [0, 1])
slowdown = max(v_ego - min(plan v over 10 s), closing speed * interp(time to contact, [15, 25] s, [1, 0]))
fade  = interp(max(steer lat accel, v_ego^2 * max path curvature over 10 s), [0.6, 1.0] m/s^2, [1, 0]),
        falling at most 0.3 m/s^3 * dt / gap a frame
```

### Rise

With `k = gain * fade`, the e2e candidate is a blend of the model and the floor:

```
e2e  = a_model + k * gap = (1 - k) * a_model + k * floor
e2e' = (1 - k) * a_model' + k * floor' + k' * gap
```

Everything that raises it comes through there: `k` coming back (the gain, the fade), the floor (a
lead pulling away, a higher set speed, the cruise candidate after engaging), the model. Since
2026-11-21 one limit covers them: the candidate rises at most `RISE_JERK` over its last value, and
faster only as the model's own acceleration does (the lift is never negative, so the model is never
held back). The limit is on the candidate, not the lift, so under a steady candidate the lift still
takes up the model's wander at once. Falls are untouched: the gain lets go at once, a lead that
brakes binds through the MPC candidate.

From 11-14 to 11-21 the limit was on `k` instead (an "authority" that re-engaged at most
`0.3 m/s^3 / gap` a second) with the lift rising at upstream's cruise jerk. That bounded `k' * gap`
and nothing else, and it did not hold on engaging: the planner seeds its cruise candidate at the
car's acceleration, so the gap starts closed, the authority filled in 4 frames, and the lift then
rode the cruise candidate up at 0.8-1.2 m/s^3. Replayed on the 11-0x drives (105 min of Set Speed
Assist, routes 9b-ab), engaging 1.5 m/s or more under the target with the model under 0.3 m/s^2
put the plan up 0.50 m/s^2 in the first second; the CX-5 kicks down at 0.35-0.42.

The value is from the CX-5's stock MRCC (routes b4-bb, 2026-11, 173 min): its command's rising jerk
while accelerating is 0.08 m/s^3 at the median, 0.31 at p90, and 0.6 at its limit, which it holds
from 0.1 to 0.3 m/s^2 with no lead (0.13 behind one). Replayed open loop on 9b-ab with the approach
jerk below:

| | lifted, lead: jerk RMS / p99 / rising p99 | lifted, no lead | all SSA RMS | plan over 0.42 at 14-24 m/s | engage, first 1 / 2 s | lift kept |
|---|---|---|---|---|---|---|
| authority 0.3, approach 0.065 | 0.225 / 0.83 / 0.65 | 0.217 / 0.88 / 0.73 | 0.221 | 9.2% | +0.50 / +0.57 | 100% |
| authority 0.3, approach 0.035 | 0.203 / 0.77 / 0.61 | 0.188 / 0.77 / 0.59 | 0.213 | 6.4% | +0.43 / +0.42 | 69% |
| rise 0.3 | 0.186 / 0.55 / 0.30 | 0.166 / 0.44 / 0.30 | 0.210 | 6.3% | +0.12 / +0.42 | 67% |
| **rise 0.2** | **0.170 / 0.52 / 0.20** | **0.149 / 0.42 / 0.20** | **0.207** | **6.1%** | **+0.06 / +0.26** | **65%** |
| rise 0.15 | 0.159 / 0.51 / 0.15 | 0.138 / 0.41 / 0.15 | 0.204 | 6.0% | +0.06 / +0.21 | 63% |

(m/s^3; "lift kept" is the mean lift, open loop, so it overstates the loss: a car left further
under keeps its lift longer.) The model alone is 0.190 behind a lead, so a lifted frame is now
smoother than an unlifted one. What is left behind a lead is upstream's: the lead MPC binds 50% of
those frames and the model alone 31%; the plan reverses by 0.2 m/s^2 or more 4.0 times a minute
there against 3.7 with the assist off (stock MRCC 3.0, on highways). In closed loop
(`test_closed_loop`), 5 m/s under the target: 0.56 m/s^2 at most (0.8 before), within 0.2 m/s in
14.5 s (10.2), overshoot 0.005 m/s.

The fade keeps a budget on the way down: it is a comfort gate (curve speed is SCC's, through the
floor, and the friction circle's) read from the path's sharpest bend over 10 s, which flickers on
14% of frames, so it falls at most `FADE_JERK / gap` a second and the flicker does not drop the
lift. Coming back it is free, and the candidate's rise paces it: a budget divided by the gap does
not hold at a closed gap (as above), and the rise limit covers that side. Against the two-sided
budget on 9b-ab the plan differs by over 0.02 m/s^2 on 0.17% of frames (mean +0.0001, jerk the
same, lift with a bend over 0.8 m/s^2 ahead 0.011 against 0.010).

The floor is chill mode's answer: the lower of the planner's own lead MPC candidate (this frame's
solution, as the host takes it) and its cruise candidate, upstream's chill cruise law
(`clip(v_target - v_ego, -0.5, ACCEL_MAX)` at upstream's cruise jerk, the gap smoothed over 1 s since
openpilot #39266, 2026-11-14) toward the target after SCC
vision, SCC map and SLA, and SCC's rebuild of it in a bend. The planner already takes the minimum
over both, so the lift can never put the e2e candidate above them: with nothing to slow for,
experimental mode drives and follows as chill does, on any car. Behind a lead that means the
personality's gap (the model alone followed a median 57 m further back on aggressive); a lead that
brakes binds through the MPC candidate at once. With no lead the MPC chases its synthetic lead near
+2 m/s^2 and the approach pace below (or the cruise candidate, when lower) is the floor. (Before
2026-10-27 the floor was its own gentler law, `clip((v_target - v_ego) / 6 s, 0, 0.8)` after stock MRCC, with a copy of
upstream's friction circle.)

The floor also closes on the target speed no harder than a production ACC
(`approach_accel(v_ego, v_target)`, the target after SCC and SLA). Upstream's cruise law asks 2.0
for any 2 m/s shortfall, and on the CX-5 the lift toward it drove 23 of 24 kickdowns on the
2026-10-30 drives. The Mazda port capped its whole command for this from 10-30 to 11-01
(mazda-longitudinal.md, "Gap to the set speed"); the cap only ever bound this lift. Replayed over
27 minutes (00000060, 62, 65) against that cap, the floor gives the same command (median difference
0.000, p99 +0.05), 1.9% of frames gentler by over 0.05 where the model eases off or a bend fades the
lift, and slightly less time above the 0.42 kickdown point at 14-24 m/s (7.1% against 7.4%).

Since 2026-11-08 the pace is a jerk-limited arrival, capped at the speed's ceiling:

```
approach = min(ceiling(v_ego), sqrt(2 * jerk(v_ego) * max(gap, 0)), gap),  jerk = 0.1 m/s^3 to 8 m/s, 0.035 from 14 m/s
```

the acceleration that, eased off at that jerk, reaches the target with none left, and within
2 * jerk of it (0.07 m/s at speed) the gap itself at 1 /s, a line through the target that goes
negative over it. The jerk was 0.065 at every speed until 2026-11-21, fitted to the table before it over all speeds; at highway speed
that asked more than stock MRCC's p90 from 1.5 m/s under. Stock's command with no lead by
shortfall (routes b4-bb, median / p90) against the two:

| speed, shortfall | stock | 0.065 | 0.035 |
|---|---|---|---|
| 18-24 m/s, 0.5-1.5 | 0.32 / 0.36 | 0.36 | 0.27 |
| 18-24 m/s, 1.5-3 | 0.40 / 0.46 | 0.53 | 0.39 |
| 18-24 m/s, 3-5 | 0.48 / 0.55 | 0.70 | 0.52 |
| 18-24 m/s, 5-10 | 0.51 / 0.66 | 0.81 | 0.67 |
| 12-18 m/s, 5-10 | 0.72 / 0.88 | 0.92 | 0.71 |
| 30-38 m/s, 1.5-3 | 0.40 / 0.46 | 0.51 | 0.38 |

It stays under the CX-5's 0.35 kickdown point to 1.75 m/s under the target and under 0.42 to
2.5 m/s (0.9 and 1.4 before).

Under 14 m/s the jerk rises to 0.1 at 8 m/s, so pulling away from a stop behind a lead keeps its
pace: in the 12 s after leaving a stop (9b-ab, lead ahead) the plan at 5-8 and 8-12 m/s is 0.79 and
0.92 m/s^2 (0.76 and 0.91 with 0.065 everywhere, 0.72 and 0.84 with 0.035 everywhere, 0.70 and 0.80
with the assist off). Stock is brisker in town too (about 1.05 m/s^2 5 m/s under, 2026-10-30).
Under 5 m/s the lift is off and the lead MPC binds under the model (plan 0.42 against the model's
0.65), so the first car lengths are the MPC's and Lead Forecast's launch gate. Nothing from 14 m/s
up moves with it. A faster rise at low speed instead changed nothing (0.84 to 0.85): the pace, not
the rise, is what binds there.

Behind a lead at speed, by how far the car sits from the MPC's gap for the personality (9b-ab,
v > 12 m/s, plan mean / jerk RMS, assist off -> 0.065 with the authority -> now):

| | share | plan mean | jerk RMS | lead MPC binds |
|---|---|---|---|---|
| within 0.3 s of the gap | 13.2 min | -0.126 -> -0.105 -> -0.106 | 0.151 -> 0.155 -> 0.153 | 68% -> 87% -> 86% |
| 0.3-1 s back | 9.1 min | -0.022 -> +0.060 -> +0.044 | 0.170 -> 0.180 -> 0.173 | 24% -> 58% -> 47% |
| over 1 s back | 17.4 min | +0.037 -> +0.107 -> +0.079 | 0.148 -> 0.195 -> 0.161 | 2% -> 4% -> 3% |

At the gap the plan is the lead MPC's, with or without the assist, and as smooth as it; the lift
brings the car there (the model alone sits back, braking a little more than the MPC asks). Taking
the pace off the floor behind a lead closes harder (+0.215 over 1 s back) but at 0.248 m/s^3 and
with the plan over 0.42 in 8.4% of 14-24 m/s frames against 6.1%: not taken. From
2026-11-09 to 11-14 the gap line was left to upstream's cruise candidate, which the planner takes
the minimum with and which then was that line; dropping it moved the pace by at most 0.017 m/s^2
after the minimum (45% of frames, 9b-a7). openpilot #39266 (2026-11-14) smooths that candidate over
1 s, so on a closing approach it lags the shrinking gap and no longer binds near the target. The
line is back in the pace so the floor settles on the target by itself. In closed loop against the
real planner and MPC (0.2 s dead time, 0.4 s lag): arrival overshoot 0.017 m/s with it against
0.041 without, lift held on above the set speed on a +-0.3 m/s^2 grade swing 0.15 s per 120 s
against 4.45, and plan jerk within 0.3 m/s of the target 0.13 m/s^3 (0.21 before #39266); launch,
lead pull-away, engage under the target and bumps of 5 km/h or more are unchanged.
Upstream's own regulation under the floor is softer since #39266: a 5 mph set-speed or SLA drop
undershoots the new target by about 0.35 m/s (0.05 before), which the lift cannot change.
On those drives the target is not v_cruise on 20% of frames (SLA or SCC); were the cruise
candidate to head for v_cruise instead, a hold at a lower target flips as often under both laws
(+-0.05: 3.0 a minute). It replaced a table of
0.38-1 of the ceiling over 0-10 m/s under, which stepped from 0.38 to 0 at the target (a pumping
risk at the set speed); a 1 m/s taper to 0 (f554787d8, one day) cost 21% of the open-road lift.
The jerk matches the table's push on the drives' frames 2 m/s or more under the target (routes
9b-a7, 93f8b8d5ea; x1.02). Replayed on those drives with planFade: 8-11% of open-road lift and 15%
of all lift gone against the table (planFade alone 0% and 8%), none where the plan drops 3 m/s,
highway frames over the 0.42 kickdown point 8.5% against 8.2%, stop coast onsets as with the table.
At +-30% jerk: 21% / 2% of open-road lift gone, push x0.85 / x1.14. Closed loop with logged model
wander at the set speed: median 0.00 m/s under, worst 0.06, no more throttle-brake changes than the
table; from 5 m/s under, within 0.2 m/s in 10.2 s (table 11.7, taper 13.1), overshoot 0.01 m/s.

The ceiling is Standard. Relaxed and Aggressive move it 0.15 m/s^2 either way, the step between
personalities in sunnypilot's accel boost (#2089, 0.1 / 0.25 / 0.4), so they differ only well under
the target, where the ceiling binds (at 21 m/s from 5, 8 and 12 m/s under); behind a lead the
personality acts through the MPC candidate's gap. 1 m/s under the set speed every personality asks
0.26, under the CX-5's 0.35 kickdown point.

sunnypilot's boost itself is merged but unwired: replayed on the 10-27..10-30 drives it was on in
86% of cruise frames, and stacked on this lift put highway e2e over 0.35 28% of the time against 4.7%.

The lift is never negative:
every slowdown is the model's, and the lift only lets go of it, at 2.4 m/s^3 so the model's own
braking is never held off. A lane change drops it; off, not in e2e, long control reset or invalid
input pass the model through. Switched off with a lift on, it goes out at the fall rate. The only
state is the rate-limited lift.

Why each input:

- `gain` reads the model's own decision. Its acceleration wanders +-0.15 m/s^2 in cruise and
  settles near -0.05..-0.1 at the set speed, where it pushes back toward its pace, so the band sits
  below both: [-0.2, -0.05] crossed 5-16 times a minute near the set speed, [-0.3, -0.1] 1-4
  times, and [-0.5, -0.2] fights gentle slowdowns (mean lift 0.26 m/s^2 through them against
  0.10). The model's throttle probability (gasPressProbs[1]) is not used: in engaged cruise it is
  p50 0.93, the lift was 0.45 m/s^2 with it against 0.47 without, and where it is low the
  acceleration band has already closed the lift 92% of the time (2026-11-04, routes 4a-62).
- `planFade` is the same decision over the plan: the model's acceleration alone filled a gentle
  coast (-0.05..-0.25) toward a light, a stop or a closing lead up to the floor, turning it into a
  hold (0000004a t+521, 00000062 t+685). The slowdown the plan sets (its lowest speed in 10 s, or
  a closing radarState lead as it nears contact) is read as a mean deceleration through the gain's
  own band, so 1-3 m/s of drop fades it, above the ~1 m/s the far plan flickers on a straight
  road. It enters through the gain, so a planned slowdown lets go as fast as the model's own
  braking does, and like the gain it only ever lets go. The lead term is there because the lead
  MPC floor stays positive while closing at a distance (p50 +0.34 m/s^2 at 75 m, -3.8 m/s on
  routes 4a-62) and the plan shows only 30% of those slowdowns. On those drives: lift on
  in 0% of plan-drop >= 3 m/s frames (was 6%), 4.1% of open-road lift removed.
- `fade` reads the road's geometry, not the plan's speeds. On a straight road the plan's speed
  10 s out swings p90 2.5 mph per 0.5 s; curvature at the current speed flags 86-100% of real bend
  approaches and is over 1.0 m/s^2 on hindsight-straight road for 0.1-3% of frames. The model's
  desiredCurvature is not used: over 61 engaged minutes the path covers it (fade differs in 2.7%
  of frames, ~10 s of lift), but the path reads sustained bends 10-20% low, so the measured
  steering lateral acceleration stays.
- Leads, FCW, hardBrakePredicted and forceDecel need no case: the lead's MPC sits below the lift
  in the planner's min(), as it bounds chill mode, and the others drive the model's acceleration
  (and so the gain) down.

**Why not hand speed back.** From 10-08 to 10-27 the controller tracked the speed it had added and
handed it back (up to -0.6 m/s^2) wherever the plan showed a slowdown ahead (History, below). On
the car that pumped: on 10-27 drives 00000045 and 00000047 (CT V3), lead-free e2e cruise flipped
the command between throttle and brake ~5 times a minute against ~2 with the toggle off, 16-20%
of frames were hand-back, and for 37-57% of braking time the model alone would not have braked.
The plan's far end flickering a slowdown dumped the added speed, the flicker cleared, the boost
ramped back up, every 15-20 s, while the car still sat 5-6 mph under the set speed. The controller
was acting on the far plan with more confidence than the model itself (its acceleration stayed
near 0 through those flickers).

**What carries the safety case now.** Whether the model sheds speed it did not choose before a
feature is not settled. The 10-08 claim that it does not came from two drives; the closed-loop sim
assumes it (the model's response is its logged acceleration plus a 0.0196 /s pull), so its
"without give-back" numbers restate the assumption. On 10-27 data: 28 lead-free apexes showed no
slope of apex lateral acceleration on added speed (-0.03 +- 0.10 m/s^2 per m/s; full carry would be
~+0.14), but every high-added case had handed back first; the 10-07 drive with no hand-back entered
apexes at 1.61 against 1.44 m/s^2 with and without added speed (n = 5 and 4), consistent with about
a third carried. Stops have no samples. So bends rest on the fade, SCC vision's own ceiling (it
caps the cruise candidate whatever the lift does) and the friction circle; stops on the model,
whose plan ends at the line however fast it arrives.

DEC on this car is radarless (`radarUnavailable`), so its blended mode means FCW, standstill or a
predicted slowdown, and its acc mode drops the e2e candidate altogether: the floor has nothing to
add under DEC.

## Constants

| Name | Value | Measurement |
|---|---|---|
| `MIN_SPEED`, `FULL_SPEED` | 5, 8 m/s | floor 1.2-1.6 against a creeping model |
| `GAIN_BP` | -0.3, -0.1 | above |
| `HORIZON`, `LEAD_TTC_BP` | 10 s; 15-25 s to contact | the plan's horizon; a lead past 25 s out is the MPC's |
| `FADE_LAT_BP`, `LAT_HORIZON` | 0.6-1.0 m/s^2, 10 s | the envelope's 1.0 bend ceiling, as a ramp |
| `RISE_JERK` | 0.2 m/s^3 | the most the lifted candidate rises, unless the model's own acceleration does ("Rise"): inside stock MRCC's rising jerk (0.08 median, 0.31 p90, 0.6 limit) |
| `APPROACH_V` over `APPROACH_BP` | 1.5, 1.75, 1.45, 1.05, 0.85, 0.65 m/s^2 at 0, 4, 9, 14, 18, 25 m/s | the CX-5's stock MRCC accelerating command p99 by speed, no lead (its wire ceiling eases 14 m/s to 0.95 for a 4>3 kickdown; that stays in the Mazda port) |
| `APPROACH_JERK_V` over `APPROACH_JERK_BP` | 0.1, 0.035 m/s^3 at 8, 14 m/s | at speed, stock MRCC's median command by shortfall with no lead, 12-38 m/s (table above); in town, the pull-away pace of the 0.065 it replaced |
| `FADE_JERK` | 0.3 m/s^3 | how fast a bend ahead takes the lift off: the knee of the 11-14 replay (0.2 / 0.3 / 0.5) |
| `LIFT_FALL` | 2.4 m/s^3 | the slowest fall, for lane changes, bends and a lower floor. Chosen when it was the only fall: at 0.5 the lift was still over 0.2 for 1.8% of frames with the model braking past -0.5; at 2.4, none. The model's braking now also drops the lift through the gain |

## Expected behaviour

What the car should feel: the lift picks up over a few seconds on an open road and from engaging (at 0.2 m/s^3, 0.4 m/s^2 in 2 s) and holds the
set speed, and behind a steady lead it closes to chill's gap for the personality; the model's own easing off or braking takes it away at once, and nothing but the model
ever slows the car. Bends ahead fade it out before the car gets there.

On the car, lead-free e2e cruise: command sign flips no more than the model alone (2-3 a minute),
deficit near the set speed p50 under 1 mph, no frames with `boost` over 0.2 while the model brakes
past -0.5, lift under 0.1 within 1 s of the model passing -0.3. Then compare apex lateral
acceleration (about 30 apexes) and yellow-light approaches against the toggle off on the same
roads. If bends come in fast, lower `FADE_LAT_BP`; if it surges at the set speed, look at `gain`.
Behind leads: lead frames under 1 s gap against chill, and closing on a car the model sees but
radarState drops (on a camera-only car the MPC then reads the road as clear; chill does the same, and
the lift lets go only once the model brakes).

## Tried and rejected

- A stateless blend: `min((v_cruise - v_ego) / 15, 0.5)` gated on a +-0.05 m/s^2 ramp of the
  model's accel and on model v(5 s) against vEgo, no lead only. The ramp sits inside the model's
  wander, doubling jerk (rms 0.16 -> 0.33, p99 0.55 -> 1.25 m/s^3); it closes at the -0.05 the
  model pushes back with, leaving 3.5-4 mph; the vEgo comparison is biased open; and the hard
  cutoff on lead status steps the output when a vision lead flickers (half of them last under 1 s).
- The trip controller (10-03 to 10-08): authority rising 1/s and falling 4/s on trips, holds of
  3 s (hazards) and 0.5 s (model braking, plan slowing over 0.75 m/s in 5 s, lateral over 1.0),
  hysteresis bands while re-arming, and a full hold for plan slowing again within 3 s. It worked,
  but every trip was a snapshot that needed memory to stop flickering, and re-arm took 3.3 s.
- Gain ramps across the hysteresis bands instead of the bands (plan drop 0.5-0.75 m/s, lateral
  0.8-1.0 m/s^2), which drops the re-arming state: a signal sitting in the band boosts at partial
  gain instead of re-tripping, so the mean boost in the 5 s before a lateral trip rose 50% and
  boost while the model asked to slow 25% (281 exp-mode segments, 10-08).
- A -0.3 trip with an instant fall and no hold: the model spends 17% of cruise in (-0.3, -0.05]
  starting slowdowns, gates flicker and authority re-armed mid-decel; 21% of purposeful slowdowns
  overridden by more than 0.5 m/s.
- Faster convergence (tau 5-6 s) with the trip controller: overrides 44% of anticipations for
  0.4 mph. The envelope's hand-back was what made `TAU` 6 s safe then.
- A headway taper with leads: leads were slower than the target in 87-96% of lead frames; only 3%
  of lead time would reach the set speed. Lead present is a trip.
- ACC drives and the model only vetoes: the model sits below the ACC cruise accel 83-92% of the
  time, so a cruise-relative veto is always on; the intent-gated form is this controller with
  instant authority, and hunts.
- The rate bound (9deca743f6, 10-08): the plan's steepest average slowdown over 2-6 s and the
  curve speeds at 0.6 m/s^2 capped total acceleration. In closed loop it closed 69% of what was
  achievable and still carried speed into a quarter of the curve apexes: added speed persists, and a
  cap on adding cannot take it back. It also closed on any plan wobble of -0.1 m/s^2 however far
  under the target the car was.
- The envelope from the plan as logged, without taking our added speed out: it scales our own
  addition up again (curve apexes 45% faster than the model, against 10%).
- The plan's added speed shed at the cruise rate over its horizon, instead of held: tracks 3% better
  but curves 10.4 -> 12.6%, slowdowns p90 0.71 -> 0.97 m/s. The model does not shed before a
  slowdown.
- A time-gap bound for leads instead of the lead hazard: stops 30 m 3.3 -> 10% (dev). It boosts
  behind a lead that then stops.
- The model's predicted brake press fading the scale-up, and a margin from the plan's growing
  position uncertainty: no change, and 7% less tracking for a little stop margin.
- Making DEC the answer: radarless DEC only blends to slow down, and its mode flips step the
  output.
- Shifting the model's velocity plan: a floor in disguise, bypassed on action-head bundles.

## History: the speed envelope (2026-10-08 to 10-27)

Replaced by the lift above; kept for the record. Its constants:


```
e2e    = a_model + boost
boost -> gain * max(0, min(floor, bound) - a_model)     at most 0.5 m/s^3 either way
floor  = clip((v_target - v_ego) / 6 s, 0, 0.8) * fade-in over 5-8 m/s,
         capped by upstream's friction circle and, without allow_throttle, the coast limit
gain   = interp(a_model, [-0.2, -0.05], [0, 1])
bound  = the speed envelope (below), dropping at once and recovering with a 1 s filter
```

`v_target` is the planner's target after SCC vision, SCC map and SLA.

**The envelope.** The model keeps speed it did not choose: on drives with the boost on, its
acceleration did not respond to the boost's extra speed before a bend or a slowdown, and shed it
at only 0.04 /s in open cruise. So speed added on a straight arrives at the next bend unless
something takes it back, and a boost-only controller trades reaching the set speed against
carrying speed into features almost one for one. The envelope works from the model's own intent:

1. Take the speed added over the model back out of the plan. `added` integrates the lift on the
   e2e candidate that reached the car, this floor's and the follow-distance assist's alike: the
   SP planner reports its output and final e2e candidate after the host has chosen
   (`delivered()`, from its publish). The model sheds it at 0.038 /s in cruise. While a lead's MPC
   holds the car below the model itself, a model-only car would be held to the same limit, so it
   converges out over 2 s; cruise binding only under the lift does not count, since it would not
   have held a model-only car.
2. Ceiling at each plan point: the model's speed scaled up toward the target where the plan is
   flat, tapering back to the model's own speed where the plan has dropped 1.5 m/s below its
   start, and never past 1.0 m/s^2 of lateral acceleration in a bend the model takes slower.
3. A backward pass in distance gives the fastest speed at every point from which 0.4 m/s^2
   still meets every ceiling further on; the boost accelerates toward its value 2 s ahead.
4. Where the plan has dropped 0.5 m/s or more, the boost must also reach the model's speed there
   at a constant rate from now, so a slowdown shapes the boost as soon as it shows.

Over the envelope with speed it added, the boost goes negative: it hands back `added / 2 s`, at
most 0.6 m/s^2 and never more than it put on, so the car reaches each slowdown at the model's
own speed. It can never make the car slower than the model alone would have been. Behind a lead
nothing new is handed back (the MPC and the gap assist own the speed there), and a hand-back
already under way eases out at the normal rate rather than stepping.

Hazards cut a positive boost at 2.4 m/s^3 and hold it off for 3 s after they clear: FCW,
hardBrakePredicted, forceDecel, any lead, gas or brake, a lane change; leaving DEC or an
invalid model hold the same way. Stops need no hazard: a plan running down to zero closes the
envelope. Off, not in e2e, long control reset, DEC active or invalid input: the model passes
through untouched and the state resets.

State: the boost, the envelope's filter, `added`, the hazard hold. The telemetry's inhibit names
the hazard, `hold`, `modelBraking` (gain at zero) or `planSlowing` (the envelope under the floor).

DEC on this car is radarless (`radarUnavailable`), so its blended mode means FCW, standstill or a
predicted slowdown, and its acc mode drops the e2e candidate altogether: the floor has nothing to
add under DEC.

| Name | Value | Measurement |
|---|---|---|
| `TAU`, `FLOOR_MAX` | 6 s, 0.8 m/s^2 | stock MRCC closes in ~7.7 s at p90 0.72; with the envelope guarding features, 6 s / 0.8 tracks 8 points better at the same feature tails |
| `MIN_SPEED`, `FULL_SPEED` | 5, 8 m/s | floor 1.2-1.6 against a creeping model |
| `GAIN_BP` | -0.2, -0.05 | model accel sd 0.11-0.19 in cruise; nearly redundant with the envelope, kept as the most direct signal |
| `TAPER_DROP` | 1.5 m/s | without it stops 30 m 5.3 -> 7.9%, apexes 10.4 -> 11.9% |
| `CURVE_LAT_ACCEL` | 1.0 m/s^2 | 1.5 let apex excess 13 -> 18% |
| `A_DEC`, `LOOK_T` | 0.4 m/s^2, 2 s | LOOK_T 1 s: apexes 6.1 vs 4.9%, slowdowns 6.4 vs 4.3% (dev) |
| `SLOWING_DROP` | 0.5 m/s | 0.75 tracks 1 point better but slowdowns 13.7 -> 15.1% |
| `BOUND_TAU` | 1 s, recovery only | without it apexes 10.4 -> 15.6% |
| `SHED_RATE`, `GIVE_T`, `GIVE_MAX` | 0.038 /s, 2 s, 0.6 m/s^2 | shed rate measured; without give-back apexes 29%, stops 16% |
| `BOOST_RATE` | 0.5 m/s^3 | fall 1.0 -> 0.5: jerk p99 1.16 -> 1.04 at the same tails |
| `HAZARD_FALL`, `HOLD_TIME` | 2.4 m/s^3, 3 s | without the hold stops 10 m 8.8 -> 14.7% |

### Its closed-loop scorecard

`tools/mazda_long/e2e_set_speed_sim.py` re-drives every experimental-mode alpha-long stretch in
the logs (252 segments, about 3.5 h) with each controller and scores it against the same sim with
the boost off: tracking on road hindsight shows was clear, speed carried into curve apexes, stops
and slowdowns, time gap to leads, jerk. Routes are split in two; everything was tuned on one half.

| | Trip controller | Rate bound (9deca743f6) | Stateless blend | Envelope |
|---|---|---|---|---|
| holdout: clear-road deficit, mph (off 9.1) | 6.4 | 6.6 | 7.4 | **5.8** |
| holdout: apexes > 0.5 m/s faster | 30% | 25% | 28% | **17%** |
| holdout: 30 m before a stop > 0.5 m/s | 25% | 25% | 25% | **12.5%** |
| holdout: slowdowns > 0.5 m/s, p90 | 31%, 1.80 | 27%, 1.84 | **15%, 1.02** | 23%, 1.12 |
| holdout: lead frames under 1 s gap (off 0.10%) | 0.35% | **0.15%** | 0.32% | 0.23% |
| holdout: output jerk p99 / p99.9, m/s^3 | 1.24 / 2.38 | 1.23 / 2.23 | 1.43 / 4.80 | **1.21 / 1.70** |
| all: clear-road deficit, mph (off 8.9) | | 6.8 | | **6.0** |
| all: apexes / stops 30 m / stops 10 m | | 15.6 / 15.8 / 18.9% | | **9.6 / 7.9 / 8.1%** |
| all: slowdowns > 0.5 m/s, p90 | | 12.3%, 0.79 | | 12.3%, **0.64** |

Leads still cut the boost; the hand-back pauses behind one, where the MPC and the gap assist own the speed. A model bundle that pushes back hard (tau
25 s against the usual 50) plans a steady slowdown once past its pace, which the envelope cannot
tell from a real one: test_closed_loop shows it closing about half the gap there.


## Re-arm lag (10-07)

Historical: this tuned the trip controller that the bound replaced on 10-08.

On the first drive (`0000028a--4df8cbb84b`) the floor reached full authority exactly 5 s after
every trip (3 s hold, 2 s rise), and 18 of 29 clear stretches tripped again before getting
there, mostly on bends and easings. A single hold for every trip treats a curve that has ended
like a lead or a stop. Hazards keep the 3 s; the model's own slowdown signals re-arm after 0.5 s
with hysteresis, which is what the hold did for them (a signal hovering at its threshold). Red-light
approaches flicker plan slowing for 10-20 s, so a repeat inside 3 s gets the full hold; model
braking is not escalated because the gain already fades the boost out as the model brakes.
Leaving DEC (FCW, standstill or a predicted stop on this car) or invalid model output holds 3 s
like a hazard; engaging experimental mode gets the short hold, since a lead or stop trips at once.

Open loop over 281 experimental-mode segments: re-arm median 5.0 -> 2.3 s (p90 5 -> 4 s, the
hazards); boost in the 20 s before 48 stops unchanged (mean 0.13 -> 0.12 m/s); none in 12 min of
cornering over 1.0 m/s^2 either way; full authority while the model asked for under -0.05 12.1% ->
12.2% of boosting. Without the escalation the stops rose to 0.19 m/s and 17% over 0.3 m/s. A lead
as a soft trip was rejected: vision leads flicker and it added speed before leads appeared.
Lead Follow Assist keeps the old 0.5/s rise.
