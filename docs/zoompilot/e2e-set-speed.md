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
lift -> gain * fade * max(0, floor - a_model)      rise at upstream's cruise jerk, fall 2.4 m/s^3 or the gain's share
floor = min(a_mpc, a_cruise, approach_accel(v_ego, v_target, personality))
gain  = interp(a_model, [-0.3, -0.1], [0, 1]) * fade-in over 5-8 m/s
fade  = interp(max(steer lat accel, v_ego^2 * max path curvature over 10 s), [0.6, 1.0] m/s^2, [1, 0])
```

The floor is chill mode's answer: the lower of the planner's own lead MPC candidate (this frame's
solution, as the host takes it) and its cruise candidate, upstream's chill cruise law
(`clip(v_target - v_ego, -0.5, ACCEL_MAX)` at upstream's cruise jerk) toward the target after SCC
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

The approach table is Standard. Relaxed and Aggressive move it 0.15 m/s^2 either way, the step
between personalities in sunnypilot's accel boost (#2089, 0.1 / 0.25 / 0.4), added before the gap
fraction so the step shrinks near the target; behind a lead the personality acts through the MPC
candidate's gap. On the CX-5, 1 m/s under the set speed, Aggressive is over the 0.42 kickdown point
below about 21.5 m/s (Standard below 17).

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
| `FADE_LAT_BP`, `LAT_HORIZON` | 0.6-1.0 m/s^2, 10 s | the envelope's 1.0 bend ceiling, as a ramp |
| rise | 1.6/1.2/0.8/0.6 m/s^3 at 0/10/25/40 m/s | upstream's cruise jerk (`op_long_cruise_jerk`): the lift stands in for cruise, on any car. Mazda's AccelShaper still caps the build at stock MRCC's 0.8 |
| `APPROACH_V` over `APPROACH_BP` | 1.5, 1.75, 1.45, 1.05, 0.85, 0.65 m/s^2 at 0, 4, 9, 14, 18, 25 m/s | the CX-5's stock MRCC accelerating command p99 by speed, no lead (its wire ceiling eases 14 m/s to 0.95 for a 4>3 kickdown; that stays in the Mazda port) |
| `APPROACH_GAP_FRACTION` over `APPROACH_GAP_BP` | 0.38, 0.47, 0.57, 0.72, 1 at 0, 1, 3, 5, 10 m/s under the target | between stock's p50 and p90 by shortfall: 0.31/0.37 at 0.5-1.5 m/s, 0.41/0.44 at 1.5-3, 0.51/0.55 at 3-5, 0.64/0.76 at 5-10 (20-24 m/s) |
| `LIFT_FALL` | 2.4 m/s^3 | at 0.5 the lift was still over 0.2 for 1.8% of frames with the model braking past -0.5; at 2.4, none |

## Expected behaviour

What the car should feel: the lift picks up within a second or two on an open road and holds the
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
