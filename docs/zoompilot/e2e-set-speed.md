# Set speed in experimental mode

Code: `openpilot/sunnypilot/selfdrive/controls/lib/e2e_set_speed/controller.py`, called from
one hook after the e2e candidate in `openpilot/selfdrive/controls/lib/longitudinal_planner.py`
through `LongitudinalPlannerSP.update_e2e_target`. Toggle "Experimental Mode Speed Assist" ("speed assist" on mici) (`ExperimentalModeSetSpeed`), off
by default, under Cruise > Alpha Longitudinal (tizi, mici, sunnylink), greyed while Dynamic
Experimental Control is on. Logged in
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

1. Take the speed this controller has added back out of the plan. `added` integrates the part of
   the boost that reached the car: the planner reports its output after choosing (`delivered()`,
   from the SP planner's publish), clipped to the boost so the gap assist's lift or a lower
   candidate taking over counts as none of ours. The model sheds it at 0.038 /s in cruise.
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
it pauses.

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

## Closed-loop scorecard

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

## Constants

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

## Expected behaviour

What the car should feel: the floor picks up within a second or two on an open road and holds the
set speed; approaching a bend, a stop or a slowdown the model plans, the extra speed comes off
gently (at most 0.5 m/s^3, 0.6 m/s^2) a few seconds before the model starts its own slowdown,
which then runs as it would have without the floor. Leads still cut the boost.

On the car: watch turn and yellow-light approaches. `e2eSetSpeed.added` should fall toward zero
before each one; `e2eSetSpeed.boost` going negative is the hand-back. If the car still arrives
fast, lower `CURVE_LAT_ACCEL` or `A_DEC`; if it feels hesitant on open road, look for `bound`
under `floor` with nothing ahead.

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
  0.4 mph. The envelope's hand-back is what makes `TAU` 6 s safe now.
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
