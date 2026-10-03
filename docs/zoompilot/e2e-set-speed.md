# Set speed in experimental mode

Code: `openpilot/sunnypilot/selfdrive/controls/lib/e2e_set_speed/controller.py`, called from
one hook after the e2e candidate in `openpilot/selfdrive/controls/lib/longitudinal_planner.py`
through `LongitudinalPlannerSP.update_e2e_target`. Toggle "Set Speed Nudge" (`ExperimentalModeSetSpeed`), off
by default, in the Alpha Longitudinal settings panel (tizi, mici, sunnylink), greyed while Dynamic
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
e2e = a_model + authority * gain * max(0, floor - a_model)
floor = clip((v_target - v_ego) / 8 s, 0, 0.6) * fade-in over 5-8 m/s,
        capped by upstream's friction circle and, without allow_throttle, the coast limit
gain  = interp(a_model, [-0.2, -0.05], [0, 1])
```

`v_target` is the planner's target after SCC vision, SCC map and SLA, so curve and limit
targets are never pushed past. Authority rises 0.5/s, falls 4/s on a trip, and re-arms only
3 s after the last trip; it freezes through that hold. The boost rises at most 0.5 m/s^3.
Off, not in e2e, long control reset, DEC active or invalid input: the model passes through
untouched and the state resets.

Trips: FCW, hardBrakePredicted, forceDecel, shouldStop or a plan dipping under 2 m/s, any lead,
gas or brake, model accel under -0.2, the plan losing more than 0.75 m/s over 5 s (from its own
v(0)), lateral acceleration over 1.0 now or anywhere on the 10 s plan, allow_throttle false, a
lane change, under 5 m/s.

DEC on this car is radarless (`radarUnavailable`), so its blended mode means FCW, standstill or a
predicted slowdown, and its acc mode drops the e2e candidate altogether: the floor has nothing to
add under DEC.

## Constants

| Name | Value | Measurement |
|---|---|---|
| `TAU` | 8 s | stock MRCC set-speed steps close with ~7.7 s |
| `FLOOR_MAX` | 0.6 m/s^2 | stock MRCC peak, median 0.57, p90 0.72 |
| `MIN_SPEED`, `FULL_SPEED` | 5, 8 m/s | v5 trip cannot see a stop below this; floor 1.2-1.6 against a creeping model |
| `AUTHORITY_RISE` | 0.5 /s | |
| `AUTHORITY_FALL` | 4 /s | p90 overridden decel on purposeful slowdowns 0.28 -> 0.11 m/s vs 2/s |
| `HOLD_TIME` | 3 s | override on purposeful slowdowns 21% -> 7% (open loop) |
| `BOOST_RISE` | 0.5 m/s^3 | removes 34 of 35 target-restore steps (48 in 2 h, p50 0.15, max 0.46) |
| `GAIN_BP` | -0.2, -0.05 | model accel sd 0.11-0.19 in cruise, below 0.3 Hz; purposeful pushes 11% -> 4% |
| `MODEL_BRAKE_ACCEL` | -0.2 | blocks 62% of purposeful slowdowns, 1.1% of drift |
| `PLAN_SLOWDOWN` | 0.75 m/s / 5 s | 60% / 2.9%; raw, a 0.3 s low-pass raised apex excess p90 2.0 -> 2.8 m/s |
| `LAT_ACCEL_MAX` | 1.0 on max(now, 10 s plan) | fires before 49/49 curve entries, median 5.1 s ahead |
| `STOP_SPEED` | 2 m/s | shouldStop fired before standstill in 2 of 29 stops |

vEgo reads 1.7-2.6% low against GPS on 2026 routes and the model's v(0) sits 0.3-1.4 m/s above
it, which is why the slowdown trip compares the plan with itself.

## Expected behaviour

Distance-domain closed loop over 22 min of the logs with the fitted 50 s model response and a
0.2 s + 0.5 s actuator: deficit median 1.55 mph (model alone 5.5), 4% of purposeful slowdowns
pushed more than 0.5 m/s, no boost at curve entry, apex lateral acceleration p90 unchanged at
2.43 m/s^2, jerk rms 0.20 vs 0.17 for the model alone.

The shipped controller replayed open loop over all 189 segments (118 min of active e2e at any
speed, leads included; `e2e_set_speed_replay.py`): boosting 20% of the time, median +0.26, p90
+0.51 m/s^2; off for a lead 20%, in the hold 15%, a stop 13%, lateral 9%; jerk p99 0.86 -> 0.93
m/s^3; full authority while the model asked for under -0.05 in 7% of the boosting time.

The residual: about a third of the model's gentle anticipations (min accel about -0.24, a turn,
stop or lead 15-30 s later, ~13 an hour) get overridden by a mean 0.34 m/s. No model signal
separated them from drift at an acceptable false-block rate; the hard trips still fire before
the turn or stop, curves p10 2.9 s ahead, stops 3-9 s.

On the car: watch turn and yellow-light approaches, and `e2e_set_speed_replay.py --logged` for
model accel under -0.05 held at full authority. If that is common, lower `FLOOR_MAX` or raise
`TAU`.

## Tried and rejected

- IQ.Pilot's version (`git.konn3kt.com/IQ.Lvbs/IQ.Pilot`, `get_e2e_accel`, param
  `expSpeedConv`): `min((v_cruise - v_ego) / 15, 0.5)` gated on a +-0.05 m/s^2 ramp of the
  model's accel and on model v(5 s) against vEgo, no lead only. The ramp sits inside the model's
  wander, doubling jerk (rms 0.16 -> 0.33, p99 0.55 -> 1.25 m/s^3); it closes at the -0.05 the
  model pushes back with, leaving 3.5-4 mph; the vEgo comparison is biased open; and the hard
  cutoff on lead status steps the output when a vision lead flickers (half of them last under 1 s).
- A -0.3 trip with an instant fall and no hold: the model spends 17% of cruise in (-0.3, -0.05]
  starting slowdowns, gates flicker and authority re-armed mid-decel; 21% of purposeful slowdowns
  overridden by more than 0.5 m/s.
- Faster convergence (tau 5-6 s): overrides 44% of anticipations for 0.4 mph.
- A headway taper with leads: leads were slower than the target in 87-96% of lead frames; only 3%
  of lead time would reach the set speed. Lead present is a trip.
- ACC drives and the model only vetoes: the model sits below the ACC cruise accel 83-92% of the
  time, so a cruise-relative veto is always on; the intent-gated form is this controller with
  instant authority, and hunts.
- Making DEC the answer: radarless DEC only blends to slow down, and its mode flips step the
  output.
- Shifting the model's velocity plan: a floor in disguise, bypassed on action-head bundles.
