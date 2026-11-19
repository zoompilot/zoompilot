# The model's lead forecast in the long MPC

Code: `openpilot/sunnypilot/selfdrive/controls/lib/lead_forecast/forecast.py`. `LongitudinalPlannerSP`
wraps the MPC instance's `process_lead` at init and refreshes the forecast in its `update()`, so
`long_mpc.py` and the upstream planner stay byte-identical. On by default with no setting: it is
how the planner models the car ahead, not a preference. `LeadForecast` (default 1) stays as a
hidden off switch for testing. Radar leads take it only with `LeadForecastRadar` (default 1 since 2026-11-06, hidden;
see [Radar leads](#radar-leads-leadforecastradar)). Logged in `longitudinalPlanSP.zoompilot.leadForecast` (per lead: blend weight,
inhibit reason, the x/v trajectory the MPC got). Tests: `lead_forecast/tests/`. Tools:
`tools/mazda_long/lead_event_index.py` (cached per-frame index of every engaged segment, slowing
leads, brake taps, cut-ins, following gaps), `tools/mazda_long/lead_forecast_replay.py` (A/B of
both MPCs over the index).

## The problem

The MPC's obstacle is the lead extrapolated from one instant: `x, v, a` from radarState, with the
acceleration fading as `a * exp(-tau t^2 / 2)`. radard gives a camera lead `tau = 0.3` (radar
tracks get a Kalman tau that starts at 1.5), so a braking camera lead is assumed to keep 96% of
its deceleration at 0.5 s, 55% at 2 s and 9% at 4 s, whatever it is actually doing. Under alpha
long the Mazda radar is silenced and every lead is a camera lead.

The model already forecasts each lead's speed and position at 0, 2, 4, 6, 8 and 10 s
(`leadsV3`). comma's openpilot#37824 (Shane Smiskol, draft since April 2026, stale) fed it to the
MPC; it never landed and targets the planner from before cruise became its own candidate.

## Forecast accuracy (35k samples)

Every alpha-long frame with a camera lead tracked continuously for the next 6 s (no position
jump over 3 m a frame), sampled every 0.2 s. Truth is the lead's own later vLead and position.
Speed error, mean / mean absolute, m/s; positive is optimistic (the lead predicted faster than it
turned out):

| case | n | horizon | constant speed | upstream | model |
|---|---|---|---|---|---|
| all | 11626 | 4 s | +0.05 / 1.53 | +0.13 / 1.04 | +0.14 / 0.89 |
| lead slows > 2.5 m/s by 4 s | 1334 | 4 s | +4.44 / 4.44 | +2.65 / 2.68 | +1.35 / 1.73 |
| same | | 6 s | +5.89 / 5.89 | +4.05 / 4.14 | +1.88 / 2.32 |
| lead speeds up > 2 m/s by 4 s | 1571 | 4 s | -3.24 / 3.24 | -1.63 / 1.67 | -0.73 / 1.18 |
| lead steady | 3573 | 4 s | -0.00 / 0.23 | +0.10 / 0.36 | +0.15 / 0.47 |
| brake tap (aLeadK < -1, speed back by 4 s) | 36 | 4 s | +0.17 / 0.43 | -2.72 / 2.72 | -2.48 / 2.71 |

By current lead acceleration, 4 s, p90 of the optimistic tail: aLeadK in [-2, -1) upstream +4.08,
model +1.77; [-1, -0.3) +3.46 vs +2.45. The model is better exactly where braking starts, and
its optimistic tail is shorter, so no "take the more cautious of the two" layer is needed.

Position: the model's own `x` reads the lead too far, +0.55 / +0.88 / +2.09 m at 2 / 4 / 6 s
(+2.7 m at 6 s even behind a steady lead). Integrating the anchored speed forecast instead gives
-0.13 / -0.26 / +0.28 m with a lower mean absolute error at every horizon.

Slow leads: with the lead under 3 m/s and the model forecasting it 1 m/s faster within 4 s, the
lead did speed up 75% of the time and stayed put 25%. With lead and ego both stopped, 99% of the
predicted launches happened. Above 3 m/s, a predicted speed-up held 85% of the time and reversed
2%.

## What the MPC gets

For leadOne and leadTwo, paired with `leadsV3[0]` and `[1]` as radard pairs them:

```
v(t) = max(0, vLead + allowance * (model v(t) - model v(0)) where that change is > 0,
                vLead +             (model v(t) - model v(0)) where it is <= 0)
allowance = max(interp(vLead, [3, 8] m/s, [0, 1]), launch) * interp(aLeadK, [-1, -0.3] m/s^2, [0, 1])
x(t) = max(dRel, upstream's min_x_lead) + integral of v(t)
```

`launch` is the lead's launch gate weight, 0 unless the lead was measured pulling away from a stop
(see [Drive-offs](#drive-offs-the-launch-gate)).

on the MPC's 13 nodes (v linear between the model's knots, x integrated exactly). The forecast
is used only when the radarState lead is present, not a radar track, the model arrays are six
finite points, and the radarState lead is the model's lead (`|x[0] - 1.52 - dRel| < 4 m`).
Otherwise the MPC gets upstream's extrapolation. Switching either way blends linearly over
0.25 s (`BLEND_RATE` 4/s); while fading out, the last forecast's shape rides on the current lead.
FCW's crash check reads the same trajectory, as in the PR.

## Replay A/B

`lead_forecast_replay.py` runs upstream's MPC and the forecast one side by side. Each frame both
are seeded with the car's planner state: the start state (the logged plan's first point), the
previous plan the change cost pulls toward, and one shared solver iterate. So until the first
frame they disagree, both see exactly what the car saw; first-crossing times are exact, later
frames show what each would ask from where the car actually was. Drives today's upstream MPC does
not reproduce to 0.1 m/s^2 with the logged personality are left out: builds from before cruise
left the MPC (it was a third obstacle until mid-2026) plan differently.

Camera leads only (0.56 h on builds that reproduce; radar leads keep upstream's path), 38
slowing-lead events (vLead drops >= 3 m/s within 8 s and stays down), times from the lead's decel
onset:

| | -0.3 m/s^2 p50 | p90 | -1.0 p50 | p90 | never -1.0 | min p50 | FCW |
|---|---|---|---|---|---|---|---|
| upstream | -1.10 s | +3.05 | +1.05 | +3.75 | 17 | -1.13 | 0 |
| forecast | -1.70 s | +2.65 | +0.95 | +3.43 | 15 | -1.14 | 0 |

DEC's crash input (`crash_cnt > 0`, which flips it to blended) never fired on either arm behind
a camera lead, nor did FCW (`> 2`).

Paired, the forecast reaches -0.3 earlier in 35% of events (p10 1.41 s earlier) and later in 14%
(p90 0.12 s later); -1.0 p10 1.30 s earlier. Peak deceleration is unchanged (paired p50 0.00
m/s^2), so braking starts earlier without getting harder. Across all camera-lead following frames
the MPC candidate differs by more than 0.3 m/s^2 in 0.5% of frames (p1 -0.23, p99 +0.17).

Brake taps (3 on reproducible builds, 20 on all): the forecast brakes slightly more, not less
(min candidate p50 -1.42 vs -1.39; -1.26 vs -1.16 on all).
The model forecasts the tap's slowdown about as long as upstream's extrapolation does (table
above). Cut-ins followed by the new lead accelerating: none on camera leads in the logs.

## Constants

| name | value | from |
|---|---|---|
| `SPEEDUP_V_BP` | 3, 8 m/s | predicted launches of slow leads: 25% wrong under 3 m/s, 2% reversed above |
| `SPEEDUP_A_BP` | -1, -0.3 m/s^2 | a braking lead forecast to speed up: 1.2% of braking-and-closing frames, but under -1 m/s^2 a quarter kept slowing |
| `MATCH_DIST` | 4 m | radarState's camera lead is `x[0] - 1.52` of the same or previous model frame |
| `BLEND_RATE` | 4 /s | full switch in 0.25 s; no step in the obstacle |
| `LAUNCH_V`, `LAUNCH_A`, `LAUNCH_T` | 0.1 m/s, 0.3 m/s^2, 0.1 s | launch gate: earliest setting that kept added false starts at 2 stops (see Drive-offs) |
| `LAUNCH_STOPPED_V`, `LAUNCH_RECENT_T` | 0.5 m/s, 2 s | the gate opens only for a lead stopped within the last 2 s |
| `LAUNCH_STALL_A`, `LAUNCH_STALL_T`, `LAUNCH_DROP_V` | 0.1 m/s^2, 0.3 s, 0.3 m/s | stall: no longer accelerating while slow, or speed off its peak; the drop rule halved added false starts (4 to 2) at no delay cost |
| `LAUNCH_SWAP_DIST` | 3 m | a camera lead's range jumping this far in a frame is another car; restarts the gate |

## Radar leads (`LeadForecastRadar`)

On by default since 2026-11-06 (a 2021-24 Camry tester drives it first). The PR's Toyota process-replay segment that fired FCW on 575 frames (the reason radar
leads were left out) is a fixture artifact: in all three Toyota regen segments (`TOYOTA`,
`TOYOTA2`, `TOYOTA3`) `leadsV3.x` past t=0 is about 0 (`[52, 0.2, -0.3, 1.5, ...]`) while `v` is
sane, and the PR's position is `dRel + (x - x[0])` made non-decreasing, so its lead becomes a
stopped car at the current range. The replay reproduces it (35 FCW onsets on the three segments);
the forecast integrates `v` and fires none.

Accuracy, `tools/mazda_long/lead_forecast_accuracy.py --source radar`: every cached segment
(engaged or stock ACC), lead on the same radar track for the next 6 s, truth the track's later
vLead and dRel plus the ego's travel. 299k samples (0.2 s apart). Radar path = what the car runs:
the radar path where `radar_matches`, upstream elsewhere (4 s position error, mean / MAE /
optimistic p90, m):

| case | upstream | camera formula | radar path |
|---|---|---|---|
| all | +0.34 / 1.62 / +2.69 | -0.03 / 1.72 / +2.38 | +0.10 / 1.49 / +2.23 |
| lead slows > 2.5 m/s by 4 s | +4.21 / 5.19 / +10.09 | +3.95 / 4.88 / +10.10 | +2.78 / 4.12 / +8.48 |
| lead braking now (aLeadK < -1) | -0.88 / 3.67 / +4.63 | +2.47 / 4.19 / +8.93 | -1.28 / 3.53 / +3.78 |
| lead speeds up > 2 m/s | -1.78 / 3.83 / +3.40 | -4.77 / 4.99 / -0.19 | -2.35 / 3.11 / +1.47 |
| brake tap | -5.92 / 6.27 / +0.71 | -1.44 / 3.01 / +2.46 | -5.98 / 6.33 / +0.71 |
| steady | +0.35 / 0.79 / +1.42 | +0.27 / 0.80 / +1.45 | +0.27 / 0.79 / +1.39 |

Radar measures what the model's 2 s knots miss: the lead's deceleration now. So the radar path:

```
allowance = max(interp(vLead, [3, 8], [0, 1]), interp(aLeadK, [0.1, 0.5], [0, 1])) * interp(aLeadK, [-1, -0.3], [0, 1])
v_f(t)    = forecast_speed with that allowance, on the model's knots
v_m(t)    = upstream's extrapolation of the measured aLeadK (its clip and tau)
v(t)      = w(t) v_m + (1 - w(t)) v_f,  w = 1 -> 0 over RADAR_HANDOVER_T = 3 s
v(t)     -= floor * max(v - v_m, 0),    floor = interp(aLeadK, [-0.5, -0.1], [1, 0])
x(t)      = max(dRel, min_x_lead) + integral of v
```

- Handover 3 s: 2 s and 4 s were each worse on more cases (4 s is better on all-sample MAE, worse
  behind slowing leads).
- Braking floor: while radar measures the lead braking, never faster than upstream. Without it the
  forecast's optimistic tail behind braking leads doubles (table). It also turns brake taps back
  into upstream's (the camera formula ignored taps less; taps are upstream's behaviour, no worse).
- Launch lift: under 3 m/s a predicted pull-away is right 77% of the time with radar truth, as with
  the camera (75%), but a measured aLeadK > 0.5 is the lead already moving. Behind launches the 4 s
  position MAE drops from upstream's 7.2 m to 4.5 m.

Match (`radar_matches`): radard pairs a track with the model's lead loosely (25% of range, 10 m/s). The
forecast needs radarState modelProb >= 0.5 (radard's low-speed override, a close stationary track no
model lead confirmed, reports 0), range within max(5 m, 15% of dRel) of `x[0] - 1.52`, and relative
speed (`v[0] - modelV2.velocity.x[0]` vs the track's own `vRel`) within 2 m/s. On our radar: |dx| p50 1.1 m,
p99 10.9 m (16% of range); |dv| p50 0.6, p90 1.9, p99 4.6 m/s. Past 2 m/s the forecast's 4 s speed MAE
is worse than upstream's (2-4 m/s: 0.82 vs 0.76, over 4: 1.56 vs 1.18), under it better or equal.
The Toyota regen segments' older model sits further from the track (|dx| p50 4.4 m, 5% of range),
hence the 15%. A changed radarTrackId drops the last forecast's shape: a matched new track takes
its own forecast at the same weight, an unmatched one goes to upstream at once instead of fading
the old car's shape onto it.

FCW reads the trajectory the MPC solved against, as for camera leads.

Replay, `lead_forecast_replay.py --source radar --arms upstream,radar,radar_fcwm,pr`, 8.57 h
following radar leads on builds that reproduce, 232 slowing-lead events:

| arm | FCW onsets | crash_cnt > 0 onsets | -0.3 earlier / later | frames off upstream by > 0.3 m/s^2 |
|---|---|---|---|---|
| upstream | 1 | 3 | | |
| radar path | 0 | 2 | 14% / 0% | 0.0% |
| radar path, FCW on the measured path | 1 | 9 | 14% / 0% | 0.0% |
| PR | 1 (another) | 0 | 15% / 18% | 0.1% |

Peak decel and brake taps are unchanged (paired p50 0.00). The effect in the MPC is small because
the near term stays on the measured accel and the floor holds braking leads to upstream; the gain
is at 3-6 s. Toyota fixtures (three 1-minute segments, old model): no FCW, candidate within
-0.11/+0.02 m/s^2 (p1/p99) of upstream. With `LeadForecastRadar` off, the forecast is bit-identical
to the camera-only version on routes 0000004a-62 (101k frames).

Not covered by data: Toyota's radar with a current model (the only Toyota rlogs with modelV2 are
the regen fixtures; commaCarSegments is CAN only). A Camry 2021-24 tester drives it first.

## Drive-offs: the launch gate

The low-speed limit made the forecast the slowest of all at a drive-off. A lead pulling away from
a stop was held at its current speed until it passed 3 m/s, while upstream's extrapolation at
least rode the lead's measured acceleration. Behind a camera lead the plan left `should_stop`
0.65 s after upstream would have and the car moved 1.4 s after the lead passed 0.5 m/s. nicko's
`mpc-lead-trajectory` branch (openpilot#37824's process_lead on sunnypilot: no low-speed limit,
the model's own x) starts with the lead, but it also starts for the quarter of predicted
pull-aways that do not happen.

`LaunchGate` (one per lead) lifts the limit only once the lead is measured moving: within 2 s of
being under 0.5 m/s, speed at or above 0.1 m/s and acceleration at or above 0.3 m/s^2 for 0.1 s
(radarState's `vLead` and `aLeadK`: the model's current lead state on camera leads, the track on
radar ones). The lift is immediate, since it only moves the obstacle away. It ends when the lead
drops back under 0.1 m/s or 0.3 m/s below its peak, or stays under 0.1 m/s^2 for 0.3 s while under
8 m/s, and then fades at `BLEND_RATE`. A new car (radar track id, or a camera range jump over 3 m)
restarts the gate. Before the gate opens nothing changes, and FCW and `hasLead` are untouched.

On camera leads the motion shows 0.6 s (p50) before `vLead` passes 0.5 m/s. At that moment the
model's pull-away is unbiased: lead travel over the next 2 / 4 s, error mean +0.32 / -0.11 m, MAE
1.16 / 3.03 m (the limit was 2.1 / 10.1 m pessimistic, upstream 1.2 / 7.2 m; 110 drive-offs).

Validation (zoompilot-research `tools/mazda_long/lead_launch/`): every stop behind a lead in the
`lead_event_index` cache, engaged or not, 97.5 h of frames. 111 camera-lead and 384 radar-lead
drive-offs (radar ones are the Mazda radar on stock ACC drives), plus 40 / 112 min of the ego
stopped behind a lead that stays stopped for 2 s or more. Closed loop from the stop: each arm runs
the planner's MPC and cruise candidates, the logged e2e candidate (model `shouldStop` and accel)
while the car is where the log had it, and the car as the Mazda: StandstillHold's debounce, 0.9 s
to motion, then the plan through a 0.25 s lag. Against the logged plan's go, arm (a) is -0.05 s
at p50 (68% within 0.15 s) on the 37 engaged camera drives, which ran (a), and the upstream arm
-0.10 s (82%) on the 11 older engaged radar drives, which ran upstream.

Camera leads; times from the lead passing 0.5 m/s, gaps in m, "excess" is the gap less the MPC's
steady following distance while moving, false starts = stops where the plan asks to go while the
lead stays stopped 2 s and (a) does not:

| arm | plan go p50 (p10 / p90) | car moves p50 (p90) | gap 2 s / 4 s p50 | excess p10, first 6 s | FCW | added false starts |
|---|---|---|---|---|---|---|
| upstream (forecast off) | -0.45 (-1.20 / +0.00) | +0.75 (+1.20) | 8.2 / 12.9 | -0.2 | 0 | 1 |
| (a) before this change | +0.20 (-1.10 / +0.55) | +1.40 (+1.80) | 8.8 / 15.6 | +2.8 | 0 | 0 |
| (b) nicko / #37824 | -1.09 (-2.30 / -0.25) | +0.25 (+0.95) | 7.4 / 11.0 | -3.8 | 0 | 26 |
| (b2) (a) without the limit | -1.05 (-2.30 / -0.20) | +0.26 (+0.95) | 7.6 / 11.1 | -3.1 | 0 | 18 |
| (c) launch gate | -0.60 (-1.20 / -0.15) | +0.65 (+1.10) | 7.9 / 11.9 | -2.1 | 0 | 2 |

Radar leads:

| arm | plan go p50 | car moves p50 | gap 2 s / 4 s p50 | excess p10 | FCW | added false starts |
|---|---|---|---|---|---|---|
| upstream | +0.12 | +1.35 | 7.9 / 12.9 | -3.2 | 0 | 0 |
| (a) | -0.15 | +1.08 | 7.7 / 12.7 | -2.6 | 0 | 0 |
| (b) | -0.85 | +0.75 | 7.4 / 12.1 | -4.2 | 0 | 41 |
| (b2) | -0.65 | +0.90 | 7.5 / 12.4 | -3.6 | 0 | 23 |
| (c) | -0.20 | +1.05 | 7.7 / 12.7 | -2.9 | 0 | 2 |

So on camera leads the gate takes back 0.8 s of the 1.3 s between (a) and (b) for 2 added false
starts against (b)'s 26, and the car moves 0.75 s sooner. The rest of (b)'s lead is the model's
prediction before any motion shows, which is what the limit is for. On radar leads (a) already
lifted on measured `aLeadK` (`RADAR_LAUNCH_A_BP`) and the gate changes little. The gap runs closer
than (a) in the first seconds (excess p10 -2.1 m) because the plan now expects the pull-away that
the lead then makes; it never closes below the standstill gap and no arm fired FCW.

Grid, camera, plan go p50 / added false starts, before the drop rule: `LAUNCH_A` 0.2 instead of
0.3 gains 0.05-0.1 s and doubles false starts (7-9 vs 4); `LAUNCH_V` 0.1 vs 0.2 gains 0.2 s at the
same count; `LAUNCH_T` 0.05-0.15 moves 0.05 s either way. The drop rule (`LAUNCH_DROP_V`) then
took 4 to 2 at no cost. Rejected: flooring the limited forecast with upstream's extrapolation of
`aLeadK` (0.05 s earlier, one more false start, and it changes the forecast before any gate
opens), and the model's own `v[0]` / `a[0]` for camera leads instead of radarState's (the same
signal a frame sooner; it needs one more frame of persistence for the same false-fire rate).

Elsewhere nothing changes: on 97.5 h of frames the MPC's lead trajectories differ from (a) on
0.52%, all with a gate open or fading (a stop within the last 2 s). Open loop (logged seeding, as
`lead_forecast_replay.py`) within 10 s of any open gate, 0.56M frames: the MPC candidate is never
lower than (a)'s (p1 +0.01 m/s^2), differs by at most +0.09 above 10 m/s (36 frames), and FCW
frames are the same (11 and 11). Process replay of plannerd on five segments (three alpha long,
two stock ACC): three plan identically (one stock segment's reported forecast differs on 3 frames,
a lead pulling away at 1 m/s), and the two with stops differ only from the drive-off on (aTarget
up to +0.77, never lower, except -0.05 for 4 s afterwards from the planner's own carried state).

On the Mazda the car moves 1.1 s (p50, 1.05-1.4) after the plan leaves `should_stop`: 0.28 s until
the command leaves the hold (StandstillHold's 0.2 s debounce, 0.05 s above 0.3 m/s^2; the plan
crosses 0.3 in its first 0.2 s in 26% of gated launches, 5% before), then the breakaway ramp to
about 1 m/s^2 and the powertrain. That is the floor under any planner change here.

## Tried and rejected

- Model `x` for position (the PR): optimistic by 2 m at 6 s; integrated speed is unbiased.
- Allowing a forecast speed-up only for a lead already pulling away, at every speed: costs the
  speed-up cases where the model was right 85% of the time. The braking-lead case it guards is
  covered by `SPEEDUP_A_BP` instead. Under 3 m/s it is what the launch gate now does: there the
  limit alone held the car until the lead reached 3 m/s (see Drive-offs).
- Dropping the low-speed limit (openpilot#37824, nicko's branch; arms (b), (b2) in Drive-offs):
  0.5 s sooner than the gate at a drive-off, but 13 times the added false starts.
- The camera formula for radar leads: optimistic behind a lead radar measures braking, and far
  more pessimistic than upstream behind accelerating leads (see Radar leads).
- FCW against the measured path for radar leads (crash check on upstream's extrapolation while
  the MPC plans on the forecast): 9 crash_cnt onsets vs 3 for upstream and 2 for the forecast
  path, and it keeps the FCW the forecast path drops.
- Raising cruise max accel 1.6 -> 2.0 (rides along in the PR): works against the Mazda accel
  envelope in `mazda-longitudinal.md`.
- Gating on lead probability: the forecast beat upstream even at raw prob < 0.8 (6 s speed MAE
  1.42 vs 1.90); radard already drops leads under 0.5.
- Free-running replay (each MPC closing its own loop open loop in the car): the upstream arm drifted
  0.3-2 m/s^2 from the logged plan, and the MPC has more than one local solution with a distant
  lead, so an arm whose warm start drifted kept a different answer to identical inputs.
