# The model's lead forecast in the long MPC

Code: `openpilot/sunnypilot/selfdrive/controls/lib/lead_forecast/forecast.py`. `LongitudinalPlannerSP`
wraps the MPC instance's `process_lead` at init and refreshes the forecast in its `update()`, so
`long_mpc.py` and the upstream planner stay byte-identical. On by default with no setting: it is
how the planner models the car ahead, not a preference. `LeadForecast` (default 1) stays as a
hidden off switch for testing. Logged in `longitudinalPlanSP.zoompilot.leadForecast` (per lead: blend weight,
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
allowance = interp(vLead, [3, 8] m/s, [0, 1]) * interp(aLeadK, [-1, -0.3] m/s^2, [0, 1])
x(t) = max(dRel, upstream's min_x_lead) + integral of v(t)
```

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

## Tried and rejected

- Model `x` for position (the PR): optimistic by 2 m at 6 s; integrated speed is unbiased.
- Allowing a forecast speed-up only for a lead already pulling away: costs the speed-up cases
  where the model was right 85% of the time, and does nothing for the slow-lead launch case that
  needs it, which the low-speed allowance covers. The braking-lead case it guards is covered by
  `SPEEDUP_A_BP` instead.
- Forecast for radar leads: the radar measures the lead and the model's lead may be another
  object (the PR's Toyota segment fired FCW on 575 frames master never did).
- Raising cruise max accel 1.6 -> 2.0 (rides along in the PR): works against the Mazda accel
  envelope in `mazda-longitudinal.md`.
- Gating on lead probability: the forecast beat upstream even at raw prob < 0.8 (6 s speed MAE
  1.42 vs 1.90); radard already drops leads under 0.5.
- Free-running replay (each MPC closing its own loop open loop in the car): the upstream arm drifted
  0.3-2 m/s^2 from the logged plan, and the MPC has more than one local solution with a distant
  lead, so an arm whose warm start drifted kept a different answer to identical inputs.
