# Following distance in experimental mode

Code: `openpilot/sunnypilot/selfdrive/controls/lib/e2e_lead_gap/controller.py`, chained after the
set-speed floor in `LongitudinalPlannerSP.update_e2e_target` (no new hunk in the upstream
planner). Toggle "Experimental Mode Follow Assist" ("follow assist" on mici)
(`ExperimentalModeLeadGap`), off by default, under Cruise > Alpha Longitudinal (tizi, mici,
sunnylink), greyed while Dynamic Experimental Control is on. Logged in
`longitudinalPlanSP.zoompilot.e2eLeadGap`. Tests: `e2e_lead_gap/tests/`. Tool:
`tools/mazda_long/e2e_lead_gap_replay.py` (`--from-index` replays every experimental segment
with a lead; `--logged` reads on-car results).

## The problem

Behind a lead the planner takes `min(mpc, cruise, e2e)`. The MPC holds the personality's gap
(`T_FOLLOW * v + 6 m + (v^2 - vLead^2) / 5`); the model keeps its own. In steady following
(lead steady, ego matched to it, lead below the set speed, no pedals) over 1.09 h of
experimental-mode alpha long with a camera lead, the gap beyond the MPC's was:

| personality | p10 | p50 | p90 | e2e binding |
|---|---|---|---|---|
| aggressive | +3.7 m | +56.9 m | +88.3 m | 89% |
| standard | -2.4 m | +7.5 m | +64.7 m | 49% |
| relaxed | -0.2 m | +30.4 m | +73.6 m | 89% |

growing with speed: +1.6 m at 5-10 m/s, +8 at 10-15, +27 at 15-20, +52 at 20-25.

A regression of the model's acceleration over 31k of those frames (vEgo > 8 m/s, steady camera
lead): `a = 0.088 + 0.0004 * dRel - 0.0031 * vEgo + 0.11 * (vLead - vEgo)`, R^2 0.63. It matches
the lead's speed and all but ignores the gap. It holds whatever gap it happens to have, which is
why it hangs back after any lead change, and why a lift that stops at the right gap stays there.

Set Speed Nudge stands down whenever there is a lead, so nothing closed this.

## The controller

```
e2e    = a_model + boost
target = authority * gain * weight * min(cap, max(0, a_mpc - a_model))
boost  = min(target, boost + 0.25 m/s^3 * dt, a_mpc - a_model)   rises slowly, falls at once
weight = interp(gap excess, [3, 10] m, [0, 1]) * interp(vEgo, [5, 8] m/s, [0, 1])
gain   = interp(a_model, [-0.4, -0.1], [0, 1])
cap    = 0.5 / 0.4 / 0.3 m/s^2  aggressive / standard / relaxed
```

`a_mpc` is the MPC candidate exactly as the planner takes it this frame, rebuilt from its
trajectories. The boost can at most tie it, so the planner's min() never follows closer or
pushes harder than chill mode would at that moment; the excess uses the closer of leadOne and
leadTwo. The gain fades the lift out as the model starts braking, so the car settles where the
model pushes back rather than overruling it.

Authority reuses the nudge's scheme (rise 0.5/s, fall 4/s, 3 s hold after the last trip) with
lead trips added: no lead or a lead change (position more than 3 m off where the last frame's
lead should be: cut-in, lane change ahead, vision swap) drop authority to zero at once; a lead
under 5 m/s, below 0.8 probability, braking (aLeadK < -0.5) or forecast to lose 1 m/s within
3 s; and the nudge's: FCW, hardBrakePredicted, forceDecel, a stop in the plan, gas or brake, model
braking (< -0.5), the model's plan losing 1 m/s over 5 s, lateral acceleration over 1 m/s^2 now or
on the plan, no throttle allowed, lane change, under 5 m/s.

## Results

Open loop over 103 experimental segments with a lead (60 min active): lifting 15% of the time
(9.3 min), boost median +0.37 m/s^2 (p90 +0.50) at a median gap excess of 23 m, e2e the binding
candidate in 96% of it, tied to the MPC 27%. Never while the model brakes below -0.2. Jerk p99
0.78 -> 0.83 m/s^3. Idle reasons: no lead 26%, lead under 5 m/s 20%, hold 16%, lead braking 7%,
uncertain 6%.

Closed loop (`test_closed_loop`, the fitted model, upstream's MPC, 0.5 s lag, steady lead at
20 m/s from 40 m beyond the gap): the model alone is still 29 m out after 90 s; with the assist
the excess is 18 m at 10 s, inside the band by 20 s and holds at ~3 m, peaking 1.7 m/s faster
than the lead. A lead braking at -1.5 m/s^2 removes the lift within 0.25 s.

## Constants

| name | value | from |
|---|---|---|
| `GAP_BP` | 3, 10 m | deadband for the model's wander; chill itself rides +4 to +12 m beyond |
| `SPEED_BP` | 5, 8 m/s | stop-and-go stays the model's; matches the nudge's fade-in |
| `CAP` | 0.5 / 0.4 / 0.3 | the personality's character; stays inside the Mazda accel envelope |
| `GAIN_BP` | -0.4, -0.1 | the model's pushback when closing: 0.11 per m/s of closing speed |
| `BOOST_RISE` | 0.25 m/s^3 | half the nudge's; it is closing on a car |
| `LEAD_JUMP` | 3 m | a tracked lead moves `vRel * dt` (0.1 m a frame at 2 m/s) |

## Tried and rejected

- Closing toward the personality gap by its own law (a P controller on the excess): duplicates
  the MPC and can disagree with it; tying to `a_mpc` keeps one definition of the gap.
- Requiring a 1 s continuous-track qualification before acting: the 3 s hold after a
  lead change already covers it.
- Merging with Set Speed Nudge under one toggle: different risk (closing on a car), so it has its
  own switch.
