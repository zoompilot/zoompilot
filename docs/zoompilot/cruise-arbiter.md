# Cruise arbiter

Code: `openpilot/sunnypilot/selfdrive/car/cruise_arbiter.py` (session and press
classification), `cruise_ext.py` (increment hooks and the dash reconciler),
`card_ext.py` (call sites), `controls/lib/speed_limit/assist_mirror.py` (plannerd mirror),
`controls/lib/speed_limit/speed_limit_assist.py` (the pcm-op-long machine, unchanged
upstream design). Tests: `car/tests/test_cruise_arbiter*.py`, `car/tests/test_icbm_reconcile.py`,
`car/tests/test_icbm_sla_*.py`, `speed_limit/tests/`.

## Setpoint ownership

There are three kinds of car, and the setpoint has a different owner on each.

| class | CarParams | who steps the set speed | SLA session runs in |
|---|---|---|---|
| pcm-op-long | `openpilotLongitudinalControl and pcmCruise` | openpilot (set speed held at the required max) | plannerd, `SpeedLimitAssist` |
| ICBM (stock ACC buttons) | `pcmCruise and not pcmCruiseSpeed` | the stock ECU, on every wheel press | card, `CruiseArbiter`; mirrored by plannerd |
| op-long without pcmCruise | `not pcmCruise` | openpilot's `v_cruise` | card, `CruiseArbiter`; mirrored by plannerd |

`CruiseArbiter.applicable` is exactly "not pcm-op-long", and a test asserts it equals
the set plannerd mirrors, because a mismatch leaves a car mirroring a permanently
disabled session. `op_owns_setpoint` (`not pcmCruise`) picks the dismiss behaviour below.

## The session model

The arbiter runs at 100 Hz inside card's `VCruiseHelper.update_enabled_state`, in the
same frame as the button events and the setpoint writer. It reuses the upstream
`AssistState` enum: `disabled`, `inactive`, `preActive` (confirm prompt open), `active`.

- `disabled -> ...`: after engagement (or a cluster change) a 0.5 s guard runs; then a
  cluster already at the limit activates silently, a known limit prompts, no limit
  goes `inactive`.
- `inactive`: a limit change re-prompts and clears `_driver_dismissed`; dialing the
  cluster onto the target latches `active` silently, but only once no press is held
  (latching mid-hold would cap a driver dialing past the limit) and only if the driver
  has not dismissed since the last limit change.
- `preActive`: resolved by a press (below), by dialing onto the target, or by the 5 s
  timeout (`inactive`).
- `active`: a limit change prompts when `confirm_needed_for_change` says so (below the
  confirm speed threshold every change prompts; at or above it a new target above the
  threshold auto-applies and only announces).

The published surface is `carStateSP.zoompilot.cruiseSession`: `state`, `vCap`, `lastIntent`,
`announceCounter`. `vCap` is the plan cap in m/s: the session target while active, the
frozen hold while prompting, `V_CRUISE_UNSET` otherwise, never 0 (the mirror treats 0
as a not-yet-received message). `announceCounter` is bumped on alert-worthy transitions
and never un-bumped, so the 20 Hz mirror cannot miss one.

### What a prompt freezes

A pending confirm prompt freezes speed at three altitudes, each with its own job:

1. The session cap. Prompting out of an active session keeps that session's last cap
   so the plan `min()` cannot release the dash toward the baseline while the driver is
   deciding. Prompting from idle publishes no cap at all.
2. The ICBM servo parks (`prompt_frozen`) with its restore patience held at zero, so a
   decline or timeout still waits out a full quiet window.
3. Card vetoes button emission with same-frame state (`gate_send_button`), because the
   servo's view of the session is one message hop stale and a frame could otherwise
   escape at prompt onset.

Why idle publishes no cap: a cap equal to the baseline changes no speed but does change
the plan source. The display-unit round trip (round to whole mph or kph, divide back)
lands a few mm/s under `v_cruise`, so SLA wins the `min()` by rounding error and
relabels `longitudinalPlanSource` as a limiter. That arms ICBM's decel overshoot against
an ordinary cruise convergence, the servo's own freeze banks the resulting gap for the
5 s window, and the timeout dumps it as a SET- burst. User report 2026-08-29 (routes
`...acdc83b60f/3` and `...821e28d2fa/12`): engaging near a known limit dropped the set
speed 2 to 4 mph roughly 5 s later, then walked it back. Before 2026-07-26 the two
compared exactly equal and cruise won the tie; the property is now explicit.

## Press classification

Every +/- press is classified exactly once, at its press edge, from the pre-frame
session snapshot (`state_prev_frame`). Downstream consumers (`press_owned` in the
increment path, the reconciler, the mirror, the servo) read the classification or the
published session instead of re-interpreting buttons.

| pressed while | class | resolves | v_cruise |
|---|---|---|---|
| session active | dismiss | at the press edge: `inactive`, `_driver_dismissed` | see dismiss semantics |
| prompt open | prompt | at release, or at the long-press tick (49 frames) | upward confirm adopts the limit |
| otherwise | normal | n/a | plain increment |

A prompt press confirms when it points the way the cluster has to move
(`compare_cluster_target`). An upward confirm raises `v_cruise` to the limit
(`adopted_this_frame`); a downward confirm leaves a baseline above the limit alone,
the active session caps the plan instead. A press against the confirm direction
declines: the session ends at once so the frozen hold releases, and the press still
counts as a normal increment. A prompt press that reaches long-press duration resolves
at the first tick instead of release, one frame before `cruise.py`'s first repeat tick
so the tick is already owned when it fires; the ECU is grid-stepping by then anyway.

`press_owned` takes the raw enumerant int because capnp `_DynamicEnum` instances do not
hash-match the ints `cruise.py` passes in. Releases stay in the press table through
their frame so `press_owned` still answers, and are swept the next frame.

## Dismiss semantics

A press on an active session dismisses it. What happens to the setpoint depends on who
owns it.

**ICBM cars** (`op_owns_setpoint == False`): the whole press is owned, no increment.
The ECU steps the dash from the session target, and the reconciler adopts that as the
new setpoint once the press settles. A `-` dismiss additionally arms the reconcile
floor (below).

**Op-long without pcmCruise** (`op_owns_setpoint == True`): nothing re-anchors on its
own, so the arbiter writes `v_cruise = min(baseline, cap)` (never above the baseline; a
CST auto-apply can leave the cap above the setpoint) and the press counts as a plain
step from there. Before this, a dismiss dropped the cap and left `v_cruise` at the old
baseline: one tap in a 45 zone and the car accelerated to 70, whichever way the tap
went. The long-press tick runs its grid snap from the re-anchored value too.

## The reconciler (ICBM cars)

The stock ECU keeps the real set speed and steps it on wheel presses while openpilot
integrates the same presses, so the two drift (grid-snapped long presses, gas-override
presses, trailing increments). `reconcile_setpoint_with_dash` adopts the dash around a
driver press, under three conditions:

1. **Agreement at press start**, decided per press before its own ECU effect lands: the
   dash was within 2 mph of the setpoint (normal cruising) or, with a session active in
   the pre-frame snapshot, within 2 mph of the session target (a settled re-anchor). A
   dash in transit matches neither: adopting it would destroy the baseline the servo is
   about to restore, since a press that aborts an SLA move knocks both regime gates idle
   on the spot. A refused first press does not taint a second press inside the same
   window whose dash does agree.
2. **Regime**: plan source is `cruise`, ICBM is not `increasing`/`decreasing`, and no
   prompt is pending (the frozen dash is not the driver's answer).
3. **Window**: 1.0 s after the last press, which absorbs the CX-5's trailing long-press
   increment (lands well inside 1 s).

The **floor** covers the case agreement refuses: a `-` press dismissing a session asked
for slower. Once the window settles, the setpoint is floored to the dash the ECU left
(never raised), whatever the regime, because a limiter or the servo can only have moved
the dash lower still. Without it the in-transit dash failed both agreement checks,
nothing re-anchored, and 3 s later the servo walked the dash 51 -> 60 inside a 45 zone.

When the arbiter itself wrote the setpoint (`adopted_this_frame`), the helper kills the
reconcile window before the reconciler runs, so the ECU's own +1 from a confirm press is
not re-adopted over the adopted limit.

## Ordering inside a card frame

`CruiseArbiter.step` (snapshot, classify, machine, cap) runs on the *computed* enabled
flag, not the raw one: on non-pcmCruiseSpeed cars "enabled" is suppressed until the
engaging button releases, and the session must run against the same notion the
increments use or it would start mid-engage-hold. Then the increment path runs,
consulting `press_owned`; then `CardExt.update_v_cruise_post` feeds the plan regime and
runs the reconciler, then publishes the session, all before `CS.vCruise` is read.

## Evidence drives

- Drive `0000000b`: t=415/461, `+` on a prompt above the setpoint left a +1 orphan with
  an inert active session (fixed by upward-confirm adoption); t=180.8, ICBM restored the
  dash toward the baseline while the confirm prompt was showing (fixed by the frozen
  hold); t=155.1, resuming with the setpoint already at the limit fired "Auto adjusting
  to speed limit" (activation that changes nothing is silent); t=187.05, dialing onto
  the limit activated with an alert and the same press's latch dismissed it one frame
  later (dial-to-target latches silently and survives its own press).
- Route 126 t=341: a driver +5 was reverted within 1.4 s by the servo (the SET+ grace in
  the ICBM servo, see `icbm.md`).
- The "seg16 F1" class: the confirm press's own dash step, seen by a 20 Hz session
  machine as a cluster change, tore down the session it had just created.

## Constants

| name | value | measurement | route |
|---|---|---|---|
| `DISABLED_GUARD_PERIOD` | 0.5 s | upstream SLA guard, kept at 100 Hz | n/a |
| `PRE_ACTIVE_GUARD_PERIOD` | 5 s | prompt window (upstream pcm machine uses 15 s) | n/a |
| `LONG_PRESS_FRAMES` | 49 | one frame before `CRUISE_LONG_PRESS = 50` | n/a |
| `RECONCILE_SETTLE_TIME` | 1.0 s | trailing long-press increment lands inside 1 s on the CX-5 2022 | ICBM corpus |
| `RECONCILE_AGREE_KPH` | 2 mph | dash-at-rest band for setpoint or session target | n/a |
| `CONFIRM_SPEED_THRESHOLD` | 80 kph / 50 mph | upstream | n/a |
| `get_minimum_set_speed` | 30 kph / 20 mph | stock ACC floor | n/a |
| `V_CRUISE_UNSET` | 255 | upstream sentinel; one fork copy in `speed_limit/__init__.py`, pinned by `test_cruise_constants.py` | n/a |

## Tried and rejected

- Four modules interpreting the same press across three processes (card, plannerd,
  selfdrived), bridged by wall-clock latches sized to the slowest consumer. Every
  shipped bug in the stack was a single press racing the 20 Hz SLA cycle, the reconcile
  window or the servo state; the phase sweeps in `test_icbm_sla_session.py` land the same
  press at six offsets across a full cycle to keep it that way.
- Running the non-pcm session machine in plannerd at 20 Hz. The confirm press's own dash
  step read as a cluster change and dismissed the session it created.
- Capping the plan at the cluster when prompting from idle. Numerically a no-op,
  but it relabels the plan source (see above).
- Adopting the dash whenever a press happened. An in-transit dash destroyed the baseline
  the servo was about to restore.
- Letting a `-` dismiss leave the baseline in place. The servo restored the old baseline
  3 s after the driver pressed "slower" in a lower zone.
- Dropping the cap on an op-long dismiss without re-anchoring `v_cruise`. One tap in a
  45 zone released the car to a 70 baseline.
- Latching dial-to-target while a press is still held. Capped drivers dialing past the
  limit.
