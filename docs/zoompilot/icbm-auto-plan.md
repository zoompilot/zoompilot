# ICBM on demand: plan

Branch `icbm-auto` in zoompilot and opendbc, off `danger-unstable` (646c2f7694 / opendbc
53b2706022). Status: planning.

## Goal

Drop the ICBM toggle. ICBM is the actuator for other features, so it runs when one of them
asks for it:

- With no ICBM consumer enabled, a drive behaves exactly as it does today with ICBM off:
  same `carControl`, same CAN TX, same HUD.
- Turning a consumer on (say "slow for curves: vision") during a drive brings ICBM up at
  the next safe point in that drive, without an ignition cycle.
- Upstream syncs stay cheap: upstream-owned lines change by one token at most, and the
  logic lives in files we own.

## Today

`CP_SP.pcmCruiseSpeed` is set once at car init from the `IntelligentCruiseButtonManagement`
param (`sunnypilot/selfdrive/car/interfaces.py` `_initialize_intelligent_cruise_button_management`)
and never changes. Every reader treats it as constant for the drive:

| Reader | Process | Owner | Effect of `pcmCruiseSpeed=False` |
|---|---|---|---|
| `selfdrive/car/cruise.py:54` | card | upstream (comma) | openpilot tracks `v_cruise` from buttons instead of the dash |
| `selfdrive/car/cruise.py:72` | card | upstream (comma) | button timers run |
| `selfdrive/controls/controlsd.py:121` | controlsd | upstream (sunnypilot edit) | `longActive` true on stock ACC |
| `selfdrive/controls/controlsd.py:181` | controlsd | upstream (sunnypilot edit) | `cruiseControl.override` under gas |
| `sunnypilot/selfdrive/car/cruise_ext.py:107,115,139` | card | sunnypilot, heavily ours | 20/30 minimum, engage gate, dash reconciliation |
| `.../intelligent_cruise_button_management/controller.py:336` | selfdrived | sunnypilot, ours | servo runs |
| `sunnypilot/selfdrive/controls/lib/longitudinal_planner.py:57` | plannerd | sunnypilot, ours | `scc_actionable`, cached at init |
| `sunnypilot/selfdrive/controls/lib/speed_limit/helpers.py:50` `pcm_machine_owns_sla` | plannerd, card | ours | SLA session owner, cached at init (arbiter, SLA) |
| `sunnypilot/selfdrive/controls/lib/speed_limit/helpers.py:65` | card init | sunnypilot | demotes SLA assist to warning |
| `selfdrive/ui/sunnypilot/onroad/hud_renderer.py:52` | ui | sunnypilot | set speed shown from `v_cruise` |
| `ui_state.has_icbm`, mici/TICI cruise layouts, sunnylink `capabilities.py` + `cruise.yaml` | ui, sunnylink | sunnypilot + ours | gating of the consumer toggles |

opendbc does not read `pcmCruiseSpeed` beyond defaulting it (`opendbc/car/interfaces.py:173`).
Every `longActive` / `cruiseControl.override` use in the carcontrollers sits behind
`openpilotLongitudinalControl`, so a stock-ACC car is unaffected by those flags (to be
re-audited per brand in phase 1).

## ICBM consumers

| Consumer | Param | Stock ACC | Alpha long (pcmCruise, Mazda) |
|---|---|---|---|
| Slow for curves: vision | `SmartCruiseControlVision` | needs ICBM | planner executes it; no ICBM |
| Slow for curves: map | `SmartCruiseControlMap` | needs ICBM | planner executes it; no ICBM |
| Speed limit assist | `SpeedLimitMode == assist` | needs ICBM | needs ICBM (`icbm_moves_speed_limits`) |
| Custom ACC increments | `CustomAccIncrementsEnabled` | needs ICBM | needs ICBM |

## Design

### Three layers, one owner each

1. **Capability** (boot, constant): `icbm_applicable(CP, CP_SP)`, unchanged. Decides
   whether ICBM can ever run on this car.
2. **Demand** (live, from params): `icbm_demanded(CP, CP_SP, params)`, a registry of the
   consumers above in one new file we own,
   `sunnypilot/selfdrive/car/intelligent_cruise_button_management/demand.py`. A consumer
   sunnypilot adds later is one entry.
3. **Active** (live, latched): card owns an `IcbmLatch` that follows demand but changes
   only at a safe point (below), and publishes the result as
   `carStateSP.zoompilot.icbmActive` (`CarStateZP`, the fork struct, so no ordinal clash with
   upstream appends). Every other process reads that field. Nobody else decides.

`CP_SP.pcmCruiseSpeed` keeps upstream's boot meaning, and on a capable car it is left `True`
(passive) at boot. It is no longer what readers check.

### Readers switch to the live flag

Each reader in the table above reads `icbm_active` instead of `not CP_SP.pcmCruiseSpeed`:

- card: `VCruiseHelperSP` (cruise_ext) exposes `self.pcm_cruise_speed`, set from the latch
  each frame. `cruise.py:54,72` change one token each:
  `self.CP_SP.pcmCruiseSpeed` -> `self.pcm_cruise_speed`.
- controlsd: `ControlsExt` adds `carStateSP` to `sm_services_ext` and exposes
  `self.pcm_cruise_speed`. `controlsd.py:121,181` change one token each.
- selfdrived: the servo's early return reads `sm['carStateSP'].zoompilot.icbmActive`
  (selfdrived already subscribes).
- plannerd: `scc_actionable`, `pcm_machine_owns_sla` and the SLA owner become per-frame
  reads of the flag instead of init-time caches (plannerd already subscribes).
- ui: `hud_renderer` and `ui_state.has_icbm` read the flag. Settings gating uses
  capability (`icbm_applicable`), not activity.

Rejected: mutating each process's `CP_SP.pcmCruiseSpeed` at runtime. Zero upstream diff on
the four predicate lines, but `carParamsSP` would stop being constant (it is logged and
persisted), controlsd/selfdrived hold read-only capnp readers, and the init-time caches
would still need edits. It hides the coupling instead of removing it.

### Safe point

The latch may change only while openpilot is disengaged and the stock cruise is not
engaged (`not CC.enabled and not CS.cruiseState.enabled`). At that instant:

- `longActive` and `override` are false in both modes, so controlsd's one-frame lag on the
  flag is harmless.
- `v_cruise` is unset in both modes; the next engage seeds it the way a fresh drive does
  today (reconciler / dash read).
- The servo is `inactive`, and no SLA session is open (the session ends on disengage), so
  ownership can move between plannerd's SLA machine and the card arbiter without handing
  over a live session.

So "turn on curve slowdown mid-drive" takes effect at the next engage. Phase 4 looks at
activating while engaged.

### Settings and params

- The ICBM toggle goes from mici, TICI and sunnylink. The consumer toggles are available
  wherever `has_long or icbm_applicable`, and their descriptions say openpilot presses the
  cruise buttons on stock ACC.
- The `IntelligentCruiseButtonManagement` key stays in `params_keys.h` (upstream owns it,
  and sunnylink/statsd still name it), but nothing reads it for behavior. card writes it as a
  mirror of `icbmActive` so sunnylink's `has_icbm` and statsd keep meaning "ICBM running".
- The SLA assist demotion at boot (`set_speed_limit_assist_availability`) checks
  `icbm_applicable` instead of `pcmCruiseSpeed`, or assist could never be what brings ICBM
  up.

## Phases

Each phase is its own commit series, replayed and pushed to `icbm-auto`; merge to
danger-unstable after phase 2 for driving.

### Phase 1: plumbing, no behavior change

- Add `CarStateZP.icbmActive`, the `IcbmLatch` in card_ext, and `pcm_cruise_speed` on the
  card and controlsd sides. The latch's input is the old toggle (`icbm_applicable and
  param`), latched at boot only, so behavior is bit-identical.
- Move every reader in the table onto the flag. Turn the init-time caches into per-frame
  reads.
- Audit per brand: `longActive` / `override` / `pcmCruiseSpeed` in opendbc carcontrollers
  and panda safety for stock-ACC paths.
- Exit: process replay identical against refs, for routes with ICBM off and on (stock ACC
  CX-5, alpha-long CX-5, one non-Mazda ICBM brand from upstream refs). Unit tests green.

### Phase 2: demand, still boot-latched

- `demand.py` registry. The latch input becomes `icbm_applicable and icbm_demanded`, still
  latched at boot.
- Remove the toggle from UI + sunnylink; rewrite the consumer gating; mirror the param.
- Fix the SLA demotion check.
- Exit: replay identical for a route with no consumers on (= ICBM off today); a route with
  SCC-V on and ICBM on today replays identical. This alone is shippable.

### Phase 3: live switching

- The latch follows demand at the safe point. card reads the consumer params on its
  existing params thread (`read_custom_set_speed_params` cadence), not at 100 Hz.
- UI: a consumer turned on while it cannot act yet shows a note until the next engage.
  Nothing onroad-locks.
- Exit: unit tests for the latch (no change while engaged; change on disengage; repeated
  toggling; demand drops mid-engagement keeps ICBM until disengage). Closed-loop harness
  (`sla_loop_harness`, `icbm_servo_harness`) across an activation. On-car: engage with
  nothing on, turn SCC-V on, cancel, re-engage, take a curve.

### Phase 4 (optional): activate while engaged

Only if phase 3 feels slow in practice. Needs `v_cruise` handed over from the dash value,
the `update_enabled_state` engage gate primed so it does not drop `enabled` for a frame,
and `longActive` flipping under an engaged stock-ACC car proven harmless per brand.
Deactivating while engaged stays out of scope.

### Phase 5: docs and release

`docs/zoompilot/icbm.md` (activation section), `cruise-arbiter.md` (owner switch),
release notes naming the users whose cars start pressing buttons: anyone with a
consumer on and ICBM off today.

## Upstream merge strategy

- Upstream-owned lines touched: `cruise.py` x2, `controlsd.py` x2, one token each. Nothing
  else in comma's files.
- sunnypilot-owned files: edits sit in files we already carry large diffs on (cruise_ext,
  interfaces, longitudinal_planner, ui_state, mici cruise layout, sunnylink cruise.yaml).
  The TICI `layouts/settings/cruise.py` is the one file with a small diff today; keep the
  change there to removing one item and the gating predicate.
- New logic in new files with our header: `demand.py`, the latch (`icbm_latch.py`
  beside it).
- cereal: one field on `CarStateZP` only.
- When sunnypilot adds an ICBM consumer: add it to `demand.py`. When sunnypilot touches the
  toggle UI: take ours (the toggle stays gone).

## Open decisions

1. **Escape hatch.** With no toggle, a user can only turn ICBM off by turning off its
   consumers. Options: none (recommended; the consumers are the switches), or a developer
   toggle "Never press cruise buttons" that vetoes demand.
2. **Custom ACC increments as a consumer.** It brings up a button servo just to change the
   step size. Keep it (today's behavior requires ICBM), or let it ask only when another
   consumer is also on.
3. **Release-notes framing** for users whose cars start pressing buttons after the update.

## Risks

- A reader missed in phase 1 keeps the boot value and disagrees with the flag. Mitigation:
  `git grep pcmCruiseSpeed` must show only `interfaces.py` and opendbc's default after
  phase 1, enforced by a test.
- The consumers' params are live in plannerd but latched in card: SCC-V on with ICBM not yet
  active must not make plannerd act (`scc_actionable` follows the flag, not the param).
- Demand flapping while disengaged (toggling in settings) flips the latch freely; harmless
  since nothing is active, but logs will show it.
- `pcm_machine_owns_sla` becoming live means the plannerd SLA machine and the card arbiter
  can each own SLA on different drives' segments. Both are already exercised; the safe point
  guarantees no open session.
