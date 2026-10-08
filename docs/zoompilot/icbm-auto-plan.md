# ICBM on demand: plan

Branch `icbm-auto` in zoompilot and opendbc, off `danger-unstable` (646c2f7694 / opendbc
53b2706022). Status: phases 1-3 implemented on icbm-auto; on-car pending.

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

- The ICBM toggle is gone from mici, TICI and sunnylink. The consumer toggles are available
  wherever `has_long or icbm_applicable`; `ui_state.has_icbm` and sunnylink's `has_icbm` now
  mean the capability. No wording was added (decided 2026-10-08).
- Custom increments are editable onroad (card reads them live); SCC and SLA already were.
- The `IntelligentCruiseButtonManagement` key stays in `params_keys.h` (upstream owns it) but
  the migration deletes the param and nothing reads it. statsd still lists it (logs False).
- sunnylink hid the alpha-long toggle whenever `has_icbm`; with `has_icbm` meaning capability
  that would hide it on every Mazda, so the condition is gone (ICBM and alpha long coexist).
- The SLA assist demotion (`set_speed_limit_assist_availability`) checks `icbm_applicable`
  instead of `pcmCruiseSpeed`, or assist could never be what brings ICBM up.

## Phases

All phases land together on danger-unstable (decided 2026-10-08).

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

Done 2026-10-07, except the process replay run:
- `CarStateZP.icbmActivation` is an enum with `unset` first, so logs from before it (process
  replay refs, old routes) fall back to the boot flag and replay unchanged.
- card: `IcbmLatch` lives on `VCruiseHelperSP`; `pcm_cruise_speed` is a property over it.
  controlsd: `ControlsExt.pcm_cruise_speed` reads `carStateSP` (added to its sm and to the
  controlsd/selfdrived process replay pubs). selfdrived: one `update_activation` line before
  the servo runs. plannerd: SCC gating per frame. ui: `hud_renderer`.
- Left for phases 2/3: `pcm_machine_owns_sla` (SLA owner, init-time in plannerd and the
  arbiter) and the boot SLA demotion.
- opendbc audit: of the ICBM brands (Mazda, Chrysler, Honda Bosch, Hyundai), `longActive`
  and `override` are read only on openpilot-long / alpha-long paths. No opendbc change.
- Test baseline: `test_following_distance` and `test_cruise_speed` (plant maneuvers) already
  fail 27 cases on danger-unstable; same count with phase 1.

### Phases 2 and 3: demand, migration, live switching

Done 2026-10-08:
- `demand.py` (the consumer registry), `migration.py` (`IcbmDemandMigrated` marker), boot
  decision in `interfaces.py` = `icbm_applicable and icbm_demanded`.
- `IcbmLatch(CP, CP_SP)`: `capable` from `icbm_applicable`, `demanded` refreshed on card's
  10 Hz params thread, `update(engaged)` from `VCruiseHelperSP.update_enabled_state` with
  `engaged = CC.enabled or CS.cruiseState.enabled`. On a change: zero both button-timer
  sets (a press frozen in the passive mode would replay as a long press), reset the engage
  gate and the reconciler, recompute the minimum set speed, move the SLA owner.
- SLA owner: `pcm_machine_owns_sla(CP, icbm_active)`. card's arbiter `set_icbm_active`;
  plannerd keeps both the machine (pcm openpilot-long cars only) and the mirror, picks per
  frame, `reset()` on a handover.
- Servo (selfdrived) re-initialises on an activation change, keeping `fast_faulted`.
- Capability: audited every ICBM brand's panda safety. Buttons pass only while
  `controls_allowed`, there is no ICBM safety flag, and opendbc never reads
  `pcmCruiseSpeed`, so a mid-drive activation needs nothing below openpilot. One wrong flag
  fixed in opendbc (957a8279c5): Chrysler CUSW (Jeep Cherokee 5th gen) claimed ICBM, but
  `chrysler_cusw.h` passes only cancel/resume.
- Tests: `test_icbm_latch.py` (latch, publication, demand, migration, card and planner
  handovers); harnesses now declare the capability.

Replay check 2026-10-08 (card, controlsd, plannerd, selfdrived against danger-unstable
4ac8ad4d65): four CX-5 drives (stock ACC and alpha long, each recorded with ICBM on and off),
settings "no ICBM feature" and "toggle on + SCC vision". Every output is identical except
selfdrived's startup alert on one alpha-long drive, which comes from adding `carStateSP` to
selfdrived's process-replay inputs (the base replay never fed it; a car always does): with
danger-unstable's replay config the branch is identical there too.

Settings locks (2026-10-08): while engaged with ICBM off, the device does not let a feature
that needs ICBM be turned on (`ui_state.icbm_start_locked`; mici selector skips "assist" via
`set_blocked_options`, TICI disables the assist button). sunnylink locks those features while
engaged on ICBM cars in both directions: its validator forbids a toggle gating on its own
value. The assist option there stays selectable when already selected.

### Phase 4: activate while engaged

Not done. A feature switched on or off while engaged takes effect at the next disengage.

### Phase 5: docs and release

`docs/zoompilot/icbm.md` (activation section), `cruise-arbiter.md` (owner switch),
release notes (the toggle is gone; turning on a cruise feature now enables the buttons;
the migration turned off features that were doing nothing).

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

## Decisions (2026-10-07)

1. **No escape hatch.** The consumers are the switches. ICBM never comes up on a car
   without the capability (`icbm_applicable`, i.e. the opendbc
   `intelligentCruiseButtonManagementAvailable` flag and the long mode).
2. **Custom increments need ICBM** and stay a consumer in every mode.
3. **No separate "cruise buttons" setting for SLA.** On stock ACC assist cannot work without
   the buttons, so the setting would have one valid value. Assist uses the buttons wherever
   the car supports them. Alpha-long cars without ICBM keep plannerd's driver-confirm machine.
4. **A one-time migration** keeps today's behavior for every existing install (below).

## Migration

Runs once in `setup_interfaces` (CP known, before anything reads demand), marked by a new
`IcbmDemandMigrated` param (`PERSISTENT | BACKUP`: restoring a pre-migration backup brings
the old settings back without the marker, so the migration runs again).

- Car without ICBM capability: nothing to do. The old param is already cleared there, and
  the consumers stay as inert as they are today.
- ICBM was on: nothing to do. Its consumers keep demanding it, and ICBM on with no consumer
  turns off, which loses nothing (it only changed engage and set-speed tracking).
- ICBM was off: turn off every consumer that ICBM-off made inert in the mode this boot is
  in, so nothing starts pressing buttons:
  - stock ACC: `SmartCruiseControlVision`, `SmartCruiseControlMap`,
    `CustomAccIncrementsEnabled` off; `SpeedLimitMode` assist -> warning (the boot demotion
    already does this today, so it is a no-op in practice)
  - alpha long: `CustomAccIncrementsEnabled` off. SCC-V/M stay (the planner runs them).
    SLA assist: see open item below.

A consumer the user turns on later demands ICBM normally. Switching from alpha long to
stock ACC with SCC-V on brings ICBM up on that drive; that is the feature as set, not a
migration gap.

Alpha long with ICBM off and SLA assist on (today's driver-confirm machine): assist stays
on and brings ICBM up, so the dash moves to the limit by itself. Decided 2026-10-07; the
release notes say so.

## Risks

- A reader missed in phase 1 keeps the boot value and disagrees with the flag. Mitigation:
  `git grep pcmCruiseSpeed` must show only `interfaces.py` and opendbc's default after
  phase 1, enforced by a test.
- The consumers' params are live in plannerd but latched in card: SCC-V on with ICBM not yet
  active must not make plannerd act (`scc_actionable` follows the flag, not the param).
- ICBM's own presses stop at 20 mph / 30 km/h on the dash (`controller.py` `v_cruise_min`).
  The driver can still set 19 mph on a Mazda, so this only bounds how low a curve or limit
  walks the dash, as it does today.
- Demand flapping while disengaged (toggling in settings) flips the latch freely; harmless
  since nothing is active, but logs will show it.
- `pcm_machine_owns_sla` becoming live means the plannerd SLA machine and the card arbiter
  can each own SLA on different drives' segments. Both are already exercised; the safe point
  guarantees no open session.
