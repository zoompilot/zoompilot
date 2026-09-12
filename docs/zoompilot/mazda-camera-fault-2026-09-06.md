# Mazda camera fault: the EPS's LKAS_FAULT bit, 2026-09-06

The cluster's "Front Camera Sensor System Malfunction" and openpilot's "LKAS Fault: Restart the
Car" are the camera's `CAM_LKAS.ERR_BIT_1` / `CAM_LANEINFO.ERR_BIT`. On the 2022 EPS they are
preceded, on every capture with a known onset, by a bit the EPS raises first: `STEER_RATE` byte 6
bit 5 (message bit 53), now decoded as `LKAS_FAULT`. This page records what the corpus says about
that bit, what triggers it, what was changed, and what is still open.

## The finding

Scanned: every rlog under `tools/mazda_long` (4,093 segments, 3,879 of them CX-5 2022 with EPS
frames, about 64 h) plus the new capture `000001bb--b4a16dc0f3` from `comma@192.168.1.144`.
Tool: `tools/mazda_long/lkas_fault_scan.py`.

| Fact | Count |
| --- | --- |
| Camera `ERR_BIT_1` rises with `LKAS_FAULT` already set | 15 of 16 |
| Rises with a known `LKAS_FAULT` onset in the same segment | 11, camera fault 5.25 to 5.55 s after the bit |
| `LKAS_FAULT` runs with a known onset that lasted 5.2 s or more without a camera fault | 0 of 10 |
| `LKAS_FAULT` runs that ever cleared (57 runs, 2,899 s) | 0 |
| EPS frames with the bit set outside a faulted drive | 0 |

The one rise without the bit (`00000043` seg 0, build `61919842ad`, July) has the EPS unblocked
and delivering right up to the fault; it is a different mechanism and is not explained here.

Byte 6 of `STEER_RATE` takes five values in the corpus: `00` (free), `04` (block: the camera is
reporting no lane lines, or nothing is arriving at all), `08` (hands-off), `14` (block with
`LKAS_TRACK_STATE`: standby from a stop, and the 3 s hands-on timer at speed), `24` (block with
`LKAS_FAULT`). `LKAS_FAULT` never appears with `LKAS_TRACK_STATE`.

The old reading, that the camera latches its fault from watching a request go undelivered, is
wrong in both directions: in `000001bb` the request had been zero for 5.25 s when the camera
faulted, and in the March lateral-only drive `test_data/drive_02` (build `5e64db2897`, alpha
long off) the EPS bit was set at 16 m/s with nothing but zero requests delivered for the 5.5 s
before the fault. The camera is reacting to the EPS bit, not to our request.

## What raises the bit

Of the 11 known onsets:

- **A break in the 0x243 stream of about 0.6 s, seven times.** The last delivered frame is
  0.613 to 0.623 s before the bit in every one (routes `00000116`, `00000117`, `00000139`,
  `00000148`, `00000102`, `drive_02`, `00000013`). The breaks were panda rejection bursts: the
  driver-torque staleness and the 25-versus-12 rate-down (`00000148`, fixed 2026-09-01), the
  MADS/panda arming order (`00000116`/`00000117`, fixed 2026-08-28), and on the March and July
  builds commands past the panda's 800-count limit. The break is what matters, not which check
  tripped: a rejection resets the panda's rate-limit reference to zero
  (`steer_torque_cmd_checks`, `desired_torque_last = 0`), so every later frame more than one
  step from zero is rejected too, and a controller that keeps ramping never gets a frame through
  again. Verified against the compiled safety model in
  `opendbc/safety/tests/test_mazda.py::test_without_the_rejection_report_a_rejection_starves_the_eps`.
- **Three restarts with the panda not forwarding** (`00000024`, `00000034`, `00000013`): the
  bit rose within 0.5 s of the log starting or of the panda entering the Mazda safety mode, with
  nothing delivered. The same timeout, with the comma's boot as the break. An EPS that has never
  had a stream (`00000116` seg 0, 6.4 s of nothing at ignition) does not raise it.
- **Two at a crawl from a stop, unexplained** (`000001bb` at 0.49 m/s, `0000001a` at 0.42 m/s,
  the latter a July alpha-long drive). Both: EPS in standby (`14`), a ramping request opposing
  the driver's torque, `LKAS_EFFECTIVE` zero, the bit 240 to 280 ms after the first nonzero
  frame, no gap and no rejected frame. The corpus has 406 other stretches of a nonzero request
  delivered into a zero-delivery standby for 0.25 s or more (213 of them starting below 0.6
  m/s, 95 opposing the driver, requests up to 1,200 counts, up to 3 s long) and none raised
  the bit. No standby guard was added: nothing in request, delivery, speed, driver torque or
  wheel motion separates these two from the 406.

## What changed (now included in develop)

The recovery was committed as opendbc `be4c7459bb` and promoted in the develop squash
`676fa14b20`, pinned by sunnypilot `75580ad1e3` on September 6. The earlier "uncommitted"
status of this section no longer applies. Presence in this checkout does not establish which
build an affected vehicle ran.

- `mazda_2017.dbc`: `STEER_RATE.LKAS_FAULT`, bit 53.
- `carstate.py`: decodes it (`lkas_fault`) for the log and tooling; the driver-facing fault
  stays the camera's `ERR_BIT_1`, unchanged. A third parser on the panda's rejected-transmit
  bus (`Bus.loopback`, src 192, `CAM_LKAS` at nan frequency) counts our refused torque requests
  each cycle as `lkas_rejected`. This replaced the EPS request echo on 2026-09-08 (see the
  replay below for why); uncommitted on danger-unstable.
- `carcontroller.py`: a nonzero `lkas_rejected` restarts the ramp from zero, which is what the
  panda accepts after a rejection. The report is one or two card cycles behind the refusal; the
  closed-loop test bounds the outage at the report delay for a lone reference reset and at the
  driver-sample staleness plus the delay for route 148's scenario, against the EPS's 60 frames.
  This closes the gap path for any rejection cause, including ones not yet seen, on every Mazda.
- Tests: `test_mazda_carstate.py::TestLkasFaultBit` and `::TestRejectionReport`,
  `test_mazda_steering.py::TestRejectionRecovery`, and three controller-through-panda runs in
  `opendbc/safety/tests/test_mazda.py`. Mazda suites: 662 passed. The golden transmit fixture
  is unchanged.

No panda safety code changed. The camera's error bits are not masked; `ERR_BIT` forwarding in
the HUD frame (2026-08-26) stays, which is why the cluster shows the warning now and did not on
older builds that faulted just the same (drive_02 in March: camera `ERR_BIT` set in 88 of 114
frames, forwarded in 0 of 120).

## What is not proven

- That a stream never breaking prevents every fault. The two crawl onsets are not the gap
  mechanism and remain open; a lateral-only user capture of one is needed (the rlog around
  the fault, any build). `lkas_fault_scan.py` on it answers which case it is in seconds.
- On-car behaviour of the recovery. Replay establishes what goes on the wire, not the EPS's
  response to a 200 ms re-ramp. Drive danger-unstable through a hard driver override at low
  speed and check `LKAS_FAULT` stays clear in the log.
- Whether the bit clears on the EPS between ignition cycles or needs a longer power-down. Every
  run in the corpus lasts to the end of its segment; drives that follow one start faulted.

## September 8 regression review: 2022+ CX-5

The change that exposed the camera warning on the cluster is opendbc `ebfefb47b6`
(August 26): `create_alert_command` began preserving `CAM_LANEINFO.ERR_BIT` instead of
implicitly packing zero. This identifies a change in warning visibility, not the introduction
of the underlying EPS fault. Reverting that field would hide the camera's reported error.

The prior fixes address different parts of the failure chain. `ca69f9773e` adds driver-torque
sample history and margin to avoid rejection from controller/panda sample disagreement.
`be4c7459bb` adds recovery from a rejection when one nevertheless occurs (the EPS echo then, the panda's rejection report since 2026-09-08). The older
non-delivery latch alone was insufficient: stopping torque after the EPS stops delivering
does not undo an already latched EPS fault. The two crawl onsets above remain unexplained.

Re-ran `lkas_fault_scan.py` against stored raw captures on September 8:

| Capture | Build | EPS fault onset | Gap before onset | Rejections before onset | Camera fault delay |
| --- | --- | --- | --- | --- | --- |
| `device_data/00000148--e00a5dce42--10/rlog.zst` | `13033a7a54` | 49.21 s, 4.55 m/s | 613 ms | 62 | 5.535 s |
| `test_data/alpha_long_logs/0000001a--99867abd18--0/rlog.zst` | `7c38742fc4` | 24.79 s, 0.42 m/s | 20 ms | 0 | 5.550 s |

Both captures have alpha longitudinal enabled; they establish two historical mechanisms,
not attribution of a new lateral-engagement report. A current affected route and its build
are needed to distinguish missing recovery, a failure despite recovery, and the unresolved
crawl condition. Preserve CAN around engagement and fault onset, including accepted/rejected
0x243 transmissions, EPS request echo and fault bit, and camera error bits. DTCs can further
identify the EPS's internal condition. No new steering behavior is justified by the model/year
and warning text alone.

Current targeted validation: Mazda steering, carstate, and compiled panda safety suites:
362 passed, 44 skipped, 200 subtests passed. These validate software behavior, not resolution
of the unexplained on-car faults.

## September 8: which change introduced it

Two changes, one for visibility and one for frequency. The pre-rewrite history is on the local
`develop-archive` branches; the hashes below are from there.

**Visibility: opendbc `ebfefb47b6`, August 26** ("pass camera state through the HUD frame").
Upstream's `create_alert_command` copies only `NO_ERR_BIT`, so the packer zeroes `ERR_BIT` in our
`CAM_LANEINFO` and the cluster never showed the camera's fault on any earlier build. From this
commit the camera's `ERR_BIT` is forwarded and the cluster reads "Front Camera Sensor System
Malfunction". Reverting it would hide the car's own fault detector.

**Frequency: opendbc `80bb222e8b`, August 29** ("match the steer winddown rate to the EPS").
`STEER_DELTA_DOWN` went 25 to 12 with the panda left at `max_rate_down = 25`; its comment reads
"Panda keeps max_rate_down = 25 as the looser backstop", which is the wrong model of
`driver_limit_check`. Once the driver bound is below the last command the panda demands a
retreat of at least `max_rate_down`, so a 12-count retreat is a violation, the violation zeroes
`desired_torque_last`, and every later frame more than one step from zero is rejected too.
Routes `00000139` seg 14 (August 30) and `00000148` seg 10 (August 31) faulted this way the next
day. The panda side was matched at 12 on September 1 and is in every develop pin since the
September 2 rewrite (`0af516f27f`).

The mechanism itself predates both: March's driver multiplier 15 (`d76ff7b0f0`) made the
controller's single stale driver sample worth 15 counts of ceiling against the panda's 6-sample
window (`00000102`, `00000013`, `drive_02`), and the MADS arming desync (`00000116`/`00000117`,
fixed August 27) starved the EPS the same way. Since `dcda06d872` (August 30) the camera's own
0x243 is forwarded whenever the panda thinks openpilot is not controlling, so a MADS desync on
its own can no longer starve the EPS: `000001c9` seg 0 (September 5) has 4,424 rejected
zero-torque frames and no gap, no `LKAS_FAULT`, no camera fault. On builds from that date a
src 192 count is not a rejection count.

What users ran, from the prebuilt workflow's run history: an August 25 cut (envelope matched at
25/25, `ERR_BIT` not forwarded, no MADS gate), then nothing until September 5 (`ERR_BIT`
forwarded, panda 12/12, sample margin, MADS gate, non-delivery latch and alert, no echo
recovery), then September 7 (echo recovery, every Mazda on the 1200/12/12/x15 envelope).
Source installs on develop carried 12-versus-25 from August 29 to September 2 with the warning
visible. The rise in reports is the September 5 cut making an existing fault class visible on
a fleet that, on develop, had just been through the 12-versus-25 window.

### Echo recovery replayed against the real EPS

Superseded 2026-09-08: the controller now reads the panda's own rejection report (src 192 on
the `can` stream) instead of inferring a rejection from the EPS echo. The tables stay as the
record of why: the echo rule was sound at the logged timing and fragile to latency.

`tools/mazda_long/replay_echo_recovery.py` runs `recover_from_rejection` as shipped (history 4,
mismatch 5) over every openpilot 0x243 in the corpus while `carControl.latActive` is set, with
the last `STEER_RATE` before each frame as the echo, and reports where it would have fired.
4,094 segments, 12.85 M controlling frames:

| Result | Count |
| --- | --- |
| Frames whose echo matches the latest command | 10,875,056 |
| ... the previous one / two / three | 1,444,011 / 41,993 / 5,574 |
| ... none of the last four | 445,050 (3.5 %) |
| Would-fire episodes | 8 |
| ... within 0.5 s of a panda rejection | 8 |
| ... on a healthy stream (false positive) | 0 |

The 3.5 % of frames with no match never run five deep without a rejection at the logged
timing. The rule is sensitive to latency, though. The echo lands 10 ms after the command at the
median and 20 ms at p90 (tx timestamp to the first `STEER_RATE` carrying the value, `000001c9`
seg 22 and `0000019e` seg 5), and re-running the replay with extra echo delay standing in for
pandad-to-card jitter gives, for the shipped rule and for two sturdier ones (`--grid`):

| Rule | +0 ms | +10 ms | +20 ms | +30 ms | +50 ms |
| --- | --- | --- | --- | --- | --- |
| history 4, any mismatch (shipped 09-06) | 0 | 0 | 158 | 25,422 | 4,632 |
| history 4, echo frozen | 0 | 0 | 0 | 1 | 956 |
| history 8, any mismatch | 0 | 0 | 0 | 1 | 1 |
| history 8, echo frozen | 0 | 0 | 0 | 1 | 1 |

Cells are false-positive episodes over the corpus; every rule detects all 8 rejection episodes
at every delay. A false positive drops the command to zero and re-ramps at 12 counts a frame,
a lateral hiccup rather than a fault, but 25,000 of them is a broken controller. The controller
now uses history 8 with the frozen-echo test: a mismatch counts only while the echo holds one
value, which is what a rejected stream looks like (the EPS keeps echoing the last request it
received) and a late echo does not. `STEER_ECHO_HISTORY = 8`, `recover_from_rejection` tracks
the mismatched value. That revision was never committed; the rejection report replaced it the
next day. Every gap-type
fault in the corpus (`00000116`, `00000117`, `00000139`, `00000148`, `drive_02`, `00000013`,
`00000102` seg 0 whose fault lands 1.4 s into seg 1) is detected 5.8 to 6.1 s before the camera
fault, that is within the first 0.3 s of the gap against the EPS's 0.6 s timeout. The two crawl
onsets (`000001bb`, `0000001a`) and `00000043` produce no episode, as expected: nothing was
rejected there. The detector is also blind by construction while the command is zero and while
lateral is inactive.

### A second class: route `0000007b--9b17f2dc01`, September 8

The first user route with the cluster warning (CX-5 2022, comma four, develop `572d36a71e` of
September 4, stock longitudinal, MADS lateral on for the whole 9 min drive, "at least 10-15"
warnings) contains none of the above: no `ERR_BIT`/`ERR_BIT_1`, no `LKAS_FAULT`, no panda
rejection, no gap in 0x243 (max 23 ms), peak request 620. Only segments 2 and 5 have rlogs and
the API refuses upload requests for that dongle. So there is a second, transient class the two
commits above do not explain. What differs on that car: the camera's own TJA is switched on
(0x440 TJA field 4, dropping to 3 and 2 around lane-line loss and driver overrides, 48 override
episodes in the drive) while ours reads 0; `MazdaTjaButton` is not declared there and no TJA
press is logged. We have never forwarded the TJA fields (copied for a few hours on August 26 by
`ebfefb47b6`, removed by `c400a03da3` the same day, zero in every pin since and in upstream),
and the camera's `TJA_TRANSITION` churn is larger on our own car without any warning. Next step
is a drive on that car with TJA switched off, plus the remaining rlogs and a timestamp of one
occurrence.

## Diagnostics still worth doing

Stored DTCs would name the EPS's condition. `tools/scripts/car/read_dtc_status.py` stops pandad,
puts the panda in ELM327 mode, opens an extended session on the FSC (`0x706`) and EPS (`0x730`)
and reads DTC status; it is active diagnostics, so run it parked, ignition on, engine off, and
read before clearing. The device was disconnected from the car during this investigation, so it
has not been run.

## Rejection-gap checker

`tools/mazda_long/lkas_starvation_check.py` previously closed a rejection burst only on two
delivered frames under 50 ms apart, which the 16 Hz forwarded camera stream never satisfies,
and reported minutes of false starvation. It now ends a gap on any delivered frame. It measures
rejection-associated gaps only; `lkas_fault_scan.py` is the check for the fault itself.
