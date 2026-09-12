# Crawl LKAS fault: route 000001e8, September 9, 2026

The full rlog confirms a third crawl-from-stop EPS fault without steering-message starvation.
An offline startup-guard evaluation does **not** justify a production controller change:
waiting for `LKAS_BLOCK` to clear would also withhold steering that the EPS successfully
applies. The exact distinguishing trigger remains unidentified.

## Fault evidence

Route `98673c0899e64dff/000001e8--2aa5bba6f7`, segment 0, build `fd4cb7e4d9`
(`jetson-trt`, opendbc pin `dee3a36070a8a1e8f4b20ef0adc7b551d6c217dc`). Retrieved the full
5,847,556-byte rlog over SSH and preserved it at
`tools/mazda_long/test_data/route_1e8/rlog_seg0.zst` (ignored analysis data).
Times below use the first CAN event as zero; the route began around 19:34:44 CDT September 8.

| Capture | EPS fault, seconds into segment | Speed | First nonzero echo to fault | Largest pre-fault request magnitude | Effective LKAS torque |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1e8 segment 0 | 31.564 | 0.411 m/s | 299 ms | 95 | 0 |
| 1a segment 0 | 24.786 | 0.416 m/s | 280 ms | 113 | 0 |
| 1bb segment 1 | 9.472 | 0.490 m/s | 249 ms | 300 | 0 |

All three change from EPS byte 6 `0x14` to `0x24`. Standby is already present at the segment
boundary; its original entry is unobserved. Driver torque opposes the request in 20/25,
23/23, and 21/21 nonzero pre-fault EPS observations, respectively.

For 1e8, the longest accepted 0x243 gap in the two seconds before the EPS fault is 22 ms;
there are zero rejected 0x243 frames in that interval. The camera's error follows 5.366 s
later, at segment time 36.929 (approximately 19:35:20.9 CDT). The reduced qlog also shows
longitudinal control never active, and no increasing CAN error counters. Alpha longitudinal
was configured, so this does not exclude an interaction with radar replacement traffic; it
excludes an actual longitudinal engagement as the immediate event.

## Successful counterexamples

Scanned 3,853 distinct cached CX-5 2022 segment paths. These are a historical cache snapshot,
not a fresh decode of every source or a count of independent vehicles. Firmware/builds vary.
Found 391 candidate stretches starting at at most 0.6 m/s with nonzero echoed request,
zero effective torque, BLOCK and TRACK set, lasting at least 300 ms, with no camera fault
onset in that segment. Segment edges and CAN gaps above 100 ms are excluded.

Raw-decoded eight controls nearest 1e8's 95-count request magnitude. All eight have no EPS
fault bit anywhere in their segment and confirm zero effective torque and byte 6 `0x14`
in the selected window. Six have opposing-driver observations. Particularly useful:

- `00000047--062015159e`, segment 2, 30.231–33.300 s: starts at 0.280 m/s,
  maximum request magnitude 99, driver opposition in **257/257 frames**, no EPS fault.
- `0000017e--2473ea703b`, segment 18, 37.061–38.951 s: starts at 0.287 m/s,
  maximum request magnitude 94, driver opposition in **149/159 frames**, no EPS fault.

Thus speed, request magnitude, standby, zero delivery, driver opposition, and 250–300 ms
exposure do not by themselves separate these faults. The 391 count differs from the earlier
406 comparison because the episode duration/filter definitions differ.

## Offline guard result

Candidate: recognize a BLOCK episode containing TRACK below 0.6 m/s, keep valid steering
messages but withhold nonzero requests until BLOCK clears. This tool evaluates which recorded
requests would be withheld; it does not run a modified controller or simulate the EPS.

Among 757 complete startup blocks lasting at least 250 ms:

- 436 reach BLOCK-clear after a zero-request tail of at least 250 ms. Readiness can occur
  without torque requests, but this does not prove it would still occur under the guard.
- 322 contain nonzero requests. The broad guard would withhold 82,004 observed request frames;
  **32,263 of those frames have nonzero effective torque**.
- Even restricting entry to zero effective torque leaves 539 blocks, of which 60 subsequently
  deliver nonzero torque before BLOCK clears; 27 exceed 50 effective counts.

A guard released by effective torque instead has a causal problem: if it withholds the request,
its own action can prevent the effective torque needed for release. Recorded future effective
torque cannot be used to demonstrate that this guard would resume. A fixed delay or speed
cutoff would avoid that dependency but has no separating threshold established here.

**Decision:** reject the proposed BLOCK-clear guard for production. Keep the existing
rejection recovery. No vehicle control or safety files were changed, and nothing was deployed.
An EPS readiness signal independent of our request, or a controlled experiment with a bounded
startup delay and explicit steering-availability behavior, is needed before claiming a remedy.
The EPS/FSC diagnostic trouble codes after an occurrence could also narrow the internal fault;
this investigation did not issue diagnostic requests to the vehicle.

## Reproduction and validation

`tools/mazda_long/replay_lkas_startup.py` produces counts, all candidate controls, eight raw
control verifications, and fault-onset measurements in JSON. It uses the existing
`.undelivered_cache`; regenerating that cache is a separate step. Historical cache metadata
records the fingerprint but does not establish firmware equivalence with the new route.

```sh
PYTHONPATH=. .venv/bin/python tools/mazda_long/replay_lkas_startup.py \
  --fault tools/mazda_long/test_data/route_1e8/rlog_seg0.zst \
  --fault tools/mazda_long/test_data/alpha_long_logs/0000001a--99867abd18--0/rlog.zst \
  --fault tools/mazda_long/test_data/route_1bb/rlog_seg1.zst \
  --json /tmp/lkas_1e8/startup.json
```

Validated all three raw fault captures and eight raw controls; checked episode handling for
left-censoring, logging gaps, normal completion, and TRACK falling before BLOCK. Independently
verified a raw frame delivering 980 counts with byte 6 `0x14` in route 7f segment 11. No
controller tests are claimed: the change is an offline analysis tool and this report.

## Protocol audit, second pass (Claude, same day)

`tools/mazda_long/audit_lkas_protocol.py` over 413 byte-distinct logs (67 fresh device
segments plus every cached fault report and comparable crawl start): 2.15 M accepted and
364 k stock-camera 0x243 frames, zero checksum residuals, zero counter skips, no accepted-stream
gap above 25 ms inside any crawl window. Status bytes 2-6 are identical in faulting and
non-faulting windows (ours `0020020000`, camera `0820020000`). The protocol is not the trigger.

Caveat on the tool: on builds before the src-192 rejection report, forwarded camera copies are
logged as src 128, so `accepted` gaps understate starvation there. Route 117 shows 22 ms gaps in
the audit but a 0.613 s EPS-echo gap in `lkas_fault_scan.py`; 116/117 are the closed rejection
class, not this one.

### What separates the three open crawl faults

All twelve fault onsets in the set were re-read raw. Nine are the known classes (rejection
starvation 116/117/139/148/drive_02/13, or the bit already set at segment start). The three open
ones (1a, 1bb seg 1, 1e8) share one property the earlier comparisons never conditioned on: each
is the **first LKAS activation of the ignition cycle**. `LKAS_EFFECTIVE` had never been nonzero
since power-up (1bb seg 0 verified: zero request, zero effective, radar silenced at 16.1 s).

Every clean crawl-speed first activation in both corpora (`.undelivered_cache` seg-0 logs plus
the fresh device segments), raw-verified:

| Route | Radar | v0 m/s | EPS byte 6 | max request | max effective | driver torque | opposing frames | Result |
| --- | --- | ---: | --- | ---: | ---: | ---: | ---: | --- |
| 1a | silenced | 0.28 | 14 | 113 | 0 | +9 | 25/25 | **fault +0.28 s** |
| 1bb seg 1 | silenced | 0.30 | 14 | 372 | 0 | +25 | 26/26 | **fault +0.25 s** |
| 1e8 | silenced | 0.28 | 14 | 95 | 0 | +10 | 21/26 | **fault +0.30 s** |
| 187 | stock | 0.53 | 14 | 145 | 0 | -14 (same sign as request) | 12/26 | ok |
| 22 | stock | 0.35 | 14 | 160 | 0 | +3 | 26/26 (trivial) | ok |
| route_53 | silenced | 0.32 | 14 | 38 | 0 | -2 | 20/26 (trivial) | ok |
| 5f, 1ba, 1e1, 1ef | mixed | 0.28-0.29 | 14 | 108-175 | 0 | <=3 | 0 | ok |
| 11d | silenced | 0.39 | 14 | 348 | 184 | -39 | 25/25 | ok |
| 1d9, 1e7 | silenced | 0.67, 1.18 | 00, 14 | 360, 384 | 256, 28 | -25, -3 | 0 | ok |

Over all 106 cached first activations at any speed: 20 had real driver opposition with the EPS
delivering torque, all fine. The only first activations with standby (0x14), zero delivery and
the driver holding >= 9 counts against the request are the three faults. The same combination
later in a drive (route_118 t248, 12c t125, 139 t732/t812, 11d t515: 25-26 opposing frames,
eff 0, requests 112-372) never faults, so the EPS's first activation after power-up is the
condition, not standby-plus-opposition in general.

The alpha-long (radar silenced) correlation (3/53 crawl onsets vs 0/381) is real but not
separable from this: every recent first activation is on alpha long, and no stock-long first
activation with real opposition exists in the corpus. Route 22's 26/26 "opposition" is 1-3 counts.

### Fix as built (opendbc, uncommitted)

Route 11d, the one cached case that seemed to deliver torque in standby at a crawl, starts
mid-drive at 22 m/s with torque already flowing; it was never a first engagement.

Two more replays fixed the rule. A hold keyed on driver torque (>= 6 counts against the
request) covers 1bb from its first frame but only the last 50-60 ms of 1e8 and part of 1a: the
driver's torque built during the first 250 ms (1e8: 0, 3, 2, -1, 3, 5, 10, 10, 11). And on all
ten non-faulting first engagements from standby below 1 m/s, the EPS's first delivered torque
came only after the standby had lifted above 1 m/s, 2.5-14 s later (routes 187, 53, 1e1, 1ef),
or never in the segment (4c, 22, 1ba). So the hold needs no driver condition:

- carstate (`update_steer_undelivered`, steer-to-zero EPS only) latches `lkas_delivered` on the
  first nonzero `LKAS_EFFECTIVE` of the cycle and derives `steer_first_engage_hold`: not yet
  delivered, BLOCK and TRACK set, `vEgoRaw` below `STEER_UNDELIVERED_ALERT_ORIGIN_SPEED`
  (1.0 m/s, the same standby-from-a-stop boundary the alert already uses).
- carcontroller obeys it in the one gate that already obeys `steer_undelivered`:
  `apply_torque = 0`. The ramp then starts from zero at `STEER_DELTA_UP`, which the panda accepts.

Replayed over the 84 seg-0 logs: the hold spans the full request-to-fault window on 1a, 1bb
and 1e8 (0.24-0.30 s) and 0.4-1.6 s on the ten others, never overlapping a delivered frame.
Tests: `TestFirstEngageHold` on the carstate rig, plus the controller-obeys test; 749 pass
across the mazda and safety suites. The only behaviour outside the three faults is a controller
restart mid-drive, which re-applies the hold once for at most one standby at a crawl.

What the driver sees: nothing new. Lateral shows engaged with no torque during a first pull-away
under 1 m/s, exactly as before, because the EPS delivered nothing there anyway.

### Follow-up: the undelivered latch is deletable

At 409 block-clear edges with a request of 200+ counts, the EPS slewed its own effective torque
at 8-12 counts per frame and never jumped. Zeroing the request during a block (the steer-to-zero
latch) therefore neither prevents a fault nor smooths the release. It still feeds the silent
steerTemporary alert; that alert can read request-versus-effective directly. Left in place so
the first drive tests one change.
