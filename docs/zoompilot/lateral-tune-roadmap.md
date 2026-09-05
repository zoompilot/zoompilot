# v2 torque tune: design and acceptance numbers

`openpilot/sunnypilot/selfdrive/controls/lib/latcontrol_torque_v2.py`, selected by
`TorqueControlTune = 2.0`, seeded once per steer-to-zero Mazda by `_seed_mazda_torque_defaults`
(from card at fingerprint, and from manager start off the last drive's CarParams; the marker is
`MazdaTorqueTuneSeeded`, because manager_init writes the 0.0 default to disk before card runs).
v2 is v0's algorithm (setpoint == the live request, error corrected
in lateral acceleration space, the speed-dependent extension owning the feedforward params)
plus four mechanisms. Each one is there because the 2026-09-01 leave-one-out replay of the
previous v2 (routes 132, 139, 12d, 12f, 123, 124, 126) tied it to a felt improvement.

## What is in v2 and why

| mechanism | does | measured reason |
|---|---|---|
| filtered-jerk friction input | the friction term sees the request differencer through a 1.2 Hz low-pass, clipped at 2.5 m/s^3, with a small-signal deadzone inside \|setpoint\| < 0.35 m/s^2; the PID error is untouched | on the old v2 the shaping was the whole -19% of sub-1 Hz command motion at lane center, most of it through the setpoint lead that was removed as a turn-in lag. On this structure it is -12% vs v0 (hf_rms 0.0372 -> 0.0327), the deadzone the larger part (0.0354 without it), the filter 0.0329 without it plus a smaller release transient (p50 0.126 -> 0.099). |
| KD, pressed-gated | `kd = 0.3 s * KP(v)` between 7.5 and 14.5 m/s on the measurement rate, zero while `steeringPressed` | removing it returns ~85% of the low-speed exit counter-swing (applied 0.313 -> 0.415, v0 0.433); the logged KD vs no-KD pair halves the exit tail tracking error (0.188 vs 0.489 RMS) |
| release handling | one-shot `i *= 0.8` on the press falling edge and a 0.3 s ramp-in of the PID error; feedforward not ramped | at release the error the driver left is p90 0.67 m/s^2 and P landed all of it in one frame (P swing p90 2.24 in 100 ms); the largest release-window delta of any mechanism when removed (0.19 / 0.88 p50 / p90) |
| curvature buffer + inactive priming | the delayed-request buffer stores curvature and is rescaled by the live v^2; while inactive the buffer and both rate filters track the live command | a lat-accel buffer reads the speed change as jerk, which now reaches the friction input (applied exit counter-swing +0.01..0.05); re-engaging against a stale buffer pushes the friction term against a held wheel until it refills |

## What left, and where it went

- Steer-limit classification and the EPS rail moved to the shared layer (`steer_limit.py`,
  `LatControlTorqueExt.update_override_torque_params` setting `lac.steer_max`). The tune
  receives `steer_limited_by_safety` meaning driver-limited only and keeps v0's freeze on it;
  the PID limits and v0's saturation check land at the rail with no tune code. This replaced
  the directional freeze, `_rail_limit_scale` and the rail-aware saturation block.
- Plan-secant setpoint jerk, divergence blend, stale-model fade, lead speed fade: no benefit
  in any regime, a 4-9 frame lag at turn-in, 2 extra reversals/s on straights.
- Unwind freeze, 0.3 m/s integrator threshold, roll/offset fade: inert (<= 0.002 output RMS).
- Rail PID limits inside the tune: inert on applied torque once the integrator is clean.

## Invariants (pinned by `tests/test_latcontrol_torque_v2.py`)

- With KD = 0, the deadzone = 0 and the jerk filter bypassed, v2 == v0 frame for frame on a
  moving request and a moving measurement, friction included.
- KP schedule identical to v0 at every speed.
- Extension output overrides (jerk-aware, NNLC) disabled regardless of params.

## Replay acceptance (open loop, `attrib/loo_replay.py`, routes 132 + 139 + 12d + 12f)

NEW vs the previous FULL: STRAIGHT hf_rms within 5%; low-speed EXIT applied counter-swing
within 0.02; TURN_ENTRY t50 not later; RELEASE max delta p90 <= 0.4.

## On-car acceptance (one drive, same roads as 139)

`tools/mazda_long/tune_version_metrics.py` split by gitCommit, new tune vs 139: straight-line
weave and step RMS not above 139; low-speed exit counter-swing and reversals not above 139;
hand-back range p50 not above 139 and no "abrupt at first" report; interventions per 100 km
not above 139; `lkas_starvation_check.py` zero rate-down bursts.
