# MADS: keeping the software and panda lateral machines in step

MADS lateral is arbitrated twice. `openpilot/sunnypilot/mads/mads.py` decides when the
software steers; `opendbc/safety/sunnypilot/mads.h` decides whether the panda passes a
steering frame to the EPS. Neither can ask the other. Each arms on an edge it observes on the
bus itself (ACC main rising, a MADS button, `controls_allowed` rising, a brake release in pause
mode) and each drops lateral on its own triggers. They stay in step only while both consume
the same edges on the same frames. When they do not, the panda rejects every steering frame,
the software counts 20 frames to the orange "Steering Blocked by Panda Safety" and 200 frames
to the red "Controls Mismatch: Lateral", and the driver has to find an edge both sides accept.

## Engaging with the brake already down (pause-on-brake)

Route `3b1bf43f9b2e8016|00000004--00dac0887c` seg 12 (2026-09-05, Mazda 3 with a CX-5 2022
EPS, stock ACC, `MadsSteeringMode` = pause): stopped at a light with the brake held, the driver
pressed ACC main three times (41.6, 52.4, 54.4 s). Each time CRZ_CTRL bit 17 rose, the software
raised `lkasEnable` and went straight to active, and the panda never armed. The PEDALS brake
bit was high on all 1999 frames of the window.

The panda side is deliberate: in pause mode `m_update_control_state` refuses to arm while
`braking.current` is set, keeps `controls_requested_lateral` pending, and arms on the release
frame. The software mirrors the pause with `pedal_pressed_non_gas_pressed`, which reads the
`pedalPressed` event, and selfdrived only raises that event on a brake edge or while moving.
A brake already held at standstill raises nothing, so the software engaged into a state the
panda would not honour. Seg 5 of the same route shows the other direction healthy: brake press
pauses both, brake release re-arms both within 10 to 50 ms.

Both files are byte-identical to sunnypilot upstream on this path, so this is a latent upstream
bug. The default steering mode is remain-active, which is why it is rarely seen.

### The fix

Gate engagement on the brake level, the signal the panda uses. In pause mode, when `lkasEnable`
arrives with `brakePressed` or `regenBraking` set, `mads.py` adds
`silentPedalPressed`, a silent NO_ENTRY event listed in `GEARS_ALLOW_PAUSED_SILENT`, so the
state machine takes its existing disabled-to-paused path. The brake-release check that already
sends `silentLkasEnable` resumes the software on the same frame the panda arms its pending
request. Nothing changes for remain-active or disengage modes, for the moving case (which
already paused one frame later through the event), or for the pause-and-resume path.

Tests: `openpilot/sunnypilot/mads/tests/test_mads_steering_mode.py`, `TestEngageWithBrakeHeld`.

## Constants

| Name | Value | Where | Measurement |
| --- | --- | --- | --- |
| `LATERAL_MISMATCH_WARN_FRAMES` | 20 | mads.py | pandaStates is 10 Hz, a fresh arm reads stale for at most 10 frames; twice that |
| `LATERAL_MISMATCH_DISABLE_FRAMES` | 200 | mads.py | upstream's `controlsMismatch` window |
| Panda re-arm after brake release | same frame | seg 5 | 10 to 50 ms behind the software's `silentLkasEnable` in pandaStates |

## Tried and rejected

- **Making the panda arm while braking.** Touches the shared mads.h, and the panda's rule is
  the safer one: a pending request that lands on the release is what pause mode promises.
- **Making the mismatch non-sticky (pause at the warn threshold instead of the red).** Hides
  the next desync of this class instead of fixing it; the driver would lose steering with no
  alert. Still worth revisiting as a guard once the edges are known to match.
- **Raising `pedalPressed` for a brake held at standstill.** Changes upstream selfdrived
  semantics that other events rely on (openpilot's own pedal disengage).
