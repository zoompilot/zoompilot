# Mazda startup and radar-session refactor

Scope: every platform in the current Mazda port (CX-5 KE/KF/2022, CX-9/2021, Mazda 3,
Mazda 6), including supported EPS swaps and the G46L radar dialect. This is shared
behavior for supported configurations, not an expansion of alpha-long eligibility.

## Evidence and priorities

The startup comparison contains 371 Mazda captures with usable CarParams and panda
health, 74 alpha-long and 297 stock-long. The supplied five captures all identify the
car, never send a radar-disable request, and have severe sustained CAN errors. Ordinary
startup interrupt warnings are also present in comparison logs, so they are not enough
to diagnose a hardware failure. See `mazda-boot-radar-2026-09-09.md` for measurements.

Of 60 comparison captures containing programming-session requests, 59 contain the
`06 50 02` response on the main bus. The session parameter P2* is a diagnostic response
timeout; it must not be confused with the separately observed S3 inactivity timeout.
Traffic remains the necessary evidence for recovery: a default-session acknowledgement
does not prove that periodic radar messages have resumed.

The code review found three independent representations of radar ownership: the
controller state machine, CarState's silence timer, and the toggle monitor's interpretation
of accFaulted. The refactor makes their contracts explicit rather than adding more delays.

## Targeted implementation

1. Extract diagnostic-session ownership from the acceleration/standstill algorithms into
   `radar_session.py`. The manager owns request scheduling, tracks programming/default
   replies, logs transition reasons, and distinguishes restoration failure from completion.
2. Observe radar silence only while independent vehicle messages remain fresh. Require
   current CAN validity before initiating teardown or adopting an already quiet radar.
   Preserve the established FSC settle and radar-silence timing; do not infer a successful
   teardown from a disconnected bus or a request that was never scheduled.
3. Gate takeover on raw stock cruise engagement, not the public flag suppressed during
   startup. Recheck prerequisites while requests are pending and restore an outstanding
   request if prerequisites disappear.
4. Let the controller explicitly report replacement ownership to CarState. Availability
   requires that ownership and the existing panda-ordering guard. Lost ownership revokes
   availability and requires an idle sample before accepting another engagement.
5. Complete an in-flight restoration even if the toggle reverses. Suppress overlapping
   synthetic traffic as soon as stock returns; require sustained fresh stock traffic
   before permitting a process cycle. A timeout stops diagnostic retries and reports a
   failure; it does not restart processes or claim restoration. Late recovery can finish.
6. Pass the manager's result directly to the card toggle monitor. Keep expected stock
   restoration distinct from an unexpected radar return and from actual ECU fault bits.
   During handback, emit disengaged commands while preserving the driver's main-switch
   state. Continue already-owned replacement traffic through the bounded recovery attempt.

7. Broker every other software stop through the same hand-back (`stock_ecu_handback.py`):
   hardwared holds `OnroadCycleRequested` (calibration reset, restart-needed toggles) and
   `OffroadModeRequested` (forced offroad; pandad reads `OffroadMode` directly and drops the
   panda's ignition within 100 ms, so the UI writes the request instead), manager holds
   `DoReboot` / `DoShutdown` / `DoUninstall`. Superseded 2026-09-10 by the correlated
   request/result records and the moving-takeover contract in
   `force-offroad-alpha-transition.md`; remote `OffroadMode` writes are blocked there. Not
   holdable: ignition off, power loss, thermal-critical offroad.
8. Bus witnesses at the CANParser's own ten-period validity (PEDALS 200 ms, ENGINE_DATA
   100 ms), not a tighter window the parser would still accept. Once owned, ownership is
   held on the controller's claim through a blip; only stock traffic ends it.

## Validation and limits

- Preserve the existing golden command fixture outside any independently justified wire
  behavior changes. Run Mazda car tests, compiled panda safety tests and toggle tests.
- Test all seven platform entries, native/legacy/swapped EPS, available/unavailable radar,
  and both supported radar dialects. Stock-long configurations must issue no session traffic.
- Exercise whole-bus loss, one missing vehicle witness, stale speed, ordinary radar gaps,
  a request not yet emitted, negative/malformed/pending replies, moving or engaged takeover,
  every UDS scheduling phase, timeout, late recovery and toggle reversal.
- Status 2026-09-09: implemented on the working tree, 524 Mazda + 234 main-repo tests green,
  golden TX fixture unchanged. On-car pending: a parked toggle flip, a calibration reset
  onroad, forced offroad onroad, and a reboot onroad, each checked for stock CRZ_INFO back on
  the bus before `started` drops and no cluster warning on the next ignition.
- Replay recorded startup CAN through the current parser/controller to check session
  transitions against observed traffic. This tests software decisions on recorded inputs;
  it cannot simulate the ECU's response to a changed diagnostic request.

Retain steering rejection recovery and the firmware-specific first-engagement hold.
Do not mask camera/EPS error bits, force a fingerprint after failed communication, raise
panda interrupt limits, or enable alpha long on an unsupported radar/EPS combination.
Physical CAN corruption and ECU-latched faults remain separate on-car validation work.
