# zoompilot notes

Findings behind the fork's constants and design choices, kept out of the code comments.
Each file carries a Constants table (name, value, measurement, route) and a Tried and
rejected section. Rlogs and the analysis scripts live in the private zoompilot-research
repo.

- mazda-longitudinal.md: radar takeover and hand-back, CRZ_INFO checksum, stop-and-go,
  MRCC state semantics, alpha-long availability
- mazda-lateral.md: 2022 EPS detection and flag, 1200/12/12 envelope, speed-dependent
  STEER_MAX, LKAS_BLOCK and the non-delivery latch, camera ERR_BIT_1 history, the camera's
  own TJA/CTS state, the bus-2 camera press and the TJA button as the MADS switch
- mazda-fingerprinting.md: VIN decode table and the EPS-swap fallback
- mads.md: the two lateral machines, engaging with the brake held in pause mode
- lateral-tune.md: v0/v1/v2 lineage, the v2 mechanisms and their attribution, the
  steer-limit classifier, the speed-bin learner and its cache
- lateral-tune-roadmap.md: the empirical roadmap for the torque tune
- cruise-arbiter.md: setpoint ownership, SLA sessions, dismiss semantics, the reconciler
- icbm.md: the button servo, actuation profiles, fast mode, restore quiet window
- e2e-set-speed.md: experimental mode's set-speed floor, the model's lazy pace, trips and their
  separation data, the rejected stateless blend
- e2e-lead-gap.md: experimental mode's follow-distance assist, how far back the model follows,
  its speed-matching behaviour, the lift tied to the MPC candidate
- lead-forecast.md: the model's lead forecast as the MPC obstacle, forecast accuracy against
  upstream's extrapolation, the replay A/B and why it is seeded from the log
- scc-curve-planning.md: the Mazda / upstream planner split, model curvature range bias, the
  whole-path horizon with a per-model near window, the commit hold below 50 mph,
  publish_ramp and the op-long budget, map retain logic and confirmation time
