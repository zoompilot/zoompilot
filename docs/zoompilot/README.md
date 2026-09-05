# zoompilot notes

Findings behind the fork's constants and design choices, kept out of the code comments.
Each file carries a Constants table (name, value, measurement, route) and a Tried and
rejected section. Rlogs and the analysis scripts live in the private zoompilot-research
repo.

- mazda-longitudinal.md: radar takeover and hand-back, CRZ_INFO checksum, stop-and-go,
  MRCC state semantics, alpha-long availability
- mazda-lateral.md: 2022 EPS detection and flag, 1200/12/12 envelope, speed-dependent
  STEER_MAX, LKAS_BLOCK and the non-delivery latch, camera ERR_BIT_1 history, TJA button
- mazda-fingerprinting.md: VIN decode table and the EPS-swap fallback
- lateral-tune.md: v0/v1/v2 lineage, the v2 mechanisms and their attribution, the
  steer-limit classifier, the speed-bin learner and its cache
- lateral-tune-roadmap.md: the empirical roadmap for the torque tune
- cruise-arbiter.md: setpoint ownership, SLA sessions, dismiss semantics, the reconciler
- icbm.md: the button servo, actuation profiles, fast mode, restore quiet window
- scc-curve-planning.md: model curvature range bias, the highway near-window horizon,
  publish_ramp and the op-long budget, map retain logic
