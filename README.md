# Autonomous UAV Flight Control — C172P

A fixed-wing flight control and navigation stack built around a JSBSim/Simulink
Software-in-the-Loop C172P model: gain-scheduled discrete LQR attitude control,
PID outer loops, a 9-state INS/GPS Extended Kalman Filter, L1 lateral guidance,
and a requirements-based verification process modeled on the practices used in
certified aerospace flight software development.

This repository documents the complete engineering process — including the
mistakes, the checkpoints, and the fixes — not just the final result. See
[`JOURNEY.md`](./JOURNEY.md) for the full development history.

**New here? Start with [`GETTING_STARTED.md`](./GETTING_STARTED.md)** — it
walks through the repo in build order, not evidence order.

## Highlights

- **Discrete LQR + PID cascade**, gain-scheduled across three trim points
  (100 / 185 / 220 ft/s), verified stable under ±30% coefficient uncertainty
  (10,000-trial Monte Carlo, 0 failures at every schedule point) with phase
  margins of 89–93° against a 30° requirement.
- **9-state nonlinear INS/GPS EKF**, validated through a 10-second GPS outage
  with correct IMU-only coasting and clean reacquisition.
- **L1 lateral guidance** integrated into a closed-loop EKF → L1 → LQR →
  actuator chain, demonstrated over an 800-second multi-waypoint mission.
- **Actuator dynamics** (rate limiter + first-order lag) added and verified
  after characterizing a real problem — a single-timestep elevator command
  spike to near-saturation during GPS reacquisition — not added speculatively.
- **Auto-generated C code from the final controller, SIL-verified against
  native Simulink** to within 0.0051% across all three control outputs
  (throttle, elevator, aileron) — the same model-based-design → code-gen →
  verify workflow used in certified aerospace flight software development.
- **17 traced requirements** (`docs/requirements.md`), each with its
  verification method, evidence, and — honestly — which requirements are
  proven against the final gain-scheduled controller versus an earlier
  checkpoint design.

## Current status

**All 17 requirements are verified against the final gain-scheduled
controller** — including a full SIL-vs-MIL comparison of the actual
generated C code (REQ-006/010), not just the earlier checkpoint version.
`docs/requirements.md` states plainly which evidence comes from the
checkpoint controller (kept for comparison, in `docs/checkpoint_evidence/`)
versus the final model (`docs/final_model_evidence/`) — nothing is left
ambiguous about which result applies to which version.

Known, documented limitations (not remediated, not hidden):
- A bounded 7.5° bank excursion during low-speed recovery, traced to engine
  torque/P-factor coupling (REQ-012).
- A repeatable path-crossing artifact at two of four waypoints in the L1
  guidance ground track, root-caused down to "likely the DLQR's response to
  the commanded bank angle" but not fully resolved (REQ-014).
- Waypoint tracking accuracy is geometry-bounded (~885 m max cross-track
  error against a ~1,061 m turn radius), not the originally-specified 10 m —
  that original number was unrealistic for this guidance law and leg
  spacing, and the requirement was rewritten to reflect what's actually
  achievable and tested (REQ-003).

## Repository structure

```
docs/              Requirements traceability, V&V evidence, standards notes
controller/        The final controller model and its build/design scripts
generated_code/    Auto-generated C from the FINAL controller, SIL-verified
EKF/               State estimation: 2-state → 6-state → 9-state progression
Guidance/          L1 lateral guidance, standalone and EKF-integrated
history/           Full chronological development record (in progress)
JOURNEY.md         Development log — what was built, what broke, what changed
requirements.md    (see docs/) — the primary verification traceability table
```

## Toolchain

MATLAB/Simulink, JSBSim, FlightGear (visualization), Python (EKF prototyping,
analysis).

## Verification methodology

Each requirement in `docs/requirements.md` is tagged with its verification
method (Analysis, Simulation, or Test) and links to the script or report that
produced its evidence. Where a result applies only to an earlier checkpoint
controller rather than the current final design, that's stated in the table
directly rather than left for the reader to assume.
