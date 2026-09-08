# Guidance/ — L1 Lateral Guidance Prototype

## What this is, and what it isn't

`l1_guidance_prototype.py` is a **standalone 2D kinematic prototype** of the L1
guidance law — pure Python, no Simulink, no JSBSim, no 6-DOF aircraft
dynamics. A point mass moving at constant speed, tracking a single straight
leg between two waypoints.

**This is not the flight guidance.** The guidance actually flying the
aircraft — integrated with the 9-state EKF, the gain-scheduled DLQR, and
full nonlinear JSBSim dynamics, verified against a real 4-waypoint mission
(`docs/requirements.md`, REQ-003, REQ-011–014) — lives in `controller/`,
as part of the final Simulink model. If you're looking for evidence about
the real system's behavior or limits, that's where it is, not here.

## Why this file is kept anyway

Same core L1 math — the cross/dot/eta formulation that computes lateral
acceleration from vehicle position and velocity relative to the desired
track — is what actually runs inside the final controller. This script is
where that law was worked out and proven correct in isolation, closed-loop
and properly working (`a_cmd` genuinely drives the vehicle's motion here,
unlike an earlier, unfixed attempt at integrating it with the EKF that was
removed from this repo entirely — see `JOURNEY.md`). It's the honest
starting point of the guidance law that made it into the real system, nothing more.

## Files

- `l1_guidance_prototype.py` — the prototype: L1 guidance law + a simple
  closed-loop kinematic simulation, one straight leg, one disturbance
  (30m initial cross-track offset).
- `l1_path_tracking.png` — the flown path converging onto the desired leg.
- `l1_lateral_acceleration.png` — the commanded lateral acceleration
  through the correction.
- `l1_eta.png` — the L1 guidance angle through the correction.

All three plots show clean, damped convergence — no discontinuities, no
sustained oscillation. This is the guidance law behaving correctly in the
simplest case it will ever see; the real system's more complex behavior
(the REQ-014 WP2/WP4 finding, for instance) only shows up once EKF
estimation, gain-scheduled attitude control, and full mission geometry are
all in the loop together — which is exactly why that evidence lives in
`docs/final_model_evidence/`, not here.
