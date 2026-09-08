# Getting Started

This document is different from everything else in the repo. `README.md`
tells you what this project accomplished. `JOURNEY.md` tells you what was
found and fixed along the way. `requirements.md` tells you what's proven
and how. **This document tells you where to start and what order to do
things in** — read this first if you're new here.

Two paths, depending on what you want:

- **"I just want to see it run"** → Quick Start, below.
- **"I want to understand or rebuild the design pipeline"** → The Design
  Pipeline section, which walks through the same sequence this project
  was actually built in.

---

## Prerequisites

- **MATLAB + Simulink**, with:
  - **Control System Toolbox** (for `dlqr`, linear analysis)
  - **Simulink Coder** + **Embedded Coder** (for the code-generation and
    SIL verification work in `controller/` and `generated_code/`)
  - Stateflow is used implicitly — MATLAB Function blocks are built on
    it, so it needs to be installed even though nothing here uses
    Stateflow charts directly.
- **JSBSim** (the flight dynamics engine this project co-simulates
  with) — a separate install, not a MATLAB toolbox. The model expects
  it reachable on your system path; check the JSBSim S-Function block's
  parameters if the model can't find it.
- **Microsoft Visual C++** (or another MATLAB-supported C compiler) —
  needed for the code-generation and SIL steps specifically, not for
  running the model in normal (MIL) simulation.

[Guessing]: exact MATLAB version isn't pinned anywhere in this repo. If
you hit a compatibility error, check it against a recent release —
nothing here relies on cutting-edge features.

---

## Quick Start — just run it

1. Open **`controller/Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy.slx`**
   in Simulink. This is the top-level model — JSBSim co-simulation, plus
   a Model block referencing the controller.
2. Hit Run. You should see the aircraft hold altitude and airspeed, fly
   a 4-waypoint mission, and survive a scripted GPS outage (25–35s into
   any test that uses the default IC).
3. To see the controller in isolation, `controller/Autopilot.slx` is the
   Model-Reference target `Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy.slx`
   points to — open it directly to inspect the control law without the
   surrounding JSBSim co-simulation.

That's the whole "does it work" check. Everything below is for actually
understanding or rebuilding the pipeline.

---

## The Design Pipeline

This is the order the project was actually built in — each stage's
output feeds the next one. Script names below are all in `controller/`
unless noted otherwise.

### 1. Trim and linearization
Before any controller can be designed, you need a trim condition (a
steady-state flight condition) and a linear model around it. This
project uses JSBSim's own trim solver, run at three airspeeds (100 /
185 / 220 ft/s) to support gain scheduling later.
→ `gain_schedule_generate_cases.py` automates this: runs JSBSim's trim
process at each design point and extracts the A/B matrices JSBSim
computes via its own linearization.

### 2. Controller design (LQR) at each trim point
With A/B matrices in hand, `dlqr()` produces state-feedback gains K at
each of the three design points.
→ `gain_schedule_design_3pt.m` does this for the longitudinal axis;
`C172_Lateral_DLQR_2state_FINAL.m` for lateral. Output: `K1`, `K2`, `K3`
per axis, saved into `C172_GainSchedule_3pt.mat`.

### 3. Gain scheduling
Three fixed-point controllers aren't enough on their own — the aircraft
needs to fly *between* those design points too. The three K matrices get
wrapped into a MATLAB Function block (`Lon_GainSched`/`Lat_GainSched`,
inside `Autopilot.slx`) that linearly interpolates K against current
airspeed.

**A subtlety worth knowing before you rebuild this yourself:** K
scheduling alone isn't sufficient — the *trim references* the LQR state
error is computed against also need to schedule with airspeed, not stay
fixed at one design point's values. This project got that wrong on the
first pass (`Lon_TrimSched` didn't exist yet) and it took real work to
find and fix — see `JOURNEY.md`'s trim-scheduling section for the full
story and exactly which trim values need scheduling versus which need
to stay fixed. Worth reading before you assume "schedule everything with
airspeed" is the right instinct — it isn't, for all five trim
constants.

### 4. State estimation (EKF)
The controller above assumes it can measure the states it needs
directly. In reality, you have noisy IMU and intermittent GPS, not
clean state feedback. `EKF/` documents this piece's own build order —
start with `EKF/README.md`, which walks through the 2-state → 6-state →
9-state progression separately from this document, since it's a
self-contained sub-project with its own iteration history.
→ `Navigation_EKF_LQG.m` + `apply_ekf_state_feedback.m` wire the EKF's estimated
states into the LQR loops built in steps 2–3, replacing the assumption
of perfect state feedback.

### 5. Lateral guidance (L1)
For the aircraft to navigate to waypoints rather than just hold
attitude, it needs a guidance law computing what bank angle to command.
→ `Guidance/README.md` documents this piece's own history —
`Guidance/l1_guidance_prototype.py` is the standalone prototype the actual
L1 logic inside `Autopilot.slx` is based on. Read that README before
assuming any Python file in that folder is the real flight guidance —
it isn't, and that distinction caused real confusion earlier in this
project's own history (see `JOURNEY.md`).

### 6. Actuator dynamics
Real control surfaces don't move instantly — they have rate limits and
lag. This project adds both (rate limiter + first-order lag) explicitly,
justified by a specific finding, not added speculatively: a raw command
spike was found during GPS reacquisition, characterized directly, and
the actuator dynamics were added specifically to smooth it.
→ `add_actuator_dynamics.m`. **These blocks live in the parent model,
not inside `Autopilot.slx`** — deliberately: actuator physics isn't
controller logic, and Simulink's SIL verification doesn't support
continuous-time elements inside the referenced model. Full reasoning
in `JOURNEY.md`.

### 7. Verification
Every piece above has a corresponding verification script, and every
verification script maps to a specific row in `docs/requirements.md`.
That file is the master index — for any REQ-xxx you want to
reproduce, its evidence column names the exact script and output files
in `docs/final_model_evidence/`.

### 8. Code generation
Once the controller is verified, `setup_autopilot_model_reference.m`
converts it from an inline subsystem into a standalone Model Reference
(required for SIL testing), and `run_sil_mil_comparison.m` verifies the
auto-generated C code (`generated_code/`) behaves identically to the
Simulink model it was generated from.

---

## Common gotchas, worth knowing before you hit them yourself

- **The `Initial Altitude` constant must match your test's actual
  starting altitude.** It's not self-adjusting — if you run a test
  starting somewhere other than 4000 ft, this needs updating by hand
  first, or the altitude-error signal will be wrong from t=0. Full
  detail in `JOURNEY.md`.
- **`GPS_Validity`'s outage window (25–35s) is scripted, not dynamic.**
  Any test run using the default IC will show a GPS dropout in that
  window — expected, not a bug, if you see a transient there.
- **MATLAB Function blocks need explicit sample times once any
  continuous element exists elsewhere in the model.** `Clock` blocks
  specifically are continuous by definition, even with a discrete
  solver configured — use `Digital Clock` instead if you're adding
  anything that needs a time reference. See `JOURNEY.md` for two
  separate times this exact issue came up.
- **Trim scheduling is not "schedule everything with airspeed."** Two
  of the five original trim constants are deliberately fixed, not
  scheduled — see step 3 above and `JOURNEY.md` for exactly why.

## Where to go from here

- Full project narrative and all bugs found/fixed along the way:
  `JOURNEY.md`
- What's verified, with what evidence: `docs/requirements.md`
- Complete file map: `REPO_STRUCTURE.md`
