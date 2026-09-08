# controller/ — Final Controller Build

This folder holds the **current, final** flight controller — not history.
It's now two files, not one, since the SIL verification work (see below)
required splitting the controller out as its own Model Reference.
Superseded design stages (the checkpoint controller, early roll
controllers, pre-gain-scheduling versions) live in `history/` instead.

## ⚠ Before running anything in this model: `Initial Altitude`

The altitude-hold loop computes its error as:

```
error = Constant (4000, commanded altitude)
        − [ Initial Altitude (Constant) + EKF_Altitude_ft (−3.28084 × D_ekf) ]
```

`Initial Altitude` is a hardcoded Constant block and **must match whatever
altitude the aircraft actually starts at for the specific test being run.**
It does not read this from the JSBSim IC file automatically. If it's left
mismatched, the controller computes a near-zero altitude error and applies
no correction — this looks like "the aircraft holds wherever it started,"
but it's actually two hardcoded constants canceling each other out. See
`JOURNEY.md` for the full trace of this finding (found while closing
REQ-015).

**Standing rule until this is fixed properly:** set `Initial Altitude` to
match the real starting altitude before every run that doesn't start at
4000 ft, and set it back to 4000 afterward.

**Real fix, correctly scoped as future work:** replace this Constant with
a Sample-and-Hold (or Memory block triggered once at t=0) that latches the
true starting altitude automatically — the same self-initializing pattern
`Navigation_EKF_LQG.m` already uses correctly for N/E/D position, from the
first real GPS reading rather than an assumption.

## Model structure: two files, not one, since the SIL work

The controller subsystem was converted to a **Model Reference** to support
SIL verification (REQ-006/010) — it's no longer a subsystem inside the
main model, it's its own file:

- **`Autopilot.slx`** — the controller itself (formerly the `Autopilot`
  subsystem, now `Simulink.SubSystem.convertToModelReference`'d into a
  standalone model). This is what gets code-generated into `generated_code/`.
- **The main flight model** (`Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy`) —
  now references `Autopilot.slx` via a Model block instead of containing
  the controller inline. Toggling that Model block's `SimulationMode`
  between `Normal` and `Software-in-the-loop (SIL)` is how REQ-006/010
  were verified — see `run_sil_mil_comparison.m` in `docs/final_model_evidence/`.

Getting `Autopilot.slx` to actually pass SIL simulation required finding
and fixing three real issues, not just the conversion itself — full
detail in `JOURNEY.md`, summarized here since they live in this file:

1. **Actuator dynamics moved out.** `RateLimiter_Elevator/Aileron` and
   `ActuatorLag_Elevator/Aileron` had ended up inside the controller
   boundary. Continuous states aren't supported in Model-block SIL, and
   more fundamentally, actuator physics isn't controller logic — moved
   back to the parent model, matching where `add_actuator_dynamics.m`
   originally placed them.
2. **`Clock` → `Digital Clock`.** A plain `Clock` block feeding
   `GPS_Validity` was flagged as a continuous-time element independent of
   the actuator dynamics — `Clock` has no sample time by definition, even
   with a fixed-step discrete solver configured. Replaced with `Digital
   Clock` at 0.008333s; `GPS_Validity`'s behavior is unchanged.
3. **Seven REQ-014 diagnostic taps removed** (`val_L1eta`, `val_L1phicmd`,
   `val_L1speed`, `val_L1wpidx`, `ekf_E_log`, `ekf_N_log`,
   `gps_valid_log`) — leftover instrumentation from the WP2/WP4
   investigation that had no path to the subsystem boundary. Reproducing
   that investigation on this model would require re-adding them as new
   Outports first.

## Trim scheduling — added after the fact, not part of the original design

`K` (via `Lon_GainSched`/`Lat_GainSched`) was always correctly scheduled
with airspeed. The trim reference values it operates against — originally
`alpha_trim`, `Thita_trim`, and one of two velocity trims — were not:
plain hardcoded Constants at the 185 ft/s cruise values, applied
regardless of which schedule point was actually active. Fixed with
`Lon_TrimSched(V)`, a MATLAB Function scheduling those three off the same
airspeed signal `K` already uses. Only three of the five original
constants were changed — `velocity_trim2` and `Throttle` correctly stay
fixed, since they represent the throttle loop's one commanded airspeed,
not the LQR's local linearization reference. Full derivation and the
distinction between the two roles in `JOURNEY.md`.

## Actuator dynamics wiring — broken by an earlier fix, since corrected

Moving `RateLimiter`/`ActuatorLag` out of `Autopilot` (to fix the SIL
continuous-time blocker, above) left their outputs disconnected —
correctly *receiving* the raw command, but not feeding anything
downstream. JSBSim received the raw, unfiltered controller output
directly for a period spanning several verification runs, including the
original REQ-006/010 SIL comparison and an early REQ-011/012 recheck.
Fixed by wiring `ActuatorLag`'s output (not `RateLimiter`'s — the lag
stage matters too, not just the rate limit) into the Vector Concatenate
feeding JSBSim, replacing the raw bypass. See `JOURNEY.md` for how this
was traced and confirmed.



- `Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy.slx` — the main flight model
  (JSBSim co-simulation + Model block referencing `Autopilot.slx`)
- `Autopilot.slx` — the controller itself, code-gen source for
  `generated_code/`
- `setup_autopilot_model_reference.m` — the one-time conversion script
  (subsystem → Model Reference), kept for provenance even though it only
  needs to run once
- `C172_Lateral_DLQR_2state_FINAL.m` (+ `.mat`) — final 2-state roll
  controller design; matches the 185 ft/s row of the gain schedule exactly
- `gain_schedule_generate_cases.py`, `gain_schedule_generate_stress_cases.py`
  — JSBSim trim/linearization automation for the 3 design points and the
  2 envelope-extreme points
- `gain_schedule_design_3pt.m` (+ `C172_GainSchedule_3pt.mat`) — the final
  K1/K2/K3 gain matrices
- `Navigation_EKF_LQG.m`, `apply_ekf_state_feedback.m` — 12-state estimated-feedback
  wiring (p̂/q̂ replacing raw sensor feedback in both LQR loops). This is
  implemented and real; there is no separate formal LQG-object performance
  validation — don't describe it as more than the wiring it actually is.
- `add_actuator_dynamics.m` — rate limiter (±1.0 normalized units/s) and
  first-order lag (τ=0.1s) on elevator and aileron, now applied in the
  parent model (see above). These parameter values are explicitly
  documented, in the script's own comments, as representative small-GA-
  servo assumptions, not a verified hardware spec — the C172 has no
  factory autopilot servo to match against.
- `fix_matlab_function_sampletimes.m` — required once the actuator dynamics'
  continuous Transfer Fcn blocks were added; pins several MATLAB Function
  blocks to the model's fixed sample time explicitly, since "inherited"
  sample time stopped resolving to discrete automatically once continuous
  states existed elsewhere in the model.

## Verification status

Verification evidence for this model lives in `docs/final_model_evidence/`,
not here — this folder is the build, not the proof. See
`docs/requirements.md` for what's been verified and how.
