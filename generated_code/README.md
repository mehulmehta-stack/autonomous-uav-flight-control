# Auto-Generated Autopilot C++ Code

## What this is

This folder contains C code **automatically generated from the final gain-scheduled controller** — not the checkpoint version, not hand-written. Generated via Simulink's Model Reference + SIL workflow from `Autopilot.slx` (the extracted, atomic controller subsystem — see `controller/`), then verified against the native Simulink model.

**Verified, not just generated:** REQ-006 and REQ-010 (`docs/requirements.md`) confirm this code's behavior matches the native controller exactly — max error 0.0051% (elevator), effectively 0.0000% (throttle, aileron), against a 5% requirement. All three outputs directly compared, not inferred from one signal.

> **Superseded checkpoint version:** the original single-point-controller generated code (model version 1.340, the one this README used to describe) is not carried forward in this delivery — it's superseded entirely by the final model's gain-scheduled, estimated-state-feedback code above, which is stronger evidence covering the same ground. This folder now holds only the final version.

- **Generator:** Simulink Coder / Embedded Coder, Model Reference SIL workflow
- **Target:** GRT (Generic Real-Time), Intel x86-64 Windows64
- **Sample time:** 0.008333 s (120 Hz, matching JSBSim's simulation step)

## Why this matters

Real certified flight control software is generally designed graphically in a model-based design tool, then auto-generated into code, rather than hand-typed from scratch. Airbus and Boeing use **SCADE** for this, which supports **DO-178C qualification** for its code generator.

Tool qualification is worth being precise about, since it's a common point of confusion: qualifying a code generator doesn't mean its output is automatically certified, and it doesn't remove the rest of the DO-178C verification objectives — requirements-based testing, structural coverage of the *model*, and the other applicable objectives still apply. What qualification actually buys you is confidence in the model-to-code translation step itself, so that transformation doesn't need to be independently re-verified line by line the way hand-written code would. That's a real and valuable property — it's just narrower than "the code is certified."

Simulink Coder — what generated this code — performs the same model → code workflow, without a qualified generator behind it. Getting Simulink-generated code to that qualified standard requires a separate paid add-on (MathWorks' DO Qualification Kit) that few students have access to.

This folder is proof of practicing that real industrial workflow, using the academic-accessible version of the same pipeline used in certified aerospace flight software development — not a claim that this code itself is certified or certification-ready.

## Controller architecture implemented in this code

**Inputs (6):** `alpha_meas`, `U_meas`, `U_meas1` (air-data), `imu` (IMU vector), `gps` (GPS vector), `dt` (sample time)
**Outputs (3):** Throttle command, Elevator command, Aileron command

This is a meaningfully more complete artifact than the checkpoint version: rather than taking pre-computed clean states (airspeed, altitude, attitude) as inputs, **this subsystem performs its own state estimation and guidance internally**, from near-raw sensor data:

- **State estimation:** an onboard EKF (confirmed present via internal signal names during code-gen cleanup — see `JOURNEY.md`) converts IMU/GPS into the navigation state the controller actually uses.
- **Lateral guidance:** an internal L1 guidance law (`PID Controller`, `PID Controller1` — confirmed standard MathWorks library blocks, not custom — and an `L1_Guidance` reference) computes the bank-angle command.
- **Gain-scheduled inner loop:** the LQR/PID cascade uses gain-scheduled gains (`gain_schedule_design_3pt.m`, `controller/`) rather than the checkpoint's single fixed `K`.

**Explicitly outside this code-gen boundary, by design:** actuator dynamics (rate limiter + first-order lag) are *not* part of this generated code. They live in the parent model, applied to this code's raw output before it reaches JSBSim. Two reasons, both real: actuator dynamics represent physical servo hardware, not controller logic, and Simulink's Model-block SIL simulation does not support continuous-time interfaces — the actuator lag's continuous states would have blocked SIL verification entirely. See `JOURNEY.md` for the full story of finding and fixing this.

## Trim condition

The gain schedule spans three trim points: 100 / 185 / 220 ft/s true airspeed, all at 4000 ft altitude. `184.999 ft/s` (cruise) matches the checkpoint's original single trim point exactly, confirming continuity across the project's progression.

## Files in this folder

- `Autopilot.c` — the controller's step logic (what runs every 0.008333s)
- `Autopilot_data.c` — the tuned parameter values (gains, trim points, saturation limits)
- `Autopilot.h`, `Autopilot_private.h`, `Autopilot_types.h`, `rtwtypes.h`, `rtmodel.h` — headers required to compile `Autopilot.c`
