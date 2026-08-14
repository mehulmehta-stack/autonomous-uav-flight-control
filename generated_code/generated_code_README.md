# Auto-Generated Autopilot C++ Code

## What this is

This folder contains C code **automatically generated from a Simulink controller model** (`JSBSim_DLQR_PID_OuterLoop_AltitudeHold_c172p`) using Simulink Coder — not hand-written. The controller itself was designed and validated as a Simulink block diagram; this code is the direct, automatic translation of that diagram into C, ready to run on embedded hardware.

- **Model version:** 1.340
- **Generator:** Simulink Coder 25.1 (R2025a)
- **Target:** GRT (Generic Real-Time), Intel x86-64 Windows64
- **Sample time:** 0.008333 s (120 Hz, matching JSBSim's simulation step)

## Why this matters

Real certified flight control software is never hand-typed from scratch — it's designed graphically in a model-based design tool, then auto-generated into code. Airbus and Boeing use **SCADE** for this, which has the added property of being pre-qualified to **DO-178C**, meaning its generated code doesn't need to be manually re-verified line by line before flying on a real aircraft.

Simulink Coder — what generated this code — performs the *same workflow* (model → auto-generated code → test the generated code) but without that certification stamp. Getting Simulink-generated code to that certified standard requires a separate paid add-on (MathWorks' DO Qualification Kit) that few students ever have access to.

This folder is proof of practicing that real industrial workflow, using the academic-accessible version of the same pipeline used in certified aerospace flight software development.

## Controller architecture implemented in this code

**Inputs (6):** airspeed (U), pitch rate, altitude, angle of attack, roll angle (phi), actual pitch angle (theta)
**Outputs (3):** Throttle command, Aileron command, Elevator command

Three separate control loops, cascaded and combined:

1. **Airspeed → Throttle** — a PI controller (Kp = 0.01, Ki = 0.001, Kd = 0, so derivative is intentionally disabled) drives throttle to hold `184.999 ft/s` (~110 knots) airspeed.

2. **Altitude → Pitch → Elevator (the main cascade)** — a PID controller (Kp = 0.01, Ki = 0.0001, Kd = 0.001) compares current altitude against a `4000 ft` target and outputs a commanded pitch adjustment (clamped to ±10–15°). That commanded pitch is added to trim and fed into an **LQR state-feedback inner loop**, which combines airspeed error, angle-of-attack, pitch error, and pitch rate — each weighted by a gain vector `K = [0.0205, 2.2685, −7.4923, −2.1182]` — to compute the final elevator deflection.

3. **Roll → Aileron** — a simple proportional roll damper (`Gain = −1.0 × roll angle`), no integral or derivative term.

## Trim condition

The whole controller is designed around straight-and-level trimmed flight at **184.999 ft/s**, with trim pitch and angle-of-attack both at **0.00266 rad (~0.15°)** — confirming the aircraft was properly trimmed before any controller design began.

## Files in this folder

- `Autopilot.c` — the controller's step logic (what runs every 0.008333s)
- `Autopilot_data.c` — the actual tuned parameter values (gains, trim points, saturation limits)
- Screenshots — folder structure from code generation, and the LQR gain array as generated
