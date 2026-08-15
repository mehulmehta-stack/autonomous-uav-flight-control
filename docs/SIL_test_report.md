# SIL Test Report

## Test setup
Controller: generated C++ code (`Autopilot.c` / `Autopilot_data.c`), tested via Simulink Model-block SIL simulation
Plant: JSBSim c172p
Test: altitude step from initial condition to 4000 ft target

## MIL vs SIL comparison
(insert your MIL vs SIL altitude plot here)

- Max error: 0.0000%
- Mean error: 0.0000%
- Result: **PASS** — generated code matches native Simulink simulation essentially exactly, confirming the auto-generated C++ correctly implements the same control logic as the Simulink model it was generated from.

## Requirement verification (from SIL run)

| Requirement | Result | Evidence |
|---|---|---|
| REQ-001 (pitch returns to trim within ±2°, post-settling) | *pending* | run pitch deviation check, insert max deviation in degrees here |
| REQ-002 (altitude hold within ±5 m, post-settling) | *pending* | run altitude deviation check, insert max deviation in meters here |
| REQ-006 (SIL matches MIL within 5%) | **PASS** | 0.0000% max error, 0.0000% mean error |
| REQ-010 (generated code passes SIL verification against MIL) | **PASS** | same evidence as REQ-006 |

## Notes
- Trim condition for this test: airspeed 184.999 ft/s, trim pitch/alpha ≈ 0.00266 rad (~0.15°)
- Sample time: 0.008333 s (120 Hz), matching JSBSim's simulation step
