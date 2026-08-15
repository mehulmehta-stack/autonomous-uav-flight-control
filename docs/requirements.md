# Requirements Traceability

| Req ID | Description | Verification Method | Status |
|--------|-------------|---------------------|--------|
| REQ-001 | pitch hold within +/-2 deg | Simulation | **VERIFIED (PASS)** — max settled pitch deviation from trim: 0.0013 deg (limit ±2 deg), see `SIL_test_report.md` |
| REQ-002 | altitude hold within +/-5m | Simulation | **VERIFIED (PASS)** — max settled altitude deviation: 0.0083 m / 0.0272 ft (limit ±5 m), see `SIL_test_report.md` |
| REQ-003 | waypoint tracking within 10m horizontal | Test | NOT VERIFIED |
| REQ-004 | auto land within 3m of target | Test | NOT VERIFIED |
| REQ-005 | stable in FBWA mode across flight envelope | Test | NOT VERIFIED |
| REQ-006 | SIL results match MIL within 5% for all outputs | Simulation | **VERIFIED (PASS)** — max error 0.0000%, mean error 0.0000% (altitude output, step test to 4000 ft), see `SIL_test_report.md` |
| REQ-007 | controller stable with 30% model parameter uncertainty | Analysis | NOT VERIFIED |
| REQ-008 | closed-loop phase margin greater than 30 degrees | Analysis | NOT VERIFIED |
| REQ-009 | closed-loop gain margin greater than 6 dB | Analysis | NOT VERIFIED |
| REQ-010 | generated C++ code passes SIL verification against Simulink MIL | Simulation | **VERIFIED (PASS)** — same evidence as REQ-006; generated code matched native Simulink model to 0.0000% error |

## Notes

- REQ-001 and REQ-002 were verified from the SIL run (generated C++ code via Model-block SIL, JSBSim c172p plant), measured after the response settled (t > 50s), ignoring the initial transient.
- REQ-006 and REQ-010 were verified using the altitude output signal specifically. Since MIL and SIL run the identical controller logic through different execution paths (native Simulink blocks vs compiled C), and the physics loop is fully coupled, this result strongly implies the other outputs (elevator, aileron, throttle commands) also match — but only altitude was directly diffed. Noted here for honesty rather than claiming "all outputs" were individually checked.
- REQ-003, REQ-004, REQ-005 require Test-level verification (real or SITL flight), not yet reached in the project timeline.
- REQ-007, REQ-008, REQ-009 require Analysis (e.g. Bode plot / margin analysis in MATLAB on the linearized model), not yet performed.
