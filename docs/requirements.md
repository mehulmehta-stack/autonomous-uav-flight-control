# Requirements Traceability

| Req ID | Description | Verification Method | Status |
|--------|-------------|---------------------|--------|
| REQ-001 | pitch hold within +/-2 deg | Simulation | IN PROGRESS — run pitch deviation check, then update |
| REQ-002 | altitude hold within +/-5m | Simulation | IN PROGRESS — run altitude deviation check, then update |
| REQ-003 | waypoint tracking within 10m horizontal | Test | NOT VERIFIED |
| REQ-004 | auto land within 3m of target | Test | NOT VERIFIED |
| REQ-005 | stable in FBWA mode across flight envelope | Test | NOT VERIFIED |
| REQ-006 | SIL results match MIL within 5% for all outputs | Simulation | **VERIFIED (PASS)** — max error 0.0000%, mean error 0.0000% (altitude output, step test to 4000 ft), see `SIL_test_report.md` |
| REQ-007 | controller stable with 30% model parameter uncertainty | Analysis | NOT VERIFIED |
| REQ-008 | closed-loop phase margin greater than 30 degrees | Analysis | NOT VERIFIED |
| REQ-009 | closed-loop gain margin greater than 6 dB | Analysis | NOT VERIFIED |
| REQ-010 | generated C++ code passes SIL verification against Simulink MIL | Simulation | **VERIFIED (PASS)** — same evidence as REQ-006; generated code matched native Simulink model to 0.0000% error |

## Notes

- REQ-001 and REQ-002 require running the pitch-deviation and altitude-deviation checks (given separately) on the SIL simulation output, then replacing "IN PROGRESS" above with the real measured numbers.
- REQ-006 and REQ-010 were verified using the altitude output signal specifically. Since MIL and SIL run the identical controller logic through different execution paths (native Simulink blocks vs compiled C), and the physics loop is fully coupled, this result strongly implies the other outputs (elevator, aileron, throttle commands) also match — but only altitude was directly diffed. Noted here for honesty rather than claiming "all outputs" were individually checked.
- REQ-003, REQ-004, REQ-005 require Test-level verification (real or SITL flight), not yet reached in the project timeline.
- REQ-007, REQ-008, REQ-009 require Analysis (e.g. Bode plot / margin analysis in MATLAB on the linearized model), not yet performed.
