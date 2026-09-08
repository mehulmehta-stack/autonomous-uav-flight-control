# docs/ — Index

`requirements.md` is the master table. This file explains how evidence
underneath it is organized — specifically, how checkpoint-controller
evidence stays visibly separate from final-gain-scheduled-model evidence,
so nobody has to infer which is which from a label in a table cell.

## Two evidence folders, split by which controller they verify

### `checkpoint_evidence/`
Everything verified against the **single-point checkpoint controller** —
the design that predates gain scheduling, L1 integration, and actuator
dynamics. Most of this evidence is kept for comparison against the
final model's equivalent results.

**Exception:** the original REQ-001/002/006/010 run (`SIL_test_report.md`,
`mil_vs_sil_altitude.png`) was not carried forward into this delivery —
it's superseded entirely by the final model's own REQ-006/010 SIL
comparison (`docs/final_model_evidence/`), which covers the same ground
with real, current numbers. Its absence here isn't an oversight; it's
not needed once the final-model equivalent exists and is stronger
evidence than what it would have compared against.

- REQ-007, REQ-008 original single-point run (`REQ-007_30pct_model_uncertainty_analysis.md/.m/.mat`, `REQ-008_REQ-009_stability_margin_analysis.md/.m/.mat`) — the design script's own sanity check depends on this comparison existing
- REQ-015 original run (`SIL_robustness_stress_test.md` and its three plots)

### `final_model_evidence/`
Everything verified against the **gain-scheduled controller** — the
current, final design.

- REQ-001, REQ-002 (`REQ001_REQ002_final_model_verification.m/.png/.mat`)
- REQ-003 (`cross_track_error_checker.m`, `CrossTrackError.png`)
- REQ-006, REQ-010 (`run_sil_mil_comparison.m` + 3 comparison PNGs +
  `REQ006_REQ010_final_model_results.mat` — all three outputs, max error
  0.0051%)
- REQ-007, REQ-008, REQ-009 gain-schedule-wide (`REQ008_REQ009_gain_scheduled_extension.m` + `.mat` results — all three closed)
- REQ-011 through REQ-014 (all `GainSchedule_*`, `L1_*Diagnostic*`,
  `L1_Eta_Discontinuity_Check*`, `L1_GroundTrack_AfterFix.png`,
  `gain_schedule_extrapolation_check.png`)
- REQ-015 (`REQ015_final_model_altitude_recovery.m/.png/.mat`, plus its
  real IC pair `c172p_cruise_init_stress.xml` / `c172_cruise_8K_FINAL_FG_STRESS.xml`)
- REQ-016, REQ-017 (`GainSchedule_Validation.*`,
  `LateralGainSchedule_Validation.*`)
- Supporting: `GPS_Reacquisition_Characterization.*` (the finding that
  justified adding actuator dynamics — not itself a numbered requirement)

### Status
**All 17 requirements now have final-model evidence.** Nothing is
checkpoint-only anymore — `checkpoint_evidence/` is kept purely for
historical comparison, not because anything is still pending.

## Unattributed / methodology-only
`verify_actuator_dynamics.m` and `run_model_advisor_C172P.m` — real
scripts, right approach, never run to completion. Kept at the `docs/`
top level, not in either evidence folder, since they don't yet verify
anything.
