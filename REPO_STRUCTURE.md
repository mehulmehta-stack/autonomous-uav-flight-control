# Repository Structure — Final (rewrite, post REQ-006/010 closure)

`EKF/` is already sorted — not repeated in detail below, just confirmed
unchanged. Everything else follows the same categorization logic as
before:

- **`docs/`** — requirements.md plus every verification/diagnostic
  artifact, split into `checkpoint_evidence/` and `final_model_evidence/`.
- **`controller/`** — what builds and configures the current, final
  Simulink model. Now two `.slx` files, not one (see below).
- **`generated_code/`** — the **final model's** generated C code only.
  The checkpoint's old code was superseded, not archived.
- **`EKF/`, `Guidance/`** — subsystem prototyping journeys.
- **`history/`** — genuinely superseded stages only.

```
autonomous-uav-flight-control/
├── README.md
├── JOURNEY.md
│
├── docs/
│   ├── README.md
│   ├── requirements.md
│   │
│   ├── checkpoint_evidence/
│   │   ├── SIL_robustness_stress_test.md + 3 pngs
│   │   ├── REQ-007_30pct_model_uncertainty_analysis.md/.m/.mat/2 pngs
│   │   └── REQ-008_REQ-009_stability_margin_analysis.md/.m/.mat/.png
│   │
│   ├── final_model_evidence/
│   │   ├── REQ001_REQ002_final_model_verification.m/.png/.mat        (REQ-001/002)
│   │   ├── cross_track_error_checker.m + CrossTrackError.png         (REQ-003)
│   │   ├── REQ008_REQ009_gain_scheduled_extension.m
│   │   │     + REQ008_REQ009_gain_scheduled_results.mat              (REQ-007/008/009, all 3 points)
│   │   ├── run_sil_mil_comparison.m
│   │   │     + REQ006_REQ010_throttle_log_comparison.png
│   │   │     + REQ006_REQ010_elevator_log_comparison.png
│   │   │     + REQ006_REQ010_aileron_log_comparison.png
│   │   │     + REQ006_REQ010_final_model_results.mat                 (REQ-006/010, all 3 outputs)
│   │   ├── GainSchedule_Stress_Validation.png/.txt                   (REQ-011, evidence from a superseded script)
│   │   ├── validate_gain_schedule_v33rd.m
│   │   │     + GainSchedule_Stress_Coupling_Check.png                (REQ-011 + REQ-012, current source)
│   │   ├── gain_schedule_extrapolation_check.png                     (REQ-011 support)
│   │   ├── GainSchedule_CombinedMission_Test.png/.txt                (REQ-013)
│   │   ├── REQ013_stress_low_throttle_diagnostic.m/.png              (REQ-013, thrust-limit finding)
│   │   ├── L1_SpeedCoupling_Diagnostic.m/.png                        (REQ-014)
│   │   ├── L1_Eta_Discontinuity_Check.m/.png                         (REQ-014)
│   │   ├── L1_guidance_correction_check.m                            (REQ-014)
│   │   ├── ground_track_check.m + L1_GroundTrack_AfterFix.png        (REQ-014)
│   │   ├── pathimage.m + GroundTrack_WithWaypoints.png               (REQ-014, earlier check)
│   │   ├── validate_gain_schedule_v3.m + GainSchedule_Validation.png/.txt
│   │   │     (drop the byte-identical validate_gain_schedule_v32nd.m duplicate) (REQ-016)
│   │   └── validate_lateral_gain_schedule.m
│   │         + LateralGainSchedule_Validation.png/.txt               (REQ-017)
│   │
│   ├── standards_awareness.md
│   ├── controls_standards_deep_read_DO178C_SCADE_Polyspace.txt
│   ├── REQ008_REQ009_stability_margin_results.mat                    (checkpoint, exact original values)
│   ├── REQ007_30pct_uncertainty_results.mat                          (checkpoint, exact original values)
│   ├── characterize_gps_reacquisition.m
│   │     + GPS_Reacquisition_Characterization.png/.txt
│   │     (justifies actuator dynamics — supporting evidence, not a numbered REQ)
│   ├── add_imu_noise_logging.m, characterize_imu_noise.m + imu_noise_characterization_data.mat
│   ├── reportderive.m, secondreport.m
│   │     (architecture-inspection attempts — diary() bug confirmed broken)
│   ├── verify_actuator_dynamics.m, run_model_advisor_C172P.m
│   │     (right methodology, never run to completion)
│   └── archive/
│       └── requirements_early_draft_archive.md
│
├── controller/
│   ├── README.md
│   ├── Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy.slx        ← main flight model (Model block, not inline subsystem)
│   ├── Autopilot.slx                                  ← controller itself, code-gen source
│   ├── setup_autopilot_model_reference.m              (one-time conversion script)
│   ├── C172_Lateral_DLQR_2state_FINAL.m + .mat
│   ├── gain_schedule_generate_cases.py
│   ├── gain_schedule_generate_stress_cases.py
│   ├── gain_schedule_design_3pt.m + C172_GainSchedule_3pt.mat
│   ├── Navigation_EKF_LQG.m + apply_ekf_state_feedback.m
│   ├── add_actuator_dynamics.m                        (now applied in the main flight model, post code-gen boundary)
│   └── fix_matlab_function_sampletimes.m
│
├── generated_code/
│   ├── README.md
│   ├── Autopilot.c, Autopilot_data.c                  ← FINAL model's generated code (only version kept)
│   └── Autopilot.h, Autopilot_private.h, Autopilot_types.h, rtwtypes.h, rtmodel.h
│
├── EKF/                                                — already sorted, unchanged
│   ├── README.md
│   ├── nav_ekf.py + 5 pngs
│   ├── EKF_9State_Nav_Writeup.md
│   ├── gps_ekf_corrected.py + gps_ekf_outage.png
│   ├── python_attitude_ekf.py + attitude_ekf_results.png
│   ├── EKF_Attitude_GPS_Fusion_Task_Corrected.md
│   ├── EKF_GPS_Code_Explanation_Beginner_Guide.md
│   ├── cmds.txt
│   └── archive/
│       ├── gps_ekf_original.py + gps_ekf_trajectory.png
│       └── EKF_Attitude_GPS_Fusion_Task__1_.md
│
├── Guidance/
│   ├── README.md
│   ├── l1_guidance_prototype.py
│   ├── l1_path_tracking.png
│   ├── l1_lateral_acceleration.png
│   └── l1_eta.png
│
└── history/
    ├── 00_tutorials/
    │     AIrcraftPitch_SystemAnalysis.m, JSBSim_Running.m, PitchAnalysis_LQR.m, PitchAnalysis_DLQR.m
    ├── 01_trim_linearization_and_early_design/
    │     C172P_Altitude_Hold_COMPLETE_ENGINEERING_DOCUMENTATION_PI_AILERON_UPDATED.txt
    │     (covers trim/linearization, the single-point checkpoint design, and
    │     the roll-controller evolution through PI, all in one document)
    ├── 02_single_point_checkpoint/
    │     C172_DLQR_4state_FINAL_VERIFIED.m + .mat
    └── 04_autopilot_subsystem_refactor/
          autopilot_subsystem_refactor.m (superseded by the EKF-integrated
          version, which lives in controller/)
```

## Duplicates — keep the first, drop the second

- `validate_gain_schedule_v3.m` and `validate_gain_schedule_v32nd.m` — byte-identical.
- `controls_standards_deep_read_DO178C_SCADE_Polyspace_1.txt` — identical to the one in `docs/`.

## Not real — do not place anywhere

- `validate_gain_schedule.m` (no suffix) and `generate_altitude_stress_ic.py` — both retracted earlier in this process; not genuine project files.

## Status

All 17 requirements now verified against the final model — REQ-006/010 closed this pass. Only remaining work: finish sorting `.slx` files into the structure above (in progress, EKF done), then push.
