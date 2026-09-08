# EKF — Progression: 2-state → 6-state → 9-state

Three estimators live in this folder, at increasing fidelity. Keeping all
three, tagged clearly, rather than only the final one, is intentional — it's
part of the project's development journey (see `JOURNEY.md` at repo root).

| Stage | File | What it estimates | Status |
|---|---|---|---|
| 1 | `python_attitude_ekf.py` | Roll, pitch only (2-state EKF) | Complete |
| 2a | `archive/gps_ekf_original.py` | Position + velocity (6-state linear KF) | **Superseded** — velocity was overwritten by a synthetic "measurement" every step regardless of GPS availability, which quietly defeated the GPS-outage test. See the header note in the file. |
| 2b | `gps_ekf_corrected.py` | Position + velocity (6-state linear KF — not yet an EKF, see its own header comment) | Complete — velocity now derived from integrated IMU acceleration, so the outage test is genuine |
| 3 | `nav_ekf.py` | Full 9-state nonlinear EKF: position, velocity, attitude | Complete — write-up: `EKF_9State_Nav_Writeup.md` |

## Documentation map

- `EKF_GPS_Code_Explanation_Beginner_Guide.md` — written during stage 2,
  covers the 6-state linear KF in detail. Updated with a pointer note where
  it originally described the 9-state EKF as a future step.
- `EKF_Attitude_GPS_Fusion_Task_Corrected.md` — task writeup for stages 1–2.
- `EKF_9State_Nav_Writeup.md` — new, covers stage 3 (`nav_ekf.py`): state
  vector, measurement model, the yaw-observability limitation, and the
  GPS-outage test results.

## Resolved

`gps_ekf_trajectory.png` is produced by `archive/gps_ekf_original.py` —
the pre-correction version of the 6-state filter, recovered from an early
draft of the task document. No longer an orphan.

## Run instructions

See `cmds.txt` for the conda environment setup and run commands.
