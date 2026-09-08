# ============================================================
# ARCHIVED — original pre-correction version.
#
# This is the "old version" referenced in gps_ekf_corrected.py's
# own comments. The bug: this script overwrites the predicted
# velocity state every step with a synthetic, idealized velocity
# "measurement" (vn_meas/ve_meas/vz_meas), instead of propagating
# velocity from integrated IMU acceleration:
#
#     x_pred[3] = vn_meas[k]
#     x_pred[4] = ve_meas[k]
#     x_pred[5] = vz_meas[k]
#
# That synthetic velocity measurement keeps refreshing regardless
# of GPS availability, which quietly defeats the point of the GPS-
# outage test: velocity stays accurate through the outage because
# it's coming from a sensor that was never actually turned off,
# not because the filter is coasting well on inertial data alone.
#
# gps_ekf_corrected.py fixes this by deriving velocity purely from
# integrated IMU acceleration (F/B matrices + imu_acceleration),
# so the outage genuinely tests inertial-only coasting.
#
# Kept here, not deleted, as the "before" half of that fix — see
# JOURNEY.md. Produces gps_ekf_trajectory.png, which is why that
# file existed with no corresponding script in the current folder.
# ============================================================

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# ------------------------------------------------------------
# Configuration
# ------------------------------------------------------------

DT = 0.1
SIM_TIME = 60.0
N = int(SIM_TIME / DT)

RNG = np.random.default_rng(7)

GPS_OUTAGE_START = 25.0
GPS_OUTAGE_END = 35.0

OUTPUT_DIR = Path(__file__).resolve().parent / "plots"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


# ------------------------------------------------------------
# True trajectory
# ------------------------------------------------------------

t = np.arange(N) * DT

true_north = 2.0 * t + 8.0 * np.sin(0.08 * t)

true_east = 1.5 * t + 5.0 * np.sin(0.11 * t)

true_alt = (
    100.0
    + 0.8 * t
    + 4.0 * np.sin(0.12 * t)
)

true_vn = np.gradient(true_north, DT)
true_ve = np.gradient(true_east, DT)
true_vz = np.gradient(true_alt, DT)


# ------------------------------------------------------------
# Synthetic inertial velocity measurements
# ------------------------------------------------------------

imu_velocity_noise_std = 0.20

vn_meas = true_vn + RNG.normal(
    0.0, imu_velocity_noise_std, N
)

ve_meas = true_ve + RNG.normal(
    0.0, imu_velocity_noise_std, N
)

vz_meas = true_vz + RNG.normal(
    0.0, imu_velocity_noise_std, N
)


# ------------------------------------------------------------
# Synthetic GPS measurements
# ------------------------------------------------------------

gps_position_noise_std = np.array([
    3.0,   # north [m]
    3.0,   # east [m]
    5.0    # altitude [m]
])

gps = np.column_stack([
    true_north,
    true_east,
    true_alt
])

gps += RNG.normal(
    0.0,
    gps_position_noise_std,
    gps.shape
)


# ------------------------------------------------------------
# EKF model
# ------------------------------------------------------------

# State:
#
# x = [north, east, altitude, vn, ve, vz]

x = np.array([
    true_north[0],
    true_east[0],
    true_alt[0] + 5.0,
    0.0,
    0.0,
    0.0
])

P = np.diag([
    10.0**2,
    10.0**2,
    10.0**2,
    1.0**2,
    1.0**2,
    1.0**2
])


F = np.array([
    [1, 0, 0, DT, 0, 0],
    [0, 1, 0, 0, DT, 0],
    [0, 0, 1, 0, 0, DT],
    [0, 0, 0, 1,  0,  0],
    [0, 0, 0, 0,  1,  0],
    [0, 0, 0, 0,  0,  1]
], dtype=float)


H = np.array([
    [1, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0]
], dtype=float)


# Process noise.
# The model is deliberately simple: uncertainty is added
# to account for imperfect velocity/inertial propagation.

Q = np.diag([
    0.02**2,
    0.02**2,
    0.05**2,
    0.08**2,
    0.08**2,
    0.08**2
])


R = np.diag(gps_position_noise_std**2)

I = np.eye(6)

estimate = np.zeros((N, 6))
covariance = np.zeros((N, 6))


# ------------------------------------------------------------
# EKF loop
# ------------------------------------------------------------

for k in range(N):

    # -------------------------
    # 1. Prediction
    # -------------------------

    x_pred = F @ x

    # Velocity measurements act as a simple inertial input.
    #
    # In a full INS this would instead come from acceleration
    # measurements transformed through the navigation equations.

    x_pred[3] = vn_meas[k]
    x_pred[4] = ve_meas[k]
    x_pred[5] = vz_meas[k]

    P_pred = F @ P @ F.T + Q

    # -------------------------
    # 2. GPS availability
    # -------------------------

    gps_available = not (
        GPS_OUTAGE_START <= t[k] < GPS_OUTAGE_END
    )

    if gps_available:

        z = gps[k]

        innovation = z - H @ x_pred

        S = H @ P_pred @ H.T + R

        K = P_pred @ H.T @ np.linalg.inv(S)

        # ---------------------
        # 3. Measurement update
        # ---------------------

        x = x_pred + K @ innovation

        P = (I - K @ H) @ P_pred

    else:

        # No GPS measurement:
        #
        # The EKF cannot perform the measurement correction.
        # It must coast using the process model.

        x = x_pred
        P = P_pred

    estimate[k] = x
    covariance[k] = np.diag(P)


# ------------------------------------------------------------
# Convert local position to latitude/longitude
# ------------------------------------------------------------

earth_radius = 6378137.0

reference_lat_deg = 22.5726
reference_lon_deg = 88.3639

reference_lat_rad = np.deg2rad(reference_lat_deg)

estimated_lat = (
    reference_lat_deg
    + np.rad2deg(
        estimate[:, 0] / earth_radius
    )
)

estimated_lon = (
    reference_lon_deg
    + np.rad2deg(
        estimate[:, 1]
        / (earth_radius * np.cos(reference_lat_rad))
    )
)

true_lat = (
    reference_lat_deg
    + np.rad2deg(
        true_north / earth_radius
    )
)

true_lon = (
    reference_lon_deg
    + np.rad2deg(
        true_east
        / (earth_radius * np.cos(reference_lat_rad))
    )
)


# ------------------------------------------------------------
# Position error
# ------------------------------------------------------------

position_error = np.sqrt(
    (estimate[:, 0] - true_north)**2
    + (estimate[:, 1] - true_east)**2
    + (estimate[:, 2] - true_alt)**2
)


# ------------------------------------------------------------
# Plot 1 — GPS outage
# ------------------------------------------------------------

fig, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True)

axes[0].plot(
    t,
    true_north,
    label="True north",
    linewidth=2
)

axes[0].plot(
    t,
    estimate[:, 0],
    label="EKF north",
    linewidth=2
)

axes[0].set_ylabel("North (m)")
axes[0].grid(True)
axes[0].legend()


axes[1].plot(
    t,
    true_east,
    label="True east",
    linewidth=2
)

axes[1].plot(
    t,
    estimate[:, 1],
    label="EKF east",
    linewidth=2
)

axes[1].set_ylabel("East (m)")
axes[1].grid(True)
axes[1].legend()


axes[2].plot(
    t,
    position_error,
    label="3D position error",
    linewidth=2
)

axes[2].axvspan(
    GPS_OUTAGE_START,
    GPS_OUTAGE_END,
    alpha=0.2,
    label="GPS outage"
)

axes[2].set_xlabel("Time (s)")
axes[2].set_ylabel("Position error (m)")
axes[2].grid(True)
axes[2].legend()

plt.tight_layout()

output = OUTPUT_DIR / "gps_ekf_outage.png"
plt.savefig(output, dpi=200)
plt.show()

print(f"Saved: {output}")


# ------------------------------------------------------------
# Plot 2 — Horizontal trajectory
# ------------------------------------------------------------

plt.figure(figsize=(9, 7))

plt.plot(
    true_east,
    true_north,
    label="True trajectory",
    linewidth=2
)

plt.plot(
    estimate[:, 1],
    estimate[:, 0],
    label="EKF trajectory",
    linewidth=2
)

outage_mask = (
    (t >= GPS_OUTAGE_START)
    & (t < GPS_OUTAGE_END)
)

plt.plot(
    estimate[outage_mask, 1],
    estimate[outage_mask, 0],
    linewidth=4,
    label="EKF during GPS outage"
)

plt.xlabel("East (m)")
plt.ylabel("North (m)")
plt.title("GPS/INS EKF Horizontal Trajectory")
plt.grid(True)
plt.legend()

plt.tight_layout()

output = OUTPUT_DIR / "gps_ekf_trajectory.png"
plt.savefig(output, dpi=200)
plt.show()

print(f"Saved: {output}")


# ------------------------------------------------------------
# Basic validation
# ------------------------------------------------------------

pre_outage = t < GPS_OUTAGE_START
during_outage = outage_mask
post_outage = t >= GPS_OUTAGE_END

print("\nValidation")
print("----------")

print(
    "Mean position error before outage: "
    f"{np.mean(position_error[pre_outage]):.3f} m"
)

print(
    "Mean position error during outage: "
    f"{np.mean(position_error[during_outage]):.3f} m"
)

print(
    "Mean position error after outage: "
    f"{np.mean(position_error[post_outage]):.3f} m"
)

print(
    "Position error at outage start: "
    f"{position_error[np.where(during_outage)[0][0]]:.3f} m"
)

print(
    "Position error at outage end: "
    f"{position_error[np.where(during_outage)[0][-1]]:.3f} m"
)
