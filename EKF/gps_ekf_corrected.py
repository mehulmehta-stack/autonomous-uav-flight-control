import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# ============================================================
# Configuration
# ============================================================

DT = 0.1
SIM_TIME = 60.0
N = int(SIM_TIME / DT)

RNG = np.random.default_rng(7)

GPS_OUTAGE_START = 25.0
GPS_OUTAGE_END = 35.0

OUTPUT_DIR = Path(__file__).resolve().parent / "plots"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


# ============================================================
# 1. Create a synthetic "true" aircraft trajectory
# ============================================================

t = np.arange(N) * DT

true_north = 2.0 * t + 8.0 * np.sin(0.08 * t)
true_east = 1.5 * t + 5.0 * np.sin(0.11 * t)
true_alt = 100.0 + 0.8 * t + 4.0 * np.sin(0.12 * t)

# Derivatives of the true trajectory.
true_vn = np.gradient(true_north, DT)
true_ve = np.gradient(true_east, DT)
true_vz = np.gradient(true_alt, DT)

true_an = np.gradient(true_vn, DT)
true_ae = np.gradient(true_ve, DT)
true_az = np.gradient(true_vz, DT)


# ============================================================
# 2. Create synthetic IMU acceleration measurements
# ============================================================
#
# A real accelerometer is not perfect.
# We simulate:
#
# measured acceleration
#     = true acceleration
#       + constant sensor bias
#       + random measurement noise
#
# The bias is intentionally NOT estimated by this simple filter.
# Therefore, during a GPS outage, the position estimate will drift.
# This is the behavior we want to demonstrate.

imu_noise_std = 0.08  # m/s^2

imu_bias = np.array([
    0.04,   # north acceleration bias
    -0.03,  # east acceleration bias
    0.06    # vertical acceleration bias
])

true_acceleration = np.column_stack([
    true_an,
    true_ae,
    true_az
])

imu_acceleration = (
    true_acceleration
    + imu_bias
    + RNG.normal(0.0, imu_noise_std, (N, 3))
)


# ============================================================
# 3. Create synthetic GPS measurements
# ============================================================

gps_noise_std = np.array([
    3.0,  # north position noise [m]
    3.0,  # east position noise [m]
    5.0   # altitude noise [m]
])

true_position = np.column_stack([
    true_north,
    true_east,
    true_alt
])

gps = (
    true_position
    + RNG.normal(0.0, gps_noise_std, (N, 3))
)


# ============================================================
# 4. Kalman-filter state
# ============================================================
#
# State:
#
# x = [north, east, altitude, vn, ve, vz]
#
# This is a linear educational INS/GPS model.
# Because the model is linear, this particular example is
# technically a standard Kalman Filter (KF), not yet an EKF.
#
# The later 9-state INS/GPS estimator in the roadmap is nonlinear
# and is where the EKF becomes necessary.

x = np.array([
    true_north[0] + 5.0,
    true_east[0] - 4.0,
    true_alt[0] + 5.0,
    0.0,
    0.0,
    0.0
], dtype=float)

P = np.diag([
    10.0**2,  # north uncertainty
    10.0**2,  # east uncertainty
    10.0**2,  # altitude uncertainty
    2.0**2,   # north velocity uncertainty
    2.0**2,   # east velocity uncertainty
    2.0**2    # vertical velocity uncertainty
])


# ============================================================
# 5. Discrete motion model
# ============================================================

# State propagation:
#
# position_next = position + velocity*dt
#                 + 0.5*acceleration*dt^2
#
# velocity_next = velocity + acceleration*dt

F = np.array([
    [1, 0, 0, DT, 0, 0],
    [0, 1, 0, 0, DT, 0],
    [0, 0, 1, 0, 0, DT],
    [0, 0, 0, 1,  0,  0],
    [0, 0, 0, 0,  1,  0],
    [0, 0, 0, 0,  0,  1]
], dtype=float)

B = np.array([
    [0.5 * DT**2, 0, 0],
    [0, 0.5 * DT**2, 0],
    [0, 0, 0.5 * DT**2],
    [DT, 0, 0],
    [0, DT, 0],
    [0, 0, DT]
], dtype=float)


# GPS measures only position.
H = np.array([
    [1, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0]
], dtype=float)


# Process/model uncertainty.
Q = np.diag([
    0.005**2,
    0.005**2,
    0.010**2,
    0.05**2,
    0.05**2,
    0.05**2
])

# GPS measurement uncertainty.
R = np.diag(gps_noise_std**2)

I = np.eye(6)

estimate = np.zeros((N, 6))
covariance = np.zeros((N, 6))


# ============================================================
# 6. Filter loop
# ============================================================

for k in range(N):

    # --------------------------------------------------------
    # PREDICTION
    # --------------------------------------------------------
    #
    # IMU acceleration is used to propagate the state.
    #
    # This is the critical difference from the old version:
    # velocity is NOT directly replaced by a synthetic
    # velocity measurement.
    #
    # During GPS outage, acceleration is the only information
    # available to propagate position and velocity.

    x_pred = F @ x + B @ imu_acceleration[k]

    P_pred = F @ P @ F.T + Q


    # --------------------------------------------------------
    # GPS availability
    # --------------------------------------------------------

    gps_available = not (
        GPS_OUTAGE_START <= t[k] < GPS_OUTAGE_END
    )


    # --------------------------------------------------------
    # UPDATE only when GPS is available
    # --------------------------------------------------------

    if gps_available:

        z = gps[k]

        innovation = z - H @ x_pred

        S = H @ P_pred @ H.T + R

        # Equivalent to:
        # K = P_pred @ H.T @ inv(S)
        #
        # solve() is numerically preferable to explicitly
        # calculating a matrix inverse.

        K = np.linalg.solve(
            S,
            (P_pred @ H.T).T
        ).T

        x = x_pred + K @ innovation

        # Joseph-form covariance update for numerical robustness.
        P = (
            (I - K @ H) @ P_pred @ (I - K @ H).T
            + K @ R @ K.T
        )

    else:

        # ----------------------------------------------------
        # GPS OUTAGE
        # ----------------------------------------------------
        #
        # No GPS measurement exists.
        #
        # Therefore there is NO measurement correction.
        #
        # The filter simply coasts using the IMU-driven
        # prediction model.

        x = x_pred
        P = P_pred


    estimate[k] = x
    covariance[k] = np.diag(P)


# ============================================================
# 7. Position error
# ============================================================

position_error = np.linalg.norm(
    estimate[:, :3] - true_position,
    axis=1
)


# ============================================================
# 8. Plot
# ============================================================

fig, axes = plt.subplots(
    3, 1,
    figsize=(11, 10),
    sharex=True
)

axes[0].plot(
    t,
    true_north,
    label="True north",
    linewidth=2
)

axes[0].plot(
    t,
    estimate[:, 0],
    label="Estimated north",
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
    label="Estimated east",
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


# ============================================================
# 9. Validation
# ============================================================

pre_outage = t < GPS_OUTAGE_START
during_outage = (
    (t >= GPS_OUTAGE_START)
    & (t < GPS_OUTAGE_END)
)
post_outage = t >= GPS_OUTAGE_END

outage_start_idx = np.where(during_outage)[0][0]
outage_end_idx = np.where(during_outage)[0][-1]

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
    f"{position_error[outage_start_idx]:.3f} m"
)

print(
    "Position error at outage end: "
    f"{position_error[outage_end_idx]:.3f} m"
)

