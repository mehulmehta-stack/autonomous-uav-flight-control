import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# ------------------------------------------------------------
# Configuration
# ------------------------------------------------------------

DT = 0.01
SIM_TIME = 20.0
N = int(SIM_TIME / DT)

RNG = np.random.default_rng(42)

OUTPUT_DIR = Path(__file__).resolve().parent / "plots"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


# ------------------------------------------------------------
# True attitude
# ------------------------------------------------------------

t = np.arange(N) * DT

true_roll = np.deg2rad(
    10.0 * np.sin(2.0 * np.pi * 0.12 * t)
)

true_pitch = np.deg2rad(
    7.0 * np.sin(2.0 * np.pi * 0.08 * t + 0.5)
)

true_roll_rate = np.gradient(true_roll, DT)
true_pitch_rate = np.gradient(true_pitch, DT)


# ------------------------------------------------------------
# Synthetic gyro measurements
# ------------------------------------------------------------

gyro_noise_std = np.deg2rad(0.35)

gyro_roll = true_roll_rate + RNG.normal(
    0.0, gyro_noise_std, N
)

gyro_pitch = true_pitch_rate + RNG.normal(
    0.0, gyro_noise_std, N
)


# ------------------------------------------------------------
# Synthetic accelerometer measurements
# ------------------------------------------------------------

g = 9.81

ax_true = -g * np.sin(true_pitch)
ay_true = g * np.sin(true_roll) * np.cos(true_pitch)
az_true = g * np.cos(true_roll) * np.cos(true_pitch)

acc_noise_std = 0.12

ax = ax_true + RNG.normal(0.0, acc_noise_std, N)
ay = ay_true + RNG.normal(0.0, acc_noise_std, N)
az = az_true + RNG.normal(0.0, acc_noise_std, N)


# Convert accelerometer readings into attitude measurements
acc_roll = np.arctan2(ay, az)

acc_pitch = np.arctan2(
    -ax,
    np.sqrt(ay**2 + az**2)
)


# ------------------------------------------------------------
# EKF initialization
# ------------------------------------------------------------

x = np.array([0.0, 0.0])

P = np.diag([
    np.deg2rad(5.0) ** 2,
    np.deg2rad(5.0) ** 2
])

Q = np.diag([
    np.deg2rad(0.08) ** 2,
    np.deg2rad(0.08) ** 2
])

R = np.diag([
    np.deg2rad(1.2) ** 2,
    np.deg2rad(1.2) ** 2
])

F = np.eye(2)
H = np.eye(2)

I = np.eye(2)

estimate = np.zeros((N, 2))


# ------------------------------------------------------------
# EKF loop
# ------------------------------------------------------------

for k in range(N):

    # -------------------------
    # 1. Prediction
    # -------------------------

    u = np.array([
        gyro_roll[k],
        gyro_pitch[k]
    ])

    x_pred = x + u * DT

    P_pred = F @ P @ F.T + Q

    # -------------------------
    # 2. Measurement
    # -------------------------

    z = np.array([
        acc_roll[k],
        acc_pitch[k]
    ])

    innovation = z - H @ x_pred

    S = H @ P_pred @ H.T + R

    K = P_pred @ H.T @ np.linalg.inv(S)

    # -------------------------
    # 3. Correction
    # -------------------------

    x = x_pred + K @ innovation

    P = (I - K @ H) @ P_pred

    estimate[k] = x


# ------------------------------------------------------------
# Plot
# ------------------------------------------------------------

fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)

axes[0].plot(
    t,
    np.rad2deg(true_roll),
    label="True roll",
    linewidth=2
)

axes[0].plot(
    t,
    np.rad2deg(acc_roll),
    label="Accelerometer roll",
    alpha=0.45
)

axes[0].plot(
    t,
    np.rad2deg(estimate[:, 0]),
    label="EKF roll",
    linewidth=2
)

axes[0].set_ylabel("Roll (deg)")
axes[0].set_title("Attitude EKF — Roll")
axes[0].grid(True)
axes[0].legend()


axes[1].plot(
    t,
    np.rad2deg(true_pitch),
    label="True pitch",
    linewidth=2
)

axes[1].plot(
    t,
    np.rad2deg(acc_pitch),
    label="Accelerometer pitch",
    alpha=0.45
)

axes[1].plot(
    t,
    np.rad2deg(estimate[:, 1]),
    label="EKF pitch",
    linewidth=2
)

axes[1].set_xlabel("Time (s)")
axes[1].set_ylabel("Pitch (deg)")
axes[1].set_title("Attitude EKF — Pitch")
axes[1].grid(True)
axes[1].legend()

plt.tight_layout()

output = OUTPUT_DIR / "attitude_ekf_results.png"
plt.savefig(output, dpi=200)
plt.show()

print(f"Saved: {output}")