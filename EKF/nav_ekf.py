
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# ============================================================
# 9-State INS/GPS Extended Kalman Filter
# ============================================================
#
# State ordering:
# x = [N, E, D, VN, VE, VD, phi, theta, psi]
#
# N,E,D     : NED position [m]
# VN,VE,VD  : NED velocity [m/s]
# phi       : roll [rad]
# theta     : pitch [rad]
# psi       : yaw [rad]
#
# IMU input:
# u = [ax, ay, az, p, q, r]
#
# ax,ay,az  : specific force measured in body frame [m/s^2]
# p,q,r     : body angular rates [rad/s]
#
# GPS measurement:
# z = [N, E, D] [m]
#
# This is an educational 9-state nonlinear INS/GPS EKF.
# It intentionally does NOT estimate accelerometer/gyro biases.
# ============================================================


# ------------------------------------------------------------
# Configuration
# ------------------------------------------------------------

DT = 0.01                  # 100 Hz IMU
GPS_DT = 0.10              # 10 Hz GPS
SIM_TIME = 60.0
N = int(SIM_TIME / DT)

GPS_EVERY = int(round(GPS_DT / DT))

# Deliberate GPS outage for validation.
# This is an additional stress test; it is not required for
# the mathematical EKF itself.
GPS_OUTAGE_START = 25.0
GPS_OUTAGE_END = 35.0

GRAVITY = 9.80665

RNG = np.random.default_rng(7)

OUTPUT_DIR = Path(__file__).resolve().parent / "nav_ekf_plots"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


# ------------------------------------------------------------
# Rotation: body -> NED
# ------------------------------------------------------------

def rotation_body_to_ned(phi, theta, psi):
    """
    Return the 3x3 direction-cosine matrix C_bn.

    Body axes:
        x = forward
        y = right
        z = down

    Navigation frame:
        N = north
        E = east
        D = down
    """
    cp = np.cos(phi)
    sp = np.sin(phi)
    ct = np.cos(theta)
    st = np.sin(theta)
    cy = np.cos(psi)
    sy = np.sin(psi)

    return np.array([
        [ct * cy,
         sp * st * cy - cp * sy,
         cp * st * cy + sp * sy],

        [ct * sy,
         sp * st * sy + cp * cy,
         cp * st * sy - sp * cy],

        [-st,
         sp * ct,
         cp * ct]
    ])


# ------------------------------------------------------------
# Euler-angle kinematics
# ------------------------------------------------------------

def euler_rates(phi, theta, p, q, r):
    """
    Convert body rates [p,q,r] into Euler angle rates.

    phi_dot   = p + sin(phi) tan(theta) q
                  + cos(phi) tan(theta) r

    theta_dot = cos(phi) q - sin(phi) r

    psi_dot   = sin(phi)/cos(theta) q
                  + cos(phi)/cos(theta) r
    """
    cp = np.cos(phi)
    sp = np.sin(phi)
    ct = np.cos(theta)
    tt = np.tan(theta)

    phi_dot = p + sp * tt * q + cp * tt * r
    theta_dot = cp * q - sp * r
    psi_dot = (sp * q + cp * r) / ct

    return np.array([phi_dot, theta_dot, psi_dot])


def body_rates_from_euler_rates(phi, theta, phi_dot, theta_dot, psi_dot):
    """
    Invert the Euler-rate relationship to generate true gyro
    measurements for the synthetic trajectory.
    """
    cp = np.cos(phi)
    sp = np.sin(phi)
    ct = np.cos(theta)

    p = phi_dot - np.sin(theta) * psi_dot
    q = cp * theta_dot + sp * ct * psi_dot
    r = -sp * theta_dot + cp * ct * psi_dot

    return np.array([p, q, r])


# ------------------------------------------------------------
# Nonlinear process model f(x,u)
# ------------------------------------------------------------

def state_derivative(x, u):
    """
    Continuous-time nonlinear INS process model.

    x = [N,E,D,VN,VE,VD,phi,theta,psi]
    u = [ax,ay,az,p,q,r]

    Accelerometer input is specific force in the body frame.
    Gravity is added in the NED frame.
    """
    N_pos, E_pos, D_pos = x[0:3]
    VN, VE, VD = x[3:6]
    phi, theta, psi = x[6:9]

    ax, ay, az = u[0:3]
    p, q, r = u[3:6]

    C_bn = rotation_body_to_ned(phi, theta, psi)

    specific_force_body = np.array([ax, ay, az])

    gravity_ned = np.array([0.0, 0.0, GRAVITY])

    acceleration_ned = (
        C_bn @ specific_force_body
        + gravity_ned
    )

    phi_dot, theta_dot, psi_dot = euler_rates(
        phi, theta, p, q, r
    )

    return np.array([
        VN,
        VE,
        VD,
        acceleration_ned[0],
        acceleration_ned[1],
        acceleration_ned[2],
        phi_dot,
        theta_dot,
        psi_dot
    ])


# ------------------------------------------------------------
# Analytic continuous-time Jacobian Fc = df/dx
# ------------------------------------------------------------

def jacobian_Fc(x, u):
    """
    Analytic 9x9 continuous-time process Jacobian.

    Fc[i,j] = partial(f_i) / partial(x_j)
    """
    phi, theta, psi = x[6:9]
    ax, ay, az = u[0:3]
    p, q, r = u[3:6]

    cp = np.cos(phi)
    sp = np.sin(phi)
    ct = np.cos(theta)
    st = np.sin(theta)
    cy = np.cos(psi)
    sy = np.sin(psi)

    tt = np.tan(theta)
    sec2 = 1.0 / (ct * ct)

    Fc = np.zeros((9, 9))

    # Position derivatives:
    # N_dot = VN, E_dot = VE, D_dot = VD
    Fc[0, 3] = 1.0
    Fc[1, 4] = 1.0
    Fc[2, 5] = 1.0

    # --------------------------------------------------------
    # Velocity derivatives
    # --------------------------------------------------------

    # a_N derivatives
    daN_dphi = (
        (cp * st * cy + sp * sy) * ay
        + (-sp * st * cy + cp * sy) * az
    )

    daN_dtheta = (
        -st * cy * ax
        + sp * ct * cy * ay
        + cp * ct * cy * az
    )

    daN_dpsi = (
        -ct * sy * ax
        + (-sp * st * sy - cp * cy) * ay
        + (-cp * st * sy + sp * cy) * az
    )

    # a_E derivatives
    daE_dphi = (
        (cp * st * sy - sp * cy) * ay
        + (-sp * st * sy - cp * cy) * az
    )

    daE_dtheta = (
        -st * sy * ax
        + sp * ct * sy * ay
        + cp * ct * sy * az
    )

    daE_dpsi = (
        ct * cy * ax
        + (sp * st * cy - cp * sy) * ay
        + (cp * st * cy + sp * sy) * az
    )

    # a_D derivatives
    daD_dphi = (
        cp * ct * ay
        - sp * ct * az
    )

    daD_dtheta = (
        -ct * ax
        - sp * st * ay
        - cp * st * az
    )

    Fc[3, 6] = daN_dphi
    Fc[3, 7] = daN_dtheta
    Fc[3, 8] = daN_dpsi

    Fc[4, 6] = daE_dphi
    Fc[4, 7] = daE_dtheta
    Fc[4, 8] = daE_dpsi

    Fc[5, 6] = daD_dphi
    Fc[5, 7] = daD_dtheta
    Fc[5, 8] = 0.0

    # --------------------------------------------------------
    # Attitude derivatives
    # --------------------------------------------------------

    # phi_dot = p + sin(phi) tan(theta) q
    #              + cos(phi) tan(theta) r
    Fc[6, 6] = (
        cp * tt * q
        - sp * tt * r
    )

    Fc[6, 7] = (
        sp * q + cp * r
    ) * sec2

    Fc[6, 8] = 0.0

    # theta_dot = cos(phi) q - sin(phi) r
    Fc[7, 6] = -sp * q - cp * r
    Fc[7, 7] = 0.0
    Fc[7, 8] = 0.0

    # psi_dot = (sin(phi) q + cos(phi) r) / cos(theta)
    Fc[8, 6] = (
        cp * q - sp * r
    ) / ct

    Fc[8, 7] = (
        sp * q + cp * r
    ) * sec2 / (1.0 / ct)  # equivalent to (...) sec(theta) tan(theta)

    # The previous expression simplifies to:
    Fc[8, 7] = (
        sp * q + cp * r
    ) / ct * np.tan(theta)

    Fc[8, 8] = 0.0

    return Fc


def jacobian_Fd(x, u, dt):
    """
    First-order discrete transition matrix:
        Fd = I + Fc * dt
    """
    return np.eye(9) + jacobian_Fc(x, u) * dt


# ------------------------------------------------------------
# Numerical Jacobian for verification
# ------------------------------------------------------------

def numerical_jacobian(x, u, epsilon=1e-6):
    """
    Central finite-difference approximation of df/dx.

    This is used only to validate the hand-derived analytic
    Jacobian; it is NOT used by the EKF during normal operation.
    """
    J = np.zeros((9, 9))

    for j in range(9):
        dx = np.zeros(9)
        dx[j] = epsilon

        fp = state_derivative(x + dx, u)
        fm = state_derivative(x - dx, u)

        J[:, j] = (fp - fm) / (2.0 * epsilon)

    return J


# ------------------------------------------------------------
# Truth trajectory
# ------------------------------------------------------------

t = np.arange(N) * DT

true_pos = np.zeros((N, 3))
true_vel = np.zeros((N, 3))
true_att = np.zeros((N, 3))

# Smooth fixed-wing-like motion in NED.
true_vel[:, 0] = 15.0 + 1.5 * np.sin(0.10 * t)
true_vel[:, 1] = 3.0 * np.sin(0.07 * t)
true_vel[:, 2] = 0.8 * np.sin(0.09 * t)

# Integrate truth position.
for k in range(1, N):
    true_pos[k] = (
        true_pos[k - 1]
        + true_vel[k - 1] * DT
    )

# Smooth attitude motion.
true_att[:, 0] = np.deg2rad(5.0) * np.sin(0.12 * t)          # roll
true_att[:, 1] = np.deg2rad(3.0) * np.sin(0.08 * t + 0.5)    # pitch
true_att[:, 2] = np.deg2rad(12.0) * np.sin(0.05 * t)        # yaw


# ------------------------------------------------------------
# Truth acceleration and IMU measurements
# ------------------------------------------------------------

true_acc_ned = np.gradient(
    true_vel,
    DT,
    axis=0
)

true_specific_force_body = np.zeros((N, 3))
true_gyro = np.zeros((N, 3))

for k in range(N):
    phi, theta, psi = true_att[k]

    C_bn = rotation_body_to_ned(phi, theta, psi)

    # a_ned = C_bn * f_body + g_ned
    # therefore:
    # f_body = C_bn^T * (a_ned - g_ned)
    true_specific_force_body[k] = (
        C_bn.T
        @ (
            true_acc_ned[k]
            - np.array([0.0, 0.0, GRAVITY])
        )
    )

    if k == 0:
        att_dot = np.gradient(
            true_att,
            DT,
            axis=0
        )[k]
    else:
        att_dot = np.gradient(
            true_att,
            DT,
            axis=0
        )[k]

    true_gyro[k] = body_rates_from_euler_rates(
        phi,
        theta,
        att_dot[0],
        att_dot[1],
        att_dot[2]
    )

# IMU noise levels.
# Bias is intentionally omitted from this 9-state filter.
ACCEL_NOISE_STD = 0.08       # m/s^2
GYRO_NOISE_STD = np.deg2rad(0.08)  # rad/s

measured_accel = (
    true_specific_force_body
    + RNG.normal(0.0, ACCEL_NOISE_STD, (N, 3))
)

measured_gyro = (
    true_gyro
    + RNG.normal(0.0, GYRO_NOISE_STD, (N, 3))
)

imu = np.column_stack([
    measured_accel,
    measured_gyro
])


# ------------------------------------------------------------
# Synthetic GPS
# ------------------------------------------------------------

GPS_POSITION_STD = np.array([
    2.0,   # north [m]
    2.0,   # east [m]
    3.0    # down [m]
])

gps = (
    true_pos
    + RNG.normal(
        0.0,
        GPS_POSITION_STD,
        (N, 3)
    )
)

H = np.zeros((3, 9))
H[0, 0] = 1.0
H[1, 1] = 1.0
H[2, 2] = 1.0

R = np.diag(GPS_POSITION_STD ** 2)


# ------------------------------------------------------------
# Initial EKF state and covariance
# ------------------------------------------------------------

x = np.array([
    true_pos[0, 0] + 2.0,
    true_pos[0, 1] - 2.0,
    true_pos[0, 2] + 2.0,
    true_vel[0, 0] - 1.0,
    true_vel[0, 1] + 0.5,
    true_vel[0, 2] - 0.5,
    true_att[0, 0] + np.deg2rad(2.0),
    true_att[0, 1] - np.deg2rad(2.0),
    true_att[0, 2] + np.deg2rad(3.0),
])

P = np.diag([
    5.0**2,
    5.0**2,
    5.0**2,
    2.0**2,
    2.0**2,
    2.0**2,
    np.deg2rad(5.0)**2,
    np.deg2rad(5.0)**2,
    np.deg2rad(8.0)**2,
])

# Process covariance.
# This is intentionally conservative because the current
# 9-state model does not estimate IMU biases.
Q = np.diag([
    0.001**2,
    0.001**2,
    0.001**2,
    0.04**2,
    0.04**2,
    0.04**2,
    np.deg2rad(0.03)**2,
    np.deg2rad(0.03)**2,
    np.deg2rad(0.03)**2,
])


# ------------------------------------------------------------
# Storage
# ------------------------------------------------------------

estimate = np.zeros((N, 9))
covariance_diag = np.zeros((N, 9))
gps_used = np.zeros(N, dtype=bool)
innovation_log = np.full((N, 3), np.nan)


# ------------------------------------------------------------
# EKF loop
# ------------------------------------------------------------

I9 = np.eye(9)

for k in range(N):

    u = imu[k]

    # ========================================================
    # 1. PREDICT
    # ========================================================

    dx = state_derivative(x, u)

    x_pred = x + dx * DT

    # Keep Euler angles numerically bounded.
    x_pred[6:9] = (
        x_pred[6:9] + np.pi
    ) % (2.0 * np.pi) - np.pi

    F = jacobian_Fd(x, u, DT)

    P_pred = (
        F @ P @ F.T
        + Q
    )

    # Force symmetry against floating-point asymmetry.
    P_pred = 0.5 * (P_pred + P_pred.T)

    # ========================================================
    # 2. GPS UPDATE — only at GPS rate and outside outage
    # ========================================================

    gps_available = (
        k % GPS_EVERY == 0
        and not (
            GPS_OUTAGE_START
            <= t[k]
            < GPS_OUTAGE_END
        )
    )

    if gps_available:

        z = gps[k]

        # h(x) = [N,E,D]
        z_pred = H @ x_pred

        innovation = z - z_pred

        S = (
            H @ P_pred @ H.T
            + R
        )

        # Avoid explicit matrix inverse.
        K = (
            np.linalg.solve(
                S,
                H @ P_pred
            )
        ).T

        x = (
            x_pred
            + K @ innovation
        )

        # Joseph covariance update:
        # numerically safer than (I-KH)P alone.
        A = I9 - K @ H

        P = (
            A @ P_pred @ A.T
            + K @ R @ K.T
        )

        P = 0.5 * (P + P.T)

        gps_used[k] = True
        innovation_log[k] = innovation

    else:

        # No GPS correction.
        x = x_pred
        P = P_pred

    # Keep attitude wrapped.
    x[6:9] = (
        x[6:9] + np.pi
    ) % (2.0 * np.pi) - np.pi

    estimate[k] = x
    covariance_diag[k] = np.diag(P)


# ------------------------------------------------------------
# Validation 1 — analytic Jacobian vs finite difference
# ------------------------------------------------------------

test_k = N // 2

Fc_analytic = jacobian_Fc(
    estimate[test_k],
    imu[test_k]
)

Fc_numeric = numerical_jacobian(
    estimate[test_k],
    imu[test_k]
)

jacobian_abs_error = np.abs(
    Fc_analytic - Fc_numeric
)

jacobian_max_error = np.max(
    jacobian_abs_error
)

print("\n============================================================")
print("9-STATE INS/GPS EKF VALIDATION")
print("============================================================")
print(f"Analytic-vs-numerical Jacobian max error: "
      f"{jacobian_max_error:.3e}")


# ------------------------------------------------------------
# Validation 2 — state errors
# ------------------------------------------------------------

position_error = np.linalg.norm(
    estimate[:, 0:3] - true_pos,
    axis=1
)

velocity_error = np.linalg.norm(
    estimate[:, 3:6] - true_vel,
    axis=1
)

# Wrapped attitude error.
attitude_error = (
    estimate[:, 6:9]
    - true_att
)

attitude_error = (
    attitude_error + np.pi
) % (2.0 * np.pi) - np.pi

attitude_error_deg = np.rad2deg(
    np.linalg.norm(
        attitude_error,
        axis=1
    )
)

# Position 1-sigma uncertainty from covariance.
position_sigma = np.sqrt(
    np.maximum(
        covariance_diag[:, 0:3],
        0.0
    )
)

outage_mask = (
    (t >= GPS_OUTAGE_START)
    & (t < GPS_OUTAGE_END)
)

pre_outage = t < GPS_OUTAGE_START
post_outage = t >= GPS_OUTAGE_END

print(f"Mean position error before outage: "
      f"{np.mean(position_error[pre_outage]):.3f} m")

print(f"Mean position error during outage: "
      f"{np.mean(position_error[outage_mask]):.3f} m")

print(f"Mean position error after outage: "
      f"{np.mean(position_error[post_outage]):.3f} m")

outage_indices = np.where(outage_mask)[0]

print(f"Position error at outage start: "
      f"{position_error[outage_indices[0]]:.3f} m")

print(f"Position error at outage end: "
      f"{position_error[outage_indices[-1]]:.3f} m")

print(f"Maximum position error: "
      f"{np.max(position_error):.3f} m")

print(f"Mean attitude error: "
      f"{np.mean(attitude_error_deg):.3f} deg")

print(f"GPS updates used: "
      f"{np.count_nonzero(gps_used)}")

print("============================================================\n")


# ------------------------------------------------------------
# Plot 1 — Position
# ------------------------------------------------------------

fig, axes = plt.subplots(
    3, 1,
    figsize=(11, 9),
    sharex=True
)

labels = [
    "North (m)",
    "East (m)",
    "Down (m)"
]

for i in range(3):

    axes[i].plot(
        t,
        true_pos[:, i],
        label="Truth",
        linewidth=2
    )

    axes[i].plot(
        t,
        estimate[:, i],
        label="9-state EKF",
        linewidth=1.5
    )

    axes[i].set_ylabel(labels[i])
    axes[i].grid(True)
    axes[i].legend()

axes[2].set_xlabel("Time (s)")

fig.suptitle(
    "9-State INS/GPS EKF — Position"
)

plt.tight_layout()

output = OUTPUT_DIR / "position_estimate.png"
plt.savefig(output, dpi=200)
plt.show()


# ------------------------------------------------------------
# Plot 2 — Attitude
# ------------------------------------------------------------

fig, axes = plt.subplots(
    3, 1,
    figsize=(11, 9),
    sharex=True
)

att_labels = [
    "Roll (deg)",
    "Pitch (deg)",
    "Yaw (deg)"
]

true_att_deg = np.rad2deg(true_att)
estimate_att_deg = np.rad2deg(estimate[:, 6:9])

for i in range(3):

    axes[i].plot(
        t,
        true_att_deg[:, i],
        label="Truth",
        linewidth=2
    )

    axes[i].plot(
        t,
        estimate_att_deg[:, i],
        label="9-state EKF",
        linewidth=1.5
    )

    axes[i].set_ylabel(att_labels[i])
    axes[i].grid(True)
    axes[i].legend()

axes[2].set_xlabel("Time (s)")

fig.suptitle(
    "9-State INS/GPS EKF — Attitude"
)

plt.tight_layout()

output = OUTPUT_DIR / "attitude_estimate.png"
plt.savefig(output, dpi=200)
plt.show()


# ------------------------------------------------------------
# Plot 3 — Position error + GPS outage
# ------------------------------------------------------------

plt.figure(figsize=(11, 5))

plt.plot(
    t,
    position_error,
    label="3D position error",
    linewidth=2
)

plt.axvspan(
    GPS_OUTAGE_START,
    GPS_OUTAGE_END,
    alpha=0.2,
    label="GPS outage"
)

plt.xlabel("Time (s)")
plt.ylabel("Position error (m)")
plt.title(
    "9-State INS/GPS EKF — GPS Outage Robustness"
)
plt.grid(True)
plt.legend()
plt.tight_layout()

output = OUTPUT_DIR / "gps_outage_error.png"
plt.savefig(output, dpi=200)
plt.show()


# ------------------------------------------------------------
# Plot 4 — Position covariance / uncertainty
# ------------------------------------------------------------

plt.figure(figsize=(11, 6))

plt.plot(
    t,
    position_sigma[:, 0],
    label="1σ North"
)

plt.plot(
    t,
    position_sigma[:, 1],
    label="1σ East"
)

plt.plot(
    t,
    position_sigma[:, 2],
    label="1σ Down"
)

plt.axvspan(
    GPS_OUTAGE_START,
    GPS_OUTAGE_END,
    alpha=0.2,
    label="GPS outage"
)

plt.xlabel("Time (s)")
plt.ylabel("Position uncertainty 1σ (m)")
plt.title(
    "9-State INS/GPS EKF — Position Covariance"
)
plt.grid(True)
plt.legend()
plt.tight_layout()

output = OUTPUT_DIR / "position_covariance.png"
plt.savefig(output, dpi=200)
plt.show()


# ------------------------------------------------------------
# Plot 5 — Horizontal trajectory
# ------------------------------------------------------------

plt.figure(figsize=(8, 7))

plt.plot(
    true_pos[:, 1],
    true_pos[:, 0],
    label="Truth",
    linewidth=2
)

plt.plot(
    estimate[:, 1],
    estimate[:, 0],
    label="9-state EKF",
    linewidth=1.5
)

plt.plot(
    estimate[outage_mask, 1],
    estimate[outage_mask, 0],
    linewidth=4,
    label="EKF during GPS outage"
)

plt.xlabel("East (m)")
plt.ylabel("North (m)")
plt.title(
    "9-State INS/GPS EKF — Horizontal Trajectory"
)
plt.grid(True)
plt.legend()
plt.tight_layout()

output = OUTPUT_DIR / "horizontal_trajectory.png"
plt.savefig(output, dpi=200)
plt.show()


print(f"Plots saved to: {OUTPUT_DIR}")
