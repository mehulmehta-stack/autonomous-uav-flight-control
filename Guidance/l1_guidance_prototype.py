import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


# Folder where all Day-2 plots will be saved
OUTPUT_DIR = Path(__file__).resolve().parent / "l1_guidance_plots"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


def l1_guidance(pos, vel, wp_start, wp_end, L1_dist):
    """
    Standalone 2-D L1-style guidance law.

    Parameters
    ----------
    pos : ndarray
        Vehicle position [x, y] in metres.
    vel : ndarray
        Vehicle velocity [vx, vy] in m/s.
    wp_start : ndarray
        Start point of desired path [x, y].
    wp_end : ndarray
        End point of desired path [x, y].
    L1_dist : float
        L1 lookahead distance in metres.

    Returns
    -------
    a_cmd : float
        Commanded lateral acceleration in m/s^2.
    eta : float
        Angle between velocity direction and L1 reference direction [rad].
    """

    # 1. Desired path direction
    path_vec = wp_end - wp_start
    path_length = np.linalg.norm(path_vec)

    if path_length < 1e-6:
        raise ValueError("Waypoint path is too short.")

    path_dir = path_vec / path_length

    # 2. Vehicle velocity direction
    speed = np.linalg.norm(vel)

    if speed < 1e-6:
        return 0.0, 0.0

    vel_dir = vel / speed

    # 3. Find the projection of the vehicle onto the path
    relative_pos = pos - wp_start
    along_track = np.dot(relative_pos, path_dir)

    # Lookahead point along the desired path
    lookahead_distance = along_track + L1_dist
    l1_point = wp_start + lookahead_distance * path_dir

    # 4. Direction from vehicle to L1 point
    l1_vector = l1_point - pos
    l1_length = np.linalg.norm(l1_vector)

    if l1_length < 1e-6:
        return 0.0, 0.0

    l1_dir = l1_vector / l1_length

    # 5. Signed angle eta
    cross = vel_dir[0] * l1_dir[1] - vel_dir[1] * l1_dir[0]
    dot = np.dot(vel_dir, l1_dir)

    eta = np.arctan2(cross, dot)

    # 6. L1 lateral acceleration command
    a_cmd = 2.0 * speed**2 / L1_dist * np.sin(eta)

    return a_cmd, eta


# ============================================================
# 2-D Kinematic Vehicle Simulation
# ============================================================

dt = 0.05
simulation_time = 40.0

speed = 15.0

# Desired straight path
wp_start = np.array([0.0, 0.0])
wp_end = np.array([300.0, 0.0])

# L1 lookahead distance
L1_dist = 25.0

# Initial vehicle state: 30 m above the desired path
pos = np.array([0.0, 30.0])
heading = np.deg2rad(0.0)

# Storage
time_history = []
position_history = []
acceleration_history = []
eta_history = []


# ============================================================
# Simulation Loop
# ============================================================

steps = int(simulation_time / dt)

for k in range(steps):

    # Current velocity from heading
    vel = np.array([
        speed * np.cos(heading),
        speed * np.sin(heading)
    ])

    # L1 guidance
    a_cmd, eta = l1_guidance(
        pos,
        vel,
        wp_start,
        wp_end,
        L1_dist
    )

    # Simple kinematic heading response
    heading_rate = a_cmd / speed
    heading = heading + heading_rate * dt

    # Update velocity
    vel = np.array([
        speed * np.cos(heading),
        speed * np.sin(heading)
    ])

    # Update position
    pos = pos + vel * dt

    # Store results
    time_history.append(k * dt)
    position_history.append(pos.copy())
    acceleration_history.append(a_cmd)
    eta_history.append(eta)


# Convert history to arrays
time_history = np.array(time_history)
position_history = np.array(position_history)
acceleration_history = np.array(acceleration_history)
eta_history = np.array(eta_history)


# ============================================================
# Save all plots in a dedicated folder
# ============================================================

OUTPUT_DIR = Path(__file__).resolve().parent / "l1_guidance_plots"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


# ============================================================
# Plot 1: Path Tracking
# ============================================================

plt.figure(figsize=(10, 6))

plt.plot(
    [wp_start[0], wp_end[0]],
    [wp_start[1], wp_end[1]],
    "--",
    label="Desired path"
)

plt.plot(
    position_history[:, 0],
    position_history[:, 1],
    label="Vehicle"
)

plt.scatter(
    pos[0],
    pos[1],
    marker="o",
    label="Final position"
)

plt.xlabel("X position (m)")
plt.ylabel("Y position (m)")
plt.title("L1 Guidance - 2-D Path Tracking")
plt.grid(True)
plt.axis("equal")
plt.legend()
plt.tight_layout()

plt.savefig(
    OUTPUT_DIR / "l1_path_tracking.png",
    dpi=300,
    bbox_inches="tight"
)


# ============================================================
# Plot 2: Commanded Lateral Acceleration
# ============================================================

plt.figure(figsize=(10, 5))

plt.plot(
    time_history,
    acceleration_history
)

plt.xlabel("Time (s)")
plt.ylabel("Lateral acceleration (m/s^2)")
plt.title("L1 Guidance - Commanded Lateral Acceleration")
plt.grid(True)
plt.tight_layout()

plt.savefig(
    OUTPUT_DIR / "l1_lateral_acceleration.png",
    dpi=300,
    bbox_inches="tight"
)


# ============================================================
# Plot 3: L1 Angle Eta
# ============================================================

plt.figure(figsize=(10, 5))

plt.plot(
    time_history,
    np.rad2deg(eta_history)
)

plt.xlabel("Time (s)")
plt.ylabel("Eta (deg)")
plt.title("L1 Guidance - Eta")
plt.grid(True)
plt.tight_layout()

plt.savefig(
    OUTPUT_DIR / "l1_eta.png",
    dpi=300,
    bbox_inches="tight"
)


print()
print("L1 simulation complete.")
print(f"Plots saved in: {OUTPUT_DIR.resolve()}")
print("  l1_path_tracking.png")
print("  l1_lateral_acceleration.png")
print("  l1_eta.png")

plt.show()
