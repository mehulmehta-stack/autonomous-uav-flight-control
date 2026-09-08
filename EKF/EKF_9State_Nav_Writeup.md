# 9-State INS/GPS Extended Kalman Filter — `nav_ekf.py`

## What this is

A 9-state nonlinear Extended Kalman Filter fusing simulated IMU and GPS
measurements into a full navigation state: position, velocity, and attitude.
This is the third and most advanced stage of the EKF progression in this
folder — see `EKF_GPS_Code_Explanation_Beginner_Guide.md` for the earlier
2-state and 6-state stages that led here.

## State vector

```
x = [N, E, D, VN, VE, VD, phi, theta, psi]
```

- `N, E, D` — NED position [m]
- `VN, VE, VD` — NED velocity [m/s]
- `phi, theta, psi` — roll, pitch, yaw [rad]

## Inputs and measurements

- **IMU (prediction, 100 Hz):** `u = [ax, ay, az, p, q, r]` — specific force
  in the body frame and body angular rates. Used to propagate the full
  9-state estimate between GPS updates.
- **GPS (correction, 10 Hz):** `z = [N, E, D]` — position only. No velocity,
  heading, or magnetometer measurement is used.

This filter intentionally does **not** estimate accelerometer or gyro bias
— that's a known, deliberate scope limit, not an oversight.

## Measurement model and what it does — and doesn't — correct

The GPS measurement matrix only touches the position rows:

```
H = zeros(3, 9)
H[0,0] = 1   # N
H[1,1] = 1   # E
H[2,2] = 1   # D
```

Velocity and attitude are never directly measured. They're corrected only
indirectly, through the cross-covariance terms that couple attitude and
velocity in the process model (attitude determines how body-frame specific
force rotates into the NED frame, so an attitude error shows up as a
velocity-prediction error, which the GPS position/velocity innovation can
then partially correct).

This works reasonably well for roll and pitch, because gravity gives a
strong, persistent coupling between attitude and specific force. It does
**not** work well for yaw: heading errors barely affect the predicted NED
velocity during steady, non-maneuvering flight, so yaw is only weakly
observable from position alone. In practice this means yaw is close to pure
gyro dead-reckoning here, and it will drift over time without a compass,
magnetometer, or GPS-course-over-ground measurement to anchor it.

**This is the known limitation to carry into the final README:** the roll
and pitch traces track truth closely; the yaw trace visibly diverges over
the 60-second run and does not fully re-converge, consistent with the
observability argument above rather than a bug.

## GPS outage test

- Outage window: **t = 25 s to t = 35 s** (10 seconds), GPS updates withheld,
  filter coasts on IMU-only prediction.
- 3D position error grows during the outage and peaks near the moment GPS
  reacquires, then collapses back down once corrections resume.
- Position uncertainty (1σ, from the filter's own covariance) grows in step
  with the outage and contracts again after reacquisition — the filter's
  internal confidence estimate tracks its actual error growth, which is the
  behavior you want to see from a Kalman filter's covariance.

## Known limitations (for the final README)

- No accelerometer/gyro bias estimation — a real IMU's bias would leak
  directly into position drift during any GPS outage longer than modeled
  here.
- Yaw is weakly observable from position-only GPS; see above.
- GPS noise is zero-mean and stationary in this simulation — no multipath,
  no correlated error, no outliers.
- This is a synthetic-truth simulation, not hardware-in-the-loop or flight
  data.
