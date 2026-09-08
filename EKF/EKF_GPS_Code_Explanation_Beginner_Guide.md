# EKF / GPS Fusion — Beginner Code Explanation

> **Purpose of this document:** explain what the GPS/IMU estimator code is actually doing, from first principles, without assuming strong Python or Kalman-filter knowledge.
>
> This document is written alongside `gps_ekf.py` and is intended to be part of the GitHub `/ekf` documentation.

---

# 1. First: what problem are we solving?

Before looking at the Python code, forget the word **EKF** for a moment.

Imagine an aircraft flying.

We want the aircraft's navigation system to know:

```text
Where am I?
How fast am I moving?
```

But the aircraft does not have one perfect sensor that tells us everything.

Instead, we have sensors with different strengths.

### IMU

The IMU contains accelerometers and gyroscopes.

For this simplified GPS example, we use the accelerometer.

It tells us approximately:

```text
How much acceleration am I experiencing?
```

From acceleration, we can calculate velocity.

From velocity, we can calculate position.

So:

```text
acceleration
     ↓ integrate
velocity
     ↓ integrate
position
```

The problem is that sensor errors also get integrated.

For example:

```text
small acceleration error
        ↓
small velocity error
        ↓
larger position error
```

This is called **inertial drift**.

---

# 2. Why do we need GPS?

GPS gives us an approximate absolute position.

For example:

```text
GPS says:

North = 100 m
East  = 50 m
Altitude = 120 m
```

GPS is useful because it does not depend on continuously integrating acceleration.

But GPS is noisy.

It may say:

```text
99.2 m
101.5 m
98.7 m
100.8 m
```

even if the aircraft is actually at 100 m.

Therefore:

- IMU is good for continuous short-term motion propagation.
- GPS is good for correcting accumulated position drift.

The basic idea is therefore:

```text
             ┌───────────────┐
             │      IMU      │
             │ acceleration  │
             └───────┬───────┘
                     ↓
                Prediction
                     ↓
             estimated state
                     ↑
                Correction
                     ↑
             ┌───────┴───────┐
             │      GPS      │
             │   position    │
             └───────────────┘
```

This is the core idea behind sensor fusion.

---

# 3. What does the filter actually do?

The filter repeats two operations:

```text
1. PREDICT
2. UPDATE
```

### Predict

Use the IMU and the aircraft motion model to predict where the aircraft should be.

```text
previous state + IMU
            ↓
       predicted state
```

### Update

When GPS is available, compare the prediction with the GPS measurement and correct the estimate.

```text
prediction + GPS
      ↓
 corrected estimate
```

So the complete loop is:

```text
             ┌─────────────┐
             │ Previous x  │
             └──────┬──────┘
                    │
                    │ IMU
                    ↓
             ┌─────────────┐
             │  PREDICT    │
             └──────┬──────┘
                    │
                    ↓
             predicted x
                    │
             GPS available?
                /       \
              YES        NO
               ↓          ↓
          UPDATE       coast
               ↓          ↓
          corrected x   predicted x
               \          /
                \        /
                 ↓      ↓
               next state
```

During a GPS outage, the filter simply cannot perform the update.

It must coast using the IMU.

---

# 4. Why are we generating synthetic sensor data?

We do not have a real aircraft IMU and GPS connected to this Python experiment.

Therefore we create a mathematical "truth" trajectory.

For example:

```python
true_north = 2.0 * t + 8.0 * np.sin(0.08 * t)
```

This creates the position that we pretend the aircraft actually has.

Think of it as the answer key.

We know the true aircraft position because **we created it**.

Then we create fake sensors that are imperfect versions of that truth.

That lets us test whether our estimator can recover the true state.

---

# 5. Creating time

The code starts with:

```python
DT = 0.1
SIM_TIME = 60.0
N = int(SIM_TIME / DT)

t = np.arange(N) * DT
```

### `DT`

```python
DT = 0.1
```

means:

```text
simulation timestep = 0.1 second
```

Therefore the simulation runs at:

```text
1 / 0.1 = 10 Hz
```

### `SIM_TIME`

```python
SIM_TIME = 60.0
```

means the simulation lasts 60 seconds.

### `N`

```python
N = int(SIM_TIME / DT)
```

means:

```text
60 / 0.1 = 600 samples
```

### `t`

```python
t = np.arange(N) * DT
```

creates:

```text
0.0
0.1
0.2
0.3
...
59.9
```

This is our simulation clock.

---

# 6. Creating the true aircraft trajectory

We create three position components:

```python
true_north = 2.0 * t + 8.0 * np.sin(0.08 * t)

true_east = 1.5 * t + 5.0 * np.sin(0.11 * t)

true_alt = 100.0 + 0.8 * t + 4.0 * np.sin(0.12 * t)
```

These are not real aircraft equations.

They are simply a controlled mathematical trajectory for testing.

For example:

```text
true_north
```

is the actual north position.

```text
true_east
```

is the actual east position.

```text
true_alt
```

is the actual altitude.

We now have our hidden "truth":

```text
TRUE AIRCRAFT
────────────────────────────

North
East
Altitude
Velocity
Acceleration
```

The estimator does not get to see this truth directly.

The truth is only used to create synthetic sensors and later evaluate the estimator.

---

# 7. Getting velocity from position

The code uses:

```python
true_vn = np.gradient(true_north, DT)
true_ve = np.gradient(true_east, DT)
true_vz = np.gradient(true_alt, DT)
```

`np.gradient()` approximately differentiates the position.

The basic physics relationship is:

```text
velocity = derivative of position
```

So:

```text
north position
      ↓ derivative
north velocity
```

and similarly for east and vertical motion.

---

# 8. Getting acceleration from velocity

Next:

```python
true_an = np.gradient(true_vn, DT)
true_ae = np.gradient(true_ve, DT)
true_az = np.gradient(true_vz, DT)
```

Again, differentiation is being used.

The physics relationship is:

```text
acceleration = derivative of velocity
```

Therefore:

```text
position
   ↓ derivative
velocity
   ↓ derivative
acceleration
```

At this point we know the "true" acceleration that our imaginary aircraft experiences.

---

# 9. Creating the synthetic accelerometer

Now comes the important sensor simulation.

We define:

```python
imu_noise_std = 0.08
```

This represents the standard deviation of the random accelerometer measurement noise.

We also define:

```python
imu_bias = np.array([
    0.04,
    -0.03,
    0.06
])
```

This represents a constant sensor bias.

The three values correspond to:

```text
North acceleration bias
East acceleration bias
Vertical acceleration bias
```

The synthetic IMU measurement is then:

```python
imu_acceleration = (
    true_acceleration
    + imu_bias
    + RNG.normal(0.0, imu_noise_std, (N, 3))
)
```

This line is extremely important.

It means:

```text
measured acceleration
=
true acceleration
+
constant bias
+
random noise
```

---

# 10. What does `RNG.normal()` mean?

This is where the random sensor noise comes from.

```python
RNG.normal(
    0.0,
    imu_noise_std,
    (N, 3)
)
```

means:

```text
generate random numbers
with:

mean = 0
standard deviation = imu_noise_std
shape = N × 3
```

So every IMU sample receives a slightly different random error.

For example, imagine the true acceleration is:

```text
2.00 m/s²
```

The sensor might report:

```text
2.06
1.95
2.03
2.10
1.97
...
```

The sensor is not broken.

It is simply noisy.

This is what real sensors do.

---

# 11. Why add a constant bias as well?

Random noise and bias are different.

### Random noise

Changes from sample to sample:

```text
+0.02
-0.04
+0.01
-0.03
...
```

### Bias

Stays approximately in the same direction:

```text
+0.04
+0.04
+0.04
+0.04
...
```

Bias is especially important for inertial navigation.

Suppose the real acceleration is:

```text
0
```

but the accelerometer has a bias:

```text
+0.04 m/s²
```

The estimator keeps believing there is a small acceleration.

That creates velocity error.

Then velocity error creates position error.

So:

```text
0.04 m/s² acceleration bias
          ↓
velocity error
          ↓
position error
```

This is why inertial navigation can drift significantly without an external reference.

---

# 12. Creating synthetic GPS

The GPS is created using:

```python
gps_noise_std = np.array([
    3.0,
    3.0,
    5.0
])
```

This means we pretend GPS has approximately:

```text
North uncertainty:    3 m
East uncertainty:     3 m
Altitude uncertainty: 5 m
```

Then:

```python
gps = (
    true_position
    + RNG.normal(0.0, gps_noise_std, (N, 3))
)
```

creates noisy GPS measurements.

So:

```text
true position
      +
GPS noise
      ↓
GPS measurement
```

The estimator sees the noisy GPS measurement, not the truth.

---

# 13. What is the filter state?

The filter needs to decide exactly what quantities it is trying to estimate.

We define:

```text
x = [north, east, altitude, vn, ve, vz]
```

So the six state variables are:

```text
1. north position
2. east position
3. altitude
4. north velocity
5. east velocity
6. vertical velocity
```

In Python, the initial state is:

```python
x = np.array([
    true_north[0] + 5.0,
    true_east[0] - 4.0,
    true_alt[0] + 5.0,
    0.0,
    0.0,
    0.0
])
```

Notice something important:

The filter is deliberately not initialized perfectly.

For example, north begins 5 m away from truth.

This allows us to see whether the GPS updates can correct the estimate.

---

# 14. What is `P`?

The code contains:

```python
P = np.diag([
    10.0**2,
    10.0**2,
    10.0**2,
    2.0**2,
    2.0**2,
    2.0**2
])
```

`P` is the **state covariance matrix**.

For a beginner, think of it as:

> "How uncertain is the filter about each part of its state?"

For example:

```text
position uncertainty ≈ 10 m
velocity uncertainty ≈ 2 m/s
```

The filter does not only estimate:

```text
x = where am I?
```

It also maintains:

```text
P = how confident am I about x?
```

This becomes extremely important in the Kalman gain calculation.

---

# 15. The prediction model

The aircraft motion is approximated by:

```text
position_next
=
position
+
velocity × dt
+
0.5 × acceleration × dt²
```

and:

```text
velocity_next
=
velocity
+
acceleration × dt
```

This is basic kinematics.

For one dimension:

```text
x_next = x + v dt + 1/2 a dt²
v_next = v + a dt
```

The code expresses the same idea using matrices.

---

# 16. What is `F`?

The matrix:

```python
F = np.array([
    [1, 0, 0, DT, 0, 0],
    [0, 1, 0, 0, DT, 0],
    [0, 0, 1, 0, 0, DT],
    [0, 0, 0, 1,  0,  0],
    [0, 0, 0, 0,  1,  0],
    [0, 0, 0, 0,  0,  1]
])
```

describes how the old state affects the next state.

For example, the first row says approximately:

```text
new north
=
old north
+
old north velocity × DT
```

The matrix is simply a compact way of writing the six state equations.

---

# 17. What is `B`?

The matrix:

```python
B = np.array([
    [0.5 * DT**2, 0, 0],
    [0, 0.5 * DT**2, 0],
    [0, 0, 0.5 * DT**2],
    [DT, 0, 0],
    [0, DT, 0],
    [0, 0, DT]
])
```

describes how acceleration affects the state.

So:

```text
F → previous state contribution

B → IMU acceleration contribution
```

The complete prediction equation is:

```text
x_pred = F x + B u
```

where:

```text
u = IMU acceleration
```

---

# 18. The most important line in the corrected code

This is the heart of the corrected experiment:

```python
x_pred = F @ x + B @ imu_acceleration[k]
```

Read it conceptually as:

```text
new predicted state
=
previous state contribution
+
IMU acceleration contribution
```

The `@` symbol means matrix multiplication.

You do not need to memorize matrix multiplication yet.

The important engineering meaning is:

```text
previous navigation estimate
          +
IMU information
          ↓
predicted navigation estimate
```

---

# 19. What does `H` mean?

GPS only measures position.

It does not directly measure all six state variables.

Our state is:

```text
[north, east, altitude, vn, ve, vz]
```

GPS gives:

```text
[north, east, altitude]
```

Therefore:

```python
H = np.array([
    [1, 0, 0, 0, 0, 0],
    [0, 1, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0]
])
```

`H` tells the filter:

> "GPS observes these three components of my six-component state."

So:

```text
x
 ↓
H
 ↓
predicted GPS-like measurement
```

---

# 20. Prediction covariance

The code has:

```python
P_pred = F @ P @ F.T + Q
```

This is the covariance prediction equation.

You can understand it conceptually as:

```text
old uncertainty
      +
uncertainty introduced by the model/sensors
      ↓
new predicted uncertainty
```

`Q` represents process/model uncertainty.

Why do we need it?

Because our model is not perfect.

The real aircraft does not exactly follow our simple equations, and the IMU is noisy.

Therefore uncertainty should increase as we propagate the state.

---

# 21. GPS measurement noise matrix `R`

The code defines:

```python
R = np.diag(gps_noise_std**2)
```

This represents GPS measurement uncertainty.

Remember:

```text
P → uncertainty in our estimate

R → uncertainty in the GPS measurement
```

The filter needs both because it must decide:

> "Should I trust my prediction more, or should I trust GPS more?"

---

# 22. What is the innovation?

When GPS is available:

```python
z = gps[k]

innovation = z - H @ x_pred
```

The innovation is simply:

```text
innovation
=
GPS measurement
-
predicted measurement
```

For example:

```text
GPS says:             102 m

filter predicted:     100 m

innovation:             2 m
```

So the filter knows:

```text
"My prediction and GPS disagree by 2 m."
```

That difference is what drives the correction.

---

# 23. What is the Kalman gain?

The code calculates:

```python
S = H @ P_pred @ H.T + R

K = np.linalg.solve(
    S,
    (P_pred @ H.T).T
).T
```

Do not worry about memorizing this yet.

Conceptually, `K` answers:

> "How strongly should I react to the measurement?"

If the filter believes GPS is very noisy:

```text
trust GPS less
```

If the filter believes its own prediction is very uncertain:

```text
trust GPS more
```

So the Kalman gain is essentially the **trust-balancing mechanism**.

---

# 24. The correction equation

The actual correction is:

```python
x = x_pred + K @ innovation
```

Conceptually:

```text
corrected estimate
=
prediction
+
correction
```

and:

```text
correction
=
Kalman gain × measurement disagreement
```

So:

```text
prediction
    +
GPS disagreement × trust factor
    ↓
corrected estimate
```

This is the key idea behind the Kalman filter.

---

# 25. What happens when GPS disappears?

This is the part we specifically fixed.

The code determines whether GPS is available:

```python
gps_available = not (
    GPS_OUTAGE_START <= t[k] < GPS_OUTAGE_END
)
```

The outage is:

```text
25 s to 35 s
```

When GPS is available:

```python
if gps_available:
```

the filter performs the measurement update.

When GPS is unavailable:

```python
else:
    x = x_pred
    P = P_pred
```

There is no GPS correction.

This means:

```text
IMU
 ↓
prediction
 ↓
prediction
 ↓
prediction
 ↓
prediction
```

The filter is effectively coasting.

---

# 26. Why does the estimate drift during the outage?

This is the most important physical lesson of this experiment.

Our IMU contains:

```text
constant bias
+
random noise
```

The filter integrates acceleration to obtain velocity and position.

Therefore:

```text
IMU error
   ↓
velocity error
   ↓
position error
```

During normal operation, GPS repeatedly corrects this error.

During outage:

```text
GPS correction = unavailable
```

Therefore the accumulated error is no longer corrected.

The position estimate drifts.

---

# 27. Why does the graph improve after GPS returns?

At 35 seconds, GPS becomes available again.

The filter goes back to:

```text
prediction
      ↓
GPS measurement
      ↓
innovation
      ↓
Kalman gain
      ↓
correction
```

Therefore the accumulated inertial error can be corrected.

This is exactly what the final graph should demonstrate.

---

# 28. Understanding your corrected graph

The most important section is the shaded GPS-outage region.

At the beginning:

```text
GPS available
```

The estimator receives regular corrections.

Then:

```text
25 s
↓
GPS OFF
```

The position error begins increasing.

At:

```text
35 s
↓
GPS ON
```

the estimator starts receiving position corrections again.

The corrected validation gave approximately:

```text
Error at outage start = 1.283 m

Error at outage end   = 9.910 m
```

So the experiment demonstrates:

```text
1.283 m
   ↓
   ↓ 10 seconds without GPS
   ↓
9.910 m
```

That is the behavior we wanted.

---

# 29. Why the first implementation was misleading

The old implementation had effectively given the filter a synthetic velocity measurement.

That meant:

```text
GPS outage
      ↓
GPS unavailable

BUT

synthetic velocity still available
      ↓
velocity state remains artificially constrained
```

Therefore position could remain unrealistically accurate.

That is why the earlier result showed something like:

```text
error during outage < error before outage
```

which was suspicious.

The corrected experiment removes that artificial velocity measurement.

Now:

```text
GPS outage
      ↓
only IMU drives propagation
      ↓
bias accumulates
      ↓
position drifts
```

This is much more physically meaningful.

---

# 30. One terminology correction: KF vs EKF

There is an important detail for your GitHub documentation.

This simplified model uses:

```text
linear state equations
```

Therefore, technically, it is a:

> **Kalman Filter (KF)**

rather than an:

> **Extended Kalman Filter (EKF)**

This is not a problem.

In fact, it is a better learning progression.

The sequence should be:

```text
Simple KF
    ↓
Understand prediction
    ↓
Understand covariance
    ↓
Understand measurement update
    ↓
Understand sensor fusion
    ↓
Understand GPS outage
    ↓
Nonlinear aircraft navigation model
    ↓
9-state INS/GPS EKF
```

The later 9-state INS/GPS estimator is the one that matters for the roadmap.

---

# 31. Where does this fit into your aircraft controls project?

This is the connection that matters most.

Your overall aircraft project is not just:

```text
aircraft → controller → actuator
```

A more realistic flight-control architecture is:

```text
             REAL AIRCRAFT
                  │
          ┌───────┴────────┐
          ↓                ↓
        Sensors          Sensors
       IMU / GPS        Air data etc.
          │
          ↓
    STATE ESTIMATOR
       (EKF / INS)
          │
          ↓
    estimated aircraft
         state
          │
          ↓
      CONTROLLER
     LQR / LQG etc.
          │
          ↓
       actuator
          │
          ↓
       AIRCRAFT
```

The estimator and controller solve different problems.

---

# 32. Estimator vs controller

This distinction is critical.

### Estimator

Answers:

```text
"What is the aircraft actually doing?"
```

Examples:

```text
roll
pitch
yaw
position
velocity
angular rates
```

Sensors are noisy, so the estimator combines them to produce the best state estimate.

### Controller

Answers:

```text
"What should I command to make the aircraft do what I want?"
```

For example:

```text
desired altitude
      ↓
controller
      ↓
elevator command
      ↓
aircraft pitch
      ↓
altitude changes
```

Therefore:

```text
Estimator = understand aircraft state

Controller = act on aircraft state
```

---

# 33. Where LQR enters

Your existing controls work uses LQR.

LQR takes an estimated state and calculates a control action.

Conceptually:

```text
desired state
      ↓
state error
      ↓
LQR
      ↓
control command
```

For example:

```text
desired pitch = 5°

estimated pitch = 3°

pitch error = 2°

LQR
 ↓
elevator command
```

But in a real system, where does:

```text
estimated pitch = 3°
```

come from?

Sensors.

And sensors are noisy.

That is where the estimator enters.

---

# 34. Where LQG enters

LQG means:

> **Linear Quadratic Gaussian**

It combines:

```text
LQR controller
+
Kalman state estimator
```

Conceptually:

```text
               ┌──────────────┐
               │    AIRCRAFT  │
               └──────┬───────┘
                      │
                    sensors
                      ↓
              ┌───────────────┐
              │ Kalman Filter │
              └───────┬───────┘
                      │
                estimated state
                      ↓
              ┌───────────────┐
              │     LQR       │
              └───────┬───────┘
                      │
                 control input
                      ↓
                  AIRCRAFT
```

This is why the EKF/Kalman-filter work is **not random side learning**.

It connects directly to the LQG task already present in your roadmap.

---

# 35. Important limitation: this simple KF is not your final aircraft estimator

Do not present this small Python example as:

> "I built a complete aircraft INS."

You have not.

This is an educational stepping stone.

The real roadmap progression is:

```text
Python attitude estimator
          ↓
simple GPS/IMU fusion
          ↓
GPS outage demonstration
          ↓
9-state INS/GPS EKF
          ↓
estimated aircraft navigation state
          ↓
LQG / advanced GNC integration
```

The 9-state estimator is where the connection becomes much closer to actual aerospace GNC.

---

# 36. Why the future 9-state EKF matters

> **Update:** the sections below were written while the 9-state EKF was
> still a future step. It has since been implemented — see `nav_ekf.py`
> and `EKF_9State_Nav_Writeup.md` in this folder for the actual design,
> results, and known limitations. Kept below as-is because it's an
> accurate record of the plan at the time, not because it's still current.

The eventual state will contain navigation quantities such as:

```text
position
velocity
attitude
```

rather than only the simplified Cartesian position/velocity state used here.

The real aircraft problem is nonlinear because:

- attitude is represented with nonlinear rotation relationships,
- accelerometer measurements depend on attitude,
- gravity must be accounted for,
- body-frame measurements must be transformed into navigation-frame quantities,
- position may be represented using latitude/longitude/altitude.

That is where the **Extended** Kalman Filter becomes useful.

The EKF handles nonlinear dynamics by locally linearizing them around the current estimate.

You do not need to implement that part yet to understand today's experiment.

---

# 37. The complete mental model to remember

For now, remember only this:

```text
                   AIRCRAFT
                      │
                      ↓
                  IMU + GPS
                      │
                      ↓
               ┌─────────────┐
               │   ESTIMATOR │
               │  KF / EKF   │
               └──────┬──────┘
                      │
              estimated state
                      │
                      ↓
               ┌─────────────┐
               │  CONTROLLER │
               │     LQR     │
               └──────┬──────┘
                      │
                control command
                      │
                      ↓
                   AIRCRAFT
```

And remember the two questions:

```text
Estimator:
"What is the aircraft doing?"

Controller:
"What should I make the aircraft do?"
```

That distinction is the bridge between your estimation work and your existing flight-control work.

---

# 38. What this task contributes to the roadmap

This task gives you several pieces of the eventual GNC stack:

```text
Sensor simulation
      ↓
State estimation
      ↓
Sensor fusion
      ↓
GPS outage handling
      ↓
Inertial drift understanding
      ↓
9-state INS/GPS EKF
      ↓
LQG connection
```

So this is not merely a Python exercise.

It is a small, controlled environment in which you learn the estimator before putting the estimator into the larger aircraft GNC architecture.

---

# 39. What you should be able to explain in an interview

After completing this task, you should be able to say:

> "I implemented a simplified inertial/GPS state estimator in Python. I generated a known truth trajectory, simulated accelerometer bias and noise and noisy GPS measurements, and used IMU acceleration for state prediction and GPS position for measurement correction. I then disabled GPS for 10 seconds so the estimator had to coast on inertial data alone. The accumulated IMU bias caused position drift, and when GPS returned the estimator corrected the accumulated error."

If asked:

**"Why does the position drift?"**

Answer:

> "Because accelerometer errors are integrated into velocity and then position, so even a small bias accumulates during inertial propagation."

If asked:

**"Why use GPS?"**

Answer:

> "GPS provides an absolute position reference that limits long-term inertial drift."

If asked:

**"Where does this connect to control?"**

Answer:

> "The estimator provides the controller with an estimate of the aircraft state. In an LQG architecture, the Kalman estimator supplies the state estimate used by the LQR controller."

---

# 40. Final mental picture

Do not think:

```text
"I am doing a random EKF Python task."
```

Think:

```text
                    MY AIRCRAFT GNC SYSTEM

       Sensors
      /        \
     IMU       GPS
      \        /
       \      /
        ↓    ↓
     STATE ESTIMATOR
        KF / EKF
           │
           │ estimated state
           ↓
       CONTROLLER
       LQR / LQG
           │
           │ control command
           ↓
        AIRCRAFT
           │
           └─────────────── feedback ────────────────┘
```

Your current Python task is simply building and understanding the **state-estimation block** in isolation before connecting it to the larger GNC system.

That is why it belongs in the roadmap.
