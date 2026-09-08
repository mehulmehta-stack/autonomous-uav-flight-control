# SIL Robustness Stress Test

## 1. Purpose

This test evaluated whether the nonlinear C172P JSBSim SIL closed loop
could recover from a larger initial altitude disturbance than the
nominal trimmed condition.

The controller was **not retuned**. The purpose was to observe the
actual behavior of the existing controller under a more demanding
initial condition and document recovery, oscillation, saturation, or
degradation.

## 2. Test Configuration

-   Aircraft/plant: **C172P in JSBSim**
-   Controller execution: **Simulink Software-in-the-Loop (SIL)**
-   Sample time: **0.008333 s (120 Hz)**
-   Altitude reference: **4000 ft**
-   Inner loop: **DLQR**
-   Outer loop: **Altitude PID**
-   LQR gain: unchanged
-   PID gains: unchanged
-   Trim constants: unchanged
-   Actuator limits: unchanged

The inner LQR continued to use perturbation states relative to the
established trim condition. The stress test therefore evaluated the
existing control architecture rather than replacing the perturbation
formulation with absolute states.

## 3. Stress-Test Condition

The normal initialization corresponds approximately to the 4000-ft
operating condition. For the stress test, only the **JSBSim initial
altitude** was changed:

``` text
Initial altitude = 3000 ft
Commanded altitude = 4000 ft
Initial altitude error = +1000 ft
```

The 4000-ft altitude command in the controller was **not changed**.

The experiment therefore tested:

``` text
Aircraft starts at 3000 ft
        ↓
Controller commands 4000 ft
        ↓
Initial altitude error = +1000 ft
        ↓
Closed loop recovers the aircraft
```

A separate stress initialization was used rather than overwriting the
nominal initialization. The other initial-condition values were kept
unchanged, making this an altitude initial-condition disturbance rather
than a redesigned 3000-ft trim condition.

## 4. Procedure

1.  Keep `Autotpilot.slx` unchanged.
2.  Keep the LQR gain, PID gains, trim values, sample time, and actuator
    limits unchanged.
3.  Set the JSBSim initial altitude to **3000 ft** using a separate
    stress initialization.
4.  Keep the controller altitude reference at **4000 ft**.
5.  Run the SIL simulation for approximately **80 s**.
6.  Inspect altitude, altitude error, elevator command, settling,
    overshoot/oscillation, and actuator saturation.
7.  Do not retune the controller based on the result.

## 5. Observed Results

### Altitude

The aircraft started at approximately **3000 ft** and climbed smoothly
toward **4000 ft**.

It reached approximately 4000 ft at around **40--45 s** and then
remained essentially at 4000 ft for the rest of the simulation.

No significant altitude overshoot was visible.

### Altitude Error

The plotted altitude-error signal was approximately:

``` text
Initial error ≈ +1000 ft
        ↓
smooth decrease
        ↓
near zero at ≈45–50 s
        ↓
remains approximately zero
```

No sustained oscillation was observed.

### Elevator Command

The elevator command showed a strong initial transient and briefly
reached approximately **-1.0**, corresponding to the configured lower
elevator saturation limit.

After the initial recovery transient, the elevator returned toward its
normal operating region.

Therefore, the stress test exposed **transient actuator saturation**,
but the saturation did not prevent recovery.

## 6. Result

**Successful recovery with transient actuator saturation.**

The controller:

-   recovered from a **1000-ft initial altitude error**;
-   reached the **4000-ft commanded altitude**;
-   showed **no significant altitude overshoot**;
-   showed **no sustained oscillation**;
-   maintained approximately 4000 ft after recovery; and
-   tolerated brief initial elevator saturation without losing recovery
    capability.

## 7. V&V Interpretation

This provides additional nonlinear SIL evidence that the implemented
closed loop can recover from the tested **+1000-ft initial altitude
disturbance** under the tested C172P JSBSim conditions.

It does **not** establish global nonlinear stability or full
flight-envelope robustness. It does not cover wind/gust disturbances,
sensor noise or bias, large airspeed disturbances, stall/post-stall
recovery, other flight-envelope conditions, hardware execution, or
real-flight behavior.

Recommended V&V conclusion:

> The existing SIL controller successfully recovered the C172P from a
> 1000-ft initial altitude disturbance and settled at the 4000-ft
> command. The recovery involved brief elevator saturation, but no
> sustained oscillation or significant altitude overshoot was observed.

## 8. Evidence

The following figures are the primary visual evidence from the stress-test
run. The image files are intended to be committed to the GitHub repository
alongside this Markdown file.

### Figure 1 — Altitude Response

**Description:** Actual altitude versus time, showing recovery from approximately
3000 ft to the 4000-ft command and subsequent altitude maintenance.

![Altitude response during SIL robustness stress test](images/SIL_robustness_altitude.png)

**Figure 1.** C172P altitude response during the 1000-ft initial-condition stress test.

---

### Figure 2 — Elevator Command

**Description:** Elevator command versus time, showing the strong initial
transient and brief excursion to the approximately -1.0 actuator limit.

![Elevator command during SIL robustness stress test](images/SIL_robustness_elevator.png)

**Figure 2.** Elevator command during the stress-test recovery.

---

### Figure 3 — Altitude Error

**Description:** Altitude error (`4000 - Altitude`) versus time, showing the
initial approximately +1000-ft error decreasing toward zero and remaining
approximately zero after recovery.

![Altitude error during SIL robustness stress test](images/SIL_robustness_altitude_error.png)

**Figure 3.** Altitude error during the stress-test recovery.

---

**GitHub image files to add:**

```text
images/SIL_robustness_altitude.png
images/SIL_robustness_elevator.png
images/SIL_robustness_altitude_error.png
```

Keep the filenames and relative paths exactly as shown above so GitHub
renders the figures automatically.

## 9. Relation to the Earlier 3000-ft Command Test

A separate test changed the **altitude command** from 4000 ft to 3000 ft
while the aircraft was initially near 4000 ft. The aircraft descended
and maintained approximately 3000 ft. That test demonstrated large
**setpoint tracking**.

This robustness test was different:

``` text
Earlier test:
Initial aircraft ≈ 4000 ft
Command = 3000 ft
→ large setpoint change

Robustness test:
Initial aircraft = 3000 ft
Command = 4000 ft
→ large initial-condition disturbance
```

The second test is the one used for the SIL robustness stress-test task.

## 10. Key Takeaway

``` text
3000 ft initial condition
        ↓
+1000 ft altitude error
        ↓
brief elevator saturation
        ↓
smooth climb
        ↓
≈4000 ft at ~40–45 s
        ↓
altitude error → ≈0
        ↓
stable altitude maintenance
```

The implemented nonlinear JSBSim SIL closed loop therefore successfully
recovered from the tested 1000-ft initial altitude disturbance, with
transient actuator saturation but without sustained oscillation or
significant overshoot.
