# REQ-008 and REQ-009 --- C172P DLQR Stability Margin Analysis

## 1. Purpose

This document explains how REQ-008 and REQ-009 were verified for the
C172P 4-state DLQR controller. The aim is to connect the controls theory
from lectures to the MATLAB code and to the Bode plot.

Requirements:

-   **REQ-008:** Closed-loop phase margin greater than 30°
-   **REQ-009:** Closed-loop gain margin greater than 6 dB

The complete chain is:

``` text
Aircraft state-space model
        ↓
DLQR state-feedback controller
        ↓
State-feedback return ratio L(z)
        ↓
Frequency response
        ↓
Bode magnitude + phase
        ↓
Gain/phase crossover frequencies
        ↓
Gain margin / phase margin
        ↓
REQ-008 / REQ-009
```

------------------------------------------------------------------------

## 2. The control system being analyzed

The C172P longitudinal model uses four states:

``` text
x = [Δu, Δα, Δθ, q]ᵀ
```

with:

-   `Δu` = forward-velocity perturbation
-   `Δα` = angle-of-attack perturbation
-   `Δθ` = pitch-angle perturbation
-   `q` = pitch-rate perturbation

The continuous model is:

``` text
ẋ = Ax + Bδe
```

After discretization:

``` text
x(k+1) = Ad x(k) + Bd δe(k)
```

The DLQR controller is:

``` text
δe(k) = -Kx(k)
```

with:

``` text
K = [0.0205   2.2685   -7.4923   -2.1182]
```

Therefore the closed-loop state matrix is:

``` text
Acl = Ad - BdK
```

For a discrete-time system, nominal stability requires:

``` text
|λi(Acl)| < 1
```

The nominal poles were:

``` text
0.7745
0.9985
0.9653
0.9835
```

so nominal discrete-time stability passed.

------------------------------------------------------------------------

## 3. What are gain margin and phase margin trying to tell us?

The usual lecture definitions are:

-   **Phase margin:** how much additional phase lag can be tolerated
    before reaching the critical instability condition.
-   **Gain margin:** how much the loop gain can increase before reaching
    the critical instability condition.

The missing connection is that both are measured from the **loop
transfer function**.

For a negative-feedback system, the critical frequency-domain condition
is associated with:

``` text
Magnitude = 1
Phase = -180°
```

or, in Bode-plot units:

``` text
Magnitude = 0 dB
Phase = -180°
```

The Bode plot tells us how close the loop comes to these two conditions
as frequency changes.

------------------------------------------------------------------------

## 4. Why our DLQR system needs a return ratio

A classical textbook loop may look like:

``` text
reference → controller → plant → output → feedback
```

and use:

``` text
L(s) = C(s)G(s)H(s)
```

Your controller is different:

``` text
aircraft states → K → elevator → aircraft → new states
```

The controller uses all four states:

``` text
δe = -Kx
```

So there is not one simple SISO error-to-output transfer function.

For stability-margin analysis we construct the equivalent
**state-feedback return ratio**:

``` text
L(z) = K(zI - Ad)^(-1)Bd
```

This represents one trip around the feedback loop.

------------------------------------------------------------------------

## 5. Deriving L(z) from first principles

Start with:

``` text
x(k+1) = Ad x(k) + Bd δe(k)
```

In the z-domain, the state response to an elevator perturbation can be
written as:

``` text
x(z) = (zI - Ad)^(-1)Bd δe(z)
```

The controller forms:

``` text
Kx(z)
```

Therefore:

``` text
Kx(z)
=
K(zI - Ad)^(-1)Bd δe(z)
```

Hence the loop return ratio is:

``` text
L(z) = K(zI - Ad)^(-1)Bd
```

### Physical interpretation

Imagine injecting a small elevator perturbation:

``` text
elevator perturbation
        ↓
C172P dynamics
        ↓
Δu, Δα, Δθ, q change
        ↓
DLQR gain K processes those states
        ↓
feedback correction
```

`L(z)` mathematically represents how much signal comes back around this
loop, and with what phase shift, for each frequency.

It is not a new physical sensor signal. It is a mathematical model of
the feedback loop used for frequency-domain analysis.

------------------------------------------------------------------------

## 6. Connecting the equation to the MATLAB line

The code is:

``` matlab
L = ss(Ad,Bd,K,0,Ts);
```

MATLAB's discrete state-space form is:

``` text
x(k+1) = A x(k) + B u(k)

y(k)   = C x(k) + D u(k)
```

Therefore:

``` matlab
A = Ad
B = Bd
C = K
D = 0
```

so MATLAB creates:

``` text
x(k+1) = Ad x(k) + Bd u(k)

y(k)   = Kx(k)
```

The input is treated as the elevator perturbation for the return-ratio
calculation.

The output is `Kx`.

Therefore the transfer function from elevator perturbation to `Kx` is
exactly:

``` text
L(z) = K(zI-Ad)^(-1)Bd
```

That is why the seemingly strange `ss(Ad,Bd,K,0,Ts)` line is useful.

------------------------------------------------------------------------

## 7. Why the negative sign is not inside L

The physical control law is:

``` text
δe = -Kx
```

The return ratio is conventionally represented as the positive loop-gain
quantity:

``` text
L(z) = K(zI-Ad)^(-1)Bd
```

The negative-feedback convention is then used when interpreting the
stability margins.

The important point is that we are analyzing the loop gain before
applying the negative-feedback sign.

------------------------------------------------------------------------

# 8. What does a Bode plot actually do?

Suppose we inject a sinusoidal perturbation:

``` text
δe(t) = A sin(ωt)
```

at one frequency.

Then repeat at another frequency:

``` text
δe(t) = A sin(ωt)
```

with a different `ω`.

The aircraft and feedback loop respond differently at different
frequencies.

At every frequency we can ask two questions:

1.  How much does the loop amplify or attenuate the signal?
2.  How much phase shift does the loop introduce?

MATLAB evaluates these quantities over many frequencies and plots them.

That creates the Bode diagram.

So the Bode plot is not a separate mystery. It is simply:

``` text
frequency
    ↓
evaluate L(z)
    ↓
magnitude + phase
    ↓
plot them
```

------------------------------------------------------------------------

# 9. Magnitude and dB

The top Bode plot shows:

``` text
Magnitude (dB)
vs
Frequency (rad/s)
```

Magnitude is converted to decibels using:

``` text
Magnitude_dB = 20 log10(|L|)
```

Important examples:

``` text
|L| = 1       → 0 dB
|L| > 1       → positive dB
|L| < 1       → negative dB
```

So the most important horizontal reference on the magnitude plot is:

``` text
0 dB
```

------------------------------------------------------------------------

# 10. Why frequency is on a logarithmic axis

The plot covers a large range:

``` text
10^-2 to 10^3 rad/s
```

A logarithmic axis lets us see several orders of magnitude clearly:

``` text
0.01 → 0.1 → 1 → 10 → 100 → 1000 rad/s
```

Each step is one decade.

This is why the frequency axis does not look like a normal linear ruler.

------------------------------------------------------------------------

# 11. Gain crossover frequency

The **gain crossover frequency** is where:

``` text
|L| = 1
```

which is:

``` text
Magnitude = 0 dB
```

Your result was:

``` text
ωgc = 25.414871 rad/s
```

In ordinary frequency:

``` text
f = ω/(2π)
```

so:

``` text
f ≈ 4.05 Hz
```

On the top graph, find the point where the magnitude curve crosses the 0
dB line. That x-coordinate is the gain crossover frequency.

------------------------------------------------------------------------

# 12. How the graph gives phase margin

This is the key bridge between the two Bode plots.

At the gain crossover frequency:

``` text
ωgc = 25.414871 rad/s
```

the magnitude is:

``` text
0 dB
```

Now go to the **phase plot at the same frequency**.

The phase is approximately:

``` text
-87.36°
```

The critical phase is:

``` text
-180°
```

Therefore:

``` text
PM = 180° + (-87.36°)
   ≈ 92.64°
```

So:

``` text
PM = 92.644565°
```

### Mental picture

``` text
phase at gain crossover
        -87.36°
           │
           │ 92.64°
           ↓
         -180°
```

The phase margin is simply the remaining phase distance to the critical
-180° point.

------------------------------------------------------------------------

# 13. Phase plot

The lower graph shows:

``` text
Phase (degrees)
vs
Frequency (rad/s)
```

Important reference:

``` text
-180°
```

At -180°, the feedback signal has accumulated the critical phase
reversal associated with the classical negative-feedback instability
condition.

The exact shape of the curve comes from the dynamics of the four-state
aircraft model and the state-feedback return ratio.

You do not need to memorize the shape.

For margin analysis, the important locations are:

``` text
0 dB on magnitude plot
-180° on phase plot
```

------------------------------------------------------------------------

# 14. Phase crossover frequency

The **phase crossover frequency** is where:

``` text
Phase = -180°
```

Your result was:

``` text
ωpc = 377.006199 rad/s
```

Converting to Hz:

``` text
f ≈ 60.00 Hz
```

Your controller sample frequency is:

``` text
fs = 120.005 Hz
```

so the Nyquist frequency is approximately:

``` text
fs/2 ≈ 60.00 Hz
```

Therefore the reported phase crossover is essentially at the Nyquist
frequency.

This is expected for the discrete-time frequency response used here.

------------------------------------------------------------------------

# 15. How the graph gives gain margin

At the phase crossover:

``` text
ωpc = 377.006199 rad/s
```

the phase is:

``` text
-180°
```

Now go to the **magnitude plot at the same frequency**.

The magnitude is approximately:

``` text
-19.72 dB
```

The critical magnitude is:

``` text
0 dB
```

Therefore the distance to the critical point is:

``` text
GM = 0 - (-19.72)
   ≈ 19.72 dB
```

Your MATLAB result was:

``` text
GM = 19.721839 dB
```

### Mental picture

``` text
0 dB       ← critical magnitude
  ↑
  │ 19.72 dB
  │
-19.72 dB  ← magnitude at -180°
```

That distance is the gain margin.

------------------------------------------------------------------------

# 16. Absolute gain margin

MATLAB also reported:

``` text
Gain Margin (absolute) = 9.68483
```

This is the same result expressed as a gain multiplier.

Conversion:

``` text
GM_dB = 20 log10(GM)
```

so:

``` text
20 log10(9.68483)
≈ 19.72 dB
```

Therefore:

``` text
9.68483 × gain
```

would be required to reach the critical gain condition at the phase
crossover.

The dB form is easier to compare with the requirement.

------------------------------------------------------------------------

# 17. How to read the entire graph yourself

Use this procedure whenever you see a Bode plot.

### Step 1 --- Find 0 dB

On the magnitude graph:

``` text
Magnitude = 0 dB
```

This gives:

``` text
Gain crossover frequency
```

For this project:

``` text
25.414871 rad/s
```

### Step 2 --- Move to the phase graph at that frequency

Read the phase:

``` text
≈ -87.36°
```

### Step 3 --- Calculate PM

``` text
PM = 180° + phase
```

Therefore:

``` text
PM ≈ 92.64°
```

### Step 4 --- Find -180°

On the phase graph:

``` text
Phase = -180°
```

This gives:

``` text
Phase crossover frequency
```

For this project:

``` text
377.006199 rad/s
```

### Step 5 --- Move to the magnitude graph at that frequency

Read:

``` text
≈ -19.72 dB
```

### Step 6 --- Calculate GM

``` text
GM = 0 - (-19.72)
   ≈ 19.72 dB
```

------------------------------------------------------------------------

# 18. Why there are two crossover frequencies

The two crossover frequencies answer different questions.

### Gain crossover

``` text
Magnitude = 0 dB
```

Question:

> At what frequency does the loop gain become unity?

Then inspect the phase there to determine **phase margin**.

### Phase crossover

``` text
Phase = -180°
```

Question:

> At what frequency does the loop reach the critical phase?

Then inspect the magnitude there to determine **gain margin**.

Remember:

``` text
0 dB crossing
    ↓
phase margin

-180° crossing
    ↓
gain margin
```

------------------------------------------------------------------------

# 19. Why the margins are called "margins"

Imagine the critical point is:

``` text
Magnitude = 0 dB
Phase = -180°
```

Your system does not reach the critical combination at the same
frequency.

Instead:

-   when magnitude reaches 0 dB, phase is still about -87.36°
-   when phase reaches -180°, magnitude is about -19.72 dB

Therefore the system has distance from the critical condition.

That distance is what we call the stability margin.

------------------------------------------------------------------------

# 20. Physical intuition for phase margin

Suppose the aircraft/control loop has additional delay or unmodeled
dynamics.

Delay adds phase lag.

If your loop is already close to:

``` text
-180°
```

at the unity-gain frequency, a small additional phase lag could cause
trouble.

Your result has:

``` text
PM = 92.64°
```

So, under this modeled return-ratio analysis, there is a substantial
amount of additional phase lag before reaching the classical -180°
condition.

This does not mean the aircraft can tolerate an arbitrary 92.64°
physical delay; it is specifically the phase-margin interpretation for
the analyzed loop.

------------------------------------------------------------------------

# 21. Physical intuition for gain margin

Suppose the effective feedback gain becomes larger than expected.

If the loop is already near:

``` text
0 dB
```

when its phase reaches -180°, only a small gain increase could cause the
critical condition.

Your result instead has:

``` text
Magnitude ≈ -19.72 dB
```

at the -180° phase point.

So the loop gain would need to increase substantially before reaching 0
dB there.

That is why the gain margin is positive and comfortably above the
requirement.

------------------------------------------------------------------------

# 22. Why this is different from REQ-007

REQ-007 and REQ-008/009 are related but not identical.

### REQ-007

We directly perturb model coefficients:

``` text
A, B
 ↓
±30% random uncertainty
 ↓
10,000 models
 ↓
closed-loop poles
```

Question:

> Does the controller remain stable under sampled model uncertainty?

### REQ-008/009

We keep the nominal model and examine the loop in the frequency domain:

``` text
A, B, K
 ↓
L(z)
 ↓
frequency response
 ↓
Bode plot
 ↓
PM / GM
```

Question:

> How far is the nominal feedback loop from the classical gain/phase
> instability conditions?

Together they provide complementary evidence.

------------------------------------------------------------------------

# 23. Important limitation

These margins should not be described as a complete proof of aircraft
robustness.

They are based on:

``` text
the selected linearized C172P model
+
the selected trim point
+
the state-feedback return ratio
+
the discrete-time model
```

They do not automatically cover:

-   nonlinear aerodynamics
-   actuator saturation
-   sensor faults
-   sensor noise/bias
-   structural flexibility
-   large flight-envelope changes
-   all MIMO interactions
-   software timing faults
-   every unmodeled dynamic

Those need separate verification activities.

------------------------------------------------------------------------

# 24. Actual project result

The MATLAB analysis produced:

``` text
Sample time: 0.008333000 s
Sample frequency: 120.005 Hz

Nominal discrete-time stability: PASS
```

### REQ-008

``` text
Phase Margin = 92.644565°
Requirement = > 30°

RESULT: PASS
```

### REQ-009

``` text
Gain Margin = 19.721839 dB
Requirement = > 6 dB

RESULT: PASS
```

Additional values:

``` text
Gain crossover frequency:
25.414871 rad/s

Phase crossover frequency:
377.006199 rad/s

Absolute gain margin:
9.68483
```

------------------------------------------------------------------------

# 25. Bode plot evidence

The project result should be stored alongside this Markdown file as:

``` text
REQ008_REQ009_stability_margins.png
```

and rendered here:

![C172P DLQR State-Feedback Stability
Margins](REQ008_REQ009_stability_margins.png)

The plot shows both the magnitude and phase responses used to calculate
the reported margins.

------------------------------------------------------------------------

# 26. Theory-to-code-to-graph map

This is the most important section to retain for future revision.

  -------------------------------------------------------------------------
  Theory                   MATLAB code              What it means in the
                                                    project
  ------------------------ ------------------------ -----------------------
  `x(k+1)=Adx+Bdδe`        `ss(Ad,Bd,...)`          Discrete C172P aircraft
                                                    dynamics

  `δe=-Kx`                 `K = dlqr(...)`          DLQR elevator feedback

  `L(z)=K(zI-Ad)^(-1)Bd`   `L = ss(Ad,Bd,K,0,Ts)`   State-feedback return
                                                    ratio

  `20log10(|L|)`           `margin(L)`              Magnitude in dB

  `|L|=1`                  0 dB crossing            Gain crossover

  `∠L=-180°`               -180° crossing           Phase crossover

  `PM=180°+phase`          `margin(L)`              Phase margin

  `GM_dB=-magnitude` at    `margin(L)`              Gain margin
  phase crossover                                   
  -------------------------------------------------------------------------

------------------------------------------------------------------------

# 27. The mental model to remember

If you forget the MATLAB syntax, remember this chain:

``` text
                 C172P aircraft
                     Ad, Bd
                       │
                       ▼
             states [Δu Δα Δθ q]
                       │
                       ▼
                       K
                       │
                       ▼
                  elevator
                       │
                       └──────────────┐
                                      │
                                      ▼
                              feedback loop
                                      │
                                      ▼
                                  L(z)
                                      │
                         ┌────────────┴────────────┐
                         ▼                         ▼
                    Magnitude                    Phase
                         │                         │
                         ▼                         ▼
                      0 dB                      -180°
                         │                         │
                         ▼                         ▼
                Gain crossover              Phase crossover
                         │                         │
                         ▼                         ▼
                  read phase                read magnitude
                         │                         │
                         ▼                         ▼
                 Phase margin                Gain margin
                         │                         │
                         ▼                         ▼
                    REQ-008                    REQ-009
                      PASS                       PASS
```

For this project:

``` text
0 dB crossing
    ↓
25.414871 rad/s
    ↓
phase ≈ -87.36°
    ↓
PM = 92.644565°
    ↓
REQ-008 PASS
```

and:

``` text
-180° crossing
    ↓
377.006199 rad/s
    ↓
magnitude ≈ -19.72 dB
    ↓
GM = 19.721839 dB
    ↓
REQ-009 PASS
```

------------------------------------------------------------------------

# 28. Interview-ready explanation

> "For the C172P DLQR controller, I constructed the state-feedback
> return ratio `L(z) = K(zI-Ad)^(-1)Bd` from the discrete aircraft model
> and the DLQR gain. This represents the signal response around one trip
> of the feedback loop. I evaluated its frequency response and used the
> Bode magnitude and phase curves to find the crossover frequencies. The
> gain crossover occurred at 25.41 rad/s, where the phase was about
> -87.36°, giving a phase margin of 92.64°. The phase reached -180° at
> 377.01 rad/s, where the magnitude was about -19.72 dB, giving a gain
> margin of 19.72 dB. Both exceed the requirements of 30° and 6 dB, so
> REQ-008 and REQ-009 passed."

------------------------------------------------------------------------

# 29. If asked "What is L(z)?"

> "It is the state-feedback return ratio. I can interpret it as
> injecting a small elevator perturbation, observing how the aircraft
> states respond, feeding those states through the DLQR gain K, and
> measuring the feedback response as a function of frequency. Its
> magnitude tells me the loop gain and its phase tells me the
> accumulated phase shift."

------------------------------------------------------------------------

# 30. If asked "How did the graph give you phase margin?"

> "I found the frequency where the magnitude crosses 0 dB. That was
> 25.41 rad/s. I then read the phase at that same frequency,
> approximately -87.36°. The remaining distance to -180° is 92.64°,
> which is the phase margin."

------------------------------------------------------------------------

# 31. If asked "How did the graph give you gain margin?"

> "I found the frequency where the phase reaches -180°, approximately
> 377 rad/s. At that frequency the magnitude is approximately -19.72 dB.
> The distance from -19.72 dB to 0 dB is therefore 19.72 dB, which is
> the gain margin."

------------------------------------------------------------------------

# 32. Final verification table

  Requirement                  Measured quantity   Requirement Result
  ------------- -------------------------------- ------------- ----------
  REQ-008          Phase margin = **92.644565°**        \> 30° **PASS**
  REQ-009         Gain margin = **19.721839 dB**       \> 6 dB **PASS**

The analysis result is therefore:

``` text
REQ-008 = PASS
REQ-009 = PASS
```
