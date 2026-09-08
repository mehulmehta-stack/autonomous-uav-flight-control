# REQ-007 --- 30% Model Uncertainty Analysis

## 1. Purpose

**Requirement:** REQ-007 --- Controller stable with 30% model parameter
uncertainty.

**Verification method:** Analysis.

This document explains exactly how REQ-007 was verified for the C172P
4-state DLQR controller, what "30% model uncertainty" means in this
project, how the Monte Carlo analysis works, how the two result plots
are constructed and interpreted, and what the final PASS result actually
proves.

The purpose of this analysis is to check whether the **same nominal DLQR
controller** remains stable when the aircraft model used by the
controller designer is changed by up to ±30% in the selected non-zero
coefficients of the linearized aircraft model.

This is a robustness analysis. It is deliberately separate from normal
MIL/SIL simulation verification.

------------------------------------------------------------------------

## 2. Requirement being verified

REQ-007 is defined in the project requirements table as:

> **Controller stable with 30% model parameter uncertainty**

The verification method is:

> **Analysis**

The requirement is therefore not being verified by flying the aircraft
or by checking one nominal simulation. Instead, the mathematical
aircraft model is deliberately modified, repeatedly tested, and checked
for closed-loop stability.

------------------------------------------------------------------------

## 3. Controller and model used

The analysis uses the final C172P 4-state longitudinal model.

The state vector is:

``` text
x = [Δu, Δα, Δθ, q]ᵀ
```

where:

-   `Δu` = perturbation in forward velocity
-   `Δα` = perturbation in angle of attack
-   `Δθ` = perturbation in pitch angle
-   `q` = pitch rate

The continuous-time aircraft model is:

``` text
ẋ = A x + B δe
```

and the DLQR controller is:

``` text
δe = -Kx
```

Therefore the discrete-time closed-loop model is:

``` text
x(k+1) = (Ad - Bd K)x(k)
```

and the closed-loop system matrix is:

``` text
Acl = Ad - Bd K
```

For a discrete-time system, stability requires every closed-loop pole to
lie strictly inside the unit circle:

``` text
|λi| < 1
```

where `λi` is a closed-loop eigenvalue.

------------------------------------------------------------------------

## 4. Nominal controller used for the analysis

The controller is designed once using the nominal aircraft model.

The sample time is:

``` text
Ts = 0.008333 s
```

which corresponds to approximately 120 Hz.

The DLQR controller produces:

``` text
K = [0.0205   2.2685   -7.4923   -2.1182]
```

The nominal closed-loop poles obtained before adding uncertainty were:

``` text
0.7745
0.9985
0.9653
0.9835
```

Their magnitudes are also:

``` text
0.7745
0.9985
0.9653
0.9835
```

The largest nominal pole magnitude was:

``` text
0.998496051358
```

Therefore the nominal controller is stable because:

``` text
0.998496051358 < 1
```

### Important observation

The dominant pole is already quite close to the discrete-time stability
boundary of `1`.

That means the controller has a relatively slow/dominant mode, so
robustness testing is useful rather than simply assuming that nominal
stability is enough.

------------------------------------------------------------------------

## 5. What does "30% model uncertainty" mean?

This phrase can be misunderstood.

In this analysis, **30% uncertainty does not mean that every possible
aircraft property, sensor, actuator, or environmental condition is being
changed by 30%.**

Instead, uncertainty is applied to selected non-zero coefficients of the
linearized aircraft state-space matrices:

``` text
A
B
```

For each selected coefficient:

``` text
coefficient_uncertain
    =
coefficient_nominal × (1 + random value between -0.30 and +0.30)
```

For example, if one model coefficient were:

``` text
A(1,1) = -0.060668
```

then one uncertainty realization could make it approximately:

``` text
-0.060668 × 0.70
```

while another could make it:

``` text
-0.060668 × 1.30
```

and another could produce any value between those limits.

The uncertainty is independently randomized for the selected
coefficients in each Monte Carlo trial.

------------------------------------------------------------------------

## 6. Which parameters were actually perturbed?

The MATLAB analysis used a numerical threshold so that values which are
effectively numerical round-off are not treated as physical model
parameters.

The threshold was:

``` matlab
threshold = 1e-12;
```

Therefore:

``` text
|coefficient| > 1e-12
```

was treated as a meaningful non-zero coefficient and perturbed.

The analysis reported:

``` text
Perturbed A coefficients: 13
Perturbed B coefficients: 3
```

Therefore:

``` text
13 + 3 = 16
```

meaningful model coefficients were independently subjected to ±30%
uncertainty.

The extremely small terms around `10^-18` in the supplied `A` matrix
were therefore treated as numerical zeros rather than physical
parameters.

------------------------------------------------------------------------

## 7. Why is the DLQR gain K kept fixed?

This is one of the most important parts of the analysis.

The controller gain is calculated from the **nominal aircraft model**:

``` matlab
K = dlqr(Ad,Bd,Q,R);
```

After that, the same `K` is used for every uncertain model.

For each uncertainty trial:

``` text
Nominal controller
       |
       v
      K
       |
       v
Uncertain aircraft model
       |
       v
A_uncertain, B_uncertain
       |
       v
Acl = Ad_uncertain - Bd_uncertain K
       |
       v
Closed-loop poles
```

We intentionally do **not** redesign the controller for every uncertain
model.

If we used:

``` matlab
K_uncertain = dlqr(Ad_uncertain,Bd_uncertain,Q,R);
```

we would be answering a different question: whether a new controller
could be designed after the aircraft model changed.

REQ-007 instead asks whether the controller that was already designed
remains stable despite model uncertainty.

------------------------------------------------------------------------

## 8. What is Monte Carlo analysis?

### 8.1 Basic idea

Monte Carlo analysis is a numerical experiment in which uncertain
quantities are randomly sampled many times.

Instead of testing only one nominal aircraft model:

``` text
A, B
```

we create many slightly different versions:

``` text
A₁, B₁
A₂, B₂
A₃, B₃
...
A₁₀₀₀₀, B₁₀₀₀₀
```

Each version represents one possible realization of the assumed model
uncertainty.

For this project:

``` text
Number of trials = 10,000
```

Each trial independently perturbs the selected model coefficients
within:

``` text
-30% to +30%
```

The controller remains the same.

The stability of each resulting closed-loop system is then checked.

### 8.2 Why do this instead of checking only +30% and -30%?

Because several coefficients are changed simultaneously. Different
combinations can interact in ways that are not obvious from checking
only one simple perturbation.

For example:

``` text
Trial 1:
A11 = -30%
A12 = +10%
A21 = -5%
...

Trial 2:
A11 = +25%
A12 = -28%
A21 = +19%
...

Trial 3:
A11 = -2%
A12 = +29%
A21 = -24%
...
```

Each trial represents a different combination of coefficient errors.

Monte Carlo sampling therefore explores a large collection of possible
uncertain models.

However, it does **not** test every mathematically possible combination.

That limitation is important and is discussed later.

------------------------------------------------------------------------

## 9. What happens inside one Monte Carlo trial?

One trial follows this sequence:

``` text
1. Start with nominal A and B
        ↓
2. Randomly perturb selected A coefficients by ±30%
        ↓
3. Randomly perturb selected B coefficients by ±30%
        ↓
4. Construct A_uncertain and B_uncertain
        ↓
5. Discretize the uncertain model at Ts = 0.008333 s
        ↓
6. Keep the original nominal DLQR gain K
        ↓
7. Calculate:
       Acl_uncertain = Ad_uncertain - Bd_uncertain K
        ↓
8. Calculate eigenvalues of Acl_uncertain
        ↓
9. Calculate the magnitude of every pole
        ↓
10. Check whether every magnitude is < 1
```

If all poles satisfy:

``` text
|pole| < 1
```

the trial is stable.

If at least one pole satisfies:

``` text
|pole| >= 1
```

the trial is classified as unstable.

------------------------------------------------------------------------

## 10. Why does \|pole\| \< 1 mean stable?

For a discrete-time linear system:

``` text
x(k+1) = Acl x(k)
```

the natural response is governed by the eigenvalues of `Acl`.

A pole with magnitude:

``` text
|λ| < 1
```

decays as time progresses.

A pole with:

``` text
|λ| = 1
```

does not decay.

A pole with:

``` text
|λ| > 1
```

grows with time.

Therefore the discrete-time stability condition is:

``` text
ALL closed-loop pole magnitudes < 1
```

The red dashed line at:

``` text
|pole| = 1
```

in the project plots is therefore the stability boundary.

------------------------------------------------------------------------

## 11. Final Monte Carlo result

The analysis was run for:

``` text
10,000 trials
```

with:

``` text
±30% coefficient uncertainty
```

The actual MATLAB result was:

``` text
Total trials:             10000
Stable trials:            10000
Unstable trials:          0
```

The resulting statistics were:

``` text
Minimum maximum-pole magnitude:
0.996410849834

Mean maximum-pole magnitude:
0.998460721361

Worst observed pole magnitude:
0.999007095691

Worst-case trial:
9869
```

Therefore:

``` text
Unstable trials = 0
```

and the requirement decision is:

# REQ-007: PASS

The MATLAB analysis explicitly reported:

``` text
REQ-007 DECISION: PASS
```

------------------------------------------------------------------------

## 12. Understanding "maximum pole magnitude"

There are four poles in every trial because the aircraft model has four
states.

For example, one hypothetical trial might produce:

``` text
Pole 1 = 0.78
Pole 2 = 0.9982
Pole 3 = 0.965
Pole 4 = 0.983
```

The magnitudes are:

``` text
0.78
0.9982
0.965
0.983
```

The largest value is:

``` text
0.9982
```

That is the **maximum pole magnitude** for that trial.

For every Monte Carlo trial, we save this largest magnitude.

After 10,000 trials we therefore have:

``` text
Trial 1      → maximum pole magnitude
Trial 2      → maximum pole magnitude
Trial 3      → maximum pole magnitude
...
Trial 10000  → maximum pole magnitude
```

These 10,000 values are what the two plots visualize.

------------------------------------------------------------------------

## 13. Plot 1 --- Pole magnitude across Monte Carlo trials

Place the first result image in the same GitHub directory as this
Markdown file using the filename:

``` text
REQ007_pole_magnitude_across_trials.png
```

Then this image will render on GitHub:

![REQ-007 pole magnitude across
trials](REQ007_pole_magnitude_across_trials.png)

### What does this graph show?

This graph answers:

> **For every uncertainty realization, how close did the worst
> closed-loop pole get to the instability boundary?**

### X-axis

``` text
Monte Carlo trial
```

The x-axis goes from:

``` text
0 → 10,000
```

Each x-axis position represents one uncertainty realization.

For example:

``` text
x = 1
```

means Monte Carlo trial 1.

``` text
x = 5000
```

means Monte Carlo trial 5000.

``` text
x = 10000
```

means Monte Carlo trial 10000.

There are therefore 10,000 trial results represented along the x-axis.

### Y-axis

``` text
Maximum closed-loop pole magnitude
```

For each trial, the analysis calculates four poles and then takes the
largest magnitude of those four poles.

That one number becomes the y-value for that trial.

For example:

``` text
Trial 100
    |
    +-- pole magnitudes:
    |      0.77
    |      0.9984
    |      0.965
    |      0.983
    |
    +-- maximum = 0.9984
```

So the plotted point at:

``` text
x = 100
y = 0.9984
```

means:

> In trial 100, the largest closed-loop pole magnitude was 0.9984.

------------------------------------------------------------------------

## 14. What is the red dashed line?

The red dashed horizontal line is:

``` text
|pole| = 1
```

This is the discrete-time stability boundary.

Therefore:

``` text
Below red line
      ↓
Stable trial

At red line
      ↓
Stability boundary

Above red line
      ↓
Unstable trial
```

In the actual result, all blue values remained below the red line.

The largest observed value was:

``` text
0.999007095691
```

which is still below:

``` text
1.0
```

Therefore no trial crossed the stability boundary.

------------------------------------------------------------------------

## 15. Why does Plot 1 look so close to 1?

This is an important observation.

The y-axis is zoomed into approximately:

``` text
0.996 → 1.000
```

The results cluster around:

``` text
0.9985
```

This happens because the dominant closed-loop pole of the nominal
controller is already close to 1.

The nominal value was:

``` text
0.998496051358
```

and the worst Monte Carlo realization was:

``` text
0.999007095691
```

So the uncertainty did not cause an instability in any of the 10,000
trials, but the dominant pole remains close to the boundary.

The correct interpretation is therefore not:

> "The system has huge stability margin."

The technically defensible interpretation is:

> "The controller remained stable for all 10,000 sampled ±30%
> coefficient uncertainty realizations, although the dominant
> discrete-time pole remained close to the unit-circle boundary."

------------------------------------------------------------------------

## 16. Plot 2 --- Distribution of maximum pole magnitude

Place the second result image in the same GitHub directory as this
Markdown file using the filename:

``` text
REQ007_pole_magnitude_distribution.png
```

Then this image will render on GitHub:

![REQ-007 pole magnitude
distribution](REQ007_pole_magnitude_distribution.png)

This graph answers a different question:

> **How frequently did different maximum pole magnitudes occur across
> the 10,000 trials?**

This is a histogram.

------------------------------------------------------------------------

## 17. What is a histogram?

A histogram groups numerical results into ranges called **bins**.

Suppose there were only ten results:

``` text
0.9981
0.9982
0.9982
0.9984
0.9984
0.9985
0.9985
0.9986
0.9987
0.9988
```

We could group them into ranges:

``` text
0.9980–0.9982
0.9982–0.9984
0.9984–0.9986
0.9986–0.9988
```

and count how many results fall into each range.

That creates a histogram.

The same concept is applied to all 10,000 Monte Carlo maximum-pole
results.

------------------------------------------------------------------------

## 18. Histogram X-axis

The x-axis is:

``` text
Maximum closed-loop pole magnitude
```

Unlike Plot 1, the x-axis here is **not trial number**.

Instead, it represents the numerical value of the maximum pole
magnitude.

For example:

``` text
x = 0.9985
```

means trials whose largest closed-loop pole magnitude was around 0.9985.

The red dashed line at:

``` text
x = 1
```

again represents the stability boundary.

------------------------------------------------------------------------

## 19. Histogram Y-axis

The y-axis is:

``` text
Number of trials
```

This is simply a count.

If a histogram bar reaches approximately:

``` text
800
```

that means approximately 800 of the 10,000 Monte Carlo trials produced a
maximum pole magnitude within that particular x-axis interval (bin).

Therefore:

``` text
Tall bar
    ↓
Many trials produced values in this range

Short bar
    ↓
Few trials produced values in this range
```

------------------------------------------------------------------------

## 20. What does the histogram tell us?

The histogram shows that most of the 10,000 trials produced maximum pole
magnitudes concentrated around approximately:

``` text
0.9985
```

The distribution remains to the left of:

``` text
1.0
```

which is the stability boundary.

The worst observed trial reached:

``` text
0.999007095691
```

but no sampled trial reached or exceeded 1.

Therefore the histogram provides a visual confirmation of the numerical
result:

``` text
unstable trials = 0
```

------------------------------------------------------------------------

## 21. Why are there two graphs?

The two graphs contain the same underlying 10,000 trial results, but
they answer different questions.

### Plot 1 --- Trial-by-trial

``` text
X = trial number
Y = maximum pole magnitude
```

It answers:

> "What happened in each Monte Carlo trial?"

It is useful for seeing whether any individual trial approaches or
crosses the stability boundary.

### Plot 2 --- Distribution

``` text
X = maximum pole magnitude
Y = number of trials
```

It answers:

> "Where are the results concentrated?"

It is useful for understanding the overall distribution of the
uncertainty experiment.

Using both is better than using only one.

------------------------------------------------------------------------

## 22. Why the result is a PASS

The decision rule is:

``` text
If any sampled uncertainty realization is unstable:
    FAIL

If zero sampled uncertainty realizations are unstable:
    PASS
```

The actual result was:

``` text
10,000 total trials
10,000 stable
0 unstable
```

Therefore:

``` text
REQ-007 = PASS
```

The worst observed value was:

``` text
0.999007095691
```

which satisfies:

``` text
0.999007095691 < 1
```

------------------------------------------------------------------------

## 23. What does this PASS actually prove?

The result demonstrates that:

> Under the specific uncertainty model used here, the nominal DLQR
> controller remained discrete-time stable for all 10,000 randomly
> sampled realizations in which the selected non-zero coefficients of
> the A and B matrices were independently perturbed by up to ±30%.

That is a meaningful numerical result for the defined experiment.

It does **not** mathematically prove stability for every possible
combination of all parameter uncertainties.

------------------------------------------------------------------------

## 24. Limitations of the Monte Carlo analysis

### 24.1 It does not test every possible model

There are infinitely many possible combinations of parameter values
within a continuous ±30% range.

10,000 samples are only a finite subset.

Therefore:

``` text
10,000 successful samples
```

does not mathematically mean:

``` text
every possible uncertain model is stable
```

------------------------------------------------------------------------

### 24.2 The uncertainty distribution is an assumption

The analysis uses independent random perturbations.

Real aircraft parameter errors may be correlated.

For example, several aerodynamic derivatives could be affected together
by:

-   changes in aerodynamic conditions
-   uncertainty in geometry
-   identification error
-   operating-point changes
-   coupling between estimated parameters

The current experiment does not model those correlations.

------------------------------------------------------------------------

### 24.3 Only A and B coefficient uncertainty is tested

This experiment does not directly include:

-   sensor noise
-   sensor bias
-   actuator saturation
-   actuator dynamics
-   computational delay
-   air-data errors
-   structural flexibility
-   nonlinear aerodynamic effects
-   atmospheric disturbances
-   changes in controller sample time

Those require separate analyses.

------------------------------------------------------------------------

### 24.4 The uncertainty is applied to the linearized model

The analysis uses the linear state-space model around the selected trim
condition.

Therefore this is primarily a **local linear robustness analysis**.

It does not prove stability over the entire nonlinear flight envelope.

------------------------------------------------------------------------

## 25. Why this analysis is still valuable

The engineering progression is:

``` text
Nominal model
    ↓
Design DLQR
    ↓
Check nominal poles
    ↓
Introduce model uncertainty
    ↓
Repeat closed-loop stability analysis
    ↓
Check whether poles cross |λ| = 1
```

This is more informative than showing only one successful nominal
simulation.

It demonstrates that the controller was examined against model
uncertainty rather than only against the exact model used to design it.

------------------------------------------------------------------------

## 26. Reproducibility

The MATLAB script uses:

``` matlab
rng(2026);
```

This fixes the random-number generator seed.

That is important because Monte Carlo analysis uses random numbers.

Without a fixed seed:

``` text
Run 1 → 10,000 random samples
Run 2 → different 10,000 samples
Run 3 → another 10,000 samples
```

With the fixed seed:

``` text
Run 1 → same sequence
Run 2 → same sequence
Run 3 → same sequence
```

This makes the reported experiment reproducible.

If the script is changed later, the seed should remain documented so
that the results can be regenerated.

------------------------------------------------------------------------

## 27. Evidence generated by the analysis

The analysis generated:

``` text
REQ007_30pct_uncertainty_results.mat
```

This MATLAB data file contains the analysis inputs and numerical
results.

Important stored quantities include:

``` text
A
B
Ad
Bd
K
Q
R
Ts
N
uncertainty
max_pole_magnitude
unstable_trials
unstable_count
stable_count
worst_pole
worst_trial
mean_max_pole
REQ007_RESULT
```

The command-window result reported:

``` text
REQ-007 DECISION: PASS
```

The command-window evidence is also preserved in the project
analysis-result PDF.

------------------------------------------------------------------------

## 28. Recommended repository structure

A clean GitHub structure is:

``` text
docs/
├── requirements.md
└── verification/
    └── REQ-007_30pct_model_uncertainty_analysis.md

analysis/
└── REQ007_30pct_uncertainty_analysis.m

results/
├── REQ007_30pct_uncertainty_results.mat
├── REQ007_pole_magnitude_across_trials.png
└── REQ007_pole_magnitude_distribution.png
```

The detailed Markdown file should live separately from
`requirements.md`.

`requirements.md` should remain a concise requirements traceability
table, while this document contains the methodology, equations, code
logic, plots, interpretation, limitations, and interview explanation.

------------------------------------------------------------------------

## 29. Interview explanation

A concise interview explanation would be:

> "For REQ-007, I performed a Monte Carlo robustness analysis of the
> C172P longitudinal DLQR controller. I independently perturbed the
> selected non-zero coefficients of the linearized A and B matrices by
> up to ±30%, generated 10,000 uncertain models, and retained the same
> nominal DLQR gain for every trial. For each realization I discretized
> the model at the controller sample time and checked the closed-loop
> eigenvalues. A trial was considered stable when all pole magnitudes
> were below one. All 10,000 trials were stable, with the worst observed
> maximum pole magnitude of 0.999007, so the defined REQ-007 analysis
> passed."

------------------------------------------------------------------------

## 30. If asked "Why Monte Carlo?"

A good answer is:

> "Because the uncertainty involves multiple model coefficients
> simultaneously. Instead of checking only one nominal model, Monte
> Carlo lets me explore many combinations of coefficient perturbations
> and see whether the closed-loop poles approach or cross the
> discrete-time stability boundary. I would not call it a formal
> robust-stability proof, because it samples a finite number of
> uncertainty realizations rather than exhaustively covering the
> uncertainty set."

------------------------------------------------------------------------

## 31. If asked "Why is the stability boundary 1?"

Answer:

> "Because the controller is implemented in discrete time. For a
> discrete-time linear system, stability requires all closed-loop
> eigenvalues to lie inside the unit circle in the z-plane, so the
> magnitude condition is \|λ\| \< 1."

------------------------------------------------------------------------

## 32. If asked "Why didn't you redesign K for every uncertainty?"

Answer:

> "Because the purpose was to test robustness of the controller I
> actually designed. I calculated the DLQR gain using the nominal model
> and held K fixed while perturbing A and B. Redesigning K for every
> uncertain model would answer a different question---whether I could
> redesign the controller after the model changed."

------------------------------------------------------------------------

## 33. If asked "Does PASS mean the controller is guaranteed stable?"

Answer:

> "No. It means it passed the defined 10,000-trial Monte Carlo analysis
> under independent ±30% perturbations of the selected non-zero A and B
> coefficients. Monte Carlo provides numerical evidence, not an
> exhaustive mathematical proof of robust stability."

This distinction should remain in the portfolio documentation.

------------------------------------------------------------------------

## 34. Final result

  Quantity                                                            Result
  --------------------------------------- ----------------------------------
  Requirement                                                        REQ-007
  Verification method                                               Analysis
  Model                                     C172P 4-state longitudinal model
  Controller                                             DLQR state feedback
  Sample time                                                     0.008333 s
  Uncertainty                                                           ±30%
  Perturbed A coefficients                                                13
  Perturbed B coefficients                                                 3
  Total trials                                                        10,000
  Stable trials                                                       10,000
  Unstable trials                                                          0
  Minimum maximum-pole magnitude                              0.996410849834
  Mean maximum-pole magnitude                                 0.998460721361
  Worst observed maximum-pole magnitude                       0.999007095691
  Worst-case trial                                                      9869
  Stability boundary                                                     1.0
  **REQ-007 result**                                                **PASS**

------------------------------------------------------------------------

## 35. Related project files

Keep these files together so that the analysis can be understood and
reproduced later:

``` text
REQ007_30pct_uncertainty_analysis.m
REQ007_30pct_uncertainty_results.mat
REQ007_pole_magnitude_across_trials.png
REQ007_pole_magnitude_distribution.png
REQ-007_30pct_model_uncertainty_analysis.md
```

The MATLAB script is the executable method, the `.mat` file is the
numerical result, the two PNG files are the visual evidence, and this
Markdown file explains the engineering reasoning behind all of them.
