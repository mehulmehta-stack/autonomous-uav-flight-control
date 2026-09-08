# Aerospace Software Standards Awareness

> **Portfolio scope note:** This document records standards and tool
> awareness for the C172P model-based flight-control project. It is
> **not a claim of DO-178C compliance or certification**. The project
> uses selected engineering practices inspired by safety-critical
> model-based development, but it does not implement the complete
> certification lifecycle.

## DO-178C / ED-12C --- 10-point summary

1.  **DO-178C is an assurance standard for airborne software.** It
    defines objectives for software life-cycle processes so that
    development and verification evidence can support an aircraft
    certification argument. It covers planning, development,
    verification, configuration management, quality assurance, and
    certification liaison.

2.  **The central idea is assurance, not simply testing.** A
    flight-software team must show that requirements, design,
    implementation, verification, and associated work products are
    developed systematically and that the applicable objectives are
    satisfied.

3.  **DAL is linked to the safety consequence of software failure.** The
    system safety/development process determines the applicable
    Development Assurance Level, and DO-178C assigns the corresponding
    software objectives and independence requirements.

4.  **DAL A corresponds to catastrophic failure conditions.** A software
    failure can contribute to loss of the aircraft or fatal
    consequences. DAL A has **71 objectives**, with **30 requiring
    independence**.

5.  **DAL B corresponds to hazardous/severe failure conditions.** A
    failure can significantly reduce safety margins or crew capability
    and may contribute to serious injury or fatalities. DAL B has **69
    objectives**, with **18 requiring independence**.

6.  **DAL C corresponds to major failure conditions.** A failure can
    significantly reduce safety margins or increase crew workload, but
    consequences are less severe than DAL A/B. DAL C has **62
    objectives**, with **5 requiring independence**.

7.  **DAL D corresponds to minor failure conditions.** Failures have
    comparatively limited safety impact. DAL D has **26 objectives**,
    with **2 requiring independence**.

8.  **DAL E means no safety effect.** If software failure has no effect
    on aircraft operation, safety, or crew workload, it is DAL E and has
    **0 objectives**. DAL is therefore an assurance-rigor
    classification, not a generic software-quality score.

9.  **Independence is a deliberate verification principle.** Where an
    objective requires independence, verification is performed
    independently from the person who produced the item being verified.
    This reduces the chance that development and verification reproduce
    the same error.

10. **For the C172P project, the key lesson is traceability and
    evidence.** The project already uses
    `requirements → model → analysis/simulation → result → PASS/FAIL evidence`.
    That is directionally similar to safety-critical engineering, but it
    is only a subset of a real DO-178C lifecycle. The project must be
    described as **DO-178C-aware**, not DO-178C-compliant.

### DAL quick reference

  -----------------------------------------------------------------------------
  DAL             Failure-condition             Objectives          Independent
                  consequence                                        objectives
  --------------- ------------------- -------------------- --------------------
  A               Catastrophic                          71                   30

  B               Hazardous / severe                    69                   18

  C               Major                                 62                    5

  D               Minor                                 26                    2

  E               No safety effect                       0                    0
  -----------------------------------------------------------------------------

### Why flight software needs DO-178C

Aircraft software can command flight controls, manage propulsion,
process critical information, and influence decisions that affect
continued safe flight. A normal software process may demonstrate that
tested cases work, but safety-critical development also needs systematic
evidence that requirements were implemented correctly, verification is
adequate, changes are controlled, and the software is appropriate for
its safety role. DO-178C provides the objective-based framework for that
assurance argument.

DO-178C also has supplements relevant to particular techniques:
**DO-331** for model-based development and verification, **DO-332** for
object-oriented techniques, **DO-333** for formal methods, and
**DO-330** for software-tool qualification considerations.

## Ansys SCADE Suite

**Ansys SCADE Suite** is a model-based environment for developing
critical embedded software using graphical and textual modeling. Its
SCADE language provides a formally defined representation of software
behavior, including data-flow and state-machine structures, rather than
making manually written C the primary design artifact. The model can be
simulated, analyzed, verified, documented, and used as the source for
automatic code generation. Ansys states that the SCADE Suite KCG C/Ada
code generator is qualified as a development tool for **DO-178C/DO-330
at TQL-1**.

### How SCADE supports DO-178C-oriented development

The important point is not simply "draw diagrams and generate C." The
safety-oriented advantage comes from making the model a controlled
engineering artifact and reducing manually rewritten implementation.

``` text
Software requirements
        ↓
SCADE model
        ↓
Model analysis / simulation / verification
        ↓
Qualified KCG code generation
        ↓
C or Ada implementation
        ↓
Target integration + verification
        ↓
Certification evidence
```

SCADE's model-based approach supports automatic code generation, model
verification, traceability, documentation, and reduced
coding/verification effort. Its qualified KCG generator can provide a
tool-qualification basis for relying on aspects of generated source-code
correctness within the applicable certification process.

### Why use SCADE instead of making hand-written C the primary artifact?

The engineering argument is not that "SCADE writes better C than
humans." It is:

``` text
Hand-written implementation
    ↓
more manual translation from design to source
    ↓
more opportunities for implementation mismatch
    ↓
more review/verification effort

versus

Controlled model
    ↓
model verification
    ↓
qualified automatic code generation
    ↓
less manual coding/translation
    ↓
more direct traceability between design and implementation
```

This does **not** eliminate verification. It changes where effort is
spent and can reduce manual coding, improve analyzability and
traceability, and provide a qualified code-generation process.

### Airbus connection

Public Ansys material reports direct Airbus use of SCADE. Ansys reports
that Airbus began using SCADE for an HMI platform in 2015 after earlier
projects had used handwritten code. Airbus personnel quoted by Ansys
described SCADE as supporting automatic code generation, standards
compliance, interoperability, and reduced safety-certification effort.

The portfolio-safe statement is:

> **Airbus uses SCADE in safety-critical software development because it
> provides a model-based environment designed for critical systems,
> qualified/qualifiable code generation, verification support,
> traceability, and integration with safety-oriented workflows.**

Do not say "Airbus never writes C" or "SCADE automatically makes
software DO-178C compliant." Neither claim is justified.

### Connection to the C172P project

The current project follows the same broad MBD philosophy:

``` text
requirements
    ↓
Simulink controller model
    ↓
MIL
    ↓
generated C/C++
    ↓
SIL
    ↓
JSBSim nonlinear plant
    ↓
verification
```

The difference is assurance depth. This project does not have a
certified SCADE toolchain, formal certification plans, independence
requirements, structural-coverage evidence, configuration-management
system, or certification liaison.

## Simulink Model Advisor --- 5-point summary

1.  **Model Advisor is a model-quality and compliance-checking
    mechanism.** It checks a Simulink model or subsystem for selected
    conditions and configuration settings that can lead to inaccurate or
    inefficient simulation or violate selected modeling guidelines.

2.  **The checks cover more than visual appearance.** Depending on
    installed products and selected check sets, Model Advisor can
    inspect unconnected lines/ports, model diagnostics, block usage,
    signal properties, configuration settings, naming/modeling
    conventions, data-flow practices, and other model-quality
    conditions.

3.  **It can be used for high-integrity and DO-178C/DO-331-oriented
    modeling guidance.** With Simulink Check, MathWorks provides
    High-Integrity System Modeling Guidelines and Model Advisor checks
    associated with DO-178C/DO-331. These guidelines cover model
    settings, block usage, block parameters, requirements
    considerations, and practices intended to make models complete,
    unambiguous, deterministic, robust, and verifiable.

4.  **A Model Advisor PASS is evidence about selected checks, not
    certification.** Model Advisor only runs checks available through
    installed products/licenses and the selected configuration. Passing
    those checks does not establish aircraft safety or DO-178C
    compliance.

5.  **For this C172P project, the useful finding is that the Simulink
    model itself is an engineering artifact.** Before relying on the
    model for code generation, we should examine connectivity,
    signal/data properties, configuration, modeling patterns,
    requirements links, and applicable high-integrity rules. The actual
    pass/fail counts must come from the local Model Advisor execution.

### Current C172P Model Advisor status

**NOT YET EXECUTED IN THIS CHAT ENVIRONMENT.**

The authoritative model is:

``` text
JSBSim_DLQR_PID_OuterLoop_AltitudeHold_c172p(2).slx
```

The project documentation identifies this SLX as the current wiring
reference. No MATLAB/Simulink runtime is available in this environment,
so no pass/fail count is invented here.

MathWorks provides the `ModelAdvisor.run` API for programmatic execution
and report generation. The supplied helper script below runs the
configuration available in the local MATLAB installation and saves the
results.

## Polyspace --- 5-point summary

1.  **Polyspace is static-analysis software for C/C++ implementation
    code.** It is relevant after or around code generation because it
    analyzes the implementation rather than the Simulink block diagram.

2.  **Polyspace Bug Finder focuses on defects, coding-rule violations,
    and code metrics.** MathWorks describes it as using static analysis
    based on abstract interpretation and reporting more than 350 types
    of coding defects, including buffer overflows, divide-by-zero,
    concurrency issues, and other security/coding problems. It also
    supports rules such as MISRA C/C++, AUTOSAR C++14, CERT C/C++, and
    CWE-related rules.

3.  **Polyspace Code Prover goes further toward formal absence-of-error
    analysis.** It analyzes code paths against possible inputs using
    abstract interpretation and attempts to prove the presence or
    absence of specified run-time errors without executing the program,
    instrumenting the code, or requiring test cases.

4.  **Code Prover checks important run-time safety properties.**
    Examples include arithmetic overflow, division by zero,
    out-of-bounds array access, pointer/memory-access problems,
    control-flow problems, data-flow problems, and certain C++ issues.
    Results distinguish proven-safe, definite-error, unreachable, and
    unproven/inconclusive cases.

5.  **For this C172P project, Polyspace is awareness-level rather than a
    claimed execution result.** The next logical place for Polyspace
    would be the generated C/C++ controller implementation. I should not
    claim to have run Polyspace unless an actual analysis report exists.

## Model Advisor vs Polyspace

``` text
              REQUIREMENTS
                   ↓
            Simulink MODEL
                   ↓
          ┌─────────────────┐
          │  MODEL ADVISOR  │
          └─────────────────┘
                   ↓
          Generated C/C++
                   ↓
          ┌─────────────────┐
          │    POLYSPACE    │
          └─────────────────┘
                   ↓
          Target implementation
```

**Model Advisor asks:**

> Is my model constructed and configured according to the selected
> modeling rules and guidelines?

**Polyspace Bug Finder asks:**

> Does my C/C++ implementation contain detectable coding defects or rule
> violations?

**Polyspace Code Prover asks:**

> Can I prove the absence of specified run-time errors over the analyzed
> code paths and input ranges?

These are different questions. Neither tool alone proves DO-178C
compliance.

## Why static analysis matters when we already test

Testing demonstrates behavior for selected test cases. Static analysis
can examine code paths and value ranges without requiring every path to
be exercised by a test.

For example, a simulation might test:

``` text
x = 10
x = 5
x = 2
x = 1
```

and never encounter:

``` text
x = 0
```

for a division operation.

A static-analysis tool can examine whether a divide-by-zero condition is
possible along analyzed paths. Similar reasoning applies to array
bounds, overflow, pointer access, and other coding defects.

This is why safety-oriented software verification uses multiple
complementary techniques.

## How the tools fit the C172P MBD pipeline

``` text
Requirements
     ↓
Simulink controller model
     ↓
Model Advisor
     ↓
MIL
     ↓
Simulink Coder
     ↓
Generated C/C++
     ↓
Polyspace Bug Finder / Code Prover
     ↓
SIL
     ↓
JSBSim nonlinear plant
     ↓
Verification evidence
```

The current project already has real evidence for several downstream
activities:

-   MIL/SIL comparison
-   generated-code SIL verification
-   discrete-time pole analysis
-   ±30% Monte Carlo robustness analysis
-   gain-margin analysis
-   phase-margin analysis

The standards-awareness work adds understanding of the professional
safety-oriented tooling around these activities.

## Interview-safe wording

> "I understand DO-178C as an objective-based assurance standard for
> airborne software, with the required rigor determined by the
> software's Development Assurance Level. I studied DAL A through E,
> SCADE's role in model-based safety-critical development and qualified
> code generation, Model Advisor for model-level quality and
> high-integrity guideline checks, and Polyspace for static analysis of
> generated or handwritten C/C++. In my C172P project I implemented
> requirements-based analysis, MIL/SIL verification and robustness
> analysis, but I would not describe the student project as DO-178C
> compliant because I have not implemented the complete certification
> lifecycle."

## Project-specific connection

The strongest connection to the current C172P project is:

``` text
Your current engineering work
        ↓
requirements
        ↓
Simulink controller
        ↓
MIL
        ↓
generated C/C++
        ↓
SIL
        ↓
JSBSim
        ↓
verification
```

The professional safety-critical extension is:

``` text
requirements
        ↓
controlled model
        ↓
model-level checks
        ↓
qualified/controlled code generation
        ↓
source-code static analysis
        ↓
requirements-based verification
        ↓
coverage / independence / configuration evidence
        ↓
certification argument
```

The difference is **assurance depth and process discipline**, not simply
which software tool is opened.

## Sources consulted

-   Ansys --- *What is DO-178C?*
-   Ansys --- *Ansys SCADE Suite*
-   Ansys --- *Efficient Development of Safe Avionics Software with
    DO-178C Objectives Using SCADE Suite*
-   Ansys --- *Drone Safety Takes Flight at Airbus, Supported by Ansys
    SCADE*
-   Ansys --- *Ansys Supports Smarter In-Cabin Communications for
    Airbus*
-   MathWorks --- *Model Advisor Checks for High-Integrity Systems
    Modeling Guidelines*
-   MathWorks --- *High-Integrity System Modeling*
-   MathWorks --- *Run Model Advisor Checks*
-   MathWorks --- *ModelAdvisor.run*
-   MathWorks --- *Save and View Model Advisor Check Reports*
-   MathWorks --- *Polyspace Bug Finder*
-   MathWorks --- *Polyspace Code Prover*
-   MathWorks --- *Polyspace Code Prover Run-Time Checks*
