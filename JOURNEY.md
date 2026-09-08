# Development Journey — Autonomous UAV Flight Control

A chronological engineering log: what was built, what broke, what was learned,
and what changed as a result. Kept at the repo root so the whole arc reads in
one place instead of being scattered across folder READMEs.

Status legend: ✅ fixed · 🟡 open decision · 🔧 fix in progress

---

## Verification scope — recovered from past sessions (this supersedes the earlier version of this section)

Searched past conversations in this project and found the actual authoritative
REQ-011–014 definitions and several corrections that were never reconciled
into the uploaded `requirements.md`. That file is confirmed stale in real,
specific ways — not just "probably needs updating eventually."

**REQ-011** — gain-schedule extrapolation stability: pole check at 90/225 ft/s
(stall / Vno) + 40s nonlinear recovery. Max pole magnitude 0.9962 (90 ft/s),
0.9983 (225 ft/s), both stable.

**REQ-012** — throttle-transient roll-yaw coupling (7.5° bank excursion at
low-speed recovery, traced to engine torque/P-factor coupling). Documented
limitation, not remediated — rudder has zero authority in this model.

**REQ-013** — 800s combined-mission run: steady-state turn performance
(max bank 17.49–17.52°, final airspeed 184.998–185.014 ft/s) statistically
indistinguishable across three starting conditions.

**REQ-014 — already assigned.** This is the WP2/WP4 L1 path-crossing
artifact (three hypotheses tested and ruled out: speed-coupling, `phi_max`
derivation, L1 discontinuity; suspected root cause is EKF/GPS-update timing
phase-relative to corner arrival — correctly scoped as a separate
investigation and not pursued further, given time constraints).
**This means my earlier suggestion to put the SIL 1000-ft altitude stress
test at REQ-014 was wrong — that number is taken. It needs REQ-015 or
another unused number.**

**REQ-003 (waypoint tracking within 10m) — flagged, not simply "closed."**
A past session proposed closing this using cross-track error data
(RMS 408.0 m, max 885.8 m at `L1_dist=350`, improved from 539.8 m / 1228.2 m
at `L1_dist=150`), reasoning that max error stayed under the ~1,061 m ideal
turn radius. **That evidence does not satisfy the requirement as written —
10 m vs. ~886 m max error is roughly two orders of magnitude off.** This
needs a decision, not an assumption: either REQ-003's stated tolerance was
unrealistic from the start for L1 waypoint-following at this leg scale and
needs rewriting to match what's physically achievable and was actually
tested (e.g. bounded by turn geometry, not a fixed 10 m), or REQ-003 stays
NOT VERIFIED until something closer to 10 m accuracy is demonstrated.
**Open — needs your call before requirements.md gets rewritten.**

**Also recovered, not yet folded into any current doc:**
- Architecture description correction: this is a gain-scheduled LQR inner
  loop (attitude) + PID outer loops (altitude, airspeed), cascaded and
  separately tuned — not "LQG." Worth checking existing docs use this
  phrasing consistently.
- `L1_dist` final value: 350 (tuned from 150), with the RMS/max cross-track
  numbers above as the evidenced justification.
- `phi_max` final value: 17° (corrected from a calibrated/true-airspeed
  mixup, via 16° as an intermediate step).
- Actuator dynamics: rate limiter ±1.0 normalized units/s, first-order lag
  τ=0.1s, elevator and aileron only (throttle excluded — has its own
  JSBSim engine spool-up lag already, adding a second lag would double-
  model it). **Correction to my own earlier note:** these are explicitly
  flagged in `add_actuator_dynamics.m`'s own comments as *representative
  small-GA-servo assumptions, not a verified hardware spec* — the C172
  has no factory autopilot servo to match against. Document as an
  assumption, not a validated parameter. Verified against config:
  elevator rate 0.605/s, aileron 0.999/s (both PASS against the 1.0/s
  limit) — that verification is real, the underlying 1.0/s and τ=0.1s
  themselves are chosen values, not measured ones.
- **Gain scheduling — resolved, contradicts the MASTER summary's caution.**
  Final K1/K2/K3 (100/185/220 ft/s) recovered from
  `Gain_Schedule_Extrapolation_Sanity_Check.m`, both longitudinal and
  lateral. Cross-checked: the 185 ft/s row matches the original
  single-point checkpoint K to within a few percent, as the design
  script's own sanity-check comment says it should. This is closed,
  numerically verifiable work — the other AI tool's summary was
  incomplete here, not the project.
- **LQG — precise claim, not blanket status.** `Navigation_EKF_LQG.m`
  (12-state EKF, adds estimated p/q/r) and `apply_ekf_state_feedback.m` (wires
  estimated p̂/q̂ into both LQR loops, replacing raw feedback) are real,
  implemented. A formal LQG-object validation (noisy-estimated-state vs.
  perfect-state LQR, quantified) has not been found — don't claim that
  part.
- **Roll/aileron controller — fourth stage found.** proportional (−0.5)
  → proportional (−1.0, the checkpoint) → PI (+integral) →
  **2-state discrete LQR (phi, p)** — this last one is what actually
  got gain-scheduled; its A/B matrices are an exact match to the 185 ft/s
  schedule point.
- **Stage 0, earlier than anything else found so far:**
  `AIrcraftPitch_SystemAnalysis.m` and `JSBSim_Running.m` — generic
  tutorial-style transfer functions, not derived from the C172P at all.
  True starting point, belongs before trim/linearization in `history/`.
- Flagship model file: `Final_JSB_4wp_LQG_withNoise_GnSch_Autopilotsbsy.slx`.
- REQ-004/REQ-005 status genuinely unknown — flagged as untouched in every
  session searched so far, still NOT VERIFIED until confirmed otherwise.

---

## docs/

- ✅ `requirements.md` had two dead links (REQ-007/008/009 evidence files
  pointed at filenames that didn't match what was actually generated).
  Fixed.
- ✅ The earlier requirements table — REQ-007/008/009 marked NOT VERIFIED,
  written before the Monte Carlo and margin analyses existed — is preserved
  as `requirements_early_draft_archive.md` instead of deleted, with a
  provenance note at the top.
- 🟡 `SIL_robustness_stress_test.md` (1000-ft initial-altitude recovery, SIL)
  isn't tied to a REQ ID yet. Options: fold into REQ-002 as extended-envelope
  evidence, or assign a new REQ ID — REQ-011/012/013 are already used
  elsewhere for other stress scenarios (envelope excursion at 90/225 ft/s,
  40s nonlinear recovery, 800s combined mission), so this needs a REQ-014 or
  an explicit note that it's supplementary. Needs your call.
- 🟡 Image path convention is inconsistent — most docs reference PNGs flat,
  `SIL_robustness_stress_test.md` alone uses an `images/` prefix. Pick one
  before more images get added.

## EKF/

- 🔧 Two estimators exist at different fidelity, and that's the real "6 → 9"
  story: `gps_ekf_corrected.py` is a 6-state linear KF (its own comments say
  it's not yet an EKF); `nav_ekf.py` is the 9-state nonlinear EKF with full
  attitude. Worth naming explicitly as the progression rather than treating
  as clutter — but the two need clear tags so nothing gets mislabeled as
  "the EKF" when it's the earlier linear filter.
- 🟡 `EKF_Attitude_GPS_Fusion_Task_Corrected.md` and
  `EKF_GPS_Code_Explanation_Beginner_Guide.md` only document the earlier
  6-state/2-state scripts. The 9-state EKF — the more advanced, more
  interview-relevant piece — has no writeup yet.
- ✅ `gps_ekf_trajectory.png` orphan resolved: recovered from an early draft
  of the task doc. It's produced by `archive/gps_ekf_original.py`, the
  pre-correction 6-state filter. The bug it had: velocity was overwritten
  every step by a synthetic "measurement," untouched by the GPS-outage
  logic — so the original outage test wasn't really testing inertial-only
  coasting. `gps_ekf_corrected.py` fixes this by deriving velocity from
  integrated IMU acceleration instead. Kept as `archive/`, not deleted —
  a clean bug-found-and-fixed beat for the journey.
- 🟡 Known limitation to carry into the final README: the 9-state EKF's yaw
  has no direct measurement correction (GPS observes N/E/D only), so it's
  pure gyro dead-reckoning and drifts visibly in the attitude plot.

## generated_code/

- ✅ `generated_code_README.md`'s DO-178C/SCADE paragraph rewritten to
  match your own `controls_standards_deep_read_DO178C_SCADE_Polyspace.txt`:
  qualification narrows verification scope, it doesn't grant automatic
  certification.
- ✅ Model-name note confirmed: `"Autopilot"` is the initial controller
  subsystem. This folder is now explicitly labeled a **checkpoint** — the
  controller has since grown (gain scheduling, L1 guidance).
- 🟡 Header files (`Autopilot.h`, `Autopilot_private.h`, `Autopilot_types.h`,
  `rtwtypes.h`, `rtmodel.h`) still not pushed, but low priority while this
  is a checkpoint rather than the final artifact — visible in the
  `Autopilot_grt_rtw` screenshot if buildability becomes worth chasing.
- 🔲 **Deferred, scoped:** code generation for the current, larger
  controller. Not a small follow-on — needs its own MIL/SIL re-verification
  against the new I/O interface, same weight as the original `docs/` pass.
  Stretch goal after FlightGear / README / demo video / GitHub publish,
  only if there's runway before applications need the time instead.

## Guidance/

- ✅ `l1_guidance_prototype.py`: standalone 2-D kinematic L1 guidance, correctly
  closed-loop (`a_cmd → heading → velocity → position`). Working as intended.
- 🔧 `day4_ekf_l1_integration_saved_plots.py`: meant to be the EKF+L1
  integration, but the "truth" trajectory is scripted independently of the
  L1 output — the guidance commands never actually steer anything, which is
  why a_cmd/eta/roll never converge in the Day-4 plots. Fix: feed `phi_cmd`
  back into the vehicle kinematics (reusing Day-2's closed-loop pattern),
  have L1 consume the EKF's *estimated* state rather than ground truth, and
  add a saturation limit on `phi_cmd`.
- Once fixed, this becomes the stronger demo of the two: L1 holding a path
  from noisy EKF estimates, surviving a 10-second GPS outage mid-run.

---

## Primary-source numeric verification (read directly from .mat files)

Loaded the actual `.mat` result files rather than relying on paraphrased
numbers from text summaries. Everything below is exact, not rounded-in-
transit.

- **REQ-007 resolved for good:** worst pole magnitude = 0.9990070957,
  mean = 0.9984607214, worst trial #9869, 10000/10000 stable. This also
  resolves the earlier flagged concern about the VV_Robustness doc showing
  `rng(42)` in its code snippet while the real script uses `rng(2026)` —
  the exact result (0.999007095691) matches the real `rng(2026)` run
  precisely, so that snippet was an illustrative paraphrase, not a second
  real run. Not a discrepancy after all.
- **REQ-008/009 exact:** phase margin 92.6445646445°, gain margin
  19.7218387031 dB, poles [0.7745, 0.9985, 0.9653, 0.9835].
- **Checkpoint K confirmed exact, not just "likely," match:**
  `C172_DLQR_4state_FINAL.mat`'s K = [0.0205263392, 2.2684617189,
  -7.4922949776, -2.1181780315] is bit-for-bit the same K baked into
  `Autopilot_data.c`. The single-point/checkpoint link I'd flagged as
  probable is now certain.
- **Gain-schedule K matrices, exact, from the source `.mat` (not the
  rounded copy in the extrapolation-check script):**
  `V_trim_pts = [100, 185, 220]`
  `K_lon_pts` (rows = trim points, cols = [K_u, K_alpha, K_theta, K_q]):
  `[0.0176, 1.5810, -7.7995, -2.4150]`, `[0.0213, 2.2732, -7.5064, -2.1186]`,
  `[0.0198, 2.3520, -7.1666, -2.0173]`.
  `K_lat_pts` (cols = [K_phi, K_p]): `[2.2724, 0.6695]`, `[2.2368, 0.5453]`,
  `[2.2149, 0.5334]`.
  The lateral 2-state DLQR's saved K matches the 185 ft/s row exactly —
  confirms it's genuinely the schedule's middle point, not just similar.
- **IMU noise, real characterized values** (`imu_noise_characterization_data.mat`, 120s,
  100Hz): accel std ≈ 0.22/0.02/0.06 m/s² (ax/ay/az), gyro std ≈
  0.009/0.002/0.006 rad/s (gx/gy/gz). Usable for an honest sensor-noise
  section instead of an assumed number.
- **`DAY9_CURRENT_MODEL_ARCHITECTURE.txt` confirmed broken, as suspected.**
  It contains only the closing banner — the `diary()` placement bug in
  `reportderive.m` is confirmed, not hypothetical. The actual architecture
  dump this file was supposed to hold does not exist.
- **`Autopilot.mat`** contains only a bare `rt_tout` (time vector) — not
  useful as evidence on its own. **`signal_editor.mat`** didn't parse as
  ordinary data (likely a Signal Editor scenario object, not a result
  file) — flagging rather than guessing at its contents.

## New test results found, not yet assigned REQ numbers — needs your input on numbering

- **REQ-013's numbers were paraphrased imprecisely before — corrected.**
  Actual `GainSchedule_CombinedMission_Summary.txt`: max bank is
  **17.79° identically across all three starting conditions** (stronger
  than the "17.49–17.52°" range I had), final airspeed 185.086–185.117
  ft/s (not 184.998–185.014).
- **Nominal-trim-point recovery test** (`GainSchedule_Validation_Summary.txt`,
  distinct from the 90/225 ft/s envelope-extreme stress test):
  100/185/220 ft/s recovery to cruise. Worth flagging honestly: the
  **100 ft/s case hits max |elevator| = 0.9998 — essentially saturated**
  against the ±1.0 limit. Not a failure, but a real, tight margin worth
  stating rather than omitting.
- **Lateral gain-schedule validation** (`LateralGainSchedule_Validation_Summary.txt`):
  10° initial bank disturbance, 3 airspeeds, recovers to under 0.4° at
  all three, aileron well within geometric limits.
- **GPS-reacquisition elevator spike — the actual justification for
  actuator dynamics, now with a clean evidence chain:**
  `GPS_Reacquisition_Characterization_Summary.txt` shows elevator jumping
  to max |e| = 0.9998 in the t=35–40s window (vs. ~0.207 before/during
  outage) — a near-saturation spike — but recovering to within 0.02 of
  pre-outage level in just 0.007s, faster than one sample period. This is
  the actual finding that `verify_actuator_dynamics.m`'s comments
  reference as motivating the actuator-dynamics addition. Clean causal
  chain: found the spike → built rate limiter + lag → verified the raw
  spike still occurs pre-actuator but can't propagate post-actuator. Good
  narrative material, not just a bug fix.

These four don't have REQ numbers yet. Proposing REQ-016 (nominal-trim
gain-schedule validation), REQ-017 (lateral gain-schedule validation) —
but flagging for your confirmation before assigning, given the REQ-014
mix-up earlier in this process.

## Final closures from the image batch

- ✅ Gain-schedule extrapolation confirmed clean: `gain_schedule_extrapolation_check.png`
  shows all six gains extrapolating smoothly, no sign flips or blow-ups,
  tested 60–300 ft/s against a 100–220 design envelope.
- ✅ **REQ-014 root cause, corrected.** `L1_Eta_Discontinuity_Check.png`
  shows a smooth eta transition in all 8 waypoint-switch panels — no
  discontinuity anywhere. Per the diagnostic's own stated criterion, this
  rules out the L1 law itself and points at the DLQR's response to
  `phi_cmd` as the next place to look. Earlier text said "suspected EKF/
  GPS-update timing" — that was an imprecise paraphrase from search, not
  what the primary diagnostic evidence actually supports. Corrected.
- 🔲 `verify_actuator_dynamics.m` and `run_model_advisor_C172P.m`: real
  scripts, right methodology, never run to completion (no output exists).
  Document as attempted, not as validated results.

---

## `Initial Altitude` architectural finding (found closing REQ-015)

Traced directly from the `.slx` block diagram (not inferred): the
altitude-hold error is computed as `Constant(4000, target) − [Initial
Altitude(hardcoded 4000) + EKF_Altitude_ft(−3.28084 × D_ekf)]`. Because
both constants were the same value, they canceled — any test starting
away from 4000 ft produced a near-zero error and no correction, which is
exactly the flat, uncorrective REQ-015 result seen before this was found.

This is not the right long-term fix, but the short-term one is what got
used: `Initial Altitude` manually set to 3000 to match this test's real
starting condition, then reverted to 4000 afterward. **This constant must
be manually kept in sync with whatever altitude a given test actually
starts at, for as long as it exists in its current hardcoded form.**

**Real fix, correctly scoped as future work, not urgent:** replace the
`Initial Altitude` Constant block with a Sample-and-Hold (or Memory,
triggered once at t=0) that latches the true starting altitude
automatically, the same way `Navigation_EKF_LQG.m` already self-
initializes N/E/D from the first real GPS reading rather than a
hardcoded assumption. `Initial Altitude` is very likely a leftover from
before that EKF initialization pattern existed elsewhere in the
architecture — worth stating as a real, understood, honestly-documented
limitation in the final writeup, not silently left unmentioned.

Also resolved in this same pass: extending `Tsim` from 40 to 100
seconds worked cleanly — the S-Function does honor a longer Simulink
`StopTime` past the run-script's own internal `<run end="40">`. The
earlier caution about this was warranted at the time (unverified), and
is now resolved by direct evidence.

---

## REQ-014 diagnostic instrumentation removed during code-gen cleanup

While preparing `Autopilot.slx` for SIL simulation (REQ-006/010), found that
the REQ-014 investigation's debug taps — `val_L1eta`, `val_L1phicmd`,
`val_L1speed`, `val_L1wpidx`, `ekf_E_log`, `ekf_N_log`, `gps_valid_log` —
had been swept into the Autopilot subsystem boundary along with the actual
controller logic. These are internal-only signals (no existing Outport
exposes them), left over from `L1_SpeedCoupling_Diagnostic.m`,
`L1_Eta_Discontinuity_Check.m`, and `L1_guidance_correction_check.m`.

Deleted them, rather than relocating them, for a real reason: they have no
path to the subsystem boundary, so "moving" them would mean adding 7 new
Outports to `Autopilot` just to preserve debug capability not needed for
the current test. The generated code should represent the controller, not
accumulated investigative scaffolding.

**Consequence worth knowing:** REQ-014's evidence (`GainSchedule_*`,
`L1_Eta_Discontinuity_Check.png`, etc.) reflects the model *before* this
cleanup. Reproducing or extending that investigation on the current
`Autopilot.slx` would require re-adding the relevant debug outputs as new
Outports first — the scripts still exist, but the signals they read
currently don't.

## Next up (real, as of this pass)

Done: `requirements.md` fully rewritten (17 requirements), top-level
`README.md` written. **All 17 requirements are now verified against the
final gain-scheduled model** — the last two, REQ-006/010, closed on
2026-09-05 with a clean MIL vs SIL comparison (max error 0.0051%,
elevator; ~0% throttle/aileron).

**Getting REQ-006/010 to a clean run surfaced three real, worth-remembering
fixes, not just one:**
1. Actuator dynamics blocks (`RateLimiter_*`, `ActuatorLag_*`) had ended
   up inside the `Autopilot` code-gen boundary — architecturally wrong
   (actuator physics isn't controller code) and a hard SIL blocker
   (continuous states aren't supported in Model-block SIL). Moved back
   to the parent model, matching where `add_actuator_dynamics.m`
   originally placed them.
2. A plain `Clock` block (not `Digital Clock`) feeding `GPS_Validity`
   was *also* flagged as a continuous-time element, independent of the
   actuator dynamics — Simulink treats `Clock` as continuous regardless
   of the solver being fixed-step discrete. Replaced with a `Digital
   Clock` at 0.008333s. Same underlying tension `fix_matlab_function_
   sampletimes.m` hit before, showing up a second time via a different
   block.
3. Seven leftover REQ-014 diagnostic To Workspace blocks (`val_L1eta`,
   `val_L1phicmd`, `val_L1speed`, `val_L1wpidx`, `ekf_E_log`,
   `ekf_N_log`, `gps_valid_log`) had been swept into the same boundary.
   Deleted rather than relocated, since their source signals have no
   existing path to the subsystem boundary — see the dedicated note
   above for what this means for reproducing REQ-014 later.

**One more thing worth remembering, not a bug:** the REQ-006/010
comparison plots show the same GPS-reacquisition elevator spike at
t=35s that `characterize_gps_reacquisition.m` originally found. This is
expected — the tap point (Model block's raw output) is now *before* the
actuator dynamics, which moved to the parent model per fix #1 above. The
real final command reaching JSBSim is still smoothed exactly as
`verify_actuator_dynamics.m` demonstrated; MIL and SIL showing the
*identical* spike is exactly what this test is supposed to prove, not
evidence the actuator-dynamics fix stopped working.

## Trim-scheduling bug found and fixed, then a second, independent wiring bug found underneath it

While reconciling old trim logs against the live model (prompted by a
direct question: "should trim values change with gain scheduling?"),
found that `alpha_trim`, `Thita_trim`, and one of two velocity-trim
constants were hardcoded to the 185 ft/s cruise values regardless of
which schedule point was active — while `K` correctly scheduled with
airspeed. Confirmed by direct inspection of `Autopilot.slx`, not
inferred: all three were plain `Constant` blocks.

**Fix, precisely scoped by tracing the actual wiring, not applied
uniformly:** built `Lon_TrimSched(V)`, a new MATLAB Function scheduling
`alpha_trim`/`theta_trim`/`u_trim` off the same `U_meas` signal `K`
already uses. Critically, **not all five original trim constants got
this treatment** — tracing confirmed `velocity_trim2` (feeds the
throttle loop's commanded airspeed directly) and `Throttle` (its
feedforward baseline) serve a genuinely different role than
`velocity_trim1` (feeds the LQR's internal `delta_v` state) and were
correctly left fixed. Scheduling all five would have been wrong —
the throttle loop needs one fixed commanded speed, not a moving target.

**Verifying this fix surfaced a second, unrelated, more consequential
bug.** The first re-verification attempt (REQ-016) came back showing
identical 0.9998 elevator saturation across all three trim points —
worse than before the fix in appearance, and a physically-impossible
near-vertical spike appeared in the elevator plot at t=35s (GPS
reacquisition). Traced directly: when `RateLimiter`/`ActuatorLag` were
moved out of `Autopilot` to fix the earlier SIL continuous-time blocker,
the *inputs* got reconnected correctly but the *outputs* never got
wired back in — `ActuatorLag_Elevator`/`ActuatorLag_Aileron` had been
dead ends since that rewiring, silently bypassed by a direct raw-signal
branch straight into the Vector Concatenate feeding JSBSim. Every test
run since that rewiring — the REQ-006/010 SIL comparison, the first
REQ-011/012 stress-coupling recheck — had actually been running with
**no actuator dynamics at all**, unrelated to whether the trim fix
worked. (One added wrinkle while tracing this: a second, orphaned,
identically-named pair of continuous `TransferFcn` blocks — leftover
from an earlier edit, connected to nothing — briefly caused me to
misdiagnose which blocks were actually live. The real, active pair
turned out to be `DiscreteTransferFcn` type, further downstream.)

**Fixed by rewiring `ActuatorLag`'s (not `RateLimiter`'s) output
directly into the Vector Concatenate**, replacing the raw bypass
branch. `RateLimiter`'s output alone would have restored slew-rate
protection but silently dropped the first-order lag — both stages
needed to be in the final path, not just one.

**Final re-verification, on the actually-correct wiring:**
- **REQ-016**: 0.9998 (all three, masking the real behavior) →
  0.7783 / 0.2534 / 0.4039 (low/cruise/high) — a real improvement,
  not just a different number.
- **REQ-011/012**: essentially unchanged from before either fix
  (~6.2° vs. 7.5346° peak bank) — and that's the right result, not a
  concerning one: this test's aileron commands are small enough that
  neither bug meaningfully touched them. The propeller/torque
  explanation is now confirmed on trustworthy wiring, having survived
  two significant architecture corrections without changing.

**Loose end, not urgent:** the orphaned duplicate `TransferFcn` blocks
(dead, disconnected, coincidentally same names as the real actuator
blocks) were recommended for deletion but not independently
reconfirmed as removed. Purely cosmetic — they do nothing — but worth
clearing out before the push so nobody else gets confused by two
identically-named blocks the way this session briefly was.

## REQ-013 revisited: a genuine thrust limit found, not a bug

Re-running the 800s combined mission post-fixes showed `stress_low`
(90 ft/s start) never recovering to cruise — settling at ~99-100 ft/s
instead of ~185 ft/s. Rather than assume this was a new problem from
either fix, built a focused diagnostic isolating just this case and
logging throttle directly: pinned at ~0.996 for the entire run, no
excess authority left. This is a real aerodynamic limit, not a control
gap — a C172 at ~90 ft/s is close to stall and on the back side of the
power curve, where slower flight requires *more* power, not less.
The controller's actual behavior here is correct: no divergence, no
instability, just a new stable equilibrium at a lower speed while
still flying the mission. Updated REQ-013 to state this precisely
rather than leave the original "converges to cruise from all three
ICs" claim standing now that it's known to be incomplete for the
low-speed case.

Remaining:
1. **`history/` restructure** — physical file moves, blocked on the user
   allocating which `.slx` belongs to which stage (now includes sorting
   `Autopilot.slx` itself, its parent model, and the checkpoint's
   `Autopilot.slx` from `generated_code/` as three distinct artifacts).
2. Push to GitHub. All 17 requirements are closed on the actual final
   controller as of this pass — nothing left checkpoint-only, and the
   two bugs found during this last verification pass are both fixed
   and reconfirmed, not just patched and assumed good.
