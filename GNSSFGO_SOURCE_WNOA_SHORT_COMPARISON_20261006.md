# Source-only WNOA versus WNOJ: frozen short ETH1 comparison

Date: 2026-10-06. User authorization: “i think is worth testing the wnoa scenario. proceed”.

## Outcome

**WNOA is numerically valid in this bounded test and substantially reduces the
current-pose prediction/revision spikes. It is not yet a validated replacement
for WNOJ across sequences or a ground-truth accuracy result.**

The same saved eight-second ETH1 CBS-OFF and CBS-ON inputs were replayed without
central IMU factors. No GLIM/Kimera rerun, live experiment, full sequence,
covariance sweep, or production change was made. The comparison uses the
converged nominal WNOJ results, not an earlier under-converged receiver.

| Quantity | OFF WNOJ | OFF WNOA | ON WNOJ | ON WNOA |
|---|---:|---:|---:|---:|
| Largest next-publication revision of the same current pose, cm | 15.698 | **4.299** | 21.380 | **6.062** |
| RMS of those position revisions, cm | 4.273 | **1.159** | 4.697 | **1.641** |
| Maximum published current speed, m/s | 1.846 | **0.317** | 2.643 | **0.668** |
| Largest same-time current velocity revision, m/s | 1.936 | **0.376** | 2.577 | **0.566** |
| As-published path length, m | 3.308472 | 1.242662 | 3.107039 | 1.286524 |
| Final retrospective support path length, m | 0.863946 | 0.844264 | 0.874406 | 0.864605 |
| Final retrospective 10 ms interpolated path length, m | 0.881479 | 0.863774 | 0.896816 | 0.885584 |

The as-published polyline includes changes between successive online estimates;
it is not the same object as the final reconstructed trajectory. Neither path
length nor a lower speed is an accuracy score. Native GLIM and native Kimera
were not modified or re-evaluated in this motion-model diagnostic.

## Scope and frozen inputs

Artifacts: [gnssfgo_source_wnoa_comparison_20261006](/home/yeranis/results/gnssfgo_source_wnoa_comparison_20261006).
The [pre-run protocol](/home/yeranis/results/gnssfgo_source_wnoa_comparison_20261006/PROTOCOL_BEFORE_RUN.md)
fixed the model, noise mapping and pass criteria before the recorded-data runs.
Protocol SHA-256: `8dc04c598053fc3c2dfbac3b6685a7d8229503815c769c6ed01999d3c02102e9`.

Reference: [full-relinearization WNOJ audit](/home/yeranis/repos/gnssFGO/GNSSFGO_SOURCE_WNOJ_FULL_RELINEARIZATION_20261006.md),
with frozen outputs in `/home/yeranis/results/gnssfgo_source_wnoj_full_relinearization_20261006/{off,on}_primary`.

Inputs in `/home/yeranis/results/gnssfgo_source_wnoj_first_spike_20261005/inputs`:

| File | SHA-256 |
|---|---|
| `off_events.csv` | `5a884a946e0e26be384e58c2b00b0592f84eee6ad559c3125fc1416ae1507eed` |
| `on_events.csv` | `a84a27812855dffa3da7dc5288d1e7f882c11784331004398858e80f37c41454` |

The eight-second journal interval is
`1727775008003475000`–`1727775016003475000` ns. The receiver starts at
`1727775008583475000` ns after 59 timestamp events and creates 75 supports at
100 ms spacing: 7.4 seconds of output support. It is independent of either
source's endpoint grid. The existing `uniform_imu` option means a timestamp
clock here: zero IMU values are used for initialization or estimation.

Unchanged: asynchronous availability, actual endpoint timestamps, source
means/full dense covariances, source scale 1, stable-ID revision replacement,
full-history iSAM2, original initial pose/velocity/angular-rate priors,
publication-convergence policy, and absence of gating/robustification/GNSS.
Source CBS remains the recorded classic protocol; downstream revisions are
not a source-to-source `live_revision` protocol.

## Existing gnssFGO code reused

The motion factor and interpolator are the existing library implementations:

- [GPWNOAPrior constructor/covariance](/home/yeranis/repos/gnssFGO/online_fgo/include/factor/motion/GPWNOAPrior.h:55)
  and [residual](/home/yeranis/repos/gnssFGO/online_fgo/include/factor/motion/GPWNOAPrior.h:183).
- [GPWNOAInterpolator](/home/yeranis/repos/gnssFGO/online_fgo/include/model/gp_interpolator/GPWNOAInterpolator.h:200),
  with the existing `calcLambda`/`calcPsi` construction.

The isolated [SourceWnoaModel.h](/home/yeranis/results/gnssfgo_source_wnoa_comparison_20261006/build/SourceWnoaModel.h)
only adapts these classes to the already established receiver interface.
[DiagnosticReplay.cpp](/home/yeranis/results/gnssfgo_source_wnoa_comparison_20261006/build/DiagnosticReplay.cpp)
is an assertion-checked mechanical fork of the validated diagnostic receiver.
Its preparation script and unified diffs are retained. The journal parser,
source scheduling, support-clock creation, stable-ID ledger, and revision
remove/insert operations are reused rather than rebuilt.

At each support WNOA estimates `X` (Pose3), `V` (world linear velocity), and `W`
(body angular velocity): 12 local degrees of freedom. There is no independent
acceleration `A`, bias `B`, gravity initialization, or Combined IMU factor.

Writing `u_i=[omega_i; R_i^T V_i]`, `r=Log(X_i^-1 X_j)`, the stock prior is

```text
e_motion = [ r - dt*u_i
             Jr(r)^-1*u_j - u_i ]
```

Its zero-noise mean follows constant body twist, not a hard requirement that
the physical robot has constant velocity. Turning can still produce world-frame
acceleration. The uncertain motion prior permits departures from that mean.

The initial guess is the zero-residual transition of this exact stock prior;
it is subsequently optimized. The separate stock `GPExtrapolatorPose3WNOA`
was not assumed validated or used: its source is marked TODO and directly
combines world velocity with body angular velocity. This task makes no change
to it. The adapter's transition was instead checked against the actual prior
at a non-identity, rotating/translating state.

Relative source factors use the same `Local(Z, T_i^-1 T_j)` residual and full
6x6 Gaussian covariance. Exact endpoints reference X; interpolated endpoints
reference X/V/W at their two bracketing supports. Shared keys are deduplicated
and their Jacobian contributions added. There is no new covariance scaling,
interpolation-noise addition, or correlation correction. Stock numerical
Jacobians (`useAutoDiff=true`, the library's historical name) are used, as in
the WNOJ comparison.

## Predeclared uncertainty mapping—not a tuned optimum

WNOJ and WNOA Qc do not have the same units. Matching their numeric values would
not match their stochastic models. For a local zero-initial-uncertainty interval:

```text
WNOJ pose-process covariance = Qc_J * dt^5 / 20
WNOA pose-process covariance = Qc_A * dt^3 / 3

At dt0 = 0.1 s:
Qc_A = (3*dt0^2/20) * Qc_J
```

| Model | Rotational Qc, each axis | Translational Qc, each axis |
|---|---:|---:|
| Frozen nominal WNOJ | 2.5 rad²/s⁵ | 10 m²/s⁵ |
| New WNOA test | 0.00375 rad²/s³ | 0.015 m²/s³ |

The full WNOA discrete process covariance is
`[[dt^3 Qc/3, dt^2 Qc/2], [dt^2 Qc/2, dt Qc]]`, including its cross blocks.
Both models consequently have per-axis 100 ms pose-process standard deviations
of 0.001118 rad and 0.002236 m. WNOA's corresponding velocity-process standard
deviations are 0.019365 rad/s and 0.038730 m/s.

Initial X/V/W standard deviations remain `1e-5 rad/m`, `10 m/s`, `3 rad/s`,
with zero derivative means. Only the nonexistent A state/prior disappears.

**This mapping matches one covariance block at one duration. It does not
equalize all derivative uncertainties or longer-duration behaviour.** The result
is therefore a model-plus-predeclared-noise comparison, not an isolated proof
that deleting acceleration alone caused every improvement. No alternate Qc was
tried after observing results.

## Numerical, causal and source-accounting checks

Release and ASan/UBSan each passed all three focused test groups:

1. Zero-noise transition and stock prior Jacobians at a general rotated state:
   maximum Jacobian discrepancy `2.22e-10`.
2. Exact, single-interpolated, same-interval, adjacent-interval and separated
   two-interpolated endpoints; unique/shared keys, full dense source covariance,
   Mahalanobis cost, factor cloning and endpoint limits. Interpolator Jacobian
   discrepancy `3.89e-10`.
3. Stationary, translating and turning synthetic graph optimization;
   finite SPD joint covariance and deterministic repeated covariance recovery.

Whole-receiver stationary/velocity/acceleration fixtures completed with causal
revision replacement. Changing every raw IMU value without changing timestamps
left all retained state/trajectory/model/dispatch products byte-identical.
Synthetic maximum position deviations were 0, `2.66e-7 m`, and `0.02210 m`,
respectively. The accelerating trajectory is deliberately not a zero-noise
WNOA solution; its nonzero mismatch is reported, not tuned away.

Each real-data mode has exactly the same bookkeeping as WNOJ:

- 2,572 received source versions; 24 outside initial support, unchanged.
- 93 unique accepted IDs: 71 GLIM and 22 Kimera.
- 2,455 replacements, 2,548 total insertions including replacements; no stale
  or duplicate revisions; no pending beliefs at end.
- 22 queued arrivals pending a right support; maximum wait 14.022274 ms.
- Source insertion never precedes availability or required support creation.
- Final graph: 225 Values keys, 74 WNOA factors, 93 source factors and 3 priors
  = 170 factors; zero A/B keys and zero central IMU factors.

Support clocks, source covariance exports, causal dispatch, source accounting,
and unsupported records are byte-identical to WNOJ. Every snapshot's source
ledger matches apart from the expected factor-slot shift from removing a prior.
Dense covariance blocks serialized from the actual source factors also match.

At all 74 publications plus final flush in each mode, an independent LM
optimizer and restarted polish used the exact same graph/information set.
Their results never fed back into iSAM2. All **150/150** passed the predeclared
convergence and agreement thresholds:

| Worst over the 75 checks | OFF | ON |
|---|---:|---:|
| Incremental column-normalized gradient | 6.60e-6 | 9.74e-6 |
| Incremental/reference position difference, m | 1.90e-8 | 2.40e-8 |
| Orientation difference, rad | 5.83e-9 | 7.98e-9 |
| Velocity difference, m/s | 2.05e-7 | 2.05e-7 |
| Angular velocity difference, rad/s | 1.37e-7 | 1.65e-7 |
| Relative objective excess | 3.94e-11 | 3.94e-11 |

Both full short replays repeated byte-identically, including state, trajectory,
factor probes, and numerical audit outputs (excluding wall-clock columns).
Removing read-only batch audits also left receiver products byte-identical.
These claims apply to the tested inputs/build, not universal bitwise invariance.

## Prediction versus source-covered history

For each publication with source information, let F be the latest endpoint of
an arrived source factor. Compare the state at the *same physical time* before
and after the next publication, separately for:

- the newest support (“current prediction”);
- the latest support at or before F (“source-covered support”).

There are 70 matched origins per mode. The information sets and times are the
same for both models. This does not mistake ordinary forward motion for a jump.

| Position revision statistic, cm | OFF WNOJ | OFF WNOA | ON WNOJ | ON WNOA |
|---|---:|---:|---:|---:|
| Current median | 0.929 | 0.354 | 0.883 | 0.435 |
| Current p95 | 10.252 | 2.427 | 11.165 | 4.093 |
| Source-covered RMS | 1.017 | 0.660 | 1.109 | 0.988 |
| Source-covered maximum | 3.051 | 2.019 | 4.565 | 3.798 |

The already identified ON update 61→62 remains the largest WNOA current-pose
revision: at physical `1727775014683475000 ns`, the previous source frontier
lags by 128.1175 ms. Position revision falls from 21.380 to 6.062 cm; velocity
revision falls from 2.577 to 0.566 m/s. It is reduced, not eliminated.

Not every derivative metric improves: source-covered velocity-revision RMS
increases from 0.02594 to 0.03278 m/s OFF and 0.03913 to 0.04359 m/s ON.
WNOA's largest final 10 ms within-interval departure from the straight support
chord is 3.83 mm OFF and 6.59 mm ON (WNOJ 7.85 and 6.89 mm); neither model has
an interval above 1 cm in these final short trajectories.

Final WNOA versus WNOJ support-position RMS difference is only **1.64 mm OFF**
and **0.89 mm ON**, using the same initial frame without a new alignment.
Maximum differences are 10.54 mm and 4.85 mm. Most of the visible benefit is
therefore in the provisional prediction, not a wholesale change of the final
source-supported path.

![Same-time revision and speed comparison](/home/yeranis/results/gnssfgo_source_wnoa_comparison_20261006/prediction_comparison.png)

## Fit to unchanged source beliefs

These are final graph residuals against input beliefs, **not GT errors**.
Translation is the rho norm of the existing SE(3) residual. The same IDs and
covariances are compared within each mode.

| Mode / source | WNOJ rho RMS, mm | WNOA rho RMS, mm | WNOJ rotation RMS, deg | WNOA rotation RMS, deg |
|---|---:|---:|---:|---:|
| OFF / GLIM | 12.390 | 12.367 | 0.05366 | 0.05107 |
| OFF / Kimera | 2.240 | 2.185 | 0.07209 | 0.06760 |
| ON / GLIM | 4.349 | 4.227 | 0.04228 | 0.03418 |
| ON / Kimera | 10.181 | 10.056 | 0.07650 | 0.06661 |

Source-fit RMS does not deteriorate in this test. However, mean squared
whitened 6D source residuals remain large: WNOA OFF GLIM/Kimera 261.4/494.7;
ON 83.5/340.6. These are optimized residual diagnostics, not an innovation-NIS
or independent chi-square calibration proof. WNOA has **not resolved source
disagreement, covariance calibration or shared-information correlations**.

Final half-squared whitened costs:

| Mode / model | Motion prior | GLIM | Kimera |
|---|---:|---:|---:|
| OFF WNOJ | 5582.04 | 9907.36 | 6129.85 |
| OFF WNOA | 5925.65 | 9279.80 | 5442.11 |
| ON WNOJ | 7143.64 | 4137.68 | 4708.76 |
| ON WNOA | 7520.74 | 2963.62 | 3746.42 |

Motion costs belong to different residual dimensions/models (18D WNOJ versus
12D WNOA per interval); their totals are not a model-selection score.

## Runtime and reproducibility

WNOA receiver-only replay, including strict publication refinement but excluding
batch snapshot audits, took 1.70 s OFF and 1.77 s ON. Retained WNOJ measurements
were 5.41 s and 5.83 s. These are indicative timings from different runs, not a
controlled real-time capacity benchmark. Including every snapshot/reference
audit took about 30 s per mode; no source estimators or visualization ran.

Exact commands used:

```bash
python3 /home/yeranis/results/gnssfgo_source_wnoa_comparison_20261006/prepare.py
bash /home/yeranis/results/gnssfgo_source_wnoa_comparison_20261006/run.sh unit
bash /home/yeranis/results/gnssfgo_source_wnoa_comparison_20261006/run.sh build
bash /home/yeranis/results/gnssfgo_source_wnoa_comparison_20261006/run.sh sanitizer
bash /home/yeranis/results/gnssfgo_source_wnoa_comparison_20261006/run.sh fixtures
bash /home/yeranis/results/gnssfgo_source_wnoa_comparison_20261006/run.sh primary
bash /home/yeranis/results/gnssfgo_source_wnoa_comparison_20261006/run.sh repeat
bash /home/yeranis/results/gnssfgo_source_wnoa_comparison_20261006/run.sh timing
/home/yeranis/.local/share/pipx/venvs/evo/bin/python /home/yeranis/results/gnssfgo_source_wnoa_comparison_20261006/analyze.py
/home/yeranis/.local/share/pipx/venvs/evo/bin/python /home/yeranis/results/gnssfgo_source_wnoa_comparison_20261006/verify.py
```

The runner uses the existing network-disabled, read-only container image
`sha256:6a664509d183e504230b22edfd747dfe1ca5bca8ae6fc34c76154db8ffd1fc8d`,
mounting source/runtime/inputs read-only and only this artifact directory writable.
The script refuses existing replay output directories; do not overwrite frozen
outputs to reproduce. Use a new directory and equivalent mounts. Compiler
commands, flags, durations and return codes are retained in `commands_*.json`.

Receiver SHA-256: `0a6a8465d315a0018b9ee569000d0378553d7805e10c2d762cf3cdcb927bdcc0`.
Focused sanitizer tests instrument the adapter/library header code they compile,
not the external prebuilt shared libraries. Existing unused base-class
missing-return compiler warnings remain; no claim of a warning-free full
repository build or a rerun of its entire test suite is made.

The [verification manifest](/home/yeranis/results/gnssfgo_source_wnoa_comparison_20261006/verification.json)
confirms **21,026 protected files unchanged**, including the production code,
input journals, frozen reference results and pre-run protocol. `git diff --check`
passes. The repository was already dirty; its preexisting changes were preserved.
No commit was created. Initial/final HEAD:
`a842e7ef72e7a40f0cdfec826f089c66af364ae0`.

## Interpretation and stop

WNOA removes the independent acceleration extrapolation state and gives a
different stochastic relation between poses and velocities. With the predeclared
noise mapping, it reduces this short test's provisional prediction excursions
while keeping the final path close and the source fit at least as good by the
reported RMS diagnostics. This is a useful positive result for the no-central-
IMU model, not proof that a motion prior can reliably predict any manoeuvre or
arbitrary measurement delay.

This window contains little total travel and cannot establish behaviour during
long sustained acceleration, sharp turns, long outages or another sequence.
GT accuracy was deliberately not evaluated or used for parameter selection.
If authorized next, the appropriate extension is the already saved longer
bounded input with these exact settings, not a Qc sweep or production switch.
No such extension was run here. Production WNOJ remains untouched.

**WNOA_SHORT_WINDOW_VALID — reduced prediction revisions; broader validation pending.**
