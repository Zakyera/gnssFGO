# ETH1 source WNOJ motion uncertainty — Step B

Step B identifies a material model sensitivity: **reducing the permissive WNOJ process uncertainty strongly suppresses the internal reversals and derivative excursions found in Step A, without central IMU factors. It also substantially increases disagreement with the unchanged source beliefs.** This is not yet a validated replacement configuration or evidence of improved ATE.

All 72 sampled graph optimizations pass the preregistered convergence checks and reproduce deterministically. Production settings, native estimators, source covariances and previous artifacts are unchanged. No ground truth, full sequence, live receiver experiment or further parameter search was used.

Artifacts: [Step B directory](/home/yeranis/results/gnssfgo_source_wnoj_step_b_20261006). Main evidence: [analysis.json](/home/yeranis/results/gnssfgo_source_wnoj_step_b_20261006/analysis.json), [snapshot_summary.csv](/home/yeranis/results/gnssfgo_source_wnoj_step_b_20261006/snapshot_summary.csv), [verification.json](/home/yeranis/results/gnssfgo_source_wnoj_step_b_20261006/verification.json). The prior reference is [Step A](/home/yeranis/repos/gnssFGO/GNSSFGO_SOURCE_WNOJ_CONVERGED_REFERENCE_STEP_A_20261006.md).

## 1. Scope and frozen information

The same saved eight-second ETH1 journals were used:

```text
OFF /home/yeranis/results/gnssfgo_source_wnoj_first_spike_20261005/inputs/off_events.csv
SHA 5a884a946e0e26be384e58c2b00b0592f84eee6ad559c3125fc1416ae1507eed

ON  /home/yeranis/results/gnssfgo_source_wnoj_first_spike_20261005/inputs/on_events.csv
SHA a84a27812855dffa3da7dc5288d1e7f882c11784331004398858e80f37c41454
```

Journal availability spans `1727775008003475000` through `1727775016003475000 ns`. There are 75 independent 100 ms support states from `1727775008583475000 ns`, covering 7.4 seconds. The 59-sample clock anchor is unchanged. Raw IMU values are not measurements or initialization inputs in this experimental `source_wnoj` model; their timestamps supply the existing independent clock. Source estimates and their established timing are untouched.

Frozen in every comparison:

- Pose X, world velocity V, body angular rate W and six-component tangent acceleration A; the existing full WNOJ prior and interpolator, with estimated acceleration.
- Original relative means and complete dense source covariance, scale one; source IDs, revisions, physical endpoints, arrival times, factor keys, support spacing and replacement policy.
- Initial pose identity with sigma `1e-5`; zero initial V/W/A with sigmas 10 m/s, 3 rad/s and 10 in the respective acceleration coordinates. These remain priors, not imposed constant derivatives.
- No central IMU factor, bias state, gravity input, GNSS, gating, robust loss or new correlation model.

Each mode retains the same 93 source IDs: 71 GLIM and 22 Kimera. Each consumes 2572 versions, with 2548 accepted insertions including 2455 replacements; 24 unsupported messages remain excluded, with no stale or duplicate revisions. Final graphs contain four priors, 74 WNOJ factors and 93 relative factors: 171 active factors, 300 keys and 1350 scalar coordinates.

Source-to-source classic CBS is not rerun or modified. Downstream stable-ID revision replacement is preserved. These are offline, copied-graph diagnostics using the information available at selected arrival bounds—not newly co-running GLIM, Kimera and gnssFGO.

## 2. Candidate assumptions fixed before execution

The [pre-run protocol](/home/yeranis/results/gnssfgo_source_wnoj_step_b_20261006/PROTOCOL_BEFORE_RUN.md) and manifest were created before the primary experiment. Protocol SHA-256:

```text
529a4512beac20d2810d7e8b3fba7f9d019b5ea51bb74648bdd11718b9a0b194
```

For one local coordinate, WNOJ has acceleration-innovation variance `q Δt`. Thus `sqrt(q Δt)` is the one-sigma acceleration innovation, not a hard acceleration limit. The nominal engineering assumption was 0.5 rad/s² angular and 1 m/s² linear acceleration innovation over 100 ms, giving `q_rot = 2.5` and `q_trans = 10`. These are explicit engineering hypotheses, **not calibration results, measured platform specifications or values selected using GT**.

| Profile | q_rot, rad²/s⁵ | q_trans, m²/s⁵ | Angular innovation std over 100 ms, rad/s² | Linear innovation std over 100 ms, m/s² |
|---|---:|---:|---:|---:|
| Baseline | 10000 | 10000 | 31.6228 | 31.6228 |
| Nominal | 2.5 | 10 | 0.5000 | 1.0000 |
| Tight: 0.1× nominal | 0.25 | 1 | 0.1581 | 0.3162 |
| Loose: 10× nominal | 25 | 100 | 1.5811 | 3.1623 |

The same four profiles were applied to OFF and ON. No additional values were tried. Smaller Qc means a stronger penalty on departures from the WNOJ motion model. It does not mean constant or zero acceleration. “Loose” is relative to the nominal candidate; it is still much stronger than the original baseline.

In the local linear interpretation, the nominal interval-averaged jerk standard deviations over 100 ms are 5 rad/s³ and 10 m/s³. The baseline translation process position-innovation standard deviation over one 100 ms interval is about 7.07 cm; nominal is 2.24 mm. These are process-noise marginals, not posterior position uncertainties or bounds on actual displacement.

## 3. Exact implementation and comparison

The isolated helper copies the existing diagnostic graph and changes only each WNOJ factor's Gaussian noise model using `cloneWithNewNoiseModel`. It calls the stock [calcQ3](/home/yeranis/repos/gnssFGO/online_fgo/include/utils/GPUtils.h:93):

```text
Q(Δt) = [ Δt⁵/20 Qc   Δt⁴/8 Qc   Δt³/6 Qc
          Δt⁴/8 Qc    Δt³/3 Qc   Δt²/2 Qc
          Δt³/6 Qc    Δt²/2 Qc   Δt Qc   ]

Qc = diag(q_rot, q_rot, q_rot, q_trans, q_trans, q_trans).
```

The full 18×18 covariance, including its cross-time-derivative blocks, is retained. No residual, state dimension or Jacobian implementation is replaced. This is exactly the noise construction used by [GPWNOJPriorFull](/home/yeranis/repos/gnssFGO/online_fgo/include/factor/motion/GPWNOJPriorFull.h:41). Source factor and prior payloads remain byte-identical at the fixed initial values across all profiles.

For this constant-Qc WNOJ model, Q has a temporal-matrix tensor Qc form and the transition has scalar-identity blocks. Qc cancels from the conditional-mean interpolation gains Lambda and Psi. The existing source endpoint interpolators can therefore be retained. This was checked at seven fractional offsets, including both endpoints, for every support interval and profile: maximum gain change `3.64e-12`. No GP interpolation uncertainty was added to source R. The existing treatment of interpolation uncertainty remains a limitation, not something resolved here.

Eight snapshots per mode use support indices 10, 20, 30, 38, 48, 58, 68 and 74. A ninth snapshot is the final graph after all remaining arrivals within the journal cutoff. Every profile starts from exactly the same baseline incremental estimate at that event. Copied-graph batch solutions never feed back into the receiver, its later initial guesses or other profiles.

The solver is the unchanged Step A reference: batch Levenberg–Marquardt with Ceres defaults, multifrontal QR, maximum 500 iterations and relative/absolute objective tolerances `1e-11`, followed by a maximum-100-iteration restart. Acceptance requires:

- Column-normalized gradient infinity norm at most `1e-5`.
- Restart maximum position change below `1e-6 m` and velocity change below `1e-4 m/s`.
- Neither iteration cap exhausted.

**72/72 pass.** Main solves take 2–15 iterations; restarts take 0–1. Normalized gradients range from `5.06e-10` to `3.95e-6`. Maximum restart changes are `3.09e-7 m` position, `3.88e-8 rad` orientation, `2.92e-6 m/s` velocity, `3.20e-7 rad/s` angular rate, `2.34e-5 m/s²` translation tangent acceleration and `2.30e-5 rad/s²` angular tangent acceleration. These establish local stationarity and repeatability, not global optimality.

## 4. Derivative excursions across the sampled graphs

Each maximum below is over all support states in all nine sampled graphs of that mode/profile. This is not an exhaustive maximum over every event in the original stream.

| Mode | Profile | Max support speed, m/s | Max translation tangent acceleration, m/s² | Max body angular rate, rad/s | Max angular tangent acceleration, rad/s² |
|---|---|---:|---:|---:|---:|
| OFF | Baseline | 10.99199 | 100.55223 | 2.42448 | 22.14187 |
| OFF | Nominal | 0.58412 | 6.35390 | 0.63946 | 7.37711 |
| OFF | Tight | 0.31799 | 2.95220 | 0.49203 | 3.99426 |
| OFF | Loose | 2.24709 | 21.11210 | 0.99251 | 10.41914 |
| ON | Baseline | 5.67177 | 51.95883 | 1.49078 | 15.65122 |
| ON | Nominal | 0.46463 | 5.97200 | 0.64004 | 6.32898 |
| ON | Tight | 0.37171 | 3.03832 | 0.49286 | 5.37086 |
| ON | Loose | 0.85083 | 8.63190 | 1.13413 | 12.71462 |

The large baseline derivatives are therefore not an unavoidable consequence of removing central IMU factors in this short section. The WNOJ uncertainty materially controls them. This does not establish the correct acceleration/jerk distribution for the full mission.

## 5. Motion between support poses

The final graph of each profile is sampled with the unchanged stock pose interpolator every 10 ms. Each row describes one coherent optimized graph, not mixed historical publications. Support interpolation errors are exactly zero in the exported positions.

| Mode | Profile | 100 ms support path, m | 10 ms sampled path, m | Max within-interval departure, mm | Sum of backward projected travel, mm | Intervals over 1 cm departure / 74 |
|---|---|---:|---:|---:|---:|---:|
| OFF | Baseline | 1.722683 | 6.162684 | 122.652 | 1806.012 | 47 |
| OFF | Nominal | 0.863946 | 0.881479 | 7.853 | 1.324 | 0 |
| OFF | Tight | 0.812378 | 0.818805 | 3.665 | 0.670 | 0 |
| OFF | Loose | 1.026771 | 1.262577 | 14.707 | 56.015 | 9 |
| ON | Baseline | 1.185825 | 3.061473 | 65.343 | 651.811 | 44 |
| ON | Nominal | 0.874406 | 0.896816 | 6.886 | 0.670 | 0 |
| ON | Tight | 0.844076 | 0.853198 | 3.913 | 0.174 | 0 |
| ON | Loose | 0.903872 | 1.041469 | 15.437 | 24.077 | 2 |

Departure means distance from the position obtained by linearly interpolating that profile's two optimized support positions at the same physical time. Backward travel sums negative projected motion along each interval's endpoint chord; it is not a claim that all real backward motion is wrong. The 1 cm count is a descriptive Step A diagnostic, not a post-hoc acceptance threshold. Path lengths are sampled polylines, not GT path errors.

Nominal and tight have zero intervals above that descriptive threshold in **all nine sampled graphs**, not only the final graph. The largest nominal departure over all sampled graphs is 7.85 mm OFF and 7.22 mm ON. Loose retains some pronounced internal motion, reaching 26.17 mm OFF and 15.54 mm ON over the sampled graphs.

At the concrete Step A problematic OFF interval, supports 47–48, the baseline travels 24.83 cm inside a 100 ms interval with 6.71 cm endpoint separation, including 9.01 cm backward projected travel. Nominal travels 1.46 cm with 1.39 cm endpoint separation and no backward travel. At ON supports 59–60, baseline travels 13.34 cm for 2.22 cm separation; nominal travels 4.72 cm for 4.70 cm separation, again without backward travel. Both intervals precede the latest already-arrived measurement endpoint.

**The support poses also change.** This is not merely drawing a smoother curve through fixed positions. Relative to baseline, nominal final support-position changes have RMS/max 6.85/17.07 cm OFF and 1.33/3.19 cm ON. Their truth errors were not measured here.

As an independent geometric diagnostic, maximum 10 ms world-position second-difference acceleration falls from 121.88 to 6.36 m/s² OFF and 66.45 to 6.78 m/s² ON under nominal. This differs from the tangent acceleration state in the preceding table; neither is a raw accelerometer reading.

## 6. Cost of smoothing: source disagreement

These are **post-fit residuals against the supplied relative beliefs**, not errors against GT. Translation and rotation use the existing six-coordinate `Pose3::Local(measured, predicted)` source residual without a new measurement model. The whitened diagnostic is `sum(rᵀ R⁻¹ r)/(6 N)` using each unchanged full covariance. It is neither an information-weight percentage nor an independent innovation NIS; correlations and fitted degrees of freedom prevent a simple chi-square interpretation.

| Mode | Profile | GLIM translation RMS, mm | Kimera translation RMS, mm | GLIM rotation RMS, deg | Kimera rotation RMS, deg | GLIM squared whitened residual / coordinate | Kimera squared whitened residual / coordinate |
|---|---|---:|---:|---:|---:|---:|---:|
| OFF | Baseline | 1.939 | 0.205 | 0.003954 | 0.000983 | 0.713 | 0.146 |
| OFF | Nominal | 12.390 | 2.240 | 0.053658 | 0.072094 | 46.513 | 92.877 |
| OFF | Tight | 12.861 | 2.706 | 0.070022 | 0.108130 | 72.501 | 205.599 |
| OFF | Loose | 10.698 | 1.611 | 0.032102 | 0.031454 | 24.691 | 21.556 |
| ON | Baseline | 0.601 | 1.292 | 0.001167 | 0.002786 | 0.095 | 0.386 |
| ON | Nominal | 4.349 | 10.181 | 0.042280 | 0.076505 | 19.426 | 71.345 |
| ON | Tight | 5.120 | 12.294 | 0.070089 | 0.133772 | 54.026 | 205.025 |
| ON | Loose | 3.688 | 7.868 | 0.013834 | 0.030446 | 4.433 | 18.846 |

For nominal, translation residual p95/max are 24.21/31.06 mm GLIM and 4.02/4.07 mm Kimera in OFF; 9.07/19.12 mm GLIM and 22.16/25.51 mm Kimera in ON. Full mean/median/RMS/p95/max summaries for every profile and snapshot are retained in `analysis.json`, along with each factor's six residual coordinates.

The larger residuals are concrete evidence of tension between stronger smoothness and the source factors under their assigned uncertainties. They do **not** identify which source is wrong, prove its covariance is miscalibrated, or prove the candidate process model is correct. Even a physically plausible motion prior can be too restrictive for real motion or conflict with another modeling approximation. Conversely, close source fit under baseline does not prove physical accuracy when it uses large unobserved inter-support excursions.

Final objective decomposition, with each entry half the summed squared whitened residual:

| Mode | Profile | WNOJ | GLIM | Kimera | Startup priors | Total |
|---|---|---:|---:|---:|---:|---:|
| OFF | Baseline | 663.828 | 151.933 | 9.651 | 0.000289 | 825.412 |
| OFF | Nominal | 5582.042 | 9907.358 | 6129.853 | 0.000062 | 21619.253 |
| OFF | Tight | 5560.181 | 15442.769 | 13569.527 | 0.000004 | 34572.477 |
| OFF | Loose | 3907.476 | 5259.149 | 1422.727 | 0.000100 | 10589.351 |
| ON | Baseline | 177.592 | 20.269 | 25.462 | 0.000063 | 223.323 |
| ON | Nominal | 7143.643 | 4137.678 | 4708.759 | 0.000053 | 15990.081 |
| ON | Tight | 7545.629 | 11507.468 | 13531.651 | 0.000004 | 32584.748 |
| ON | Loose | 2536.506 | 944.308 | 1243.842 | 0.000759 | 4724.657 |

Different Qc values define different objectives, and OFF/ON provide different source data. **Do not select a winner from these total costs.** Priors having small residual cost also does not imply that they have no anchoring effect.

## 7. What this establishes—and does not

The controlled result supports a specific mechanism: the previous WNOJ uncertainty permits large derivative changes and internal bending while fitting very confident relative beliefs closely. Changing only its process-noise covariance suppresses those behaviors in converged solutions in both modes. Therefore the Step A defect is not just incremental solver stopping and is not evidence that a central IMU factor is mathematically indispensable.

However, this test does not establish a healthy source-driven fusion baseline. The candidates trade substantial source-fit loss for smoother motion, with support-position changes as well. The nominal/tight solutions are locally converged and less oscillatory, but neither their accuracy nor their generalization is established. Tight is not automatically preferable because it is smoothest. OFF/ON source covariance and correlation issues are unchanged; removing central IMU does not remove information shared through the source estimators and classic CBS.

The production `imu_wnoj` baseline remains unchanged. No candidate is promoted or saved as the official live configuration. A future decision would require independent validation of the motion assumptions and the source-fit trade-off, rather than another GT-selected Qc sweep. That work, incremental-solver integration, and longer/other-sequence tests are outside this task and were not run.

## 8. Validation and preservation

- Isolated Release diagnostic build succeeded using the existing runtime. The first build exposed a C++ null-pointer overload ambiguity in diagnostic code; an explicitly typed null factor fixed compilation. Both logs are retained. No model formula changed and no dependency was installed.
- 72/72 graph references pass the fixed convergence/restart criteria. Repeated numerical outputs are byte-identical; summaries match excluding wall time.
- The baseline profile matches all prior Step A snapshot numerical products, including optimized states, dense interpolated poses, costs, iteration histories, gradients and factor probes.
- Twelve original receiver output/accounting products per mode remain byte-identical to Step A. Revision-event rows match excluding timing; summary accounting matches excluding runtime. Diagnostics therefore did not change the causal receiver.
- Every profile has identical initial state values and identical source/prior payload probes: keys, full Gaussian covariance and fixed-state residuals. WNOJ unwhitened residual changes are exactly zero. Maximum reconstructed covariance relative error is `2.43e-14`, with SPD checks; maximum interpolation-gain change is `3.64e-12`.
- All 1059 predeclared protected files pass SHA-256 preservation checks, including the protocol, Step A artifacts, source inputs, production/model code, native outputs, profiles and installed receiver.
- `git diff --check` passes. No new production unit test was needed for the isolated copied-noise study. This task does not claim a new complete ROS test-suite or sanitizer run.

## 9. Reproduction, runtime and repository state

Primary command:

```bash
docker run --rm --name source-wnoj-step-b-primary \
  --network none --read-only --user 1000:1000 \
  --tmpfs /tmp:rw,exec,size=2g -e LD_LIBRARY_PATH=/runtime/lib \
  -v /home/yeranis/results/gnssfgo_source_wnoj_step_b_20261006:/audit \
  -v /home/yeranis/repos/gnssFGO:/source:ro \
  -v /home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime:/runtime:ro \
  -v /home/yeranis/results/gnssfgo_source_wnoj_first_spike_20261005:/previous:ro \
  sha256:6a664509d183e504230b22edfd747dfe1ca5bca8ae6fc34c76154db8ffd1fc8d \
  python3 /audit/run.py primary
```

The same mounts/image are used with `run.py build_retry` and `run.py repeat`, with separate container names. `commands_*.json` retains exact compiler/executable argument arrays. Use fresh output directories to reproduce; the scripts reject overwriting existing run logs. Each mode executes:

```text
WNOJ_AUDIT_INDICES=10,20,30,38,48,58,68,74
/audit/build/step_b_replay --receiver-model source_wnoj
--mode sources --timeline record --solver isam2
--support-policy uniform_imu --support-period-ns 100000000
--receiver-startup-samples 59
--event-stream /previous/inputs/{off,on}_events.csv
--out /audit/{off,on}_primary
```

The copied receiver source is unchanged from Step A. Profile construction and checks reside only in [SnapshotAudit.h](/home/yeranis/results/gnssfgo_source_wnoj_step_b_20261006/build/SnapshotAudit.h). Analysis and verification:

```bash
/home/yeranis/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 \
  /home/yeranis/results/gnssfgo_source_wnoj_step_b_20261006/analyze.py
/usr/bin/python3 /home/yeranis/results/gnssfgo_source_wnoj_step_b_20261006/verify.py
```

Release compilation took 32.99 s. Primary processes took 47.02 s OFF and 47.27 s ON; deterministic repeats took 46.66 s and 46.72 s. Each process includes four-profile diagnostics at nine snapshots, full Jacobian/linear-health calculations and exports. These are not live-throughput benchmarks. Optimization/export/post-solve diagnostic times sum to approximately 20.46 s OFF and 20.58 s ON; initial copying and pre-solve health work are outside those latter sums.

Executable SHA-256:

```text
e9836b2ae004731ce98f5e27f9b3984f94e26d356dc488b0f5fe13f96426a6d7
```

HEAD remains `a842e7ef72e7a40f0cdfec826f089c66af364ae0`. No commit was created. The preexisting dirty worktree is preserved; this report is the only new repository file from Step B. Diagnostic code/results are isolated in the new results directory.

Step B complete: process-uncertainty sensitivity demonstrated; source-fit trade-off unresolved; no production setting selected.
