# ETH1 transfer of the fixed SPX-3 Kimera covariance addition

2026-10-02. Two new **offline causal receiver replays** of the retained ETH1 120-second CBS-OFF/ON journals. No GLIM, Kimera or bag-playback run; no production change or parameter search.

**The ATE benefit transfers, but this is not a complete solution.** With exactly the previous SPX-3 Kimera-only addition, ETH1 final fused ATE improves from **0.048359 to 0.040255 m OFF**, and **0.107042 to 0.064701 m ON**. ON improves more, but remains worse than matched OFF. Translation and rotation RPE worsen in both modes, for both online and final outputs. Do not promote this to a universally validated configuration.

## 1. Predeclared intervention and frozen settings

The values come unchanged from [the SPX-3 source-isolation experiment](/home/yeranis/repos/gnssFGO/GNSSFGO_SPX3_SOURCE_MODEL_SOURCE_ISOLATION.md), not from fitting ETH1:

| Receiver source | Rotation sigma, rad | Translation sigma, m |
|---|---:|---:|
| `kimera_front` | 0.0000450299900067 | 0.000601137810046 |
| `glim_hesai` | 0 | 0 |

In the existing `[phi,rho]` tangent, for every Kimera revision:

`Sigma_effective = Sigma_reported + diag(sigma_phi² I3, sigma_rho² I3)`.

The per-axis added variances are **2.0277000000035014e-9 rad²** and **3.6136666666690074e-7 m²**. The same absolute addition is applied OFF and ON. This is an additive model-discrepancy experiment, not covariance multiplication, diagonalization, clipping, or an ON-only adjustment. All 30 off-diagonal entries are retained. Each revision starts from its own reported matrix; additions do not accumulate across revisions.

Combined IMU, WNOJ `Qc=10000`, ALL_RIGHT receiver integration, initial priors, 59 startup IMU samples, independent 100 ms support, full-history iSAM2, stable-ID revision replacement and source covariance scale 1.0 are unchanged. No gating, robust loss, health weighting, GNSS measurement or change to classic source-to-source CBS. The recorded native source settings are preserved, including the saved Kimera `current` packet path; no source rerun silently substitutes a different IMU implementation.

Changing source uncertainty changes the estimated states. Consequently state-dependent support accelerations and preintegration covariance diagnostics also change. Their **models/settings** are frozen; their resulting numerical values are not falsely claimed identical. This is a complete causal receiver replay, not a covariance swap followed only by final batch refinement.

The [protocol](/home/yeranis/results/gnssfgo_eth1_spx3_kimera_uncertainty_transfer_20261002/protocol.md) was saved before the new runs and scores. Neither value was adjusted after seeing either result.

## 2. Data, receiver provenance and implementation check

Source capture: [ETH1 live120 zero](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live120_zero_20260930/), mission `2024-10-01-11-29-55`, first 120 s. Native GLIM and native Kimera ROS odometry remain the original estimator outputs. “Fusion” below means the gnssFGO joint receiver, not either native trajectory or a single-source receiver.

Original journal SHA-256:

* OFF: `d75e2cf25ad413b2c4f9f93e5331b601a695d5cb996a6ab44d32d543010f49e0`
* ON: `fe0054add31063df0d36be106411d44ad439f4e748d740a88d94da3663bc7a38`

The archived [factor-audit receiver](/home/yeranis/results/gnssfgo_spx3_live120_factor_audit_20260929/grandtour_ct_replay_factor_audit), hash `b85c3e9aed77b42fe4a1e9e08bda844eb18f9eddd799b628eeb4fcbbf52338e4`, was reused without rebuilding. Its retained zero reconstructions were rechecked against all **11 pinned original output files per mode**, including both trajectories and state histories: byte-identical. These are reused zero controls, **not new zero-addition replays performed in this task**. Their evaluator scores were recomputed and reproduced to below `1e-12`.

This executable predates the CLI source-noise option. To preserve that known receiver, [the input adapter](/home/yeranis/results/gnssfgo_eth1_spx3_kimera_uncertainty_transfer_20261002/prepare.py) materializes the addition in separate `off_effective_events.csv` and `on_effective_events.csv` files. These are **receiver-effective test inputs, not new reported source exports**. The original journals retain the reported matrices. All other fields and non-Kimera rows are byte-identical, including means, endpoint/arrival timestamps, IDs, revisions and IMU values/order. A zero-addition transformation is byte-identical to its original journal.

The archived [`EventStream::Next()`](/home/yeranis/results/gnssfgo_spx3_live120_factor_audit_20260929/GrandTourCsvReplay.audit.cpp:440) directly parses the covariance and [`AddRelativeFactor()`](/home/yeranis/results/gnssfgo_spx3_live120_factor_audit_20260929/GrandTourCsvReplay.audit.cpp:604) passes it to the existing Gaussian noise construction. There is no second addition in this receiver. A small standalone C++ check calls the actual existing [`EffectiveSourceCovariance()`](/home/yeranis/repos/gnssFGO/online_fgo/include/offline_process/SourceModelDiscrepancy.h:46) on every selected original matrix. It matches every derived matrix **exactly**, maximum difference **0**, over **14,250 Kimera revision messages per mode**. It does not build or run another estimator. [C++ validation](/home/yeranis/results/gnssfgo_eth1_spx3_kimera_uncertainty_transfer_20261002/adapter_cpp_validation.json).

## 3. Evaluation and the original three trajectories

The unchanged prism evaluator uses no-scale SE(3) alignment and the existing CPT7-to-AP20 lever arm. Bounds are **1727775008.775065899–1727775127.674791098**. Every fused result uses exactly **654 prism matches**, covering **64.9 aggregate seconds in five segments**. This is partial prism coverage, not 120 continuous seconds of absolute-position truth. Every fused RPE result uses the same **1,179 one-second intervals** against the retained IE-TC reference. All eight fused prism masks and all eight relative-time masks match exactly.

The native source results are retained, not regenerated:

| Original estimator output | CBS | ATE, m | 1 s t-RPE, m | 1 s r-RPE, deg |
|---|---|---:|---:|---:|
| GLIM | OFF | 0.038007 | 0.027251 | 0.351298 |
| GLIM | ON | 0.028978 | 0.033387 | 0.429674 |
| Kimera ROS odometry | OFF | 0.541016 | 0.045950 | 0.541482 |
| Kimera ROS odometry | ON | 0.365236 | 0.030284 | 0.571374 |

Native GLIM has 655 prism matches / 1,179 RPE intervals; native Kimera has 315 / 1,151. These retain their established native-output sampling, not a newly resampled common grid across all estimators. Thus the table identifies the familiar source trajectories, while the controlled causal claim below compares **fusion with fusion on identical masks**. Both native ATEs improve under ON, but their relative-error changes are not uniformly beneficial.

### Final retrospectively revised fusion

| CBS | Kimera addition | ATE, m | 1 s t-RPE, m | 1 s r-RPE, deg | Evaluation path, m | GT-overlap estimated path, m |
|---|---|---:|---:|---:|---:|---:|
| OFF | zero, retained control | 0.048359 | 0.015597 | 0.414167 | 71.824 | 47.913 |
| OFF | fixed SPX-3 value | **0.040255** | 0.017246 | 0.424911 | 71.964 | 48.001 |
| ON | zero, retained control | 0.107042 | 0.018430 | 0.515790 | 71.928 | 47.912 |
| ON | fixed SPX-3 value | **0.064701** | 0.019587 | 0.553276 | 71.982 | 47.937 |

### Online/as-played fusion output from causal replay

| CBS | Kimera addition | ATE, m | 1 s t-RPE, m | 1 s r-RPE, deg | Evaluation path, m | GT-overlap estimated path, m |
|---|---|---:|---:|---:|---:|---:|
| OFF | zero, retained control | 0.051511 | 0.031413 | 0.575945 | 72.336 | 48.369 |
| OFF | fixed SPX-3 value | **0.046247** | 0.033310 | 0.716101 | 72.553 | 48.502 |
| ON | zero, retained control | 0.122472 | 0.054332 | 0.942747 | 76.687 | 50.271 |
| ON | fixed SPX-3 value | **0.090641** | 0.055861 | 0.964970 | 77.183 | 50.536 |

“Online” is the saved as-played support-state output when processing the journal's causal arrivals; this experiment itself was offline. “Final” incorporates later accepted revisions into historical states. Neither column uses additional final LM optimization. Evaluation path is the prism-offset path within the fixed evaluation bounds, before GT-gap masking. The matched reference path is **47.790 m** in every fused row.

Full untrimmed CPT7-body path lengths, zero → addition, are OFF final **71.867 → 72.018 m**, ON final **72.005 → 72.062 m**, OFF online **72.405 → 72.611 m**, ON online **76.787 → 77.245 m**. These differ from the prism-offset/windowed paths above by definition.

## 4. What transfers, and what does not

Final ATE falls **16.76% OFF** and **39.56% ON**. The ON-minus-OFF final ATE gap falls from **58.68 to 24.45 mm**, closing **58.34%** of that measured gap. Online ATE falls **10.22% OFF / 25.99% ON**, but ON remains worse there too.

The trade-offs matter:

* Final t-RPE worsens **10.57% OFF / 6.28% ON**; final r-RPE worsens **2.59% / 7.27%**.
* Online t-RPE worsens **6.04% OFF / 2.81% ON**; online r-RPE worsens **24.33% / 2.36%**.
* On SPX-3, this exact addition changed final OFF/ON ATE from **0.047570 / 0.066802** to **0.043810 / 0.041311 m**. ETH1 shows the same direction of ATE improvement, but **does not reproduce the OFF/ON ranking reversal**. Even SPX-3's added-noise ON case had worse RPE than its matched OFF case.

At the last accepted revision, median effective/reported Kimera rotation-trace ratios are **1.304 OFF / 2.450 ON**; translation-trace ratios are **1.765 / 2.696**. GLIM ratios are exactly 1. The fixed absolute addition therefore has a larger median proportional effect on the smaller ON matrices. These are trace summaries, not scalar optimizer weights; full dense matrices still determine factor whitening. Reported-versus-effective matrices and full ratio distributions are retained in the artifacts.

This intervention demonstrates sensitivity to how Kimera uncertainty is modeled, using one unchanged setting across two sequences. It does **not** establish that covariance contraction is the only cause, that the added variance is statistically calibrated, or that source/temporal/shared-IMU correlations have been removed. It also does not establish that a lower ATE means universally better local motion. ETH1 was already inspected extensively, so this is an untuned cross-sequence transfer check, not a previously unseen blind holdout study.

**Decision: keep the result as a partial, metric-dependent transfer. Do not adopt it as a universal production setting or increase the addition until ON wins.** The existing production defaults and runbooks remain unchanged. No next sequence or new value was run.

## 5. Validation and artifacts

Both replays completed successfully with identical accounting to their zero controls:

* 11,986 IMU events; 1,193 support states; 1,189 active GLIM IDs and 581 active Kimera IDs.
* 70,711 / 70,748 revision messages OFF/ON. The same 25 unsupported revision messages per mode remain unsupported; no new drop, duplicate or stale-revision outcome. Pending at end: 0.
* Final graph: **4,158 factors** = 1,189 GLIM + 581 Kimera + 1,192 Combined IMU + 1,192 WNOJ + 4 priors. New revisions replace existing IDs; they do not add concurrent copies of one ID.
* Support clock, causal dispatch, unsupported-belief log, source accounting and measurement-event log are byte-identical to baseline. Revision identity/slot/order/outcomes and the initial state are identical.
* All effective Kimera matrices are finite, exactly symmetric and positive definite; minimum eigenvalues **2.9061e-9 OFF / 2.7654e-9 ON**. The raw-source matrices were not overwritten.
* Actual Gaussian factor covariance differs from its supplied effective matrix by at most **8.14e-20**. Independently recomputed full-matrix residual whitening agrees to relative error below **1.77e-15**.
* All eight fused scoring masks agree. Recomputed zero-control metrics match retained values. All **105 pinned input/configuration/production files** have unchanged hashes.
* Receiver runtimes were **1,014.9 s OFF / 1,339.2 s ON**, with the cases running concurrently. These are unpaced diagnostic replay costs, not a new live-throughput benchmark.

No new receiver build or production test-suite run was necessary or claimed. The new C++ adapter-equivalence check and experiment-specific integrity checks passed. No new repeated-replay determinism claim is made; retained zero reconstructions were reused and reverified.

Artifacts: [experiment directory](/home/yeranis/results/gnssfgo_eth1_spx3_kimera_uncertainty_transfer_20261002/), [metrics](/home/yeranis/results/gnssfgo_eth1_spx3_kimera_uncertainty_transfer_20261002/metrics.json), [validation](/home/yeranis/results/gnssfgo_eth1_spx3_kimera_uncertainty_transfer_20261002/validation.json), [preflight](/home/yeranis/results/gnssfgo_eth1_spx3_kimera_uncertainty_transfer_20261002/preflight.json), and [linked-runtime hashes](/home/yeranis/results/gnssfgo_eth1_spx3_kimera_uncertainty_transfer_20261002/runtime_manifest.json).

New final trajectory hashes:

* [OFF](/home/yeranis/results/gnssfgo_eth1_spx3_kimera_uncertainty_transfer_20261002/off/trajectory.tum): `ae0cd2649dfd15cf9df6eeef2c188f6b092bbf7b009e9e6b3a15e21f9b129c8d`
* [ON](/home/yeranis/results/gnssfgo_eth1_spx3_kimera_uncertainty_transfer_20261002/on/trajectory.tum): `8d5229081643d98e1fb45ef21136bf94ee659a1012d3c5611736f4bb58ca964f`

## 6. Reproduction and repository state

Preparation and validation use the existing NumPy environment at `/home/yeranis/.local/share/pipx/venvs/evo/bin/python`. The unchanged receiver runs in `ubuntu-jammy-humble-nvidia`, after sourcing `/opt/ros/humble/setup.bash` and `/workspace/fork/install/setup.bash`. Its command is:

```bash
/home/yeranis/results/gnssfgo_spx3_live120_factor_audit_20260929/grandtour_ct_replay_factor_audit \
  --event-stream EFFECTIVE_EVENTS.csv --out FRESH_OUTPUT_DIRECTORY \
  --mode sources --timeline record --solver isam2 \
  --support-policy uniform_imu --support-period-ns 100000000 \
  --receiver-startup-samples 59
```

The exact executed arguments and runtimes are in `runs.json`. `prepare.py` performs the immutable-input checks and materializes the effective inputs; `check_adapter.py` compiles only the helper check; `run.py` launches exactly the two receiver cases; `score.py off` and `score.py on` evaluate the new outputs and retained controls; `validate.py` checks accounting, covariance application, masks and hashes. Scripts refuse to overwrite run/evaluation products: reproduce in a fresh artifact directory, not over these outputs.

Scoring used the existing `cbsms:noetic-cuda-glim` image (`sha256:7ad3854c699fb9a292aa5d42f8f890dfbd5dccb16633e043a93d135221418aaa`), `--pull never --network none`, read-only input mounts and a writable new artifact directory. The unchanged `score_grandtour_prism.py` and `score_grandtour_relative_pose.py` commands, exact bounds and paths are recorded in each mode's `*_scoring_validation.json`. Nothing was installed or downloaded.

No production source/configuration was edited and no commit was made. The pre-existing dirty tracked diff is unchanged (`69dd475a5338e94d4c489df9d2382b11e31fd51c523d13f39c9dd99605d2f4ef`); the worktree was not falsely made or reported clean. Final HEAD remains **`a842e7ef72e7a40f0cdfec826f089c66af364ae0`**.

**Fixed-setting transfer: ATE benefit confirmed on this ETH1 window; RPE trade-offs and a remaining CBS-ON fusion deficit remain.**
