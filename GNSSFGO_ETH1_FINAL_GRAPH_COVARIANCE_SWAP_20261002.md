# ETH-1 final graph covariance swap test

Date: 2026-10-02. This is the four-case, frozen-factor test authorized after the ETH-1 covariance and factor-interaction investigations. It uses the existing first-120-second classic CBS-OFF and CBS-ON recordings. No GLIM, Kimera, bag playback, CBS exchange, or live receiver experiment was rerun.

**Result: the recorded covariance change materially contributes to the fusion degradation in this ETH-1 window, but does not explain the entire difference.** Assigning ON covariances to the unchanged OFF graph increases ATE from **0.048166 to 0.099096 m**. Assigning OFF covariances to the unchanged ON graph reduces ATE from **0.107068 to 0.069228 m**. The latter remains worse than OFF and slightly worsens translation RPE. This is evidence about the complete covariance matrices in this graph, not proof of a universal scalar correction or a defect in Schur covariance extraction.

The [complete artifacts](/home/yeranis/results/gnssfgo_eth1_final_covariance_swap_20261002/) include source matching, isolated diagnostic builds, original and optimized trajectories, all factor residuals, optimization logs, evaluation masks, and the rejected reconstruction attempt. The accepted outputs are **offline, final retrospective receiver solutions**, not native trajectories or new live outputs.

## Four controlled cases

| Case | Frozen base graph and source means | Covariances assigned to both source families |
|---|---|---|
| A | OFF | OFF, unchanged control |
| B | ON | ON, unchanged control |
| C | ON | OFF donor matrices |
| D | OFF | ON donor matrices |

For C versus B, and D versus A, the source means, endpoint timestamps, factor keys, interpolation objects, support states, Combined IMU factors, WNOJ factors, and priors are identical. Only the two source families' Gaussian noise matrices change. The non-source factors are literally the same shared factor objects in the control and swapped graph.

This distinction matters: OFF and ON base graphs need not have identical IMU preintegration/interpolation contents just because they use the same raw IMU. Their construction can depend on the evolving source-influenced states. The test preserves those contents **within each base graph**; it does not substitute covariance during causal replay or claim that cross-base differences isolate source means alone.

The frozen operating point is Combined IMU, WNOJ `Qc=10000`, receiver ALL_RIGHT integration, independent 100 ms IMU support clock, 59 startup IMU samples, original priors, full-history iSAM2 reconstruction, full six-dimensional source factors, scale 1, and no GNSS, added source-model noise, gating, health weighting, or robust loss. Classic source-to-source CBS remains whatever was recorded; downstream stable-ID revisions retain their original replacement semantics.

## Exact source matching and covariance intervention

The inputs are the original journals under:

`/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live120_zero_20260930/{off,on}_receiver/events.csv`.

| Input | SHA-256 |
|---|---|
| OFF journal | `d75e2cf25ad413b2c4f9f93e5331b601a695d5cb996a6ab44d32d543010f49e0` |
| ON journal | `fe0054add31063df0d36be106411d44ad439f4e748d740a88d94da3663bc7a38` |

All 11,986 physical IMU rows are equal. The final accepted source information comprises **1,189 GLIM and 581 Kimera physical intervals**, exactly matched by `(source, from_timestamp_ns, to_timestamp_ns)`. The donor is the last recorded revision for that interval, not a same-number revision or a nearest-time match. OFF has 70,711 revision messages and ON 70,748; the original 25 unsupported messages remain excluded in each. No accepted interval is omitted or duplicated; pending beliefs at completion are zero.

The donor files preserve all 36 numeric covariance entries, including dense rotation–translation coupling. All donor matrices are finite, exactly symmetric at saved precision, and positive definite. Minimum full-matrix eigenvalues in the stored mixed-unit coordinates are `2.02971e-10` OFF and `2.16017e-10` ON. See [preflight.json](/home/yeranis/results/gnssfgo_eth1_final_covariance_swap_20261002/preflight.json).

The [existing relative factor](/home/yeranis/repos/gnssFGO/online_fgo/include/factor/odometry/GPInterpolatedDoublePose3BetweenFactor.h:115) computes the Pose3 local residual between the recorded mean and the relative pose of the two interpolated endpoints, in the existing `[phi,rho]` coordinates. Both runs use the same source/body and residual conventions. `NoiseModelFactor::cloneWithNewNoiseModel()` copies the factor and replaces only its noise model; the donor is installed with `Gaussian::Covariance(matrix, false)` to retain the dense matrix. There is no new residual, adjoint conversion, Schur complement, covariance scaling, or covariance recomputation.

This is a deliberate numerical covariance assignment in the existing residual coordinates. It is not a claim that the OFF uncertainty distribution, recentered on the ON mean, is a calibrated probability model.

For all 1,770 factors in each base, validation found:

- identical connected keys and factor dimension;
- maximum unwhitened residual difference **0** before optimization;
- maximum endpoint/support Jacobian difference **0**;
- maximum installed covariance reconstruction error **2.20229e-20**;
- unchanged non-source factor objects.

Each final graph contains 4,158 factors: 1,189 GLIM, 581 Kimera, 1,192 Combined IMU, 1,192 WNOJ, and four initial priors, on 1,193 support states.

## Common final optimization

All four cases receive the **same diagnostic batch Levenberg–Marquardt refinement**, including the unchanged controls. Each control and swap starts from its own base's exact original iSAM2 final estimate. The batch pass optimizes the existing objective; it does not reconstruct motion factors from the newly optimized states.

Parameters were declared before scoring: maximum 100 iterations, relative and absolute objective-change tolerances `1e-8`, remaining GTSAM legacy LM defaults, including initial damping `1e-5`. No solver setting was selected using ground truth. A/B converged in two iterations, C in four, and D in three; none reached the iteration limit. Refinement took 0.257–0.520 s per case after reconstruction. These are batch timings, not live-throughput measurements.

Consequently, A/B differ slightly from the previously published unrefined final receiver ATEs, `0.048359/0.107042 m`. The control refinements produce only **0.356 mm OFF and 0.110 mm ON RMS position-shape change**, after removing one common rigid transform. In comparison, the swaps produce **64.986 mm OFF and 44.454 mm ON** RMS shape change relative to their respective refined controls. Extra optimizer iterations are therefore not being mistaken for the covariance intervention.

## Matched trajectory metrics

The unchanged repository AP20 prism and IE-TC evaluators use the original bounds `1727775008.775065899`–`1727775127.674791098`, CPT7-to-prism lever arm, no-scale SE(3) ATE alignment, and 1 s RPE. Every case has exactly the same **654 prism timestamps** and **1,179 RPE intervals**. Prism ATE covers **64.9 aggregate seconds in five segments**, not continuous ground truth throughout the 120-second capture. IE-TC RPE uses the retained CPT7 reference and should not be regarded as statistically independent of CPT7 IMU.

| Case | Base / covariance | ATE m | 1 s t-RPE m | 1 s r-RPE deg | Full path m | GT-overlap estimate path m |
|---|---|---:|---:|---:|---:|---:|
| A | OFF / OFF | 0.048166 | 0.015598 | 0.414169 | 71.824096 | 47.912751 |
| B | ON / ON | 0.107068 | 0.018427 | 0.515559 | 71.928276 | 47.912359 |
| C | ON / OFF | 0.069228 | 0.018686 | 0.471202 | 71.921848 | 47.915213 |
| D | OFF / ON | 0.099096 | 0.017708 | 0.414278 | 71.760749 | 47.798251 |

Paths follow the evaluator's prism-equivalent trajectory convention. Matched GT path is `47.790475 m` for every case. Exact metrics and masks are in [metrics.json](/home/yeranis/results/gnssfgo_eth1_final_covariance_swap_20261002/metrics.json) and [evaluation](/home/yeranis/results/gnssfgo_eth1_final_covariance_swap_20261002/evaluation/).

The original native estimators were untouched. For context, their retained ATEs are GLIM `0.038007 OFF → 0.028978 m ON`, and native Kimera ROS odometry `0.541016 → 0.365236 m`, as documented in the [original live result](/home/yeranis/repos/gnssFGO/GNSSFGO_ETH1_LIVE120_CLASSIC_ZERO_SOURCE_MODEL_20260930.md). Those native sampling masks differ from the receiver mask. A–D are fusion diagnostics, not alternative definitions of native GLIM or Kimera.

## How the factors respond

The following are final objective costs, `0.5 rᵀΣ⁻¹r`, not percentages of trajectory influence and not calibrated NIS statistics. Source costs across different covariance assignments have different weighting; the physical residuals below help interpret them. IMU/WNOJ costs are directly comparable within each base because their factors are unchanged.

| Case | GLIM cost | Kimera cost | Combined IMU cost | WNOJ cost |
|---|---:|---:|---:|---:|
| A | 43,258.73 | 8,473.22 | 11,480.78 | 3,968.20 |
| B | 547,994.21 | 30,853.50 | 50,446.91 | 5,850.70 |
| C | 318,606.85 | 10,920.67 | 24,109.29 | 4,626.23 |
| D | 59,007.91 | 20,100.89 | 13,024.84 | 4,299.39 |

Four priors contribute a further cost below 1.5 in each case.

| Case | GLIM rho residual RMS mm | GLIM phi residual RMS deg | Kimera rho residual RMS mm | Kimera phi residual RMS deg |
|---|---:|---:|---:|---:|
| A | 7.212 | 0.016990 | 5.373 | 0.013664 |
| B | 19.362 | 0.034191 | 4.091 | 0.008092 |
| C | 20.022 | 0.058653 | 4.044 | 0.015355 |
| D | 7.089 | 0.013785 | 4.176 | 0.006874 |

These are **post-fit measurement residuals**, not errors against ground truth. Rho is the existing tangent translation coordinate, not a newly introduced Cartesian measurement.

The particularly useful controlled observation is A→D: the unchanged OFF source measurements are fitted more tightly, especially Kimera rotation, while the unchanged IMU/WNOJ factors are fitted less well and ATE approximately doubles. Thus better agreement with those source factors does not automatically mean a more accurate fused trajectory. B→C releases some of the source fit, substantially reduces IMU/WNOJ cost, and improves ATE and rotational RPE, but not translation RPE. The covariance assignment changes the graph's compromise; this is not just a lower or higher printed trace with no effect on the solution.

Full per-factor initial/final residuals, distribution summaries, and raw versus rigid-aligned trajectory movement are retained in [analysis.json](/home/yeranis/results/gnssfgo_eth1_final_covariance_swap_20261002/analysis.json) and the accepted case directories.

## Interpretation and limits

1. **The weighting explanation has direct experimental support here.** D increases ATE by **105.74%** with unchanged OFF source means and non-source factors. C reduces ON ATE by **35.34%** with unchanged ON source means and non-source factors.
2. C closes **64.24% of the numerical B−A ATE gap**, but remains **0.021061 m** above A. That gap-closure number is an effect size for this counterfactual, not a universal fraction of causation or a statistical variance decomposition.
3. This does **not** prove that universally smaller covariance trace is the mechanism. We swapped full matrices, including directional structure and coupling. Nor does it identify GLIM versus Kimera separately; both source families were swapped together.
4. It does not establish that the Schur complement is algebraically wrong, quantify missing source/common-information correlations, or separate remaining source-mean, motion/interpolation, and statistical-model effects. Different base graphs remain different.
5. OFF donor matrices require a counterfactual OFF source run. They are an explanatory diagnostic, **not an online covariance rule**, and do not justify a universal scale or per-sequence tuning. No new setting is frozen as a result.

## Reconstruction failure and accepted validation

The first isolated build used the current receiver source. OFF reconstructed the reference exactly. ON began with tiny numerical differences and eventually failed the declared reconstruction gate: maximum saved online position-coordinate difference was `0.003805914 m`, final position-coordinate difference `0.000603866 m`, and final quaternion-coordinate difference `0.000330816`. The gate was not relaxed to accept it. That preliminary `on/` directory is **rejected and never scored as B/C**.

Only ON reconstruction was repeated, using the previously identity-validated archived receiver source from the September-29 factor audit, with the new final covariance diagnostic compiled in a separate translation unit. The accepted `on_archived/` output, before refinement, is byte-identical to the original in all eleven checked products: final/online trajectories, final/online states, support clock, support accelerations, model covariance export, belief residuals, source accounting, causal dispatch, and unsupported messages. OFF passes the same eleven checks plus its source-model covariance export. Maximum accepted reconstruction state difference is therefore **zero** in both modes. The cause of the first build's numerical mismatch was not pursued as a separate algorithm investigation.

The [final validation ledger](/home/yeranis/results/gnssfgo_eth1_final_covariance_swap_20261002/validation.json) records accepted paths, all four trajectory hashes, convergence, identical evaluation masks, the rejected attempt, and input integrity. Covariance donor SHA-256 values are OFF `55cc9875356055057846cfe17ae4345d19ca7aac919f36a865c747c35e45ecfb` and ON `2f72216778c502b23db990880e87caa492991802b654ccb47af18ebe3311cbb7`.

## Reproduction and repository status

Artifacts are in `/home/yeranis/results/gnssfgo_eth1_final_covariance_swap_20261002/`. `prepare.py` validates/extracts donor matrices. `build_diagnostic.py` builds the isolated current-source copy used for accepted A/D; `build_archived.py` builds the archived copy plus helper used for accepted B/C. Both reuse the established Release CMake flags and link command in `workspaces/gnssfgo_ct_ws/build/online_fgo/src/offline_process`. Exact compiler/link arguments and hashes are saved in the two build manifests.

Inside the existing `ubuntu-jammy-humble-nvidia` container, source `/opt/ros/humble/setup.bash` and `/workspace/fork/install/setup.bash`. The accepted run commands have this form; use fresh output directories:

```bash
GNSSFGO_SWAP_DONOR=/home/yeranis/results/gnssfgo_eth1_final_covariance_swap_20261002/on_donors.csv \
  /home/yeranis/results/gnssfgo_eth1_final_covariance_swap_20261002/grandtour_final_covariance_swap \
  --event-stream /home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live120_zero_20260930/off_receiver/events.csv \
  --out FRESH_OFF_OUTPUT --mode sources --timeline record --solver isam2 \
  --support-policy uniform_imu --support-period-ns 100000000 --receiver-startup-samples 59

GNSSFGO_SWAP_DONOR=/home/yeranis/results/gnssfgo_eth1_final_covariance_swap_20261002/off_donors.csv \
  /home/yeranis/results/gnssfgo_eth1_final_covariance_swap_20261002/grandtour_archived_final_covariance_swap \
  --event-stream /home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live120_zero_20260930/on_receiver/events.csv \
  --out FRESH_ON_OUTPUT --mode sources --timeline record --solver isam2 \
  --support-policy uniform_imu --support-period-ns 100000000 --receiver-startup-samples 59
```

Each executable emits its untouched reconstructed trajectory plus `control.tum` and `swapped.tum`. Accepted OFF reconstruction plus diagnostics took 1,085.10 s; the archived ON reconstruction took 1,072.40 s. The rejected ON attempt is recorded separately and does not constitute another scientific case.

`score.py` calls the unmodified repository evaluators in the existing local `cbsms:noetic-cuda-glim` image with networking disabled and the project/data mounted read-only. `analyze.py` and `finalize.py` check reconstruction identity, unchanged production source, factor invariants, common masks, and residual/movement summaries. Python syntax checks passed. No full production rebuild or ASan suite is claimed: the only executable changes are isolated diagnostic copies outside the repository.

Production `GrandTourCsvReplay.cpp` is unchanged from the entry SHA-256 `82c17ce0ac3da3de209ace00c30b7cc6c3acbce83ad68ffcf99a38f891688d19`. No source estimator settings, official runbook, production binary, or frozen input was changed. The repository was already dirty; those changes were preserved. No commit was created. HEAD remains `a842e7ef72e7a40f0cdfec826f089c66af364ae0`.

**Stopped after the four-case comparison. No covariance correction or follow-up experiment was implemented.**
