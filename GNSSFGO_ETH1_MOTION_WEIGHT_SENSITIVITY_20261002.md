# ETH1 matched IMU and WNOJ weight sensitivity

Date: 2026-10-02. This authorized test asks whether making the fused trajectory
more strongly driven by GLIM and Kimera, by weakening the IMU and WNOJ
penalties equally, recovers a CBS-ON advantage over CBS-OFF.

**It does not recover that advantage in this recorded ETH1 window.** Weaker
motion penalties slightly improve OFF ATE, but leave ON ATE worse and degrade
relative accuracy, particularly ON rotation. The graph fits GLIM more closely
at the expense of Kimera and motion consistency. At 0.01 motion weight, the
ON angular-rate state develops large excursions despite numerical convergence.
No new production setting is selected.

These are **six offline final nonlinear graph diagnostics** using the existing
first-120-second ETH1 OFF/ON recordings. No GLIM, Kimera, bag playback or live
receiver experiment was rerun. Original source trajectories remain unchanged.
The [artifacts](/home/yeranis/results/gnssfgo_eth1_motion_weight_sensitivity_20261002/)
contain the predeclared [protocol](/home/yeranis/results/gnssfgo_eth1_motion_weight_sensitivity_20261002/TEST_PROTOCOL.md),
isolated build, run commands, reconstructed trajectories, six diagnostic
trajectories, residuals, Jacobians, numerical checks and evaluator outputs.

## Exact intervention

For each original OFF or ON graph, minimize

`J_GLIM + J_Kimera + lambda * J_CombinedIMU + lambda * J_WNOJ + J_initial_priors`

at exactly `lambda = 1, 0.1, 0.01`, selected before scoring. The same weights
apply to both modes. Smaller lambda means **weaker motion penalties**. It does
not mean a smaller motion covariance or a known percentage of trajectory
influence.

Each complete motion factor's Gaussian square-root information is multiplied
by `sqrt(lambda)`, equivalent to covariance divided by lambda. This includes
the Combined IMU bias-evolution residuals, not just its pose/velocity rows.
All source factors and initial priors retain their original shared objects.
Source covariance scale remains 1. There is no covariance donor swap, source
scaling, gating, robust loss, health weighting or GNSS factor.

The recorded classic source CBS exchange and downstream stable-ID revision
replacement are unchanged. The frozen receiver uses ALL_RIGHT integration,
59 initialization samples, independent 100 ms IMU-driven support, Combined IMU,
WNOJ with stored `Qc=10000`, and full-history iSAM2 reconstruction. This test
weakens the WNOJ **factor penalty** without rebuilding endpoint interpolation
objects using a different Qc. Preintegrated IMU contents, support accelerations,
physical timestamps, factor keys and source endpoint interpolation remain fixed
within each base graph. OFF and ON base graphs can differ through their
original state-dependent construction; we do not assert that their entire
non-source contents are identical to each other.

Each final graph has 1,189 GLIM, 581 Kimera, 1,192 Combined IMU, 1,192 WNOJ
factors and four priors on 1,193 support states. Every setting starts from its
own mode's original final iSAM2 estimate, never the preceding weighted result.
All six use the same diagnostic batch Levenberg–Marquardt refinement as the
[previous covariance test](/home/yeranis/repos/gnssFGO/GNSSFGO_ETH1_FINAL_GRAPH_COVARIANCE_SWAP_20261002.md):
maximum 100 iterations, relative and absolute objective-change tolerances
`1e-8`, remaining legacy defaults. The production optimizer is not modified.

## Original GLIM and Kimera outputs

These are the retained **original estimator trajectories**, not single-source
gnssFGO reconstructions. They were neither rerun nor reoptimized here.

| Mode | Original estimator | ATE m | 1 s t-RPE m | 1 s r-RPE deg | Full path m |
|---|---|---:|---:|---:|---:|
| OFF | GLIM | 0.038007 | 0.027251 | 0.351298 | 72.532760 |
| ON | GLIM | 0.028978 | 0.033387 | 0.429674 | 73.217407 |
| OFF | Kimera ROS odometry | 0.541016 | 0.045950 | 0.541482 | 69.827659 |
| ON | Kimera ROS odometry | 0.365236 | 0.030284 | 0.571374 | 71.429407 |

Provenance is the [original ETH1 live report](/home/yeranis/repos/gnssFGO/GNSSFGO_ETH1_LIVE120_CLASSIC_ZERO_SOURCE_MODEL_20260930.md).
Native GLIM has 655 prism matches and Kimera 315, versus the fused trajectory's
654. Native versus fused ATE is therefore not an identical-sampling comparison.
Within each estimator's OFF/ON comparison the existing evaluation is retained.
Native ATE improves for both sources, but their RPE changes are not uniformly
beneficial.

## Matched fused results

These are **final retrospectively revised and batch-refined fused solutions**,
not new online outputs. The unchanged weight-1 controls reproduce the previous
diagnostic controls byte-for-byte. They differ slightly from the historical
unrefined final iSAM2 ATEs of 0.048359 and 0.107042 m because of the common
final batch refinement.

| Mode | Motion weight | ATE m | 1 s t-RPE m | 1 s r-RPE deg | Full path m | GT-overlap estimate path m |
|---|---:|---:|---:|---:|---:|---:|
| OFF | 1 | 0.048166 | 0.015598 | 0.414169 | 71.824096 | 47.912751 |
| ON | 1 | 0.107068 | 0.018427 | 0.515559 | 71.928276 | 47.912359 |
| OFF | 0.1 | 0.046548 | 0.017537 | 0.493692 | 71.861310 | 47.933452 |
| ON | 0.1 | 0.110879 | 0.020148 | 1.010907 | 72.031020 | 47.967973 |
| OFF | 0.01 | 0.045399 | 0.018663 | 0.720219 | 71.935065 | 47.968608 |
| ON | 0.01 | 0.108654 | 0.042251 | 3.612936 | 73.476067 | 48.698655 |

All six use the unchanged AP20 prism and IE-TC evaluators, CPT7-to-prism lever
arm, no-scale SE(3) ATE alignment and original bounds
`1727775008.775065899–1727775127.674791098`. The 654 prism timestamps and 1,179
one-second RPE intervals match exactly across cases. ATE covers **64.9 aggregate
seconds in five GT segments**, not continuous full-window truth. Matched GT path
is 47.790475 m. Paths use the evaluator's prism-equivalent convention. IE-TC
RPE shares CPT7 information and is not an independent IMU reference.

ON remains worse than matched OFF in all three trajectory metrics at every
tested weight. Its ATE disadvantage changes from 0.058902 m to 0.064331 m and
0.063255 m. The 0.01 ON rotational RPE is about seven times its own weight-1
value. The slight OFF ATE improvement is accompanied by worse OFF t-RPE and
r-RPE; it does not justify adopting the weaker setting.

Exact results are in [metrics.json](/home/yeranis/results/gnssfgo_eth1_motion_weight_sensitivity_20261002/metrics.json).

## What changes inside the graph

The following are source residuals at the optimized estimate, **not errors
against GT**. Rho is the existing SE(3) tangent translation coordinate. Source
noise matrices are unchanged, so their objective costs are directly comparable
across weights within a mode.

| Mode | Weight | GLIM rho RMS mm | GLIM phi RMS deg | Kimera rho RMS mm | Kimera phi RMS deg |
|---|---:|---:|---:|---:|---:|
| OFF | 1 | 7.212 | 0.016990 | 5.373 | 0.013664 |
| OFF | 0.1 | 5.840 | 0.013029 | 6.186 | 0.012940 |
| OFF | 0.01 | 4.873 | 0.010477 | 6.730 | 0.012943 |
| ON | 1 | 19.362 | 0.034191 | 4.091 | 0.008092 |
| ON | 0.1 | 15.025 | 0.025383 | 6.762 | 0.012803 |
| ON | 0.01 | 9.578 | 0.018431 | 6.622 | 0.012217 |

For ON, the GLIM objective cost falls `547994 → 319149 → 113310`, while Kimera's
cost changes `30853 → 75005 → 60656`. Their combined cost decreases, but this
is **not** improved agreement with both sources simultaneously. In this test
the optimizer fits GLIM more closely while sacrificing some Kimera agreement.
OFF exhibits the same source-cost trade-off: GLIM `43259 → 27401 → 18610`,
Kimera `8473 → 9349 → 10192`.

To compare motion consistency fairly, evaluate its residuals in the **original
unscaled covariance metric**, rather than mistaking a smaller applied weight
for an improvement:

| Mode | Weight | Original-metric IMU cost | Original-metric WNOJ cost |
|---|---:|---:|---:|
| OFF | 1 | 11480.8 | 3968.2 |
| OFF | 0.1 | 60791.7 | 7819.9 |
| OFF | 0.01 | 325813.0 | 28328.8 |
| ON | 1 | 50446.9 | 5850.7 |
| ON | 0.1 | 776429.2 | 35288.8 |
| ON | 0.01 | 8704065.7 | 592366.9 |

These are post-fit objective diagnostics, not calibrated NIS statistics or
percentages of trajectory influence. The graph is visibly making the intended
trade: lower combined source cost, much worse motion consistency. That trade
does not yield the proposed ON trajectory benefit.

## State excursions and numerical behavior

All six optimizations report convergence in 2–7 iterations, remain finite,
decrease their respective objectives and avoid the 100-iteration limit.
The weakest setting nevertheless has important state excursions:

| Mode | Weight | Angular-rate RMS rad/s | Maximum angular-rate norm rad/s | Maximum accel-bias norm m/s² |
|---|---:|---:|---:|---:|
| OFF | 1 | 0.436 | 1.861 | 0.191 |
| OFF | 0.1 | 0.618 | 2.711 | 0.204 |
| OFF | 0.01 | 1.039 | 5.092 | 0.229 |
| ON | 1 | 0.560 | 2.035 | 0.324 |
| ON | 0.1 | 1.219 | 5.468 | 0.469 |
| ON | 0.01 | 4.477 | 22.376 | 0.545 |

These are the estimated `w` angular-rate states, not raw gyro readings. ON at
0.01 has an RMS `w` change of 4.352 rad/s from baseline, maximum gyro-bias norm
0.003247 rad/s, and RMS velocity change 0.06795 m/s. Position-shape RMS change
after removing one rigid displacement is only 0.01457 m, illustrating why
position ATE alone would miss substantial deterioration elsewhere. No new
reference-based physical plausibility threshold was fitted after observing
these values.

Each final Jacobian is 49,974 by 21,474, with no zero-norm column. The
column-normalized gradient infinity norm divided by residual norm ranges
`1.38e-8–8.72e-6`. Identically seeded sparse normal-equation probe solves have
relative residual `5.10e-16–5.54e-16` and solution relative error
`3.69e-9–2.42e-8`. These checks show no obvious numerical solve breakdown;
they are not a global condition number, a covariance calibration test or a
proof of global optimality. All state/residual distributions and trajectory
changes are retained in [analysis.json](/home/yeranis/results/gnssfgo_eth1_motion_weight_sensitivity_20261002/analysis.json).

## Validation and provenance

- Both reconstructed original graphs reproduce all eleven checked original
  trajectory/state/support/accounting/dispatch products byte-for-byte. Both
  weight-1 trajectories also reproduce the prior batch controls byte-for-byte.
- Every weaker graph retains 1,774 unchanged source/prior objects and clones
  exactly 2,384 motion factors. Raw residual and Jacobian differences are zero.
  Maximum relative whitening error is `3.10e-16`, relative covariance scaling
  error `4.39e-16`, and source covariance reconstruction error `2.21e-20`.
- All six evaluation masks match. All original source captures, journal data,
  profiles and GT inputs covered by the historical manifest match their hashes.
  **The historical 76-entry manifest is not wholly current:** 75 match; its
  mutable `workspaces/gnssfgo_ct_ws/build/online_fgo/src/offline_process/grandtour_ct_replay`
  entry differs. That build file's modification/change times are September 30,
  before this task, and it was not executed or linked as the receiver here.
  Its current SHA-256 is
  `0f175e4db81acb3624442ed7b3b232201e1b876676f2c148d0a747f009e706fa`.
  The actual archived receiver object is separately pinned in the new
  preflight/build manifest; byte-identical reconstructions validate its result.
  The stale entry is retained explicitly in
  [frozen_input_integrity.json](/home/yeranis/results/gnssfgo_eth1_motion_weight_sensitivity_20261002/frozen_input_integrity.json),
  not silently marked equal. This task did not rebuild the production executable.
- Production receiver source and the repository's existing tracked diff are
  unchanged. The source SHA-256 remains
  `82c17ce0ac3da3de209ace00c30b7cc6c3acbce83ad68ffcf99a38f891688d19`.
  Python syntax checks passed. No full production test-suite or sanitizer run
  is claimed for this isolated diagnostic.

`build.py` compiles only the diagnostic helper and links the already validated
archived receiver object from the prior test. Its historical function symbol
`RunFrozenSwap` is only the existing hook at the final graph; this experiment
does not perform a source covariance swap. `run.py` reconstructs OFF and ON
once each and applies all three settings. `score.py`, `analyze.py` and
`finalize.py` produce the metrics and validation ledger. Exact compiler, linker
and receiver arguments are saved in the artifact manifests.

Inside the existing `ubuntu-jammy-humble-nvidia` environment, source
`/opt/ros/humble/setup.bash` and `/workspace/fork/install/setup.bash`, then run
the copied `build.py` and `run.py` in a fresh artifact directory. The receiver
arguments are the original event stream, `--mode sources --timeline record
--solver isam2 --support-policy uniform_imu --support-period-ns 100000000
--receiver-startup-samples 59`. Never overwrite the retained outputs. Scoring
uses the existing local `cbsms:noetic-cuda-glim` image with networking disabled
and read-only project/data mounts. Reconstruction took 1125.95 s OFF and
1315.91 s ON, run concurrently. Each final optimization took 0.26–0.90 s;
neither number is a new live-throughput result.

No commit was created. HEAD remains
`a842e7ef72e7a40f0cdfec826f089c66af364ae0`. The worktree was already dirty and
was preserved; the new report is not a claim of a clean worktree.

## Conclusion and stop

The tested hypothesis is not supported on this window: reducing the combined
IMU/WNOJ influence by 10 or 100 times does not reveal an ON fusion improvement.
The motion terms were contributing useful regularization, especially for
rotation. Stronger relative influence of the source beliefs improves their
combined objective fit, but does not make their compromise more accurate.

This does not prove CBS cannot improve fusion, identify IMU versus WNOJ
separately, establish a universal setting, or explain all source correlation
and belief-representation effects. It rules out this proposed simple remedy
over the tested range for this frozen experiment. No further weight was tried,
no source setting was changed, and no official configuration was updated.
