# ETH-1 receiver diagnostic: Kimera rotation–translation covariance coupling

## Scope

This is a two-case replay of the **frozen first 120 s ETH-1** classic CBS-OFF
and CBS-ON receiver journals. The native GLIM and Kimera source runs were not
repeated. The goal was to determine whether the covariance coupling *inside*
each 6D Kimera belief accounts for the large CBS-ON ATE loss of the full
gnssFGO graph. The matching full-covariance results are the previously
validated [live receiver](/home/yeranis/repos/gnssFGO/GNSSFGO_ETH1_LIVE120_CLASSIC_ZERO_SOURCE_MODEL_20260930.md)
and its byte-identical [default-path regression](/home/yeranis/repos/gnssFGO/GNSSFGO_ETH1_KIMERA_CHANNEL_ABLATION_20260930.md).

The diagnostic changes **only** Kimera's installed receiver covariance:

```text
Original:   [ Sigma_phi_phi  Sigma_phi_rho ]
            [ Sigma_rho_phi  Sigma_rho_rho ]

Diagnostic: [ Sigma_phi_phi       0       ]
            [      0        Sigma_rho_rho ]
```

Both complete dense 3×3 marginal blocks, all six residual/Jacobian rows, the
Kimera aligned mean, and every GLIM factor remain unchanged. The two blocks
are positive definite because they are principal blocks of the validated
positive-definite source covariance. This is a **receiver diagnostic**, not a
claim that the discarded source cross covariance is incorrect or a proposed
production setting. It does not remove correlation *between* GLIM and Kimera
or between overlapping source beliefs.

## Frozen execution and validation

The input is exactly the original `off_receiver/events.csv` or
`on_receiver/events.csv`, SHA-256 respectively
`d75e2cf25ad413b2c4f9f93e5331b601a695d5cb996a6ab44d32d543010f49e0`
and `fe0054add31063df0d36be106411d44ad439f4e748d740a88d94da3663bc7a38`.
The ON journal contains classic bidirectional source CBS; this replay does not
change that exchange. The 100 ms independent support clock, 59-sample
initialization, ALL_RIGHT CPT7 Combined IMU, WNOJ `Qc=10000`, priors,
full-history iSAM2, exact physical-time interpolation, causal arrival order,
stable-ID revision replacement and source covariance scale 1 are unchanged.
No GNSS, source-model addition, gate, robust loss or health weighting was used.

The existing GrandTour receiver executable gained one opt-in selector,
`--kimera-covariance full|blockdiag`, in
[GrandTourCsvReplay.cpp](/home/yeranis/repos/gnssFGO/online_fgo/src/offline_process/GrandTourCsvReplay.cpp).
`full` is the default. In `blockdiag`, the factor still has dimension six and
uses the existing relative-pose factor and its interpolation/Jacobian path.
`Gaussian::Covariance(..., false)` preserves the dense off-diagonal entries
*within* each 3×3 marginal. On every accepted Kimera insertion or revision,
the installed Gaussian matrix is compared with the declared diagnostic
matrix; the run fails if the six-dimensional factor or matrix is wrong, or if
no nonzero cross entry was removed. The Release executable SHA-256 is
`0f175e4db81acb3624442ed7b3b232201e1b876676f2c148d0a747f009e706fa`.

Both runs used the existing ROS Humble container, sourced
`/opt/ros/humble/setup.bash` and `/workspace/fork/install/setup.bash`, and ran:

```text
grandtour_ct_replay --event-stream ORIGINAL_EVENTS.csv --out FRESH_OUTPUT \
  --mode sources --timeline record --solver isam2 \
  --support-policy uniform_imu --support-period-ns 100000000 \
  --receiver-startup-samples 59 --kimera-covariance blockdiag
```

The [artifact directory](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_kimera_covariance_blockdiag_20260930/)
contains both complete receiver outputs, the exact scoring and validation
scripts, [metrics.json](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_kimera_covariance_blockdiag_20260930/metrics.json),
and [validation.json](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_kimera_covariance_blockdiag_20260930/validation.json).
Both runs accepted 1,189 GLIM and 581 Kimera unique belief IDs and preserved
the original 70,711 OFF / 70,748 ON revision messages. There were 1,193
support states and zero pending beliefs at completion. In each mode the
support clock, source accounting, causal dispatch, unsupported-belief list,
and original source covariance export are byte-identical to the frozen
receiver. Maximum installed-factor covariance error was `5.43e-20` OFF and
`2.71e-20` ON. The maximum removed phi–rho covariance entry was
`1.0725e-6` OFF and `4.9712e-7` ON in covariance units.

The original GrandTour AP20 prism and IE-TC scorers were used with the
identical GPS bounds `1727775008.775065899`–`1727775127.674791098`,
CPT7-to-prism lever arm, no-scale SE(3) alignment, and 1 s RPE. All new
online/final outputs have the **same 654 matched prism timestamps** and
1,179 valid RPE pairs as their original corresponding full-covariance
receivers. Prism coverage is 64.9 aggregate seconds in five segments of the
120 s run; it is not continuous ground truth.

## Matched receiver results

"Online" is the causal as-played trajectory. "Final" is the retrospectively
revised graph after all source messages arrived. Both are gnssFGO receiver
products, not native GLIM or Kimera trajectories.

| CBS | Kimera covariance in receiver | Output | ATE (m) | 1 s t-RPE (m) | 1 s r-RPE (deg) | Full path (m) |
|---|---|---|---:|---:|---:|---:|
| OFF | Original full 6×6 | Online | 0.051511 | 0.031413 | 0.575945 | 72.336 |
| OFF | Block-diagonal diagnostic | Online | 0.050830 | 0.031345 | 0.573095 | 72.335 |
| ON | Original full 6×6 | Online | 0.122472 | 0.054332 | 0.942747 | 76.687 |
| ON | Block-diagonal diagnostic | Online | 0.116158 | 0.054373 | 0.941355 | 76.774 |
| OFF | Original full 6×6 | Final | 0.048359 | 0.015597 | 0.414167 | 71.824 |
| OFF | Block-diagonal diagnostic | Final | 0.046491 | 0.015417 | 0.414928 | 71.826 |
| ON | Original full 6×6 | Final | 0.107042 | 0.018430 | 0.515790 | 71.928 |
| ON | Block-diagonal diagnostic | Final | 0.099185 | 0.018478 | 0.516184 | 71.960 |

For context, the original ON GLIM-only *receiver* had ATE `0.068069 m`
online and `0.036467 m` final. With both Kimera channels and block-diagonal
within-belief covariance, the ON receiver remains far worse on ATE. Native
source ATE in the original live run improved OFF→ON: GLIM
`0.038007→0.028978 m` (655 prism matches) and Kimera ROS odometry
`0.541016→0.365236 m` (315 matches). Native and receiver masks differ, so
these native metrics are context rather than direct preservation scores.

Removing the Kimera phi–rho covariance cross block lowers ON ATE by
`6.314 mm` online and `7.857 mm` final; OFF changes are `0.681 mm` and
`1.868 mm`. The ON–OFF ATE gap shrinks only `7.94%` online and `10.20%`
final. Thus the within-belief covariance coupling has a measurable effect,
but **it is not the main explanation** for the ON full-fusion ATE loss in this
window. The previous rotation-only and rho-only ablations showed that neither
channel alone reproduces the loss. Their remaining combined effect can arise
from jointly enforcing both residual channels, their source means, the
independent-factor receiver assumption, overlap among beliefs, and nonlinear
coupling with GLIM/IMU/WNOJ. This diagnostic does not separate those remaining
mechanisms or certify a new covariance model.

## Convergence check and limits

The existing [fresh-factor ETH-1 audit](/home/yeranis/repos/gnssFGO/GNSSFGO_ETH1_COVARIANCE_AND_FACTOR_INTERACTION_AUDIT.md)
linearized the **unchanged original full-covariance** final graphs at their
iSAM2 estimates and solved the full normal equations. The relative linear
solve residual was `1.55e-14` OFF and `3.50e-14` ON. The unapplied joint
Gauss–Newton step, after removing a common rigid motion, had RMS position
shape change `0.356 mm` OFF and `0.109 mm` ON (ON maximum `0.338 mm`).
That is much smaller than the approximately `59 mm` original final ON–OFF
ATE gap and the `71 mm` ON full-versus-GLIM-only gap. It argues against
simple failure to take one more local solver step as the primary explanation.
It is a **local linearized check**, not a global nonlinear optimum proof;
this stage did not change iSAM2 iterations or run an optimizer sweep.

The GT covers only part of ETH-1 and the result is one sequence/window. A
smaller ATE after discarding true covariance cross terms would not justify
deploying block-diagonal covariance. Correlation between source estimates and
the receiver's common IMU remains unmeasured here.

gnssFGO HEAD remained `a842e7ef72e7a40f0cdfec826f089c66af364ae0`.
The worktree was already dirty; no new commit was made. `git diff --check`
passes. Frozen source runs, journals, default receiver results, and their
evaluation artifacts were not overwritten.
