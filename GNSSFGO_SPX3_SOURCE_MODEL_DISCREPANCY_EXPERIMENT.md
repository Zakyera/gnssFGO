# SPX-3 receiver-side source-model discrepancy experiment

Date: 2026-09-29. This is a **receiver-only offline replay** of the two frozen,
previously live 120 s SPX-3 classic-CBS receipt journals. GLIM, Kimera, their
source-to-source CBS exchange, and their native trajectories were **not rerun**.
The test asks whether a modest, fixed extra uncertainty for each source belief
changes the matched CBS-OFF versus CBS-ON gnssFGO result. It is not a new
production setting or a statistically calibrated covariance model.

## Frozen setup and implementation

The receiver remains full-history iSAM2 on its independent 100 ms support grid,
with ALL_RIGHT CPT7 Combined IMU, WNOJ `Qc=10000`, the same priors and 59-sample
startup. It still replaces factors by stable source-belief ID as revisions arrive.
There is no GNSS, covariance multiplication, NIS gate, robust loss, health
weighting, temporal re-pairing, or source-estimator change. The original live
source runs used SPX-3 Hesai, front stereo, and shared CPT7 with the documented
classic CBS protocol; their exact journals are the replay inputs.

The new default-zero receiver option is repeatable by source ID:

```text
--source-model-noise SOURCE_ID,ROTATION_SIGMA_RAD,TRANSLATION_SIGMA_M
```

At every insertion or stable-ID replacement, it forms one effective full
covariance in GTSAM Pose3 `[phi,rho]` tangent order:

```text
Sigma_effective = Sigma_reported
                + diag(sigma_phi² I3, sigma_rho² I3).
```

The reported 6×6 matrix is never overwritten; all its rotation/translation
cross terms and within-block off-diagonals are retained. Every exact or GP
interpolated source factor uses the same effective matrix. The saved final
active-factor audit records **both** complete 6×6 matrices. The saved residual
Mahalanobis diagnostic also uses the matrix actually passed to the factor.
The new option defaults to no addition, and configured IDs with no accepted
factor now fail rather than silently do nothing. This last ID guard was added
after the scored replays; it changes no factor math for either accepted journal.

The single additive setting was recorded in
[`predeclared_experiment.md`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_source_model_discrepancy_20260929/predeclared_experiment.md)
before any replay result. Each added *variance* is 10% of the corresponding
source/channel median latest-factor CBS-OFF reported trace divided by three,
from the retained source-uncertainty audit. This was an engineering pilot
choice, **not** an ATE-optimal or independently calibrated value:

| Actual journal source ID | σφ (rad) | σρ (m) |
|---|---:|---:|
| `glim_hesai` | 0.0000561019904579 | 0.000244015026860 |
| `kimera_front` | 0.0000450299900067 | 0.000601137810046 |

The same two values were applied to each respective source in both OFF and ON.
The frozen journal hashes are OFF
`f7a95731644f6031d4c9f6a2787c2f545391d35d242c35dae1e3a238cd93820d`
and ON
`a96521520f7f60cb4dbefb569d430ecb13748c56c1490b8860c4ca6dc53d1425`.
The four scored cases are OFF/ON with zero addition and OFF/ON with this one
addition. An initial pair of *incomplete, unscored* additive attempts used the
display names `glim`/`kimera` rather than these exact journal IDs. They were
stopped before results were inspected; their partial files remain in
`*_invalid_source_id`. The values above were not adjusted.

## Ground-truth evaluation

The unchanged existing evaluators and window `1730565903.588829994`–
`1730566022.688653946` were used. Prism ATE uses SE(3) alignment without
scale and the known CPT7→AP20 lever arm; it has 954 matches over 94.9 s in
five GT segments. IE-TC 1 s relative-pose RPE has 1,181 valid pairs in every
case. Thus the table uses the same physical support and scoring masks, but
prism ATE is **not** coverage of every instant of the 120 s run. “Online” is
the receiver's as-played trajectory; “final” is retrospectively revised after
later belief revisions and is not an online time-series result.

| CBS | Receiver setting | Output | ATE (m) | 1 s t-RPE (m) | 1 s r-RPE (°) | Full path (m) | Prism-matched path (m) |
|---|---|---|---:|---:|---:|---:|---:|
| OFF | zero / frozen | online | 0.069367 | 0.027762 | 1.750184 | 79.123 | 72.239 |
| ON | zero / frozen | online | 0.078541 | 0.047590 | 1.911003 | 81.866 | 74.801 |
| OFF | additive | online | 0.070616 | 0.027018 | 1.719477 | 79.018 | 72.204 |
| ON | additive | online | 0.061669 | 0.047372 | 1.888534 | 81.839 | 74.780 |
| OFF | zero / frozen | final | 0.047570 | 0.011439 | 0.380927 | 78.283 | 71.645 |
| ON | zero / frozen | final | 0.066802 | 0.012325 | 0.651236 | 78.465 | 71.831 |
| OFF | additive | final | 0.048888 | 0.011093 | 0.372128 | 78.292 | 71.650 |
| ON | additive | final | 0.048404 | 0.012437 | 0.636920 | 78.504 | 71.858 |

The prism reference path is 71.778 m on this mask for all eight rows. Native
source trajectories are a **different output** and remain exactly those of
the original live runs: OFF GLIM ATE 0.007046 m, OFF Kimera 0.655087 m,
ON GLIM 0.015811 m, ON Kimera 0.170926 m. This receiver replay neither
changes nor reevaluates them.

Under the additive setting, ON is 8.95 mm lower ATE than OFF online, whereas
it was 9.17 mm higher with zero addition. In the final retrospectively revised
graph, ON is only **0.48 mm lower ATE** than OFF, versus 19.23 mm higher at
zero. ON's own final ATE falls by 18.40 mm; OFF's rises by 1.32 mm. These are
single-window descriptive differences, not confidence intervals. Importantly,
ON still has worse 1 s RPE than OFF with the same setting: final translation
0.012437 versus 0.011093 m and rotation 0.636920 versus 0.372128°. Its
online t-RPE gap remains much larger (0.047372 versus 0.027018 m). The
additive model therefore **improves the global ATE ordering but does not solve
local relative accuracy**. The fused trajectories also remain substantially
less accurate in ATE than native GLIM on this window.

## How much uncertainty was actually added?

Median final active-factor trace ratios `effective/reported`, from the logged
full matrices, are:

| Mode/source | Rotation trace ratio | Translation trace ratio |
|---|---:|---:|
| OFF GLIM | 1.100 | 1.100 |
| OFF Kimera | 1.100 | 1.100 |
| ON GLIM | 1.233 | 1.110 |
| ON Kimera | 1.694 | 2.295 |

The *absolute* extra covariance is the same for a source in OFF and ON. It
matters proportionally more to Kimera under ON because its reported covariance
had contracted: ON Kimera's median reported translation trace was
`8.3723e-7 m²` versus OFF's `1.0841e-5 m²`. The extra term does not drive a
covariance toward zero; it increases it. A larger covariance reduces the
same residual's quadratic cost `rᵀ Σ_effective⁻¹ r` and information curvature
`Jᵀ Σ_effective⁻¹ J`. It is **not** a percentage ownership of the fused path.
Both sources were changed together, so this experiment does not isolate which
source/channel produced the ATE change. It also does not model GLIM–Kimera
cross-correlation or the common CPT7 information explicitly.

## Validation and reproducibility

The zero-addition online and final TUM files are byte-identical to the original
live receiver files in both modes. The four SHA-256 pairs (online, final) are:

| Case | Online | Final |
|---|---|---|
| OFF zero | `f6869fa6af040d724e98abb72f4b2ac028e03639ecc862681c954062d21be27c` | `5e06cf4c4e4af63de40c95b1cdbeaa905ad41954448359fdbaf73a8fa7cdb20e` |
| ON zero | `1df3780b7b55fab27adbc275e0d5d88a6be548a676c6d967654d91cf701415c3` | `64609f58be978689b0e10fb22349593fa683701b881bdfd096043272d784bc00` |
| OFF additive | `1d5982a017622a0b2fd088bf070703fa3381d97dc95e9f34354160b7c6faeee9` | `907c5f4eabff41cc7d8455288b1341ed0e8225d83ee016632d0fe080ea16ba86` |
| ON additive | `dbf21c3f72a4c5be3c721e55e5d4b1f5329cb5897f668e9953f68c2358d9a952` | `7a3607781de643dc394c8d45a988cc7a9cd0dc1bb5c216196e5803de57f8a4fe` |

Across all four cases the support clock, unsupported list, source accounting,
causal dispatch and stable-ID revision identities agree with each mode's frozen
baseline. OFF has 1,190 GLIM + 591 Kimera unique factors; ON has 1,190 + 593.
All have 1,195 support states and zero pending events. Each mode's zero and
additive runs have exactly matching source-reported 6×6 matrices and means
(same original journal); the effective matrix changes only by the specified
three rotation and three translation diagonal entries. All off-diagonal
entries compare exactly; the largest logged construction error is below
`1e-21`. The largest difference between the Gaussian factor's covariance and
the intended effective covariance is `1.13e-17`. The zero-control hashes prove
the default graph has no numerical regression.

The receiver and new focused test target build in the existing ROS Humble
workspace. Four new covariance/ID tests and 14 unchanged CT-factor/IMU-support/
causal-clock tests pass. A package-wide build was attempted but is blocked by
the pre-existing LiDAR target's missing
`visualization_msgs/msg/marker_array.hpp`; no production dependency or source
was altered to work around it. No ASan/UBSan or full-suite claim is made.
`git diff --check` passes. The gnssFGO research worktree was already dirty;
unrelated existing edits were preserved. There is no new commit. HEAD remains
`a842e7ef72e7a40f0cdfec826f089c66af364ae0`.

The four scored replays used receiver binary SHA-256
`61aff5524fca0f05440c3aabebaa4e69c8faf8f81d69bc2455328978ca11f15b`.
The final rebuild with the source-ID guard (no factor-math change) is
`2f188a661e87a63092271b2b325bed0fa3ab0471951f596c86a7a45f8ebf4530`.
The exact command for an additive case is the original replay command with:

```text
--source-model-noise glim_hesai,0.0000561019904579,0.000244015026860
--source-model-noise kimera_front,0.0000450299900067,0.000601137810046
```

The retained directory
[`gnssfgo_spx3_source_model_discrepancy_20260929`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_source_model_discrepancy_20260929/)
contains all four trajectories, summaries, full reported/effective covariance
audits, predeclared setting, scoring JSON, and invariant validation scripts/
results. These were headless offline receiver replays, **not** a test that the
modified receiver sustains real-time 1× GrandTour playback.

## Decision

This is a promising but **mixed** single-sequence result: the fixed additive
source-model uncertainty substantially improves CBS-ON fused ATE and narrowly
reverses the final OFF–ON ATE ranking, while ON still loses in short-horizon
translation and rotation RPE. It is insufficient evidence to adopt these
SPX-3-derived values as an official cross-sequence/live setting. The existing
saved production zero-addition configuration remains unchanged.
