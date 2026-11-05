# ETH1 full sequence: reference CBS-OFF / CBS-ON comparison

Saved at the user's request on 2026-10-21. **CP-K23 is the completed full ETH1 reference comparison.** Refer to this report and its archived run when interpreting or reproducing this result. Older partial runs and older solver configurations are separate evidence.

Both captures, all three validation gates per mode, and evaluation completed. CBS-ON improved native GLIM, native Kimera and their live fusion in this pair. This is one matched experimental pair without parameter tuning.

## Frozen experiment

- Mission: GrandTour ETH1 `2024-10-01-11-29-55`.
- Selected-topic window: offset 0, duration 346.32 s, through the final Hesai scan; 0.20x headless playback and 25 s drain. Exact record bounds: 1727775007855396309 to 1727775354175396309 ns.
- GLIM: Hesai `/boxi/hesai/points`, CPT7 `/boxi/cpt7/imu`, 5 s lag, CP-K16a MAIN GPU batching and CP-I1/CP-I3 all-LOCAL full covariance for every belief version; corrected MAIN means.
- Kimera: Alphasense front-left/front-right stereo, shared CPT7, 25 keyframes; CP-K9 private build, `CPK9_PRIOR_REFRESH=1`, `FL4Q_ENABLE=0`, `FL4M_RETIREMENT=remove`; native source IMU override unset.
- Source exchange: classic bidirectional `active_window_temporary`; 0.20 s belief horizon, one pair per update, stride 1, receive delay 0; Gaussian dense 6x6 scale 1 in both directions. Native-IMU mean alignment retains the existing mean-only covariance alignment approximation.
- Receiver: CP-K19, 7 s fixed-lag `source_wnoa`, incremental publication, independent 100 ms supports; no central IMU or GNSS factors. Sources still consume CPT7 internally. State is pose, world velocity and body angular rate. Qc variances remain `[.00375,.00375,.00375,.015,.015,.015]`; prior standard deviations remain pose `1e-5`, velocity `10`, angular rate `3`.
- Downstream stable-ID revisions replace active factors. `seal_at_absorption` records revisions of known absorbed factors as expired without mutating the solver; their last accepted information remains in the prior. Unknown first arrivals referencing expired supports still fail.
- No robust loss, covariance scaling, health weighting, tuning, default promotion or fresh saved-journal replay.

Actual binaries, copied profiles, launch chain and 2,391 protected hashes are retained in the run directory linked below. Use these pinned artifacts rather than rebuilding defaults.

## Original trajectories and live fusion

ATE uses every saved pose across the journey. RPE is one-second RMSE on supported intervals. Path lengths refer to the evaluator's prism lever arm. Reference: `/boxi/inertial_explorer/tc/odometry` from the mission's IE-TC bag. IE-TC shares CPT7 and is not independent prism ground truth. ATE uses per-trajectory rigid SE(3) alignment without scale; relative errors do not use that global alignment.

| Trajectory | CBS | ATE RMSE m | Translation RPE m | Rotation RPE deg | Path m |
|---|---|---:|---:|---:|---:|
| Original GLIM | OFF | 0.039683 | 0.020328 | 0.349810 | 203.082664 |
| Original GLIM | ON | 0.037421 | 0.016090 | 0.329321 | 203.093810 |
| Original Kimera ROS odometry | OFF | 4.496229 | 0.440356 | 0.567768 | 196.006964 |
| Original Kimera ROS odometry | ON | 0.340930 | 0.019842 | 0.559695 | 200.644192 |
| Fusion ONLINE, published during playback | OFF | 0.166126 | 0.062154 | 2.765685 | 237.711338 |
| Fusion ONLINE, published during playback | ON | 0.103489 | 0.045039 | 2.635882 | 225.876905 |

Reference path is approximately 201.478 m at dense output timestamps and 201.031 m at Kimera's saved timestamps.

| Original/online output | ATE reduction ON vs OFF | Translation RPE reduction | Rotation RPE reduction |
|---|---:|---:|---:|
| GLIM | 5.70% | 20.85% | 5.86% |
| Kimera | 92.42% | 95.49% | 1.42% |
| Fusion ONLINE | 37.70% | 27.54% | 4.69% |

Kimera's aligned position error p95 falls from 10.0588 m to 0.5375 m. Fusion ONLINE p95 falls from 0.2492 m to 0.1692 m.

## Coverage and matched timestamps

- GLIM: 3,455 poses each mode, 345.399612 s saved span.
- Kimera: 1,492 OFF / 1,493 ON poses, 342.598092 s span; known 3.2 s startup gap and approximately 5.1 s keyframe intervals near the stationary tail. The frontend processed 3,460 post-startup frames. Sparse keyframes do not indicate camera loss. Every actual saved pose is scored for ATE.
- Fusion: 3,456 ONLINE and 3,456 finalized poses per mode, 345.5 s span.
- One-second RPE skips unsupported interpolation. Estimated-pose interpolation is limited to 0.5 s; reference interpolation to 0.05 s. Path lengths connect actual samples and include chords across gaps.
- The additional common mask contains 2,970 timestamps: 296.699717 s supported across a 298.299716 s envelope, in three segments. It excludes the sparse tail from dense comparisons. It is distinct from full-saved ATE.

| Trajectory | Common-mask ATE OFF m | Common-mask ATE ON m |
|---|---:|---:|
| Original GLIM | 0.040253 | 0.037264 |
| Original Kimera | 4.500021 | 0.340734 |
| Fusion ONLINE | 0.167950 | 0.104539 |
| Fusion DELAYED finalized | 0.157444 | 0.100291 |

The matched timestamps confirm the direction of the full-saved comparison.

## Delayed fixed-lag-finalized fusion

These historical estimates incorporate later information within the fixed-lag window. They are a separate product from the ONLINE path and do not represent full-history smoothing.

| CBS | ATE RMSE m | Translation RPE m | Rotation RPE deg | Path m |
|---|---:|---:|---:|---:|
| OFF | 0.155965 | 0.016563 | 0.317889 | 201.913713 |
| ON | 0.098779 | 0.014114 | 0.311266 | 202.791644 |

Finalized ATE improves 36.67%. The ONLINE output still has elevated rotation RPE and accumulated path length: ON 2.6359 deg / 225.877 m versus finalized 0.3113 deg / 202.792 m. These metrics establish a publication-versus-corrected-path difference, not its exact cause.

## Covariances retained

Pooled summaries include all received belief versions, including revisions and versions subsequently expired. They are revision-weighted distributions, not matched-edge ratios or optimizer information contributions.

| Covariance block | OFF median trace | ON median trace | OFF p95 | ON p95 |
|---|---:|---:|---:|---:|
| GLIM translation m2 | 2.100151e-6 | 2.079463e-6 | 3.004894e-6 | 2.943990e-6 |
| GLIM rotation rad2 | 7.440736e-9 | 7.457412e-9 | 2.373338e-8 | 2.323895e-8 |
| Kimera translation m2 | 4.841053e-6 | 4.440526e-6 | 3.679104e-4 | 5.215695e-5 |
| Kimera rotation rad2 | 4.096114e-8 | 3.219576e-8 | 2.105525e-7 | 9.574497e-8 |

GLIM traces change little. Kimera median translation/rotation traces decrease about 8.3%/21.4%, with a much larger reduction in its translation upper tail. All matrices were finite/symmetric with positive minimum eigenvalues. Transport covariance error was exactly zero. LOCAL provenance does not establish statistical independence or calibrated error uncertainty; means/charts and shared IMU remain coupled. Traces alone do not establish factor dominance.

## Exchange, revisions and engineering

OFF injected zero source-to-source factors. ON injected 1,473 factors in each direction, with 1,460 Kimera retirements and matching prior refreshes.

| Receiver accounting | OFF | ON |
|---|---:|---:|
| Received belief versions | 202,965 | 202,988 |
| Insertions | 4,924 | 4,925 |
| Replacements | 197,885 | 197,908 |
| Before-initialization exclusions | 25 | 25 |
| Expired versions of known absorbed IDs | 131 | 130 |
| Pending / unsupported at end | 0 / 0 | 0 / 0 |
| Retained supports / peak slots | 71 / 174 | 71 / 174 |

Expired versions came from 20 Kimera IDs per mode; no GLIM versions expired. Forgone mean and covariance changes are retained in the detailed expiry audit. This policy intentionally omits those late corrections.

| Wall processing time | OFF median / p95 ms | ON median / p95 ms |
|---|---:|---:|
| GLIM scan | 61.391 / 153.062 | 58.186 / 137.424 |
| GLIM MAIN | 28.647 / 51.905 | 28.548 / 52.300 |
| Kimera backend after 5 s | 82.852 / 136.580 | 82.524 / 104.145 |
| Fusion support cycle | 55.428 / 107.762 | 52.243 / 72.860 |

Capture wall times were 1,781.275 s OFF and 1,779.082 s ON. GLIM's queue maximum was one. MAIN used 4,909,738 / 5,292,978 prepared consumptions and zero synchronous MAIN reevaluations. ON had 13 scans over 500 ms, maximum 758.680 ms, without growing backlog. Final-20-second median scan times were 58.533 / 59.751 ms. This establishes the observed 0.20x run; it is not a 1x capacity or exhaustive batching-preservation certificate. No nonlinear stationarity certification is claimed.

Both cases passed their LOCAL provenance, Kimera lifecycle and receiver/conservation gates. All 2,391 protected hashes remained unchanged. The controller's post-scoring Docker process-list format failed; the original failure is preserved. Authorized manual cleanup verified no experiment jobs, stopped both containers and confirmed release of the temporary sleep inhibitor.

## Exact retained evidence

- [Protocol](experiments/eth1_awake_full_cpk23_20261021/PROTOCOL.txt)
- [Closeout report](experiments/eth1_awake_full_cpk23_20261021/REPORT.txt)
- [Detailed metrics and audits](experiments/eth1_awake_full_cpk23_20261021/RESULTS_REPORT.txt)
- [Machine-readable results](experiments/eth1_awake_full_cpk23_20261021/results.json)
- [Closure verification](experiments/eth1_awake_full_cpk23_20261021/closeout.json)
- Run root: `/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_awake_full_cpk23_20261021/prepared`.
- Original paths: `full_{off,on}_source/odom_imu.txt` and `kimera_odometry.csv`.
- Fusion paths: `full_{off,on}_receiver/receiver/online_trajectory.tum` and `finalized_trajectory.tum`.
- Covariances/revisions: `full_{off,on}_receiver/events.csv`, source logs, receiver archive and validation records.

Never rerun controllers into these completed directories. Preserve this report as the ETH1 reference when comparing a new sequence.
