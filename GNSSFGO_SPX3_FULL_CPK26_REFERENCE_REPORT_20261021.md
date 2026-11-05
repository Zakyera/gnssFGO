# SPX-3 full usable sequence: CBS-OFF / CBS-ON comparison

Experiment completed on 2026-10-21; reference report finalized on 2026-10-22. **Both CP-K26 runs completed and passed their three validation gates. CBS-ON substantially improved original Kimera and improved live fusion; original GLIM ATE was essentially unchanged.** This is the fresh matched restart, not a combination with the interrupted CP-K25 attempt. No tuning or rebuild was performed.

One terminal Kimera factor in **each** mode could not be inserted because its interpolation required a support beyond the recorded clock. Both omissions satisfy the identical, predeclared finite-recording contract. They remain explicitly counted; this report does not claim zero factor loss or knowledge of the missing factors' effect.

## 1. Frozen experiment

| Item | Configuration |
|---|---|
| Mission | GrandTour SPX-3 `2024-11-02-17-43-10` |
| Window | Established offset 32.0 s; full usable 194.02 s through the last Hesai record, not the older 134 s crop |
| Playback | Headless 0.20x, 25 s drain, port 11476 |
| Inputs | 1,941 Hesai scans; 19,402 CPT7 samples; 1,940 synchronized front stereo pairs |
| GLIM | Hesai `/boxi/hesai/points`, CPT7 `/boxi/cpt7/imu`, 5 s lag; CP-K16a MAIN GPU batching |
| GLIM export | Corrected MAIN means; CP-I1/CP-I3 all-LOCAL dense covariance for every first and revised version |
| Kimera | Alphasense front-left/front-right stereo and shared CPT7; 25 keyframes; CP-K9 private build |
| Kimera lifecycle | `CPK9_PRIOR_REFRESH=1`, `FL4Q_ENABLE=0`, `FL4M_RETIREMENT=remove`; native source IMU override unset |
| Source CBS | Classic bidirectional `active_window_temporary`; 0.20 s horizon, one pair per update, stride 1, receive delay 0; no source `live_revision` |
| Fusion | CP-K19, 7 s fixed-lag `source_wnoa`, independent 100 ms supports; no central IMU or GNSS factors |
| Fusion state | Pose, world velocity, body angular rate |
| WNOA / priors | Qc `[.00375,.00375,.00375,.015,.015,.015]`; prior standard deviations pose `1e-5`, velocity `10`, angular rate `3` |
| Weighting | Full 6x6 Gaussian covariance, scale 1; no robust loss, health weighting or covariance tuning |

OFF disables source-to-source CBS injections; ON enables them. Downstream stable-ID revisions replace active factors in both modes. Under `seal_at_absorption`, revisions of known absorbed IDs expire without modifying the solver; the last accepted information remains in its prior. Unknown first arrivals on retired supports still fail. No expired revisions occurred here.

Native-IMU mean alignment retains the existing **mean-only covariance-alignment approximation**. All-LOCAL covariance is not a claim that the two estimators' means or information are statistically independent.

Selected-topic record bounds are `1730565902872888651` to `1730566096892888651` ns. The last Hesai record is `1730566096888473034` ns. The initial 32 s and later camera/IMU-only tails are outside this established usable window. Source profiles and protected binaries were verified; all **2,613 protected files remained unchanged**.

## 2. Original trajectories and live fusion — primary comparison

These are **original GLIM, original Kimera ROS odometry, and their fusion**, not GLIM-only/Kimera-only reconstructions through the fuser. ATE uses every saved pose across each output's entire saved journey. All saved poses have reference brackets.

Reference: mission IE-TC `/boxi/inertial_explorer/tc/odometry`. It **shares CPT7 IMU and is not independent prism ground truth**. The established evaluator applies the prism lever arm and per-path rigid SE(3) alignment without scale for ATE. One-second RPE uses physical relative intervals without that global alignment. Path lengths below are at the prism lever arm, not the raw CPT7 origin.

| Trajectory | CBS | ATE RMSE m | 1 s translation RPE m | 1 s rotation RPE deg | Path m |
|---|---|---:|---:|---:|---:|
| Original GLIM | OFF | 0.016897 | 0.006559 | 0.264549 | 90.482644 |
| Original GLIM | ON | 0.016885 | 0.006691 | 0.241736 | 90.563018 |
| Original Kimera ROS odometry | OFF | 0.552484 | 0.144630 | 0.570683 | 89.125830 |
| Original Kimera ROS odometry | ON | 0.120067 | 0.024174 | 0.493077 | 90.610933 |
| Fusion ONLINE, published during playback | OFF | 0.058603 | 0.033177 | 1.347218 | 99.677696 |
| Fusion ONLINE, published during playback | ON | 0.044991 | 0.028503 | 1.322960 | 97.284724 |

| Output | ATE change with ON | Translation RPE change | Rotation RPE change |
|---|---:|---:|---:|
| GLIM | 0.07% lower: effectively unchanged | 2.00% worse | 8.62% better |
| Kimera | **78.27% better** | **83.29% better** | 13.60% better |
| Fusion ONLINE | **23.23% better** | **14.09% better** | 1.80% better |

Reference path at the corresponding timestamps: GLIM 90.329 m; Kimera 90.273 m; fusion 90.432 m. Sampling density and the lever arm affect polyline length.

The result is a substantial Kimera improvement accompanied by better fusion without material GLIM ATE degradation. Fusion still has higher ATE than GLIM alone. This establishes an OFF-to-ON improvement in this matched experiment, not superiority over the best source or a causal explanation based on covariance alone.

## 3. Coverage: complete saved journey and common-mask check

| Output | Saved poses OFF / ON | Saved span OFF / ON s | Largest sample gap OFF / ON s |
|---|---:|---:|---:|
| GLIM | 1,933 / 1,933 | 193.199633 / 193.199633 | 0.101319 / 0.101319 |
| Kimera | 951 / 950 | 193.798311 / 193.798311 | 0.599993 / 0.699994 |
| Fusion ONLINE | 1,935 / 1,935 | 193.400000 / 193.400000 | 0.100000 / 0.100000 |
| Fusion DELAYED | 1,935 / 1,935 | 193.400000 / 193.400000 | 0.100000 / 0.100000 |

Relative to the first/last selected IMU samples, GLIM has approximately 0.715 s startup and 0.096 s tail gaps; Kimera 0.198 s startup and 0.013 s tail gaps. Fusion spans `1730565903.454194`–`1730566096.854194` s, starting 0.581 s after the selected record start and ending 0.039 s before its end. Startup/tail gaps are disclosed, not filled artificially.

Each mode processed 1,939 stereo frames and produced consecutive backend keyframe IDs. Kimera's differing keyframe counts are retained, not resampled into invented native output. Full-saved ATE includes every actual saved pose; RPE and common-grid interpolation reject estimate gaps above 0.5 s and reference gaps above 0.05 s.

The identical common mask contains **1,913 timestamps**, spanning **193.099816 s**, with **190.899818 s of contiguous covered intervals** across four segments. Nineteen of 1,932 grid queries lack acceptable support. This is an additional matched check, not a replacement for the full-saved table above.

| Output on common mask | ATE OFF / ON m | 1 s t-RPE OFF / ON m | 1 s r-RPE OFF / ON deg |
|---|---:|---:|---:|
| Original GLIM | 0.016852 / 0.016835 | 0.006639 / 0.006771 | 0.267949 / 0.244836 |
| Original Kimera | 0.528145 / 0.117599 | 0.141057 / 0.017972 | 0.560152 / 0.484999 |
| Fusion ONLINE | 0.056171 / 0.042964 | 0.033501 / 0.028864 | 1.364642 / 1.340299 |
| Fusion DELAYED | 0.050914 / 0.039193 | 0.009683 / 0.008047 | 0.256877 / 0.261454 |

Common-mask path lengths OFF/ON: GLIM 90.350452/90.401356 m; Kimera 88.631082/90.126566 m; ONLINE fusion 94.090004/93.954237 m; DELAYED fusion 89.917512/90.109995 m. Reference 90.345650 m. Polyline chords across sampling gaps and sampling density affect these values.

## 4. Delayed fixed-lag-finalized fusion — not online

| CBS | ATE RMSE m | 1 s translation RPE m | 1 s rotation RPE deg | Path m |
|---|---:|---:|---:|---:|
| OFF | 0.051392 | 0.009980 | 0.254114 | 90.299096 |
| ON | 0.039281 | 0.008132 | 0.258433 | 90.519202 |

These poses incorporate later information while retained in the fixed lag and are finalized when leaving it. They are neither the original as-played output nor a full-history retrospective solution. ON improves delayed ATE by 23.57% and translation RPE by 18.52%; rotation RPE worsens by 1.70%.

The ONLINE paths are longer than the reference (99.68/97.28 m versus about 90.43 m), and ONLINE rotational RPE is appreciably higher than finalized RPE. This remains a meaningful output-quality limitation. The aggregate metrics do not by themselves identify its cause.

## 5. Covariance and revision evidence

All received versions are pooled below, including revisions and versions subsequently excluded at initialization or the terminal boundary. They are not independent measurements, and this is **not** a matched-edge/revision-age calibration study or a measurement of optimization influence. Full covariance directions and Jacobians matter; rotational and translational traces must not be added as a unitless weight.

| Source/block | Median OFF | Median ON | p95 OFF | p95 ON |
|---|---:|---:|---:|---:|
| GLIM translation trace m² | 1.651401e-6 | 1.645293e-6 | 2.985544e-6 | 2.956588e-6 |
| GLIM rotation trace rad² | 8.610949e-8 | 8.782412e-8 | 1.697550e-7 | 1.689913e-7 |
| Kimera translation trace m² | 6.915672e-6 | 4.701547e-6 | 4.772707e-5 | 3.811691e-5 |
| Kimera rotation trace rad² | 5.750391e-8 | 4.356550e-8 | 1.835379e-7 | 8.336178e-8 |

GLIM median translation trace changes by −0.37% and rotation by +1.99%. Kimera medians fall by **32.02% translation** and **24.24% rotation**. Kimera accuracy improves in the same run, but this association does not establish that these uncertainties are calibrated or that covariance reduction caused fusion improvement.

Recorded matrices are finite and symmetric, with positive observed eigenvalues. Minimum full-matrix eigenvalues OFF/ON are GLIM `4.34939e-9/4.32179e-9` and Kimera `7.31634e-10/1.66815e-11`. Full-matrix eigenvalues mix rotation/translation units; positivity is a numerical validity check, not a physical uncertainty floor or universal bound. Maximum covariance payload transport difference is exactly zero in both modes; maximum mean round-trip error is below `1.51e-14`.

| Source versions | OFF | ON |
|---|---:|---:|
| GLIM received versions / stable IDs | 92,057 / 1,930 | 92,340 / 1,930 |
| GLIM IDs with revisions | 1,929 | 1,929 |
| Kimera received versions / stable IDs | 22,800 / 924 | 22,850 / 926 |
| Kimera IDs with revisions | 923 | 925 |

Median last/first trace ratios across revised IDs: GLIM translation 0.6353/0.6357 and rotation 0.8062/0.8057 OFF/ON; Kimera translation 0.3173/0.2150 and rotation 0.5861/0.5194. These describe within-ID evolution, not the OFF-to-ON ratio. LOCAL provenance checks cover every GLIM version through the recorded native-local route and loaded provider; they do not independently certify covariance calibration or remove all cross-estimator correlations.

## 6. Live exchange, lifecycle and conservation

OFF accepted no source-to-source injections. ON accepted **922 GLIM→Kimera** and **933 Kimera→GLIM** injections. Kimera performed **898 external-factor retirements and 898 prior refreshes**; maximum recorded prior-cache error was `2.76602e-10`, below the `1e-8` check. Obsolete FL4q was not executed.

| Receiver accounting | OFF | ON |
|---|---:|---:|
| Journal events | 134,260 | 134,593 |
| Received belief versions | 114,857 | 115,190 |
| Inserted stable-ID factors | 2,851 | 2,853 |
| Replacements | 111,955 | 112,286 |
| Pre-initialization exclusions | 50 | 50 |
| Terminal unsupported factors | 1 | 1 |
| Expired / duplicate / stale versions | 0 / 0 / 0 | 0 / 0 / 0 |
| Absorbed factors / active at end | 2,750 / 101 | 2,752 / 101 |
| Pending at end | 0 | 0 |
| Retained supports / peak slots | 71 / 176 | 71 / 176 |
| ONLINE published / finalized poses | 1,935 / 1,935 | 1,935 / 1,935 |

Conservation is exact: inserts + replacements + initialization exclusions + terminal omissions = received versions. Every ONLINE pose was published while the sources were running. The gate set passed in each mode: LOCAL provenance, source lifecycle, and receiver checks including the explicit EOF contract.

There was **no new saved-journal deterministic replay or exhaustive batching-preservation test** in this experiment. Incremental/cache checks do not certify nonlinear stationarity. Recorded source cost increases (16 OFF, 2 ON) are observations, not stationarity certificates.

## 7. Runtime and delay

Values below are wall-clock milliseconds, median / p95. Source stages overlap only where their implementation does so; do not sum unrelated pipeline stages into a latency estimate.

| Stage | OFF ms | ON ms |
|---|---:|---:|
| GLIM whole scan update | 37.653 / 45.385 | 36.818 / 58.801 |
| GLIM MAIN | 10.518 / 12.257 | 10.373 / 12.381 |
| GLIM post callback | 23.894 / 31.501 | 22.950 / 28.756 |
| Kimera backend, after first 5 s | 51.129 / 70.058 | 53.450 / 73.504 |
| Kimera smoother, after first 5 s | 13.485 / 17.721 | 14.551 / 19.363 |
| Kimera outgoing stage, after first 5 s | 34.785 / 52.778 | 35.404 / 53.405 |
| Fusion support cycle | 43.688 / 62.748 | 43.989 / 61.377 |
| Receiver receipt-to-processing | 9.659 / 18.303 | 9.879 / 18.702 |

GLIM maximum scan update was 227.969/259.092 ms OFF/ON, with none above 500 ms and internal queue maximum 1. Its last-20-s medians were 37.510/37.394 ms: no growing late-run median was observed. MAIN recorded 890,922/897,958 prepared hits and **zero synchronous calls**.

Fusion support-cycle maxima were 88.199/72.108 ms. Transport peak queue was 48/49 events; p95 published output age was approximately **0.090 dataset seconds** in both modes, maximum about 0.10006 s. These output-age and wall-clock processing metrics are different quantities.

Each capture took about **16 min 56 s wall time**: 1,016.459 s OFF and 1,016.609 s ON, including startup/drain overhead. The full pair, validation, scoring and cleanup finished at **20:16:36 local time**, about 35.5 minutes after launch. This supports successful sustained operation at 0.20x for this pair, not a guarantee at 1x or on other missions.

## 8. Exact end-of-recording limitation

The omitted factors are OFF Kimera `949→950 revision 0` and ON `948→949 revision 0`, covering the **same physical interval**:

- Start: `1730566096670909900` ns.
- End: `1730566096870908000` ns.
- Last receiver support: `1730566096854194000` ns.
- Next required support: `1730566096954194000` ns.
- Last recorded clock: `1730566096884199000` ns.

The endpoint is 16.714 ms beyond the last support; the next required support is 69.995 ms beyond the final clock. The factor is within the recorded measurement period, but interpolation lacks its future bracket. No support was fabricated. The predeclared audit accepts only this finite-recording boundary case, retains exact accounting, and leaves all other checks unchanged. This is not an ignored late revision of an already absorbed ID. Its effect on a hypothetical longer recording is unknown.

## 9. Evidence and closeout

- [Frozen protocol](/home/yeranis/repos/gnssFGO/experiments/spx3_awake_full_cpk26_20261021/PROTOCOL.txt) and [execution log](/home/yeranis/repos/gnssFGO/experiments/spx3_awake_full_cpk26_20261021/REPORT.txt).
- [Complete machine-readable results](/home/yeranis/repos/gnssFGO/experiments/spx3_awake_full_cpk26_20261021/results.json) and [generated detailed report](/home/yeranis/repos/gnssFGO/experiments/spx3_awake_full_cpk26_20261021/RESULTS_REPORT.txt).
- [Evaluator summary](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_awake_full_cpk26_20261021/prepared/evaluation_full/summary.json), [controller status](/home/yeranis/repos/gnssFGO/experiments/spx3_awake_full_cpk26_20261021/status.json), and [successful closeout](/home/yeranis/repos/gnssFGO/experiments/spx3_awake_full_cpk26_20261021/closeout.json).
- Run root: `/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_awake_full_cpk26_20261021/prepared`.
- Within each `full_off_source` / `full_on_source`: original GLIM `odom_imu.txt`, original Kimera `kimera_odometry.csv`, source mean/covariance/revision and timing records. Within each `full_off_receiver` / `full_on_receiver`: `events.csv` and `receiver/online_trajectory.tum`, `receiver/finalized_trajectory.tum`.

Both containers are stopped; controller failure and cleanup error are null. Scoped inhibition ended with the finite controller; no permanent power policy was changed. No persistent follow-up, commit, push, default promotion or further experiment was initiated.

Companion reference reports: [ETH1 CP-K23](/home/yeranis/repos/gnssFGO/GNSSFGO_ETH1_FULL_CPK23_REFERENCE_REPORT_20261021.md) and [EIG-2 CP-K24](/home/yeranis/repos/gnssFGO/GNSSFGO_EIG2_FULL_CPK24_REFERENCE_REPORT_20261021.md). The three completed pairs share the direction of Kimera and ONLINE fusion ATE improvement; they do not establish universal improvement or calibrated uncertainty.
