# EIG-2 full usable sequence: CBS-OFF / CBS-ON comparison

Saved on 2026-10-21 as the EIG-2 companion to the [ETH1 CP-K23 reference report](/home/yeranis/repos/gnssFGO/GNSSFGO_ETH1_FULL_CPK23_REFERENCE_REPORT_20261021.md). **Both 355.02 s captures completed; CBS-ON improves the ATE of original GLIM, original Kimera and their live fusion in this matched pair.** No parameter tuning or source rerun was performed.

Qualification: the original ON validator failed its requirement for zero unsupported factors at end of recording. One final Kimera factor required a future interpolation support beyond the recorded clock. A separately recorded, identical OFF/ON terminal-boundary audit passed, along with all remaining checks. The factor remains omitted; the original failure and all captured trajectories are unchanged. These are results under that explicit finite-recording contract, not a claim that every factor was incorporated.

## Frozen experiment

- Mission: GrandTour EIG-2 `2024-11-03-13-59-54`.
- Full usable window: established offset **5.7 s**, duration **355.02 s**, through the last Hesai scan; **0.20x** headless playback and 25 s drain. Exact selected-topic record bounds: `1730638816563546034` to `1730639171583546034` ns. Initial 5.7 s and subsequent camera/IMU-only tails are excluded. This is not the older 305 s crop.
- Input preflight: 3,551 Hesai scans, 35,502 CPT7 samples and 3,550 synchronized front stereo pairs; reference coverage and input hashes checked.
- GLIM: Hesai `/boxi/hesai/points`, CPT7 `/boxi/cpt7/imu`, 5 s lag; CP-K16a MAIN GPU batching, CP-I1/CP-I3 all-LOCAL full covariance for every belief version and corrected MAIN means.
- Kimera: Alphasense front-left/front-right stereo, shared CPT7, 25 keyframes; CP-K9 private build, `CPK9_PRIOR_REFRESH=1`, `FL4Q_ENABLE=0`, `FL4M_RETIREMENT=remove`; native source IMU override unset.
- Source exchange: classic bidirectional `active_window_temporary`; 0.20 s belief horizon, one pair per update, stride 1, receive delay 0; Gaussian dense 6x6 covariance, scale 1 both ways. No source `live_revision`. Native-IMU mean alignment retains the existing mean-only covariance-alignment approximation.
- Fusion: CP-K19, 7 s fixed-lag `source_wnoa`, incremental publication, independent 100 ms supports; no central IMU or GNSS factors. State: pose, world velocity, body angular rate. Qc variances remain `[.00375,.00375,.00375,.015,.015,.015]`; prior standard deviations remain pose `1e-5`, velocity `10`, angular rate `3`.
- Stable-ID revisions replace active receiver factors. Under `seal_at_absorption`, known absorbed-ID revisions expire without changing the solver; the last accepted information remains in the prior. Unknown first arrivals on retired supports still fail. No expired revisions occurred in this pair.
- Same frozen binaries/model as CP-K23; saved EIG-2 profiles verified, including byte equality with the corresponding CP-K23 profiles. No robust loss, health weighting, covariance tuning, model change, rebuild or default promotion.

All 2,464 protected file hashes and the original trajectory/event/state hashes remain unchanged. Use archived binaries and profiles, not rebuilt defaults.

## Original trajectories and live fusion

ATE scores **every saved pose over each trajectory's complete saved journey**, not a selected short section. All saved poses have reference brackets. The reference is mission IE-TC `/boxi/inertial_explorer/tc/odometry`; it shares CPT7 and is **not independent prism ground truth**. The existing evaluator applies the prism lever arm and rigid SE(3) alignment without scale for ATE. One-second RPE uses physical relative intervals without that global alignment. Path lengths below are evaluated at the prism lever arm, not the raw CPT7 origin.

| Trajectory | CBS | ATE RMSE m | Translation RPE m | Rotation RPE deg | Path m |
|---|---|---:|---:|---:|---:|
| Original GLIM | OFF | 0.051085 | 0.018645 | 0.634210 | 189.321356 |
| Original GLIM | ON | 0.044555 | 0.015874 | 0.629456 | 189.489614 |
| Original Kimera ROS odometry | OFF | 1.882644 | 0.108961 | 0.844753 | 178.190542 |
| Original Kimera ROS odometry | ON | 0.688834 | 0.025637 | 0.840830 | 185.977289 |
| Fusion ONLINE, published during playback | OFF | 0.167508 | 0.043220 | 2.413468 | 225.562174 |
| Fusion ONLINE, published during playback | ON | 0.135765 | 0.039077 | 2.399313 | 221.245882 |

Reference sampled path: 188.098 m at GLIM timestamps, 186.631/186.634 m at OFF/ON Kimera timestamps, and 188.126 m at fusion timestamps. Different sampling rates produce different polyline lengths even for the same reference motion.

| Original/online output | ATE reduction ON vs OFF | Translation RPE reduction | Rotation RPE reduction |
|---|---:|---:|---:|
| GLIM | 12.78% | 14.87% | 0.75% |
| Kimera | 63.41% | 76.47% | 0.46% |
| Fusion ONLINE | 18.95% | 9.59% | 0.59% |

Kimera aligned position-error p95 falls from 3.9811 m to 1.0338 m. Fusion ONLINE p95 falls from 0.2777 m to 0.2046 m. Fusion remains less accurate than GLIM alone; the matched OFF-to-ON fusion improvement is the result here, not superiority over the best source. This pair and ETH1 show the same direction of ATE change, but do not establish universal improvement or isolate its mechanism.

## Coverage and matched timestamps

| Output | Saved poses OFF / ON | Saved span OFF / ON s | Largest pose interval OFF / ON s |
|---|---:|---:|---:|
| Original GLIM | 3,543 / 3,543 | 354.199316 / 354.199316 | 0.101291 / 0.101291 |
| Original Kimera | 1,761 / 1,761 | 354.697183 / 354.797183 | 0.799994 / 0.999993 |
| Fusion ONLINE | 3,545 / 3,545 | 354.400000 / 354.400000 | 0.100000 / 0.100000 |
| Fusion DELAYED finalized | 3,545 / 3,545 | 354.400000 / 354.400000 | 0.100000 / 0.100000 |

Relative to first/last captured IMU, GLIM has 0.7110 s startup and 0.1000 s tail gaps; Kimera has 0.1946 s startup and 0.1185/0.0185 s OFF/ON tail gaps. Fusion starts 0.5854 s after the selected window begins and ends 0.0346 s before it ends. No estimates are fabricated in these gaps. Kimera frontend processed 3,549 pairs per mode and backend/native odometry produced 1,761 sequential outputs. Full capture does not mean identical output sampling.

One-second RPE skips unsupported endpoint interpolation: maximum estimated-pose interval 0.5 s and reference interval 0.05 s. Full-saved RPE counts are 3,532 GLIM, 3,529/3,514 Kimera OFF/ON, and 3,533 for each fusion product. Path lengths join actual samples, including chords across gaps; they are not independent traveled-distance measurements.

The additional identical common mask across all eight curves has **3,526 timestamps**, **352.299664 s** of contiguous coverage within a **354.099662 s** envelope. Sixteen grid queries are excluded; there are two nonzero-duration segments and one isolated final timestamp. Common-mask RPE has 3,505 valid one-second pairs. This is supplementary to full-saved ATE, not a replacement for it.

| Trajectory | Common ATE OFF / ON m | Common translation RPE OFF / ON m | Common rotation RPE OFF / ON deg |
|---|---:|---:|---:|
| Original GLIM | 0.051115 / 0.044502 | 0.018686 / 0.015899 | 0.636611 / 0.631845 |
| Original Kimera | 1.881040 / 0.688001 | 0.109332 / 0.025664 | 0.847567 / 0.841824 |
| Fusion ONLINE | 0.165490 / 0.134561 | 0.043372 / 0.039214 | 2.422711 / 2.408786 |
| Fusion DELAYED finalized | 0.165486 / 0.132400 | 0.021763 / 0.014847 | 0.669116 / 0.669584 |

The common timestamps confirm the main ATE comparison. Exact masks, common-mask path lengths and segment endpoints are retained in the machine-readable evaluation.

## Delayed fixed-lag-finalized fusion

These historical estimates incorporate later information available before finalization. They are **not ONLINE trajectories and not full-history smoothing**.

| CBS | ATE RMSE m | Translation RPE m | Rotation RPE deg | Path m |
|---|---:|---:|---:|---:|
| OFF | 0.166193 | 0.021705 | 0.666499 | 188.511761 |
| ON | 0.132314 | 0.014837 | 0.666964 | 188.669543 |

Finalized ATE improves 20.39% and translation RPE 31.64%; rotation RPE is effectively unchanged, slightly worse by 0.07%. ONLINE rotation RPE and path length remain elevated: ON 2.3993 deg / 221.246 m versus finalized 0.6670 deg / 188.670 m. This identifies a difference between published and subsequently corrected paths, not its exact cause. Do not substitute the finalized path for online performance.

## Covariances and revisions retained

All payloads use dense 6x6 covariance with rotation then translation blocks, scale 1. These pooled statistics include **all received versions**, revisions, before-initialization versions and the one unsupported terminal ON version. They are not independent measurements, matched-edge OFF/ON ratios, or optimizer information contributions.

| Covariance block | OFF median trace | ON median trace | OFF p95 | ON p95 |
|---|---:|---:|---:|---:|
| GLIM translation m2 | 1.847275e-6 | 1.871498e-6 | 3.201099e-6 | 3.234206e-6 |
| GLIM rotation rad2 | 6.727904e-9 | 6.822130e-9 | 1.879347e-8 | 1.853728e-8 |
| Kimera translation m2 | 7.856488e-7 | 6.666840e-7 | 2.672277e-5 | 3.704339e-5 |
| Kimera rotation rad2 | 2.358577e-8 | 1.758713e-8 | 9.547110e-8 | 6.807582e-8 |

GLIM median traces increase only 1.31% translation / 1.40% rotation. Kimera medians decrease 15.14% / 25.43%, but its translation p95 **increases 38.62%**. Thus the distribution does not undergo one uniform scale change. This is descriptive evidence, not a covariance-tuning recommendation.

| Revision summary | OFF | ON |
|---|---:|---:|
| GLIM IDs / received versions | 3,541 / 170,366 | 3,541 / 170,354 |
| Kimera IDs / received versions | 1,744 / 43,344 | 1,744 / 43,323 |
| GLIM median last/first translation trace ratio | 0.618647 | 0.617707 |
| GLIM median last/first rotation trace ratio | 0.761360 | 0.760156 |
| Kimera median last/first translation trace ratio | 0.672491 | 0.318022 |
| Kimera median last/first rotation trace ratio | 0.592347 | 0.460711 |

Ratios use 3,540 revised GLIM IDs / 1,743 revised Kimera IDs per mode. Every matrix is finite and symmetric (maximum symmetry error zero); minimum eigenvalues are GLIM 1.581775e-10 OFF / 1.565275e-10 ON and Kimera 5.612564e-10 OFF / 7.192989e-10 ON. Transport covariance discrepancy is exactly zero. LOCAL provenance does not certify statistical independence or calibrated likelihood: corrected means/charts, shared IMU and exchange still couple estimates. Traces alone do not tell us which factor dominates optimization.

## Exchange, receiver accounting and timing

OFF injects no source-to-source CBS factors. ON injects **1,740 GLIM-to-Kimera** and **1,746 Kimera-to-GLIM** factors. Kimera has 1,717 planned retirements and matching prior refreshes; maximum checked cache error is 7.0539e-9, within the existing 1e-8 gate. These lifecycle checks are not a nonlinear stationarity certificate.

| Receiver accounting | OFF | ON |
|---|---:|---:|
| Received belief versions | 213,710 | 213,677 |
| Insertions | 5,283 | 5,282 |
| Replacements | 208,377 | 208,344 |
| Before-initialization exclusions | 50 | 50 |
| Expired / duplicate / stale versions | 0 / 0 / 0 | 0 / 0 / 0 |
| Unsupported terminal versions | 0 | 1 |
| Pending after EOF | 0 | 0 |
| Absorbed factors / active factors at end | 5,186 / 97 | 5,185 / 97 |
| Retained supports / peak slots | 71 / 175 | 71 / 175 |

Accounting is exact: inserted + replaced + initialization exclusions + terminal omissions equals received. Mean log round-trip error is at most 3.60e-14 OFF / 5.40e-14 ON. Every one of the 3,545 ONLINE poses per mode was published while the sources were running.

| Wall processing time | OFF median / p95 ms | ON median / p95 ms |
|---|---:|---:|
| GLIM scan | 55.609 / 142.748 | 57.483 / 159.518 |
| GLIM MAIN | 24.359 / 41.979 | 25.209 / 43.548 |
| Kimera backend after 5 s | 88.457 / 110.759 | 92.038 / 122.500 |
| Kimera smoother after 5 s | 27.578 / 38.510 | 29.045 / 49.789 |
| Kimera outgoing diagnostics after 5 s | 53.631 / 65.569 | 54.017 / 65.775 |
| Fusion support cycle | 49.094 / 77.996 | 50.416 / 78.138 |
| Receiver receipt to processing | 10.482 / 23.958 | 10.900 / 23.982 |

Capture wall durations: 1,821.328 s OFF / 1,823.396 s ON. GLIM internal queue maximum is one in both modes. MAIN has 4,691,886 / 4,943,020 prepared consumptions and **zero synchronous MAIN reevaluations**. OFF has no scan over 500 ms; ON has 17, maximum 731.373 ms, without sustained backlog or truncation. Last-20-second median scan time is 57.501 / 61.080 ms. Receiver output age p95 is approximately 0.0902 dataset seconds in both modes; transport peak queues are 49 / 46 and end pending counts are zero. This demonstrates the observed 0.20x operating point, not 1x capacity or exhaustive batching equivalence.

## Exact terminal omission and validation status

ON's final Kimera `1759 -> 1760`, revision 0:

| Quantity | Timestamp ns |
|---|---:|
| Physical start | 1730639171360676500 |
| Physical end | 1730639171560675100 |
| Last available receiver support | 1730639171548930000 |
| Next required support | 1730639171648930000 |
| Final input clock / belief arrival | 1730639171579221000 |

The endpoint is **11.7451 ms after the final support**. The next required support is **69.709 ms beyond the last real input clock**. The receiver cannot interpolate this endpoint with the available supports. Existing [Receiver.h::end](/home/yeranis/repos/gnssFGO/experiments/receiver_finalization_cpk19_20261021/Receiver.h:375) explicitly records such pending inputs as `unsupported_at_end`; an existing legacy test covers this behavior. It is not a late revision of an absorbed state, estimator crash, or missing camera stream.

Original ON validation stopped at its zero-unsupported assertion. That script, failed log, controller status and all captured outputs remain intact. CP-K24A applies a supplementary contract equally to OFF and ON: only measured endpoints in the final incomplete 100 ms interval, requiring exactly the next support beyond the final clock, with retained starting support, causal arrival, EOF-only classification and exact conservation can be excluded. Interior gaps, future measurement stamps, retired starts and more distant missing supports are rejected. Both modes pass this audit and every remaining original receiver check; the original source LOCAL/lifecycle gates also passed.

No support was invented, no trajectory shortened, and no optimization or source was rerun. **The missing factor was not recovered.** Its counterfactual effect on a revised path is unknown; a small count is not proof of no accuracy effect. No general future-run admission policy or implementation repair is promoted by this report.

## Retained evidence and closeout

- [Capture protocol](/home/yeranis/repos/gnssFGO/experiments/eig2_full_cpk24_20261021/PROTOCOL.txt)
- [Chronological report, including original failure](/home/yeranis/repos/gnssFGO/experiments/eig2_full_cpk24_20261021/REPORT.txt)
- [Supplemental EOF protocol](/home/yeranis/repos/gnssFGO/experiments/eig2_full_cpk24_20261021/eof_review/PROTOCOL.txt)
- [Detailed metrics and validation evidence](/home/yeranis/repos/gnssFGO/experiments/eig2_full_cpk24_20261021/RESULTS_REPORT_EOF_REVIEW.txt)
- [Machine-readable results](/home/yeranis/repos/gnssFGO/experiments/eig2_full_cpk24_20261021/results_eof_review.json)
- [Saved-input fingerprints](/home/yeranis/repos/gnssFGO/experiments/eig2_full_cpk24_20261021/eof_review/validation_inputs.json)
- [Supplemental closeout](/home/yeranis/repos/gnssFGO/experiments/eig2_full_cpk24_20261021/eof_review/closeout.json)

Run root: `/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eig2_full_cpk24_20261021/prepared`. Original outputs: `full_{off,on}_source/odom_imu.txt` and `kimera_odometry.csv`; fusion outputs: `full_{off,on}_receiver/receiver/online_trajectory.tum` and `finalized_trajectory.tum`. Full covariance/revision histories remain in `full_{off,on}_receiver/events.csv`, source logs and the receiver archive. Evaluation masks and reference hashes are in `evaluation_full/summary.json`.

Both containers are restored to stopped after saved-data scoring. No source/solver repeat, exhaustive batching observer, tuning, push or next experiment was performed. Nonlinear stationarity is not certified. The write-up follows the established ETH1 report structure, while explicitly separating the original strict failure from the supplementary EOF validation. Never rerun controllers into these completed directories.
