# FL4g: matched ETH1 first-120 s live CBS-OFF / classic CBS-ON

Date: 2026-10-15. Both cases completed with the frozen [predeclared protocol](PROTOCOL.md), and both saved receiver journals replayed byte-identically. This is a **negative 120 s ON result**, not a tuning run: native GLIM improves under classic CBS-ON, but native Kimera degrades sharply and the live fusion is worse. The favorable FL4f first-30 s result therefore does not extend unchanged to this longer window. No estimator, covariance, CBS, or receiver-model setting was changed after seeing the result.

**Subsequent FL4h accounting clarification:** the580 G→K count below denotes constructed candidates, not final retained admissions. The saved Kimera log contains22 recovery transactions:18 candidate admissions are undone, leaving562 retained admissions. Eleven older external factors are additionally removed by existing residual guards. These are internal source recoveries, not process failures or downstream conservation failures. See the [source-failure audit](../fixed_lag_live_fl4h_20261015/REPORT.md). Source factors are Gaussian/scale1 with health/Huber disabled, but the existing temporary-factor recovery is not guard-free. The original trajectory metrics are unchanged.

## Exact experiment and provenance

- GrandTour ETH1 mission `2024-10-01-11-29-55`, bag offset `0`, requested duration `120.0 s`, playback `0.10×`. Native GLIM uses Hesai+CPT7 with the historical 5 s source lag; native Kimera uses front stereo+CPT7 with a 25-state window. The copied OFF/ON GLIM configuration trees are byte-identical; so are the copied Kimera profile trees (`diff -qr`). All input bags existed before playback.
- Source-to-source CBS: `classic`, `active_window_temporary`, 0.20 s horizon, at most one horizon pair per update, full dense `schur_relative_between` 6×6 covariance, covariance scale 1. Both sources publish in both modes. OFF disables external reception; ON enables bidirectional external reception. Source `live_revision=false`. Receiver-side stable-ID revision replacement is a different operation.
- Concurrent downstream gnssFGO: the **same** FL4e candidate executable in both cases, SHA-256 `53c96a2b7a762800cf2291166dc78520269098684411de641b52bee6a54dc5c9`. Independent 100 ms CPT7 support after 59 startup samples, 7 s incremental fixed-lag source-WNOA, unchanged Qc/priors, **no central IMU factor**, no GNSS, Gaussian GLIM/Kimera factors, no gate, robust loss, health weighting, or covariance scaling. Source and receiver ran together; these are not offline source trajectories fed to a later fusion run.
- Fresh successful run root: `/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_fixed_lag_live120_fl4g_20261015/prepared`. Preflight verified the prior 845 protected inputs and froze 879 paths including the copied binary/library overlay. No earlier run artifact was overwritten. Separate result validation is in `/home/yeranis/results/gnssfgo_eth1_fixed_lag_live120_fl4g_20261015/{off,on}`.

## Completion, exchange, and bounded runtime

| Check | OFF | ON |
|---|---:|---:|
| Original GLIM poses / span | 1,191 / 119.00 s | 1,191 / 119.00 s |
| Original Kimera ROS-odometry poses / span | 584 / 119.60 s | 584 / 119.60 s |
| G→K / K→G constructed source candidates (see FL4h) | 0 / 0 | 580 / 581 |
| Receiver GLIM / Kimera belief versions | 56,429 / 14,250 | 56,509 / 14,250 |
| Receiver inserted / replaced / excluded-before-init | 1,770 / 68,884 / 25 | 1,770 / 68,964 / 25 |
| Fused online / fixed-lag-finalized poses | 1,193 / 1,193 | 1,193 / 1,193 |
| Pending / unsupported-at-end | 0 / 0 | 0 / 0 |
| Retained supports / peak slots | 71 / 175 | 71 / 175 |
| Peak transport queue | 47 | 49 |
| Whole-support service mean / p95 / max, wall s | .052 / .076 / .109 | .060 / .108 / .166 |
| Receipt-to-processing p95, wall s | .024 | .026 |
| Case wall time | 1,245.74 s | 1,245.72 s |

Both cases consumed 11,986 CPT7 IMU events, and the native outputs reached the requested tail. Kimera has a 3.20 s initial pose gap; after startup its maximum gap was 0.20 s. Receiver online support spans 119.2 s and every online pose was published during source playback. Pre-playback effective-mode checks passed; ON actually injected in both directions, while OFF injected zero. No source, transport, or receiver failure was reported. The receiver covariance payload matched the exported journal **exactly** (`max error=0`); the relative-mean log round-trip error stayed below `5.7e-14`. Every belief version was conserved and accounted for. Replaying the OFF and ON journals reproduced all numerical products and respectively 72,960 and 73,040 archived files byte-for-byte.

This is a bounded 0.10× operating point, **not** a 1× throughput certificate. The current incremental publication policy does not certify nonlinear stationarity or exact full-history equivalence; the receiver summary explicitly records `stationarity=not_checked_incremental_policy`.

## Primary accuracy: identical physical timestamps

The common mask contains **1,160 timestamps at 10 Hz**, from Unix time `1727775011.600` to `1727775127.500` (115.9 s elapsed), after Kimera's startup gap. Its timestamp digest is `23c6a34297e3bde3928306cf38b91b00a92b1b6eba863ea000ed958a09b9417e`. Every row below uses that same mask, the same no-scale rigid SE(3) ATE alignment, and 1 s relative-pose RPE at the CPT7-to-prism lever arm. Full-window reference: GrandTour IE-TC GNSS/INS `/boxi/inertial_explorer/tc/odometry`, which **shares the CPT7 IMU** with the estimators and is not independent prism ground truth.

| Mode / trajectory | ATE m | 1 s t-RPE m | 1 s r-RPE deg | Path m |
|---|---:|---:|---:|---:|
| OFF original GLIM | 0.044372 | 0.026395 | 0.414797 | 72.232 |
| ON original GLIM | **0.025248** | **0.018466** | **0.367217** | 71.897 |
| OFF original Kimera ROS odometry | 0.458070 | 0.045923 | 0.553567 | 69.628 |
| ON original Kimera ROS odometry | **3.745897** | **0.350497** | 0.609450 | 67.959 |
| OFF fused **online/as-played** | 0.055480 | 0.030942 | 1.233298 | 74.167 |
| ON fused **online/as-played** | **0.131993** | **0.108307** | 1.385400 | 79.884 |
| OFF fused fixed-lag-finalized | 0.052336 | 0.017217 | 0.362169 | 71.764 |
| ON fused fixed-lag-finalized | **0.127574** | **0.052380** | 0.397481 | 72.945 |

The ON native GLIM ATE improves by 43%, but native Kimera ATE becomes 8.2× larger. The **actual online** fused ATE becomes 2.38× larger and its translational RPE 3.50× larger. The retrospective finalized graph is also worse ON (ATE 2.44×; translational RPE 3.04×), so this is not solely a live-output timing artifact. Finalized states include later revisions and must **not** be labelled online. The complete saved-trajectory score for each original output row, its GT overlap and path, source file/hash, and the common-mask values are retained in `evaluation_full/summary.json` under the run root; the main table is the fair OFF/ON comparison.

For clarity, scoring **every saved pose** of each original output against the same IE-TC reference (rather than resampling onto the identical-time mask) gives:

| Mode / saved trajectory | Rows / span s | ATE m | 1 s t-RPE m | 1 s r-RPE deg | Path m |
|---|---:|---:|---:|---:|---:|
| OFF original GLIM | 1,191 / 119.00 | 0.044144 | 0.026421 | 0.411857 | 72.582 |
| ON original GLIM | 1,191 / 119.00 | 0.025391 | 0.018701 | 0.364886 | 72.300 |
| OFF original Kimera | 584 / 119.60 | 0.463011 | 0.045912 | 0.553459 | 70.012 |
| ON original Kimera | 584 / 119.60 | 3.767876 | 0.350346 | 0.609261 | 68.750 |
| OFF fused **online/as-played** | 1,193 / 119.20 | 0.059287 | 0.030557 | 1.219072 | 75.939 |
| ON fused **online/as-played** | 1,193 / 119.20 | 0.137793 | 0.106902 | 1.370012 | 83.106 |
| OFF fused finalized | 1,193 / 119.20 | 0.056127 | 0.017016 | 0.358254 | 72.192 |
| ON fused finalized | 1,193 / 119.20 | 0.129593 | 0.051733 | 0.393422 | 73.580 |

These full-saved scores have different sample timestamps and hence are supplementary; the preceding common-mask table is the direct matched comparison. Kimera's 3.2 s startup gap is not filled with invented poses.

## Independent partial prism cross-check

The AP20 prism bag does **not** cover the whole 120 s. Its existing scorer matched about 65.1–65.3 s for GLIM/fusion and 62.4 s for Kimera. These are role-specific partial masks, not replacements for the common-mask table. The separate prism ATEs (OFF→ON) are: original GLIM `0.0401→0.0219 m`, original Kimera `0.5452→4.3616 m`, fused **online** `0.0621→0.1743 m`, and fused finalized `0.0577→0.1627 m`. Thus the independent reference supports the direction of the deterioration within its available portion. Exact retained scores: `prism_partial/summary.json`.

## Exported belief covariance

All captured 6×6 covariance versions were finite and symmetric, with positive minimum eigenvalue in each source/mode (`≥2.06e-10` GLIM; `≥7.27e-10` Kimera). Median traces over **all versions** (not posterior covariance or a direct graph-information measure):

| Sender | OFF tr Σφ | ON tr Σφ | OFF tr Σρ | ON tr Σρ |
|---|---:|---:|---:|---:|
| GLIM | 1.405e-8 rad² | 8.365e-9 rad² | 2.177e-6 m² | 1.640e-6 m² |
| Kimera | 2.455e-8 rad² | 3.458e-8 rad² | 1.654e-6 m² | 2.936e-6 m² |

The OFF/ON physical edge-ID sets and endpoint timestamps match exactly: 1,189 GLIM edges and 582 Kimera edges. On each matched edge's **last observed revision**, the median ON/OFF trace ratios are GLIM `0.857` rotation / `0.875` translation and Kimera `1.810` rotation / `3.688` translation. The fraction with lower ON translation trace is 84.8% for GLIM but only 8.6% for Kimera. Kimera's ON covariance generally **expanded**, not collapsed, while its native trajectory degraded. Covariance trace alone does not establish calibration, correctness, or the complete `JᵀΣ⁻¹J` contribution. This experiment does not identify the cause of Kimera's long-window failure and makes no weighting change.

## Audit boundary and artifacts

- OFF/ON source copied GLIM and Kimera profile trees are identical; the intended source difference is reception/injection only. The receiver configuration is identical except labels/sessions, with the same executable and all frozen input paths verified before both cases. ON launch was admitted only after OFF structural and deterministic checks passed.
- Fused online TUM SHA-256: OFF `c98c04b4a51789969f2a455a71552af7e6d47b8c7ff0fc4bc5eb2117c108eea4`; ON `c6ca3aef563b01e3834ecab317e292aed350db8cc7545eaac505f4e9882210be`. Finalized SHA-256: OFF `11a4eb12af8a7fb0132acf49a0485d487306791872d43952b91d35a71146b574`; ON `c113cef6577eb5c562e4f2e8f0cc24ac24d4a747ca664a91af1d29f9bd41c2dd`.
- Native original outputs: `pilot120_{off,on}_source/odom_imu.txt` and `pilot120_{off,on}_source/kimera_odometry.csv`. Receiver journals, complete graph archives, online/finalized trajectories, effective source parameters, and transport logs remain under the run root. Separate `validation.json` and `repeat_validation.json` for each case are under the results root above.
- No additional sequence, revised source method, covariance tuning, or Rerun visualization was launched. The longer-window ON degradation is a valid observation, not proof that a particular source factor, covariance component, or WNOA setting caused it.
