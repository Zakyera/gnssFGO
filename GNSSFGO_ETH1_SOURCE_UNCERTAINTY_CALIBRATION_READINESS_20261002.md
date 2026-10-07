# ETH1 source uncertainty calibration readiness

Date: 2026-10-02. Frozen-data analysis only; no GLIM, Kimera, gnssFGO or bag-playback run.

**The reported source uncertainty is small compared with the observed disagreement with IE-TC, including in CBS-OFF. However, these retained references do not identify a calibrated additive source covariance. No noise floor, covariance scale or fusion setting was changed.**

The important new finding is that the original IE-TC bag contains uncertainty information that the earlier pose-only export discarded. It is incomplete for this calibration: odometry pose covariance is entirely zero-filled; NavSatFix contains a nonzero position matrix but marks its covariance type unknown; the raw messages contain marginal standard deviations, not joint cross-time pose covariance. AP20 is independent position evidence but has neither orientation nor covariance fields and covers only part of this window.

This does **not** mean the reference trajectory is useless or that the previous ATE results are invalid. Ranking paths against a retained reference and identifying millimetre-scale six-dimensional measurement uncertainty are different requirements.

## Scope and frozen inputs

The inputs are the first-120-second ETH1 classic CBS OFF/ON capture, mission `2024-10-01-11-29-55`, under [the frozen run directory](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live120_zero_20260930/). The current runbook and gnssFGO `AGENTS.md` were read. This audit does not reinterpret the native GLIM or native Kimera trajectories as single-source gnssFGO paths.

The receiver operating point remains Combined IMU, WNOJ `Qc=10000`, ALL_RIGHT receiver integration, 59 startup samples, independent 100 ms support clock, full-history iSAM2, stable-ID source revision replacement, and zero added source-model noise. Classic source CBS is unchanged. The native Kimera capture uses its saved `current` packet path; it is not silently relabeled as the earlier experimental RIGHT Kimera implementation.

Exactly **1,189 GLIM and 581 Kimera intervals** match OFF/ON by source and serialized physical endpoints. Both describe approximately 0.20 s relative motion. At each age—first publication, latest available by 1 s, latest available by 2 s, last observed—there is one sample per accepted physical belief, not thousands of supposedly independent revisions. Last observed does not certify retirement or exclude capture-end censoring.

The unchanged audited helper evaluates

`e = Log(Z_source^-1 (T_IE(i)^-1 T_IE(j))) = [phi, rho]`.

IE-TC orientation uses SLERP and position uses linear interpolation at the exact serialized belief endpoints with the existing 50 ms bracket gate. All accepted intervals are evaluable. No trajectory alignment or ATE optimization is involved in this relative comparison. `rho` is the existing SE(3) logarithm coordinate, not a substituted Cartesian residual.

The [protocol](/home/yeranis/results/gnssfgo_eth1_uncertainty_calibration_20261002/protocol.md), [analysis script](/home/yeranis/results/gnssfgo_eth1_uncertainty_calibration_20261002/analyze.py), and [derived artifacts](/home/yeranis/results/gnssfgo_eth1_uncertainty_calibration_20261002/) preserve the scope and reproducible calculations.

## Reported uncertainty and reference disagreement

For readability the table uses `sqrt(trace(Sigma_block))`: the root-sum variance in each three-dimensional channel. This is not a per-axis standard deviation, a 95% radius or a scalar fusion weight. Every entry is a median over the same accepted intervals within that source. Full dense matrices remain available and unchanged.

| Last observed belief | Reported rho root-sum variance mm | Rho discrepancy to IE-TC mm | Reported phi root-sum variance deg | Phi discrepancy to IE-TC deg |
|---|---:|---:|---:|---:|
| GLIM OFF | 1.473 | 5.250 | 0.00679 | 0.11078 |
| GLIM ON | 1.290 | 14.891 | 0.00507 | 0.20186 |
| Kimera OFF | 1.190 | 2.894 | 0.00810 | 0.10915 |
| Kimera ON | 0.800 | 3.787 | 0.00371 | 0.10642 |

The discrepancy is **not assigned entirely to the source**. It includes reference error, source error, possible systematic mismatch and their dependence. Nevertheless, the small matrices and large descriptive discrepancies mean we should not claim that these exported Schur covariances have already passed an absolute uncertainty-calibration test.

Revision evolution is also important:

| Source and mode | First to last reported rho root-sum variance mm | First to last rho discrepancy mm |
|---|---:|---:|
| GLIM OFF | 1.913 → 1.473 | 7.223 → 5.250 |
| GLIM ON | 1.715 → 1.290 | 15.307 → 14.891 |
| Kimera OFF | 2.397 → 1.190 | 4.170 → 2.894 |
| Kimera ON | 2.900 → 0.800 | 7.864 → 3.787 |

Kimera ON's rotational root-sum variance contracts from **0.01029° to 0.00371°**, while median rotational discrepancy changes from **0.20179° to 0.10642°**. Its mean improves, but claimed precision increases faster than this descriptive error reduction. This is evidence to investigate, not a fitted correction coefficient.

### Dense whitening and temporal dependence

The analysis computes `eᵀ Sigma_source^-1 e` by Cholesky whitening, retaining all cross terms. It also computes each three-dimensional marginal statistic. As a purely descriptive benchmark, the fractions below the six-dimensional chi-square 95% threshold, 12.5916, are:

| Last observed belief | Fraction below the benchmark |
|---|---:|
| GLIM OFF | 2.103% |
| GLIM ON | 1.850% |
| Kimera OFF | 0.688% |
| Kimera ON | 0.000% |

These are **not calibrated NIS/NEES pass rates or hypothesis-test p-values**: the denominator deliberately contains only the source matrix, not an identified covariance of the source-minus-reference discrepancy. Treating them as a valid 95% coverage test would assume the reference is exact and the statistical model is appropriate—the assumptions being investigated.

Discrepancies also are not independent white samples. The centered lag-one vector correlation of rho discrepancies is **0.791 / 0.510 for GLIM OFF/ON** and **0.772 / 0.695 for Kimera OFF/ON**. These are descriptive correlations in the local residual coordinates, not a fitted stationary process; overlapping intervals and shared smoother/reference history contribute. No confidence interval based on independent edges is claimed.

Across contiguous 10 s endpoint bins, the median per-edge ratio `||rho|| / sqrt(trace(Sigma_rho))` ranges from **2.37–6.42** for GLIM OFF, **3.59–16.25** for GLIM ON, **1.20–3.78** for Kimera OFF, and **3.00–6.54** for Kimera ON. This variation does not support deriving one universal OFF→ON multiplier from a single aggregate trace. The [full summaries](/home/yeranis/results/gnssfgo_eth1_uncertainty_calibration_20261002/discrepancy_summary.json) include all four ages, error RMS, means, covariance health and block summaries. Local-frame mean errors are not interpreted as calibrated physical sensor biases.

## What the original reference messages contain

The original bag is [ETH1 CPT7 IE-TC](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/datasets/GrandTour/2024-10-01-11-29-55_cpt7_ie_tc.bag). Each listed IE-TC stream contains **69,601 messages at exactly 200 Hz**, spanning `1727775008.000000000`–`1727775356.000000000`. There are **23,779 messages per stream** inside the retained evaluation bounds `1727775008.775065899`–`1727775127.674791098`.

| Topic below `/boxi/inertial_explorer/tc/` | Verified contents | Calibration limitation |
|---|---|---|
| `odometry` | `nav_msgs/Odometry`, `enu_origin` → `cpt7_imu`; all 69,601 pose covariance arrays are zero | No usable declared pose covariance; zero does not certify perfect truth |
| `navsatfix` | `sensor_msgs/NavSatFix`; nonzero 3×3 position matrix; covariance type is 0 for all 23,779 window samples | Type 0 is UNKNOWN; no attitude, cross-time covariance or source-reference covariance |
| `raw` | `gnss_msgs/GnssRaw`; ECEF position standard deviations, heading/roll/pitch standard deviations, quality and satellite fields | Marginal uncertainty only; embedded schema does not document units or supply the necessary joint covariance |

The odometry twist covariance is nonzero, but it was **not repurposed as pose covariance**. The ROS message definitions distinguish these quantities, and pose covariance uses `[x,y,z,rotX,rotY,rotZ]`, not the receiver's `[phi,rho]` ordering. [ROS pose message definition](https://raw.githubusercontent.com/ros/common_msgs/noetic-devel/geometry_msgs/msg/PoseWithCovariance.msg).

Under the NavSatFix field's specified metre-squared units, the stored position matrix has median `sqrt(trace)` **7.634 mm** and p95 **14.185 mm**. This is an absolute-position uncertainty indicator, **not** the reference uncertainty over a 0.20 s relative edge. Its type is explicitly unknown. The recorded `status=-50` and `service=50` are retained as-is and not used to certify fix quality. [ROS NavSatFix definition](https://raw.githubusercontent.com/ros/common_msgs/noetic-devel/sensor_msgs/msg/NavSatFix.msg).

Raw encoded median standard-deviation values are ECEF `(0.005, 0.003, 0.005)` and heading/roll/pitch `(0.0450581, 0.00212902, 0.00215289)`. They are saved as **encoded values**, without claiming that an untraced converter establishes their units or converting them to receiver tangent covariance. The embedded message defines quality 1 as best and 6 as worst: this window has 20,867 / 2,811 / 101 samples at quality 1 / 2 / 3. All raw ambiguity-status values are the message's `FIXED=1`. Good quality flags do not supply cross-time covariance or remove systematic error.

NovAtel describes Inertial Explorer's reported accuracy as an estimate, with attitude standard deviations produced by its GNSS/INS filter, and warns that estimated position accuracy does not include certain systematic errors. That supports treating these fields as quality information rather than exact truth; it does not establish this dataset converter's field mapping. [NovAtel Inertial Explorer Common Plots](https://docs.novatel.com/Waypoint/Content/Inertial_Explorer/Common_Plots_IE.htm).

The saved [reference audit](/home/yeranis/results/gnssfgo_eth1_uncertainty_calibration_20261002/reference_audit.json), [raw embedded schema](/home/yeranis/results/gnssfgo_eth1_uncertainty_calibration_20261002/GnssRaw_embedded_definition.txt) and [reference arrays](/home/yeranis/results/gnssfgo_eth1_uncertainty_calibration_20261002/reference_metadata.npz) preserve the evidence. No converter was modified or reference covariance invented.

### Independent prism coverage

[The AP20 bag](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/datasets/GrandTour/2024-10-01-11-29-55_ap20_prism_position.bag) contains 4,547 `geometry_msgs/PointStamped` messages in `leica_total_station`, with 1,300 inside the evaluation bounds. They contain position only: no attitude or covariance. The longest in-window gap is **52.293 s**.

Using a declared 150 ms gap rule on raw prism samples yields nine segments, including isolated points, totaling **65.118 s** of segment spans. This inventory is not a changed ATE mask. The frozen fused ATE mask remains its existing **654 matches, 64.9 aggregate seconds in five segments** after evaluator sampling/gating.

With a strict 50 ms reference-bracket requirement at both physical endpoints, only **258/1,189 GLIM** and **142/581 Kimera** intervals have prism timestamp support. Even those are not full SE(3) reference observations. The prism/body lever arm is known by the existing evaluator, but using IE-TC orientation to transform the prism would reintroduce that reference's orientation and shared-input dependence. No such substitution was made here.

## Why a covariance floor is not identified yet

To first order in a common error tangent, the discrepancy is `e ≈ epsilon_reference - epsilon_source`. Its covariance is

`S_e ≈ Sigma_source + Sigma_reference - C_source,reference - C_source,reference^T`.

The reference term itself is a *relative-transform* covariance. For endpoint perturbation Jacobians A and B it includes

`Sigma_reference,ij ≈ A P_i A^T + B P_j B^T + A P_ij B^T + B P_ji A^T`.

The saved per-epoch standard deviations do not identify `P_ij`, the full endpoint pose blocks or the source-reference cross terms. In the simpler scalar position case, `var(p_j-p_i)=sigma_i²+sigma_j²-2 cov(p_i,p_j)`: nearby errors can cancel strongly. Assuming two independent endpoint errors, or treating both as zero, would silently choose different uncertainty models.

IE-TC, GLIM, Kimera and the receiver share CPT7 information. CBS adds another path for common information between source graphs. Neither marginal covariance health nor Schur-complement correctness proves that exported posterior beliefs are independent likelihoods. An additive diagonal floor cannot represent all of these cross-correlations or remove systematic errors. It may still be a useful approximation, but it requires an explicit validation claim, not the assertion that it makes the graph statistically correct.

Consequently we cannot defensibly set `D = empirical discrepancy covariance - reported covariance`, nor fit a floor by forcing the above chi-square fractions to 95%. That would absorb unknown reference error, dependence and possible bias into a number attributed solely to the source. A trustworthy bound showing those terms negligible could also permit calibration; none was established from these exports.

## Decision and next requirement

**No calibrated source-model covariance is frozen by this audit.** Keep the production settings unchanged. The previously explored SPX3 engineering floor remains a sensitivity experiment, not a calibration result, and no covariance-intersection implementation was added.

Before estimating a numerical correction, establish either an independent sufficiently accurate relative-pose reference with characterized uncertainty, or defensible bounds/models for the existing reference's relative error and shared-input dependence. Tracing the original IE-TC exporter and uncertainty definitions is a possible first step, but marginal units alone will not recover missing temporal covariance. Partial prism constraints can validate position-related quantities; they cannot certify the complete 6×6 belief covariance.

If that requirement is met, the already-implemented opt-in model `Sigma_effective = Sigma_reported + D`, with nonnegative rotation/translation variance additions, is a small candidate to calibrate—not an instruction to implement a new method. Fit and validate one rule for both OFF and ON, keep revision age explicit, use time-blocked validation, and freeze it before evaluating another sequence. A likelihood-based fit must include its covariance-normalization term, not merely reduce whitened residuals by inflating uncertainty. Calibration is not a promise that ON ATE will beat OFF.

This task stops at the reference/calibration gate. It does not start another sequence or an ATE-driven parameter sweep.

## Validation and reproduction

- The original IE-TC bag reproduces all 69,601 cached reference positions and quaternions **exactly**, with identical timestamps under the original ROS `Time.to_sec()` arithmetic.
- A preliminary timestamp-identity assertion detected that `float64(epoch_ns)*1e-9` and `secs + nsecs/1e9` differ by at most **0.2384 microseconds**. The bag-verification code was corrected to use the original exporter arithmetic; belief interpolation and frozen reference arrays were not changed. This was an analysis conversion issue, not an estimator result.
- All 16 source/mode/age groups reproduce the previous median error/trace summaries to maximum absolute difference **5.56e-17**.
- 100 synthetic SE(3) identity/right-perturbation and dense-whitening checks pass: maximum round-trip coordinate error **5.95e-17**, whitening quadratic-form difference **3.26e-19**.
- All inspected selected source matrices are finite, symmetric and strictly positive definite. Numerical validity is not statistical calibration.
- **27 pinned inputs/configuration/production files have identical before/after hashes.** This includes the two event journals, both reference bags, existing native/fused trajectories and receiver source. No frozen artifact was overwritten.
- Successful analysis took **6.17 s**; no source/receiver run, new build or production test suite was needed or claimed. Zero covariance parameters were fitted.
- Existing tracked worktree diff hash is unchanged: `69dd475a5338e94d4c489df9d2382b11e31fd51c523d13f39c9dd99605d2f4ef`. The worktree was already dirty and was not cleaned. Only this report and its new analysis directory were created.
- No commit was made. HEAD remains `a842e7ef72e7a40f0cdfec826f089c66af364ae0`.

Reproduce in a fresh copy of the analysis directory to retain the current generated products:

```bash
PYTHONDONTWRITEBYTECODE=1 /home/yeranis/.local/share/pipx/venvs/evo/bin/python \
  /home/yeranis/results/gnssfgo_eth1_uncertainty_calibration_20261002/analyze.py
```

The script writes only beside itself. It reuses the existing NumPy/SciPy/rosbags runtime and audited residual helper; nothing was installed or downloaded. [Input hashes](/home/yeranis/results/gnssfgo_eth1_uncertainty_calibration_20261002/inputs_before.json) and [validation results](/home/yeranis/results/gnssfgo_eth1_uncertainty_calibration_20261002/validation.json) provide the machine-readable record.

**UNCERTAINTY_CALIBRATION_NOT_IDENTIFIED_FROM_CURRENT_REFERENCES**
