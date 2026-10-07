# ETH1 WNOJ prediction and motion derivative uncertainty

## Conclusion

The current source-only WNOJ graph increases uncertainty when predicting beyond available beliefs, but its nominal uncertainty does not adequately describe several large subsequent revisions. The motion estimate changes substantially already at the last source-connected support, before the entirely prediction-only interval. This is not simply a case of a reliable starting velocity and acceleration followed by too long a wait for the next measurement.

In the largest retained CBS-ON transition, predicted position is revised by **21.38 cm** where the earlier graph's standard deviation in that direction is **1.47 cm**. At the last source-connected support, velocity changes by **1.064 m/s**, versus a directional standard deviation of **0.0584 m/s**. Acceleration changes there by **14.42 m/s²**, versus **1.012 m/s²**. These are same-physical-time revisions, not robot motion between timestamps.

This is a warning about the current assumed model and its sensitivity to changing information, **not a calibrated statistical rejection, ground-truth error measurement, or proof that WNOJ cannot predict without an IMU**. Later batches replace old factors as well as add new ones, so they are not independent measurements conditioning an otherwise unchanged posterior.

No settings were changed. No GLIM, Kimera, receiver replay or nonlinear optimization was run. This diagnostic only reconstructs and linearizes already-saved, converged graph snapshots.

Prior: [prediction versus measured-region distinction](/home/yeranis/repos/gnssFGO/GNSSFGO_SOURCE_WNOJ_PREDICTION_TAIL_DISTINCTION_20261006.md). Evidence: [artifacts](/home/yeranis/results/gnssfgo_source_wnoj_derivative_uncertainty_20261006), [summary](/home/yeranis/results/gnssfgo_source_wnoj_derivative_uncertainty_20261006/summary.json), [verification](/home/yeranis/results/gnssfgo_source_wnoj_derivative_uncertainty_20261006/verification.json).

## Frozen experiment and comparison

The same eight-second ETH1 OFF/ON journals and actual receiver states from [the full-relinearization validation](/home/yeranis/repos/gnssFGO/GNSSFGO_SOURCE_WNOJ_FULL_RELINEARIZATION_20261006.md) are used. There are 71 snapshots with source information per mode, including the final graph, and 70 origins per mode with a subsequent information batch. The four earlier source-free publications are outside this comparison.

Frozen settings are X/V/W/A full WNOJ, Qc diagonal `[2.5,2.5,2.5,10,10,10]`, 100 ms independent support spacing, original broad derivative priors, full dense source covariance at scale one, stable-ID revision replacement and the existing causal availability schedule. This is the experimental no-central-IMU model, not the frozen production Combined-IMU baseline. IMU timestamps supply a clock; raw IMU values are neither measurements nor initialization inputs here.

Every snapshot is reconstructed from its retained revision ledger and the exact original event payloads, using the existing `FullWnojRelativeFactor`, `GPWNOJPriorFull` and initial priors. Before analysis, every factor's type, slot, keys, full covariance and residual are checked against its saved payload probe. The states are read from `incremental_states.csv`; there is no new solution replacing them.

The source journals remain:

| Mode | SHA-256 |
|---|---|
| OFF | `5a884a946e0e26be384e58c2b00b0592f84eee6ad559c3125fc1416ae1507eed` |
| ON | `a84a27812855dffa3da7dc5288d1e7f882c11784331004398858e80f37c41454` |

Both are under `/home/yeranis/results/gnssfgo_source_wnoj_first_spike_20261005/inputs/`. Retained snapshots are under `/home/yeranis/results/gnssfgo_source_wnoj_full_relinearization_20261006/{off,on}_primary/`.

## Physical coverage and graph connectivity differ slightly

Let F be the latest physical endpoint among already-arrived source factors. Three support locations matter:

| Location | Meaning |
|---|---|
| Covered support | Latest support at or before F; it lies inside an active source relative interval in this dataset |
| Source-connected boundary | Right support used to interpolate F; source factors still depend on it even though it is usually after F |
| Current support | Newest published state |

The interpolation dependency matters: a timestamp after the physical endpoint is not necessarily independent of source factors. We explicitly verified that no active source factor touches a support after the source-connected boundary.

Of the 70 origins per mode, 48 have an entirely source-free suffix after that boundary; in 22 the boundary is already the current support. The entirely prediction-only duration is 0–200 ms, with median 100 ms. The physical gap after F remains the previously measured median 128 ms and maximum 208 ms. No timing, support grid, pairing or output policy changed.

## What the graph reports as uncertain

GTSAM QR marginals are computed from the full graph at each saved estimate, including all cross covariance. For a three-component quantity, the following scale is `sqrt(trace(P))`: the square root of the sum of its three marginal variances. It is **not** a per-axis standard deviation or a 95% confidence radius.

Median scales across the same 70 origins:

| Quantity and location | CBS OFF | CBS ON |
|---|---:|---:|
| Position at covered support | 1.354 cm | 1.364 cm |
| Position at source-connected boundary | 1.368 cm | 1.384 cm |
| Position at current prediction | 2.488 cm | 2.592 cm |
| Velocity at covered support | 0.0204 m/s | 0.0231 m/s |
| Velocity at source-connected boundary | 0.1003 m/s | 0.0995 m/s |
| Velocity at current prediction | 0.2871 m/s | 0.2875 m/s |
| Linear tangent acceleration at covered support | 0.780 m/s² | 0.768 m/s² |
| Linear tangent acceleration at source-connected boundary | 1.766 m/s² | 1.745 m/s² |
| Linear tangent acceleration at current prediction | 2.477 m/s² | 2.469 m/s² |

Uncertainty is large at startup and contracts as beliefs arrive. Thus the model does recognize weaker derivative knowledge and growing prediction uncertainty. However, the later revisions show that these local, model-conditional covariances are not sufficient evidence of reliable predictions.

Position blocks are rotated from the local pose tangent into world coordinates before comparing world-position changes. Velocity is already world-frame. Acceleration refers to the stored WNOJ tangent derivative, not raw accelerometer data or a separate world-acceleration measurement. Rotation, angular rate and angular acceleration statistics are also retained in the numerical summary.

## Revision size relative to nominal uncertainty

For each old snapshot and a fixed physical timestamp, the later change is d. The diagnostic computes `sqrt(dᵀ P_old⁻¹ d)` using the corresponding full 3×3 marginal. This measures revision size in the graph's nominal uncertainty units. It is **not innovation NIS, NEES against truth, or a chi-square calibration test**.

| Location and quantity | OFF median / maximum | ON median / maximum |
|---|---:|---:|
| Covered velocity | 1.64 / 5.77 | 1.05 / 8.40 |
| Covered linear tangent acceleration | 1.76 / 11.95 | 1.69 / 14.99 |
| Boundary velocity | 2.91 / 13.99 | 1.74 / 18.29 |
| Boundary linear tangent acceleration | 2.08 / 10.75 | 0.98 / 14.25 |
| Current predicted position | 0.92 / 10.85 | 0.51 / 14.58 |
| Current predicted velocity | 2.77 / 11.59 | 1.37 / 15.69 |
| Current predicted linear tangent acceleration | 1.98 / 8.57 | 1.04 / 10.94 |

There are both small and large changes; instability is not present in every batch. The graph does not merely retain universally diffuse derivative uncertainty. Substantial revisions sometimes occur in directions to which the earlier local approximation assigns relatively small uncertainty, including inside the source-covered region.

The samples overlap in time and reuse state history. They are not independent experimental trials, and this table does not establish an OFF/ON accuracy ranking.

## The concrete large CBS ON revision

Between snapshots 61 and 62, compare physical timestamp `1727775014683475000 ns`. The earlier source-connected boundary is support 60 at `1727775014583475000 ns`, leaving one purely predictive 100 ms interval. The physical source frontier is `1727775014555357500 ns`.

The next batch introduces one new GLIM ID, one new Kimera ID and 62 revision replacements, plus a support extension. Its effects cannot be attributed to a single source callback.

| Quantity | Actual same-time change | Prior standard deviation along change direction |
|---|---:|---:|
| Boundary position | 3.299 cm | 0.816 cm |
| Boundary velocity | 1.064 m/s | 0.0584 m/s |
| Boundary linear tangent acceleration | 14.416 m/s² | 1.012 m/s² |
| Current predicted position | 21.380 cm | 1.468 cm |
| Current predicted velocity | 2.577 m/s | 0.1643 m/s |
| Current predicted linear tangent acceleration | 15.575 m/s² | 1.423 m/s² |

At the older endpoint-covered support, acceleration also changes by 4.994 m/s² against a directional standard deviation of 0.444 m/s². The derivative sensitivity therefore already exists before the pure prediction suffix. The OFF 59→60 transition shows the same pattern: boundary velocity changes by 0.762 m/s against 0.0605 m/s directional uncertainty, and the current position changes by 15.462 cm against 1.469 cm directional uncertainty.

These comparisons concern changed estimates. They do not establish which earlier or later estimate is closer to the actual motion.

## Separating inherited uncertainty from new process uncertainty

For each existing WNOJ factor on the source-free suffix, linearization gives Jacobians H_i and H_j with covariance Q. The exact local Gaussian transition is:

```text
F_step = -inverse(H_j) H_i
Q_step = inverse(H_j) Q inverse(H_j)^T
P_next = F_step P_previous F_step^T + Q_step
```

This uses the existing full nonlinear factor's Jacobians and all 18 state coordinates. It is not a second process model, a diagonal approximation or a production propagation change. Repeated composition separates the current marginal into propagated boundary covariance and accumulated process covariance. It agrees with direct graph marginalization to maximum relative matrix error **1.52e-12** across all 142 snapshots.

For the 48 nonempty prediction suffixes per mode, the median fractions of variance trace supplied by newly accumulated process uncertainty are:

| Current quantity | OFF | ON |
|---|---:|---:|
| Position | 15.30% | 15.30% |
| Velocity | 32.19% | 32.19% |
| Linear tangent acceleration | 67.85% | 67.85% |

At the specific ON61 event, only **2.29% of current position variance** comes from new process uncertainty in the final 100 ms; **97.71% is inherited from the boundary state's joint uncertainty**. Velocity process contribution is 12.17%; acceleration process contribution is 49.26%.

These are uncertainty decompositions, **not percentages of the observed jump's cause**. Inherited uncertainty includes pose, velocity, angular rate, acceleration and their cross terms. The exported decomposition retains self terms and signed cross terms; it does not assign independent positive weights to coupled state blocks.

## What this means for the next model decision

There is no evidence here of a broken covariance propagation step through the WNOJ-only suffix. The propagated and direct marginals agree. The current graph instead produces motion estimates that can later be revised far more than its nominal local uncertainty suggests. Prediction extends those provisional estimates, magnifying their visible consequences.

This does not isolate a single cause. In particular, factor replacement changes the historical information set rather than adding independent observations; the nonlinear local Gaussian approximation can change; source covariance and shared-information assumptions remain uncalibrated; and the current source-factor interpolation uses a conditional mean without a separate GP interpolation uncertainty term. These are limitations or candidate mechanisms, not causes established by this task.

The current configuration should therefore **remain unvalidated for reliable current-time prediction**. It should not be promoted merely because the graph converges or produces a smooth final retrospective path. Conversely, neither changing WNOJ to another process model nor reinstating central IMU is justified solely by this result.

The most direct next mechanism test would use one already-saved large transition and separate **new endpoint beliefs** from **revisions of previously inserted beliefs** on isolated graph copies, retaining the same priors and uncertainties. That would identify which information change first alters the boundary derivatives before considering a model or weighting change. It has not been run here; no additional sweep or full sequence follows this diagnostic.

## Figure

Red shows the next-batch revision at the same physical timestamp as the old current prediction. Blue shows its earlier nominal marginal scale. Large startup uncertainty is retained, not cropped. The comparison is not ground-truth error, and the blue line is not a confidence bound.

![Prediction revisions and nominal uncertainty](/home/yeranis/results/gnssfgo_source_wnoj_derivative_uncertainty_20261006/prediction_uncertainty_vs_revision.png)

## Validation and preservation

- All 142 reconstructed graphs match retained factor types, slots and keys. Dense covariance discrepancy is exactly zero; maximum unwhitened residual discrepancy is `2.12e-14`.
- Full queried covariance matrices are finite and positive definite. Cross blocks are retained, including endpoint interpolation dependencies.
- Synthetic checks validate positive process covariance, stationary `q dt^5/20` position scaling and agreement between full graph marginalization and joint transition propagation.
- Four complete numerical export files reproduce byte-for-byte across all 142 snapshots.
- Focused ASan/UBSan runs pass for the synthetic checks and four selected real snapshots, including the large OFF and ON events. Sanitized exports match Release exactly. Existing linked libraries are not sanitizer-instrumented.
- Independent subtraction of ON61/ON62 raw state rows and a NumPy covariance solve reproduce the velocity revision distance `15.69181322148327`.
- All **20,867 protected files** preserve their SHA-256 hashes. Production code, source outputs and previous experiment products are unchanged. The figure was visually inspected.
- Initial diagnostic attempts had a C++ alias collision and an overly restrictive synthetic assertion that ignored SE(3) coupling during moving-state process noise. The scalar scaling assertion was restricted to its valid stationary case; full cross-block checks remain. Successful real-data products are exclusively under `primary3/`. Failed logs remain retained; no receiver output was affected.

## Reproduction and repository state

The pre-analysis protocol, payload exports, exact compiler arguments, build logs, source utility, full covariance blocks and summary statistics are saved in the [artifact directory](/home/yeranis/results/gnssfgo_source_wnoj_derivative_uncertainty_20261006).

The existing local image `sha256:6a664509d183e504230b22edfd747dfe1ca5bca8ae6fc34c76154db8ffd1fc8d` was run with network disabled, a read-only root filesystem and read-only source/runtime/prior mounts. `/audit` is the new artifact directory; `/source` is `/home/yeranis/repos/gnssFGO`; `/runtime` is `/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime`; `/previous` is the full-relinearization artifact root; `/tail` is the preceding prediction-tail artifact root. No dependency was installed.

Container driver calls were `python3 /audit/run.py primary3`, `repeat` and `sanitizer`. Exact argv are recorded in `commands_primary3.json`, `commands_repeat.json` and `commands_sanitizer.json`. Exclusive output creation prevents accidental overwrites; reproduction requires a fresh output label/destination. Host analysis uses `/home/yeranis/.local/share/pipx/venvs/evo/bin/python` with `analyze.py`, `verify.py` and `plot.py`.

Release compilation took 21.81 s and the 142-graph diagnostic 5.37 s; repeated export took 5.28 s. Sanitizer compilation took 58.24 s and the four real graphs 0.53 s. These are read-only diagnostic costs, not receiver throughput measurements.

HEAD remains `a842e7ef72e7a40f0cdfec826f089c66af364ae0`; no commit was created. The pre-existing dirty worktree is preserved and `git diff --check` passes. The only new repository file from this task is this report.

**MOTION_DERIVATIVE_REVISIONS_EXCEED_NOMINAL_UNCERTAINTY**
