# ETH1 measured trajectory and prediction tail distinction

## Conclusion

The largest position and velocity revisions are concentrated in the newest prediction, beyond the latest arrived source endpoint. **But the already-covered part is not fully settled: its estimated acceleration still changes substantially.** This is not a clean division into a perfectly healthy measured trajectory and a problematic prediction alone.

On the same saved eight-second ETH1 test, the maximum next-batch position change at the **exact latest source endpoint** is 0.90 cm CBS-OFF and 2.71 cm CBS-ON. At the **previously current predicted timestamp**, the maxima are 15.70 cm and 21.38 cm. Every change compares the same physical timestamp before and after more information becomes available; ordinary movement between timestamps is excluded.

This supports the predict-then-revise analogy with an EKF. The important difference is that this no-central-IMU graph estimates velocity and acceleration from its pose constraints and WNOJ model, rather than predicting from new raw IMU measurements. A later factor/revision can change both current and historical states. No ground-truth accuracy or covariance calibration conclusion follows from these comparisons.

**No GLIM, Kimera or receiver replay was run.** This is read-only analysis of the previously retained, numerically converged snapshots. Factors, Qc, source covariance, priors, support timing, causal arrivals, source CBS and production outputs are unchanged.

Prior: [publication convergence report](/home/yeranis/repos/gnssFGO/GNSSFGO_SOURCE_WNOJ_FULL_RELINEARIZATION_20261006.md). New [artifacts](/home/yeranis/results/gnssfgo_source_wnoj_prediction_tail_20261006), [numeric summary](/home/yeranis/results/gnssfgo_source_wnoj_prediction_tail_20261006/summary.json), [verification](/home/yeranis/results/gnssfgo_source_wnoj_prediction_tail_20261006/verification.json).

## What was compared

The inputs are the 75 saved actual receiver snapshots per mode: supports 1–74 plus final export, from `/home/yeranis/results/gnssfgo_source_wnoj_full_relinearization_20261006/{off,on}_primary/`. All 150 previously passed the fixed numerical convergence comparison. The analysis uses `incremental_states.csv`, the actual converged receiver states, not a new offline solution substituted for them.

The eight-second journal spans `1727775008003475000`–`1727775016003475000 ns`. Its 75 support states start at `1727775008583475000 ns`, with 100 ms spacing and 7.4 s output span. The unchanged model is full WNOJ X/V/W/A, `Qc=diag(2.5,2.5,2.5,10,10,10)`, stable-ID source revision replacement, full dense source covariance at scale one, and the original priors. Raw IMU values are not used by this branch; their timestamps supply the independent clock.

At each publication, define:

- `t_now`: the newest support timestamp, whose state is the current predicted output.
- `F_G`, `F_K`: the latest physical endpoints in the active, already-arrived GLIM and Kimera revision ledgers.
- `F=max(F_G,F_K)`: the latest source endpoint from either source. No waiting for both sources or new pairing is introduced.
- `t_c`: the newest actual support timestamp at or before F.

“Endpoint-covered” means at or before F. It is not an observability or accuracy certificate. All 70 selected `t_c` values per mode also lie inside an active source relative interval, not merely before an unrelated later endpoint.

Four early publications have no source frontier yet and are retained separately, not labeled measured. The main comparisons use the remaining **70 matched publication origins per mode**. In 27, GLIM supplies the latest endpoint; in 43, Kimera does. The receiver is not using GLIM as a master source.

| Timing quantity across those 70 origins | Median | p95 | Maximum |
|---|---:|---:|---:|
| Current prediction beyond F | 128.116 ms | 208.431 ms | 208.437 ms |
| Covered support `t_c` before exact endpoint F | 71.894 ms | 91.635 ms | 91.659 ms |

These quantities match in OFF and ON at the sampled publications. Position and orientation at **exact F** are queried using the unchanged `FullWnojEndpoint::pose()` interpolation. Velocity, angular rate and tangent acceleration comparisons use actual support states at `t_c`, not fabricated derivatives at F. The covered-support statistics therefore describe a point up to about 92 ms older than the exact endpoint.

For each origin, the same physical timestamp is queried at the next publication, first later endpoint coverage, and final reconstruction. Final estimates are later estimates, **not ground truth**. The first-coverage check has 69 cases per mode; the last predicted support never becomes endpoint-covered before the retained cutoff and remains censored.

The logs provide 100 ms publication snapshots, not complete converged states after every individual source callback. A transition can include support extension, several source inserts and many revisions. Consequently, this report attributes changes to the **next recorded information batch**, not to a uniquely isolated GLIM or Kimera factor. The final flush is also retained as a batch and need not be 100 ms after its predecessor.

## Same timestamp position corrections

Every row below compares `p_after(t) − p_before(t)` at fixed t. Statistics are over the same 70 origins, not counts of independent measurements.

| Mode and queried position | Median, cm | RMS, cm | p95, cm | Maximum, cm |
|---|---:|---:|---:|---:|
| OFF previously current prediction | 0.929 | 4.273 | 10.252 | 15.698 |
| OFF exact previous source endpoint F | 0.195 | 0.268 | 0.465 | 0.901 |
| OFF previous covered support `t_c` | 0.360 | 1.017 | 2.229 | 3.051 |
| ON previously current prediction | 0.883 | 4.697 | 11.165 | 21.380 |
| ON exact previous source endpoint F | 0.214 | 0.871 | 2.161 | 2.714 |
| ON previous covered support `t_c` | 0.408 | 1.109 | 2.713 | 4.565 |

Prediction-position changes are about 4.2 times the RMS changes at the newest covered support. Relative to the exact endpoint, the ratios are about 15.9 OFF and 5.4 ON. This is a descriptive comparison of different locations' fixed-time revisions, not a causal decomposition or weighting percentage.

The exact endpoint can be more position-stable than a nearby older support: the factor constrains relative poses at its exact endpoints through interpolation, not each intervening support independently. The table does not certify that every state behind F is equally stable.

Maximum orientation changes at the exact endpoint are `0.003420 rad` OFF and `0.013182 rad` ON. Complete angular and positional summaries are retained in `summary.json` and [exact endpoint changes](/home/yeranis/results/gnssfgo_source_wnoj_prediction_tail_20261006/exact_endpoint_changes.csv).

## Velocity and acceleration do not tell the same story

Same-time changes after the next information batch:

| Quantity and location | OFF median | OFF RMS | OFF max | ON median | ON RMS | ON max |
|---|---:|---:|---:|---:|---:|---:|
| Velocity change at old current prediction, m/s | 0.295 | 0.601 | 1.936 | 0.182 | 0.575 | 2.577 |
| Velocity change at old covered support, m/s | 0.0187 | 0.0259 | 0.0686 | 0.0123 | 0.0391 | 0.1298 |
| Linear tangent acceleration change at old current prediction, m/s² | 2.480 | 4.314 | 12.319 | 1.332 | 3.949 | 15.575 |
| Linear tangent acceleration change at old covered support, m/s² | 0.822 | 2.734 | 8.383 | 0.914 | 2.471 | 10.469 |

The newest covered support's velocity changes are much smaller: prediction/covered RMS ratios are 23.2 OFF and 14.7 ON. But acceleration-change ratios are only 1.58 and 1.60. Large acceleration revisions are already present inside the source-covered region.

At publication, maximum speed is 1.846 m/s OFF and 2.643 m/s ON at the current prediction, versus 0.317 and 0.366 m/s at the latest covered support. Maximum linear tangent acceleration is 9.323/11.550 m/s² at the current prediction and 8.163/10.223 m/s² at that covered support.

The boundary proxy is not the whole historical graph. Across **all common covered support states**, the largest next-batch velocity changes are 0.320 m/s OFF and 0.446 m/s ON, larger than at the selected latest covered support. Corresponding maximum position changes are 3.051/4.565 cm. Full state-by-state records are in `off_all_same_time_changes.csv` and `on_all_same_time_changes.csv`; these reuse old states across snapshots and are not independent samples.

Acceleration here is the model's stored tangent derivative, not a raw accelerometer reading or a separately measured world-frame physical acceleration. No true-motion bound was imposed. The evidence establishes sensitivity of these estimates to later information, not that the physical platform necessarily violated a motion limit.

## Concrete CBS ON transition

The largest position and velocity corrections occur between publications 61 and 62.

| Event | Timestamp ns |
|---|---|
| Physical time of the predicted state being compared | `1727775014683475000` |
| Earlier publication availability | `1727775014683476000` |
| Latest source endpoint then | `1727775014555357500` |
| Next publication availability | `1727775014783476000` |
| Latest source endpoint after the batch | `1727775014755356300` |

Before the batch, that state is 128.1175 ms beyond the latest arrived endpoint. Afterward, it is inside endpoint coverage. The batch contains **two new IDs**—one GLIM and one Kimera—and **62 revision replacements**, in addition to the next support extension.

At the same physical time `1727775014683475000 ns`:

| State component | Before | After |
|---|---|---|
| Position, m | `(0.025823, 0.172685, -0.328755)` | `(0.041558, -0.019217, -0.421691)` |
| World velocity, m/s | `(-0.242110, 2.300993, 1.277448)` | `(-0.017530, 0.044883, 0.052419)` |
| Linear tangent acceleration, m/s² | `(-0.766337, 9.917776, 5.870308)` | `(0.237973, -3.451947, -2.056428)` |
| Speed, m/s | 2.642926 | 0.071200 |

Position changes by **21.380 cm**, velocity by **2.577046 m/s**, and linear tangent acceleration by **15.575341 m/s²**. Over that same information batch, the old exact source endpoint changes by only **0.919 cm**. The old latest covered support changes by 1.176 cm, 0.04289 m/s in velocity, and 4.994 m/s² in linear tangent acceleration.

The visible displacement between two successive published positions decomposes exactly as:

```text
p_new(t_new) - p_old(t_old)
  = [p_new(t_old) - p_old(t_old)]    same-time revision
  + [p_new(t_new) - p_new(t_old)]    motion in the new snapshot
```

Here the same-time revision norm is 21.380 cm, while the new snapshot's motion over the next 100 ms is 1.400 cm. The published displacement is 22.774 cm. These are vector terms; their norms should not generally be added as scalars. This example shows why an as-published polyline can appear to jump even when the later reconstructed path is short.

An OFF example at publications 59→60 has a 15.462 cm same-time prediction correction versus a 0.590 cm old-endpoint correction; speed changes from 1.846 to 0.277 m/s. The largest OFF position correction, 63→64, is 15.698 cm. Full pre/post vectors, arrival counts and endpoint/support comparisons are saved in [OFF traces](/home/yeranis/results/gnssfgo_source_wnoj_prediction_tail_20261006/off_peak_traces.json) and [ON traces](/home/yeranis/results/gnssfgo_source_wnoj_prediction_tail_20261006/on_peak_traces.json).

## What subsequent information changes

For 69 previously current predictions per mode, the frontier first reaches the compared physical time at a later retained snapshot. Median observed waiting time is 0.20 s; the maximum is about 0.30 s. This is publication-resolution waiting time, not an exact source-callback latency.

| Same-time position revision | OFF RMS, cm | OFF max, cm | ON RMS, cm | ON max, cm |
|---|---:|---:|---:|---:|
| Prediction until first observed endpoint coverage | 4.267 | 15.698 | 4.856 | 21.380 |
| Prediction until final reconstruction | 4.578 | 17.081 | 5.066 | 21.350 |
| Exact source endpoint until final reconstruction | 0.760 | 1.618 | 1.537 | 3.484 |

The final reconstruction remains noncausal with respect to the earlier publication and is not used to replace it. The experiment's output policy has not been changed to conceal prediction errors.

## Interpretation and limits

Three findings are established:

1. Finishing optimization does not remove the large provisional prediction revisions; the compared snapshots already meet the numerical convergence targets.
2. Source-endpoint positions are substantially less revision-sensitive than the newest predicted positions. The large visible jumps are concentrated beyond current measurement coverage.
3. Motion derivatives are not equally settled. Acceleration changes substantially even behind the frontier, and some older covered velocities also change. Thus a good pose estimate does not certify a reliable acceleration estimate for prediction.

The pattern is consistent with uncertain or sensitive motion-derivative estimates being amplified when predicting beyond the latest source endpoint. That is an interpretation, not proof that a particular source factor, covariance block or WNOJ parameter is responsible. This analysis has not measured calibrated uncertainty or isolated each callback's effect.

Therefore the next unresolved model question is **how well velocity and acceleration are constrained before they are used to predict forward**, not simply whether we should publish a delayed path. A delayed-output policy alone would hide the largest tail excursions while leaving derivative sensitivity unresolved. No such policy, derivative prior, source weighting or IMU reintroduction is implemented here.

## Figures

The first figure compares motion estimates at the current support and latest covered support in each snapshot. Those locations have different physical timestamps; it is a comparison of regions, not a correction calculation. The second figure computes every correction at an identical physical timestamp before and after the information batch.

![Motion and measurement coverage](/home/yeranis/results/gnssfgo_source_wnoj_prediction_tail_20261006/motion_and_coverage.png)

![Same physical time corrections](/home/yeranis/results/gnssfgo_source_wnoj_prediction_tail_20261006/same_time_corrections.png)

## Validation and reproducibility

- All 150 active revision ledgers reconstruct exactly from causal insertion records up to the respective snapshot availability timestamp.
- All 20,820 protected prior files, inputs and production files retain their hashes. Native source trajectories, actual published output and final reconstructed output are not modified.
- The read-only endpoint query calls the existing `FullWnojEndpoint::pose()` with the unchanged nominal Qc. It constructs no factor graph and invokes no optimizer.
- 1,556 pose queries are exported. Against 1,132 retained support/dense-pose checks, maximum position discrepancy is `1.26e-14 m` and rotation discrepancy `4.27e-16 rad`.
- Exact endpoint, fractional constant-velocity and out-of-range rejection checks pass. Release and ASan/UBSan query outputs match exactly. Existing linked runtime libraries are not sanitizer-instrumented; the ABI-compatible Eigen allocation flag is retained.
- The first diagnostic export had a CRLF serialization error and was rejected by strict CSV parsing before use. Only corrected `endpoint_poses_v2*.csv` products feed the result. No receiver output was affected.
- Repeated postprocessing produces byte-identical numerical results in all eleven checked analysis products. This does not supersede the prior receiver's tiny unaudited-prefix repeatability caveat.
- An independent direct subtraction of the saved ON61/ON62 state rows verifies the reported 21.380 cm and 2.577046 m/s corrections.
- Both figures were visually inspected. No GT, ATE/RPE, accuracy-based tuning, new source run or receiver run was performed.

The [pre-analysis protocol](/home/yeranis/results/gnssfgo_source_wnoj_prediction_tail_20261006/PROTOCOL_BEFORE_ANALYSIS.md) SHA-256 is `4254f01f469a19dd9139cb1590b70810364417c9b61129c90fad47e4fe78c50e`. Scripts, exact build commands, logs, input preservation manifest, analysis rows and verification JSON are in the artifact directory.

Postprocessing commands:

```bash
/home/yeranis/.local/share/pipx/venvs/evo/bin/python \
  /home/yeranis/results/gnssfgo_source_wnoj_prediction_tail_20261006/analyze.py
/home/yeranis/.local/share/pipx/venvs/evo/bin/python \
  /home/yeranis/results/gnssfgo_source_wnoj_prediction_tail_20261006/finalize.py
```

The query utility was compiled and run in the existing frozen image `sha256:6a664509d183e504230b22edfd747dfe1ca5bca8ae6fc34c76154db8ffd1fc8d`, with network disabled and source/runtime/prior artifacts read-only. Its exact argv are in `commands_v2.json`; the driver is `run_query.py v2`. Release build took 12.50 s, export 0.035 s, sanitizer build 30.51 s and sanitizer export 0.220 s. These are query-utility costs, not fusion timing measurements. Reproduction of its exclusive output files requires a fresh label/destination.

Repository HEAD is unchanged at `a842e7ef72e7a40f0cdfec826f089c66af364ae0`; no commit was created. The only new repository file from this task is this report. The existing dirty worktree is preserved, not declared clean; `git diff --check` passes. No production or model change follows this diagnostic.

**PREDICTION_DOMINATED_POSITION_CHANGES_WITH_COVERED_STATE_REVISIONS**
