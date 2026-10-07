# ETH1 source driven WNOJ 40 second comparison

The first real-data comparison completed in all four requested receiver cases. Removing central raw IMU use makes the receiver fit most GLIM/Kimera relative beliefs much more closely, but the new model produces an unhealthy, highly variable **as-played** trajectory. Its final retrospectively revised trajectory is much better than its as-played output, but does not outperform the existing receiver overall. **Do not promote this configuration to the live baseline.**

No GLIM or Kimera estimator was rerun. No covariance, Qc, prior, source setting, factor policy or estimator code was changed during this experiment. The source-driven mode is the previously implemented opt-in model from [the implementation report](/home/yeranis/repos/gnssFGO/GNSSFGO_SOURCE_WNOJ_IMPLEMENTATION_20261005.md). This report evaluates that frozen implementation; it does not implement a fix.

## Scope and input provenance

Artifacts: [gnssfgo_eth1_source_wnoj_40s_20261005](/home/yeranis/results/gnssfgo_eth1_source_wnoj_40s_20261005).

Both inputs come from the retained [ETH1 live 120 second experiment](/home/yeranis/repos/gnssFGO/GNSSFGO_ETH1_LIVE120_CLASSIC_ZERO_SOURCE_MODEL_20260930.md), mission `2024-10-01-11-29-55`, source bag offset zero. That source capture ran at 0.10×. **The present experiment is offline causal receiver replay**, not a new live source run.

Original journals:

- [OFF events](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live120_zero_20260930/off_receiver/events.csv).
- [ON events](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live120_zero_20260930/on_receiver/events.csv).

The bounded prefix starts at the first recorded central IMU availability, `1727775008003475000 ns`, and ends inclusively at `1727775048003475000 ns`, exactly 40 seconds later. This is a precisely defined receiver-journal window, not an assertion that the first bag record has that same timestamp. All original event rows through the cutoff are copied unchanged and in their original order. An explicit end marker is appended at the cutoff. Later arrivals are excluded, including later revisions of edges physically inside the window. There is no post-cutoff source drain.

| Input | OFF | ON |
|---|---:|---:|
| IMU rows | 4,000 | 4,000 |
| GLIM belief-version messages | 17,805 | 17,769 |
| Kimera belief-version messages | 4,275 | 4,275 |
| Total belief-version messages | 22,080 | 22,044 |
| Distinct source IDs and endpoint pairs before support checks | 574 | 573 |

The physical IMU timestamps and all six motion values are identical between OFF and ON. Their ordered physical-payload SHA-256 is `630b56874ae53a0e2290871b35e008474775bcfdbfd3d7cdccebf66267f805a7`. Availability jitter and source arrivals differ between the original OFF/ON captures. These differences were preserved, not normalized away. Within each mode, both receiver models consume exactly the same journal.

Prefix SHA-256:

```text
OFF 89884ce8a4ebab0496dda5f140a0c390a3f8435a1d4289e03d50200acd5a3010
ON  8a89b6b58a605615511c7eb3e1514ffcd8d9233f3b1f8988a064e62d77417182
```

The [manifest](/home/yeranis/results/gnssfgo_eth1_source_wnoj_40s_20261005/manifest.json) pins 96 protected files, including original journals, native exports, saved profiles, resolved parameters, evaluator scripts, ground-truth bags and receiver implementation/binaries.

## Frozen source and receiver configurations

Sources remain the saved ETH1 Hesai+CPT7 GLIM and front AlphaSense stereo+CPT7 Kimera. Source-to-source CBS is **classic**, not `live_revision`: 0.20 s horizon, one pair per update, publication stride one, persistent external factors and full `schur_relative_between` covariance, scale 1. The original Kimera `current` packet convention is unchanged. Source diagnostics independently supplied current-smoother revisions to the receiver.

Shared receiver settings are source-independent 100 ms support spacing, the clock anchored to the 59th IMU timestamp, full-history iSAM2, stable-ID revision replacement, full dense 6×6 source covariance with scale 1, no added uncertainty, no gating, no robust kernel, no health weighting and no GNSS. iSAM2 uses QR, relinearization threshold 0.01, skip 1 and unused-factor-slot reuse. Qc stays at six diagonal variances of 10,000. No extra batch optimization was added at the cutoff; “final” means the final incremental estimate after the included journal events.

| Receiver component | Existing `imu_wnoj` | Experimental `source_wnoj` |
|---|---|---|
| Support variables | Pose X, velocity V, angular rate W, bias B | Pose X, velocity V, angular rate W, tangent acceleration A |
| Central raw IMU | Combined IMU, exact ALL_RIGHT integration | No integration or IMU measurement factor |
| Motion prior | Existing WNOJ with fixed IMU-derived acceleration inputs | Existing full WNOJ with estimated acceleration at both supports |
| Interpolation | Existing WNOJ interpolator | Existing full WNOJ interpolator |
| IMU startup values | Existing 59-sample gravity/bias initialization | Ignored; timestamps only establish the common clock |
| Prediction initial guess | IMU preintegration | Zero-process-noise full-WNOJ transition |
| Initial priors | Existing pose/velocity/bias/angular-rate priors | Previously frozen pose/velocity/angular-rate/acceleration priors |

The old prior sigmas remain pose `[0.174533,0.174533,0.00174533,1e-5,1e-5,1e-5]`, velocity 0.001 per axis, bias `[0.1,0.1,0.1,0.01,0.01,0.01]`, and angular rate 0.1. The new priors remain identity pose with sigma `1e-5`, zero velocity with sigma 10 m/s, zero angular rate with sigma 3 rad/s, and zero tangent acceleration with sigma 10 per coordinate. They were set before real-data scoring, not adjusted here.

Consequently this is a comparison of two complete, frozen receiver models, **not a single-factor deletion with otherwise identical state and initialization**. In the new mode, acceleration is estimated and can change under the jerk prior; it is not constant throughout the trajectory. The existing full-WNOJ numerical derivative path validated in the preceding task remains selected. Library mathematics were not edited.

This removes direct central raw-IMU reuse. It does not remove common IMU information or CBS feedback correlations already embedded in the two source beliefs.

## Causality and factor accounting

All four cases create 395 supports from `1727775008583475000` to `1727775047983475000 ns`, spanning 39.4 seconds after startup. Relative endpoints remain at their own physical timestamps; an update waits for both its logged arrival and the required supports. In this window all accepted source factors have two interpolated endpoints. Revisions replace the same source/session/endpoint ID, not a second simultaneous factor.

| Accounting per receiver model | OFF | ON |
|---|---:|---:|
| Active GLIM belief IDs | 391 | 390 |
| Active Kimera belief IDs | 182 | 182 |
| Total active relative factors | 573 | 572 |
| Successful version insertions, including replacements | 22,055 | 22,019 |
| Old versions removed | 21,482 | 21,447 |
| Early version messages without support | 25 | 25 |
| Duplicate or stale revisions | 0 | 0 |
| Pending at end | 0 | 0 |

Within each OFF/ON mode, these files are byte-identical between models: support clock, causal dispatch, source accounting, unsupported-belief accounting and source covariance output. Maximum source-to-factor covariance entry difference is `9.32e-21` OFF and `3.39e-20` ON. Native and effective covariance tables match across models; no uncertainty addition occurred.

The new receiver's actual factor-type audit finds **zero IMU factors and zero bias variables**, 395 acceleration variables, 394 full-WNOJ factors and four initial priors. Its final total is 971 factors OFF and 970 ON. The retained slight OFF/ON source-count difference comes from actual availability at the common cutoff, not the receiver-model change.

## Evaluation and reference limitations

The unchanged project evaluators are `score_grandtour_prism.py` and `score_grandtour_relative_pose.py`. ATE uses AP20 prism positions, the existing CPT7-to-prism lever arm and independent rigid SE(3) alignment without scale. One-second RPE uses CPT7 IE-TC, the same lever arm and no global alignment. Ground truth never enters either receiver.

For strict primary comparisons, all 12 native/fused products are evaluated on the **same 364 physical timestamps** from `1727775011.583475113` to `1727775047.883475065`. Native outputs are resampled only for evaluation with the existing linear-position/SLERP convention and maximum 0.5 s gap; no estimator input is resampled. The early native Kimera output gap prevents scoring every initial support. Selection is by timestamp availability, never by error.

All primary ATE masks are identical: **113 prism matches**, from `1727775011.583475113` to `1727775022.783475161`, giving **11.2 seconds** of coverage and **3.605208 m** reference path. ATE therefore does **not** measure the entire 40-second window. All primary RPE masks are identical: **353 one-second pairs** across the longer common interval. [Mask verification](/home/yeranis/results/gnssfgo_eth1_source_wnoj_40s_20261005/evaluation_masks_verified.json) and [evaluation protocol](/home/yeranis/results/gnssfgo_eth1_source_wnoj_40s_20261005/evaluation_protocol.json) retain the exact choices.

Native GLIM is the familiar saved `odom_imu.txt` export, not a gnssFGO GLIM-only graph. Native Kimera is the original ROS-odometry export, not optimizer `state_`. The GLIM export was produced at the end of the original 120 s source run; it is **not a reconstructed source snapshot at the present 40 s arrival cutoff**. Its first-window positions differ from the saved publication stream by RMS 0.01045 m OFF and 0.01509 m ON, with maxima 0.05047/0.08777 m. The native GLIM rows below preserve the established native-export reference convention; only receiver-to-receiver comparisons have the strict same-information cutoff guarantee. Both original-sampling native scores are also retained in `metrics.json` to make resampling effects auditable.

## Native and fused results

Distances below are the prism-frame path over the common evaluation timestamps, including portions without prism GT. They are not full-sequence distances.

| Mode | Original source output | ATE m | 1 s t-RPE m | 1 s r-RPE deg | Path m |
|---|---|---:|---:|---:|---:|
| OFF | Native GLIM | 0.055799 | 0.040203 | 0.549638 | 14.1798 |
| ON | Native GLIM | 0.043442 | 0.040966 | 0.573803 | 14.2304 |
| OFF | Native Kimera ROS odometry | 0.029156 | 0.022980 | 0.633515 | 13.4249 |
| ON | Native Kimera ROS odometry | 0.055881 | 0.034982 | 0.693908 | 13.7150 |

In this particular short section, Kimera ON is worse than OFF. Its previously reported improvement over 120 seconds must not be transplanted into this window.

### Final retrospectively revised fusion

| Receiver | Mode | ATE m | 1 s t-RPE m | 1 s r-RPE deg | Path m |
|---|---|---:|---:|---:|---:|
| Existing IMU + WNOJ | OFF | 0.040787 | 0.021710 | 0.598192 | 13.8297 |
| Existing IMU + WNOJ | ON | 0.043175 | 0.023975 | 0.641904 | 13.8606 |
| Source-driven WNOJ, no central IMU | OFF | 0.053275 | 0.038924 | 0.588118 | 15.0931 |
| Source-driven WNOJ, no central IMU | ON | 0.047587 | 0.041878 | 0.593297 | 15.4127 |

Within the new model, ON improves short-coverage ATE by about 10.7%, but worsens translational RPE by about 7.6%, slightly worsens rotational RPE, and lengthens the path. Relative to the old receiver in the same mode, the new receiver has worse ATE and translational RPE, with a modest rotational-RPE improvement. This is not an overall accuracy gain.

### As played fusion

These are the immutable estimates emitted at each support creation during causal replay. They are not overwritten by later revisions, and they are not a separately co-running live experiment.

| Receiver | Mode | ATE m | 1 s t-RPE m | 1 s r-RPE deg | Path m |
|---|---|---:|---:|---:|---:|
| Existing IMU + WNOJ | OFF | 0.056858 | 0.043400 | 0.941328 | 14.0063 |
| Existing IMU + WNOJ | ON | 0.065017 | 0.066909 | 1.631149 | 15.7370 |
| Source-driven WNOJ, no central IMU | OFF | 0.231602 | 0.223941 | 6.254165 | 85.2320 |
| Source-driven WNOJ, no central IMU | ON | 0.159135 | 0.313593 | 5.481558 | 105.5222 |

The new as-played output is clearly unhealthy even though the final revised graph looks much better. ON improves its limited-coverage ATE and rotational RPE but further worsens translational RPE and path inflation. Reporting only the final ATE would conceal the main failure in this pilot.

All unrounded metrics, evaluation/GT-overlap paths and individual error samples are in [metrics.json](/home/yeranis/results/gnssfgo_eth1_source_wnoj_40s_20261005/metrics.json) and [evaluation](/home/yeranis/results/gnssfgo_eth1_source_wnoj_40s_20261005/evaluation).

## Source residuals

These are **post-fit residuals of the final active belief versions**, not innovations, calibrated NIS or GT-relative belief errors. The reported squared whitened norm is `rᵀ R⁻¹ r`, using the same unchanged dense source covariance and existing Pose3 local-coordinate residual.

| Receiver | Mode | Source | Translation residual RMS m | Rotation residual RMS rad | Median squared whitened norm | Mean squared whitened norm |
|---|---|---|---:|---:|---:|---:|
| IMU + WNOJ | OFF | GLIM | 0.008983 | 0.000366 | 27.967 | 111.184 |
| IMU + WNOJ | OFF | Kimera | 0.004805 | 0.000255 | 27.299 | 51.550 |
| IMU + WNOJ | ON | GLIM | 0.017819 | 0.000525 | 273.286 | 888.411 |
| IMU + WNOJ | ON | Kimera | 0.004968 | 0.000171 | 82.090 | 173.008 |
| Source WNOJ | OFF | GLIM | 0.001105 | 0.000057 | 0.432 | 1.753 |
| Source WNOJ | OFF | Kimera | 0.002103 | 0.000078 | 1.862 | 3.768 |
| Source WNOJ | ON | GLIM | 0.001137 | 0.000038 | 0.734 | 2.193 |
| Source WNOJ | ON | Kimera | 0.015136 | 0.000123 | 1.225 | 303.536 |

The much lower typical source residuals confirm that the new graph conforms more closely to received relative motion. That alone does not establish more accurate or sensible motion between those measurements.

The ON Kimera mean is dominated by one retained tail factor: source `kimera_front`, edge `183→184`, revision 0, physical interval `1727775047755173400–1727775047955172500 ns`, arrival `1727775047977187500 ns`. Its final translation residual is **0.203917 m**, rotation residual **0.001640 rad**, and squared whitened norm **54,781.842**. No factor was gated or discarded. The payload, full covariance and residual are retained in `analysis.json` under `worst_event_trace`. Its being the tail factor is an observation, not a proven explanation of the residual.

## Velocity and acceleration health

Statistics cover all 395 initialized supports, including startup and the terminal prediction region, not just the GT mask.

| Receiver | Mode | Product | Speed median m/s | Speed p95 m/s | Speed maximum m/s | Full support body path m |
|---|---|---|---:|---:|---:|---:|
| IMU + WNOJ | OFF | Final | 0.395 | 0.651 | 0.879 | 13.957 |
| IMU + WNOJ | ON | Final | 0.390 | 0.641 | 0.880 | 13.999 |
| IMU + WNOJ | OFF | As played | 0.392 | 0.643 | 0.877 | 14.188 |
| IMU + WNOJ | ON | As played | 0.398 | 0.645 | 0.862 | 15.873 |
| Source WNOJ | OFF | Final | 0.606 | 2.522 | 3.905 | 15.498 |
| Source WNOJ | ON | Final | 0.894 | 2.242 | 9.325 | 16.118 |
| Source WNOJ | OFF | As played | 2.365 | 15.358 | 42.935 | 89.121 |
| Source WNOJ | ON | As played | 2.998 | 11.712 | 22.490 | 106.019 |

The largest as-played speeds occur at stream offsets **14.780 s OFF** and **20.780 s ON**, not only at startup. Their complete emitted states and final states at the same physical timestamps are retained in `worst_event_trace`.

New-mode acceleration norms below describe the existing full-WNOJ **tangent acceleration**, not raw accelerometer readings or necessarily Cartesian world acceleration.

| Mode | Product | Translational tangent acceleration median / p95 / max m/s² | Angular tangent acceleration median / p95 / max rad/s² |
|---|---|---|---|
| OFF | Final | 5.346 / 24.966 / 52.201 | 4.375 / 12.461 / 30.265 |
| ON | Final | 10.454 / 31.106 / 45.862 | 5.289 / 12.574 / 24.845 |
| OFF | As played | 16.007 / 82.451 / 132.026 | 12.609 / 28.837 / 49.041 |
| ON | As played | 19.417 / 64.380 / 164.716 | 10.536 / 29.801 / 40.254 |

For context only, the old model's fixed IMU-derived translational acceleration inputs have median about 1.53, p95 about 6.7 and maximum about 15.6 m/s². They are not optimized A variables, so this is not a like-for-like acceleration-estimation comparison. Mean, RMS and full descriptive statistics are in [analysis.json](/home/yeranis/results/gnssfgo_eth1_source_wnoj_40s_20261005/analysis.json).

The evidence establishes poor derivative/as-played behavior at this frozen operating point. It does not yet isolate how much comes from motion-prior strength, delayed-source prediction, initialization, interpolation flexibility or incremental optimization. No new cause is asserted from ATE alone, and none of these components was tuned here.

## Computational cost

Primary cases ran sequentially, without source estimators or graph visualization.

| Receiver | Mode | Wall time s | Wall time per 40 s data | Revision-update median ms | p95 ms | Maximum ms |
|---|---|---:|---:|---:|---:|---:|
| IMU + WNOJ | OFF | 116.79 | 2.92× | 1.242 | 19.816 | 111.724 |
| IMU + WNOJ | ON | 149.33 | 3.73× | 1.402 | 30.783 | 141.433 |
| Source WNOJ | OFF | 192.90 | 4.82× | 1.189 | 37.128 | 90.588 |
| Source WNOJ | ON | 268.72 | 6.72× | 5.558 | 43.557 | 82.293 |

The new model is slower here despite removing Combined IMU. It estimates A and uses the previously validated numerical derivative path. Those are relevant implementation differences, not a measured runtime attribution. These standalone replay timings do not certify live co-running capacity at any bag speed.

## Reproducibility and validation

The prepared prefixes and all commands are retained in [prepare.py](/home/yeranis/results/gnssfgo_eth1_source_wnoj_40s_20261005/prepare.py), [run.py](/home/yeranis/results/gnssfgo_eth1_source_wnoj_40s_20261005/run.py), [commands.json](/home/yeranis/results/gnssfgo_eth1_source_wnoj_40s_20261005/commands.json), and [evaluation_commands.json](/home/yeranis/results/gnssfgo_eth1_source_wnoj_40s_20261005/evaluation_commands.json).

Exact receiver invocation, repeated for `mode=off/on` and `model=imu_wnoj/source_wnoj` with fresh output directories:

```bash
/binary/grandtour_source_wnoj \
  --receiver-model MODEL --mode sources --timeline record --solver isam2 \
  --support-policy uniform_imu --support-period-ns 100000000 \
  --receiver-startup-samples 59 \
  --event-stream /audit/inputs/MODE_events.csv --out /audit/MODE_MODEL
```

Receiver image: `sha256:6a664509d183e504230b22edfd747dfe1ca5bca8ae6fc34c76154db8ffd1fc8d`. The container had a read-only root filesystem, no network, UID/GID 1000, a temporary `/tmp`, and `LD_LIBRARY_PATH=/runtime/lib`.

| Container path | Host path | Access |
|---|---|---|
| `/audit` | `/home/yeranis/results/gnssfgo_eth1_source_wnoj_40s_20261005` | Read/write new artifacts only |
| `/binary` | `/home/yeranis/results/gnssfgo_source_wnoj_20261005/build` | Read-only |
| `/runtime` | `/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime` | Read-only |
| `/baseline`, verification only | `/home/yeranis/results/gnssfgo_source_wnoj_20261005/baseline` | Read-only |

Evaluation used the existing local `cbsms:noetic-cuda-glim` image, ID `sha256:7ad3854c699fb9a292aa5d42f8f890dfbd5dccb16633e043a93d135221418aaa`, with the project mounted read-only at its absolute host path, `/audit` writable, `/opt/ros/noetic/setup.bash` sourced, and `python3 /audit/score.py`. No ROS master, source node or bag playback was launched. Ground-truth bags were read by the unchanged scoring scripts.

Validation completed:

- All four primary replays exited successfully; states are finite and no pending belief remains.
- All primary prism and RPE masks are identical.
- Source-use, causal dispatch, covariance and support-clock files match exactly between receiver models within each mode.
- The new graph's factor-type audit verifies that central IMU factors and bias variables are absent.
- The preserved original receiver binary reproduces the new executable's default-mode OFF result **byte-for-byte** in ten numerical output files, including final/as-played poses, full exported states, source residuals and source covariance.
- The new ON determinism repeat is **byte-identical** in the same ten numerical output files, including final/as-played trajectories and exported states. Timing fields are intentionally excluded. Both successful verification records are in [repeat_regression.json](/home/yeranis/results/gnssfgo_eth1_source_wnoj_40s_20261005/repeat_regression.json).
- All 96 pinned protected files remain unchanged. This task added only experiment preparation/analysis artifacts and this report. No production or experimental estimator source was edited.

Synthetic/unit/sanitizer coverage belongs to the preceding implementation task, documented there. This experiment did not rebuild the receiver or claim a new full repository test-suite run.

The isolated experimental binary SHA-256 remains `51f05fcc6f90f04dfd7d239ddd100982030b4cf20bfdd40bd5ce43b93eca69c5`. The production binary and its preserved copy remain `0f175e4db81acb3624442ed7b3b232201e1b876676f2c148d0a747f009e706fa`. Repository HEAD remains `a842e7ef72e7a40f0cdfec826f089c66af364ae0`. No commit was created. The pre-existing dirty worktree is preserved; it is not described as clean. `git diff --check` passes.

## Conclusion and stopping point

The implementation runs on real saved arrivals and demonstrably removes direct central raw-IMU influence. It also exposes a serious limitation: closely fitting the sources with this frozen source-WNOJ configuration does not produce healthy as-played velocity/acceleration or a competitive causal trajectory. Final retrospective ATE alone would be misleading.

The limited new-model OFF→ON ATE improvement is mixed with worse translational RPE and cannot establish the desired full-window CBS benefit. Native Kimera itself is worse in this short section, and prism GT covers only 11.2 seconds. No universal CBS or WNOJ conclusion follows.

Keep the existing baseline unchanged and the new mode experimental. No longer sequence, second dataset, source rerun, live trial, covariance change or corrective follow-up was performed.
