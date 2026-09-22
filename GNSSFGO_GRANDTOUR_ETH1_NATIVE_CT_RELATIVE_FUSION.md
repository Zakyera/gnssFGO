# gnssFGO native continuous-time GrandTour relative-belief experiment

Date: 2026-09-22

## Outcome

The first 40 seconds of GrandTour ETH1 now run through three deterministic
gnssFGO fixed-lag factor-graph controls:

1. native-time GLIM only;
2. native-time Kimera only;
3. GLIM and Kimera together.

Kimera is not rounded to the GLIM support grid and is not transformed using the
earlier EKF temporal-alignment approximation. Its two native timestamps are
represented by continuous-time WNOJ interpolation inside the relative factor.

| Case | GLIM factors | Kimera factors | AP20 ATE RMSE | 1 s t-RPE RMSE | 1 s r-RPE RMSE | Full path |
|---|---:|---:|---:|---:|---:|---:|
| GLIM | 389 | 0 | 0.070180 m | 0.047558 m | 1.815599 deg | 14.294099 m |
| Kimera | 0 | 180 | 0.019461 m | 0.017590 m | 0.315221 deg | 13.794918 m |
| Joint | 389 | 180 | 0.047067 m | 0.033465 m | 1.164319 deg | 13.912390 m |

The joint result lies between the two single-source controls for all three
reported errors. This is a successful integration result, not yet a general
performance conclusion. In particular, AP20 prism coverage is only 14.100256 s
of the 38.999880 s graph window. The 1 s RPE values use CPT7 IE-TC over all 380
valid full-window pairs and are therefore the more representative 40 s result.

## Inputs

- Mission: GrandTour `2024-10-01-11-29-55` (ETH1).
- Window: 1727775008.675043106 to 1727775047.674923182.
- IMU CSV:
  `/home/yeranis/cbs_imu_host_experiment_20260914/imu.csv`
  (`925d6cd8c10000434a9f92dd0ba4565798a5a2ef51cf249c400147a32a11a708`).
- ALL_RIGHT GLIM beliefs:
  `/home/yeranis/cbs_cpt7_common_convention_40s_pilot_20260921/all_right/glim_beliefs.csv`
  (`59ee8ead374f8aee87b464e4834f83ca0c1590ec3c175667752c97d46b104316`).
- ALL_RIGHT Kimera beliefs:
  `/home/yeranis/cbs_cpt7_common_convention_40s_pilot_20260921/all_right/kimera_beliefs.csv`
  (`e831dd552ca784c90b90351a716a2b948768637d13fdd061f5fa64d02b551122`).
- AP20 prism and CPT7 IE-TC references are the same retained bags used by the
  preceding GLIM baseline.

No source mean or covariance was altered. The full dense 6x6 covariance in
right-local Pose3 tangent order `[phi,rho]` is passed directly to each source
factor.

## Graph architecture

The runner is
`online_fgo/src/offline_process/GrandTourCsvReplay.cpp`, built as
`grandtour_ct_replay`. It supports `--mode glim|kimera|joint`.

The common state grid is the union of the native GLIM endpoints: 391 support
states. Every support contains Pose3, world velocity, IMU bias and body angular
rate. Consecutive support states receive the unchanged components from the
GLIM baseline:

- one GTSAM `CombinedImuFactor` using ALL_RIGHT fractional integration;
- one gnssFGO `GPWNOJPrior`;
- a 3 s `IncrementalFixedLagSmoother` update.

A source endpoint is dispatched as follows:

| Endpoint relationship to support grid | Factor |
|---|---|
| both exact | `BetweenPose3Factor` |
| one interpolated | `GPInterpolatedSinglePose3BetweenFactor` |
| both in one support interval | `GPInterpolatedSameIntervalPose3BetweenFactor` |
| both in distinct intervals | `GPInterpolatedDoublePose3BetweenFactor` |

All 389 GLIM factors are exact. Of 182 Kimera beliefs, 180 have both endpoints
inside two distinct support intervals and therefore use the double-interpolated
factor. Kimera ordinals 0 and 181 are outside the bracketing GLIM support range
and are deterministically excluded. No endpoint is snapped or extrapolated.

Each Kimera residual is therefore evaluated as

```text
Log(Z_K^-1 * (T_hat(t_i)^-1 * T_hat(t_j)))
```

where each `T_hat` is a WNOJ interpolation from its adjacent support states.
The original Kimera covariance remains the factor noise model. GP interpolation
uncertainty is represented by the uncertain support variables and WNOJ priors
in `H P H^T`; it is not added a second time to the source covariance.

The joint graph uses separate GLIM and Kimera factors and consequently makes
the first-order independence approximation implicit in a conventional factor
graph. It does not model their hidden shared-IMU/common-history dependence.

## Jacobian correction and validation

The stock single- and double-interpolated relative-factor implementations
omitted the `Between(Pose3)` endpoint Jacobian in their velocity and angular
rate chain rules. For example, they evaluated `H_local H_interp` instead of
`H_local H_between H_interp`. The missing chain factors were restored for all
affected blocks:

- single factor: H2, H3, H5 and H6;
- double factor: H2, H3, H5, H6, H8, H9, H11 and H12.

Independent central finite differences validate:

- same-support interpolation;
- single interpolation with the interpolated pose as either endpoint;
- distinct-support WNOA double interpolation;
- distinct-support WNOJ double interpolation;
- fixed-lag optimization and interpolation covariance recovery;
- preservation of a dense measurement covariance.

The stock closed-form WNOJ interpolation Jacobians did not pass the independent
test (maximum block error 0.06199 in the synthetic case). The experiment uses
the interpolator's existing numerical-Jacobian mode instead. This changes no
state, motion prior, measurement or covariance; it only requests the already
implemented numerical derivative for WNOJ interpolation. With this mode all
12 double-factor blocks pass the finite-difference check.

## Detailed results

| Quantity | GLIM | Kimera | Joint |
|---|---:|---:|---:|
| Accepted beliefs | 389 | 180 | 569 |
| Beliefs lacking support | 0 | 2 | 2 |
| Double-interpolated factors | 0 | 180 | 180 |
| Internal CPT7-pose path | 14.320746 m | 13.862504 m | 13.979968 m |
| Maximum active values | 124 | 124 | 124 |
| AP20 matched poses | 142 | 142 | 142 |
| AP20 matched duration | 14.100256 s | 14.100256 s | 14.100256 s |
| Valid 1 s IE-TC RPE pairs | 380 | 380 | 380 |

Posterior factor residuals confirm that every accepted factor participates in
the solution. They are optimization diagnostics, not accuracy measurements:

| Case | Mean translation residual | Maximum translation residual | Mean rotation residual | Maximum rotation residual |
|---|---:|---:|---:|---:|
| GLIM | 0.006333 m | 0.029662 m | 0.000234 rad | 0.001109 rad |
| Kimera | 0.000386 m | 0.001800 m | 0.000017 rad | 0.000089 rad |
| Joint | 0.005958 m | 0.034845 m | 0.000212 rad | 0.001219 rad |

The very strong Kimera-only result should be treated cautiously until longer
windows and sequences with broader reliable GT coverage are run. It is not
evidence that covariance tuning is unnecessary or that one estimator is
universally superior.

## Scope boundary

This remains an offline physical-time experiment. A factor is added when its
right bracketing support state is processed. `record_timestamp_ns` is retained
in the belief input but is not used to simulate delayed online availability.
This is not the causal historical replay architecture previously validated for
the EKF, and these numbers must not be described as an online causal result.

This experiment also does not use the prior EKF pair map, mean alignment,
aligned covariance, synchronous 12D update, CI, gating, robust loss, covariance
scaling or source-specific tuning.

## Reproducibility

Two complete replays produced byte-identical trajectory, state and residual
files for every case:

| Case | `trajectory.tum` SHA-256 | `states.csv` SHA-256 | residual SHA-256 |
|---|---|---|---|
| GLIM | `097c9e1c5a8189c8c22ad8f64340c26cdce5b0f0f9289f1d97b2e58b9bcf928d` | `4eab2528d76741c3548de770ac7b80137371f3cc72a2c817f99bd51d37a99692` | `87491009ffc5535e9502502f3dd6b027a18527d5de2e999de849bbea000ec17f` |
| Kimera | `8db2bbc508cd0df0841b8427de2467f943409bab858488316391c486aa7e3f37` | `f120f25e3f0ad295524f88007413129378de331440d04a9ff4eea771bf083a35` | `dee661297f0f580fec31f7493aaa215946cbce3f40bed40a9e78667353e910a2` |
| Joint | `f625f660954accb9090674a0b0f08e811296ec3187dbe10f007693616938c5e0` | `1a1b66a4d3b868f837833d49002760e61e6be9ddef4efa8a7222cb46e6a8f131` | `7bfca374a9ad2536e9ae091e8c1308d12973c5c297ae70f2b3eca0c5bb43f8fe` |

Artifacts are retained at:

```text
/home/yeranis/results/gnssfgo_grandtour_eth1_ct_native_autodiff_20260922
```

Validation status:

- full Release package build: pass;
- focused CT factor tests: 6/6 pass;
- ALL_RIGHT overlap tests: 4/4 pass;
- deterministic repeated replay: pass;
- `git diff --check`: pass;
- package-wide CTest: the two scientific tests and three existing linters pass;
  five repository-wide legacy lint tests fail on pre-existing files
  (`copyright`, `cpplint`, `flake8`, `lint_cmake`, `uncrustify`);
- separate combined ASan/UBSan execution is not valid in the retained binary
  environment: the instrumented test process repeatedly terminates inside the
  image with `AddressSanitizer:DEADLYSIGNAL` before producing a usable report.

## Interpretation and next boundary

The concrete technical question is answered: gnssFGO can consume a native
relative Pose3 belief whose two endpoints are at arbitrary, distinct
continuous timestamps, optimize it in `IncrementalFixedLagSmoother`, and keep a
bounded active trajectory. Native GLIM+Kimera fusion also runs deterministically
on ETH1 without timestamp rounding.

The next defensible step is a longer, causal-availability experiment or a
second GrandTour sequence using the same frozen graph settings. It should not
start by tuning either source covariance from this 40 s result.
