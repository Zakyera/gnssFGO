# ETH1 WNOA and WNOJ numerical fix and completed 40 second comparison

## Outcome

The floating-point acceptance problem is resolved in the isolated experimental
receiver. All four frozen cases completed: CBS-OFF/WNOJ, CBS-OFF/WNOA,
CBS-ON/WNOJ and CBS-ON/WNOA, **without central IMU factors**. All 1,580
publication checks and 24 selected copied-graph batch comparisons passed.
Each case was repeated without the expensive snapshot audit; causal and final
trajectories, states, source accounting and refinement decisions were
byte-identical.

WNOA reduces the corrections to the current predicted pose when subsequent
information arrives. Position revision RMS falls from 40.10 to 14.87 mm in
OFF and from 77.20 to 27.46 mm in ON. Its final reconstructed support positions
remain close to WNOJ: 1.20 mm RMS difference in OFF and 1.76 mm in ON.
This supports WNOA as a less erratic prediction model **in this frozen short
experiment**. It does not solve the CBS-ON versus CBS-OFF accuracy gap, establish
a universally preferable motion model, or certify a live receiver.

Only the numerical step-acceptance policy changed. Qc, source covariance,
priors, motion equations, source arrivals and revision replacement are frozen.
The production/default receiver and its old strict helper remain unchanged.
No GLIM/Kimera rerun, central IMU factor, live run, full-sequence run or tuning
was introduced.

This completes the comparison stopped in
[the original report](/home/yeranis/repos/gnssFGO/GNSSFGO_ETH1_WNOA_WNOJ_40S_COMPARISON_20261006.md).
That report and its artifacts remain unchanged as the record of the failure.
New artifacts are in
[/home/yeranis/results/gnssfgo_eth1_roundoff_40s_20261006/](/home/yeranis/results/gnssfgo_eth1_roundoff_40s_20261006).

## Numerical acceptance rule

The unchanged publication requirement is a normalized gradient at most `1e-5`:

```text
g = ||diag(J^T J)^(-1/2) J^T r||_infinity
E = 0.5 * ||r_whitened||^2
```

This is a local first-order stationarity criterion, not proof of a global
minimum. Refinement still uses at most 20 transactional iSAM2 copies with full
relinearization/full solve. Factor slots and factor pointers must remain
unchanged. Rejected candidates do not replace the accepted solver.

The new opt-in predicate is
[CheckPublicationStep](/home/yeranis/repos/gnssFGO/online_fgo/include/offline_process/PublicationRoundoffAcceptance.h:21).
It preserves ordinary finite non-increasing objective steps. A positive
objective difference is allowed only when **all** of these conditions hold:

1. The increase is at most `8 * double_epsilon * max(1, E_before, E_candidate)`.
2. The candidate's actual normalized gradient is at most the unchanged `1e-5`.
3. Its gradient is strictly smaller than the preceding gradient.
4. Objectives, gradients and the state step are finite, with valid nonnegative
   objectives/gradients.

This is a small arithmetic comparison allowance, not a certified bound on
every nonlinear factor's floating-point error. It cannot admit a sequence of
increasing, unconverged steps: the exception requires convergence on that
very step. Material increases and invalid candidates remain rejected.
The allowance was recorded in `PROTOCOL_BEFORE_RUN.md` before executing the
comparison; it was not selected from trajectory accuracy.

The integration is in the isolated
[FullPublicationRefinement.h](/home/yeranis/results/gnssfgo_eth1_roundoff_40s_20261006/build/wnoa/FullPublicationRefinement.h:18).
The candidate gradient is now evaluated before deciding whether an objective
increase is acceptable. The old `BoundedISAM2Refinement.h` and its six tests
were not modified. The source-tree additions are only the opt-in predicate and
its focused test file; existing launch defaults do not enable this policy.

### Previously failing step

Exactly one of the 2,210 accepted refinement steps in the four primary cases
used the exception: OFF/WNOA at support 344, physical time
`1727775042983475000 ns`, 34.98 s after journal start.

| Quantity | Value |
| --- | ---: |
| Objective before candidate | 170127.49697813028 |
| Candidate objective | 170127.49697813034 |
| Increase | 5.820766091346741e-11, two floating-point units |
| Relative increase | 3.421414054010913e-16 |
| Allowed arithmetic difference | 3.0220714282714727e-10 |
| Gradient before candidate | 1.0836412817209803e-5 |
| Candidate gradient | 3.4549476777284284e-8 |
| Aggregate local-coordinate state step norm | 3.0214266056032748e-7 |

The aggregate step norm mixes state coordinates and units; it is not a position
displacement in metres. A separate LM solve on a copied graph agrees with the
accepted state to maximum differences of `4.4741e-11 m` in position,
`2.4721e-12 rad` in orientation, `7.5313e-10 m/s` in velocity and
`1.6099e-10 rad/s` in angular rate. The copied solution never feeds back into
the receiver. The acceptance change therefore resolves the numerical stop
without accepting the formerly unconverged state or weakening the threshold.

## Frozen data and model configuration

Source capture: ETH1 mission `2024-10-01-11-29-55`, retained native Hesai/CPT7
GLIM and front-stereo 25-state Kimera in
`/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live120_zero_20260930/`.
Source-to-source behavior is the saved classic CBS protocol, not experimental
`live_revision`. Downstream stable-ID revision replacement is separate.
The source-side IMU conventions are those of the retained profiles and were
not changed by this experiment.

The two arrival journals are under
`/home/yeranis/results/gnssfgo_eth1_source_wnoj_40s_20261005/inputs/`:

| Journal | SHA-256 |
| --- | --- |
| `off_events.csv` | `89884ce8a4ebab0496dda5f140a0c390a3f8435a1d4289e03d50200acd5a3010` |
| `on_events.csv` | `8a89b6b58a605615511c7eb3e1514ffcd8d9233f3b1f8988a064e62d77417182` |

Availability spans `1727775008003475000` through `1727775048003475000 ns`
inclusive. After 59 IMU timestamp events, the independent clock starts at
`1727775008583475000 ns`; 395 supports at 100 ms spacing span 39.4 s. Raw IMU
**values are ignored**: timestamps supply the clock, not prediction
measurements or startup gravity/bias estimates. No post-cutoff revisions are
drained into the graph.

Both models retain full-history iSAM2 with QR, full native 6x6 source covariance
at scale one, no gating, no robust loss, no health weighting, no GNSS and no
bias variables. Initial pose is identity with sigma `1e-5 rad/m`; world
velocity is zero with sigma `10 m/s`; body angular rate is zero with sigma
`3 rad/s`.

| Setting | WNOJ | WNOA |
| --- | --- | --- |
| Variables at each support | Pose X, world velocity V, body angular rate W, local tangent acceleration A | X, V, W |
| Existing motion prior | `GPWNOJPriorFull` | `GPWNOAPrior` |
| Existing interpolation | `GPWNOJInterpolatorFull` | `GPWNOAInterpolator` |
| Qc diagonal | `[2.5,2.5,2.5,10,10,10]` | `[.00375,.00375,.00375,.015,.015,.015]` |
| Rotational / translational Qc units | rad²/s⁵ / m²/s⁵ | rad²/s³ / m²/s³ |
| Extra initial prior | A=0, sigma 10 | None |
| Final variable keys | 1,580 | 1,185 |

Both use the same numerical-Jacobian path as the previous experiment. WNOA's
Qc was fixed before the earlier eight-second comparison by matching the
process **pose-covariance block** at 100 ms:
`q_WNOA = (3 * 0.1^2 / 20) * q_WNOJ`. This does not match all derivative
blocks or all prediction horizons. There was no new Qc sweep or GT-based
model selection here.

## Causality and source accounting

Beliefs are inserted only after both their recorded availability time and
their required receiver supports are available. A newer revision removes
the old factor for the same stable ID and replaces its mean and covariance;
it does not create a second concurrent observation. Both endpoints retain
their physical timestamps and use the existing GP interpolation path.

| Accounting | CBS-OFF, both models | CBS-ON, both models |
| --- | ---: | ---: |
| Source version events | 22,080 | 22,044 |
| Unique GLIM IDs | 391 | 390 |
| Unique Kimera IDs | 182 | 182 |
| Subsequent replacements | 21,482 | 21,447 |
| Unsupported pre-initialization versions | 25 | 25 |
| Stale revisions, duplicates, pending at end | 0 / 0 / 0 | 0 / 0 / 0 |

Within each OFF/ON mode, both motion models have byte-identical support clocks,
causal dispatch, source accounting, unsupported-event records and source
covariance exports. Every publication snapshot has the same source ID/revision
set across models. The selected snapshots also verify exact equality of 2,429
OFF and 2,428 ON full 6x6 source covariance matrices across models. Reported
and effective source covariance entries are identical: no added model noise
or scaling is present.

## Prediction and later reconstruction

The causal/as-played trajectory records the latest physical support at each
publication using information available then. The final trajectory revises all
historical supports using arrivals through the 40 s cutoff. Neither is a new
native source trajectory. The execution here is offline causal replay, not
three estimators running together live.

For each publication k, the revision metric compares the estimate of
**the same physical state t_k** before and after publication k+1. It does not
confuse ordinary robot movement from t_k to t_(k+1) with correction. There are
390 such comparisons after source information becomes available, including
the last regular publication to the final cutoff snapshot.

| Case | Current position revision RMS, m | Maximum, m | Current velocity revision RMS, m/s | Maximum published speed, m/s |
| --- | ---: | ---: | ---: | ---: |
| OFF WNOJ | 0.040098 | 0.181974 | 0.502792 | 2.089842 |
| OFF WNOA | 0.014873 | 0.057042 | 0.124519 | 0.772263 |
| ON WNOJ | 0.077197 | 0.483187 | 0.985386 | 5.798296 |
| ON WNOA | 0.027459 | 0.140178 | 0.228032 | 1.462366 |

WNOA reduces current-position revision RMS by 62.9% in OFF and 64.4% in ON.
The corresponding orientation revision RMS changes from 0.03888 to 0.01761
rad in OFF and from 0.04438 to 0.02070 rad in ON. The largest ON position
revision is at journal time 20.68 s, support 201 to 202, where the latest
source endpoint is 128.19 ms behind the current support. It falls from
0.48319 m to 0.14018 m. WNOA still has non-negligible corrections.

The mean age of the latest arrived source endpoint is about 86.8 ms OFF and
86.6 ms ON; the 95th percentile is 208.37 ms and maximum 208.44 ms. These are
prediction-tail durations, not CPU latency. They are the same across models.

At the latest support covered by at least one arrived source endpoint, position
revision RMS is smaller: 8.907 to 7.170 mm OFF and 18.126 to 15.459 mm ON.
However, covered-state velocity revision RMS increases slightly, from 0.04011
to 0.04212 m/s OFF and from 0.07676 to 0.08616 m/s ON. Angular-rate revision
RMS also increases there. Thus WNOA does **not** improve every derivative
quantity, and endpoint coverage alone does not guarantee full observability.

Final WNOA versus WNOJ support-position differences are 1.202 mm RMS / 6.096 mm
maximum OFF and 1.760 mm RMS / 5.502 mm maximum ON. Their final orientation
differences are 0.000566 rad RMS OFF and 0.000856 rad RMS ON.

| Case | As-played full support path, m | Final full support path, m | Final dense interpolated path, m |
| --- | ---: | ---: | ---: |
| OFF WNOJ | 22.995337 | 14.069385 | 14.128951 |
| OFF WNOA | 15.468111 | 14.065782 | 14.122525 |
| ON WNOJ | 36.808566 | 14.576443 | 14.894144 |
| ON WNOA | 17.715229 | 14.524339 | 14.830385 |

These are paths of the receiver state origin. The excess as-played distance
reflects temporal inconsistency of successive current-pose publications; it
must not be presented as additional physical travel. The plot below separates
published speed from same-time position revision.

![Published speed and same-time correction comparison](/home/yeranis/results/gnssfgo_eth1_roundoff_40s_20261006/prediction_comparison.png)

## Native and fused accuracy on unchanged evaluation masks

The existing GrandTour evaluators are used without changing alignment or
scoring. All primary rows share 364 timestamps from
`1727775011.583475113` through `1727775047.883475065 s`.
The timestamp SHA-256 is
`042b30bea327afd6c6a0ec1cf44ae86518b10237ec16fe98aba50eb9a6f79d16`.

ATE uses CPT7-to-prism lever-arm conversion and no-scale SE(3) Kabsch alignment
against AP20 prism positions. **Prism coverage is only 113 matches over 11.2 s,
not the entire 40-second experiment.** Its matched reference path is 3.605208 m
for every primary row. One-second RPE uses 353 identical IE-TC relative pose
reference pairs over the longer common section; no global alignment is needed
for that relative metric.

| Mode | Trajectory | Prism ATE RMSE, m | 1 s t-RPE RMSE, m | 1 s r-RPE RMSE, deg |
| --- | --- | ---: | ---: | ---: |
| OFF | Native GLIM | 0.055799 | 0.040203 | 0.549638 |
| OFF | Native Kimera ROS odometry | 0.029156 | 0.022980 | 0.633515 |
| OFF | Fused WNOJ as-played | 0.068066 | 0.070220 | 3.094011 |
| OFF | Fused WNOJ final | 0.044937 | 0.023570 | 0.469669 |
| OFF | Fused WNOA as-played | 0.051067 | 0.038430 | 1.457819 |
| OFF | Fused WNOA final | 0.044923 | 0.023611 | 0.474092 |
| ON | Native GLIM | 0.043442 | 0.040966 | 0.573803 |
| ON | Native Kimera ROS odometry | 0.055881 | 0.034982 | 0.693908 |
| ON | Fused WNOJ as-played | 0.079488 | 0.127298 | 3.618377 |
| ON | Fused WNOJ final | 0.046024 | 0.028948 | 0.551567 |
| ON | Fused WNOA as-played | 0.058563 | 0.062818 | 1.904373 |
| ON | Fused WNOA final | 0.045906 | 0.028814 | 0.552936 |

Native GLIM is the retained **end-of-source-run** `odom_imu` export from the
original 120 s run, cropped for evaluation. It is not a native GLIM snapshot
reconstructed at the 40 s arrival cutoff. Against retained published GLIM
odometry, its first-window position difference is 10.45 mm RMS OFF and
15.09 mm RMS ON. Native Kimera is the retained ROS odometry, not a gnssFGO
Kimera-only control. These native-source provenance distinctions matter when
comparing source and receiver preservation. They do not affect the matched
WNOA/WNOJ comparison, which has identical causal information sets.

All primary rows use common resampling chosen by time coverage only. Auxiliary
native scores at original sampling are retained in `metrics.json` but are not
mixed into the common-mask table. The native Kimera ON trajectory is worse
than OFF in this short section; historical longer-window CBS gains must not
be assumed to hold in every prefix.

For completeness, the evaluator's path fields are below. These are paths of
the **lever-arm-converted prism point**, not the receiver-origin paths above.
The common-grid path covers 36.3 s; the GT-matched path covers 11.2 s.

| Trajectory | Common-grid prism path, m | GT-matched estimate path, m |
| --- | ---: | ---: |
| OFF native GLIM | 14.179776 | 4.036056 |
| OFF native Kimera | 13.424903 | 3.479227 |
| OFF WNOJ as-played | 23.632460 | 8.984141 |
| OFF WNOJ final | 13.924244 | 3.794893 |
| OFF WNOA as-played | 15.404627 | 4.496890 |
| OFF WNOA final | 13.921603 | 3.798048 |
| ON native GLIM | 14.230357 | 3.844802 |
| ON native Kimera | 13.715011 | 3.667108 |
| ON WNOJ as-played | 37.079002 | 8.437119 |
| ON WNOJ final | 14.338595 | 3.849902 |
| ON WNOA as-played | 17.854376 | 4.688062 |
| ON WNOA final | 14.293933 | 3.837825 |

## Source fit and remaining statistical limitations

These are residuals of the final optimized source factors, not errors against
GT and not independent innovation NIS measurements. Translation is the rho
coordinate of the existing SE(3) residual.

| Case | GLIM rho RMS, m | Kimera rho RMS, m | GLIM rotation RMS, rad | Kimera rotation RMS, rad | GLIM mean squared whitened residual | Kimera mean squared whitened residual |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| OFF WNOJ | 0.007095 | 0.006715 | 0.001332 | 0.001609 | 295.714 | 598.314 |
| OFF WNOA | 0.006941 | 0.006556 | 0.001115 | 0.001399 | 214.067 | 453.505 |
| ON WNOJ | 0.009705 | 0.005804 | 0.001669 | 0.000793 | 470.574 | 434.218 |
| ON WNOA | 0.009489 | 0.005758 | 0.001368 | 0.000680 | 351.718 | 347.677 |

The high normalized source residuals remain. Numerical convergence and smaller
prediction corrections do not validate source uncertainty calibration or
source independence. Removing central IMU factors also does not remove the
shared IMU information already inside GLIM and Kimera, or correlations induced
by classic CBS exchange. No covariance or correlation remedy was attempted.

## Numerical validation and preservation

Nine new focused tests cover ordinary non-increasing acceptance, the two-ULP
case, convergence requirements, meaningful increases across objective scales,
invalid/nonfinite inputs, stationary no-op, actual nonlinear iSAM2 convergence,
budget exhaustion, truly worsening-step rollback and candidate-exception
rollback. They run alongside the six unchanged bounded-refinement tests.
All 15 pass in Release and all 15 pass with ASan/UBSan. The external GTSAM and
gnssFGO runtime libraries are prebuilt and unsanitized; this is a focused
instrumented test, not a claim that the full dependency stack was sanitized.
Leak detection was disabled; address/undefined checks halt on error.

| Case | Passed publication checks | Largest publication gradient | Passed batch checkpoints | Largest checkpoint position difference, m |
| --- | ---: | ---: | ---: | ---: |
| OFF WNOJ | 395 | 9.94575e-6 | 6 | 2.26649e-9 |
| OFF WNOA | 395 | 9.98625e-6 | 6 | 4.13183e-9 |
| ON WNOJ | 395 | 9.85086e-6 | 6 | 2.19339e-9 |
| ON WNOA | 395 | 9.87891e-6 | 6 | 4.78347e-9 |

Checkpoints are supports 74, 194, 294, 344, 394 and the final information-set
snapshot. All state timestamps match. Batch checks require both gradients
at most `1e-5`, relative objective excess at most `1e-6`, maximum position
and orientation differences at most `1e-4 m/rad`, and velocity/angular-rate
differences at most `1e-3` in their units. Restart and iteration-budget checks
also pass. The largest observed orientation difference is `1.44351e-8 rad`,
velocity difference `5.02623e-8 m/s`, and angular-rate difference
`5.69254e-7 rad/s`. This is local agreement from the incremental solution's
initialization, not a search for every possible optimum.

Preservation checks in `verification.json` establish:

- All four first-eight-second causal prefixes, 74 full-state snapshots and
  source ledgers per case are byte-identical to their earlier references.
- Complete OFF/WNOJ outputs are byte-identical to the old completed 40 s run.
- OFF/WNOA output before support 344 is byte-identical to the old stopped run.
- Every case's repeat has byte-identical causal/final trajectories, states,
  acceleration exports, residuals, covariance exports, clocks and source ledgers.
  Refinement-step records are byte-identical; convergence records agree after
  excluding wall-clock duration.
- All 3,364 newly protected file hashes and 21,026 inherited protected hashes
  match, including the old artifacts and frozen input/runtime files.
- All 12 primary evaluation rows have identical prism masks and RPE pairs,
  matching the previously frozen masks.
- No central IMU/bias/gravity path or source model covariance was enabled.
- `git diff --check` passes.

## Computational cost

The primary runs include full publication snapshots and independent batch
references. Repeats disable those audits but keep strict publication
refinement. Times are sequential wall-clock measurements on this machine,
not co-running live throughput tests.

| Case | Primary time, s | Repeat time, s | Primary peak child RSS, KiB | Repeat peak child RSS, KiB |
| --- | ---: | ---: | ---: | ---: |
| OFF WNOJ | 66.48 | 63.73 | 403452 | 383044 |
| OFF WNOA | 28.04 | 25.63 | 210864 | 142108 |
| ON WNOJ | 85.75 | 81.15 | 427092 | 332432 |
| ON WNOA | 31.49 | 28.93 | 205256 | 146928 |

WNOA has fewer state variables and lower cost here. Neither these times nor
the accuracy table establishes live real-time capacity, particularly when
GLIM and Kimera also consume CPU/GPU resources.

## Reproduction and implementation boundaries

Commands below use the existing artifact root and therefore document this
execution, rather than authorize overwriting it. Preparation/run/scoring
scripts refuse existing outputs; a fresh reproduction needs a new isolated
root and corresponding mount paths.

```bash
python3 /home/yeranis/results/gnssfgo_eth1_roundoff_40s_20261006/prepare.py
bash /home/yeranis/results/gnssfgo_eth1_roundoff_40s_20261006/run.sh unit wnoj
bash /home/yeranis/results/gnssfgo_eth1_roundoff_40s_20261006/run.sh sanitizer wnoj
bash /home/yeranis/results/gnssfgo_eth1_roundoff_40s_20261006/run.sh build wnoj
bash /home/yeranis/results/gnssfgo_eth1_roundoff_40s_20261006/run.sh build wnoa
bash /home/yeranis/results/gnssfgo_eth1_roundoff_40s_20261006/run.sh primary wnoj off
bash /home/yeranis/results/gnssfgo_eth1_roundoff_40s_20261006/run.sh primary wnoa off
bash /home/yeranis/results/gnssfgo_eth1_roundoff_40s_20261006/run.sh primary wnoj on
bash /home/yeranis/results/gnssfgo_eth1_roundoff_40s_20261006/run.sh primary wnoa on
```

Each primary case was checked before proceeding with
`/home/yeranis/.local/share/pipx/venvs/evo/bin/python` and
`/home/yeranis/results/gnssfgo_eth1_roundoff_40s_20261006/check_case.py MODEL MODE`.
Analysis, repeats and evaluation:

```bash
/home/yeranis/.local/share/pipx/venvs/evo/bin/python /home/yeranis/results/gnssfgo_eth1_roundoff_40s_20261006/analyze.py
bash /home/yeranis/results/gnssfgo_eth1_roundoff_40s_20261006/run.sh repeat wnoj off
bash /home/yeranis/results/gnssfgo_eth1_roundoff_40s_20261006/run.sh repeat wnoa off
bash /home/yeranis/results/gnssfgo_eth1_roundoff_40s_20261006/run.sh repeat wnoj on
bash /home/yeranis/results/gnssfgo_eth1_roundoff_40s_20261006/run.sh repeat wnoa on
bash /home/yeranis/results/gnssfgo_eth1_roundoff_40s_20261006/score.sh
/home/yeranis/.local/share/pipx/venvs/evo/bin/python /home/yeranis/results/gnssfgo_eth1_roundoff_40s_20261006/verify.py
```

`run.py` records exact compiler flags, executable arguments and environments in
`command_*.json`; `evaluation_commands.json` records exact scoring calls.
The receiver uses `WNOJ_INCREMENTAL_PROFILE=nominal`,
`WNOJ_STRICT_PUBLICATION=1`, `OMP_NUM_THREADS=1`, `--timeline record`,
`--solver isam2`, `--support-policy uniform_imu`,
`--support-period-ns 100000000`, `--receiver-startup-samples 59` and
`--receiver-model source_wnoj` or `source_wnoa`. These environment-variable
names are inherited by the isolated WNOA build too.

Build/replay container image:
`sha256:6a664509d183e504230b22edfd747dfe1ca5bca8ae6fc34c76154db8ffd1fc8d`.
Existing scoring image:
`sha256:7ad3854c699fb9a292aa5d42f8f890dfbd5dccb16633e043a93d135221418aaa`.
Both run network-disabled with read-only roots and read-only source inputs;
only the new artifact directory and temporary filesystem are writable.
Build logs contain the inherited abstract GP-interpolator missing-return
warnings; no new build failure remains.

| Executable | SHA-256 |
| --- | --- |
| WNOJ `build/wnoj/replay40` | `e02045a32d149ee378498e0b0dbf95ab7dd9825a9d074c419a7ae16be0502c7b` |
| WNOA `build/wnoa/replay40` | `aea13b62cfd563a2cebf8cb52dcdb6fee5d0128d24a54d3620221fb4dd4f821a` |

Source-tree additions are
`online_fgo/include/offline_process/PublicationRoundoffAcceptance.h` and
`online_fgo/test/test_publication_roundoff.cpp`. Experimental integrations,
logging diffs and test-build commands remain under the new artifact root.
The new test is built explicitly by the audit script; no production CMake
target or default receiver configuration was changed in this task.

Starting and final repository HEAD:
`a842e7ef72e7a40f0cdfec826f089c66af364ae0`.
No commit was created. The repository already contained unrelated modified
and untracked work; it is **not a clean worktree**. Those files were preserved.

## Conclusion

The numerical acceptance failure is fixed and the frozen four-case comparison
is complete. WNOA is a valid, less erratic no-central-IMU prediction model on
this 40 s ETH1 journal, with deterministic output and tight local batch
agreement. Final source-constrained trajectories and their accuracy are almost
unchanged relative to WNOJ. CBS-ON still scores worse than CBS-OFF in the fused
results for this section, and source statistical consistency remains unresolved.
There is no basis here to claim that the full fusion problem is solved or to
change production defaults automatically. Stop after this comparison.
