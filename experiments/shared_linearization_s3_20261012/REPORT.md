# Native Gaussian shadow lifecycle integration

**Continuation:** The subsequently authorized first40s ETH1 recorded-equation
gate passed. See the [real-data S3 report](../shared_linearization_s3_capture_20261012/REPORT.md).
The report below preserves the earlier implementation/synthetic-gate boundary.
Old LOCAL remains authoritative and S4 has not started.

## Result and current boundary

The S3 **implementation and synthetic integration gate passes**. A private,
default-off observer now connects the S2 native Gaussian backend to the actual
`IncrementalFixedLagSmootherExtWithFallback` lifecycle and GLIM's native admission
boundary. Existing independent LOCAL covariance remains authoritative. The
observer neither publishes its covariance nor changes the optimization model.

**S3 real-data shadow validation remains pending.** No GrandTour playback, ROS
estimator run, fusion experiment or timing test was launched. The retained B0
source/receiver logs do not contain complete pre-retirement MAIN Gaussian
snapshots; the previous GPU checker captures independent LOCAL, not this MAIN
history. These are synthetic integration results, not a live GPU reuse or
end-to-end speedup claim.

## Implemented connections

The private build mechanically overlays three existing translation units. The
source checkouts, headers/class layouts, installed libraries and frozen profiles
are unchanged. `build.py` records the exact replacements and compiler/link commands.

| Boundary | Observation without changing authoritative behavior |
| --- | --- |
| Bridge before `updateSidecar`, before appending CBS | Positively admit native factor objects, including startup pose/velocity/bias constraints. `GLIM_SHARED_NATIVE_SHADOW=1` is required in this private build. Enabling after initialization fails closed. |
| Outer fallback-wrapper `update` | Establish a thread-local observation scope for that registered owner, including empty updates. Stage identity changes and explicit removals. |
| Inner update immediately after `isam_.update`, before marginalization | Match admission objects to actual returned insertion slots; verify identities; deep-copy native Gaussian equations and MAIN `theta` before their deletion. No nonlinear `error`, `linearize` or estimate query is added here. |
| Inner update after marginalization | Verify surviving slot identities, record actual fixed separator keys and stage S2's separate native elimination/prior. MAIN marginal factors are not admitted. |
| Outer update after successful fallback-state maintenance | Commit the staged candidate history. Stack unwinding aborts/invalidate shadow without changing MAIN recovery. |
| `fallback_smoother`, including estimate-time fallback outside update | Permanently invalidate this owner's shadow. Do not bootstrap from reconstructed MAIN anchors/prior. A later empty update cannot hide this reset. |
| Existing outgoing endpoint-mean query | Optionally calculate and log shadow covariance at those already-requested means. The existing covariance provider, result construction and publication remain untouched. |
| Outer smoother destruction | Remove the registry entry so a reused object address cannot inherit old provenance. |

Each native ID is monotonic within one owner lifetime and retains ownership of
its nonlinear factor until retirement/removal. Slot and pointer alone are not
identities. Every successful capture receives a new *equation-snapshot version*;
this does not mean MAIN relinearized every factor at every update. A refreshed
cached block replaces its previous version. Identical factor classes can be
native or external; classification is exclusively by positive admission.

The S2 backend and its 25 tests are byte-unchanged. It retains the separately
eliminated native prior, checks frozen separator charts, includes endpoint cross
covariance, transports to current endpoint tangent coordinates and uses the
existing full `schur_relative_between` helper. No second nonlinear optimizer,
fresh native factor evaluation, noise rescaling, loading, new anchor or MAIN
Bayes-tree covariance is used by the candidate. Gaussian elimination and
covariance solves still incur cost.

## Actual smoother tests

Seventeen new lifecycle tests plus all 25 S2 tests pass. The principal fixture
uses the actual fallback wrapper/inner solver, not a replacement mock solver.
It contains native and external relative-pose factors of the same class,
nonzero state corrections, relinearization, extra empty updates and fourteen
retirement steps. It performs 33 captures; its 17 admitted native factors end
as 15 absorbed plus two active contributions. External factors are not admitted.

An independent dense oracle scatters the captured native augmented blocks,
retains its own dense prior and uses LDLT Schur elimination. It does not use the
candidate's elimination or prior query to generate its expected result. The
Gaussian blocks are intentionally shared inputs; S2's independent raw-matrix
tests separately validate construction and covariance algebra.

| Check | Observed result | Predeclared gate |
| --- | ---: | ---: |
| Native augmented information versus dense oracle | max relative error `3.23556e-16` | `1e-9` |
| Endpoint joint covariance versus dense oracle | max relative error `1.40247e-12` | `1e-8` |
| MAIN `theta`, estimates and cached Gaussian matrices, observer on versus off | Exact equality on identical synthetic inputs | `1e-12` maximum allowed |
| Extra counted native/external relative-factor evaluations caused by observation/query | 0 | 0 |
| Native contribution accounting | 17 = 15 absorbed + 2 active | Exact |
| Numeric diagnostics across Release repeat and sanitizer build | Exact equality | Deterministic |

A separate actual `ImuFactor` fixture carries pose, velocity and bias variables
through ten updates and multiple retirements, with native pose-relative and
bias-between factors. All 30 native contributions remain accounted for; nine
Values remain active and endpoint covariance queries pass. These are synthetic
IMU inputs, not GrandTour or a substitute for the real GPU cache gate.

Other checks cover same-update removal/insertion, slot reuse by a new native ID
or excluded external factor, missing caches, duplicate/missing admission,
observer exceptions, MAIN update failures, fallback during update and outside
update, owner cleanup, default-off behavior, mid-run enable rejection and
independent owner histories. Failure tests intentionally print
`SHARED_NATIVE_SHADOW_FAILURE`; those expected diagnostics are not failed tests.

## Non-regression and private builds

| Check | Result |
| --- | --- |
| Release `release_3`, two executions | 42/42 each |
| ASan + UBSan `asan_2`, leak detection enabled | 42/42, no sanitizer findings |
| Existing E1 source regression suite with private library chain | 18/18 |
| Private bridge `ldd -r` | No missing or undefined symbols |
| Same disabled-mode control executable against original versus private CPU library | 659,210 bytes of MAIN state/chart/cache output identical |
| Protected baseline integrity | All 680 files, 19 S1 pins, six source HEAD/clean-worktree checks and original MAIN CPU library unchanged |

The original-library comparison covers 18 synthetic state additions and their
extra empty updates. Its output SHA-256 is
`28411780ffa43dbaaf3937dfa77d5951ea8a9fc1b448edbd1600a94a9f3f49ba`.
Both enabled-versus-disabled and original-versus-private checks are necessary:
they test different non-regression boundaries.

Focused sanitizers instrument the new observer, S2 backend and modified CPU
smoother translation units. Previously built dependencies and CUDA device code
are not sanitizer-instrumented; this is not whole-workspace/device certification.
No ROS no-data startup was performed; the bridge check is a dynamic-link check.

Private artifacts are under
`/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/shared_linearization_s3_20261012/`.
Final Release CPU library SHA-256:
`dd9008f93357e31e720a0222a3475a6736c68f52cb0fd6759dd2263d7240c209`.
Final Release bridge SHA-256:
`1e10eccd452002ee761d6570821b88f3bc46bbd0382b1d17e64a2d361ddd8bbd`.
The frozen private GTSAM library remains
`369f45866a82c40170ad7cfa83b4bc6b7f8a6f4ae22db34115daee926529bb37`.

`evidence/` contains commands, build identities, test XML/logs, loader maps,
numeric summaries and non-regression results. Full control outputs remain in
the private run directory. Earlier development attempts remain separate: the
initial 38-test gate passed; adding the original-library driver exposed a missing
Pose3 include in that driver. It was corrected before the final complete builds.

## Remaining S3 work and stopping point

Before accepting S3 on real data, prepare the bounded oracle-recording harness
and no-data startup check, then obtain explicit authorization for **one ETH1
first-40-second shadow capture** using the frozen B0 settings. Record enough
native equation/chart/retirement evidence to compare an independent same-chart
oracle through early, middle and late windows. The current private bridge
hook logs candidate covariances; those logs alone are not an independent oracle.
Bound expensive diagnostic covariance queries and record their overhead before
choosing the shadow capture's observation schedule. This affects diagnostics,
not factor admission, update cadence or source publication.

Keep old LOCAL authoritative, all MAIN/source settings unchanged, and stop the
candidate on cache, identity, chart, rank or reset failures. No automatic retry,
fresh GPU fallback, covariance substitution or S4 promotion is permitted.
Shadow execution runs both covariance paths, so its runtime cannot establish
candidate-only speed. A real-data mathematical pass must precede the separately
authorized S4 covariance-publication/timing gate.

This covariance definition removes explicit incoming CBS Gaussian information,
not CBS's indirect influence on MAIN poses, correspondences and native
linearizations. It does not establish calibrated uncertainty, improved accuracy
or independence of the two estimators.

Parent checkpoint: `030bc8cef71e23a9758b28c70b7a4d5e2e5db66e`. The implementation,
evidence and plan update are saved in the commit containing this report on
`engineering/shared-native-gaussian-s3-20261012`. S2 and the E1b.4a recovery tag
remain intact. No push or default promotion was performed.
