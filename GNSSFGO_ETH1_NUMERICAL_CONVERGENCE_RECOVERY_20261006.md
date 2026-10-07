# ETH1 source-only numerical convergence recovery

This report continues `GNSSFGO_ETH1_STABLE_COST_REPLAY_VALIDATION_20261006.md`.
The work concerns the experimental no-central-IMU receiver. It does not change
the production Combined-IMU baseline or native GLIM/Kimera estimation.

## Final outcome — complete frozen-window numerical validation passes

All four cases complete the retained approximately 120-second ETH1 window,
including its last support at 119.78 s. Each passes all 1193 publication checks
and a repeated execution with **byte-identical scientific outputs** (both
as-played and final states/trajectories, covariance exports, source accounting,
support times and refinement decisions). No run is being left active.

| Case | Valid publications | Maximum normalized gradient | Maximum refinement passes | Copied-graph checks | Runtime, s |
| --- | ---: | ---: | ---: | ---: | ---: |
| WNOA OFF | 1193/1193 | 9.98625e-6 | 2 | 29 | 711.38 |
| WNOA ON | 1193/1193 | 9.93863e-6 | 2 | 34 | 751.94 |
| WNOJ OFF, exact acceleration derivatives | 1193/1193 | 9.94551e-6 | 3 | 14 | 759.64 |
| WNOJ ON, exact acceleration derivatives | 1193/1193 | 9.98729e-6 | 2 | 14 | 812.96 |

The same 1e-5 gradient limit and 20-pass cap apply throughout. At the old WNOJ
ON failure, support 1174 / 117.98 s, the corrected causal replay converges in
two ordinary passes to gradient **3.0457812901e-7**, without private polishing.
No final accepted case needs multi-pass private polishing. Positive stable-cost
changes admitted by stationary consensus remain explicitly logged; the largest
relative change among final cases is 3.535e-15, not a claimed strict decrease.

All 91 selected copied-graph checks pass: both the receiver and independently
run LM reference satisfy the gradient criterion and the pre-existing state
agreement limits. Across these checks, maximum position discrepancy is
7.410e-9 m; maximum WNOJ acceleration discrepancy is 7.466e-6 in the existing
six-component tangent units. Full numerical details are retained per snapshot.

The WNOA first-40-second prefixes remain byte-identical to their frozen baseline.
The corrected WNOJ prefixes are **numerically equivalent, not byte-identical**:

| Maximum over 394 complete prefix snapshots | WNOJ OFF | WNOJ ON |
| --- | ---: | ---: |
| Position, m | 1.67467e-8 | 5.01986e-8 |
| Orientation, rad | 5.43008e-8 | 6.03833e-9 |
| Velocity, m/s | 8.52743e-8 | 2.36472e-7 |
| Angular rate, rad/s | 6.34948e-7 | 1.07854e-7 |
| Six-component acceleration | 3.73618e-6 | 3.04297e-6 |

Prefix revision ledgers and support clocks are unchanged. Every final case uses
1770 unique source IDs: 1189 GLIM and 581 Kimera. OFF applies 68916 replacements,
ON 68953, with zero pending, stale or duplicate versions at end. The same 25
pre-initialization versions lack endpoint support and are excluded consistently;
they are not newly gated measurements. No central IMU factor/bias/gravity
initialization is introduced.

Thirty focused tests pass in Release and ASan/UBSan. The final integrity check
covers **55,799 protected files, with zero mismatches**. This includes the frozen
inputs and prior artifacts. The final combined summary is
`/home/yeranis/results/gnssfgo_eth1_exact_accel_20261007/final_statistics.json`;
the per-case `validation_*.json`, `analysis.json` and compiler/command logs provide
the checks behind this table. WNOA artifacts remain in the local-prediction
trial root; corrected WNOJ artifacts are in the exact-acceleration root.

This resolves the observed **numerical stopping problem**, not all estimation
limitations. It is offline causal replay, not a live-throughput certificate or
evidence that CBS-ON improves accuracy. For transparency, as-played path lengths
are WNOA OFF/ON 75.485/79.401 m and WNOJ OFF/ON 91.872/135.290 m; WNOJ ON still
has sizeable causal corrections (maximum one-publication same-time position
revision 0.483 m). Numerical convergence does not make those trajectories
equally accurate. No GT-based tuning or new accuracy experiment was performed.
Wall times include detailed logging and concurrent independent replays, so they
must not be used as isolated real-time benchmarks.

## Problem and correction

The previous replay stopped because its publication guard required a strict
cost decrease even at nearly stationary solutions. At support 616 (62.18 s),
fresh QR solves of the identical graph differed by approximately 1e-13 in their
local steps, enough to change the sign of a roughly 1e-11 cost difference.
Simply substituting a Values result into iSAM2 would leave its internal tree
and linearization inconsistent, so it is not used.

The first trial safely rebuilt iSAM2's complete QR tree at the current estimate.
It preserved every factor pointer, null slot, key and downstream revision index.
This passed support 616 with stable cost change -2.768891445281217e-11 and
normalized gradient 6.252650407086217e-8. However it stopped at support 808
(81.38 s): incremental and rebuilt candidates had gradients 1.0102063e-7 and
6.9873289e-8, but evaluated stable cost changes +1.5880183e-10 and +2.6985595e-10
on a total cost of 310831.8082. Rebuilding alone is therefore not the solution.
This failed trial is retained, not overwritten.

The recovery policy distinguishes **strict descent** from **stationarity
within numerical tolerances**. Ordinary accepted steps and the stable residual
descent check remain unchanged. After rejection, enter one transactional QR
recovery. Accept its fresh rebuild if it passes the existing strict-descent check,
or the following separately logged stationary-consensus check:

1. Both independently factored QR candidates have normalized gradient <=1e-5.
2. Both predict a resolved negative objective change.
3. Both actual stable cost changes and predicted changes have absolute magnitude
   <=1e-11 max(1,E). This is the existing copied LM reference's relative cost
   tolerance, not a covariance setting or a GT-selected parameter.
4. Every local state coordinate agrees between the two candidates within
   sqrt(double epsilon), using |da-db|/(1+max(|da|,|db|)). Coordinates retain the
   existing manifold/SI conventions; this is a numerical comparison, not a
   posterior covariance or physical-error metric.
5. State steps, objectives and diagnostics are finite; factor slots/pointers
   and the full state key set are unchanged.

Here the unchanged gradient metric is
`max |J^T r| / sqrt(diag(J^T J))`, evaluated at the candidate estimate using a
fresh factor linearization, not the cached iSAM2 linearization point.

The fresh solver is swapped in only after acceptance. No factors are added,
dropped or reweighted by the recovery. Null factor slots survive bulk loading
by temporarily disabling slot reuse; the original reuse policy is then restored.
Only full-history Gauss-Newton QR, relinearizeSkip=1 and no marginalized fixed
variables are supported. Rejection or exceptions discard private candidates;
previously accepted ordinary refinement steps are retained, but no invalid
publication is emitted.

### Late WNOJ failure and bounded private polishing

The stationary-consensus trial completes OFF/ON WNOA and OFF WNOJ. ON WNOJ
stops at support 1155 (116.08 s), reproducibly in both runs. After its second
ordinary refinement, the retained gradient is 1.15325e-5. The next incremental
and fresh-QR candidates have gradients 1.82323e-5 and 1.59259e-5. Neither passes
the unchanged 1e-5 threshold, even though their objective changes are tiny.
The copied LM reference also stops at that nonstationary point; its zero
iterations are **not** accepted as proof of convergence.

The recovery implementation allows private polishing for the remaining
passes of the existing 20-pass budget. Each private pass compares an incremental
full solve and a freshly factored QR solve from the same seed. Intermediate
nonstationary states are not committed or published. Both actual and predicted
cost changes must stay within the existing 1e-11 relative-cost neighbourhood,
with resolved predicted descent. All changes are measured against the original
retained anchor, so repeated small increases cannot accumulate unchecked.
Predicted descent in each later pass must use the **current private seed's**
fresh linearization, not the retained anchor's old linear model. The first
bounded-polishing implementation got this reference wrong: at support 1155,
pass 4 had gradient 1.01288469e-5, cumulative actual change +1.4414833e-9 and
old-anchor predicted change +2.1264726e-10. It rejected prematurely again.
That failed implementation/run is retained for transparency. The corrected
helper `CumulativeCostWithLocalPrediction` combines cumulative actual cost
change against the fixed anchor with predicted change from the current seed.
The journal explicitly labels these rows `local_prediction=1`.

Commit still requires the original strict-descent or stationary-consensus
criteria above. There is no gradient-threshold relaxation. A recovery pass can
perform two QR solves: the bound is 20 passes, not 20 individual solves.

Single-thread TBB execution was also tested. It reduces resource use, but does
not guarantee byte-identical results in all full cases; it does not alone cure
the late WNOJ failure. Numerical reproducibility and byte reproducibility are
reported separately. The precise source of the remaining last-bit arithmetic
variation is not established by this investigation.

This is explicitly **not a certificate of exact objective descent**, a rigorous
forward-error bound for nonlinear SE(3) residuals, a global-optimality proof or
a proof of statistical consistency. Positive evaluated changes admitted under
stationary consensus are retained in the logs, not labelled as strict descent.
The reason for this numerical termination change is stated above; the new
protocol was written before the new replays, without consulting GT accuracy.

## Frozen scientific settings

### Subsequent derivative diagnosis and isolated correction

The late WNOJ failure required a second, concrete numerical correction, not a
larger iteration budget. At saved support 1174, long-double accumulation of
`J^T r` agrees with the usual double accumulation (8.75571893917e-6 versus
8.75571893925e-6 after CSV reconstruction). Summation precision is not the
observed cause. A five-point derivative check of the unchanged residuals at
steps 1e-3, 3e-4 and 1e-4 gives maximum normalized gradients 8.71903e-6,
8.80604e-6 and 8.74616e-6. The maximum is in acceleration coordinate a1138[4];
pose, velocity and angular-rate maxima are approximately 1.4e-8, 3.1e-7 and
5.3e-8. Stock numerical acceleration Jacobians change the late QR iterates
enough to move their computed gradient across the publication threshold.

Manual stock LM iterations do not solve this: its first step reaches gradient
1.04632e-5, then the remaining 19 calls return the same estimate. A subsequent
stock-derivative QR refinement takes 18 private passes. This is recorded in
`local_prediction.../logs/saved1174.log`; three malformed newline literals in
that exploratory diagnostic print the integer 23662 as a separator. The later
independent-gradient diagnostic corrects that logging issue. Neither diagnostic
changes any receiver state or source data.

The isolated helper `ExactWnojAcceleration.h` changes **only acceleration
Jacobian evaluation**, not the WNOJ equations or measurement residual. In the
motion factor, for interval dt and right-Jacobian inverse J at the relative pose:

```text
dr/da_i = [-0.5 dt^2 I; -dt I; -I]
dr/da_j = [0; 0; J]
```

For interpolation, retain the original library's Lambda, Psi and local xi.
If H is the right-local derivative of `X_i Exp(xi)` with respect to xi:

```text
dT(t)/da_i = H Lambda(0:6,12:18)
dT(t)/da_j = H Psi(0:6,12:18) J
```

GTSAM's existing Expmap/compose derivatives supply H. No numerical step size is
tuned. The stock residual methods and all non-acceleration Jacobian blocks
are still called unchanged. This does not enable the library's complete
alternative analytic-Jacobian branch. In particular, it does not assume every
block of that branch is interchangeable with the selected numerical path.

`ExactAccelerationSourceWnoj.h` provides an explicit opt-in endpoint factory
using the same `FullWnojEndpoint` and relative-factor classes. The standard
`SourceWnojModel.h`, stock GP headers, production receiver and defaults are
unchanged. The experimental replay selects this factory and the corresponding
derived motion factor. An initial copied-header prototype is retained as
`SourceWnojModel.prototype_not_used.txt`; it is no longer on the include path.

On the identical reconstructed support-1174 graph, one fresh QR solve with
these blocks reaches gradient **4.6088505558e-7** without further refinement.
Repeated saved-graph diagnostics give identical output. The starting residual
costs are identical to the stock reconstruction, with the same 2920 factors,
4700 variable keys, covariance payloads and endpoint timestamps. The fixed
1e-5 gradient limit, 1e-11 relative-cost tolerance and 20-pass limit remain.

Thirty focused Release tests and the same 30 ASan/UBSan tests pass after the
explicit factory is installed. New tests cover both motion-Jacobian modes,
interpolation endpoints and five interior fractions, independently differentiated
acceleration blocks at three step sizes, bit-identical residual/pose values,
unchanged non-acceleration blocks/noise, derived-factor cloning, shared-endpoint
relative factors and invalid interval rejection. Full replay/repeat artifacts
are in `/home/yeranis/results/gnssfgo_eth1_exact_accel_20261007`.

All cases use the retained ETH1 OFF/ON journal from
`/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live120_zero_20260930`.
This is offline **causal replay**, not a new live source run. Every source mean,
full covariance, stable ID, revision and availability timestamp remains frozen.
Classic source CBS exchange is not replaced by experimental live_revision.

The independent support grid is 100 ms, driven by raw IMU timestamps only, with
the same 59-sample clock offset. There are no raw IMU measurement factors, bias
states, gravity initialization, GNSS factors, robust losses, gating or scaling.
Stock GP interpolation values/equations, priors and full-history iSAM2 QR are
unchanged. The final WNOJ experiment uses the isolated exact acceleration
derivatives described above; WNOA does not need that addition.

WNOA uses Pose3 X, world velocity V and body angular rate W, with Qc diagonal
`[.00375,.00375,.00375,.015,.015,.015]`. WNOJ additionally has tangent acceleration,
with Qc `[2.5,2.5,2.5,10,10,10]`. Initial priors remain identity X (sigma1e-5),
zero V (sigma10), zero W (sigma3), and WNOJ zero acceleration (sigma10).

The 120 s requested window contains 119.850027 s of recorded IMU timestamps;
its final grid time is 119.78 s after journal start. Thus successful completion
requires 1193 support states/publication checks, not a fabricated exact 120 s
sample. The input hashes are:

```text
OFF 44d89960cc69941ec13e2e6299120ef3eb8398e16cffa2149d521af8be948590
ON  8ba7ea3a306de88c763635380ddc6f359cd5219f688a46d253779cfe271715fe
```

## Validation results

Twenty-seven focused Release tests and the same 27 ASan/UBSan tests pass. They
include preservation of null slots and marginal covariance, subsequent stable-ID
replacement, rejected solver profiles, invalid-slot rejection, stable Gaussian
cost evaluation, stationary consensus, genuine worsening/exception rollback and
the unchanged legacy bounded-refinement tests. Two new cyclic nonlinear tests
verify that private polishing cannot publish a nonstationary result or overrun
its budget, and leaves the retained solver unchanged on that rejected recovery.
An additional cubic-factor test exercises successful multi-pass private
polishing: only the final converged solver is committed, its factor pointer is
preserved, and its final state agrees with the known root. A final unit test
ensures that local predicted descent cannot hide a cumulative objective change
outside the frozen bound. The final expanded test results are in
`command_unit.json` and `command_sanitizer.json` in the local-prediction root.
External frozen shared libraries
are not sanitizer-instrumented; this is not the complete ROS repository suite.

In the stationary-consensus trial, all four 40 s replays complete and pass
state/ledger/copy-reference validation.
OFF WNOA, ON WNOA and OFF WNOJ have byte-identical scientific outputs to the
previous frozen short run. ON WNOJ is **not byte-identical** to the old binary:
the first difference occurs at support 13, before any fallback, and no rebuild
or consensus fallback occurs anywhere in that short case. Maximum differences
over all 394 full-state snapshots are:

| State | Maximum difference |
| --- | ---: |
| Position | 5.4687011e-8 m |
| Orientation | 6.7013106e-9 rad |
| Velocity | 2.9214693e-7 m/s |
| Angular rate | 1.1978439e-7 rad/s |
| Six-component acceleration | 5.0157446e-6 in mixed tangent units |

All ON WNOJ source/model covariance exports, event accounting, support times and
revision ledgers are byte-identical. The checker initially failed its requested
byte-equality test; numerical comparison then passed the already-existing
copied-reference state tolerances. Those physical-state tolerances were not
relaxed. The precise binary-level arithmetic origin is not localized here;
the evidence excludes use of the new fallback as the first cause. Determinism
is checked separately using repeated executions of the identical new binary.

The local-prediction long-case trial has completed. OFF/ON WNOA and OFF WNOJ
each complete all 1193 publication checks, with byte-identical repeated
scientific outputs. ON WNOJ passes the former support-1155 failure but exhausts
the unchanged 20-pass budget at support 1174 (117.98 s); its retained gradient
is 1.3227769359566751e-5. The repeat fails at the same point. This trial is
therefore **not a complete four-case solution**. A targeted saved-graph
derivative investigation follows; no new source run is involved.

The corrected causal run passes the formerly failing support 1155. Its first
three attempts reproduce the retained failure values exactly; pass 4 retains
gradient 1.01288469e-5 but now correctly observes **local** predicted descent
-3.03419390e-9 (the obsolete anchor prediction had been +2.12647261e-10).
Pass 5 yields two agreeing QR candidates with gradient 7.99728903e-6.
The cumulative actual change is +8.91985895e-10 and current-seed prediction is
-2.50814293e-9. It is accepted as stationary consensus, **not strict descent**.
Only 5 of the existing 20 passes are used; the following support 1156 also
converges normally. These are causal replay measurements, unlike the separate
saved-graph regression below.

### Saved late-failure graph regression

`build/wnoj/SavedGraph.cpp` reconstructs the saved support-1155 active graph from
its ledger and the original journal, using the original factor constructors,
Qc and priors. It is a read-only numerical diagnostic, not a second receiver
or causal replay. All 2872 factor key lists match; all 4624 variable keys are
restored. Maximum per-factor objective difference after CSV pose reconstruction
is 6.50536e-9. Total cost differs by 4.66e-10 on 667883.853704264.

This is not an exact serialized iSAM2 checkpoint: quaternion reconstruction and
refactorization alter last-bit arithmetic. In particular the reconstructed
seed's normalized gradient is 8.65407e-6 rather than the retained 1.15325e-5.
After fresh QR construction the gradient is 1.12381e-5. The corrected private
refinement then converges in two passes to 7.28922e-6, with cumulative stable
cost change -1.65850e-9 and current-seed predicted change -2.65027e-9.
The final accepted pass explicitly uses `local_prediction=1`. This demonstrates
the multi-pass path on the retained mathematical graph but does not substitute
for the end-to-end causal replay. No diagnostic solution feeds back into it.
The first two diagnostic executions are byte-identical. A third launcher check
also converges in two passes, with gradient 9.06563e-6 and stable final change
-1.70935e-9, but follows different last-bit acceptance decisions. Thus this
diagnostic demonstrates repeatable convergence, not universal bitwise identity.
All three reconstructed input graph costs and seed gradients are identical.

### Launcher bookkeeping issue (not hidden)

During the final trial, a read-only mount for the saved-graph diagnostic was
added to the same shell script whose Bash processes were waiting on the full
replay containers. This was an operational mistake: the four WNOA wrappers
resumed at their old file offset and interpreted a trailing hash fragment as
a command, exiting 127 **after** the replay children had completed with code 0.
Their `command_*.json`, complete outputs and independent validation checks
distinguish successful estimator execution from this launcher error.
The original wrapper was restored byte-for-byte before WNOJ completion, and
the diagnostic mount moved to a separate `run_saved.sh`. No already-running
container, replay binary, input or output was changed. `WRAPPER_NOTE.md`
records this explicitly; future launch scripts must remain immutable while
running. The error is not counted as an estimator divergence or silently
reported as an all-clean shell launch.

## Reproduction and provenance

Failed strict-descent trial:
`/home/yeranis/results/gnssfgo_eth1_qr_recovery_20261006`.
Stationary-consensus trial artifacts:
`/home/yeranis/results/gnssfgo_eth1_stationary_consensus_20261006`.
Single-thread trial:
`/home/yeranis/results/gnssfgo_eth1_serial_qr_20261007`.
Failed stale-prediction bounded-polishing trial:
`/home/yeranis/results/gnssfgo_eth1_bounded_polish_20261007`.
Final corrected local-prediction implementation and validation:
`/home/yeranis/results/gnssfgo_eth1_local_prediction_20261007`.
Each has its own pre-run protocol, mechanical source diffs, immutable prior
artifact hashes, build/run command logs and per-publication diagnostics.

The existing frozen runtime is used in Docker image
`sha256:6a664509d183e504230b22edfd747dfe1ca5bca8ae6fc34c76154db8ffd1fc8d`,
network disabled, container filesystem read-only, runtime/inputs/repo mounted
read-only. Only the new artifact directory is writable.

The stock GTSAM runtime's `ISAM2-impl.h:386` explicitly sets the relinearization
threshold to zero under `forceFullSolve`; refinement therefore does not rely
on merely checking the old 0.01 threshold again. The original nominal update
settings are unchanged outside this publication-refinement policy.

No commit has been made. HEAD remains
`a842e7ef72e7a40f0cdfec826f089c66af364ae0`. The pre-existing dirty worktree is
preserved, not described as clean. Production entry points and defaults are
unchanged; this remains an explicitly selected experimental numerical policy.

The reusable implementation is in
`online_fgo/include/offline_process/FullHistoryPublicationRefinement.h`,
`TransactionalISAM2Rebuild.h` and `StationaryQRConsensus.h`. The full-policy
header is mirrored into both isolated compiled wrappers; final hashes are
recorded with the completed validation artifacts.
The focused tests were rerun after switching their include to this repository
header. There is no hidden dependency on an unrecorded external wrapper.

### Commands actually used

The stationary-consensus/local-prediction trial root is
`/home/yeranis/results/gnssfgo_eth1_local_prediction_20261007`. The wrapper's exact
container invocation and every expanded compiler/replay command are retained in
`run.sh` and `command_*.json`. Builds use `-O3 -DNDEBUG`; focused sanitizer builds
use `-fsanitize=address,undefined`. No ROS launch or source estimator is started.

```bash
bash /home/yeranis/results/gnssfgo_eth1_local_prediction_20261007/run.sh unit wnoj
bash /home/yeranis/results/gnssfgo_eth1_local_prediction_20261007/run.sh sanitizer wnoj
bash /home/yeranis/results/gnssfgo_eth1_local_prediction_20261007/run.sh stable_build wnoa
bash /home/yeranis/results/gnssfgo_eth1_local_prediction_20261007/run.sh stable_build wnoj
bash /home/yeranis/results/gnssfgo_eth1_local_prediction_20261007/run.sh saved_build wnoj
bash /home/yeranis/results/gnssfgo_eth1_local_prediction_20261007/run.sh saved wnoj
# After separating the diagnostic mount from the immutable replay launcher:
bash /home/yeranis/results/gnssfgo_eth1_local_prediction_20261007/run_saved.sh saved wnoj
for model in wnoa wnoj; do
  for mode in off on; do
    bash /home/yeranis/results/gnssfgo_eth1_local_prediction_20261007/run.sh primary "$model" "$mode"
    bash /home/yeranis/results/gnssfgo_eth1_local_prediction_20261007/run.sh repeat "$model" "$mode"
  done
done
```

Each replay runs with `WNOJ_STRICT_PUBLICATION=1`, `AUDIT_RECORD_ALL=1`,
`WNOJ_INCREMENTAL_PROFILE=nominal`, the unchanged independent 100 ms support
grid and record-time causal scheduling. TBB and OpenMP concurrency are limited
to one in each final replay process. Multiple independent cases can run in
parallel; their wall times are not isolated throughput benchmarks.

These commands document the completed directories' provenance, not permission
to overwrite them. Reproduction must use a fresh artifact root and fresh
output directories. `validate.py` and `summarize.py` use the existing numeric
environment `/home/yeranis/.local/share/pipx/venvs/evo/bin/python`.

The isolated exact-acceleration WNOJ follow-up uses
`/home/yeranis/results/gnssfgo_eth1_exact_accel_20261007/run.sh`, with the same
read-only runtime/input mounts and explicit recorded compiler flags. Its final
commands are `stable_build2 wnoj`, `saved_build wnoj`, `saved wnoj`,
`unit2 wnoj`, `sanitizer2 wnoj`, and `primary/repeat wnoj off/on`.
The first prototype binaries are retained under `*_prototype`, not executed by
the final replay commands. Exact helper/test sources and hashes are retained in
`source_snapshot/` and `source_provenance.json`. No running launcher was edited
in this follow-up. The frozen journals are reused; GLIM/Kimera are not started.
