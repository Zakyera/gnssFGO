# E1b.4a — observer-free ETH1 first40s timing

**PASS: the LOCAL-update timing gate and existing functional gates passed.**
The measured LOCAL bottleneck is greatly reduced without a new model change.
This is one bounded timing validation, not full-sequence capacity acceptance.

## Scope and provenance

One new capture of ETH1 mission2024-10-01-11-29-55, bag0–40s at0.10x,
with native GLIM, native Kimera and the frozen E2 source-WNOA receiver running
concurrently. No second source capture, rebuild, tuning, GT scoring or retry.
The expensive independent snapshot checker from the preceding correctness gate
was absent. Existing low-overhead E1b timing and bounded source covariance
diagnostics remained enabled to match the retained E1b.3c baseline.

- Run root: `/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_e1b4a_timing40_20261012`.
- Baseline: sibling `gnssfgo_e1b_profile40_mapguard_20261012`.
- Task branch: `engineering/e1b4a-local-update-batching-20261012`.
- Parent: `64be474f7a84d272fbbcddef0b577f81877dbff4`; this task adds only
  protocol/harness/analysis/evidence, not estimator implementation changes.
- Timing harness/report/evidence commit:
  `575bcf3308723039ed98acbc15787df3e1bf3942`; the following documentation
  checkpoint records that identity. No push.
- Candidate: sibling `e1b4a_local_update_batching_20261012/release_2`.
  GLIM wrapper SHA256 `83592aba553adbc2879f27b5f2cf3551826ab497a58201ba0101543934f49dec`;
  GTSAM SHA256 `369f45866a82c40170ad7cfa83b4bc6b7f8a6f4ae22db34115daee926529bb37`.
- Receiver SHA256 `9d2ef832941c5ff2592da8428cb80104352590bcfea50c187624aaf4e90d60e0`.

Only GLIM received `CBS_E1B_LOCAL_UPDATE_GPU_BATCHING=1`. Its actual mapped
libraries passed the resolved-path gate before playback. Kimera and receiver
did not load the candidate libraries. The observer library/root was absent.
Production libraries and the default-off selector were not changed.

## Frozen settings

GLIM: Hesai/CPT7,5s lag; Kimera: front stereo/shared CPT7,25 states,experimental
IMU-support override unset (`current`). Saved ETH1 profiles match byte-for-byte.
Classic bidirectional source CBS uses `active_window_temporary`, removes the
received factors before marginalization, native-IMU mean alignment,.20s horizon,
one pair/update,full `schur_relative_between` covariance,scale1,Gaussian,no
health weighting. No source `live_revision`; both graph inspectors were off.

Downstream revisions retain stable-ID replacement and causal arrival semantics.
Receiver is X/V/W source-WNOA,full-history iSAM2QR,independent100ms timestamp
clock,no central IMU/bias/acceleration/GNSS factors,Qc `[.00375]*3+[.015]*3`,
pose/velocity/angular-rate prior sigmas1e-5/10/3,scale1,Gaussian,no gating.
E2 exact-Values reuse and nominal stationarity threshold1e-5 remain unchanged.

## Timing results

All-scan statistics use391 matched scans; the additional startup LOCAL row is
included only in the context-total sums. Percentiles use the existing linear
interpolation rule. Durations are host elapsed time, not isolated GPU kernel time.

| Measured scope | Retained baseline | Batched candidate |
| --- | ---: | ---: |
| LOCAL median |117.148ms|7.600ms|
| LOCAL p95 |218.181ms|14.391ms|
| LOCAL mean |100.078ms|7.302ms|
| LOCAL maximum |240.219ms|17.759ms|
| LOCAL total including startup |39.131s|2.856s|
| Complete GLIM scan median |307.376ms|182.365ms|
| Complete GLIM scan p95 |535.236ms|385.682ms|
| Complete GLIM scan mean |254.138ms|162.292ms|
| Complete GLIM scan maximum |843.719ms|709.788ms|
| Late30–40s LOCAL median/p95 |188.566/227.914ms|13.041/14.817ms|
| Late30–40s complete scan median/p95 |410.022/621.552ms|244.919/427.677ms|

The predeclared LOCAL median and p95 <=80% of baseline gate passes: actual ratios
are6.487% and6.596%. LOCAL total measured time is13.70x lower. Complete-scan
median falls40.67%,p95 falls27.94%,mean falls36.14%. Complete-scan p95 also passes
the800ms headroom gate at10Hz sensors/0.10x playback; the late-window p95 does too.

Controller wall duration is445.34s before and446.40s now: both runs are paced by
the same400s playback plus startup/drain. This test frees compute headroom; it
does not make a fixed-rate playback finish faster.

### Evidence that the intended path changed

| GPU factor linearization context | Baseline synchronous / cached | Candidate synchronous / cached |
| --- | ---: | ---: |
| LOCAL update |143950 /0|0 /130972|
| MAIN update |168874 /0|162254 /0|
| LOCAL covariance reconstruction |0 /195592|0 /190426|

Every observed LOCAL GPU factor linearization in this candidate consumed a
result prepared by the batcher; none took the per-factor synchronous fallback.
This is exact-current-Values batch preparation, not stale reuse across updates.
The count difference between live captures must not be described as intentional
factor removal: scheduling/solver history is not identical between these runs.

The old LOCAL synchronous path occupied37.724s,96.405% of its exclusive time.
That measured path is now0s. Batch preparation is included in graph-linearize
scope, now66.10% of LOCAL exclusive time; measured marginalization is1.06% of
the much smaller LOCAL total. Nested inclusive context times are not additive.

MAIN remains the clear remaining measured bottleneck: its root total changes
42.562→43.264s,including41.935s in synchronous GPU linearization in the candidate.
Its complete-scan timer mean is110.683ms out of162.292ms total. MAIN/direct-loop
batching was intentionally outside this patch. No follow-up optimization ran.
Median active keys stay150 in both early/late windows; late non-null factors
median793 and factor slots median825 match the retained baseline.

## Functional validation and coverage

Preflight passed18 focused Release regressions,source parameters/flags,profile
and raw-bag provenance,linkage/layout and actual loaded-library checks. Existing
E1b.4a synthetic/sanitizer and sampled real-input equivalence evidence remains
separately recorded; no new claim of a clean workspace-wide build or sanitizer
run is made here. All677 protected hashes and six clean source-repository HEADs
remain unchanged after the capture and journal replay.

| Output | Rows | Timestamp span | Path length |
| --- | ---: | ---: | ---: |
| Native GLIM |391|38.99988s|14.03687m|
| Native Kimera ROS odometry |184|39.59978s|13.74659m|
| Fused online/as-played |393|39.20000s|15.36280m|
| Fused final retrospective |393|39.20000s|14.02244m|

These are coverage/path diagnostics, not GT error results. CPT7 has3986samples
over39.85001s. Native startup offsets/tails pass the existing gates; Kimera has
the retained approximately3.2s startup pose gap,then maximum0.200001s pose gap.
No whole40s GT ATE claim is made.

- 181 Kimera→GLIM and180 GLIM→Kimera aligned factors constructed; wire and
  constructed full covariances exact,causal endpoint cutoffs valid,zero pre-stage
  rejects. Logged temporary lifecycles: GLIM181queued/156removed/25remaining;
  Kimera180tracked/156removed/24remaining. Sampled dense covariance whitening,
  Hessian/gradient and same-Values residual checks pass. This does not prove
  statistical independence from all past marginal-prior information.
- Receiver consumes21946versions:17696GLIM+4250Kimera.570accepted stable IDs,
  21351replacements,25explicit unsupported versions,pending/duplicate/stale0.
  Source revision payloads and ordering match the saved journal exactly.
  All admitted versions accounted for:570+21351+25=21946.
- 393strict publications pass,max normalized gradient9.851213209e-6.
  Maximum source-factor covariance numerical difference1.694066e-20.
  Final graph:392WNOA+570relative+3initial priors=965factors;
  zero central IMU/bias/acceleration factors/variables.
- Transport maximum queue42,sampled monitor maximum3,final queue0.
  Sampled output-age median/p95/max0.060003/0.090004/0.100005s.
- One unchanged-executable replay of the exact saved journal passes all existing
  tests. All nine explicitly hashed trajectory/state/clock/event files are
  byte-identical; maximum X/V/W/orientation differences0. This is deterministic
  receiver replay, not proof that separate source captures share identical paths.
- All owned processes exited; only the containers' idle tail/shell remain.

## Limits, decision and recovery

This is one observer-free candidate capture versus one retained baseline, not
a paired simultaneous benchmark. Desktop GPU clocks/load were not controlled.
Both runs have the same settings/sensor window and source-injection counts, but
live scheduling and revision payloads differ. Prior exact-input synthetic and
six real-snapshot tests establish their stated equivalence scope; there was no
independent ordinary solver following every update in this timing capture.

**Accept the first40s LOCAL timing result.** Keep default off and the private
candidate isolated. Do not infer full160/fullETH1 or1x capacity from this result.
The remaining MAIN and full-history receiver costs still need their own bounded
decisions. No covariance/method change,MAIN extension,longer run or push followed.

Reproduce the analysis with `analyze_profile.py --run-root RUN_ROOT`; full
comparison and raw-evidence pointers are in `comparison.json`. `prepare.py`,
`preflight.py`, `validate.py` and `summarize.py` retain the one-shot procedure;
they refuse existing outputs. `evidence/` holds the small validation/harness
archive and byte hashes; raw profiles,ROS logs,bags,journal and trajectories
remain under the run root and are not copied into Git. The execution ledger
records this completed stage and its follow-up commit identity. No push.
