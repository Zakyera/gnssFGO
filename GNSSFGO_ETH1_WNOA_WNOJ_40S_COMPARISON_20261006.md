# ETH1 WNOA/WNOJ 40-second comparison

## Result and stopping point

**The four-case comparison is incomplete.** CBS-OFF/WNOJ completed the frozen
40-second journal and passed all selected numerical checks. CBS-OFF/WNOA
stopped at support **344**, at **34.98 seconds after journal start**, because
the unchanged publication acceptance policy rejected an objective increase of
**5.820766091346741e-11**. That is two representable floating-point increments
at an objective of about 170,127.

The retained WNOA state agrees with a separate copied-graph batch solve to
**1.973e-9 m maximum position difference across all 345 support states**.
This is evidence of a numerical acceptance limitation, not observed WNOA
trajectory divergence. Nevertheless, the retained state's normalized gradient
was `1.0836412817209803e-5`, above the frozen `1e-5` criterion. The run therefore
does not pass the agreed publication rule.

CBS-ON, deterministic 40-second repeats, and ground-truth scoring were not run
after this stop. No tolerance, Qc, covariance, source settings or model was
changed to complete the comparison.

| Planned case | Execution | Numerical result |
| --- | --- | --- |
| CBS-OFF, WNOJ without central IMU | Complete 40 s journal; 395 support states | Pass: 394 support-publication checks plus final check; five batch checkpoints |
| CBS-OFF, WNOA without central IMU | Stopped at support 344, journal time 34.98 s | First 343 support checks pass; next check rejects a two-ULP objective increase |
| CBS-ON, WNOJ without central IMU | Not started | Withheld under the predeclared stop condition |
| CBS-ON, WNOA without central IMU | Not started | Withheld under the predeclared stop condition |

Artifacts: `/home/yeranis/results/gnssfgo_eth1_wnoa_wnoj_40s_20261006/`.
The pre-run specification is `PROTOCOL_BEFORE_RUN.md`; machine-readable findings
are `check_off_wnoj.json`, `check_off_wnoa.json`, and `stopped_verification.json`.

## Frozen inputs and models

The source is the retained ETH1 mission `2024-10-01-11-29-55`, using the
`gnssfgo_eth1_live120_zero_20260930` OFF/ON source journals. There was no new
GLIM/Kimera execution. Source sensors, classic CBS behavior and source-side
IMU conventions remain those of the saved profiles; no source convention was
silently changed. In these receiver models raw IMU **values are unused**:
timestamps supply the independent support clock only.

Input files are under
`/home/yeranis/results/gnssfgo_eth1_source_wnoj_40s_20261005/inputs/`:

| File | SHA-256 |
| --- | --- |
| `off_events.csv` | `89884ce8a4ebab0496dda5f140a0c390a3f8435a1d4289e03d50200acd5a3010` |
| `on_events.csv` — prepared, not executed | `8a89b6b58a605615511c7eb3e1514ffcd8d9233f3b1f8988a064e62d77417182` |

The journal starts at `1727775008003475000 ns`. Its inclusive availability
cutoff is `1727775048003475000 ns`. The support clock starts after 59 raw IMU
timestamp events at `1727775008583475000 ns`, then advances every 100 ms.
The complete grid has 395 supports spanning 39.4 s. No later revisions are
drained into the 40-second result.

Both models use full-history iSAM2, QR, full native source covariance with
scale one, stable-ID factor replacement, no central Combined IMU factor,
no GNSS, no bias states, no gravity initialization, no gating, no robust loss
and no health weighting. Initial pose is identity with sigma `1e-5` rad/m,
initial world velocity is zero with sigma `10 m/s`, and initial body angular
rate is zero with sigma `3 rad/s`.

| Setting | WNOJ | WNOA |
| --- | --- | --- |
| Support variables | Pose X, world velocity V, body angular rate W, local tangent acceleration A | X, V, W only |
| Stock motion implementation | `GPWNOJPriorFull`, `GPWNOJInterpolatorFull` | `GPWNOAPrior`, `GPWNOAInterpolator` |
| Qc diagonal | `[2.5,2.5,2.5,10,10,10]` | `[.00375,.00375,.00375,.015,.015,.015]` |
| Rotational/translational Qc units | rad²/s⁵, m²/s⁵ | rad²/s³, m²/s³ |
| Additional initial prior | A=0, sigma 10 | None |

These are the settings validated in
`GNSSFGO_SOURCE_WNOJ_FULL_RELINEARIZATION_20261006.md` and
`GNSSFGO_SOURCE_WNOA_SHORT_COMPARISON_20261006.md`. In particular, this is not
the older 40-second WNOJ run with Qc=10000.

WNOA's Qc was selected before the earlier eight-second experiment to match
WNOJ's process pose-covariance block at 100 ms. That does not equate every
derivative covariance block or uncertainty over every prediction horizon.
There was no new noise sweep or selection using ground truth.

## Exact numerical stopping event

The frozen helper
`/home/yeranis/results/gnssfgo_source_wnoj_full_relinearization_20261006/build/FullPublicationRefinement.h`
uses at most 20 empty force-relinearization/full-solve updates on transactional
iSAM2 copies. The normalized gradient must be at most `1e-5`; every accepted
step must have a non-increasing finite objective.

The normalized gradient is
`||diag(JᵀJ)^(-1/2) Jᵀr||∞`, evaluated at the current estimate. It is a local
first-order stationarity test, not proof of global optimality. The objective
is half the total squared whitened residual. The exact zero-increase comparison
is in `online_fgo/include/offline_process/BoundedISAM2Refinement.h`, function
`NonIncreasingFiniteError`, lines 49–51.

At physical time `1727775042983475000 ns`, availability time
`1727775042983482000 ns`, the WNOA graph contained 345 support states
(1,035 variable keys), 344 WNOA factors, three initial priors, 340 GLIM factors
and 157 Kimera factors. There were no central IMU factors. All retained state
values were finite.

| Operation at support 344 | Objective | Normalized gradient | Decision |
| --- | ---: | ---: | --- |
| Before strict refinement | 170127.71172853044 | 0.38856112242778718 | Refine |
| First refinement step | 170127.49697813028 | 1.0836412817209803e-5 | Accepted, but gradient still above threshold |
| Second refinement candidate | 170127.49697813034 | Not evaluated | Rejected by exact objective comparison |
| Independent LM on a copied graph | 170127.49697813005 | 4.470493975329831e-8 | One LM iteration; restart unchanged |

The rejected increase is `5.820766091346741e-11`, relative increase
`3.421414054010913e-16`. One floating-point unit at this magnitude is
`2.9103830456733704e-11`; the increase is exactly two such units. The gradient
misses its threshold by 8.364%. The logged `nan` for the rejected candidate's
gradient is an intentional “not computed” marker in the helper, **not evidence
that the state became NaN**. Rejection happens before its gradient is evaluated.

Maximum retained-state versus independent-LM differences at the same physical
timestamps, over all 345 states:

| Quantity | Maximum difference |
| --- | ---: |
| Position | 1.9727266081e-9 m |
| Orientation | 1.3081981751e-8 rad |
| World velocity | 3.9916594077e-8 m/s |
| Body angular rate | 2.7555353010e-7 rad/s |

The copied LM optimizer starts at the incremental estimate and uses the same
factors, covariance and priors. Its solution never feeds back into the receiver.
Agreement establishes a numerically tiny local discrepancy here; it does not
prove global optimality or motion-model accuracy.

The second candidate was rolled back. Support 344 was saved for diagnosis but
not emitted to the as-played trajectory: `online_states.csv` ends at support
343. A new support and source factors had already been added when the
publication guard stopped execution; this is a failed-publication snapshot,
not a completed final trajectory.

Relevant evidence is in `off_wnoa/publication_convergence.csv`,
`off_wnoa/publication_steps.csv`, and `off_wnoa/snapshot_344/`. The last source
endpoint was `1727775042955201400 ns`, about 28.274 ms behind the current
physical support. This stopping event is not evidence of an unusually long
measurement-arrival outage.

## Checks completed before the stop

Both new builds exactly reproduce their respective frozen eight-second
as-played trajectory, online state rows and support clock. All 74 retained
first-eight-second full-state snapshots and source ledgers are byte-identical.
The comparison deliberately excludes each short run's final snapshot, whose
information set includes arrivals after its last regular publication.

| Check | WNOJ OFF | WNOA OFF |
| --- | --- | --- |
| Release diagnostic build | Pass | Pass |
| First-eight-second byte equivalence | Pass | Pass |
| Batch checkpoint near 8 s | Pass | Pass |
| Batch checkpoint near 20 s | Pass | Pass |
| Batch checkpoint near 30 s | Pass | Pass |
| Checkpoint near 40 s and final | Pass | Not reached |
| Strict publication checks | All 395 pass | 343 pass; support 344 fails |
| Largest retained publication gradient | 9.9457462587e-6 | 1.0836412817e-5, at failure |

The WNOJ batch comparisons have maximum position difference `2.267e-9 m`,
velocity difference `5.027e-8 m/s`, angular-rate difference `5.693e-7 rad/s`,
orientation difference `1.444e-8 rad`, and acceleration difference `7.466e-6`
in the corresponding mixed angular/linear acceleration coordinates. Its final
support path is 14.0693848661 m. This path length is not a ground-truth score.

For every one of the first 344 diagnostic publication snapshots, WNOJ and
WNOA have identical source IDs/revisions and identical physical/availability
timestamps. WNOA's entire executed dispatch is an exact prefix of WNOJ's:
497 unique source IDs and 18,467 replacements. Every insertion occurs no earlier
than source availability and required support availability. At the selected
8/20/30-second checkpoints, all 787 checked full 6×6 source covariance matrices
match exactly between models. Each copied-graph reference also verifies that
the original factor payloads were not mutated.

The completed WNOJ case consumes 573 source IDs (391 GLIM, 182 Kimera), with
21,482 replacements and 25 pre-initialization unsupported versions. It has no
stale/duplicate revisions or pending beliefs at the end.

Wall time including diagnostic recording and selected batch references was
63.29 s for complete WNOJ and 21.01 s for the stopped WNOA prefix. Peak child
RSS was approximately 409 MiB and 167 MiB respectively. These are unequal
durations with instrumentation, **not a controlled speed comparison or proof
of live throughput**. Neither run stopped from memory exhaustion or timeout.

The build logs retain existing abstract GP-interpolator stub warnings about
missing returns. No new estimator code path was introduced to address them.
The current task compiled the isolated Release builds but did not rerun the
earlier mathematical unit/sanitizer suite; no such new pass is claimed.

## Interpretation and remaining comparison

The 40-second WNOJ extension passes its numerical criteria. WNOA's first
stopping event is a floating-point-scale objective increase coupled with a
slightly missed gradient threshold, while the saved solution is essentially
identical to a locally converged batch reference. The evidence does **not**
show that WNOA cannot predict the motion, that a central IMU is necessary, or
that either model produces better CBS-ON fusion.

A justified next step would be a narrowly scoped numerical acceptance-policy
check on this saved failure, with explicit finite-precision and state-agreement
criteria and synthetic regression tests. It should not involve changing Qc,
source covariance, priors, or selecting a model using ATE. No such policy change
was implemented here.

The prepared full-comparison analysis and evaluation scripts remain unexecuted.
They require all four cases to pass before producing the planned comparison.
Consequently there is no new OFF/ON accuracy table or declared WNOA/WNOJ winner.
The retained scoring protocol uses 364 common timestamps, 113 prism matches
over about 11.2 s, and 353 one-second IE-TC RPE pairs. A future result must not
call that prism ATE “full 40-second ground-truth coverage.” It must also retain
the caveat that native GLIM is the saved end-of-source-run export, whereas the
receiver uses only arrivals through the 40-second cutoff.

## Reproduction and preservation

Commands executed, in order:

```bash
python3 /home/yeranis/results/gnssfgo_eth1_wnoa_wnoj_40s_20261006/prepare.py
bash /home/yeranis/results/gnssfgo_eth1_wnoa_wnoj_40s_20261006/run.sh build wnoj
bash /home/yeranis/results/gnssfgo_eth1_wnoa_wnoj_40s_20261006/run.sh primary wnoj off
/home/yeranis/.local/share/pipx/venvs/evo/bin/python /home/yeranis/results/gnssfgo_eth1_wnoa_wnoj_40s_20261006/check_case.py wnoj off
bash /home/yeranis/results/gnssfgo_eth1_wnoa_wnoj_40s_20261006/run.sh build wnoa
bash /home/yeranis/results/gnssfgo_eth1_wnoa_wnoj_40s_20261006/run.sh primary wnoa off
/home/yeranis/.local/share/pipx/venvs/evo/bin/python /home/yeranis/results/gnssfgo_eth1_wnoa_wnoj_40s_20261006/check_case.py wnoa off
/home/yeranis/.local/share/pipx/venvs/evo/bin/python /home/yeranis/results/gnssfgo_eth1_wnoa_wnoj_40s_20261006/finalize_stopped.py
```

The WNOA run exits 1 by design at the publication guard; its checker exits 2
because the case is incomplete. Preparation and run scripts refuse to overwrite
an existing experiment directory. Reproduction requires a new output root.
`prepare_evaluation.py` was also executed to prepare `score.py`; `score.sh` was
not executed. The NumPy checks use the existing `evo` Python environment, not
the system Python, which does not contain NumPy.

The receiver container was network-disabled, with a read-only filesystem,
read-only production repository/runtime/inputs and only this new artifact root
writable. Image:
`sha256:6a664509d183e504230b22edfd747dfe1ca5bca8ae6fc34c76154db8ffd1fc8d`.
Exact compiler flags, commands, environment variables, wall times and RSS are
retained in `command_*.json` and `run.sh`.

Only isolated diagnostic copies were changed: lightweight state/ledger logging,
sparse diagnostic gradients instead of dense eigenspectra, selected copied batch
checks, and an explicit halt after the frozen refinement routine fails. The
refinement routine and motion models themselves are byte-unchanged. All edits
are preserved in `*_DiagnosticReplay.diff` and `*_SnapshotAudit.diff`.

Binary SHA-256:

- WNOJ: `a1abb3ab879339487c0e5b3dcf9637c9ea98740bbcdb45c184536f898b223631`.
- WNOA: `23288dcc708ac2209c7df1d762736a72b0bd95a3230eeefa9c689b525ca0c7dd`.

All 16 task-pinned files and all 21,026 files represented by the merged inherited
protection manifests match their recorded hashes. `git diff --check` passes.
The repository was already dirty; its status was unchanged before this new report
was added. No production estimator file was modified and no commit was created.

Starting and final HEAD: `a842e7ef72e7a40f0cdfec826f089c66af364ae0`.

**Status: incomplete comparison, stopped at the frozen numerical acceptance criterion.**
