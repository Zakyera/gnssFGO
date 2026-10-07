# ETH1 frozen replay extension and numerical stopping point

## Result

The longer replay used the **exact validated WNOA executable**, without any
model, covariance, prior, source or numerical-policy change. CBS-OFF/WNOA
produced valid publications through **96.88 seconds** after journal start,
then stopped at support **964**, at **96.98 seconds**. The frozen acceptance
rule rejected a floating-point-scale objective increase that exceeded its
allowance, even though the candidate reached the required gradient tolerance.

This is an incomplete 120-second experiment, not a failed CBS-ON accuracy
comparison. CBS-ON and WNOJ were not started after the first validity failure,
as specified before the run. No completed-trajectory scores or deterministic
120-second repeat are claimed. No settings were adjusted to force completion.

The valid prefix shows bounded publication behavior: current-pose correction
RMS is 1.70 cm, maximum 9.35 cm, and maximum published speed is 1.047 m/s.
All 11 selected batch checks before the failure passed. These observations
do not establish ground-truth accuracy or certify the failed publication.

Artifacts:
[/home/yeranis/results/gnssfgo_eth1_wnoa_wnoj_120s_20261006/](/home/yeranis/results/gnssfgo_eth1_wnoa_wnoj_120s_20261006).
The preceding validated short experiment is
[the 40-second report](/home/yeranis/repos/gnssFGO/GNSSFGO_ETH1_WNOA_WNOJ_40S_NUMERICAL_FIX_20261006.md).

## Frozen configuration and available data

This remains an **offline causal replay**, not a new live GLIM/Kimera run.
The source is the retained ETH1 mission `2024-10-01-11-29-55`, from
`/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live120_zero_20260930/`.
Native GLIM and Kimera source settings and classic CBS exchange are untouched.

The receiver has no Combined IMU factors, bias states or gravity initialization.
It uses only the raw IMU timestamps for its independent 100 ms support clock;
the raw IMU values do not drive its prediction. GLIM and Kimera relative beliefs
arrive at their recorded availability times and connect their physical
endpoints through the existing GP interpolation. Stable-ID revisions replace
earlier source factors rather than add duplicate observations.

The executed WNOA case retains:

- X pose, world V velocity and body W angular-rate variables; no acceleration
  variable.
- Existing `GPWNOAPrior` and `GPWNOAInterpolator`, including their existing
  numerical-Jacobian configuration.
- Qc diagonal `[.00375,.00375,.00375,.015,.015,.015]`.
- Identity pose prior with sigma `1e-5 rad/m`, zero velocity with sigma
  `10 m/s`, and zero angular rate with sigma `3 rad/s`.
- Full-history iSAM2, QR, full native 6x6 source covariance at scale one.
- No GNSS, gating, robust loss, health weighting or covariance augmentation.
- At most 20 full-relinearization refinement updates; normalized-gradient
  threshold `1e-5`; the unchanged stationary-only 8-epsilon objective allowance.

WNOJ was prepared with its unchanged short-run settings as the comparison,
but was not executed. Production defaults were not changed.

The target arrival cutoff is `1727775128003475000 ns`, 120 seconds after
`1727775008003475000 ns`. The original capture's last IMU timestamp is
`1727775127853502000 ns`, at 119.850027 seconds. Consequently the available
grid contains **1,193** supports, from `1727775008583475000` through
`1727775127783475000 ns`, spanning 119.2 seconds. No missing end-of-capture
IMU interval was fabricated.

Every source event line and its order were preserved; only the end-of-stream
marker was normalized to the declared cutoff. No additional information was
created or drained after that cutoff. The first-40-second journal prefixes
are exactly identical to the previous inputs in both modes. The physical
IMU timestamp/value streams are also identical across OFF and ON.

| Prepared input | OFF | ON |
| --- | ---: | ---: |
| IMU timestamp events | 11,986 | 11,986 |
| Source version events | 70,711 | 70,748 |
| Supported unique GLIM IDs | 1,189 | 1,189 |
| Supported unique Kimera IDs | 581 | 581 |
| Replacements if the complete journal is processed | 68,916 | 68,953 |
| Unsupported pre-initialization versions | 25 | 25 |

These are **input inventory counts**, not claims that the stopped receiver
consumed the complete stream. At failure it had inserted 1,427 IDs and
performed 55,337 replacements: 960 active GLIM factors and 467 active Kimera
factors. Recorded insertion times respect source and support availability.

## Exact failure at support 964

Physical timestamp: `1727775104983475000 ns`.
Support availability timestamp: `1727775104983497000 ns`.
The graph contains 965 support states, 2,895 variable keys and 2,394 factors:
964 WNOA priors, 3 initial priors and 1,427 source factors. The newest source
endpoint is only 28.6098 ms behind the current support.

| Operation | Objective | Normalized gradient | Decision |
| --- | ---: | ---: | --- |
| Before publication refinement | 342988.16288207402 | 0.46057489388874134 | Refine |
| First candidate | 342987.60748445161 | 1.2922106403310365e-5 | Accepted; not yet stationary |
| Second candidate | 342987.60748445254 | 9.5010152684237752e-8 | Rejected by objective bound |

The second candidate's objective increase is **9.313225746154785e-10**, or
**16 floating-point units**, a relative increase of `2.7153242691361596e-15`.
The fixed allowance is **6.092683823845358e-10**. Its gradient is well below
`1e-5`, but satisfying the gradient requirement alone is insufficient under
the frozen rule. The objective allowance must also pass.

The proposed aggregate local-coordinate state step norm is
`7.7252328014211463e-7`. This combines different state units and is not a
position displacement. The rejected solver copy was not committed. The retained
state is finite and its current speed is 0.987233 m/s, but its gradient remains
`1.29221e-5`, so the publication is correctly withheld under the present policy.
The causal output ends at support 963, at 96.88 seconds; support 964 exists
only in the failed diagnostic snapshot.

The saved copied-graph LM reference and its restart both returned the retained
state unchanged, reporting zero completed iterations. Their gradient also
remained `1.29221e-5`. **Zero state difference here is not an independent
convergence certificate.** The checker therefore marks this reference
checkpoint invalid, rather than treating identical outputs as proof of validity.

The 40-second fix admitted a two-unit increase; this longer attempt encounters
a sixteen-unit increase. The prior report explicitly described the 8-epsilon
allowance as a small comparison allowance, not a certified bound on the total
nonlinear objective's evaluation error. This run demonstrates that the same
allowance does not eliminate every numerical acceptance stop on the larger
graph. The trace does not prove why the floating-point difference grew, or
that the true mathematical objective increased by that amount.

This is not a timeout, out-of-memory termination, non-finite state or observed
trajectory explosion. It also does not prove that a central IMU factor is
required or that WNOA's motion model has failed. The concrete failed condition
is the unchanged numerical acceptance comparison.

## Valid prefix checks and stability

There are 963 successful strict publication checks plus the initial pose row,
giving 964 causal trajectory rows. Eleven copied-graph checkpoints before the
failure pass the same criteria as the 40-second experiment. They include the
scheduled 40/60/80-second checks and checks at each admitted roundoff exception.
Eight refinement steps used the previously validated roundoff exception.
The largest pre-failure checkpoint differences from the copied reference are:

| Quantity | Maximum difference |
| --- | ---: |
| Position | 7.462e-9 m |
| Orientation | 6.347e-9 rad |
| World velocity | 3.812e-8 m/s |
| Body angular rate | 1.083e-7 rad/s |

The first 40 seconds of causal trajectory, states and support clock are
byte-identical to the validated OFF/WNOA run. All 394 corresponding full-state
publication snapshots and source ledgers also match byte-for-byte. The old
40-second final cutoff snapshot is deliberately excluded: it represents a
different information-set boundary than a continuing publication.

For stability, each row below compares the estimate of the **same physical
state** before and after the next successful publication. The failed
publication is excluded. There are 958 comparisons after source information
becomes available; these are corrections, not ground-truth errors or ordinary
motion between successive timestamps.

| Correction | Median | RMS | 95th percentile | Maximum |
| --- | ---: | ---: | ---: | ---: |
| Current position, m | 0.008378 | 0.016997 | 0.035278 | 0.093498 |
| Current velocity, m/s | 0.080285 | 0.121444 | 0.237219 | 0.555170 |
| Current orientation, rad | 0.005749 | 0.016224 | 0.034211 | 0.078798 |
| Source-covered position, m | 0.004748 | 0.008606 | 0.016812 | 0.060094 |

The source-covered row uses the latest support no later than an arrived source
endpoint; this does not assert full observability. Median published speed is
0.520971 m/s, 95th percentile 0.972641 m/s and maximum 1.047103 m/s. The
maximum prediction tail is 208.451 ms. The published receiver-origin path
length is 53.887359 m. It is not a GT path or final reconstructed path.

There is no matched longer CBS-OFF/ON or WNOA/WNOJ accuracy conclusion.
Ground-truth scoring was not run after the numerical stop, consistent with
the user's priority of validating the model rather than ranking CBS accuracy.

## Execution cost and preservation

Wall time was 143.64 seconds, including diagnostic snapshots and copied batch
checks. Peak child RSS was 465,320 KiB, about 454 MiB. The process exited 1
at the explicit publication guard; it was not killed by resource limits.
This is not a benchmark of three estimators running live together.

All **31,567** protected files match their pre-run hashes, including historical
results, source data, configurations, runtime and validated executables. The
two new journal hashes also remain unchanged. Additional cross-checks verified
24,384 earlier recorded file hashes and all 193 saved build files against their
prior manifests. The two executable hashes were checked before launch.
`git diff --check` passes.

No executable was rebuilt, and no estimator source or existing artifact was
modified. New files are restricted to this extension's preparation/check/report
artifacts and this report. The preceding Release and focused ASan/UBSan tests
remain retained evidence for the unchanged implementation; no new test-suite
execution or sanitized full replay is claimed.

## Reproduction and artifact identities

Executed commands:

```bash
python3 /home/yeranis/results/gnssfgo_eth1_wnoa_wnoj_120s_20261006/prepare.py
bash /home/yeranis/results/gnssfgo_eth1_wnoa_wnoj_120s_20261006/run.sh primary wnoa off
python3 /home/yeranis/results/gnssfgo_eth1_wnoa_wnoj_120s_20261006/prepare_checks.py
/home/yeranis/.local/share/pipx/venvs/evo/bin/python /home/yeranis/results/gnssfgo_eth1_wnoa_wnoj_120s_20261006/preflight.py
/home/yeranis/.local/share/pipx/venvs/evo/bin/python /home/yeranis/results/gnssfgo_eth1_wnoa_wnoj_120s_20261006/check_case.py wnoa off
/home/yeranis/.local/share/pipx/venvs/evo/bin/python /home/yeranis/results/gnssfgo_eth1_wnoa_wnoj_120s_20261006/finalize_stopped.py
```

The replay ran asynchronously while the diagnostic scripts were prepared.
Preparation and wrapper refuse existing outputs; reproduction requires a new
isolated output root. The full replay command, environment, exit status and
resource measurements are in `command_off_wnoa.json`. `check_case.py` exits 2
because the case is incomplete and fails the frozen numerical condition.
Prepared full-comparison analysis, repeat and scoring scripts were not executed.

The replay uses the same network-disabled, read-only container image
`sha256:6a664509d183e504230b22edfd747dfe1ca5bca8ae6fc34c76154db8ffd1fc8d`.
The validated executable/runtime are mounted read-only; only the new artifact
root and temporary filesystem are writable. The executable is still named
`replay40`, but has no 40-second input limit; the journal controls the duration.

| Artifact | SHA-256 |
| --- | --- |
| Executed WNOA binary | `aea13b62cfd563a2cebf8cb52dcdb6fee5d0128d24a54d3620221fb4dd4f821a` |
| Original retained OFF journal | `d75e2cf25ad413b2c4f9f93e5331b601a695d5cb996a6ab44d32d543010f49e0` |
| Prepared OFF journal | `44d89960cc69941ec13e2e6299120ef3eb8398e16cffa2149d521af8be948590` |
| Prepared ON journal, not executed | `8ba7ea3a306de88c763635380ddc6f359cd5219f688a46d253779cfe271715fe` |

Machine-readable evidence is in `stopped_verification.json`,
`check_off_wnoa.json`, `valid_prefix_transitions.csv`, the publication logs,
and `off_wnoa/snapshot_964/`. The latter contains the full retained state,
factor payload probe, source ledger, linearized gradient diagnostics,
copied-reference states and iteration log.

Starting and final HEAD remain
`a842e7ef72e7a40f0cdfec826f089c66af364ae0`. No commit was created. The worktree
was already dirty and remains so; unrelated user work was preserved.

## Remaining boundary

The no-central-IMU WNOA receiver now has a longer, numerically checked published
prefix than the previous 40-second test, but **the full 120-second extension
is not validated**. Its concrete blocker is the finite-precision publication
acceptance policy at support 964. Resolving that policy requires a separately
justified numerical change; simply increasing the constant until the run
passes is not validation. No such change was made here.
