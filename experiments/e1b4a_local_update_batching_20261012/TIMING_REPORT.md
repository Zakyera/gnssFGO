# E1b.4a — frozen LOCAL-update timing result

**Synthetic timing gate passed. No live-capacity claim or default change.**
The isolated batching path substantially reduces this fixture's LOCAL update
time while preserving every tested numerical and solver-decision comparison.

## Scope and provenance

Authorized by "ok proceed to the timing test", 2026-10-12. This continues
E1b.4a in `GNSSFGO_RUNTIME_ENGINEERING_PLAN_20261011.md`; it does not implement
another optimization. Parent report/HEAD:
`2a3823c7387baeb48b38db4a8e19bfd96c4f8d79`, implementation/test commit
`440fba99e9ce4d59dec03758b9d06f55927b2470`.

Unmodified private `release_2/test_solver_3`, SHA256
`223232f359c3580886a2ff75f91daf26548289c51b0115366678b0878babfa5f`, ran once.
No rebuild, estimator parameter change, dataset playback, source or receiver
experiment, tuning, performance retry or push. The candidate remains default-off.
Ordinary and batched modes use the same executable/private GTSAM library, with
the scoped callback respectively disabled/enabled; this is not a cross-build
performance comparison against an old installed DSO.

The frozen runner and `TIMING_PROTOCOL.txt` describe the exact invocation.
Local evidence root:
`/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/e1b4a_local_update_batching_20261012`.

## Workload and measurement

- Two synthetic workloads: CPU-only; 64 new GPU VGICP factors per update after
  initialization. Each uses 40 updates, .1 s supports and .65 s fixed lag,
  native-style X/V/B states, IMU/bias/pose factors, removal/replacement and
  marginalization. This tests GLIM's LOCAL-side solver, not central IMU fusion.
- The retained geometric fixture has **864 points**, a .5 voxel resolution and
  four CUDA streams. Many independent factors use this same synthetic cloud/map.
  It tests batching many factors; it is not a real Hesai scan/window workload.
- Independent mutable factor/solver histories, both modes warmed once per
  workload, then six alternating paired repetitions. No best-run selection.
  Total: **960 measured updates**, 240 per mode/workload, 12 paired comparisons.
- All 40 steps, including initialization, enter the primary statistics.
  p95 uses linear interpolation at `(n-1)*.95`. Per-round results are retained.
- Timing encloses `solver.update()` and profiler/dispatch scopes. Input setup,
  `calculateEstimate()`/snapshot work and final covariance evaluation are outside.
  No intermediate extra covariance linearizations perturb the timing workload.
  Native synchronous-call warning output is unchanged and included when emitted
  inside update: this is not a pure CUDA kernel-time measurement.
- CPU affinity 2, `OMP_NUM_THREADS=1`; Intel Core Ultra 9 275HX, RTX5070Ti Laptop,
  NVIDIA driver 580.178.04. CPU governor was `powersave`; GPU clocks not locked.
  Desktop GPU activity remained present (16% before, 18% after); no unrelated
  process was terminated. These are endpoint load observations, not a continuous
  load/clock recording. Alternating repetitions limit, but do not eliminate,
  scheduling/thermal interference.

The entire harness took **31.281 s wall time**, including warmups and checks.

## Timing result

| Workload | Statistic | Ordinary | Batched | Change |
| --- | --- | ---: | ---: | ---: |
| GPU-heavy | Median update | 112.310 ms | 4.30792 ms | 96.16% shorter; 26.07x speedup |
| GPU-heavy | p95 update | 115.43625 ms | 4.66825 ms | 95.96% shorter; 24.73x speedup |
| GPU-heavy | Mean update | 102.10431 ms | 4.30853 ms | 95.78% shorter |
| GPU-heavy | Maximum update | 125.996 ms | 14.8115 ms | Descriptive maximum, not the acceptance gate |
| CPU-only | Median update | 0.156420 ms | 0.156728 ms | +0.20% / +0.31 microseconds |
| CPU-only | p95 update | 0.179174 ms | 0.185760 ms | +3.68% / +6.59 microseconds |
| CPU-only | Mean update | 0.149008 ms | 0.152024 ms | +2.02% / +3.02 microseconds |

Frozen GPU gate was candidate **median AND p95 <=80%** of ordinary. Observed
ratios are **3.84% and 4.04%**, respectively. Every paired GPU run clears that
gate independently, not only the pooled result:

| Round | First mode | Ordinary median | Batched median | Ordinary p95 | Batched p95 |
| --- | --- | ---: | ---: | ---: | ---: |
| 0 | Batched | 112.616 ms | 4.32524 ms | 114.43285 ms | 4.59838 ms |
| 1 | Ordinary | 109.347 ms | 4.26958 ms | 116.59600 ms | 4.65936 ms |
| 2 | Batched | 112.493 ms | 4.31391 ms | 117.29255 ms | 4.59038 ms |
| 3 | Ordinary | 111.8465 ms | 4.30214 ms | 114.53580 ms | 4.60443 ms |
| 4 | Batched | 112.8825 ms | 4.32773 ms | 115.12475 ms | 4.72018 ms |
| 5 | Ordinary | 113.0505 ms | 4.30437 ms | 118.92280 ms | 4.93054 ms |

CPU fallback is not free. Its small pooled overhead is within the observed
paired-run variability: median changes range from -7.61% to +9.95%; p95 changes
from -5.61% to +8.23%. Thus there is **no demonstrated material CPU regression
beyond this benchmark's repeatability**, not proof of zero overhead. Six very
short CPU histories do not justify a precise universal overhead estimate.

## Why it is faster, and what remains

Across the six GPU-heavy histories:

| Count inside measured updates | Ordinary | Batched |
| --- | ---: | ---: |
| GPU factor linearize calls | 111,822 | 111,822 |
| Synchronous fallback calls | 111,822 | 384 |
| Prepared-result/cache consumptions | 0 | 111,438 |
| Scoped graph callback requests | 0 | 468 |
| Successfully batched graph requests | 0 | 462 |

**No GPU factor evaluations were dropped:** the same logical linearize calls
now mostly consume results prepared together at the exact requested Values.
The expensive per-factor synchronous fallback count falls by **99.66%**.
Each run still has 64 synchronous calls at update index1; the remaining updates
have none in this workload. This is consistent with the intentionally untouched
individual-factor paths; this capture does not add call-site attribution for
those 64 calls. No extension was made to eliminate them.

Active keys match exactly and stay bounded at 21; factor-slot vector size reaches
472 (includes holes, not an assertion that all472 slots are live factors).
Per-update GPU-call counts, key counts and slot sizes agree across both modes.
CPU-only requests all take ordinary fallback:468 requests, zero GPU batches.

The large fixture speedup does **not** imply 26x faster whole GLIM. MAIN,
direct-loop cases not exercised at production frequency, other frame work,
real point-cloud sizes, thread sharing and the gnssFGO receiver remain outside
this performance claim. The previous live profile's96.4% LOCAL observation
motivates this boundary but is not a measured live speedup of this patch.

## Numerical and preservation checks

The benchmark's one composite GoogleTest passed, containing all12 paired solver
history comparisons and24 dense covariance matrix comparisons. The parser checks
all960 rows and exact alternating order; no missing or duplicate samples.

| Comparison over paired histories | Maximum difference |
| --- | ---: |
| Position | 0 |
| Velocity | 0 |
| Gyro/accel bias coordinates | 0 |
| Orientation | 3.70537e-18 rad |
| Complete final covariance (105x105) | 0 |
| Final endpoint-relative covariance (6x6) | 0 |

All update estimates and linearization points, factor types/keys/slots,
relinearization/re-elimination counts and tested batch/incremental/marginalization
branch-call counts match. Full covariance in this timing test is checked at the
end, **not at every step**. Earlier per-step covariance, exception, repeated-run,
Release and sanitizer results are in `REPORT.md`; they were not rerun here.

Before and after: all670 baseline hashes, 672 host-protected hashes (including
preparation inputs), 429 container build-input hashes and six clean source
repository HEADs match. These overlapping sets must not be summed as unique
files. Test executable, wrapper and GTSAM candidate hashes also match. Saved
`ldd` resolves the expected private GTSAM/CBS/GPU/profile dependencies.

Operational notes: the existing container was initially stopped; its inspected
startup command is only `tail -f /dev/null`. It was restarted without sensors.
Two preflight metadata-check mistakes were caught before launching: host-only
paths had to be checked on the host, and the finalized committed runner had to
be pinned rather than its older build-time snapshot. Failed preflight records
are retained in `timing_audit/` and `timing_audit_v2/`; successful pre/post evidence
is in `timing_audit_v3/`. No test executable, runner or gate was changed and
the benchmark itself ran exactly once, exited0, and left no estimator running.

## Evidence, decision and stop

- `release_2/timing1/`: raw log, GoogleTest XML, exact command/elapsed time,
  `ldd` and success record.
- `timing_summary.json`: all statistics, per-round results, count checks and
  SHA256 of raw evidence; tracked copy alongside this report.
- `timing_audit_v3/pre.json`, `post.json`: raw integrity/environment evidence.
- `TIMING_PROTOCOL.txt`, `analyze_timing.py`, `timing_integrity.py`: reproducible
  frozen scope and read-only analysis/integrity scripts.

**Decision: E1b.4a synthetic timing gate passed; stop here.** The next sensible
gate is a separately authorized bounded real-source equivalence/throughput
capture using this same default-off candidate. Do not declare the live-delay
problem solved, enable the default, extend another solver path or run full160
on the strength of this fixture alone. No production/frozen estimator file was
edited. Local timing-evidence commit follows the parent above, on the same
engineering branch; no push.
