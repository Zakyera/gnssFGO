# ETH1 unchanged timing reference before shared linearization

**B0 passed.** One fresh 40-second classic CBS-ON capture reproduces the large
LOCAL batching benefit, with the original functional checks and byte-identical
saved-journal receiver replay passing. MAIN remains the dominant measured cost.
This is the unchanged E1b.4a implementation, not the supervisor's proposed
shared-native-covariance implementation.

## Scope and identity

- Parent commit: `6bcc1d9b37c1ff245ae361a032cff30b82002796`.
- Run root: `/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_shared_linearization_b0_20261012`.
- ETH1 mission `2024-10-01-11-29-55`, bag offset 0, duration 40 s, playback 0.10x.
- One concurrent native GLIM, native Kimera and source-WNOA receiver capture;
  no second source run, rebuild, tuning, GT scoring, or retry.
- Retained private GLIM build: `runs/e1b4a_local_update_batching_20261012/release_2`.
  Wrapper SHA256 `83592aba553adbc2879f27b5f2cf3551826ab497a58201ba0101543934f49dec`;
  GTSAM SHA256 `369f45866a82c40170ad7cfa83b4bc6b7f8a6f4ae22db34115daee926529bb37`.
- Receiver SHA256 `9d2ef832941c5ff2592da8428cb80104352590bcfea50c187624aaf4e90d60e0`.

The prior successful harness was mechanically copied with only fresh output
paths, session namespace and ROS port 11418. `preparation.json` records the
25 copied text files and their original/new hashes. Both existing containers
were initially stopped and were restarted without rebuilding or reconfiguration.
Effective launch parameters, profile trees, source flags, private libraries and
actual loaded mappings passed the retained checks.

GLIM uses Hesai/CPT7 and a 5 s lag; Kimera uses the saved front-stereo/shared-CPT7
25-state profile with its experimental IMU-support override unset. Classic
bidirectional `active_window_temporary` exchange, native-IMU mean alignment,
0.20 s horizon, one pair/update, full `schur_relative_between` covariance,
scale 1, Gaussian factors and no health weighting remain unchanged. GLIM's
private LOCAL GPU-update batching remains enabled. Graph inspectors and the
expensive independent snapshot oracle are off; retained timing and bounded
covariance diagnostics remain on.

The receiver remains source-WNOA X/V/W, full-history iSAM2 QR, independent 100 ms
timestamp clock, no central IMU/bias/acceleration/GNSS factors, E2 exact-Values
reuse, Qc `[.00375,.00375,.00375,.015,.015,.015]`, prior sigmas 1e-5/10/3,
stable-ID every-version replacement and the strict 1e-5 publication check.
Source-to-source CBS is not `live_revision`.

## Overall timing comparison

These are three independent live captures of the same requested sensor window.
They share settings, but not identical arrival interleaving, factor Values or
solver decisions. All-scan statistics use 391 matched scans. Times are host
elapsed milliseconds; p95 uses the retained linear-interpolation convention.

| Scope | Before LOCAL batching | Previous batched capture | Fresh B0 batched capture |
| --- | ---: | ---: | ---: |
| LOCAL median | 117.148 | 7.600 | 10.965 |
| LOCAL p95 | 218.181 | 14.391 | 15.385 |
| LOCAL maximum | 240.219 | 17.759 | 17.019 |
| Complete-scan median | 307.376 | 182.365 | 206.942 |
| Complete-scan p95 | 535.236 | 385.682 | 437.376 |
| Complete-scan maximum | 843.719 | 709.788 | 750.035 |
| Complete-scan mean | 254.138 | 162.292 | 183.525 |

The fresh LOCAL median is 90.64% below the unbatched reference and its p95 is
92.95% below it. Complete-scan median/p95 are 32.67%/18.28% lower than that
reference. The original LOCAL <=80%-of-unbatched gate passes. Complete-scan
p95 also passes the unchanged 800 ms headroom gate at 10 Hz sensors/0.10x playback.

Compared with the previous *batched* capture, B0's complete-scan median and p95
are about 13.5% and 13.4% higher. This is not a newly introduced code regression:
no estimator bytes/settings changed. It is also not proof that timing variation
is harmless at longer duration. B0 performed more factor evaluations; the
causes of the changed live solver history were not separately isolated here.
Desktop GPU load/clocks were not controlled.

### Where the time is now

| Profiled scope | Previous total s | B0 total s | B0 median / p95 / max ms |
| --- | ---: | ---: | ---: |
| MAIN update | 43.264 | 50.719 | 147.315 / 383.223 / 449.616 |
| LOCAL update | 2.856 | 3.415 | 10.901 / 15.382 / 17.019 |
| LOCAL covariance reconstruction | 4.023 | 4.164 | 12.066 / 14.210 / 21.244 |
| Revision-export scope | 8.514 | 8.346 | 10.755 / 81.484 / 153.407 |

Root-scope statistics include startup where present; that explains the small
LOCAL median difference from the matched-scan table. Nested timers are not
added together as independent costs.

| Factor-equation context | Previous synchronous / cached calls | B0 synchronous / cached calls |
| --- | ---: | ---: |
| MAIN | 162254 / 0 | 185378 / 0 |
| LOCAL update | 0 / 130972 | 0 / 155556 |
| LOCAL covariance reconstruction | 0 / 190426 | 0 / 196620 |

Every observed LOCAL update GPU linearization still consumed a prepared batch
result; none used synchronous per-factor fallback. MAIN synchronous calls
occupied 49.153 s of its 50.719 s root scope. MAIN call count increased 14.25%
from the previous batched capture; its mean synchronous-call cost increased
about 2.59%. This is consistent with most of that scope's time increase coming
from more calls, without establishing why its solver history changed.

Within the much smaller LOCAL update, graph linearization/batch preparation
accounts for 70.28% of exclusive time; measured iSAM marginalization is 0.86%.
The former per-factor synchronous bottleneck did not return.

### Early and late behavior

| B0 scope | Early 0–10 s median / p95 ms | Late 30–40 s median / p95 ms |
| --- | ---: | ---: |
| LOCAL | 2.085 / 9.144 | 14.029 / 15.839 |
| MAIN scan timer | 3.511 / 100.132 | 215.516 / 400.073 |
| Complete scan | 29.962 / 214.441 | 256.272 / 469.574 |

Windows follow the retained analyzer's first processed scan convention, not an
invented extra tail. The late complete-scan p95 also passes 800 ms. LOCAL active
keys have median 150 in both windows and maximum 153; late nonnull factors have
median 793 and factor slots median 825. A bounded state count alone does not
make MAIN's factor-evaluation cost negligible.

Controller wall times are 445.34 s unbatched, 446.40 s previous batched and
445.37 s B0. Playback pacing dominates completion time; lower compute cost
creates headroom rather than shortening the fixed-rate bag playback.

## Functional and transport validation

- 18 existing Release regressions passed. Native alignment, lifecycle,
  covariance diagnostics, coverage, conservation and strict stationarity passed.
  No new sanitizer or clean full-workspace build is claimed.
- 181 Kimera-to-GLIM and 180 GLIM-to-Kimera factors constructed. Wire and
  constructed covariances match exactly; causal endpoint cutoffs passed and
  pre-stage rejects were zero. Temporary lifecycles are GLIM 181 queued,
  156 removed, 25 remaining; Kimera 180 tracked, 156 removed, 24 remaining.
- Receiver consumed 21970 versions: 17720 GLIM and 4250 Kimera. Accounting is
  570 accepted stable IDs + 21375 replacements + 25 explicitly unsupported
  versions = 21970. Pending, duplicate and stale counts are zero.
- All 393 publications passed; maximum normalized gradient is
  `9.747059773858745e-6`. Maximum source-factor covariance numerical difference
  is `2.710505431213761e-20`.
- Final graph: 392 WNOA + 570 relative + 3 initial priors = 965 factors.
  Central IMU, bias and acceleration counts remain zero.
- Transport maximum queue 43, sampled controller queue maximum 12, final queue
  zero. Sampled output-age median/p95/max: 0.050007/0.090005/0.100005 s.
- The single saved-journal repeat reproduced all nine explicitly compared
  trajectory/state/clock/event files byte-for-byte; maximum X/V/W/orientation
  differences were zero. This does not claim separate source captures are
  deterministic or numerically identical.
- All B0-owned estimator, playback, master and receiver processes exited.
  The existing containers remain idle.

The late-capture process memory snapshot records resident memory of about
576 MiB for GLIM, 319 MiB for Kimera and 76 MiB for the receiver. The corresponding
reported process high-water marks at that observation were 576/352/76 MiB.
These are single observations, not final peak-memory certification; GPU memory
is not included in process RSS. See `memory_snapshot.json`.

## Output coverage

| Output | Rows | Timestamp span s | Path length m |
| --- | ---: | ---: | ---: |
| Original/native GLIM | 391 | 38.999880 | 14.027596 |
| Original/native Kimera ROS odometry | 184 | 39.599778 | 13.783451 |
| Fused online/as-played | 393 | 39.200000 | 15.353862 |
| Fused final retrospective | 393 | 39.200000 | 13.996308 |

CPT7 supplied 3986 timestamps spanning 39.850009 s. Startup offsets and tails
pass the retained coverage gates. Kimera retains its approximately 3.2 s startup
pose gap, then a maximum 0.200001 s gap. These are coverage/path diagnostics,
not GT accuracy results. No whole-window or partial-window ATE is claimed.

## Preservation and decision

All 680 new-run protected hashes passed. The inherited 677 protected hashes and
19 additional S1 source/build/library hashes also passed again after validation;
six frozen source repositories remain clean at their pinned HEADs. S1's CPU
library hash is a current-file pin, not retroactive historical binary proof.

**Accept B0 as the fresh unchanged first40s reference.** LOCAL batching's
benefit is retained, but MAIN still dominates. The proposed shared-equation
covariance backend cannot be assumed to remove MAIN's existing synchronous
cost or reproduce the large speedup already achieved. Its numerical and runtime
merits remain to be tested independently.

The next saved stage is S2: default-off Gaussian information helper and dense
correctness tests, then a separately gated shadow capture. S2 was not started.
No full160 run, new sequence, accuracy tuning, default promotion or push follows.

`reference_comparison.json` contains the three-way timing results;
`comparison.json` retains the original analyzer's unbatched comparison;
`evidence/` stores the small validator/harness records. Raw logs, journal,
trajectories and receiver repeat remain under the run root. The preparation,
preflight, validation and comparison scripts refuse existing output products.
Documentation and the runtime ledger are committed locally on the existing
planning branch; the final handoff identifies the resulting commit.
