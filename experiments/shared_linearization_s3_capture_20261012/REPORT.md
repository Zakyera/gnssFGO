# Native Gaussian covariance reuse passes the ETH1 shadow test

S3's authorized first-40-second ETH1 shadow capture **passes**. MAIN's existing native
Gaussian equations supported a separate native-only marginal history through
313 retirement events. Twenty endpoint covariance checkpoints agree with an
independent dense calculation to near floating-point precision. The candidate
did not publish covariance or replace the existing independent LOCAL solver.

This validates the covariance construction at MAIN's recorded charts on this
window. It does not prove equality to independently optimized LOCAL covariance,
calibrated uncertainty, statistical independence from CBS, improved ATE, or a
speedup. Both covariance paths ran during the capture.

## Frozen run and scope

Exactly one source capture ran: ETH1 mission `2024-10-01-11-29-55`, bag offset 0,
duration 40 s, playback 0.10x. GLIM used the saved Hesai/CPT7 profile and 5 s lag;
Kimera used the saved front-stereo/shared-CPT7 25-state profile with its experimental
support override unset. Classic bidirectional `active_window_temporary` CBS,
native mean alignment, 0.20s horizon, one pair/update, dense
`schur_relative_between`, scale 1 and Gaussian loss were unchanged. E1b.4a LOCAL
GPU batching remained enabled. Source graph inspectors were off.

The concurrently running E2 receiver was unchanged: source-WNOA X/V/W,
independent 100 ms supports, full-history iSAM2 QR, stable-ID replacement,
Qc `[.00375,.00375,.00375,.015,.015,.015]`, existing priors, strict 1e-5 publication
gradient, no central IMU/bias/acceleration/GNSS factors. No tuning or GT scoring.

Capture root:
`/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_shared_linearization_s3_20261012`.
Private build/fixtures:
`/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/shared_linearization_s3_capture_20261012`.

The controller completed in 445.405 s. Resolved source parameters and Kimera flags
matched B0 apart from output paths; receiver configuration matched apart from
the session, port and byte-identical wrapper's location. Actual mapped libraries
were checked before playback. The first preflight caught that expected wrapper
path difference; the check was corrected, with both logs retained, before any
data played. There was no failed source attempt or retry.

## Recorder and independent oracle

The recorder extends the private S3 observer only. It saves pre-retirement native
weighted Gaussian blocks, ordered keys/dimensions, stable IDs, snapshot versions,
slots, complete pose/velocity/bias reference Values, explicit removals, retiring
keys and fixed separators. Separate successful-commit records distinguish staged
input from accepted history. A terminal failure would retain its pending input.
Unchanged blocks are elided using zero-tolerance numerical equality; written
matrix values retain binary double precision. This equality does not distinguish
positive and negative zero. No re-whitening, nonlinear evaluation, correspondence
search, extra anchor or diagonal loading is introduced.

The independent Python/SciPy oracle scatters those recorded native blocks and
maintains its **own** dense Schur prior. It eliminates retiring variables from
that prior plus incident native contributions only, retaining nonincident native
factors once. It never initializes from the candidate prior or MAIN's mixed
marginal information. Reference-chart continuity, dimensions, IDs, versions,
removals and contribution accounting are checked at every committed update.

At epochs 5, 45, ..., 765, the first eligible outgoing pair supplies the already
requested MAIN endpoint means. Its candidate graph is saved before the query,
followed by full 12x12 joint covariance, tangent transport, current-chart joint
covariance and full 6x6 relative covariance. The oracle compares complete
augmented information, including linear and constant terms, and endpoint cross
covariance. Independent analytic SE(3) transport and finite differences provide
additional checks. Source publication cadence and payloads remain authoritative
old-LOCAL outputs. The unchanged S2 backend and its rank rejection are retained.

## Real-data mathematical results

| Quantity | Maximum relative Frobenius difference | Predeclared gate |
| --- | ---: | ---: |
| Complete native augmented information | 3.8482e-17 | 1e-9 |
| Joint endpoint covariance at stored charts | 6.0111e-15 | 1e-8 |
| Analytic chart transport | 3.3088e-16 | 1e-8 |
| Joint covariance at current endpoint means | 6.0681e-15 | 1e-8 |
| Full relative covariance | 7.0594e-15 | 1e-8 |
| Independent finite-difference covariance propagation | 2.5442e-10 | 1e-6 |

Maximum absolute Frobenius differences were 5.8195e-7 for augmented information,
2.2862e-20 for joint covariance and 1.5922e-20 for relative covariance. Information
has large entries; its maximum entrywise difference was 4.7684e-7. Maximum sampled
native-information condition number was 9.8970e7, below the unchanged rejection
boundary. No jitter, rank-policy relaxation or error was needed.

All 783 staged updates committed. There were 313 retirement events. Native
contribution conservation was exactly **5321 admitted = 4513 absorbed + 808 active
+ 0 explicitly removed**. Explicit external CBS factors and MAIN marginal factors
were never positively admitted. Maximum pre-retirement native count was 824,
with 156 Values and scalar dimension 780; sampled post-retirement query dimension
reached 765. This is bounded-window evidence, not arbitrary-duration certification.

The 20 checkpoints cover startup, middle and late portions of the run. These are
sampled covariance checks, not covariance solves at every update; lifecycle and
chart accounting cover all 783 updates. `real_oracle.json`, `absolute_errors.json`
and the 52,118,518-byte native journal retain the numeric evidence.

## Functional validation and non-regression

- Release 44/44 twice; ASan/UBSan 44/44, no findings. Instrumentation covers the
  new observer/recorder/S2 and modified CPU units, not prebuilt CUDA/dependencies.
- Existing GPU/source regression 18/18. Disabled private versus original MAIN
  output: 659,210 bytes identical. Enabled recorder fixture adds no counted native
  evaluations and leaves MAIN states/charts/cached matrices unchanged.
- Recorded synthetic pose history and mixed IMU/velocity/bias history pass the
  independent oracle through 14 and 7 retirements. Five Python validator tests
  pass, including corrupted covariance/information, missing cross covariance and
  truncated journal rejection.
- No-data GLIM startup/shutdown passed with seven library paths checked, including
  the actual private CPU smoother library. Source settings/maps were checked again
  during the real launch before playback.
- Native mean alignment, source lifecycle, covariance diagnostics, temporal
  coverage, transport conservation and strict receiver publication checks pass.
  GLIM queued 181 external factors and removed 156; Kimera tracked 180 and removed 156.
- Receiver conservation: 21970 versions = 570 initial IDs + 21375 replacements
  + 25 explicitly unsupported startup versions. Queue ended at 0, maximum 47.
  All 393 publications were stationary; maximum normalized gradient 9.85976e-6.
- The one saved-journal receiver repeat produced nine byte-identical trajectory,
  state, support, model-covariance and event files. This is repeatability of the
  same recorded journal, not byte identity between separate live captures.
- All 716 run-protected files and 680 inherited baseline files checked unchanged;
  all 19 S1 pins and six pinned source HEAD/clean-worktree checks pass. S2 backend
  and existing 25 tests are byte-unchanged. Installed libraries/defaults untouched.

Native and fused outputs remain separate:

| Output | Rows | Timestamp span s | Path m |
| --- | ---: | ---: | ---: |
| Original GLIM odometry | 391 | 38.99988 | 13.96666 |
| Original Kimera ROS odometry | 184 | 39.59978 | 13.75979 |
| Fused online/as-played | 393 | 39.20000 | 15.32317 |
| Fused final retrospective | 393 | 39.20000 | 13.95815 |

The 40 s bag window contains 3986 IMU samples spanning 39.85001 s. Existing startup
coverage differences remain explicit: Kimera has a 3.20 s startup gap, then maximum
gap 0.20 s; GLIM maximum gap 0.10099 s. These are coverage/path checks, not ATE scores.

## Diagnostic cost and next boundary

The 20 covariance queries took 2.837 s total, median 159.656 ms, p95 171.204 ms and
maximum 174.710 ms. They include dense rank inspection and Gaussian covariance
work; avoiding LiDAR reevaluation does **not** make them cost-free. Recorder-stage
serialization/comparison/flush time was 0.308 s total. These timers do not isolate
all observer copying, validation and marginalization overhead.

Complete-scan median/p95 was 210.474/431.506 ms versus B0's 206.942/437.376 ms.
Old LOCAL still ran, with profiler median/p95 10.335/14.834 ms. Receiver output-age
median/p95/max was 0.0600/0.0900/0.1000 s. Separate live runs have different
interleavings, so these numbers establish operating headroom for this shadow
test, not a candidate speedup.

Stop at S3. S4 should first use the retained equations to measure candidate-only
Gaussian maintenance and required covariance-query cadence, before a separately
authorized covariance-publication run disables old nonlinear LOCAL. The sampled
159.7 ms query cost must not be extrapolated into a promised faster per-belief
provider. No S4 implementation, provider switch, new source run or push occurred.

MAIN-conditioned native Jacobians can still carry CBS's indirect influence
through means/correspondences. This construction removes explicit received
Gaussian information and avoids inheriting MAIN's mixed marginal prior; it does
not certify calibrated uncertainty or independent source information.

## Checkpoint

Parent: `1cf5a22750316695d28c7654668e17a7c5fd52af`. Sources, protocol, harness and
small validation evidence are saved in the commit containing this report on
`engineering/shared-native-gaussian-s3-20261012`. Large journal/raw run artifacts
remain local; the E1b.4a recovery tag and earlier checkpoints remain intact.

Release private CPU SHA256:
`65603577ccae502274a4edb4f2c215f2be24963dff02385767256293e0f1f1b6`.
Private bridge SHA256:
`80bbe5867efbc84eaf0faf13f4a63eb700bc1bd0e5e888e5dfac5b9f985b6c48`.
Receiver SHA256:
`9d2ef832941c5ff2592da8428cb80104352590bcfea50c187624aaf4e90d60e0`.

S3_REAL_DATA_SHADOW_VALID
