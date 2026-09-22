# Full GrandTour ETH1 gnssFGO run

Date: 2026-09-22

## Outcome

The complete ETH1 source-estimator pass succeeded, and the full GLIM-only
continuous-time graph succeeded. The full Kimera-only and GLIM+Kimera graphs
did **not** produce valid complete trajectories: both became numerically
unstable near 220 s mission time and then terminated with a GTSAM
`IndeterminantLinearSystemException` on an angular-rate state.

Consequently, no full-sequence fused ATE/RPE is reported. Reporting a metric
from the divergent prefix or silently truncating the sequence would be
scientifically invalid.

This full run changes the interpretation of the encouraging 40 s pilot. The
40 s result is reproducible, but it does not extend to the complete sequence
with the same frozen graph and source covariances.

## Frozen experiment

- Dataset: GrandTour mission `2024-10-01-11-29-55` (ETH1).
- Playback length: 348 s.
- GLIM: same profile as the validated 40 s ETH1 pilot, CBS off, CPT7
  `ALL_RIGHT` support.
- Kimera: same stereo/CPT7 profile as the validated 40 s ETH1 pilot, CBS off,
  CPT7 `ALL_RIGHT` support.
- Fusion: gnssFGO continuous WNOJ trajectory, `CombinedImuFactor`, native
  source relative beliefs, full dense source covariance, full iSAM2.
- Causal chronology: source availability at `record_timestamp_ns`; factor
  endpoints remain at the physical source timestamps.
- No covariance scaling, robust loss, gating, GNSS factor, or source-specific
  tuning was introduced.

Retained source files:

| Input | Rows | SHA-256 |
|---|---:|---|
| Native GLIM `odom_imu.txt` | 3,378 poses | `68a74a06dce13dc2f8ffb0f883b2a1ac4fae0d366f9c2afee5fa12addc3052f1` |
| Native Kimera `kimera_odometry.csv` | 1,493 poses | `f0976234f775abd8bdb4c4b6a4bbda12ab5f799979204c77cbf596184b98bffb` |
| GLIM relative beliefs | 3,319 beliefs | `65fb7979aca6cec7ebacf80bb84ccd90af716ae88b599a09f9b8f672551196b9` |
| Kimera relative beliefs | 1,474 beliefs | `dacf0f7eb1d0622d4c58b947122d1976079ba4d5de083fd93977cf51202e9e99` |
| CPT7 IMU CSV | 34,847 samples | `925d6cd8c10000434a9f92dd0ba4565798a5a2ef51cf249c400147a32a11a708` |

Artifacts are retained under:

```text
/home/yeranis/repos/V4RL/cbs_cpt7_all_right_full_eth1_20260922
/home/yeranis/results/gnssfgo_grandtour_eth1_full_20260922
```

## Runtime recovery and regression

The lost container build products were restored without changing estimator
mathematics:

- ROS 2 Humble container: `ubuntu-jammy-humble-nvidia`;
- exact RWTH GTSAM source commit:
  `b5a4359d5e6d40c71e26633c961a0fd99ed72f06`;
- NumCpp 2.16.1;
- ROS Humble `ublox_serialization` package;
- GTSAM and GTSAM_UNSTABLE installed from the same RWTH source tree.

The rebuilt runner was first replayed on the frozen 40 s joint case. Its
trajectory SHA-256 is still exactly:

```text
86d3470bf66de818c90186f65136225a8e3ae7d91a8c85411afae6b7feaf3482
```

It is byte-identical to the previously validated causal 40 s result.

## Causal support availability

On the full sequence, 29 Kimera beliefs become available about 78--85 ms
before the future GLIM support state needed to interpolate their endpoint has
been created. The old runner treated this valid condition as a fatal input
error.

The runner now retains such an already-arrived belief unchanged and inserts
it at the earliest support timestamp at which every referenced interpolation
key exists. Measurement time, mean, covariance, and source record time are
unchanged. This code path is inactive in the 40 s pilot, as proven by the
byte-identical regression above.

The causal full run processed eight of these deferred beliefs before the
subsequent numerical failure. The first was Kimera ordinal 1058:

```text
physical endpoint:       1727775223454172600 ns
record/arrival time:     1727775223491426876 ns
right support available: 1727775223573866000 ns
additional support wait: 82.439124 ms
```

## Full graph results

| Graph | Timeline | Accepted before termination | Support states written | Result |
|---|---|---:|---:|---|
| GLIM only | physical | 3,319 GLIM | 3,358 | Completed; 201.467 m internal path |
| Kimera only | physical | 1,114 Kimera | 2,243 | Diverged; indeterminate `w2241` |
| GLIM + Kimera | physical | 2,190 GLIM + 1,088 Kimera | 2,195 | Diverged; indeterminate `w2193` |
| GLIM + Kimera | causal record time | 2,099 GLIM + 1,083 Kimera | 2,189 | Diverged; indeterminate `w2186` |

The causal joint graph first exceeds 10 m/s estimated speed at support 2179
(`1727775227574373000 ns`) and exceeds 100 m/s at support 2181. The physical
joint graph shows the same failure family, exceeding 10 m/s at support 2172
and 100 m/s at support 2173. Kimera-only fails slightly later, exceeding
10 m/s at support 2222 and 100 m/s at support 2230. GLIM-only never exceeds
10 m/s and completes the full support grid.

Because physical-time joint processing also fails, the instability is not
caused by the causal pending-support queue. Because Kimera-only fails while
GLIM-only completes, the failure enters with the long Kimera factor stream;
it is not a generic full-duration IMU/support-graph failure.

![Full graph stability](/home/yeranis/results/gnssfgo_grandtour_eth1_full_20260922/graph_stability_speed.png)

## Native-source and GLIM-graph metrics

The existing GrandTour evaluators were used unchanged:

- ATE: AP20 Leica prism with the calibrated CPT7-to-prism lever arm and an
  independent no-scale SE(3) Kabsch alignment;
- RPE: 1 s SE(3) relative error against CPT7 Inertial Explorer tightly coupled
  odometry.

The full available output window gives:

| Valid trajectory | ATE RMSE | 1 s t-RPE | 1 s r-RPE | Evaluated path | RPE pairs |
|---|---:|---:|---:|---:|---:|
| Native GLIM | **0.028346 m** | 0.018774 m | **0.289044 deg** | 202.941 m | 3,411 |
| Native Kimera | 8.595737 m | 0.200726 m | 0.528511 deg | 175.863 m | 2,948 |
| gnssFGO GLIM-only control | 0.119855 m | **0.016562 m** | 0.313075 deg | 202.650 m | 3,411 |
| gnssFGO GLIM+Kimera | invalid | invalid | invalid | invalid | invalid |

ATE support is not continuous over the whole mission because the AP20 prism
has gaps. Native GLIM and the GLIM-only graph each have about 226.5 s of
matched prism coverage. Kimera has about 192.8 s.

For a strict same-timestamp comparison, the intersection of the longest
continuous segment of all valid outputs is:

```text
1727775011.575121 to 1727775211.474488
duration: 199.899367 s
grid: 2,000 native GLIM timestamps
```

| Valid trajectory | ATE RMSE | 1 s t-RPE | 1 s r-RPE | Common path |
|---|---:|---:|---:|---:|
| Native GLIM | **0.027943 m** | 0.020734 m | **0.262601 deg** | 147.779 m |
| Native Kimera | 7.388433 m | 0.236024 m | 0.503958 deg | 123.034 m |
| gnssFGO GLIM-only control | 0.094938 m | **0.018885 m** | 0.293636 deg | 147.771 m |

The common-interval trajectory shape makes the long-horizon Kimera drift
visible. GLIM and the GLIM-only graph remain nearly coincident in shape;
Kimera shortens and bends the route substantially.

![Common trajectory comparison](/home/yeranis/results/gnssfgo_grandtour_eth1_full_20260922/native_and_glim_graph_common_xy.png)

## Kimera availability and covariance observations

Kimera does not provide one continuous full-length trajectory. Its longest
continuous segment is 199.999 s, followed by an 85.800 s segment and several
short or isolated late segments. The retained output contains 18 gaps larger
than 0.25 s, including nine gaps larger than 1 s.

Its relative covariances remain highly confident despite the large
long-horizon source error. Across 1,474 Kimera beliefs:

- median rotational covariance trace: `6.2372e-08 rad^2`;
- median translational covariance trace: `1.2736e-05 m^2`;
- median minimum eigenvalue: `7.1843e-09`;
- median maximum eigenvalue: `7.9338e-06`.

These statistics do not by themselves prove that covariance is the sole root
cause, but they explain why the continuous graph gives Kimera substantial
authority even after the native trajectory has developed metres of drift.
No covariance was changed in this run.

## Validation

- Release runner build: pass.
- Focused CT factor and ALL_RIGHT tests: 2/2 pass.
- Frozen 40 s joint trajectory regression: byte-identical pass.
- Full GLIM-only graph: complete and bounded.
- Repeated full GLIM-only graph: trajectory, state, residual, event, support,
  and online-output files are byte-identical (the JSON summary differs only
  in measured wall-clock runtime).
- Full causal and physical joint graphs: same failure family near 220 s; no
  fused metric accepted.
- Focused `cpplint` and `uncrustify` on the modified runner: pass.
- `git diff --check`: pass.
- Package CTest: 5/10 pass. The two scientific tests plus `cppcheck`, `pep257`,
  and `xmllint` pass. The five failures are the repository's existing broad
  copyright/style debt (`copyright`, `cpplint`, `flake8`, `lint_cmake`, and
  `uncrustify`), including legacy and generated sources.

## Conclusion

The full ETH1 experiment is **not a successful full-sequence fusion result**.
It establishes three concrete facts:

1. the native full ETH1 GLIM result is very accurate;
2. the native Kimera result degrades severely over the longer sequence and is
   discontinuous late in the mission;
3. adding the unchanged Kimera relative factors makes the current continuous
   graph numerically unstable around 220 s, both in physical-time and causal
   processing, whereas GLIM-only completes.

The next investigation should isolate the first Kimera-driven loss of graph
consistency around mission time 210--225 s before running more full fusion
experiments. This report does not tune or change the fusion model.
