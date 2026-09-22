# GrandTour ETH1 native-source versus gnssFGO fusion comparison

Date: 2026-09-22

## Corrected result

The correct three-way comparison is the native GLIM trajectory, the native
Kimera trajectory, and the joint GLIM+Kimera trajectory produced by gnssFGO.
The earlier `IMU + GLIM` and `IMU + Kimera` rows were useful factor-graph
controls, but they were not the estimators' native trajectories and must not
be presented as such.

On the largest continuous interval shared by all three outputs, native Kimera
has the lowest position ATE and translational RPE. Native GLIM has better
rotational RPE than native Kimera. The joint gnssFGO result lies between the
two sources: it improves GLIM's ATE and translational RPE, while slightly
worsening GLIM's rotational RPE. It does not outperform native Kimera in
position on this sequence.

## Inputs and estimator roles

All inputs come from the same ETH1 ALL_RIGHT 40-second run of GrandTour
mission `2024-10-01-11-29-55`.

| Result | Role | Retained input | SHA-256 |
|---|---|---|---|
| Native GLIM | LiDAR-inertial source estimator output | `/home/yeranis/repos/V4RL/cbs_cpt7_common_convention_40s_pilot_20260921/sources/all_right/odom_imu.txt` | `3439b749734fa6966a64556e34e7e9e94506572cda8e3ea0178cd105f3cb7bbb` |
| Native Kimera | Visual-inertial source estimator output | `/home/yeranis/repos/V4RL/cbs_cpt7_common_convention_40s_pilot_20260921/sources/all_right/kimera_odometry.csv` | `6e93ef08e7638d9549bc0bc1d6f44111c63dd5e2f743feff5875fb533429ad36` |
| gnssFGO joint | Replacement fusion layer using GLIM and Kimera relative beliefs | `/home/yeranis/results/gnssfgo_grandtour_eth1_ct_causal_20260922/record/joint/trajectory.tum` | `86d3470bf66de818c90186f65136225a8e3ae7d91a8c85411afae6b7feaf3482` |

No estimator was rerun or changed for this comparison. The gnssFGO result is
the final full-iSAM2 trajectory after every causally arriving factor has been
incorporated. Both native source files report the common child/body frame
`cpt7_imu`; their arbitrary global `map` and `odom` frames are removed by the
same no-scale SE(3) alignment used throughout the GrandTour work.

## Common evaluation support

The phrase "40-second run" describes the requested sensor-playback window. It
does not mean that every estimator published a continuous 40 seconds of pose
output:

| Output | Poses | Timestamp span | Continuous-coverage observation |
|---|---:|---:|---|
| Native GLIM | 391 | 38.999880 s | One continuous segment |
| Native Kimera | 184 | 39.599778 s | Two initial poses, then a 3.199979 s startup gap, then one 36.199800 s segment |
| gnssFGO joint | 391 | 38.999880 s | One continuous segment on the GLIM support grid |

The primary comparison does not interpolate across Kimera's startup gap. It
uses the intersection of the longest continuous segment of every output:

```text
start:    1727775011.575120926 s
end:      1727775047.674922943 s
duration: 36.099802017 s
grid:     362 native GLIM timestamps
```

Kimera is linearly interpolated in position and slerped in orientation only
between its normal adjacent output poses. The largest distance from an
evaluation timestamp to a native Kimera pose is 82.679 ms, and the maximum
allowed source interval is 250 ms. GLIM and gnssFGO are already on this grid.
The resulting three TUM files contain byte-identical timestamp columns.

## Ground-truth support and metrics

The same existing evaluators and conventions are used:

- position ATE: AP20 Leica prism, CPT7-to-prism lever arm, independent SE(3)
  Kabsch alignment for each trajectory, no scale;
- 1-second relative pose error: CPT7 Inertial Explorer tightly coupled
  odometry, 0.1-second sampling, no global alignment.

The AP20 prism is not present for the complete trajectory. On the common grid,
all three estimates have exactly the same 113 matched pose timestamps, one
continuous 11.200178-second prism segment. The RPE evaluation spans the entire
36.099802-second common continuous interval and has 351 valid pairs for every
result.

| Trajectory | ATE RMSE (prism) | t-RPE RMSE, 1 s | r-RPE RMSE, 1 s | Common-interval path | Prism poses | RPE pairs |
|---|---:|---:|---:|---:|---:|---:|
| Native GLIM | 0.058035 m | 0.037273 m | **0.512250 deg** | 14.071497 m | 113 | 351 |
| Native Kimera | **0.008485 m** | **0.011850 m** | 0.585292 deg | 13.391822 m | 113 | 351 |
| gnssFGO GLIM+Kimera | 0.044028 m | 0.021742 m | 0.523496 deg | 13.752039 m | 113 | 351 |

The path values above are measured after applying the physical CPT7-to-prism
lever arm, as reported by the prism evaluator. They do not use a scale
correction.

## Interpretation

Relative to native GLIM, gnssFGO fusion:

- reduces ATE by 24.1%;
- reduces translational RPE by 41.7%;
- increases rotational RPE by 2.2%.

Relative to native Kimera, gnssFGO fusion:

- has 5.19 times the ATE;
- has 83.5% higher translational RPE;
- has 10.6% lower rotational RPE.

Therefore the surprising Kimera result is not solely an artifact of the
gnssFGO `IMU + Kimera` control. The native Kimera trajectory itself is very
accurate in position on this interval. However, the earlier graph-only Kimera
control must still not be substituted for native Kimera: its IMU/WNOJ graph
optimization materially changes the result, especially rotation.

This 40-second sequence does not show the current joint formulation beating
the best native source. It shows complementary but incomplete fusion: Kimera
pulls GLIM toward better translation, while GLIM contributes better native
short-horizon rotation than Kimera. The fused solution compromises between
the two rather than dominating both.

The prism ATE is the more independent position check but covers only 11.2
seconds. The full common-window RPE uses the CPT7 IE-TC solution, which shares
the physical IMU and should not be described as fully independent ground
truth.

## Why the earlier numbers differed

The unrestricted native reports scored trajectories at their own publication
timestamps. They consequently used different prism samples and durations:
native GLIM had 142 matched poses over 14.100 s, while native Kimera had 59
matched poses over 11.400 s. The corrected primary table resamples all three
trajectories onto the same 362 timestamps and gives all three exactly the same
113 prism observations. This changes the absolute numbers slightly and makes
the comparison valid.

The previously reported gnssFGO rows named `IMU + GLIM` and `IMU + Kimera`
remain numerical controls for the graph implementation. They answer what the
central inertial/WNOJ graph does with one belief source. They do not answer how
the original GLIM and Kimera systems performed by themselves.

## Reproducibility and artifacts

Common-grid construction is implemented in:

```text
online_fgo/scripts/prepare_grandtour_common_trajectory.py
```

The complete evaluation artifacts are retained at:

```text
/home/yeranis/results/gnssfgo_grandtour_eth1_native_vs_fused_20260922/
```

Important files are:

- `common_grid_summary.json`;
- `native_glim_common.tum`;
- `native_kimera_common.tum`;
- `fused_gnssfgo_common.tum`;
- `*_prism.json` and `*_prism_aligned.csv`;
- `*_rpe_direct.json` and `*_rpe_pairs.csv`.

The common-grid trajectory hashes are:

| Trajectory | SHA-256 |
|---|---|
| Native GLIM | `be0f1904f413b13a886b1d1705868cfb0acd347282e3d07298a57b5d78075179` |
| Native Kimera | `8beb10d8fe9cfdab660336d9b21bd1f45a5865589ba3838306d67c3a0fa1d13b` |
| gnssFGO joint | `68b424c0912574bdabaf0ccca1294d29bd559cf80ac51381e5013d013b730146` |

The direct native-file RPE check and the common-grid RPE agree: exactly for
GLIM, within 0.0016 mm and 0.00038 deg for Kimera, and below numerical output
precision for gnssFGO. The table uses the direct native-file RPE values to
avoid double interpolation.

## Conclusion

For this ALL_RIGHT ETH1 pilot, the scientifically correct comparison is:

```text
native GLIM
native Kimera
gnssFGO(GLIM relative beliefs + Kimera relative beliefs + IMU)
```

gnssFGO is the fusion replacement for the EKF. Its one-source graph runs are
controls, not source-estimator baselines.
