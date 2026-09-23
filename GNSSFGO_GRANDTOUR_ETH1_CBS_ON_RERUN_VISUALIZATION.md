# ETH1 CBS-ON gnssFGO Rerun presentation

Date: 2026-09-23. This is a **new matched visualization replay**, not the headless 0.15× control documented in `GNSSFGO_GRANDTOUR_ETH1_CBS_ON_RATE015_CONTROL.md`. The earlier bag did not record GLIM/Kimera factor-graph inspector streams, so it could not faithfully show the actual injected source factors. No estimator mathematics was changed.

## Open the presentation

The corrected point-cloud presentation recording is:

`/home/yeranis/repos/V4RL/cbs_eth1_cbs_on_rerun_20260923/GrandTour_ETH1_CBS_ON_GNSSFGO_POINTCLOUD_DASHBOARD.rrd`

The prior four-trajectory recording remains preserved at:

`/home/yeranis/repos/V4RL/cbs_eth1_cbs_on_rerun_20260923/GrandTour_ETH1_CBS_ON_GNSSFGO_4TRAJ_GRAPHS.rrd`

It was opened in the native Rerun viewer with:

```bash
rerun --new --detach-process --memory-limit 8GB /home/yeranis/repos/V4RL/cbs_eth1_cbs_on_rerun_20260923/GrandTour_ETH1_CBS_ON_GNSSFGO_POINTCLOUD_DASHBOARD.rrd
```

The dashboard has three upper views: GLIM's live factor graph, Kimera's live factor graph, and a **top-right point-cloud view with all four overlaid trajectories**. The lower row shows the active injected Kimera→GLIM and GLIM→Kimera factor counts and the run/metric notes. The timeline is shared. In the trajectory view: blue is Leica prism ground truth, green native GLIM, orange native Kimera, and red the causal gnssFGO joint graph. The factor-graph panels retain their own coordinate frames; they are **not** overlaid onto the independently aligned prism trajectories.

The gray point cloud is a display-thinned **static final Hesai/GLIM map from the exact same source replay**, not a live scan. The helper reads 150 saved GLIM submaps (7,498,652 native points), displays 374,933 deterministic samples, transforms each submap from its local origin through `T_world_origin`, then applies the exact no-scale rigid GLIM-to-prism scorer alignment and the same visualization origin as the four paths. Across all submaps, `T_world_lidar * T_lidar_imu` at the first saved frame agrees with the corresponding GLIM IMU-odometry position to within 0.000483 m, confirming the map and GLIM path use the same source world. This is a visualization transform only; none of the estimators was rerun or changed.

The corrected file uses a distinct Rerun application/recording identity and repackages the source data without its older saved dashboard blueprint. This prevents the older CBS tab/layout from superseding the corrected top-right view. The saved factor-graph data and both external CBS factor streams remain present. The viewer was opened in a separate native Rerun window, and a [desktop capture](/home/yeranis/repos/V4RL/cbs_eth1_cbs_on_rerun_20260923/pointcloud_dashboard_verified.png) confirmed that the new upper-right view shows the cloud and all four paths alongside the two factor graphs. At the very beginning of playback, the graph inspectors can appear empty until their first recorded events; scrub later in the shared timeline to inspect injected factors.

The entire available estimator trajectories are displayed, not merely the samples used by the prism scorer. Leica gaps are separated rather than drawn as fictitious continuous paths. The data was transformed to the physical prism point and rigidly aligned using each estimator's saved no-scale Kabsch transform. The script asserts against each evaluator's aligned CSV. The red gnssFGO trajectory was calculated **offline from the saved beliefs of this same Rerun source run**, not live inside GLIM or Kimera. The old presentation launcher also ran an EKF sidecar; it is not the red path and is not used in the gnssFGO graph.

## Matched run provenance

The existing `run_grandtour_eth1_presentation_host.sh` launcher recorded a 348 s ETH1 bag at 0.15×, with Hesai GLIM, front-stereo Kimera, shared CPT7, bidirectional CBS, 0.20 s belief horizon, factor-graph inspector strides 5/5, and the established source profiles. Run directory:

`/home/yeranis/repos/V4RL/cbs_eth1_cbs_on_rerun_20260923/sources/eth1_cbs_on_rerun_rate015/`

Its raw and compacted RRDs, ROS bag, source trajectories, logs, resolved parameters, and evaluator outputs are retained there. The host launcher verified the source RRD before this overlay was made. The source yielded 3,456 GLIM poses, 1,493 Kimera poses, 1,473 inserted GLIM→Kimera factors, 1,477 Kimera→GLIM factors, and zero logged inactive-window rejections. Both external spatial factor classes were verified in the saved RRD as `LineStrips3D`, rather than inferred solely from message counts:

- `/glim/factor_graph_inspector/spatial/factors/cbs_k2g_pose_between`
- `/kimera/factor_graph_inspector/spatial/factors/cbs_g2k_pose_between`

The unchanged unique-edge exporter produced 3,454 GLIM and 1,474 Kimera belief rows in `/home/yeranis/repos/V4RL/cbs_eth1_cbs_on_rerun_20260923/beliefs/`. The unchanged causal `grandtour_ct_replay --mode joint --timeline record --solver isam2` used those exact CSVs and the same raw IMU CSV as the headless control. It accepted 3,454 GLIM and 1,473 Kimera factors, skipped one Kimera belief without support, and completed all 3,456 graph support states. No GNSS factors were used. The graph output is in `/home/yeranis/results/gnssfgo_grandtour_eth1_cbs_on_rerun_20260923/joint/`.

## Scores belonging to this presentation run only

The prism ATE uses the AP20 lever arm and no-scale Kabsch alignment; 1 s RPE uses the CPT7 Inertial Explorer reference. Matched prism elapsed coverage differs for Kimera, so do not interpret its ATE as exactly the same temporal sample set as the other two rows.

| Output | ATE RMSE (m) | 1 s t-RPE (m) | 1 s r-RPE (deg) | Full path (m) | Matched prism elapsed coverage (s) |
|---|---:|---:|---:|---:|---:|
| Native GLIM | 0.047210 | 0.030556 | 0.350927 | 205.711 | 226.600 |
| Native Kimera | 1.594751 | 0.037618 | 0.616363 | 201.421 | 192.899 |
| gnssFGO GLIM+Kimera | 0.132461 | 0.019774 | 0.363695 | 202.482 | 226.600 |

The red graph path is physically bounded, but its ATE is worse than native GLIM in this matched replay. This recording's Kimera ATE (1.595 m) also differs materially from the **separate headless 0.15×** run (0.481 m) despite similar source settings. Enabling the live inspectors and old EKF sidecar changes load; asynchronous source processing is not proven repeatable. These two runs must not be silently combined or used as identical controls.

## Validation

The combined RRD passed `rerun rrd verify`. Inspection of the recording found all four trajectory entities, the static point cloud, and both spatial CBS factor-line entities. The corrected recording's validation JSON is next to the corrected RRD, named `GrandTour_ETH1_CBS_ON_GNSSFGO_POINTCLOUD_DASHBOARD_validation.json`; its maximum full-path versus scorer-CSV positional discrepancy is below `7.1e-11` m for all three estimators. The corrected RRD SHA-256 is `e46f6e4c52072f383af118cf247788106f3009b03f3956c386ca349c628fa5d9`. The original source recording SHA-256 remains `f1934b83f75534e26a9e204cffa4e240d1a12520c08e1c731a5a945cb09a4d80`, and the original four-trajectory RRD remains unchanged with SHA-256 `15ee0c56278a894f5600975949bb8d44bc9a9d23ac200f1b893a78f3e43b3f90`.

The standalone Rerun 0.31.4 `--screenshot-to` attempt raised an internal viewer panic (`Failed to take store hub from the Viewer`), so no automated screenshot is claimed. The ordinary native viewer launch stayed active, and the on-disk RRD plus its entities were verified. The helper `/home/yeranis/repos/gnssFGO/tools/grandtour_eth1_gnssfgo_rerun.py` changes no estimator or factor; it only packages the matched trajectories and saved live graph recording for inspection.
