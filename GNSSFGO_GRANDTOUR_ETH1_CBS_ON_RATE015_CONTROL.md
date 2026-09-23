# GrandTour ETH1 CBS-ON: controlled 0.15× source replay

Date: 2026-09-23. This is one source replay of the same 348 s ETH1 window used in the 2026-09-22 0.30× run, followed by an unchanged gnssFGO causal graph replay. No estimator, covariance, fusion, or evaluator code was changed.

## Why this run was necessary

The retained September 7 CBS-ON Kimera result (0.662930 m ATE, 202.722 m path) was real, but its 0.15× bag playback was not comparable to the recent 0.30× replay (5.949668 m ATE, 1,372.959 m path). The latter had only 257 GLIM-to-Kimera factor insertions and 919 logged inactive-window rejections. We therefore changed playback rate only, not the source profiles or receiver parameters, relative to the September 22 replay.

The current 0.15× and 0.30× runs use the same `experiment_runner_used.sh` SHA-256, `60f32130718df5fc13b30a1a0d4fb8dec5d1557335ab1bcbab9ceaa8ae6eace3`. A recursive comparison of the copied GLIM and Kimera profile directories found no differences. Both request a 348.0 s ETH1 window, bidirectional CBS, shared CPT7 IMU, 0.20 s CBS horizon, factor-graph strides of 5, and no presentation graph. The current run did **not** set `KIMERA_IMU_SAMPLE_SUPPORT` or `GLIM_IMU_SAMPLE_SUPPORT`; notably, Kimera used its default `current` interpolated-border packet, **not** the experimental `right` packet. gnssFGO's own IMU support remains `right_interval_end`. This distinction must not be hidden behind an `ALL_RIGHT` label.

The current source command in the existing `cbsms_ws_gpu` container was:

```bash
CBSMS_CBS_ENABLE=true CBSMS_CBS_DIRECTION=bidirectional \
CBSMS_PRESENTATION_ENABLE=false CBSMS_KIMERA_MODE=stereo_shared_cpt7 \
CBSMS_BAG_DURATION_SEC=348.0 CBSMS_BAG_RATE=0.15 \
CBSMS_GLIM_FACTOR_GRAPH_STRIDE=5 CBSMS_KIMERA_FACTOR_GRAPH_STRIDE=5 \
CBSMS_MAX_HORIZON_PAIRS_PER_UPDATE=1 CBSMS_ODOM_PUBLISH_STRIDE=1 \
CBSMS_CBS_BELIEF_RECEIVE_START_DELAY_SEC=0.0 \
CBSMS_RUN_ROOT=/workspace/cbs_eth1_cbs_on_rate015_20260923/sources \
CBSMS_RUN_NAME=current_full_cbs_on_rate015 \
CBSMS_RECORDING_ID=current_full_cbs_on_rate015 \
/workspace/cbs_gtsam4.3/src/cbsms/tools/run_grandtour_eth1_glim_kimera.sh
```

The source run has `complete.marker`; its saved parameters, bag, logs, and native trajectories are in `/home/yeranis/repos/V4RL/cbs_eth1_cbs_on_rate015_20260923/sources/current_full_cbs_on_rate015/`. The recent 0.30× reference is `/home/yeranis/repos/V4RL/cbs_eth1_cbs_on_full_20260922/sources/all_right_full_cbs_on/` (directory name notwithstanding, its Kimera selector was not RIGHT).

## Source health and belief exchange

| Observation | Current 0.15× | Recent 0.30× |
|---|---:|---:|
| GLIM output rows | 3,456 | 2,816 |
| Kimera output rows | 1,492 | 1,491 |
| GLIM→Kimera factors inserted | 1,472 | 257 |
| Kimera→GLIM factors inserted | 1,476 | 1,200 |
| Logged inactive-window rejections | 0 | 919 |
| Native Kimera ATE (m) | 0.481104 | 5.949668 |
| Native Kimera full output path (m) | 201.461 | 1,372.959 |

The 0.15× run's latest applied GLIM→Kimera factor reached target index 1480, only 11 behind the final Kimera index. Its largest consecutive native Kimera position step was 0.232 m; the 0.30× run's late maximum was 1,104.7 m. The present source run therefore avoids the observed catastrophic late jump. This is a single asynchronous replay, not proof that playback rate alone deterministically causes every difference.

The unchanged unique-edge exporter produced 3,454 GLIM and 1,473 Kimera beliefs in `/home/yeranis/repos/V4RL/cbs_eth1_cbs_on_rate015_20260923/beliefs/`. There were 18 duplicate Kimera publications removed by the frozen first-seen exact-edge deduplication policy. The graph accepted 1,472 Kimera beliefs and skipped one for missing support; that distinction is explicit below.

## Native source and gnssFGO results

The prism ATE scorer uses the AP20 prism lever arm and no-scale SE(3) Kabsch alignment. The 1 s relative-pose scorer uses the CPT7 Inertial Explorer reference. Native source rows and graph rows are different estimators and should not be conflated. In particular, the gnssFGO graph is a fusion framework, not GLIM's native trajectory. There are **no GNSS factors** in this experiment.

| Trajectory | ATE RMSE (m) | 1 s t-RPE RMSE (m) | 1 s r-RPE RMSE (deg) | Full output path (m) | Matched prism elapsed coverage (s) |
|---|---:|---:|---:|---:|---:|
| Native GLIM | 0.040709 | 0.030805 | 0.351193 | 205.682 | 226.600 |
| Native Kimera | 0.481104 | 0.035333 | 0.578477 | 201.461 | 192.699 |
| gnssFGO GLIM-only | 0.152739 | 0.022473 | 0.364253 | 202.833 | 226.600 |
| gnssFGO GLIM+Kimera | 0.170832 | 0.020765 | 0.363675 | 202.692 | 226.600 |

The two gnssFGO graph scores have identical prism matched coverage (2,276 poses over 226.600 s) and identical 1 s RPE support (3,411 pairs). On this common graph evaluation, adding Kimera improves t-RPE by 0.001708 m (7.6%) and r-RPE by 0.000578 deg, but worsens ATE by 0.018093 m (11.8%). It is therefore a **mixed**, not an overall improvement. Native GLIM's 0.0407 m ATE is markedly better than either graph trajectory; the source trajectory must remain a separate baseline.

The unchanged gnssFGO command used `grandtour_ct_replay` with `--timeline record --solver isam2`, the same raw IMU CSV and exported beliefs, `--mode glim` for the control and `--mode joint` for fusion. The joint graph completed with 3,454 GLIM and 1,472 Kimera factors, 3,456 support states, 201.459 m internal path, and 456.1 s graph wall time. The GLIM-only graph completed with 3,454 GLIM factors, 3,456 support states, 201.594 m internal path, and 100.1 s graph wall time. There was no GNSS, covariance rescaling, robust gate, or code modification. Both graphs use the same GLIM belief file and IMU CSV. Graph output is in `/home/yeranis/results/gnssfgo_grandtour_eth1_cbs_on_rate015_20260923/`; evaluator JSONs are in `/home/yeranis/repos/V4RL/cbs_eth1_cbs_on_rate015_20260923/metrics/`.

## Reproducibility checks

| Artifact | SHA-256 |
|---|---|
| Source runner copied into 0.15× run | `60f32130718df5fc13b30a1a0d4fb8dec5d1557335ab1bcbab9ceaa8ae6eace3` |
| Raw IMU CSV supplied to gnssFGO | `925d6cd8c10000434a9f92dd0ba4565798a5a2ef51cf249c400147a32a11a708` |
| GLIM belief CSV | `67ae437d670b5981c290e24439ebfda6e4bd3bab2773b904e9128aefc45a41ee` |
| Kimera belief CSV | `f0ab7a45e01b20f71f7c51a93cdccf833188bc966847205e4d1620d09a7ba2c4` |
| gnssFGO executable | `0a5247085378bc54434def0890d2e143f102dfc323c285de65a91fb3f78f35b8` |
| GLIM-only graph TUM | `6e78ed98f1270c0e7f78d6aaa34a267c74027797a9878ec0c160319ce3139a31` |
| Joint graph TUM | `0e85c710ad74433e73000a873d80d6833c721d1c93663c7a319210f180de170e` |

No source or fusion implementation was changed. This run addresses the earlier mistake of treating a 0.15× historical source trajectory as equivalent to a 0.30× current replay, and the separate mistake of calling the launcher `ALL_RIGHT` without checking its actual Kimera IMU selector. The old September 7 estimator executable was not retained, so this current controlled run cannot establish old-vs-new binary equivalence. It does show that, under the **current** executable/configuration, the slower playback yields much denser active CBS exchange and a healthy full native Kimera trajectory and completed graph, while fusion does not beat the GLIM-only graph on ATE.
