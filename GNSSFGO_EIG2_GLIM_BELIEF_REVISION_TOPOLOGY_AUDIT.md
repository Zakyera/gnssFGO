# EIG-2 GLIM belief revision and topology audit

Date: 2026-09-23. This is a **GLIM source-side, CBS-OFF diagnostic**. No Kimera, gnssFGO, GNSS, GP, receiver IMU, covariance tuning, or fusion update was run or changed. It follows [the single-source preservation audit](GNSSFGO_EIG2_SINGLE_SOURCE_PRESERVATION_AUDIT.md), which found that the first large full-sequence loss occurs in the exported GLIM belief product.

## Result

**Endpoint revision is the dominant measured source-side mechanism in this bounded window.** The 0.20 s publication edges cannot preserve even GLIM's *causal online* path because a shared endpoint has a different smoother estimate at the publication of its next edge. Matured edges from the same GLIM run reconstruct its shutdown path to within sub-millimetre output differences. Connecting the two 0.20 s chains improves robustness and one publication-time ATE, but does **not** remove the publication-time error. This is a limitation of a fixed, first-publication product of an evolving smoother, **not proof of a GLIM estimator bug** or a decomposition of the entire 305 s EIG-2 run.

For successive exported edges, the intervening source pose generally differs:

\[
Z^{\rm pub}_{ij}=(T_i^{(j)})^{-1}T_j^{(j)},\qquad
Z^{\rm pub}_{jk}=(T_j^{(k)})^{-1}T_k^{(k)}.
\]

Since \(T_j^{(j)}\ne T_j^{(k)}\), composition does not telescope to the online or finalized source trajectory. On the actual 0.20 s chains, that shared-pose change has **4.09 mm median / 6.32 mm RMS** translation and **0.0349° median / 0.0585° RMS** rotation over 588 internal joins. Replacing the publication edges with their finalized means eliminates the large reconstruction drift in this window.

## Bounded run and exact diagnostic path

The primary run is GrandTour EIG-2, mission `2024-11-03-13-59-54`, Hesai `/boxi/hesai/points` plus CPT7 `/boxi/cpt7/imu`, starting 5.7 s into the original bags for a requested 65.0 s at 0.10× playback. It produced **643 poses** and **641 unique 0.20 s GLIM beliefs**; zero Kimera processes and zero external CBS injections. The retained source run's GLIM configuration was reused. The resolved GLIM parameters differ from that run only in the endpoint-audit flag, output-dump path, and disabled Rerun graph visualization; in particular the frozen `glim_cbs_odom_max_horizon_pairs_per_update: 1`, 0.20 s horizon, direct marginal source, `schur_relative_between` covariance mode, `observe_only` CBS mode, and receiver-disabled setting are unchanged. The 5.0 s fixed lag comes from the retained `config_odometry_gpu.json`.

Run: [normal diagnostic directory](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_endpoint_revision_normal_20260923). Exact commands/configuration: [runner](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_endpoint_revision_audit_20260923/run_glim_only.sh), [resolved parameters](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_endpoint_revision_normal_20260923/resolved_glim_params.yaml). The GLIM build used `source /opt/ros/noetic/setup.bash && cd /workspace/cbs_gtsam4.3 && catkin build glim_ros -j8` and succeeded. Launch set `kimera_enable:=false`, `enable_ekf:=false`, `cbs_enable:=false`, `glim_cbs_mode:=observe_only`, and `glim_export_endpoint_audit_enable:=true`.

An initial **preflight** accidentally requested 25 horizon pairs per update rather than the retained value 1. It was stopped, kept separately at `_eig2_glim_endpoint_revision_audit_20260923`, and was used only to test parsing/evaluation. **No preflight score appears in the results below.** The primary normal run was started cleanly after restoring the frozen value 1.

`GlimCbsBridge::onSmootherUpdateFinish()` calls `publishOdometryAndRerun()` then `publishOutgoingBeliefs()` on the same smoother. The pre-existing `GLIM_CBS_EXPORT_ENDPOINT_ROW` records both `calculateEstimate<Pose3>` endpoint means in `getOdometryBeliefsDirectForPairs()`. The only source change here adds two **flag-gated diagnostic rows** in [cbs_bridge.cpp](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/glim_ros1/src/glim_ros/cbs_bridge.cpp): `GLIM_CBS_EXPORT_BELIEF_ROW` records source, stable `(from,to)` ID, revision 0, publication/physical stamps, exact `[phi,rho]` mean and dense 6×6 published covariance; `GLIM_CBS_ACTIVE_POSE_ROW` records every active pose and smoother update until retirement. Neither outgoing belief nor smoother state is modified. The flag remains false by default.

The unchanged [CBS exporter](/home/yeranis/repos/ethzasl_msf/msf_core/tools/export_cbs_unique_relative_beliefs.py) was applied to the saved [ROS bag](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_endpoint_revision_normal_20260923/recordings/cbs.bag). **All 641/641** bag-exported first-publication means and all 36 covariance entries matched the diagnostic publication log at `1e-12` numeric tolerance. The full [belief CSV](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_endpoint_revision_normal_20260923/beliefs.csv) includes record and physical endpoint timestamps. There were zero duplicate selected publications. Stable identity is `(source='g', from_index, to_index)`.

## Comparison population and the three edge products

The final 51 poses were still inside the 5 s active window when playback ended. They are **right-censored**, not falsely called finalized. The common analysis therefore uses the first **592 fully retired poses** (59.100 s), yielding **590** current 0.20 s edges. The AP20 prism reference matches all 592 common-window poses. Every case in the table below comes from this *one* primary GLIM execution.

- **A, publication:** the exact first-published `Z=(T_i^pub)^-1 T_j^pub` and its retained full covariance. Each of these 590 edges was published once under the normal one-pair setting.
- **B, finalized:** `Z=(T_i^final)^-1 T_j^final` after both source poses leave the fixed-lag window. This is offline and noncausal, solely a preservation control. The last active pose differs from GLIM's [shutdown `odom_imu.txt`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_endpoint_revision_normal_20260923/dump/odom_imu.txt) by only **0.0456 mm RMS** over the 592 common poses.
- **C, revision-aware diagnostic:** revision 0 is A; at each subsequent smoother update, a new mean version is recorded whenever either endpoint changes, carrying the last value of a retired endpoint forward. The [revision CSV](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_endpoint_revision_normal_20260923/analysis/revision_aware_edges.csv) contains **29,740 versions**, median **50 versions/edge**, with stable ID and increasing revision. Its final version equals B within a `1e-10` right-local SE(3) check. This is a **mean-only diagnostic**; later revision covariances were not recomputed and no protocol/receiver consumed these versions. The explicit [last-version edge product](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_endpoint_revision_normal_20260923/analysis/t1_revision_last_edges.csv) scores identically to B.

The individual [publication edge table](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_endpoint_revision_normal_20260923/analysis/publication_edges.csv) retains source ID/index, physical and record timestamps, age/lifetime, publication covariance token, and endpoint/relative revision magnitudes. The original publisher does not itself implement a revised-belief protocol.

## How much the endpoints and edges revise

All **590/590** studied relative edges changed by more than 1 µm before both endpoints retired. Their publication-to-final error is
\(e_{ij}=\Log[(Z_{ij}^{pub})^{-1}Z_{ij}^{final}]\), in GLIM/GTSAM's right-local `[phi,rho]` order.

| Quantity, 590 edges unless noted | Median | RMS | p95 | Maximum |
|---|---:|---:|---:|---:|
| From endpoint translation revision (m) | 0.002671 | 0.005798 | 0.013469 | 0.023633 |
| To endpoint translation revision (m) | 0.005834 | 0.009708 | 0.020535 | 0.038982 |
| Relative-edge translation revision (m) | **0.004072** | **0.006534** | **0.013230** | **0.024238** |
| From endpoint rotation revision (deg) | 0.021337 | 0.068417 | 0.163531 | 0.300628 |
| To endpoint rotation revision (deg) | 0.047785 | 0.111707 | 0.279736 | 0.409118 |
| Relative-edge rotation revision (deg) | **0.031271** | **0.056483** | **0.119222** | **0.230592** |

Publication happens while endpoints are **far from** the 5 s fixed-lag boundary: the physical start endpoint has ~4.800 s lag margin, the end endpoint ~5.000 s. End/old-start age **at record availability** is median 0.120/0.320 s. The old/new endpoints remain active for median 4.744/4.944 s after publication and receive median 47/49 subsequent smoother updates. These narrow age/lifetime ranges mean the correlations below are **descriptive**, not a causal regression.

| End-endpoint age at publication (s), quartiles | Translation revision median / RMS / p95 / max (m) | Rotation revision median / RMS / p95 / max (deg) |
|---|---|---|
| 0.100–0.110 | .00366 / .00531 / .00961 / .01811 | .0260 / .0424 / .0825 / .1608 |
| 0.110–0.120 | .00359 / .00491 / .00969 / .01592 | .0255 / .0401 / .0790 / .1639 |
| 0.120–0.130 | .00459 / .00698 / .01315 / .02424 | .0310 / .0617 / .1482 / .2306 |
| 0.130–0.157 | .00577 / .00834 / .01750 / .02316 | .0489 / .0745 / .1595 / .1912 |

| End-endpoint time to retirement (s), quartiles | Translation revision median / RMS / p95 / max (m) | Rotation revision median / RMS / p95 / max (deg) |
|---|---|---|
| 4.862–4.903 | .00463 / .00738 / .01506 / .02379 | .0344 / .0584 / .1195 / .1965 |
| 4.903–4.944 | .00400 / .00689 / .01497 / .02424 | .0266 / .0583 / .1405 / .2306 |
| 4.944–4.993 | .00432 / .00657 / .01415 / .02148 | .0329 / .0554 / .1130 / .1601 |
| 4.993–5.047 | .00359 / .00507 / .00931 / .01428 | .0295 / .0536 / .1114 / .1912 |

The full [machine-readable summary](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_endpoint_revision_normal_20260923/analysis/summary.json) also contains quartiles for old-start age, both lag margins, and both post-publication update counts. End-age has Pearson correlation 0.237 with translation revision and 0.331 with rotation revision; time to end retirement has -0.137 and -0.051. All edges are emitted at essentially the same physical lag margin and undergo ~49–50 further updates, so neither lag-margin nor update-count variation supports a separate effect estimate.

## Online source control and topology test

The first active-pose snapshot for each source index reproduces the saved `/glim/cbs/odometry` pose topic with maximum right-local pose discrepancy **1.7×10⁻¹²** (TUM time formatting differs by at most 50 ns). The source's actual causal path is therefore [source_online_first.tum](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_endpoint_revision_normal_20260923/analysis/source_online_first.tum), not the later shutdown dump. On the same 592-pose window it scores **0.017052 m ATE**; its matured and shutdown paths score **0.013218** and **0.013221 m**. The publication-edge reconstructions below are much worse than *either* source path. Publication belief-vs-shutdown disagreement is not called a GLIM bug: the beliefs were true to their publication-time smoother snapshot, but successive snapshots disagree at shared historical poses.

T1 contains only current `i→i+2` edges and has two 296-pose parity components. Its two component scores use **two separate diagnostic native gauges** and independent SE(3) alignments; a two-gauge combined full-path ATE would be invalid. T2 contains `i→i+1` edges from the same active snapshots. T3 retains all T1 edges and adds just **13 consecutive bridges** at indices `0,50,...,550,590` (roughly every 5 s plus the tail), making one connected, singly-edge-resilient graph. T3's cycles were solved in an isolated pose-only GTSAM graph with an exact root gauge and **equal unit noise for all edges solely to isolate means/topology**; it is not a receiver covariance experiment. T1/T2 trees are exact SE(3) compositions, for which covariance weights cannot affect the mean fit.

The unchanged GrandTour AP20-prism scorer uses CPT7→prism lever arm and one no-scale SE(3) Kabsch alignment; 1 s RPE uses the CPT7 IE-TC bag. Comparisons within a row pair have identical timestamps/masks. T1's half-rate RPE (580 valid pairs per component) should not be compared numerically with the 0.10 s full-grid RPE (581 valid pairs); its publication/final comparison is valid.

| Source/product, common retired window | Components | Edges | Prism ATE (m) | 1 s t-RPE (m) | 1 s r-RPE (deg) |
|---|---:|---:|---:|---:|---:|
| Actual online GLIM, first snapshots | — | — | 0.017052 | 0.019589 | 0.659927 |
| Matured GLIM active poses | — | — | 0.013218 | 0.016161 | 0.652353 |
| Shutdown `odom_imu.txt` | — | — | 0.013221 | 0.016162 | 0.652362 |
| T1 publication, even chain | 2 | 590 total | **0.063477** | 0.027143 | 0.987241 |
| T1 publication, odd chain | 2 | 590 total | **0.064316** | 0.025358 | 0.950268 |
| T1 finalized, even / odd | 2 | 590 total | **0.013073 / 0.013361** | 0.018545 / 0.018285 | 0.961429 / 0.929920 |
| T1 final revision C, even / odd | 2 | 590 total | **0.013073 / 0.013361** | 0.018545 / 0.018285 | 0.961429 / 0.929920 |
| T2 consecutive publication | 1 | 591 | **0.086103** | 0.024044 | 0.696700 |
| T2 consecutive finalized | 1 | 591 | **0.013218** | 0.016161 | 0.652353 |
| T3 0.20 s + periodic bridges, publication | 1 | 603 | **0.063618** | 0.024735 | 0.692642 |
| T3 0.20 s + periodic bridges, finalized | 1 | 603 | **0.013218** | 0.016161 | 0.652353 |

Publication-to-final **sum of per-edge error norms** is 3.074 m / 25.154° for T1, 2.664 m / 25.781° for T2, and 3.134 m / 25.702° for T3. These are *absolute accumulated edge discrepancies*, not signed trajectory drift. Direct T1 publication composition ends **0.420 m** from the shutdown pose after 59.1 s despite two favorable diagnostic root gauges; T2 ends **0.556 m** away with one root. T3's optimized publication path ends **0.384 m** away and has 0.143 m raw position RMS from the shutdown path. All finalized edge sets have zero internal edge error by construction and reproduce the source path up to the last-active-versus-shutdown difference (~0.05 mm RMS).

Single-edge-removal topology sensitivity: **590/590 T1 edges** and **591/591 T2 edges** are graph bridges, so removing any one disconnects its path. **0/603 T3 edges** are graph bridges. As a numerical control, deleting T3's largest-revision edge `509→511` (24.24 mm relative change) leaves the graph connected but changes its publication reconstruction by **0.0184 m RMS / 0.0914 m maximum** position. Its ATE changes 0.063618→0.062717 m while t-RPE changes 0.024735→0.027777 m. The corresponding finalized T3 reconstruction remains at 0.013218 m ATE. This is sensitivity, **not a basis for deleting or gating that edge**.

The [topology edge products](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_endpoint_revision_normal_20260923/analysis), [individual evaluator JSONs](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_endpoint_revision_normal_20260923/analysis/evaluation), and diagnostic-only [Python builder](tools/eig2_glim_revision_topology_audit.py) / [GTSAM pose graph](tools/eig2_glim_topology_pose_graph.cpp) allow every reported comparison to be checked.

## Decision table

| Question | Answer from this 59.1 s common window |
|---|---|
| How much do active poses revise? | 591/592 move >1 µm; first-to-final pose translation median/RMS 5.83/9.69 mm, rotation 0.0476/0.1115°. |
| How much relative-edge error comes from revisions? | All 590 relative edges move >1 µm; 4.07/6.53 mm translation median/RMS and 0.0313/0.0565° rotation. Neighboring publication edges disagree at their shared endpoint by 4.09/6.32 mm median/RMS on T1. |
| How much degradation remains with finalized edges? | None material in this window: T1 per-chain 0.0131/0.0134 m ATE, T2/T3 0.013218 m versus 0.013221 m shutdown GLIM. |
| How much is due to the disconnected 0.20 s topology? | It removes the even↔odd gauge and makes every T1 edge critical. But connecting T1 with 13 bridges still scores 0.063618 m from publication edges, whereas finalized T1 already scores native per chain. Thus it is a real structural weakness, **not the dominant measured mean-error cause here**. |
| Do 0.10 s finalized edges reconstruct native GLIM? | Yes: T2 finalized scores exactly the matured source to shown precision; raw position RMS to shutdown is 0.060 mm. |
| Do 0.10 s publication edges reconstruct causal online GLIM? | No: 0.086103 m ATE and 0.192 m raw position RMS to the causal first-snapshot path, versus online GLIM's 0.017052 m ATE. Their shared endpoints change 3.15/4.87 mm median/RMS between successive edge publications. |
| Would revision-aware publication theoretically recover the matured source path? | Yes **for the means**, if all revisions were retained until both endpoints finalized: final C equals B at `1e-10` right-local tolerance and has identical T1 scores. Causal delivery, changing covariance, receiver replacement semantics and cost were **not** implemented or tested. |

## Validation, provenance, and limits

- The normal-run bag, shutdown TUM, belief CSV and summary SHA-256 are respectively `ce4df788da82ebcc52e2e23499204c356fac1e6ad34ec22d04001e1064d6f2c1`, `94d99567e21c4e448b5ab1253ab1c79dab7dc89f6d44d02978bed52db722c22b`, `cfc5a6073d35683190310690a2af7f57476c383898c40bb460ba65dafdd343e7`, and `12d4c44b703f40c2f081c5b790ba022a89efaf2727d7148624ee481721b39590`.
- The frozen previous full-run GLIM shutdown TUM and belief CSV still hash to `88bfcf1bedbd9b94eceeeea8e9421b74d667ae28a8f7a4b4e426159e2f94deec` and `4ba70e11298ba0f8337fb9806b7986f3ecbc2eb4eee817b16743059abfa43782`. EIG-2 Hesai/CPT7 input bags hash to `7bab574843ddbe0c554eeb9d33b8989d7aeff19f6c54210482336766d2d54f0e` / `325f642669e5747a34c25b03cc6139b3927804f6b6519d6079acc4ca2045a20b`.
- GLIM Release build succeeded. The parser passed endpoint→mean, active snapshot→endpoint, all **641** saved belief mean/covariance, topology connectivity, and final-revision→final-edge assertions. The isolated T3 graph converged from cost 0.0228135 to 0.000137083 for publication edges; finalized cost is ~`2e-27`. Original evaluation scripts, source covariances and input bags were unchanged.
- The GLIM source checkout was **already dirty** before this task (including the bridge file); only two flag-gated log blocks were added here. It is intentionally not claimed clean or committed wholesale. The gnssFGO diagnostics are isolated tools, not production gnssFGO factors. There is no result here about Kimera or full 305 s behavior. No CBS protocol, covariance, gnssFGO receiver, or estimator mathematics was changed.

**Decision: publication-time endpoint revision / non-telescoping is the primary observed loss in the bounded EIG-2 source product; sparse two-chain topology is an independent observability and resilience weakness. Stop at this source-side audit.**
