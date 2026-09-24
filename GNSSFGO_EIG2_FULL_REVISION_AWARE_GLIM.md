# EIG-2 full-sequence revision-aware GLIM preservation

Date: 2026-09-24. Scope: the complete requested **305 s EIG-2 CBS-OFF GLIM-only** source window, using the bounded experiment's 0.20 s `i→i+2` two-parity topology and the same full gnssFGO factor graph. No Kimera, CBS-ON, GNSS factor, fusion, covariance scaling, topology change, or estimator tuning was performed.

## Answer

**Yes: stable-BeliefID factor replacement recovers most of the full-sequence first-publication preservation loss.** On this new single-source execution, R0's final graph has **0.402425 m** prism ATE, whereas causal revision-aware R1's final graph has **0.051038 m**. The noncausal last-jointly-valid oracle R2 has **0.051080 m**. R1 and R2 final trajectories differ by only **0.94 mm RMS** in the original common gauge. Relative to the same-run matured source ATE of **0.042848 m**, R1 closes about **97.7%** of R0's ATE gap; it does **not** exactly reproduce the native source. The prior approximately 0.406 m figure was from a different source execution and is only historical context.

The **causally available online trajectory**, saved before future revisions arrived, is a different product: R1-online scores **0.053447 m ATE**, versus R0-online **0.441859 m**. The retrospective 0.051038 m score must not be presented as instantaneous online accuracy. This is GLIM-only preservation evidence, not evidence for Kimera fusion or a live revision wire protocol.

## Controlled results

All comparable full/source/online rows use one unchanged GrandTour AP20 prism evaluator with the CPT7-IMU-to-prism lever arm and a separate no-scale SE(3) Kabsch alignment per trajectory. The **identical GT timestamp mask** contains 1,899 of 2,992 support poses. Because prism GT has gaps, matched samples total 189.096 s but span the 299.100 s first-to-last support window. The unchanged IE-TC 1 s evaluator supplies 2,981 translational/rotational RPE pairs. Path lengths below are the evaluator's full prism-frame path and path over the matched GT samples; full receiver internal IMU-origin path lengths are listed separately below.

| Full EIG-2 product | Belief semantics | ATE (m) | 1 s t-RPE (m) | 1 s r-RPE (deg) | Full prism path (m) | GT-matched path (m) | Online/final |
|---|---|---:|---:|---:|---:|---:|---|
| Native GLIM online | Source trajectory at first/current publication | 0.044467 | 0.023045 | 0.603698 | 185.119 | 137.873 | Online source |
| Native GLIM matured | Retired/finalized source trajectory | 0.042848 | 0.019066 | 0.596824 | 182.807 | 136.678 | Matured source |
| R0 final | First GLIM publication, one factor/ID | 0.402425 | 0.019142 | 0.621141 | 182.362 | 136.576 | Final graph |
| R1 online | Record-time causal first/newer replacements | 0.053447 | 0.030601 | 0.630839 | 184.745 | 137.570 | As-played state |
| R1 final | Same causal replacements after all arrivals | 0.051038 | 0.013951 | 0.594893 | 181.965 | 136.349 | Retrospectively revised graph |
| R2 final | Last jointly covariance-valid revision only | 0.051080 | 0.013955 | 0.594941 | 181.969 | 136.350 | Noncausal oracle |
| R3 final | Finalized endpoint mean, R2 covariance | 0.047827 | 0.013953 | 0.594875 | 181.959 | 136.350 | Noncausal control |
| R0 online | First-publication receiver state as played | 0.441859 | 0.047086 | 0.661190 | 185.702 | 138.298 | As-played state |

The order of the two R0 rows is intentional: a final graph can retrospectively revise earlier support states even without a newer GLIM measurement. Receiver-internal path lengths (m) are R0 **179.304**, R1 **178.857**, R2 **178.862**, and R3 **178.852**. They are not interchangeable with the evaluator's prism-lever-arm paths above.

### Pose-only control and topology

The source GLIM beliefs remain two disconnected parity chains, each with 1,496 support poses and 1,495 edges. Pose-only ATE is therefore scored **separately** with a distinct gauge for each chain; a combined pose-only ATE would be invalid. Even/odd pose-only ATEs (m): R0 **0.672588 / 0.778577**; R1 **0.046458 / 0.046406**; R2 **0.046460 / 0.046414**; R3 **0.042870 / 0.042826**. The full receiver's unchanged Combined IMU/WNOJ factors connect its support states. No parity bridge or 0.10 s topology was added.

## Source execution, data products, and right censoring

The successful run is [the full source directory](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_full_revision_305s_retry_20260924). It reused the [bounded GLIM-only launch](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_endpoint_revision_audit_20260923/run_glim_only.sh) with only its requested duration parameter set to `305.0` s: same Hesai, CPT7, 5.7 s start offset, 0.10× playback, direct smoother marginal, 5 s fixed lag, `observe_only`, `max_horizon_pairs_per_update=1`, `schur_relative_between`, Kimera and CBS receiver disabled, and diagnostic full mean-plus-6×6-covariance revision log enabled. The source estimator mathematics and published ROS belief stream were not altered. The first-publication [ROS bag](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_full_revision_305s_retry_20260924/recordings/cbs.bag) was finalized and passed the existing exporter.

The source generated **3,043** pose rows and **3,041** unique first-published beliefs; the final **51** poses/beliefs remained inside the 5 s source lag at shutdown. They are explicitly **right-censored**, not called finalized and not included in the common receiver comparison. The retained population is **2,992** retired poses and **2,990** stable BeliefIDs. Every retained ID has its first ROS-bag payload matched to revision 0 in the source log, including all 36 covariance entries, within `1e-12`; no duplicate selected first publications or source covariance errors occurred. The bag exporter's generic `connected_components` row-chain count is not the graph topology; the audited graph has the two parity chains above.

The [revision converter](tools/eig2_revision_stream_export.py) used the unchanged recorded ROS record timestamp for revision 0 and the logged source availability timestamp for later revisions. It produced [first.csv](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_full_revision_305s_retry_20260924/revision_products/first.csv), [revisions.csv](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_full_revision_305s_retry_20260924/revision_products/revisions.csv), [last_revision.csv](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_full_revision_305s_retry_20260924/revision_products/last_revision.csv), and [finalized.csv](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_full_revision_305s_retry_20260924/revision_products/finalized.csv). R3 uses finalized *means* and the R2 last jointly valid full covariance; it does not invent a nonexistent covariance after the older endpoint has left the lag.

## Receiver semantics and preservation checks

All four full receiver cases used the same IMU CSV, support-state grid, startup and four initial priors, right-supported CPT7 convention, Combined IMU factors, WNOJ factors, state initialization, GLIM `BetweenPose3Factor` construction, iSAM2 configuration, and native dense source covariance. R0 inserted the first version by record availability. R1 inserted version 0 and **atomically removed/replaced** the factor for `(source,from_index,to_index)` whenever a newer complete mean-plus-covariance version became available. R2 and R3 insert their selected measurements at physical time and are explicitly noncausal controls. Each full case accepted **2,990 distinct GLIM IDs**, zero Kimera IDs, zero unsupported edges; R1 received **144,876** valid revision messages, with 2,990 first insertions and **141,886 replacements**. No duplicate, stale, or rejected revisions were observed. The final active graph has the expected **8,976 factors = 4 priors + 2×2,991 motion factors + 2,990 GLIM factors**, not 144,876 GLIM factors. Peak active variable count was **11,968**.

Final raw same-gauge position separation over all 2,992 common timestamps:

| Pair | RMS (m) | Maximum (m) | Interpretation |
|---|---:|---:|---|
| R1–R0 | 0.708609 | 1.227394 | First publications produce a materially different graph; first separation above 1 cm is at 0.8 s. |
| R1–R2 | 0.000938 | 0.003130 | Causal replacement closely reproduces the last-valid oracle; no timestamp exceeds 1 cm. |
| R2–R3 | 0.008475 | 0.019089 | Finalized means are not identical to last jointly valid means; first separation above 1 cm is at 162.6 s. |

R1≈R2 is strong but **not byte equality**. Different factor insertion/relinearization order and initialization from each graph's then-current state can leave a millimetre-scale numerical difference. R2–R3 is a separate, physical last-valid-versus-finalized mean distinction: source edge difference is 0.152 mm translation RMS (5.081 mm maximum) and 0.000552° rotation RMS (0.01677° maximum). Their graph-level separation can be larger, while their ATEs remain close. No discrepancy was suppressed or tuned away.

## Online accuracy during playback

The [online trajectory files](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_full_revision_305s_retry_20260924/revision_products/full_r1) capture the **current** estimate when each support time was played; final `trajectory.tum` is extracted after all known revisions have been applied. Cumulative ATE uses each complete online trajectory's own fixed no-scale evaluation alignment, **not** an alignment refit at each checkpoint or any estimator input. All checkpoint rows use the same matched GT timestamps in both receiver cases; 120–150 s adds no matches because prism GT is unavailable there.

| Playback elapsed (s) | Matched poses so far | R0-online cumulative ATE (m) | R1-online cumulative ATE (m) |
|---:|---:|---:|---:|
| 30 | 301 | 0.182698 | 0.060204 |
| 60 | 601 | 0.276243 | 0.061657 |
| 90 | 666 | 0.294012 | 0.062003 |
| 120 | 709 | 0.292656 | 0.062194 |
| 150 | 709 | 0.292656 | 0.062194 |
| 180 | 779 | 0.375482 | 0.060557 |
| 210 | 1,079 | 0.381933 | 0.055490 |
| 240 | 1,379 | 0.417976 | 0.053111 |
| 270 | 1,661 | 0.427866 | 0.051765 |
| 300 | 1,899 | 0.441859 | 0.053447 |

Thus replacement benefits the **as-played causal estimate** throughout the sampled run, not only the final retrospective graph. Its online t-RPE remains 0.030601 m, worse than its final graph's 0.013951 m: a final-graph RPE should not be used as the real-time RPE.

## Revision dynamics, covariance, and runtime

| Diagnostic | Result |
|---|---:|
| Revision messages / retained IDs | 144,876 / 2,990 |
| Revisions per ID, median / mean / p95 / max | 48 / 48.454 / 49 / 49 |
| Revision-0 latency from physical edge end, median | 0.1497 s |
| Last jointly valid latency, median / mean / p95 / max | 4.9001 / 4.8969 / 4.9842 / 5.0415 s |
| Revisions per occupied record second, median / mean / p95 / peak | 485 / 475.0 / 534 / 580 |
| iSAM2 revision update time, median / mean / p95 / max | 0.522 / 1.124 / 3.747 / 63.593 ms |
| Replacement-only update time, median / mean / p95 / max | 0.510 / 1.099 / 3.671 / 63.593 ms |
| Total revision-update time | 162.844 s |
| Full R1 receiver replay wall time / simulated support time | 194.610 s / 299.100 s |
| Processing real-time factor on this offline machine | 1.537× |

For comparison, wall times for R0/R2/R3 were 88.276/26.419/26.116 s. R1 is the costly product; the 305 s source itself was played at 0.10×, and these receiver timings do **not** establish end-to-end live throughput. Memory bytes were not instrumented; peak factor/variable counts are the available size measures. The median last-valid latency reflects the frozen 5 s source lag, not a receiver timeout.

Revision 0's median relative-translation difference to the both-endpoint finalized source edge was **4.583 mm** (6.514 mm RMS); at 25/50/75/100% of each ID's versions the medians fell to **0.0740/0.00308/0.000433/0.000257 mm**. Coordinate step size between consecutive source revisions was median/p95/max **0.00315/0.996/20.14 mm** translation and **0.0000194/0.00717/0.136°** rotation. These are logged coordinate differences, not an SE(3)-log increment test. Median last/first rotational and translational native covariance trace ratios were **0.769** and **0.618**; source covariance was recomputed, not frozen or scaled. The [full machine-readable summary](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_full_revision_305s_retry_20260924/revision_products/full_result_summary.json) retains the p95/max mean and covariance step statistics and per-case score provenance.

## Plots

The plots show [same-gauge trajectory separation](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_full_revision_305s_retry_20260924/revision_products/trajectory_separation.png), [online cumulative ATE](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_full_revision_305s_retry_20260924/revision_products/online_cumulative_ate.png), [revision rate](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_full_revision_305s_retry_20260924/revision_products/revision_rate.png), [revision latency](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_full_revision_305s_retry_20260924/revision_products/revision_latency.png), and [mean/covariance revision magnitudes](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_full_revision_305s_retry_20260924/revision_products/revision_magnitude.png). The [per-event ledger](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_full_revision_305s_retry_20260924/revision_products/full_r1/revision_events.csv) provides individual update timing and factor counts.

## Validation and reproducibility

- [Source topology audit](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_glim_full_revision_305s_retry_20260924/analysis/summary.json) confirms first-bag/source payload equality, the two-parity topology, 51 right-censored rows, and zero source errors. The existing audit needed only a diagnostic Python recursion-limit increase to traverse the 3,000-edge chains; no graph definition or source mathematics changed.
- The same retained [score script](tools/eig2_revision_score.sh) evaluated all cases. The additional [online error analysis](tools/eig2_full_revision_online_errors.py) and [full-result analysis](tools/eig2_full_revision_analysis.py) assert common timestamp masks and keep raw products separate from scored/processed outputs. The five plots were visually checked. All full/source/online results have the identical 1,899-row GT mask; 1 s RPE uses the existing IE-TC evaluation path.
- The existing full receiver, pose-only receiver, revision converter, GLIM source maths, IMU/WNOJ, priors, and covariance formulation were not modified. This run reuses the previously built validated binaries and bounded controls; no new synthetic test, sanitizer pass, or repeat of the expensive full source run was claimed.
- Source bag SHA-256: `cafa6d6cee6e27670788ab00add469cb4b6c38b41f7f9e78574a9956cbba5359`; ordered first/revision/last/finalized CSV SHA-256: `49154b46701244dcf830b6944835721774db7750408458000fad0fbc1cf56a48`, `a7b600211b6f80f665040ad57a8723473864875563fdc6bd437ece6c82949d80`, `022bc18a4a5de290d30652c2cf7d3cc3b01c61f3406f689fbae4bd555dbff1c0`, `7239b08149de088ad0850e495b2a57b79bd5b7cb02b33c491964d3e2f0b0e8f3`.
- The GLIM nested checkout was already dirty and remains separate from this report. Only the retained launcher duration was parameterized and the Python source-audit traversal capacity increased. Neither is an estimator change. Previous experiment folders were not overwritten. An initial root-user launch failed before producing usable data because it could not open the pre-existing `/tmp/glim_odom.log`; the completed run used the container's normal `cbsms` user and a new output directory.

Reproduction: in the existing Noetic container set `CBS_EIG2_AUDIT_RUN_DIR=/workspace/cbs_gtsam4.3/runs/_eig2_glim_full_revision_305s_retry_20260924` and `CBS_EIG2_AUDIT_DURATION_SEC=305.0`, then invoke the retained `run_glim_only.sh`; export the finalized `recordings/cbs.bag` with the unchanged `export_cbs_unique_relative_beliefs.py`; run `tools/eig2_revision_stream_export.py --retired-pose-count 2992` on that bag export and `estimators.log`; then run the retained `grandtour_ct_replay` in `glim`/`isam2` mode with `--timeline record` and `first.csv` for R0, add `--revision-beliefs revisions.csv` for R1, or use `--timeline physical` with `last_revision.csv`/`finalized.csv` for R2/R3. The run's unchanged IMU CSV has SHA-256 `c4d7b68674849c4be3b3b83198050c564ff310fa2167a42aa73217321b933fca`. The source and receiver paths are recorded in the linked artifacts.

**Conclusion:** on this complete EIG-2 CBS-OFF GLIM-only run, first-publication staleness is the dominant cause of the approximately 0.045-to-0.406 m preservation gap. Causal full-covariance factor replacement reduces final receiver ATE to 0.051 m and as-played online ATE to 0.053 m, near the 0.043 m matured same-run source and 0.051 m last-valid oracle. The residual source-to-receiver gap, the approximately 5 s revision maturation, and revision-processing cost remain explicitly measured limitations. Stop here: no Kimera or fusion experiment was run.
