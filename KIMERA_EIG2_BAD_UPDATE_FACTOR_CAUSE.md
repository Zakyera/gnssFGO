# EIG-2 Kimera bad-update factor-cause audit — stopped at reproduction gate

**Result: factor attribution is not valid yet.** The bounded CBS-OFF reproduction used the same source bags, 5.7 s bag offset, clock seed, profile, and 0.10× playback as the retained 305 s run. It processed Kimera backend updates through 1106, including 1084 and 1093. The source keys, physical timestamps, and revision IDs agree, but the optimized relative beliefs cease to agree materially beginning around update 776. By update 1084, the 1079→1080 tangent translation differs by 0.0983 m between runs. Accordingly, the reproduction is **not the historical optimization event** whose factor cause was requested. No visual/IMU/prior cause is claimed.

## Scope and retained references

- Historical CBS-OFF EIG-2 run: [full305](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_kimera_full_revision_20260924/full305), GrandTour mission `2024-11-03-13-59-54`, Alphasense front stereo + CPT7 IMU, incoming CBS disabled. Its [estimators.log](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_kimera_full_revision_20260924/full305/estimators.log) SHA-256 is `76179f085c441b67e2c90f8d52f1e7546a09b52fcea3f4c49e320f02dca9aa6a`; the [revision CSV](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_eig2_kimera_full_revision_20260924/full305/revision_products/revisions.csv) remains `13c297ddcba5ccea221258e7aa024844489d163886bc46552f2e91f84804ab47`.
- Bounded reproduction: [bounded_original_semantics](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_kimera_eig2_bad_update_factor_cause_20260925/bounded_original_semantics), bag start 5.7 s, duration 221.5 s, playback 0.10×, seed clock 1730638811, 1106 Kimera output rows, zero inbound G2K/K2G factors. Its [run metadata](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_kimera_eig2_bad_update_factor_cause_20260925/bounded_original_semantics/run_metadata.txt) and [estimators.log](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_kimera_eig2_bad_update_factor_cause_20260925/bounded_original_semantics/estimators.log) are retained; log SHA-256 `989dda57f1c75a6a4aaecf71cbc62533f13dfcbf2accb1b18147f07c4efea8c8`.
- The same three EIG-2 bag files were opened in both run logs. The CPT7 bag SHA-256 is `325f642669e5747a34c25b03cc6139b3927804f6b6519d6079acc4ca2045a20b`. Recursive file comparisons found no difference between the saved Kimera parameter profiles or GLIM configurations. The saved parent launch files differ only by the new diagnostic argument in the mono branch. They do not document the exact compiled Kimera binary/dirty source tree of the historical run, so source-code identity cannot be asserted from those files alone.

The historical local-error jump remains the one documented in [KIMERA_EIG2_REVISED_BELIEF_SOURCE_CAUSE.md](/home/yeranis/repos/gnssFGO/KIMERA_EIG2_REVISED_BELIEF_SOURCE_CAUSE.md): 1079→1080 rises from 0.020782 to 0.102268 m at update 1084; three adjacent edges deteriorate at 1093. These numbers are properties of the **historical** run, not of this reproduction.

## Checkpoint feasibility and diagnostic design

There is no validated complete restart checkpoint for this Kimera/BPSAM pipeline. GTSAM's `ISAM2` has a serialization hook, but [Kimera's `saveGraph`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO/src/backend/VioBackend.cpp:2068) calls graph visualization output, not estimator serialization. The [fixed-lag wrapper](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbs/include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h:70) also owns key timestamps and marginalization bookkeeping; Kimera's backend owns old/new smart-factor maps and other continuation state, while the frontend owns feature/IMU continuation state. Saving poses and a factor list would not preserve these. I did **not** call such a dump a checkpoint or construct an approximate one.

Flag-gated, read-only code was added at [VioBackend.cpp:1027](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO/src/backend/VioBackend.cpp:1027), [VioBackend.cpp:1274](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO/src/backend/VioBackend.cpp:1274), and [the fixed-lag wrapper](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbs/include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h:70) for updates 1082–1085 and 1091–1094 only. When enabled, it is designed to clone pre/post graphs, save full values and factor/observation tables, marginalization keys/slots/anchors, evaluate family costs on one graph, compute a local linearized influence, and attempt an unchanged-graph LM reference solve. It does not change optimizer factors, covariances, or update calls. However, **the bounded replay did not activate this flag**, so none of these target factor outputs exists and the implementation has not been validated on a target graph.

There is a second archival limitation: this checkout's `SmartStereoProjectionPoseFactor` cannot be validated through the available Boost graph round trip (its constructor/serialization path does not support the attempted load). Consequently, even if the diagnostic is later activated, its per-factor tables, linear system, and in-process reference solve must not be presented as a complete, reloadable nonlinear graph checkpoint without a separately validated serializer.

## Launch preflight and bounded replay record

Two *early aborted* setup attempts exposed configuration errors before any target graph was interpreted:

1. The first did not set `KIMERA_CBS_REVISION_AUDIT=1`, so it could not validate the source revisions.
2. The second set the revision audit but the then-current runner had dropped the archived runner's `--start=5.7` and seed-clock controls. Its first edge therefore had the wrong physical timestamp. The original controls were restored in the diagnostic runner, without touching estimator mathematics.

The one target-reaching run then used the correct archived bag/clock semantics. Its first edge was timestamp-identical to the original; spot checks at updates 379 and 700 differed by at most `7.2e-9` and `1.3e-8` in compared numeric belief fields. It reached and passed both target updates. Afterward I discovered that [the parent launch's diagnostic argument](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/glim_ros1/launch/grandtour_eth1_glim_kimera_cbs.launch:241) had been attached to the mono Kimera node, while this run used the nested stereo launch; inspection of the actual process command line confirmed the flag was absent. The stereo forwarding chain was corrected **after** the run in the [GrandTour stereo launch](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO-ROS/launch/grandtour_eth1_kimera_stereo.launch:35), [Newer College stereo adapter](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO-ROS/launch/kimera_vio_ros_newer_college_2021_stereo.launch:70), and [base node launch](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO-ROS/launch/kimera_vio_ros.launch:277). Read-only `roslaunch --args=/kimera_vio_ros/kimera_vio_ros_node` now resolves `--kimera_bad_update_audit_dir=/tmp/kimera_audit_preflight` through the complete stereo include chain. This verifies wiring, **not** a captured target graph. No second target-reaching replay was started.

## First reproducibility divergence

The read-only [comparison script](/home/yeranis/repos/gnssFGO/tools/kimera_eig2_reproduction_trace.py) compared every common outgoing revision row up to update 1100, rather than selecting only the two bad events. Its [per-update CSV](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/_kimera_eig2_bad_update_factor_cause_20260925/bounded_original_semantics/reproduction_trace.csv) has SHA-256 `1982d0607dde6394bd9c605c180d3c553bed69776b976ee62ac75f64eaa2e5ab`.

| Reproduction landmark | First update | Maximum differing component of `[phi,rho]` among that update's common revisions |
|---|---:|---:|
| Above `1e-6` | 761 | `1.21e-6` |
| Above `1e-4` and `1e-3` | 776 | `0.001322` |
| Above `1e-2` | 810 | `0.014767` |
| Above `5e-2` | 932 | `0.088443` |

The source row identities, first/active revisions, and endpoint timestamps match at **all 1100 compared updates**. Through update 700 the largest relative-mean component difference is `1.13e-8`; through 760 it is `9.01e-7`. At 775 the latest edge 774→775 differs by only `0.0000512 m` in tangent translation. At 776, 775→776 differs by `0.001391 m` in tangent translation and `4.79e-5 rad` in tangent rotation. Importantly, the logged `cbs_marginalization_graph_factor_count` first differs at exactly 776: original 880, reproduction 874 (both 884 at 775). This counter is the **local CBS covariance graph**, not a direct inventory of the active optimizer graph; it is evidence of changed source-side graph content, not proof of which visual track, IMU factor, or marginal prior changed. Both runs report smoother success at every one of their 1106 common backend updates.

The effect accumulates well before 1084: at 1083, the 1079→1080 tangent translation already differs by `0.01107 m`. The target comparisons are:

| Historical vs reproduced revision | Relative tangent translation difference | Rotation difference | Largest 6×6 covariance-entry difference |
|---|---:|---:|---:|
| 1079→1080 at 1084 | 0.09834 m | 0.000225 rad | `1.38e-5` |
| 1084→1085 at 1093 | 0.06417 m | 0.000206 rad | `2.82e-5` |
| 1085→1086 at 1093 | 0.05721 m | 0.000206 rad | `3.13e-5` |
| 1086→1087 at 1093 | 0.05766 m | 0.000208 rad | `3.51e-5` |

These are **between-run disagreements for the same numbered rows**, not local IE-TC errors or a cost decomposition. They fail the requested close-agreement gate. The divergence may be sensitive to frontend tracks, incremental solver history, scheduling, or historical binary differences; these artifacts do not isolate which. The matching index/timestamp stream and first factor-count difference at 776 are concrete observations, not a license to call visual factors the cause.

## Requested factor tests: not interpretable

There is no valid update-1084/1093 nonlinear graph snapshot from the bounded reproduction, and its source beliefs differ materially from the historical event. Therefore I did not produce or interpret:

- same-post-graph visual/CombinedImu/marginal-prior cost changes;
- the connected visual tracks, stereo disparities/parallax/depth, IMU residuals or bias/velocity changes at those historical updates;
- `H^{-1}(-g_f)` family pulls or a production-versus-reference solve of the historical graph.

Producing numerical values for a different trajectory and labeling them as the original bad update would be misleading. The original [source-cause report](/home/yeranis/repos/gnssFGO/KIMERA_EIG2_REVISED_BELIEF_SOURCE_CAUSE.md) already establishes that the revised belief mean equals `Log(X_i^{-1}X_j)` to numerical precision; this run does not overturn that algebraic extraction check.

## Validation and non-regression

- `catkin build cbs kimera_vio --no-status -j2 --make-args -j2` succeeded in the existing ROS Noetic container in Release configuration. `bash -n` on the runner, Python compilation of the analysis helper, and `git diff --check` for touched packages passed.
- The post-run stereo launch chain was verified with read-only `roslaunch --args`; it was **not** exercised by another estimator run. The target snapshot/reference/linear-influence code likewise remains unvalidated in a real target graph and must be treated as diagnostic work-in-progress, not a scientific result.
- Historical log, revision CSV, and IE-TC reference hashes are unchanged. No CBS-ON, fusion, full 305 s run, weight change, topology change, or source-belief policy change occurred.
- The workspace had substantial unrelated pre-existing modifications across nested repositories. I preserved them. This task added opt-in diagnostic code, launch forwarding, two read-only comparison scripts, and this report; I did not claim a clean worktree or create a commit on top of the existing dirty state.

## Five explicit answers

1. **What factor families actually cause the correction at 1084?** Unknown. No historical or sufficiently matching reproduced graph snapshot is available. The first material between-run divergence precedes it by 308 backend updates.
2. **What factor families cause the shared correction at 1093?** Unknown for the same reason; the three specified relative beliefs differ from the retained source by roughly 5.7–6.4 cm in tangent translation in this replay.
3. **Does production optimization solve the same objective as a more exhaustive reference solve?** Not established. The exact historical graph was not saved; this bounded replay's target-only reference diagnostic was not enabled on its stereo node, and in any case its beliefs do not reproduce the historical event.
4. **Is the revised relative belief itself mathematically valid?** Yes as a **mean extraction** from that run's optimized endpoint poses: the prior source audit independently verified `Z_ij=X_i^{-1}X_j`/`Log` at numerical precision, with no endpoint-order or timestamp bug. This says nothing about whether its factors or weights make the optimized local motion accurate against IE-TC.
5. **What is the smallest justified next experiment/change?** First isolate why the source graph/history becomes run-dependent near update 776. A future, separately authorized normal-start diagnostic should capture the active visual factor/track identities, added/removed factors, iSAM result, and marginal prior over approximately 760–780, with the now-verified stereo flag wiring and an early reproducibility gate. Only after reproducibility is established should 1084/1093 factor pulls be interpreted. No CBS policy, covariance, topology, or fusion change is justified by this audit.
