# Classic CBS baseline recovery: launch and runtime audit

Date: 2026-09-25. This audit concerns native GLIM/Kimera CBS exchange. No estimator or fusion replay was launched, and no production source or launch default was changed. Historical artifacts remain untouched. The configuration below is recovered from saved evidence; a new execution has not yet been certified equivalent to a historical binary.

## Findings that determine the recovery

1. The latest EIG-2 40 s ON run explicitly enabled the newer live-revision protocol. This is confirmed by `glim_cbs_live_revision_enable: true` in its saved `resolved_glim_params.yaml`, rather than inferred from a directory name. It received 4,500 G→K and 4,549 K→G insertion/replacement events. The successful historical full EIG-2 run received 1,517 and 1,519 respectively over 305 s, all with distinct accepted source IDs in that run. The new test is not a reproduction of that historical exchange schedule.
2. The generic GrandTour runner defaults are unsuitable as an implicit historical preset: playback `0.75`, Kimera `mono`, maximum horizon pairs `25`. The successful ETH1 run explicitly used `0.15`, `stereo_shared_cpt7`, and `1`. The recent EIG-2 40 s run did explicitly select its intended stereo/0.10/one-pair settings, so those defaults are a future launch hazard, not an established mistake in that run.
3. The current live-revision selector defaults to false in the runner and launch. Thus the new mode was selected by the recent experiment, not silently made the universal default.
4. Current Kimera also contains a persistent cross-update receiver-match rule, and BPSAM contains sender-alternative replacement. Their presence in a dirty diff against the August 22 package HEAD is **not evidence that they were introduced after the September successful runs**. BPSAM alternative replacement is explicitly documented and exercised in an August 24 Newer College diagnostic. Blindly removing it would risk undoing established behavior.
5. The September 23 ETH1 native run at 0.15× already retained the historical benefit: native Kimera ATE 0.481104 m, compared with 0.662930 m in the September 7 ON run. The saved GLIM and Kimera profile trees compare identically. It requested 348 s instead of 346 s and changed visualization load, and historical binaries are not pinned, so this is corroboration rather than byte-level reproduction.

## Shared CBS mechanism to recover

The scientific recovery target is the exchange algorithm exercised by successful Newer College **and** GrandTour runs. Dataset-specific sensors, calibration, lag, start delay, playback, and estimator trajectory difficulty are separate controls. Do not call two runs algorithmically different because those dataset settings differ; do not call them algorithmically equivalent solely because both launches say `cbs_on`.

| Mechanism | Historical evidence / required comparison with current execution |
|---|---|
| Source relative mean and identity | Source graph snapshot and `(source, from, to)` identity used for every published relative belief; document if first publication or rolling-window update. |
| Source covariance | Full dense 6×6 `schur_relative_between` covariance from the intended same-snapshot local joint marginal; Kimera historical flags select the local smoother rather than a persistent covariance sidecar. Compare the actual graph used for extraction and source→receiver matrix values, not only the launch string. |
| Physical interval and receiver match | The approximately 0.20 s sender horizon, duration/timestamp matching and accepted source/receiver edge map. Per-update publication budget and start delay can differ by dataset; record their *effective* values and accepted events. |
| Factor semantics | Bidirectional persistent external relative-pose factors, scale 1.0 in both directions, health weighting disabled in the cited successful runs. Track queued, inserted, replaced, retired and rejected factors. |
| Publication/replacement policy | Historical rolling-window reapplications and odometry-key/receiver-edge replacements are real. Compare those exact rules with the opt-in September stable-ID live-revision protocol rather than equating all repeated payloads. |
| Feedback and evaluation product | Identify whether outgoing covariance excludes received CBS factors or retains them in a marginal prior; compare native published Kimera odometry to the same output, not to a different backend-state or fusion trajectory. This prior-history question remains unresolved by the saved launch files alone. |

### Retained native Kimera CBS successes in both datasets

These are saved OFF→ON native odometry ATE/RMSE pairs, not new evaluations. They establish that the earlier beneficial behavior was observed in **multiple sequences**, while leaving the exact source-binary chronology open.

| Dataset / sequence and run | Kimera OFF→ON error | Qualification |
|---|---:|---|
| Newer College Collection 3 Maths-Easy, official 216 s | 2.832943→2.702542 m | Official full run, persistent bidirectional CBS. |
| Newer College Collection 3 Maths-Medium, first OFF versus ON *repeat 2* | 3.284995→2.676151 m | First ON trial was **42.929955 m**; the repeat's factor counts differed, so it is a successful retained run, not proof of deterministic benefit. |
| Newer College Collection 1 Quad-Easy, matched 200 s OFF / ON zero-delay | 6.069798→2.465788 m | Both retained evaluators cover 852 Kimera poses and 198.556 s. |
| GrandTour ETH1 full | 9.114752→0.662930 m | OFF playback 0.30×, ON 0.15×; saved source profiles match. |
| GrandTour SPX-3 full | 1.056540→0.243733 m | Matched 0.30× playback. |
| GrandTour EIG-2 full | 1.309657→0.479174 m | Matched 0.10× playback. |

GrandTour SNOW-3 is a counterexample to a universal improvement: 0.352356→0.347218 m, a small change. The Newer College Maths-Medium first ON trial and the four-run Maths-Easy pilot show substantial run-to-run sensitivity under persistent CBS. Preserve these observations when evaluating a restored algorithm; select success by provenance and complete runs, not by the lowest ATE.

Dataset-specific contract examples are retained below only to reproduce particular recorded runs. Newer College Maths-Easy used cam4 monocular/Alphasense IMU, a 50-state Kimera window, 20 s CBS receive delay and up to 25 horizon pairs/update at 0.75×. ETH1 used front stereo/shared CPT7, a 25-state window, zero receive delay and one pair/update at 0.15×. The algorithm comparison above does **not** require those dataset settings to be equal.

Classic Newer College also cannot be defined as "one publication per source edge." Its rolling windows could repeat/update earlier edge payloads. The August 24 four-repeat audit recorded 281–378 same-sender reapplications per 45 s trial. This predates the new explicit revision protocol. Persistent factors also can be replaced under the historical odometry-key rules; "persistent" does not mean every arriving payload accumulates as a distinct independent factor.

## Example GrandTour launch contract

The existing runner remains `src/cbsms/tools/run_grandtour_eth1_glim_kimera.sh`. For a classic ETH1 baseline, its effective settings must be explicit:

```bash
CBSMS_MISSION=2024-10-01-11-29-55
CBSMS_BAG_START_SEC=0.0
CBSMS_BAG_DURATION_SEC=346.0
CBSMS_BAG_RATE=0.15
CBSMS_KIMERA_MODE=stereo_shared_cpt7
CBSMS_CBS_ENABLE=true
CBSMS_CBS_DIRECTION=bidirectional
CBSMS_CBS_LIVE_REVISION_ENABLE=false
CBSMS_MAX_HORIZON_PAIRS_PER_UPDATE=1
CBSMS_ODOM_PUBLISH_STRIDE=1
CBSMS_CBS_ODOM_HORIZON_SEC=0.20
CBSMS_CBS_BELIEF_RECEIVE_START_DELAY_SEC=0.0
KIMERA_IMU_SAMPLE_SUPPORT=current
```

This is a configuration specification, not a command executed by this audit. The current `KIMERA_IMU_SAMPLE_SUPPORT=current` branch is the historical packet path; using the later experimental RIGHT packet would introduce another variable. GLIM uses its existing source implementation. Use copies of the saved successful `glim_config/` and `kimera_profile/`, not mutable profile defaults. Validate both covariance scales, disabled health policy, timestamp matching, and the stereo launch's gflags after launch expansion. Use a new output directory.

For the existing EIG-2 40 s diagnostic window, the corresponding mission is `2024-11-03-13-59-54`, offset 5.7 s, duration 40 s, playback 0.10×, with the matching mission seed clock and bags. The explicit revision flag must be false for a test of the classic publication path. This bounded run could establish the effect of the newer revision mode; it cannot by itself reproduce a full-sequence ATE.

The old presentation switch also enabled the historical EKF: the current runner sets both `ekf_enable` and `factor_graph_enable` from `presentation_enable`. Native-only recovery must avoid starting that EKF. Turning presentation off also changes visualization load, so that change must be declared and held equal in the OFF/ON control. A separate inspector-only setting would require an explicit small launcher edit; it has not been introduced here.

## What is established, and what is still unproven

The current 40 s experiment selected a different source-publication policy. This is a verified launch/runtime difference. It is not yet proof that it causes the observed accuracy loss. Source covariance scales, basic sensor profiles, lag, and the relative covariance formula remain the same in the compared GrandTour saved configurations.

The historical package commits are August 22 (`Kimera-VIO e1148f64`, `cbs eb72f0c`) with accumulated dirty changes. Their HEADs do not identify the binaries used on September 7 or 21. Current source differences cannot be dated by `git diff` alone. The current cross-update match rule is unconditional for persistent factors; its introduction relative to the successful September runs remains unproven. BPSAM replacement demonstrably existed in August. Recovery must preserve this distinction rather than invent a rollback boundary.

Historical success was also not deterministic in every trial: the August 24 Maths-Easy four-repeat audit used identical resolved parameters and produced Kimera ATEs 0.569820, 0.831287, 0.601084, and 0.856880 m against OFF 0.839915 m, with different accepted factor sequences. This supports checking runtime coverage and actual accepted beliefs in addition to launch files. It does not erase the successful recorded runs or establish the cause of the latest result.

The acceptance target is recovery of the saved execution contract: sensors, calibration, native output definition, exchange direction, publication policy, factor lifetime, source covariance, startup, and runtime coverage. A lower ATE alone is not proof of equivalence. Record resolved ROS parameters **and** Kimera command-line flags, inherited IMU selectors, package HEADs plus dirty-diff hashes, binary hashes, source profiles, accepted source/receiver identities and factor lifecycle. Compare the same native odometry product and physical evaluation interval in OFF/ON.

## Retained evidence

- [Successful ETH1 ON metadata](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/grandtour_eth1_presentation/grandtour_eth1_stereo_shared_cpt7_no_sidecar_bidirectional_stride1_rate015_glimfg30_full_346s_live_rerun_20260907/run_metadata.txt).
- [Successful EIG-2 ON metadata](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/grandtour_eig2_glim_kimera/eig2_stereo_shared_cpt7_cbs_on_bidirectional_full305_start5p7_rate010_20260921/run_metadata.txt).
- [Newer College official configuration](/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/docs/2026-08-22_newer_college_maths_easy_official_results.md); [August 24 resolved GLIM parameters](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/newer_college_maths_easy_official_full_20260824/cbs_on_full/resolved_glim_params.yaml).
- [August 24 receiver replacement diagnostic](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/newer_college_maths_easy_cbs_receiver_edge_dedupe_20260824/REPORT.md); its estimator log contains 29 `active_alternative_replaced` events.
- [August 24 repeated-run audit](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/newer_college_maths_easy_cbs_on_four_repeat_20260824/REPORT.md).
- [Recent EIG-2 ON resolved settings](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eig2_revised_cbs_information_short_20260925/on_40s_queue_fixed/resolved_glim_params.yaml).
- [September 23 ETH1 reproduction report](/home/yeranis/repos/gnssFGO/GNSSFGO_GRANDTOUR_ETH1_CBS_ON_RATE015_CONTROL.md).

Status: historical configuration contracts recovered; source behavior chronology partly unresolved; no new runtime equivalence or accuracy claim. No fusion, tuning, source rollback, or new estimator execution was performed.
