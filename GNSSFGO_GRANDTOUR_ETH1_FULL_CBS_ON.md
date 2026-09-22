# Full GrandTour ETH1 CBS-ON source and gnssFGO comparison

Date: 2026-09-22

## Result

Bidirectional CBS belief exchange was active throughout the full ETH1 source
run. Kimera continued to publish nearly to the end, but it did **not** yield a
healthy full trajectory: the last two transitions include 77.7 m and
1,104.7 m jumps. The unchanged gnssFGO GLIM+Kimera graph failed near 82 s,
and therefore there is **no valid full-sequence fused ATE/RPE**. This is an
earlier failure than the CBS-OFF joint graph's approximately 220 s failure.

The answer to “does Kimera survive the whole sequence?” depends on the
meaning of survive: it publishes 1,491 poses spanning 344.4 s, but its late
trajectory is physically implausible. Publishing is not evidence of usable
odometry.

## Matched source experiment

- Mission: GrandTour `2024-10-01-11-29-55` (ETH1), 348 s playback at 0.30×.
- GLIM: Hesai + shared CPT7 IMU, `ALL_RIGHT` support.
- Kimera: AlphaSense front stereo + the same CPT7 IMU, `ALL_RIGHT` support.
- Same GLIM and Kimera profile directories and 0.20 s target-horizon belief
  export as the previous full CBS-OFF run. The copied GLIM `config.json` and
  Kimera `ImuParams.yaml` are byte-identical across the two source runs.
- Changed source condition: `CBSMS_CBS_ENABLE=true`,
  `CBSMS_CBS_DIRECTION=bidirectional`. Both receive flags were true;
  GLIM used `inject_persistent`. No EKF or presentation graph was launched.
- The source runner passed its own completion/coverage checks. Recorded
  exchange: 2,814 GLIM-to-Kimera messages, 257 queued GLIM-to-Kimera factors,
  and 1,200 Kimera-to-GLIM persistent injections. The latest applied
  GLIM-to-Kimera factor reached Kimera target 1477, 13 indices before its
  final index. These counts demonstrate actual exchange, not just publishing.

The unchanged source runner is
`/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/tools/run_grandtour_eth1_glim_kimera.sh`.
Its recorded copy, ROS logs, native trajectories, bag, resolved parameters,
and metadata are in
`/home/yeranis/repos/V4RL/cbs_eth1_cbs_on_full_20260922/sources/all_right_full_cbs_on/`.
The CBS-OFF comparison is
`/home/yeranis/repos/V4RL/cbs_cpt7_all_right_full_eth1_20260922/sources/all_right_full/`.

The same unique-edge exporter retained every connected component, selected
the latest approximately 0.20 s horizon belief per array, and deduplicated
exact edges. CBS-ON yielded 2,527 GLIM and 1,471 Kimera beliefs under
`/home/yeranis/repos/V4RL/cbs_eth1_cbs_on_full_20260922/beliefs/`.
The prior CBS-OFF export had 3,319 GLIM and 1,474 Kimera beliefs. The
GLIM source stream is therefore materially sparser under this CBS-ON run;
the graph comparisons are not identical-factor ablations.

| Retained CBS-ON input | SHA-256 |
|---|---|
| Native GLIM `odom_imu.txt` | `beaeb7a8f8ee827ad54112be425a53c81fe87469a960906430a5dd25d137044e` |
| Native Kimera `kimera_odometry.csv` | `2c7ecd5ae533fb366a2c5a1030ea28c6dc039ca57eaa3fa94f588c402e9e9451` |
| GLIM relative beliefs | `fc1996eeb81c4e5f18020cb497429850d1d890c01bbe0f63133e102eb0731ff0` |
| Kimera relative beliefs | `07e23a77434cdf8528731791013833f3d64536441335d2018f2d1f554104de90` |

## Native source results

The same AP20 Leica prism/lever-arm/no-scale SE(3) Kabsch evaluator and 1 s
CPT7 Inertial Explorer relative-pose evaluator were used. These are the
available-output scores; matched GT coverage is reported because it is not
the complete playback.

| Native source | CBS | ATE RMSE (m) | 1 s t-RPE (m) | 1 s r-RPE (deg) | Full output path (m) | Matched prism coverage (s) |
|---|---|---:|---:|---:|---:|---:|
| GLIM | OFF | 0.028346 | 0.018774 | 0.289044 | 202.941 | 226.5 |
| GLIM | ON | 0.033789 | 0.027232 | 0.478518 | 204.480 | 226.3 |
| Kimera | OFF | 8.595737 | 0.200726 | 0.528511 | 175.863 | 192.8 |
| Kimera | ON | 5.949668 | 0.266666 | 0.718235 | **1,372.959** | 192.5 |

The lower Kimera ATE under CBS-ON does not establish a healthy improvement:
its local RPE worsened, and the full output path is implausible. Consecutive
Kimera poses near the end jump 77.7 m, then 1,104.7 m; the largest jump is
between timestamps `1727775347453505513` and `1727775352553477813` ns.
Both ON and OFF Kimera outputs have intermittent gaps; ON has 19 gaps above
0.25 s (10 above 1 s). Different sampled timestamps and prism coverage also
limit direct ATE comparison.

## gnssFGO graph results

The same full iSAM2 continuous WNOJ/IMU graph, native full 6×6 belief
covariances, and source-factor implementation were used. The causal timeline
inserts each available belief using its `record_timestamp_ns`; its factor
endpoints remain at the physical source timestamps. No GNSS, robust loss,
gating, covariance scaling, or new fusion mechanism was added. The physical
timeline joint run is a diagnostic control only.

| CBS-ON graph | Timeline | Accepted before termination | Result |
|---|---|---:|---|
| GLIM only | causal record time | 2,527 GLIM | Completed: 2,672 support states; 199.192 m internal path |
| Kimera only | causal record time | 398 Kimera | Failed near `x800`, about 83 s |
| GLIM + Kimera | causal record time | 709 GLIM + 389 Kimera | Failed near `w779`, about 82 s |
| GLIM + Kimera | physical time control | 804 GLIM + 408 Kimera | Failed near `w816`, about 85 s |

The three failures raised GTSAM `IndeterminantLinearSystemException`.
Before the causal joint failure, estimated speed first exceeded 10 m/s at
support 773 and 100 m/s at support 774. The physical-time joint graph also
fails, so delayed causal insertion is not by itself an explanation.
Kimera-only fails in the same early region, while the GLIM-only graph
completes. This localizes the graph problem to cases containing Kimera
beliefs but does not prove a unique root cause. In particular, bidirectional
CBS exchange makes the source beliefs informationally dependent, whereas the
current gnssFGO graph does not model that cross-source dependence.

The completed CBS-ON GLIM-only graph scores 0.395226 m ATE,
0.026835 m 1 s t-RPE, and 0.459570 deg 1 s r-RPE. Its matched prism
coverage is 202.2 s, less than native GLIM's 226.3 s, so this is not a
same-support superiority claim. The CBS-OFF GLIM-only graph reference was
0.119855 m ATE, 0.016562 m t-RPE, and 0.313075 deg r-RPE.

No full gnssFGO Kimera-only or fused trajectory metric is reported: both
graphs terminate early. Their incomplete prefixes must not be scored as if
they represented the full ETH1 mission.

Graph output and failure-prefix files are retained under
`/home/yeranis/results/gnssfgo_grandtour_eth1_cbs_on_20260922/`.
Native evaluation JSON files are under
`/home/yeranis/repos/V4RL/cbs_eth1_cbs_on_full_20260922/metrics/`.

## Interpretation and validation

CBS-ON exchange is demonstrably active and Kimera keeps publishing, but
this frozen full ETH1 configuration does **not** deliver a usable gnssFGO
fusion result. The CBS-ON Kimera source shows a catastrophic late jump,
and Kimera-bearing graphs become indeterminate much earlier than in the
CBS-OFF full run. The source feedback, sparse GLIM belief export, native
Kimera failure, unmodelled shared information, or their interactions remain
possible contributors; this experiment alone does not separate them.

Validation: the source runner completed its internal coverage and
bidirectional-injection assertions; the two source profile copies match the
CBS-OFF profiles; both belief exports contain finite, positive-definite
6×6 covariances; the unchanged gnssFGO GLIM-only control completed. The
gnssFGO code and source estimator configurations were not edited for this
comparison. A full fused ATE/RPE validation is impossible because the joint
graph did not complete.
