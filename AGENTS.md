# GrandTour/Newer College CBS and gnssFGO work

**Latest saved launch/settings (2026-10-17):** read
`GNSSFGO_GRANDTOUR_FL4V_LAUNCH_20261017.txt` FIRST for current ETH1 work, plus
`experiments/checkpoints/20261017_fl4v/README.txt`. This freezes the actual
FL4v private builds: temporary classic CBS, shared-native GLIM covariance,
FL4q Kimera, FL4e 7-second fixed-lag WNOA without central IMU, scales1, 0.10x.
It supersedes older configuration pointers below for this experiment, NOT
their historical evidence. The120s ON completed; the OFF reference stopped
early and only a105.4s common prefix is compared. No automatic retry, new run,
default promotion or claim that the OFF failure is fixed is authorized.

For the current incremental runtime engineering work, first read and update
`GNSSFGO_RUNTIME_ENGINEERING_PLAN_20261011.md`. Identify its task ID before
editing, and record evidence/status after each task. Do one hypothesis per patch;
do not silently execute the whole plan. Recovery sources/build instructions are
in `experiments/checkpoints/20261011/README.md`. This latest snapshot uses classic
`active_window_temporary` sources and source-driven WNOA without central IMU
factors; the older baseline descriptions below are distinct configurations.

For the latest bounded ETH1 fixed-lag live OFF/ON milestone, read
`experiments/fixed_lag_live_fl4f_20261014/MILESTONE.md` and its report before
reusing settings. It archives the exact source/receiver profiles and compact
validation evidence. The FL4e receiver repair remains an isolated candidate,
not the default build; the 30 s result does not certify longer or 1x operation.

Before running or interpreting CBS-ON GLIM/Kimera source experiments, read
`/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/cbsms/docs/CLASSIC_CBS_ON_RUNBOOK_20260925.md`.
It is the current operational reference for the shared classic CBS protocol
and the saved sequence-specific profiles. Do not silently substitute the
experimental source-to-source `live_revision` protocol.

Keep source CBS exchange separate from downstream gnssFGO stable-ID revision
replacement. For OFF/ON reporting, the primary comparison is always the
original/native GLIM trajectory, original/native Kimera ROS-odometry
trajectory, and fused gnssFGO trajectory in each mode. gnssFGO GLIM-only and
Kimera-only graphs are diagnostic receiver controls, not native trajectories.
Distinguish the receiver's causal as-played output from its final
retrospectively revised output, and never label offline replay as a live
co-running receiver.

Verify effective launch parameters, copied profiles, input identities, and
evaluation masks for each sequence. This instruction does not authorize a
new run, protocol change, or covariance tuning.

For the 2026-10-07 source-preservation checkpoint and latest experimental WNOA
build, read `experiments/checkpoints/20261007/README.md`. That archive preserves
the actual diagnostic build separately from the default CMake target, including
the live-run profiles and validation limits. Do not infer the latest experiment
from the older WNOJ/default-build paragraphs below.

For live/asynchronous GrandTour receiver work, read
`GNSSFGO_GRANDTOUR_CLASSIC_LIVE_BASELINE_20260929.md` and
`GNSSFGO_GRANDTOUR_LIVE_RECEIVER_RUNBOOK.md`, then use the saved
`config/grandtour_live_receiver.json` template. Source count is configurable,
not fixed at the synthetic test's four LiDAR/three camera example. Keep the
independent IMU-driven support clock separate from source arrival times.
The current live adapter observes full diagnostic revision logs while classic
source CBS remains unchanged. The matched SPX-3 120-second OFF/ON run validated
co-running sources and receiver at 0.10x with graph inspectors disabled; it is
not evidence of 0.3x, 0.5x, or 1x capacity or a universal sequence profile.
Do not call final retrospective paths live.

An opt-in source-driven WNOJ receiver is documented in
`GNSSFGO_SOURCE_WNOJ_IMPLEMENTATION_20261005.md`. It estimates acceleration
and omits central raw-IMU factors/initialization; it is only synthetically
validated, not a replacement for the frozen live baseline. Default mode remains
`imu_wnoj`. Do not enable `source_wnoj` in an experiment without authorization,
or mistake the older `--audit-use-imu false` control for this new model.
