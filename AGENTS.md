# GrandTour/Newer College CBS and gnssFGO work

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
