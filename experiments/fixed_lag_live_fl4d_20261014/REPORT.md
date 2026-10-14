# FL4d — matched ETH1 first-30-second CBS-OFF/ON comparison: stopped

## Intended comparison and frozen settings

The [predeclared protocol](PROTOCOL.md) asks whether classic bidirectional CBS changes the **original/native GLIM**, **original/native Kimera ROS odometry**, and **online/as-played gnssFGO fusion** over one matched ETH1 30-second window, and how the actual dense 6×6 source beliefs change. The already completed and validated FL4c CBS-OFF capture was the control. The only new estimator run was one CBS-ON live source/receiver capture at 0.10×. No covariance, WNOA, source, or receiver mathematics were tuned or changed.

The ON [static preflight](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_fixed_lag_live30_off_fl4c_20261014/prepared/preflight_on.json) passed: the receiver JSON differs from OFF only in labels/source sessions, and the source launch uses the same pinned S4d/LOCAL overlays, ETH1 Hesai/front-stereo/CPT7 profiles, 30-second bag window, and Gaussian source covariance settings. The intended source difference is classic CBS receive/injection enabled bidirectionally, while diagnostic publication remains on in both cases. The [effective before-playback check](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_fixed_lag_live30_off_fl4c_20261014/prepared/pilot30_on_source/effective_mode_check.json) passed for loaded libraries, actual subscriptions, `active_window_temporary`, `schur_relative_between`, and `live_revision=false`. Downstream used the unchanged pinned FL4a 7-second fixed-lag, source-driven WNOA receiver with independent 100 ms support clock, no central IMU factor, full covariance scale 1, Gaussian factors, and stable-ID replacement. Receiver SHA-256: `15a2ae458f5142c0b3341e9e444b25fc41f05a263cefbbb877c18c4a527718bc`.

The protocol would have scored a common full-window CPT7 IE-TC mask, with its shared-IMU limitation stated, and the independent prism only on its approximately 11.2-second coverage. No scoring was performed because ON did not complete.

## Actual result and failure evidence

**Stopped: no valid 30-second OFF/ON scientific comparison.** The ON receiver exited during playback after approximately 14.78 seconds of the input IMU stream (first retained IMU `1727775008003475000`, last committed event time `1727775022783478000`). The [controller failure marker](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_fixed_lag_live30_off_fl4c_20261014/prepared/pilot30_on_control/failed.txt) says `receiver stopped during playback`; no ON `complete.json` exists. The [receiver log](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_fixed_lag_live30_off_fl4c_20261014/prepared/pilot30_on_receiver/receiver.log) gives the decisive error:

```
FL4A_STREAM_FAILURE Requested non-existent variable from VariableIndex: 120 71
```

GTSAM symbol code 120 is `x`, so the requested missing key is `x71`. The last successful [stream-progress event](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_fixed_lag_live30_off_fl4c_20261014/prepared/pilot30_on_receiver/receiver/stream_progress.csv) is 8128, an IMU event that published support 142 and finalized/retired pose index 71. Its [commit record](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_fixed_lag_live30_off_fl4c_20261014/prepared/pilot30_on_receiver/receiver/archive/commits/8128/records.txt) shows `finalized_incremental,71` and the absorbed GLIM/Kimera factor identities. The next recorded journal event is a Kimera belief revision, but the receiver did not emit a commit for it. This establishes the boundary of the failure; it does **not** prove that this specific revision or CBS covariance caused the missing-key error. The exact internal VariableIndex/factor-lifecycle defect is not yet established.

The latest [transport status](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_fixed_lag_live30_off_fl4c_20261014/prepared/pilot30_on_receiver/transport_status.json) had 1,473 IMU samples, 6,646 beliefs (5,546 GLIM; 1,100 Kimera), 142 published poses, maximum queue 21, and zero queued events. It was a recent snapshot, not a final conservation ledger. The adapter's later broken-pipe error followed the receiver exit. There is no evidence here of a growing transport backlog or completed source/receiver run. The controller aborted the source run; the ON source data, trajectories, and covariance logs are partial and **must not** be scored or compared as 30-second results.

## Preservation, limits, and decision

The completed FL4c OFF artifacts remain unchanged. A read-only post-failure rehash found all 814 protected original inputs/artifacts unchanged; the owned ON source, receiver, and ROS-master processes had stopped. No automatic retry, saved-journal replay, model edit, covariance analysis, or accuracy comparison was made after failure. Existing working-tree changes unrelated to this run were preserved.

This is an **engineering blocker in the experimental fixed-lag receiver under the ON event workload**, not a finding about whether CBS-ON improves accuracy or about source covariance calibration. The bounded next diagnostic, if separately authorized, is to reproduce and inspect the missing `x71` VariableIndex/factor-lifecycle transition around event 8128 on the saved prefix, without launching a new source experiment or changing estimator mathematics. Only after the receiver passes that gate should a fresh matched ON capture and the predeclared native/fused/covariance comparison be attempted.

gnssFGO HEAD at this report: `d3d152a28440f97ced3bd7a50ee072874beafe19`; this task has uncommitted experiment harness/report files and makes no claim of a clean worktree.
