# FL4g — ETH1 first-120 s matched live fixed-lag extension

Status: **completed and validated** (2026-10-15); see [report](REPORT.md). One longer-window extension
of the completed [FL4f 30 s milestone](../fixed_lag_live_fl4f_20261014/REPORT.md).
No performance-driven tuning, model redesign, source change, or new sequence.

## Frozen controls

- GrandTour ETH1 `2024-10-01-11-29-55`, bag offset 0, duration **120.0 s**,
  0.10×. GLIM Hesai/CPT7 with the exact FL4f 5 s profile; Kimera front
  stereo/CPT7 with the exact FL4f 25-state profile. Both profiles must be
  byte-identical between OFF and ON.
- Source-to-source CBS remains `classic` and `active_window_temporary`, full
  dense `schur_relative_between` 6×6 covariance, scale 1, 0.20 s horizon,
  one pair/update, publish stride 1, no health weighting. Both publish in
  both modes. OFF disables external reception; ON enables bidirectional
  reception. `live_revision=false` in both.
- Same private FL4e receiver binary in both modes, SHA-256
  `53c96a2b7a762800cf2291166dc78520269098684411de641b52bee6a54dc5c9`.
  It uses a 7 s incremental fixed-lag source-WNOA graph, independent 100 ms
  CPT7 support after 59 samples, no central IMU factor, unchanged Qc/priors,
  Gaussian source factors and downstream stable-ID revision replacement.
  No GNSS factors, robust loss, gating, or covariance scaling.
- Run source and receiver concurrently, headless and with source factor graph
  inspectors disabled. The receiver is ready before bag playback. Use fresh,
  separate OFF/ON directories and source sessions. Preserve the FL4f data and
  every prior run unmodified.

## Required 120 s gate

Before playback verify exact binary/library/profile hashes, mission bags,
source launch mode and receiver JSON equality except labels/sessions. Refuse
an occupied ROS port, a pre-existing output, or any changed input. Each
source runner must verify resolved ROS parameters and Kimera flags **before**
playing the bag. OFF must show zero cross-source injections; ON must show
actual injections in both directions. Require native GLIM and Kimera output
through the requested tail, explicitly reporting gaps; receiver online poses
must span at least 118 s and advance during playback. Require exact received
belief-version/accounting conservation, source-to-journal full covariance
equality, zero pending/unsupported at the end, bounded supports/slots, queue
peak below 1000, and whole-support p95 service at most 0.8 wall seconds at
0.10×. Repeat each completed journal deterministically. Stop at the first
source, receiver, coverage, integrity, or runtime gate failure; preserve the
partial evidence and do **not** auto-retry or score it as complete.

After OFF validates, admit ON with the same executable/configuration. Then
score original/native GLIM, original/native Kimera ROS odometry, and fused
**online/as-played** trajectories, with finalized trajectories separate.
Use the same CPT7 IE-TC reference, prism lever arm, no-scale SE(3) alignment,
1 s RPE, and one common physical-time mask. IE-TC shares CPT7 data; any
independent AP20 prism coverage is a separate partial-window check. Report
source covariances and processing/queue statistics without tuning.

This bounded 0.10× test does not certify 1×, arbitrary duration, exact
full-history equivalence, or nonlinear stationarity. The FL4e fix remains
isolated from the default build.
