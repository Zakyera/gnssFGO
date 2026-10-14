# FL4f fixed-lag ETH1 live milestone — frozen Git snapshot

This directory preserves the successful **matched first-30-second live** CBS-OFF / classic CBS-ON experiment, not a production receiver release. Read [the result](REPORT.md), [the FL4e failure repair](../fixed_lag_failure_fl4e_20261014/REPORT.md), and [the predeclared protocol](../fixed_lag_live_fl4d_20261014/PROTOCOL.md) before reusing it. The runtime-engineering ledger records the preceding FL1–FL4 investigations.

## What this Git snapshot contains

- `frozen_settings/source.env`: byte-identical source-launch environment from the successful run. Set `CBSMS_RUN_NAME` and `CBSMS_CBS_ENABLE` explicitly; those are the intended OFF/ON switches. The run-root, container, ROS port, and absolute paths are **historical machine-local values**, not portable defaults.
- `frozen_settings/receiver_off.json` and `receiver_on.json`: byte-identical copies of the actual receiver configuration JSONs. They differ by label/session; both select `source_wnoa`, `fl4a_fixed_lag_incremental`, 7 s lag, the independent 100 ms CPT7 support clock, and the same candidate binary launcher.
- `frozen_settings/receiver_fl4e.sh`: actual receiver launcher. It pins one CPU and points to the locally retained candidate executable, SHA-256 `53c96a2b7a762800cf2291166dc78520269098684411de641b52bee6a54dc5c9`. **The executable itself is not in Git.** Rebuild and verify it from the isolated FL3h/FL3i/FL4e sources before any reuse.
- `frozen_settings/glim_config/` and `kimera_profile/`: exact copied OFF profile trees from the successful run; OFF and ON trees were recursively identical. These preserve the ETH1 Hesai/CPT7 and front-stereo/CPT7 source configurations and calibration. Do not substitute a generic SPX-3 or monocular profile.
- `frozen_settings/preflight_validation.json`, `off/{validation,repeat_validation}.json`, `on/{validation,repeat_validation}.json`, `evaluation_full_summary.json`, and `belief_covariance_matched.json`: retained compact validation and scientific summaries. The detailed result tables and caveats are in `REPORT.md`.

The source environment and receiver JSONs were checked byte-for-byte against the successful run root, and both saved profile trees were checked recursively. No runtime journals, bag files, compiled binaries, receiver archives, native trajectories, or Rerun recordings are committed; those remain in the local run root recorded in `REPORT.md`. Their path and trajectory hashes make it possible to identify the original artifacts but do not make this repository alone a full data archive.

## Scientific and engineering boundary

The two source estimators and gnssFGO receiver co-ran at 0.10×. CBS-ON improved original GLIM, original Kimera, and **online/as-played** fused ATE on the identical 25.9 s common mask. The lower fixed-lag-finalized ATE is a later, retrospective output. This short result does not establish 1× throughput, arbitrary-duration stability, independent full-window prism accuracy, nonlinear stationarity, or strict FL3i whole-replay equivalence. The FL4e change is an isolated candidate repair, **not a change to the default gnssFGO build**. Do not silently promote it or tune source covariances from this milestone.
