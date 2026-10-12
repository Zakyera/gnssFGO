# E1b.3b: repaired ETH1 first-40-second profiling attempt

**Outcome: blocked before playback. No profiling or trajectory result was produced.** Exactly one CBS-ON capture was launched; the protocol forbids an automatic second attempt after a source failure.

## Frozen attempt

- Branch: `engineering/e1b-profile40-fixed-20261012`; parent `391e936dd846a831d9cffc343c2b21b9f53b2171`.
- Prepared run: `/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_e1b_profile40_fixed_20261012/prepared`.
- Intended ETH1 bag seconds 0–40 at 0.10×, Hesai/front stereo/CPT7, classic CBS-ON with active-window-temporary external factors, unchanged E1/E2 source-driven WNOA receiver, 100 ms independent support clock, no central IMU, no covariance scaling or robust loss. The prior E1b.3 source/receiver settings were retained; only output paths, ROS port 11408, source session and repaired private GLIM wrapper path changed.
- Repaired `libglim_ros.so` SHA-256: `567bb4bc3f0e1dacf8994ecf7ca4aa12ff128e0f91640dda3adf760cd7c85df8`; wrapper allocation: 3344 bytes. Preflight passed 668 protected hashes, 18 regression tests, library linkage, bag provenance and effective launch-parameter comparison (`preflight/validation.json`).

## Exact failure

The source started GLIM and Kimera, then `verify_effective.py` failed its pre-playback process-map check at line 64:

```text
AssertionError: libglim.so
```

The generated gate demanded that **all six** GLIM-related DSOs appear in `/proc/<glim-pid>/maps` under the new `fixed_release` directory. That was an incorrect path assumption introduced by the mechanical derivation of this fresh harness. Only `libglim_ros.so` is a regular file in `fixed_release`; `libglim.so`, `libcbs.so`, `libe1b_profile.so`, `libgtsam.so.4.3a1` and `libgtsam_points_cuda.so.1.2.1` are absolute symlinks to the unchanged `e1b_local_profile_20261012/release_2` libraries. Linux process maps report those resolved targets, not the symlink paths. The retained E1b.3a **actual process map** (`runs/e1b_startup_fix_20261012/fixed_on/maps.txt`) confirms the repaired wrapper loaded from `fixed_release/libglim_ros.so` and the dependencies from `release_2`. The current capture did not save its maps because the assertion precedes `effective_mode_check.json`, but the identical library set and its preflight linkage support this diagnosis.

This is an **admission-gate false rejection**, not evidence that GLIM, Kimera or the receiver failed on sensor data. The source controller terminated both estimators after the assertion. There is no `rosbag.log`, source `complete.marker`, trajectory, or LOCAL timing profile. The receiver's transport status records 0 IMU samples, 0 GLIM beliefs, 0 Kimera beliefs and 0 published poses. Its subsequent failure is a consequence of the stopped source.

Evidence: `profile40_on_control/source.log`, `profile40_on_control/failed.txt`, `profile40_on_source/estimators.log`, `profile40_on_receiver/transport_status.json` and `preflight/ldd.log` under the prepared run root. The previous E1b.3 capture and repaired-library startup artifacts remain untouched.

## Validation and disposition

- Post-stop rehash: all 668 protected entries still match the preflight manifest.
- No GLIM, Kimera, bag-playback, receiver or ROS master process remains in the owned containers.
- No estimator settings, factors, covariances, solver or profiling implementation were changed in this attempt. No accuracy, throughput, LOCAL-subphase or speedup conclusion is possible.
- A **future separately authorized** capture should use a symlink-aware map gate: require the repaired wrapper under `fixed_release`, require each dependency under its verified real target in `release_2`, and verify their pinned hashes. Test that gate against retained `fixed_on/maps.txt` before starting playback. Use a new output root; do not overwrite this failed attempt. No rerun was made here.

**Status: BLOCKED_PRE_PLAYBACK_GATE.**
