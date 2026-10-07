# CBS / gnssFGO research checkpoint — 2026-10-07

This is a preservation checkpoint, not a new algorithm, release certification,
or authorization to run an experiment. Existing source code and reports were
saved without retuning or changing estimator behavior.

## What is here

- `results/gnssfgo_eth1_local_prediction_20261007/build/wnoa/` contains the actual
  experimental WNOA replay translation unit and local headers used by the
  retained live ETH1 experiment. **This is not the normal CMake replay target.**
  Its five C++ source/header files are byte-identical to the local originals.
- `results/` also preserves the local-prediction numerical validation, the
  prediction/interaction audit, and the frozen covariance counterfactual and
  source-isolation scripts, protocols, build commands and small JSON summaries.
- `live_run/` preserves live-run controllers, frozen transport copies, explicit
  receiver configurations, protected hashes and retained validation summaries.
- `profiles/eth1/` and `profiles/spx3/` preserve the source-profile files named
  by the saved launch environments. These profiles are sequence-specific, not
  interchangeable defaults.
- `ARCHIVE_MANIFEST.json` maps every archived file to its original local path,
  original SHA-256, archived SHA-256, size and mode. The few text files lacking
  a final newline have one added in this archive, explicitly recorded by
  `final_newline_added`. Remove exactly that final byte when reconstructing an
  original whose protected checksum must match. No original file was changed.

## Build and restoration boundaries

The archived scripts retain their original absolute paths and Docker mounts.
They are provenance/reproduction inputs, **not a portable one-command launcher**.
Review their protocols and restore the required inputs/mount layout into a new
working location before executing anything. Do not rerun their preparation
steps over frozen artifacts. Historical helper branches may reference earlier
diagnostic files not included here.

The validated WNOA binary SHA-256 was
`fb56ffc5c6c9cb575d6129289688caeabb4e3baba32e429b857f3d6a30e50d1f`.
The exact successful compiler invocation is retained in
`results/gnssfgo_eth1_local_prediction_20261007/command_stable_build_wnoa.json`.
It uses the pinned local runtime under
`/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime`;
`live_run/protected_manifest_v2.json` records library/executable hashes.
Container-image identities and mount details are retained in the command and
execution-environment JSON files. Binaries, container images, installed GTSAM
headers/libraries, raw datasets, event streams, graph snapshots, large logs,
trajectories and Rerun recordings are **not** uploaded. They remain local.
This repository alone therefore does not provide a complete data/runtime backup
or establish a fresh-machine bit-reproducible build.

## Scientific / operational status

Read `../../../GNSSFGO_ETH1_LIVE_WNOA_OFF_ON_20261007.md` (from the repository
root, use `GNSSFGO_ETH1_LIVE_WNOA_OFF_ON_20261007.md`) for the evidence and limits.
The matched bounded 120-second comparison exists; the attempted full 346-second
OFF run did not complete successfully. Do not describe this as a validated full
ETH1 OFF/ON experiment or as demonstrated real-time throughput.

Classic source-to-source CBS and downstream stable-ID replacement are separate.
The latest experimental receiver uses source-driven WNOA with no central IMU
measurement factors; the older documented Combined-IMU/WNOJ baseline remains a
different configuration. The saved classic ETH1 source profile deliberately
unsets the experimental Kimera IMU-support override. Do not replace these
explicit run settings with assumptions from earlier ALL_RIGHT pilots.

Native GLIM, native Kimera ROS odometry, fused online/as-played, and final
retrospectively revised fusion are distinct outputs. Retrospective fusion must
never be labeled online. Covariance counterfactuals are diagnostics, not adopted
production weighting. No robust loss or new weighting method is introduced by
this checkpoint.

## Companion repositories and checks

The coordinated branch is `checkpoint/grandtour-wnoa-20261007` in Zakyera's
`gnssFGO`, `cbsms`, `cbs`, `liorf`, `glim`, `glim_ros1`, `Kimera-VIO` and
`Kimera-VIO-ROS`. The `cbsms` checkpoint supplies the exact-commit workspace
manifest and the recovery index. Treat this as an archival snapshot of the
current research tree, including opt-in diagnostics, not a claim that every
experimental option is a production baseline.

For this checkpoint, source hashes, file types/sizes, credential patterns and
script syntax are checked. Existing test/experiment results are retained as
historical evidence; no new estimator run or full build is performed merely to
push this snapshot.
