# Pre-engineering preservation checkpoint

This snapshot saves the current research implementation before the next runtime
optimization. It does not implement a new change or certify a completed 160 s run.
Read `../../../GNSSFGO_RUNTIME_ENGINEERING_PLAN_20261011.md` for the one-task-at-a-time
execution ledger. The cbsms `docs/GITHUB_CHECKPOINT_20261011.md` and dedicated
`.repos` manifest identify the coordinated Zakyera commits.

## Actual build and current configuration

`results/gnssfgo_vw_jacobian_20261011/build/` contains the actual private replay
translation unit, source-model headers and tests. Its local build includes the
headers in `results/gnssfgo_publication_cache_20261010/build/`, notably
`FullPublicationRefinement.h`, `CachedGaussianEvaluation.h` and `FailureProbe.h`.
Preserve include precedence: candidate, preceding cache build, pinned runtime,
then gnssFGO source. `command_build.json`, `run.py` and provenance files retain
the original compiler invocation, mounts and library identities.

Reference executable (not uploaded):
`/home/yeranis/results/gnssfgo_vw_jacobian_20261011/build/replay`, SHA256
`62124b496ad6f854079218862acb149892aa452d4af908c50bc96067e49c9dbb`.
This is NOT the repository's older default CMake replay target.

The current experiment is Gaussian source-driven WNOA, no central IMU factors,
independent 100 ms supports, full-history iSAM2 and downstream stable-ID revision
replacement. Source-to-source CBS is classic `active_window_temporary`, not
`live_revision`. Exact effective settings are retained under
`runs/gnssfgo_eth1_vw_jacobian_live160_20261011/prepared/`; original ETH1 sensor
profiles are under `profiles/eth1/`. Do not substitute old persistent-CBS, WNOJ,
Combined-IMU, Huber or covariance-scale diagnostic settings from other archives.
Prior alternatives are retained as research history only, not enabled defaults.

## Contents and integrity

The 865 byte-identical archived files (about 14.5 MB uncompressed) include recent
controllers, launch files, profiles, private C++ sources, tests, protocols, audit
scripts and selected small validation summaries from October 8–11. Original
absolute path, SHA256, size and mode are in `ARCHIVE_MANIFEST.json`. No newline
normalization or estimator source rewriting was performed.

Run `python3 verify_archive.py` here to verify the archived bytes. Use
`python3 verify_archive.py --check-local-originals` on the original machine to
also check original paths; those paths need not exist on a restored machine.
`PRE_CHECKPOINT_REPOSITORIES.json` records committed HEADs and dirty-file hashes
before checkpoint commits (after adding the plan/instruction pointers).

The final live capture stopped at 143.790032 s due to the frozen queue limit,
then drained. Its report and partial validation are retained. Do not describe it
as a completed 160 s comparison or claim a new accuracy evaluation. The earlier
Release/sanitizer/replay results remain historical, not tests performed while
creating this checkpoint.

## Recovery limits

Recover into a new checkout/build location, never over frozen artifacts or dirty
work. Use the cbsms exact-commit manifest and read the run protocols before any
execution. Archived scripts still use historical absolute paths/mounts; they are
provenance inputs, not a portable automatic relaunch command.

Raw datasets, full revision/event journals, trajectories, graph snapshots, large
logs, Rerun recordings, binaries, generated builds, installed GTSAM/ROS libraries
and container images stay local. Copied vendor/runtime headers are not included;
recover the pinned runtime separately. Protected manifests record their hashes.
Thus GitHub provides a code/configuration/research checkpoint, NOT a complete
data/runtime backup or a proven fresh-machine bit-reproducible environment.

Existing source changes are checkpointed in their owning repositories without
modification. Verification here is archive/hash/syntax and Git hygiene; no new
GLIM, Kimera, gnssFGO, ROS run, full build, tuning or optimization is performed.
