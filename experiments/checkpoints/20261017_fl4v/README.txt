ETH1 FL4v recovery archive (2026-10-17)
=====================================
Read ../../../GNSSFGO_GRANDTOUR_FL4V_LAUNCH_20261017.txt first.
manifest.json maps every archived byte copy to its exact original path, SHA256
and size, and records the actual private binary identities (binaries local).
verify_archive.py checks hashes and Python/shell/JSON/XML syntax without
running/importing launch controllers; GLIM JSON-with-comments is hash-checked.
archive_checkpoint.py is the one-shot archival recipe; it refuses overwrite.
The scripts under snapshot retain historical absolute paths; do NOT execute
them in place or mistake them for a portable fresh-run launcher.

Source builds:
- kimera_fl4q: actual generated VioBackend.cpp, CBS/GTSAM header overlays,
  fallback/prior/audit code, build commands, and original Release/sanitizer
  receipts. This overlay, not base Kimera HEAD alone, produced the pinned DSO.
- glim_s4: actual shared-native provider sources/overlays and build commands.
  Its original E1b dependencies remain pinned by existing repository archives
  and the binary identities; do not silently substitute unmodified libraries.

Receiver build provenance:
The receiver is candidate_live from fixed_lag_failure_fl4e_20261014, not the
default CMake executable. Its actual Live.cpp is committed in
experiments/fixed_lag_live_fl4a_20261014; the FL3i Receiver.h/SlotISAM2 sources
and FL2b lifecycle dependencies remain committed in this repo. Use the repaired
experiments/fixed_lag_failure_fl4e_20261014/RecordedFixedLagSmoother.cpp instead
of the older FL3i file when reconstructing this candidate. The archived FL4a
build command is provenance for that build recipe, not a claim that the old
unmodified command directly builds FL4e. Preserve original include order:
FL3i, FL2b, receiver_endpoint_reuse_e2b, E1/E2 correctness candidate, VW Jacobian
build, publication-cache build, then saved runtime/GTSAM/ROS2 dependencies.
The three custom result-directory include trees are archived under
receiver_dependencies. See experiments/fixed_lag_failure_fl4e_20261014/REPORT.md
for the exact repair. Rebuilding was NOT performed during checkpointing.

Historical evidence remains in fixed_lag_live_fl4s (30s matched), fl4t (failed
OFF), and fixed_lag_live_on_fl4v (completed120s ON). The archive's evidence
directory contains actual FL4v acceptance, conservation, coverage, repeat and
evaluation records. common_prefix_off_on.json beside the FL4v report is the
105.4s OFF/ON comparison. Neither the receiver's delayed finalized output nor
that common prefix may be relabeled as full120s online OFF/ON.

No estimator changes/default promotion here. Captures and expensive artifacts
remain in their original paths. Archive verification is not a new C++ test or
performance run; existing test receipts retain their original scope and dates.
