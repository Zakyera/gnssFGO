# E1b.4a checkpoint before shared linearization

The supervisor's proposal is saved in
`../../../GLIM_SHARED_LINEARIZATION_LOCAL_COVARIANCE_PLAN_20261012.md`.
This checkpoint preserves the working independent LOCAL solver before any
alternative covariance implementation.

## Recovery references

- Baseline commit: `3206123f4812d66158db58bd1f577167fe901875`.
- Baseline branch: `engineering/e1b4a-local-update-batching-20261012`.
- Annotated local tag: `checkpoint/e1b4a-before-shared-linearization-20261012`.
- Plan branch: `planning/glim-shared-linearization-20261012`.
- Main report: `../../../experiments/e1b4a_timing40_20261012/REPORT.md`.
- Integrity record: `PRESERVATION.json`.

The baseline commit retains the implementation/build scripts, synthetic tests,
exact-snapshot checker, timing harness and small validation evidence. The six
source repositories remain at their recorded clean HEADs. Private build source
snapshots, command recipes and binaries remain in their original directories,
whose hashes and locations are recorded in the integrity record.

Recover into a **new checkout/private build directory** at the baseline tag;
do not reset another worktree or overwrite frozen artifacts. Read the timing
protocol and verify its protected manifest and effective library paths before
any authorized run. The existing candidate build uses container-absolute
symlinks; copying only its top-level directory would not recover its dependencies.

## Retained local artifacts

Workspace: `/home/yeranis/repos/V4RL/cbs_gtsam4.3`.

- Working private GLIM build:
  `runs/e1b4a_local_update_batching_20261012/release_2`.
- Pinned dependency build:
  `runs/e1b_local_profile_20261012/release_2`.
- Completed first-40-second capture:
  `runs/gnssfgo_e1b4a_timing40_20261012`.
- Its frozen run profile/manifest: `prepared/` within that capture.
- Receiver executable:
  `/home/yeranis/results/gnssfgo_e1_e2_correctness_audit_20261011/candidate/replay`.

The independent LOCAL batching selector remains default-off and privately
enabled in the saved timing profile. This is not a production promotion.

## Backup boundaries

This task creates a local recovery tag, integrity record and committed plan.
It does **not** push to GitHub, clone runtime binaries/data into Git, or create
an off-machine backup. Existing raw data, recordings, journals and builds are
preserved in place. The earlier October 11 archive remains complementary
recovery evidence; it does not replace the October 12 build identities here.

Verification in this task is hashes, source HEAD/status checks, documentation
review and Git hygiene. No new estimator execution, build, Release/sanitizer
test, covariance change or accuracy experiment is implied.
