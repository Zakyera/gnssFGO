# Shell-wrapper bookkeeping issue

The already-running full replay containers used the original run.sh, without
the /failed mount. While they ran, a read-only /failed mount was added to that
same shell file for the saved-graph diagnostic. Bash retained its original file
offset while waiting for Docker, then read a trailing fragment of the changed
file after Docker returned. All four WNOA OFF/ON primary/repeat wrappers show
outer-shell exit127 (a trailing hash fragment reported as command-not-found),
although their recorded replay child returncode is0 and all output validation
passes. This did not alter the already-created container, command, solver,
inputs or output. Do not conflate launcher exit status with estimator status.

The original wrapper was restored byte-for-byte before the WNOJ replays ended.
The saved-graph mount is now in a SEPARATE run_saved.sh. Both recorded saved-graph
executions had already finished successfully before this restoration. No replay
binary was rebuilt or output overwritten to address this shell-only issue.

Use immutable per-run launch scripts; never edit a script while Bash is waiting
inside it. The full replay's authoritative command/child status is recorded in
command_<case>.json, and validity is independently checked in validation_<case>.json.
