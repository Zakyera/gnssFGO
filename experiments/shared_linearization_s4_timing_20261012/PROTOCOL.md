# Saved native Gaussian timing protocol

S4a measures the unchanged S2 covariance implementation using S3's saved ETH1
first40s weighted equations. Parent is `36ded67cc20275d55656842bc39a293810e390d4`.
No sensor replay, estimator run, covariance publication, optimization patch,
model change, tuning or push is authorized in this task.

Replay all 783 successful stage/commit records and the 20 recorded endpoint
queries. Build an original and a mechanically timer-instrumented standalone
Release executable against the pinned GTSAM. Preserve every numerical operation,
rank threshold, chart, factor identity, retirement and covariance helper.
Deserialize outside measured history/query intervals. The journal preserves
weighted information matrices, not original Jacobian-factor classes; reconstruct
Hessian factors and state this representation limitation explicitly.

First require all contribution counts to match the journal, full recorded query
graph information within relative Frobenius 1e-9, and all covariance/transport
blocks within relative Frobenius 1e-8. Original and instrumented output matrices
must be byte-identical. Reuse S2's existing tests; run focused ASan/UBSan and
parser negative controls. Preserve all source/runtime and journal hashes.

Then use one warm-up per executable followed by three interleaved measured
original/instrumented passes. Keep every epoch/checkpoint, including startup;
report early/middle/late, median/p95/max and totals. Timers separate transactional
history staging/commit, factor validation/copying, retirement elimination,
graph assembly, dense rank inspection, sparse elimination, joint marginal query,
chart transport, and relative covariance extraction. Nested timers are not added
to inclusive totals. Parsing, validation, output and I/O are separate overhead.

Determine required production query cadence from retained code/logs; if exact
unsampled requests are absent, report that limit and label extrapolation as an
estimate, not an end-to-end benchmark. Compare with S3's recorded query times and
B0's old LOCAL update/covariance timings only with their different-input and
dual-path limits. Do not claim replacement speed from this CPU benchmark.

Stop after attribution and one evidence-backed next engineering proposal. No
rank-check removal, solver substitution, provider switch or fresh live capture.
