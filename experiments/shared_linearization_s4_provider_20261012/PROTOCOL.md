# S4d: isolated native-Gaussian publication provider

Authorized scope: one ETH1 first40s CBS-ON capture at 0.10x after correctness and
startup gates; compare with the retained B0 capture. No automatic run retry.

## Single change and preserved boundaries

The private opt-in provider replaces only the existing ordinary outgoing LOCAL
covariance route. MAIN means, pair selection and covariance extraction formula
remain unchanged. The independent nonlinear LOCAL sidecar is not allocated in
this mode. S4c Gaussian history, chart transport, rank certificate and original
eigenvalue fallback are reused. No native nonlinear factor is reevaluated by the
new provider. Invalid history or unsupported configuration stops admission/run;
there is no covariance fallback to MAIN.

Existing later diagnostic revisions use MAIN covariance, unlike ordinary first
publications. Preserve this code path. This is explicitly NOT a downstream
revision covariance-policy experiment. MAIN-conditioned native Jacobians are
not independent CBS-OFF information; equality with independent LOCAL is not an
acceptance condition. Coupled source trajectories can change.

## Frozen experiment

Reuse B0 ETH1 mission 2024-10-01-11-29-55, offset0, duration40, rate0.10;
Hesai/CPT7 and front stereo; GLIM5s/Kimera25states; current Kimera IMU override
unset; classic active_window_temporary, no source live_revision, 0.20s outgoing
horizon and one pair/update. Receiver remains E2 cached WNOA/no central IMU,
100ms independent support clock, full-history iSAM2, Qc/priors/scale1/Gaussian
unchanged. No graph inspectors. Preserve full diagnostic revision logging.

## Gates declared before execution

1. Existing lifecycle/native-history regressions plus adapter tests pass in
   Release and ASan/UBSan. Verify full covariance within 1e-10 relative error
   against the unchanged extraction helper, transactional failure, no additional
   native evaluations and unchanged MAIN. Default-off control remains exact.
2. No-data startup, effective parameters, loaded private DSOs, frozen receiver,
   input hashes and zero prior controllers verified before playback.
3. One fresh output root; stop on provider error/reset, source failure, receiver
   non-nominal publication or original functional/lifecycle validation failure.
4. Full saved receiver replay must retain existing exact-repeat checks.
5. Report complete-scan median/p95/max and early/late bins; MAIN, native history,
   ordinary covariance and revision time separately; do not add nested timers.
   Predeclared B0 complete-scan p95=437.376ms, 0.10x budget gate p95<800ms.
   Speedup is measured only if complete-scan latency improves; paced total wall
   duration alone is not an unpaced throughput measurement. One matched-setting
   pair is not a statistically controlled same-input benchmark.

Protect original sources, libraries, B0/S3/S4 artifacts. All builds and runtime
switches are private. Save report and update both plans, then stop. No push.
