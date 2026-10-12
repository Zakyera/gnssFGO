# ETH1 shared native covariance provider 160 second validation

The user authorized one 160-second ETH1 CBS-ON capture at 0.10x on
2026-10-12. This extends the validated S4d build without changing estimation.
The question is whether its complete-GLIM timing and correct native-information
history remain sustainable beyond the previous first 40 seconds.

## Frozen configuration

Use S4d release_2 libglim_ros SHA256
0114b435785ae85f58a69f1156abd1a2a320dc7138ca0b2dd4bfd56715d4ebea and
libgtsam_points SHA256
42fb1e8a931ebc86a06ae41a2eaf6ff39b9efbe5a1a4fd9737ee06edb2274228.
The E2 receiver remains SHA256
9d2ef832941c5ff2592da8428cb80104352590bcfea50c187624aaf4e90d60e0.
No rebuild or installed/default-library change.

ETH1 mission 2024-10-01-11-29-55, offset0, duration160, rate0.10;
Hesai/CPT7 GLIM5s and front stereo/CPT7 Kimera25states; Kimera experimental
IMU override unset. Classic active_window_temporary, source live_revision
false, horizon0.20s, one pair/update, dense schur_relative_between, scale1,
Gaussian and existing native safeguards. No synchronous graph inspectors.
Full diagnostic revision export and bounded diagnostic sample counts unchanged.

Private GLIM_SHARED_NATIVE_PROVIDER=shared_native replaces only the ordinary
outgoing LOCAL covariance path and disables independent LOCAL. MAIN still
determines the mean. Later diagnostic revision covariance remains MAIN.
Native Gaussian blocks/prior are separate from explicit CBS information, but
their MAIN-conditioned charts do not establish statistical independence.

Receiver: source_wnoa, no central IMU factors, independent100ms clock, existing
59 timestamp startup, full-history iSAM2 QR, E2 exact-Values reuse, Qc
[.00375,.00375,.00375,.015,.015,.015], pose/velocity/omega prior sigmas
1e-5/10/3. Covariance scale1, no new gates/robust loss/health weighting.

## Acceptance and stop conditions

1. Verify S4d and prior protected artifacts, exact effective profiles/libraries,
   receiver identity, free resources and no competing source/controller.
   Rerun the same focused Release fixture without overwriting earlier tests.
2. Fresh root and one launch only. Stop on source/provider/history failure,
   reset, receiver non-nominal publication, queue1000, existing deadline or
   original validation failure. No retry or automatic corrective change.
3. Require full native-source and fused coverage using the existing duration
   checks. Reconcile all revisions, stable-ID replacements and unsupported
   intervals. Keep normalized publication gradient <=1e-5. Compare one exact
   saved receiver-journal replay, including online and final products.
4. Verify every ordinary full6x6 wire covariance equals the provider output,
   finite/symmetric/positive definite at original tolerance, zero independent
   LOCAL and zero provider GPU reevaluation. Each native contribution must be
   active, absorbed or removed exactly once; report bounded active history and
   retiring counts. Do not mislabel the last query as the final callback.
5. Report complete scan and MAIN/covariance/revision timings, every10s plus
   early0–40/middle40–120/late120–160. Desired full and late scan p95<800ms
   retains 20% headroom against the paced nominal1s scan interval. Missing this
   target is a performance miss, distinct from functional failure. Record source
   availability delays, receiver age/queue, memory and throughput. Do not add
   overlapping timers; native capture/commit remains included in MAIN.

Compare first40 timing with retained S4d/B0 and longer timing with preserved
E1/E2 live160 where available, explicitly as separate coupled captures, not
same-input causal attribution. No ATE tuning or new estimator experiment.
Native GLIM, native Kimera ROS odometry, fusion online/as-played and fusion final
remain separate. Preserve all prior artifacts. Update both plans/report and stop.
