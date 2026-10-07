# ETH1 live WNOA, frozen before the run

Authorized: short headless live ETH1 check, then matched full CBS-OFF/ON only
if the short check passes. Native GLIM + native Kimera + downstream fusion
run concurrently. This is not an offline reconstruction labeled live.

Mission 2024-10-01-11-29-55, offset 0, smoke 30 seconds, full 346 seconds,
playback 0.1x. Exact retained ETH1 Hesai/CPT7/front-stereo native profiles.
Classic source-to-source CBS, never live_revision. All current-smoother
revision diagnostics retained for downstream stable-ID factor replacement.

Receiver: exact previously validated isolated WNOA executable, full-history
iSAM2, 100 ms source-independent timestamp clock, Qc diagonal
[.00375,.00375,.00375,.015,.015,.015], identity pose prior sigma1e-5,
velocity zero sigma10, body angular velocity zero sigma3. No central IMU
factors, no acceleration/bias variables, no gravity initialization. IMU
timestamps provide the existing clock only. No covariance additions/scaling,
gating, robust kernel, health weighting, GNSS, or parameter tuning.

Strict nominal publication convergence is mandatory. Deep graph snapshot / LM
diagnostics and Rerun disabled; this changes diagnostic overhead only.
Before native launch, compare file/pipe streaming and a previously validated
online prefix byte-for-byte. Preserve production Combined-IMU/WNOJ defaults.

Short-run gates: both native sources finish with intended effective settings;
receiver publishes before playback finishes; no lost version, zero pending
endpoints at end (document unsupported startup beliefs separately); stable-ID
replacement accounting; no numerical-publication failure; bounded output age,
no sustained growing queue; exact replay of the newly captured journal.
Investigate any failed gate before authorizing the full runs automatically.

Report native GLIM, native Kimera ROS odometry, fusion online, fusion final
separately with existing no-scale prism/1s IE-TC evaluator and common masks.
No accuracy-based tuning or choosing source profiles based on the outcome.
