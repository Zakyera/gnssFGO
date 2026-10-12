# Native Gaussian history prototype

S2 follows the saved shared-linearization plan and completed S1/B0. Parent
commit: 4b2804cf094c590c8404e3c5fecd00ce682b2dbc. Authoritative independent LOCAL,
MAIN, source profiles and receiver remain untouched. Only private CPU synthetic
builds/tests are authorized here, not live capture or covariance publication.

The prototype accepts a positively identified native subset of MAIN's cached
Gaussian slots at its recorded theta. It keeps its own Gaussian prior, eliminates
retiring keys with GTSAM, and transports endpoint joint covariance to the current
pose tangents before the unchanged Schur-relative helper. It never accepts MAIN's
marginal prior as native. It does not optimize a second nonlinear graph or
re-evaluate native LiDAR/IMU factors. The unchanged relative covariance helper
still evaluates its small endpoint-pose residual/Jacobians. Direct CBS information exclusion is not statistical
independence from the CBS-influenced MAIN state.

The test adapter supplies the snapshot lifecycle contract; S3 must implement and
validate the real pre-retirement MAIN hook and native-admission registry. No
production launch selector is added. The helper is Disabled by default; Shadow
must be selected explicitly and has no publication interface.

Predeclared gates: relative Frobenius error <=1e-9 for Gaussian/Schur algebra,
<=1e-8 for covariance, exact copy-through and deterministic repeats. Numerical
manifold Jacobian oracle uses central differences at 1e-6 with absolute <=1e-7.
Report absolute errors and condition numbers. No loading, scaling, pseudoinverse,
or new anchor. Reject underconstrained systems and changed prior reference charts.
Run private Release, UBSan, ASan+UBSan; do not claim GPU sanitizer coverage.

Protect the 680-file B0 manifest, S1 pins and six source HEADs before/after.
Build inside cbsms_ws_gpu using pinned GTSAM with existing Eigen ABI convention.
Private output root: runs/shared_linearization_s2_20261012. Refuse existing build
attempt directories. Stop after S2 report; S3 is a separate authorization.
