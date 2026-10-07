# Frozen ETH1 WNOA prediction/uncertainty audit

Authorization: user approved Step 2, prediction and uncertainty validation using
the existing approximately 120 s ETH1 OFF/ON runs. No source or receiver replay,
optimization, tuning, production integration, or live experiment is authorized
by this analysis. Only reconstruct and linearize immutable saved graphs.

Inputs: gnssfgo_eth1_local_prediction_20261007/{off,on}_wnoa and their original
gnssfgo_eth1_wnoa_wnoj_120s_20261006/inputs/{off,on}_events.csv. Retain their
hashes, source header/runtime identities and the initial repository status.

All publications: compare current state, latest support not after the latest
arrived source endpoint F, and the right support of F (the source-connected
boundary) with the next snapshot at exactly the same timestamp. Also compare
the old current state with first endpoint coverage and final reconstruction.
Startup without source information and end-of-window censoring remain explicit.

Covariance: reconstruct selected saved graphs using the exact frozen WNOA
factor/endpoint adapter, initial priors, full source covariance, revision ledger
and factor slots. Do not optimize. Select every 20th source-informed publication,
first/last, plus five largest each of endpoint delay, pure prediction duration,
position/velocity/orientation/angular-rate revision, speed and angular rate.
Include selected original full-audit snapshots to verify dense payload probes.
This is a stress sample, not an unbiased population calibration assessment.
Recover full 12D joint state marginals and endpoint-support cross blocks by QR.
Validate source-free suffix covariance against F P F^T + Q constructed from
the existing motion-factor Jacobians. Report process versus inherited covariance.

Interpolation: sample the last five support intervals of every snapshot at
10 ms, and the full final history. Compare arc length/chord, excursions and
finite-difference velocity with the saved support velocities. Do not call a
smooth path accurate. Preserve dense poses for reproducibility.

Interpretation: revision size is not truth error; revision-to-marginal ratios
are not NIS/NEES or chi-square tests. A changed factor version is replacement,
not necessarily independent new information. No GT-based parameter selection.
Document conditional-mean interpolation and shared-source-information limits.
Report observed maxima/distributions; do not invent a physical pass threshold.

Validation: synthetic stationary and moving-state covariance/transition tests,
exact/interpolated endpoint checks, factor key/dimension/cost reconstruction,
all available selected dense payload probes, repeat diagnostic export, focused
ASan/UBSan on synthetic and selected real graphs, independent numeric checks,
and frozen-input integrity. Production code remains unchanged. Deliver a short
plain-language conclusion and a reproducible repository Markdown report.
