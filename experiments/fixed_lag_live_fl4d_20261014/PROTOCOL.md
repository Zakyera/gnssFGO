# FL4d: matched ETH1 30 s CBS-OFF/ON, fixed-lag live WNOA

This is a predeclared engineering and descriptive-accuracy comparison, not a
covariance-tuning experiment. OFF is the completed, validated FL4c capture;
ON is one fresh concurrent source+receiver capture. Do not overwrite OFF or
retry a failed ON automatically. Do not change estimator mathematics.

## Questions

1. Does classic bidirectional CBS change native GLIM and native Kimera relative
   to the matched OFF sources, and are actual cross-source factors injected?
2. Does the *online/as-played* fixed-lag fusion change in the same direction as
   the native sources on the same physical reference interval?
3. What full 6x6 beliefs were actually sent between sources and to gnssFGO,
   and how do their covariance spectra and matched-edge errors differ? Do not
   infer calibration or independence from covariance trace alone.

## Frozen conditions

ETH1 mission `2024-10-01-11-29-55`, bag offset 0, 30 s, 0.10x, headless.
Hesai+CPT7 GLIM with 5 s source lag; front-stereo+CPT7 Kimera with 25-state
window and its saved calibration/profile. One 0.20 s horizon pair per update,
publish stride 1, classic source CBS, full dense `schur_relative_between`
covariance, scales 1, no source health weighting, source factor mode
`active_window_temporary`. Both modes publish diagnostic beliefs. The ONLY
intended source-estimation difference is receive/inject false (OFF) versus
bidirectional true (ON); `live_revision` stays false. Verify effective flags,
subscribers, loaded DSOs and injection counts before/after playback.

Downstream gnssFGO uses the same pinned FL4a receiver binary as FL4c: one
persistent 7 s fixed-lag graph, independent 100 ms support clock after the
59th IMU sample, source-driven WNOA Qc/priors unchanged, no central IMU
factor, full source covariance scale 1, Gaussian, stable-ID replacement, no
gating/GNSS/health weighting. No graph rendering in the hot path. ON uses a
fresh source session and output directory but the same receiver code/profile.

## Primary retained products and evaluation

For OFF and ON separately, save original/native GLIM odometry, original/native
Kimera ROS odometry, gnssFGO online/as-played, and fixed-lag-finalized (last is
NOT an online or full-history retrospective trajectory). Retain ordered event
journals, source-to-source outgoing/received belief records and the downstream
current-smoother revision stream, including stable IDs, revision/availability
time, physical endpoints, relative mean and all 36 covariance entries.
Cross-check source-log versus transported payloads and exact factor-use counts.

Primary full-path reference: ETH1 CPT7 IE-TC `/boxi/inertial_explorer/tc/odometry`
in `2024-10-01-11-29-55_cpt7_ie_tc.bag`, using the existing CPT7-to-prism lever
arm, no-scale SE(3) alignment and 1 s RPE evaluator. Label IE-TC a tightly
coupled GNSS/INS reference sharing CPT7 data, NOT independent prism GT. Score
every saved pose against a valid reference bracket; report actual spans, rows,
gaps, path lengths, ATE, 1 s translational/rotational RPE. Also compute a
single common 10 Hz OFF/ON/native/fused time mask from the intersection of
valid reference and source coverage, rejecting interpolation across gaps
>0.5 s. The mask is fixed from timestamps, not errors, and used for the
primary matched comparison. Prism covers only about 11.2 s of this bag window;
report it separately as a partial independent-reference sanity check, never
as full-30-second ATE.

Covariance diagnostics: source-wise rotation/translation marginal trace,
eigenvalue range, full 6x6 PSD/symmetry, first-versus-last revision evolution,
and counts. Compare source beliefs against IE-TC relative transforms only on
identical physical intervals, with the reference's shared-IMU limitation
stated. Record whitened residuals descriptively; no gate, rescale or tuning.

## Acceptance/stop rules

Do not score ON if profiles, source executable mapping, input identities,
receiver binary or timing differ outside the intended CBS receive switch.
Require both native outputs to span at least 28 s; report Kimera's known
startup gap. Require receiver online coverage >=28 s, exact event conservation,
full payload equality, no unsupported lag losses, zero pending at EOF, active
supports <=71 and no growing queue (max <1000). At 0.10x, whole-support p95
service <=0.8 s. Repeat ON from its saved event journal with byte-identical
numeric outputs and commit archive. On source/receiver/validation failure,
stop and preserve evidence; no automatic retry or tuning.

Interpret fusion accuracy as exploratory: the FL3i strict whole-replay
numerical-equivalence gate remains failed and this fixed-lag publication path
does not certify nonlinear stationarity. Do not claim a validated algorithmic
advantage from this one short sequence. After the matched pair, report what
changed and decide the next scientifically useful window/sequence separately.
