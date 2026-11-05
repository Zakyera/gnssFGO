Full GrandTour relative-belief covariance and translation accuracy
2026-10-23. Read-only extension of covariance_timeseries_full3_20261022.

WHAT TO OPEN
plots/eth1_covariance_and_translation_error.png
plots/eig-2_covariance_and_translation_error.png
plots/spx-3_covariance_and_translation_error.png
Each overview has eight plots: four rows (GLIM first, GLIM last, Kimera first,
Kimera last), covariance on the left and matched relative translation error on
the right. Each row is also saved as a larger two-panel PNG/SVG in its sequence
folder. The previous rotation-covariance figures remain untouched.

READING THE PLOTS
Blue is CBS OFF, orange CBS ON. Time is the physical belief's ending timestamp
minus the same selected-topic capture start used in the previous charts.
Trace is in m^2; translation error is in mm. Both use labelled log scales to
show small and large discrepancies without clipping. Y limits are identical
for a given source/metric across all missions and first/last selections.
All samples are shown, with no smoothing, averaging or decimation. Lines break
across endpoint gaps greater than 0.5 s. Missing endpoints are not zero error.

The left and right values come from the EXACT SAME exported belief version.
OFF and ON retain their own physical sampling; nearby times are not forcibly
matched by integer keyframe ID. Every selected pair has its own exact reference
endpoints. These plots do not perform a paired OFF/ON hypothesis test.

WHAT ACCURACY MEANS HERE
The covariance describes a short relative belief, not an absolute trajectory.
Let Z=Pose3::Expmap(mu), using the receiver's six-dimensional wire encoding.
Zref = inverse(Tref(t_from)) * Tref(t_to).
The plotted geometric translation error is ||translation(Z)-translation(Zref)||.
This is approximately 0.20-second relative-motion error, NOT whole-journey ATE
or 1-second RPE. It needs no global trajectory alignment or scale fit.

The reported covariance trace is the lower-right 3x3 translation block of the
right-local Pose3 covariance (rotation first). It is not fused posterior
covariance, WNOA uncertainty, a per-axis standard deviation, or a 95% radius.
The plotted Cartesian translation discrepancy and the SE(3) tangent translation
residual are not algebraically identical; both are retained separately in CSV.
No ratio, confidence-coverage claim, covariance rescaling or calibration is
inferred from these pictures. Trace omits directional/cross-term information.

REFERENCE AND FRAMES
Reuse the existing FL4y reference loader and SE(3) math, without modifying them.
Topic: /boxi/inertial_explorer/tc/odometry, child frame cpt7_imu. The original
source beliefs also describe CPT7-body motion. Linear position interpolation
and shortest-path SO(3) interpolation at integer-nanosecond source endpoints;
maximum allowed reference bracket 50 ms, no extrapolation. Actual bracket and
coverage counts are in audit.json. No prism lever arm is applied to this
body-relative comparison. No alignment of beliefs onto receiver support times
and no peer covariance bridging is attempted.

IE-TC shares CPT7 with GLIM and Kimera: it is not independent prism truth or
noiseless ground truth. Reference uncertainty and cross-correlations are not
estimated. The graphs show discrepancies against this reference, not NEES/NIS
or proof of covariance calibration.

POPULATION AND COVERAGE
The exact first/last samples from the previous covariance plots are preserved.
First = first distinct transport payload for each source/session/edge ID.
Last = final exported distinct version in the saved recording, retrospective.
Last does not mean one mutually consistent final trajectory snapshot. Revision
ages differ across beliefs and modes, particularly near recording boundaries.
All exported incoming beliefs are included, including startup-excluded,
expired revisions and terminal EOF omissions. Not all were installed in gnssFGO.

ETH1 CP-K23: 346.32 s capture, offset 0. Kimera short-belief endpoints stop near
300.90 s OFF / 302.10 s ON; native ROS odometry continues. The gray final region
means no new short-belief endpoints, NOT missing native trajectory or zero
uncertainty/error. Late revisions can still refer to older intervals.
EIG-2 CP-K24/24A: 355.02 s capture, offset 5.7 s. Original strict EOF failure
and supplemental finite-recording audit preserved; one terminal ON factor
omitted in the receiver, but included in the incoming-belief population here.
SPX-3 CP-K26: 194.02 s capture, offset 32 s, fresh completed restart, not CP-K25.
One terminal Kimera factor omitted by the receiver in each mode.

PROVENANCE AND REPRODUCTION
audit.json pins the exact journals, previous selection CSVs, prior audit,
reference bags and helper scripts by SHA-256. Each selected event row is checked
against source/session/IDs/revision/arrival/endpoints and original covariance
traces. Full selected matrices are checked finite, symmetric and positive
definite. All selected endpoint references must be supported or generation
stops. Existing SE(3) exp/log tests, a known 5 mm error and global-transform
invariance test run before plotting. Inputs are rehashed after generation.

covariance_and_error_samples.csv retains IDs, physical/arrival times, revisions,
trace, actual Cartesian and tangent translation error, rotation error, decoded
belief translation and corresponding reference translation. Original full
covariances remain in the unchanged journals identified by event_line.

Run the new make_plots.py using the bundled Python. It reuses existing scipy,
matplotlib and rosbags from the evo environment for unavailable capabilities.
It refuses to overwrite an existing plots directory. It does not call any
historical evaluator, controller, replay, estimator, container or optimizer.
All previous plots, results, launch settings and covariance values are unchanged.
