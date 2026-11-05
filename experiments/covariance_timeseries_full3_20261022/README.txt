FULL ETH1 / EIG-2 / SPX-3 COVARIANCE TIME SERIES — 2026-10-22

Purpose: 8 OFF/ON overlay plots per completed sequence, using existing saved
journals only. No estimator or fusion rerun, tuning, or input modification.

For each sequence, the rows are GLIM translation, GLIM rotation, Kimera
translation, Kimera rotation. Left column: first exported version per stable
belief ID. Right column: last exported version per ID (retrospective).
Every plot overlays blue CBS-OFF and orange CBS-ON. The full observation window
is shown, including startup and tail. Each plot is supplied as PNG and SVG;
the overview combines the same eight graphs. The ZIP contains all three sets.

What is plotted?
The source's LOCAL relative-Pose3 measurement covariance, as exported into the
fusion transport journal with scale1. Not absolute trajectory uncertainty,
not fused posterior covariance, not WNOA process covariance, not residuals.
Rotation trace = C00+C11+C22, rad^2.
Translation trace = C33+C44+C55, m^2.
No sum of the two incompatible-unit blocks, and no diagonal replacement of the
actual full6x6 factors. Lower trace means less reported aggregate uncertainty
in that block, not automatically greater accuracy or optimizer dominance.

Time axis = physical to_timestamp_ns minus the SAME capture start for OFF/ON.
It is the location in the recorded motion of the relative measurement, not
arrival time or optimization wall time. CSV includes both endpoint and arrival
times plus interval length and transport revision to preserve the distinction.
ETH1 CP-K23: offset0,346.32s.
EIG-2 CP-K24/24A: offset5.7s,355.02s.
SPX-3 CP-K26: offset32s,194.02s; NOT the interrupted CP-K25 attempt.

No smoothing, moving median, decimation or percentile clipping. Every selected
sample is plotted. Logarithmic y axes reveal orders of magnitude; zero cannot
appear on a log axis. Same y limits for each source/block across all sequences,
versions and modes. Rotation and translation have their own units/scales.
Lines break when physical endpoint spacing exceeds0.50s; no samples removed.
Intervals are not normalized to the same duration; all exported intervals in
these journals are <=0.30s. Gray tail means there are no new exported belief
endpoints, not that uncertainty becomes zero.

ETH1 Kimera's last new belief endpoint is300.898313491s OFF/302.098307591s ON.
Native odometry continues beyond this: sparse late keyframes do not supply
new pairs within the configured short belief horizon. Revisions of earlier
intervals continue arriving; some last-export ages reach45.22s. The physical
endpoint plot must not stretch those older beliefs into the empty tail.

First = first received distinct payload, wire revision0. Last = final received
distinct payload with the same source/session/fromID/toID. The adapter can
renumber source revisions; transport revision is explicitly named in outputs.
Last is NOT a causal time series or one jointly consistent smoother snapshot.
Its age varies, and tail beliefs have less opportunity to mature. All exported
versions remain in scope: pre-initialization factors, expired ETH1 revisions,
and the documented terminal omissions in EIG-2 ON and SPX-3 OFF/ON. A last
export is therefore not necessarily the version finally used by the receiver.

OFF/ON traces use their actual independent native sampling. Numeric keyframe
IDs are not cross-run physical matches. Exact endpoint-pair overlap counts are
saved in audit.json; overlays are descriptive, not matched-edge ON/OFF ratios.
No claim of calibrated/independent covariance or universal CBS scaling follows.

Validation: stable IDs retain endpoints; transport revisions start0 and increase
consecutively; covariance diagonal entries are finite/positive; pooled counts,
stable-ID counts, revised-ID counts, medians and p95 reproduce the frozen
reference reports. SHA256 of all six input journals and three result files is
recorded; checked unchanged after plotting. Full covariance symmetry/PSD and
transport equality were checked in the completed runs, not recomputed here.

Reference reports:
/home/yeranis/repos/gnssFGO/GNSSFGO_ETH1_FULL_CPK23_REFERENCE_REPORT_20261021.md
/home/yeranis/repos/gnssFGO/GNSSFGO_EIG2_FULL_CPK24_REFERENCE_REPORT_20261021.md
/home/yeranis/repos/gnssFGO/GNSSFGO_SPX3_FULL_CPK26_REFERENCE_REPORT_20261021.md

Reproduction: bundled Python runs make_plots.py here, which refuses an existing
plots directory unless --refresh-plots explicitly regenerates these derived
figures after validating the prior input hashes. NumPy is bundled; matplotlib is borrowed read-only from
the existing evo Python3.12 environment. No dependencies are installed/modified.
