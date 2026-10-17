# FL4i — was the latest ETH1 run launched correctly?

2026-10-15. Saved-artifact audit only: no estimator/receiver replay, bag playback,
source modification, covariance tuning, or solver experiment. This follows the
user's request to check settings and GLIM's engineering changes before pursuing
FL4h's solver hypothesis.

## Findings

1. **FL4g correctly extended the frozen FL4f 30 s setup.** No accidental change
   of sensor profile, covariance scale, source revision protocol, native Kimera
   IMU path, or loaded source-library path was found.
2. **That setup is not the earlier successful persistent-CBS setup.** It uses
   temporary external factors, native-IMU mean alignment, and a different GLIM
   outgoing covariance provider. `classic` means the ordinary source publication
   protocol, not equivalence of all receiver/marginalization semantics.
3. **The major Kimera failure predates the shared-native/Jacobian provider.**
   A retained October 11 run, using the independent nonlinear LOCAL graph,
   already gives 2.8497 m native Kimera ATE on exactly the current 120 s mask.
   It also develops large short-increment errors after about 75–90 s.
4. **The latest ON GLIM is not less accurate than the older ON GLIM.** Both its
   native trajectory and actual wire relative means are better on the compared
   intervals. This rejects a simple explanation that the speed-up damaged
   GLIM's mean estimates and those bad estimates poisoned Kimera.
5. The exact causal change between healthy persistent CBS and the failing
   temporary/aligned implementation is **not isolated** by these archives.
   Factor lifetime, native-only covariance history, alignment, source recovery
   implementation and asynchronous execution changed together. Neither
   temporary factors nor the shared provider are individually convicted or
   exonerated by one live comparison.

## 1. Launch and profile checks

Current artifact root:
`/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_fixed_lag_live120_fl4g_20261015/prepared`.

Against
`runs/gnssfgo_fixed_lag_live30_off_on_fl4f2_20261014/prepared`:

- `source.env` differs only in run root and requested duration, 30→120 s.
- Actual before-playback GLIM and Kimera effective-settings dictionaries are
  identical for the corresponding OFF/ON modes.
- Full actual Kimera argv is identical after replacing run/output paths and
  excluding ROS's unique log filename. Workspace library mappings also match
  after normalizing the run-root paths.
- Twelve distinct mapped core-library paths resolve to frozen hashes, all
  verified unchanged, including the private CBS/GLIM/GTSAM libraries. Other
  mapped auxiliary libraries have the same recorded paths but no historical
  hash in this manifest; this is not an all-dependencies binary certificate.
- All compared source trees contain the same 16 GLIM configuration files and
  14 Kimera profile files, byte-for-byte: September 7, October 5, October 11,
  B0, S4d, FL4f, and current FL4g OFF/ON.
- Latest settings remain ETH1 offset0, rate0.10×, Hesai/CPT7, front stereo/CPT7,
  GLIM5s, Kimera25states, one 0.20s horizon pair/update, scales1, health/Huber
  disabled, source `live_revision=false`. Native Kimera uses `current`, not the
  earlier experimental RIGHT packet path.
- Frozen gnssFGO is the same FL4e source-WNOA 7s incremental fixed-lag receiver,
  independent100ms supports, no central IMU factor, stable-ID downstream
  replacement. It does not send corrections back to GLIM or Kimera. Its
  fixed-lag change therefore cannot directly explain the native Kimera failure.

Existing source recovery guards remain enabled. “Gaussian/no Huber” does not
mean guard-free: FL4h reconciled580 constructed G→K candidates to562 retained
admissions. No new gating was added in this audit.

## 2. What differs from successful historical CBS-ON?

| Property | Successful persistent ETH1, Sept7 / Oct5 | Oct11 pre-provider | Latest FL4g |
|---|---|---|---|
| Source publication | Classic, no explicit live revisions | Same | Same |
| External factor lifetime | Persistent; can enter marginal prior | Removed before endpoint retirement | Same temporary policy |
| Native-IMU temporal mean alignment | Not enabled in saved launch | Enabled both directions | Enabled both directions |
| GLIM mean | MAIN smoother endpoint relative pose | MAIN | MAIN |
| GLIM ordinary outgoing covariance | Direct MAIN smoother joint marginal | Independent nonlinear LOCAL/native graph | Native-only Gaussian history from MAIN linearizations |
| Source sensors/calibration/lag/profile bytes | ETH1 historical profile | Identical | Identical |
| Source scale/health/Huber | 1 / off / off | Same | Same |
| Playback | Sept7 ON0.15×; Oct5 matched0.10× | 0.10× | 0.10× |

The shared provider is selected by `GLIM_SHARED_NATIVE_PROVIDER=shared_native`
in FL4g's `glim_profile{,_on}.launch:116`. The resolved ROS string
`glim_cbs_outgoing_marginal_source=direct` alone is insufficient to identify
the actual covariance source. The loaded private
`runs/shared_linearization_s4_provider_20261012/release_2/overlay/cbs_bridge.cpp`
checks the selector at683–693 and uses `publicationCovariance()` at3699–3721;
the old joint-marginal branch starts at3723.

The [S4d report](../shared_linearization_s4_provider_20261012/REPORT.md) explicitly
defines native information evaluated at MAIN's recorded charts, with its own
native-only marginal history. It does **not** claim equality to the independently
optimized old LOCAL covariance. This was more than a bit-exact acceleration of
the old covariance provider, although the Schur relative-pose extraction formula
is unchanged. Later diagnostic revisions sent to gnssFGO still use the existing
MAIN-covariance path; do not confuse them with ordinary source-to-source wire
beliefs analyzed below.

This provider requires `active_window_temporary`. Blindly changing only the
factor-mode flag to persistent is not a valid controlled comparison: its startup
contract rejects that combination.

## 3. Original/native trajectories: identical current evaluation mask

Only saved trajectories were scored using FL4g's unchanged evaluator. All rows
below use its exact archived mask/hash:1,160 timestamps,115.9s,
`1727775011.5999973–1727775127.4998868`, after Kimera startup. Full-window IE-TC
reference, same CPT7-to-prism lever arm, rigid no-scale ATE,1s RPE. IE-TC shares
CPT7 and is not independent prism GT. This avoids comparing old full346s prism
metrics with current first120s IE-TC metrics.

| Saved source capture | Native GLIM ATE m | Native Kimera ATE m | GLIM t-RPE m | Kimera t-RPE m |
|---|---:|---:|---:|---:|
| Sept7 successful persistent ON, cropped | .038786 | .365249 | .033584 | .032521 |
| Oct5 persistent OFF | .030474 | .458070 | .025995 | .045923 |
| Oct5 persistent ON | .036950 | .375084 | .033733 | .031494 |
| Oct11 temporary/aligned ON, **independent LOCAL** | .034628 | **2.849717** | .023983 | **.334658** |
| Latest FL4g OFF | .044372 | .458070 | .026395 | .045923 |
| Latest FL4g temporary/aligned ON, **shared-native** | **.025248** | **3.745897** | **.018466** | **.350497** |

Oct11 was an incomplete requested160s capacity run that stopped later, around
143.8s. Its saved sources cover this entire first120s comparison; this table
does not relabel it a completed160s experiment.

Important counterpoints:

- Latest OFF GLIM ATE is worse than Oct5 OFF by13.90mm, although its t-RPE changes
  only .025995→.026395m. Do not claim old/new GLIM runs are numerically identical
  or universally improved. No identical-input counterfactual attributes this
  difference to the provider, GPU scheduling or another source implementation
  change. Under OFF there is no returned CBS factor to carry a covariance change
  back into MAIN.
- Oct5 OFF and latest OFF Kimera contain **exactly equal timestamps, positions,
  and quaternions on all584 saved rows**. Thus the observed large native Kimera
  difference is in its CBS-enabled behavior, not a changed camera calibration,
  missing tail, or generic OFF trajectory regression.
- Short tests genuinely looked healthy: on the common36s portion of B0/S4d,
  native GLIM ATE .036652→.029721m and Kimera .098197→.058684m. On the common25.9s
  mask, FL4f Kimera .042746m and FL4g's prefix .039993m. The120s failure appears
  later; it does not contradict the bounded30s observation.

### Failure already present before the provider

RMS error of consecutive native Kimera ~0.20s output increments against IE-TC,
in metres; this is not ATE or1sRPE:

| Source capture | 75–90s | 90–105s | 105s to current common tail |
|---|---:|---:|---:|
| Oct5 persistent ON | .01030 | .01000 | .01379 |
| Oct11 temporary/aligned independent LOCAL ON | .02019 | .06676 | .19101 |
| Latest shared-native ON | .03553 | .12322 | .17651 |

The large late Kimera deformation is therefore not a new phenomenon introduced
by S4d or by gnssFGO's new fixed-lag receiver. Its precise magnitude does differ.

## 4. Actual GLIM wire beliefs, not inferred from its trajectory

Compared first outgoing `/kimera/cbs/odom_belief_in` measurements on1,157
identical GLIM source edges/timestamp intervals in the common section. Convert
the existing `[phi,rho]` payload with SE(3) Exp and compare against the IE-TC
relative transform at its physical endpoints, without fitted alignment/scale.
These are raw sender intervals; current receiver-aligned means are separately
verified in FL4h. No pooling of later diagnostic revisions.

| GLIM wire source | Relative translation RMS m | Relative rotation RMS deg |
|---|---:|---:|
| Oct5 persistent ON | .024973 | .301392 |
| Oct11 independent LOCAL ON | .011122 | .155074 |
| Latest shared-native ON | **.009544** | **.151413** |

Matched latest/Oct11 covariance-trace ratios have median **1.00363 translation**
and **1.03104 rotation**; p95 **1.17405 /1.24868**. Latest/Oct5 medians are
**1.15345 /1.16452**. This is not a blanket collapse of GLIM covariance following
the new provider. Covariance traces alone do not establish calibration or full
directional information, and different live histories prevent treating these
ratios as an isolated provider-only effect.

In the more closely bounded B0→S4d40s comparison,359 matched edges give translation
error RMS .010336→.009534m; covariance median ratios1.01652translation and
1.04774rotation. Again, no evidence that this change made GLIM's transmitted
relative means worse.

## 5. Interpretation and stop point

The latest run was appropriately launched **for the agreed current milestone**,
not as a reproduction of historical persistent CBS. The name “classic” hid an
important distinction in our discussion. The significant source-side changes
preceded the GLIM speed-up, and a large Kimera failure was already present in
that earlier configuration. FL4h additionally found that good incoming GLIM
means can be followed by a much worse Kimera optimized edge; it did not prove
which prior/solver/lifecycle mechanism causes that response.

Do not revert the GLIM speed work, tune covariance, blame WNOA, or increase
solver iterations from this evidence. The next bounded question, if authorized,
is the source-side persistent→temporary/aligned transition and Kimera's actual
factor/prior handling, with the incoming information controlled. A mode switch
alone is not yet a valid test because the provider imposes its own mode contract.
There is no authorization here to launch that test or modify anything.

## Evidence and reproducibility

Saved audit root:
`/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_fixed_lag_live120_fl4g_20261015/audit_fl4i/`

- `audit_saved.py`, `summary.json`: exact native paths/hashes, configuration-tree
  comparisons, common-mask and short-prefix scores.
- `beliefs_and_launch.py`, `belief_launch_summary.json`: actual wire means,
  covariances, matched-edge comparisons, native-step bins, complete GLIM
  resolved-parameter differences.
- `verify_launch.py`, `launch_check_v2.json`: effective30/120s dictionaries,
  normalized argv/library paths, core hashes and explicit unpinned auxiliaries.
  The earlier `launch_check.json` is retained; v2 resolves symlink aliases.
- [Prior FL4h source audit](../fixed_lag_live_fl4h_20261015/REPORT.md),
  [FL4g result](../fixed_lag_live_fl4g_20261015/REPORT.md),
  [historical protocol](../../CLASSIC_CBS_RESTORATION_AND_REVISION_COMPARISON.md).

ROS container was initially stopped; it was started with its verified idle
`tail -f /dev/null` command for the existing evaluator/rosbag reader only. No
GLIM, Kimera, gnssFGO, or sensor playback was launched. The audit's first mask
reconstruction stopped on an assertion; it was corrected to recover and verify
the exact archived timestamp hash before scoring. Original data/configurations
and previous scientific results were not overwritten. No build/test suite was
needed because no estimator implementation was edited.

At handoff the container contained only its idle PID1 and was restored to its
original stopped state. `git diff --check` passed; Kimera-VIO, CBS and glim_ros1
source worktrees remain clean. Pre-existing gnssFGO audit edits were preserved.
