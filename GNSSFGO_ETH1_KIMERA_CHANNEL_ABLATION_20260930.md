# ETH-1 Kimera measurement-channel ablation in the frozen gnssFGO receiver

## Scope and products

This is a bounded diagnostic on the first 120 s of GrandTour ETH-1 mission
`2024-10-01-11-29-55`. It replays the previously recorded classic CBS-OFF and
CBS-ON **causal receiver event journals**; GLIM and Kimera were not rerun.
The native source outputs and the existing full GLIM+Kimera gnssFGO results
remain the frozen reference from the [live experiment](/home/yeranis/repos/gnssFGO/GNSSFGO_ETH1_LIVE120_CLASSIC_ZERO_SOURCE_MODEL_20260930.md).
These new trajectories are offline gnssFGO receiver controls, **not** native
GLIM or native Kimera trajectories and not a new co-running live experiment.

The question is whether the CBS-ON full-joint loss is attributable to
Kimera's rotational or translational relative-measurement channel. To avoid
selecting a covariance scale from ATE, both source modes were tested with
two fixed subspaces:

| Diagnostic | GLIM factor | Kimera factor |
|---|---|---|
| Rotation | Complete original 6D `[phi,rho]` | Existing `phi` residual/Jacobian rows 0–2 and full dense 3×3 `Sigma_K(phi,phi)` marginal |
| Rho | Complete original 6D `[phi,rho]` | Existing `rho` residual/Jacobian rows 3–5 and full dense 3×3 `Sigma_K(rho,rho)` marginal |

`rho` is the translation-tangent coordinate of the existing SE(3) Log
residual, not a new Cartesian position measurement. It can still depend on
the measured relative rotation through the Log map. Discarding a channel
uses its **marginal** covariance block, not a Schur complement or artificial
large variance. The original full 6×6 Kimera factor, including its nonzero
phi–rho cross block, is unchanged. The two 3D diagnostics therefore do not
form an additive decomposition of the original full likelihood.

## Frozen event/receiver semantics

The input is the original `off_receiver/events.csv` or
`on_receiver/events.csv` from the linked live run, **unaltered**. It contains
the original IMU events and recorded availability order, 1,189 GLIM belief
IDs and 581 Kimera belief IDs, with every saved source revision. The ON
Kimera and GLIM beliefs already embody their bidirectional classic CBS
exchange; removing a Kimera channel downstream does not undo source-side
CBS information. Each diagnostic keeps the source-independent 100 ms
support clock, 59 IMU startup samples, exact ALL_RIGHT CPT7 integration,
Combined IMU, WNOJ `Qc=10000`, original priors, full-history iSAM2,
physical-time interpolation, stable-ID factor replacement, full native GLIM
covariance, and source covariance scale 1. There is no GNSS, added
source-model uncertainty, robust loss, gating, or health weighting.

An opt-in `--kimera-channel full|rotation|rho` selector was added to the
existing GrandTour CSV diagnostic runner in
[`GrandTourCsvReplay.cpp`](/home/yeranis/repos/gnssFGO/online_fgo/src/offline_process/GrandTourCsvReplay.cpp).
The default is `full`. The selected factor wraps the **existing** relative
factor and interpolators, evaluates their full six-vector residual/Jacobians,
then selects exactly three rows. At the first Kimera insertion in each run,
the selected residual and every Jacobian block are checked against the
corresponding full-factor rows to `1e-12`; the installed Gaussian covariance
is checked against the source's 3×3 marginal to `1e-12`. A selected-channel
run fails if no Kimera factor was actually selected.

The diagnostic uses `Gaussian::Covariance(marginal, false)` to retain the
dense 3×3 matrix. GTSAM's default `smart=true` automatically dropped small
off-diagonal entries in this dataset; that would not have been the requested
marginal. The valid runs' maximum installed-vs-source covariance difference
is at most `5.43e-20` across all inserted factors. This dense option applies
only to the new 3D diagnostic wrapper; the existing default full-6D factor
path was not changed.

The diagnostic executable was built in Release in the existing ROS Humble
container. Its SHA-256 is
`f9919afdd6ffbde99d352c3af59f2e5770dd051926842e4b717754348fabf9ae`.
The four accepted output directories have suffix `_valid4` under the
[artifact directory](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_kimera_channel_ablation_20260930/).
Earlier unsuffixed/`valid*` directories are **invalid attempts**, excluded
from all scores: the first selector missed the actual `kimera_front` source
ID and silently reproduced the full path; the subsequent fail-closed
covariance check caught GTSAM's 3×3 diagonal simplification. Those attempts
are retained only for debugging provenance. No invalid trajectory appears
in the tables below.

The unchanged journal SHA-256 values are `d75e2cf25ad413b2c4f9f93e5331b601a695d5cb996a6ab44d32d543010f49e0`
(OFF) and `fe0054add31063df0d36be106411d44ad439f4e748d740a88d94da3663bc7a38`
(ON). Each diagnostic used the existing replay executable in the ROS Humble
container, sourced `/opt/ros/humble/setup.bash` and
`/workspace/fork/install/setup.bash`, and invoked:

```text
grandtour_ct_replay --event-stream EVENTS.csv --out OUTPUT_DIRECTORY \
  --mode sources --timeline record --solver isam2 \
  --support-policy uniform_imu --support-period-ns 100000000 \
  --receiver-startup-samples 59 --kimera-channel rotation|rho
```

`EVENTS.csv` was the matching original OFF or ON journal; no event was
rewritten. The frozen full-factor control used the same command without
`--kimera-channel`.

## Evaluation

The same frozen GPS bounds `1727775008.775065899`–`1727775127.674791098`
and original scorers were used: AP20 prism with CPT7 lever arm and no-scale
SE(3) alignment for ATE, and IE-TC at 1 s for translation/rotation RPE.
The eight new online/final products and the original full-joint controls
share **exactly the same 654 prism timestamps**, 64.9 s aggregate prism
coverage in five segments, and 1,179 valid RPE pairs. The prism does not
continuously cover the whole 120 s. The full-path column is the receiver's
path over the requested window, not the 47.790 m matched GT path.

### Online/as-played receiver trajectory

GLIM-only and Full are the already-frozen results, not rerun source-estimator
experiments. Rotation and Rho are the four new diagnostic receiver replays.

| CBS mode | gnssFGO factor set | ATE (m) | 1 s t-RPE (m) | 1 s r-RPE (deg) | Full path (m) |
|---|---|---:|---:|---:|---:|
| OFF | GLIM only | 0.049909 | 0.044001 | 1.104008 | 73.334 |
| OFF | GLIM + Kimera rotation | 0.049084 | 0.041913 | 1.120681 | 73.262 |
| OFF | GLIM + Kimera rho | 0.051646 | 0.032646 | 0.497301 | 72.330 |
| OFF | GLIM + Kimera full | 0.051511 | 0.031413 | 0.575945 | 72.336 |
| ON | GLIM only | 0.068069 | 0.065721 | 1.070211 | 80.059 |
| ON | GLIM + Kimera rotation | 0.095773 | 0.063759 | 1.091906 | 79.243 |
| ON | GLIM + Kimera rho | 0.072842 | 0.055528 | 0.924394 | 77.501 |
| ON | GLIM + Kimera full | 0.122472 | 0.054332 | 0.942747 | 76.687 |

### Final retrospectively revised receiver trajectory

This is the optimized graph after all recorded revisions arrived; it is not
what the online receiver had estimated at each earlier instant.

| CBS mode | gnssFGO factor set | ATE (m) | 1 s t-RPE (m) | 1 s r-RPE (deg) | Full path (m) |
|---|---|---:|---:|---:|---:|
| OFF | GLIM only | 0.035142 | 0.021627 | 0.388291 | 72.158 |
| OFF | GLIM + Kimera rotation | 0.036137 | 0.021300 | 0.443813 | 72.165 |
| OFF | GLIM + Kimera rho | 0.044751 | 0.015580 | 0.357643 | 71.809 |
| OFF | GLIM + Kimera full | 0.048359 | 0.015597 | 0.414167 | 71.824 |
| ON | GLIM only | 0.036467 | 0.021920 | 0.641766 | 72.011 |
| ON | GLIM + Kimera rotation | 0.054552 | 0.021769 | 0.581343 | 72.029 |
| ON | GLIM + Kimera rho | 0.048072 | 0.018239 | 0.559644 | 71.950 |
| ON | GLIM + Kimera full | 0.107042 | 0.018430 | 0.515790 | 71.928 |

For orientation only, the original **native** CBS-OFF→ON outputs were GLIM
ATE `0.038007→0.028978 m` (655 prism matches) and Kimera ROS odometry ATE
`0.541016→0.365236 m` (315 matches). They are not gnssFGO single-source
trajectories and do not use the receiver's exact 654-point mask.

## Interpretation

On the common receiver mask, **neither Kimera channel alone explains the
full CBS-ON joint ATE loss**. Relative to ON GLIM-only, ON rotation-only
raises online/final ATE `0.0681→0.0958` / `0.0365→0.0546 m`; ON rho-only
raises it to `0.0728` / `0.0481 m`. The full 6D Kimera factor raises it
much further to `0.1225` / `0.1070 m`. The analogous OFF comparisons are
much smaller in ATE. The full ON result therefore reflects an important
**combined-channel / graph interaction** not reproduced by either marginal
channel experiment alone. This is an empirical observation, not a proof
that the phi–rho covariance cross block or CBS common-information
correlation is the sole cause. Both the full covariance coupling and the
nonlinear reoptimized trajectory change when one channel is removed.

The metrics are mixed rather than uniformly worse. Under ON, rho-only
improves 1 s t-RPE and r-RPE relative to ON GLIM-only; the full ON factor
has the best final r-RPE of the four ON receiver products (`0.515790°`),
but by far the worst ATE. OFF full and rho-only also improve short-horizon
translation relative to OFF GLIM-only. ATE alone does not select a
scientifically valid weight.

The frozen Kimera 6×6 covariances have nonzero phi–rho cross terms. Across
581 last-installed Kimera factors, the median largest absolute normalized
phi–rho entry is `0.532` OFF and `0.299` ON (see
[`covariance_cross_block.json`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_kimera_channel_ablation_20260930/covariance_cross_block.json)).
The 3D marginal controls intentionally omit these cross terms, so one
cannot add their independent ATE changes to predict the 6D result.

## Validation and non-regression

- Each valid control accepted the same 1,189 GLIM and 581 Kimera unique IDs,
  preserved the original OFF/ON revision counts (70,711/70,748 messages),
  skipped the same 25 early unsupported beliefs, ended with zero pending,
  and reproduced the mode-specific support clock byte-for-byte. Direct
  `source_accounting.csv` is authoritative for per-source counts; two legacy
  summary counters recognize only generic IDs and stay zero for the actual
  `glim_hesai`/`kimera_front` registry IDs.
- The online/final GT timestamp masks are identical to the original
  corresponding full-joint receiver and across all eight new products.
  [`validation.json`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_kimera_channel_ablation_20260930/validation.json)
  records these checks, and [`metrics.json`](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_kimera_channel_ablation_20260930/metrics.json)
  links to every raw scorer output under `evaluation/`.
- A separate default-path (`--kimera-channel` omitted) full-6D regression
  replay completed for both OFF and ON. In each mode,
  `online_trajectory.tum`, `trajectory.tum`, `support_clock.csv`, and
  `source_accounting.csv` are **byte-identical** to the frozen live receiver's
  files. OFF online/final trajectory SHA-256 values are
  `bf976d3f9aac1a83ffa8571e75eb15ed69fbc03227e52c3dae52b88077d6c8bc`
  / `87036b1d29594c9923a486224e7314e5ccba4b676fabfee32d3d21395244aafe`;
  ON values are `07f397c77fdafe0faca1e3e644fbf3a8e5dad4d5d96028a8ef6074c1c64644e3`
  / `53b39d896dbd7a1a8b4fed1cd845256861bf62e20cf584584c88c03eb973d3d7`.
  The complete regression outputs are `off_full_regression_resumed/` and
  `on_full_regression_resumed/`; earlier unsuffixed regression directories
  were interrupted and are not used. No native source, frozen factor journal,
  old report, or evaluation reference was overwritten.
- The gnssFGO repository was already dirty before this task. The only
  production-tree file edited here is the opt-in diagnostic selector in
  `GrandTourCsvReplay.cpp`; normal full-6D behavior remains the default.
  The active gnssFGO HEAD is `a842e7ef72e7a40f0cdfec826f089c66af364ae0`.
  No method tuning or follow-up fusion experiment was performed.
