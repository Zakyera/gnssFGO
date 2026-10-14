# FL4f: matched ETH1 first-30 s live CBS-OFF / classic CBS-ON

The exact launch/receiver settings and copied source calibration profiles for
this result are preserved in the [Git milestone snapshot](MILESTONE.md).

Date: 2026-10-14. This is the bounded experiment authorized after the FL4e
saved-journal fix. It does **not** tune the receiver, source covariances, or CBS.
The source estimators and the 7 s fixed-lag WNOA gnssFGO receiver ran together
during 0.10× bag playback. The two successful cases used the **same** FL4e
receiver executable, SHA-256
`53c96a2b7a762800cf2291166dc78520269098684411de641b52bee6a54dc5c9`.

## Scope and provenance

- Mission: GrandTour ETH1 `2024-10-01-11-29-55`, bag offset 0, duration 30 s.
  Native GLIM: Hesai+CPT7, 5 s source lag. Native Kimera: front stereo+CPT7,
  25-state window. Source configurations/profile trees are byte-identical
  between OFF and ON (`diff -qr` returned no differences).
- Classic CBS, `active_window_temporary`, 0.20 s source horizon, one pair per
  update, dense Schur-relative 6×6 belief covariances at scale 1. Both sources
  publish in both modes. OFF disables reception; ON enables bidirectional
  reception. Source `live_revision=false`. Downstream stable-ID replacement is
  distinct from source-to-source CBS.
- Receiver: independent 100 ms support clock after 59 CPT7 samples, 7 s
  fixed-lag source-WNOA, no central IMU factor, unchanged Qc/priors, Gaussian
  external factors, no gate/health weighting/GNSS. See the frozen
  [FL4d protocol](../fixed_lag_live_fl4d_20261014/PROTOCOL.md) and
  [FL4e repair](../fixed_lag_failure_fl4e_20261014/REPORT.md).
- Successful run root:
  `/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_fixed_lag_live30_off_on_fl4f2_20261014/prepared`.
  Input/library manifest: `protected_manifest.json` (845 SHA-verified entries).
  Earlier FL4f directories retain two **pre-playback setup failures**: Docker
  permission denial, then a missing copied source-library overlay. Neither
  played the bag. The corrected fresh directory includes the pinned overlay;
  no estimator result or failed capture was overwritten.

## Completion and source exchange

| Check | OFF | ON |
|---|---:|---:|
| Native GLIM poses / span | 291 / 29.00 s | 291 / 29.00 s |
| Native Kimera ROS odometry poses / span | 134 / 29.60 s | 134 / 29.60 s |
| Source G→K / K→G injected | 0 / 0 | 130 / 130 |
| Receiver belief versions (GLIM / Kimera) | 12,862 / 3,000 | 12,884 / 3,000 |
| Receiver inserted / replaced / excluded-before-init | 420 / 15,417 / 25 | 420 / 15,439 / 25 |
| Receiver online / finalized support poses | 293 / 293 | 293 / 293 |
| Pending / unsupported-at-end | 0 / 0 | 0 / 0 |
| Maximum transport queue | 30 | 24 |
| p95 whole-support service time | 0.129 s | 0.150 s |
| Case wall time | 348.84 s | 348.85 s |

Both cases had 2,986 central IMU events, complete source and receiver markers,
valid before-playback effective-mode checks, and no source/transport failure.
The receiver covariance payload was byte-numerically identical to the journal
payload (`max error=0`); the relative-mean log round-trip discrepancy was below
`2.3e-14`. All belief versions were accounted for. Replaying each ordered saved
journal reproduced its numerical trajectories and summaries plus 16,430 OFF /
16,424 ON archived commit files byte-for-byte. The online output was published
during source playback, not obtained by an offline estimator replay.

## Accuracy: complete saved trajectories

The full-saved-output table scores **every original pose row** against
GrandTour's CPT7 Inertial Explorer tightly coupled GNSS/INS odometry. Each
estimate is rigidly aligned once in SE(3), with no scale, at the physical
CPT7-to-prism lever arm. One-second RPE uses the same reference and skips
interpolation across Kimera's 3.2 s startup gap. This IE-TC reference shares
CPT7 measurements with the estimators: it is **not independent prism GT**.

| Mode / trajectory | Rows | Span s | ATE m | t-RPE m | r-RPE deg | Path m |
|---|---:|---:|---:|---:|---:|---:|
| OFF original GLIM | 291 | 29.00 | 0.044035 | 0.041003 | 0.588259 | 8.894 |
| OFF original Kimera ROS odometry | 134 | 29.60 | 0.060015 | 0.026048 | 0.659453 | 8.338 |
| OFF fused **online/as-played** | 293 | 29.20 | 0.052990 | 0.044821 | 3.128157 | 11.510 |
| OFF fused fixed-lag-finalized | 293 | 29.20 | 0.042544 | 0.021995 | 0.499515 | 8.703 |
| ON original GLIM | 291 | 29.00 | 0.019781 | 0.020622 | 0.491247 | 8.682 |
| ON original Kimera ROS odometry | 134 | 29.60 | 0.042371 | 0.024512 | 0.702531 | 8.431 |
| ON fused **online/as-played** | 293 | 29.20 | 0.033595 | 0.040581 | 2.132611 | 10.964 |
| ON fused fixed-lag-finalized | 293 | 29.20 | 0.015328 | 0.016716 | 0.467630 | 8.661 |

The fixed-lag-finalized trajectory is a post-run output after later revisions
have been incorporated; it must **not** be described as the online result.
Kimera's first-to-next pose gap is 3.20 s, although its saved timestamps span
29.60 s. Its full-output ATE scores its actual 134 rows, not invented poses in
that gap.

## Identical-time comparison

The primary fair multi-source mask is 260 common 10 Hz timestamps over
`1727775011.600`–`1727775037.500` (25.90 s, SHA-256 time-mask digest
`e0450fc6010b7f2267674259d68df91e3689d2d39e43c06656101ec2dac3c483`).
It starts after the Kimera startup gap; estimates are not interpolated across
gaps greater than 0.5 s. All paths use identical GT timestamps and the same
alignment/scoring conventions.

| Mode / trajectory | ATE m | t-RPE m | r-RPE deg | Path m |
|---|---:|---:|---:|---:|
| OFF original GLIM | 0.044864 | 0.042236 | 0.613881 | 8.755 |
| OFF original Kimera | 0.060388 | 0.026089 | 0.659653 | 8.183 |
| OFF fused **online/as-played** | 0.051990 | 0.047459 | 3.305490 | 10.445 |
| OFF fused fixed-lag-finalized | 0.040920 | 0.023254 | 0.526912 | 8.535 |
| ON original GLIM | 0.020256 | 0.021082 | 0.516705 | 8.524 |
| ON original Kimera | 0.042746 | 0.024543 | 0.702987 | 8.276 |
| ON fused **online/as-played** | 0.033126 | 0.042945 | 2.253837 | 10.078 |
| ON fused fixed-lag-finalized | 0.015669 | 0.017398 | 0.494697 | 8.491 |

The online CBS-ON fusion ATE improves on this mask (`0.051990→0.033126 m`),
as do both native source ATEs. The ON finalized ATE is lower still, but it is
retrospective. The online path remains much longer than the 8.441 m reference
polyline, and its rotational RPE remains much larger than native GLIM/Kimera;
the as-played-versus-finalized difference must not be hidden by reporting only
the final graph. Native Kimera rotational RPE becomes slightly worse ON while
its ATE and translation RPE improve. This is one short interval, not a
universal performance conclusion.

The independent AP20 prism bag supplies only a partial-window sanity check,
not a 30 s ground-truth score: matched elapsed coverage is 14.1–14.2 s for
GLIM/fusion and 11.4 s for Kimera because of its startup gap. Its role-specific
ATEs are in `prism_partial/summary.json`; the differing masks prohibit using
them as the primary OFF/ON multi-source comparison.

## Exported covariance behavior

All receiver-facing 6×6 covariances in this capture were finite, symmetric
(`max |C-Cᵀ|=0`) and positive definite in the exported sample set (minimum
eigenvalues at least `3.05e-10` for GLIM and `9.93e-10` for Kimera). The table
shows the median trace of the rotational `[phi]` and translational `[rho]`
marginal blocks over **all logged versions**, not a posterior covariance or a
direct measure of graph influence.

| Sender | OFF tr Σφ | ON tr Σφ | OFF tr Σρ | ON tr Σρ |
|---|---:|---:|---:|---:|
| GLIM | 1.082e-8 rad² | 7.252e-9 rad² | 2.191e-6 m² | 1.532e-6 m² |
| Kimera | 2.391e-8 rad² | 2.508e-8 rad² | 9.298e-7 m² | 9.983e-7 m² |

On exactly matched physical source edges, comparing each edge's **last
observed** ON version against its OFF version:

| Sender | Matched edges | Median ON/OFF rotation-trace | Median ON/OFF translation-trace | Fraction with lower ON translation-trace |
|---|---:|---:|---:|---:|
| GLIM | 289 | 0.856 | 0.777 | 93.4% |
| Kimera | 132 | 1.052 | 1.326 | 35.6% |

Thus this window does **not** support the simple hypothesis that both source
covariances collapse under CBS-ON. GLIM's exported covariance usually
contracts, whereas Kimera's final matched-edge translation trace typically
increases. Trace alone neither calibrates accuracy nor determines total
`JᵀΣ⁻¹J` influence; no weighting change was made.

## Validation boundary and artifacts

- Identical candidate receiver, source input/config/calibration trees, bag
  window/rate, support model, and evaluation mask in both modes. The only
  intended source behavior difference was classic bidirectional reception.
- OFF/ON validated reports:
  `/home/yeranis/results/gnssfgo_fixed_lag_live_fl4f_20261014/{off,on}/{validation.json,repeat_validation.json}`.
  Detailed trajectories, complete journals, source diagnostics, receiver
  archives, and effective settings are under the successful run root above.
- Scores: `evaluation_full/summary.json` (full saved and common-mask IE-TC),
  `evaluation_full/belief_covariance_matched.json` (matched last-version
  traces), `prism_partial/summary.json` (separately labelled short prism view).
  No Rerun render, longer sequence, covariance tuning or new source method was
  launched.
- Receiver online trajectory SHA-256 OFF
  `8e4ffba8bf44dd84810c5fc3ce064c45091e359516cfa13c37cd97661ca6d05d`,
  ON `bce36eb30dd23d1d9d6e0d7d12ad7f9a62e21f28c1957f0e06be70fff4985374`.
  Finalized OFF `2a28fa0133fc90680455ea64694cf19a1f64e5eed31ee2996de2fcad29b400b7`,
  ON `7b101e6d31c07823c61c584bef514b0b07c84c60212c8772ee7665c4299d5601`.
- gnssFGO HEAD during capture: `d3d152a28440f97ced3bd7a50ee072874beafe19`.
  No commit or source mathematics change in FL4f. Prior FL3i strict whole-
  replay numerical failure remains; this incremental publication policy does
  not certify nonlinear stationarity or 1×/longer-run capacity. The bounded
  0.10× result should not be promoted as an unconditional algorithm guarantee.

**Decision:** The corrected fixed-lag receiver completed one clean matched
30 s live OFF/ON comparison. CBS-ON improved native GLIM, native Kimera and
online fused ATE on this interval; the covariance changes are source-specific,
not a universal common scale decrease. Preserve this as a measured short-run
result and discuss limitations before choosing another experiment.
