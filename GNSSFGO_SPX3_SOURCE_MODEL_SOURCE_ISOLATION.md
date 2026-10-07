# SPX-3 receiver source-model uncertainty: source-isolation controls

Date: 2026-09-29. This is a **headless offline receiver replay** of the frozen
120 s SPX-3 CBS-OFF and classic CBS-ON arrival journals. Native GLIM, native
Kimera, their live CBS exchange, and the source belief payloads were not rerun
or changed. The question is which receiver source accounts for the ATE change
observed when fixed additive source-model uncertainty was applied to both
sources in [the preceding experiment](GNSSFGO_SPX3_SOURCE_MODEL_DISCREPANCY_EXPERIMENT.md).
It is not a new production configuration or a GT-optimized parameter search.

## Frozen graph and four receiver settings

The receiver remains full-history iSAM2, with its independent 100 ms support
grid, ALL_RIGHT CPT7 Combined IMU, WNOJ `Qc=10000`, identical priors and
59-sample startup, record-time causal dispatch, and stable-ID replacement of
revised GLIM/Kimera factors. No GNSS, gate, robust loss, health weighting,
covariance scaling, or source-estimator change was introduced. The original
source-to-source CBS-ON run followed the *classic* protocol, not the
experimental `live_revision` protocol.

The receiver option adds a diagonal model-discrepancy covariance in Pose3
`[phi,rho]` order to an otherwise untouched, full reported 6×6 source matrix:

```text
Sigma_effective = Sigma_reported
                + diag(sigma_phi² I3, sigma_rho² I3).
```

The two fixed values were declared before the preceding experiment and were
**not selected from this experiment's ATE**:

| Actual journal source ID | sigma_phi (rad) | sigma_rho (m) |
|---|---:|---:|
| `glim_hesai` | 0.0000561019904579 | 0.000244015026860 |
| `kimera_front` | 0.0000450299900067 | 0.000601137810046 |

For each CBS mode, the 2×2 controls are: neither source (zero), GLIM only,
Kimera only, and both sources. Zero/both are the **frozen, already scored**
controls from the preceding experiment; only the four missing single-source
receiver replays were run here. The binary was unchanged across these new
replays (SHA-256 `2f188a661e87a63092271b2b325bed0fa3ab0471951f596c86a7a45f8ebf4530`).

The input journal SHA-256 hashes are OFF
`f7a95731644f6031d4c9f6a2787c2f545391d35d242c35dae1e3a238cd93820d`
and ON
`a96521520f7f60cb4dbefb569d430ecb13748c56c1490b8860c4ca6dc53d1425`.

## Matched receiver results

The existing prism ATE evaluator uses the unchanged
`1730565903.588829994`–`1730566022.688653946` window, SE(3) alignment
without scale, and the known CPT7-to-AP20 lever arm. There are 954 matched
prism poses over 94.9 s in five GT segments, **not continuous GT for every
instant**. The IE-TC 1 s relative-pose evaluator has 1,181 valid pairs in
every row. Online means *as played at each arrival*; final means the
retrospectively revised graph after later factor revisions.

| CBS | Receiver addition | Output | ATE (m) | 1 s t-RPE (m) | 1 s r-RPE (deg) | Full path (m) | Prism-matched path (m) |
|---|---|---|---:|---:|---:|---:|---:|
| OFF | zero | online | 0.069367 | 0.027762 | 1.750184 | 79.123 | 72.239 |
| OFF | GLIM only | online | 0.073664 | 0.028372 | 1.723902 | 79.155 | 72.263 |
| OFF | Kimera only | online | 0.066867 | 0.026403 | 1.739199 | 79.001 | 72.192 |
| OFF | both | online | 0.070616 | 0.027018 | 1.719477 | 79.018 | 72.204 |
| ON | zero | online | 0.078541 | 0.047590 | 1.911003 | 81.866 | 74.801 |
| ON | GLIM only | online | 0.085276 | 0.046729 | 1.896614 | 81.751 | 74.703 |
| ON | Kimera only | online | 0.056297 | 0.048190 | 1.906873 | 81.942 | 74.867 |
| ON | both | online | 0.061669 | 0.047372 | 1.888534 | 81.839 | 74.780 |
| OFF | zero | final | 0.047570 | 0.011439 | 0.380927 | 78.283 | 71.645 |
| OFF | GLIM only | final | 0.052806 | 0.011728 | 0.373714 | 78.269 | 71.630 |
| OFF | Kimera only | final | 0.043810 | 0.010865 | 0.378702 | 78.306 | 71.664 |
| OFF | both | final | 0.048888 | 0.011093 | 0.372128 | 78.292 | 71.650 |
| ON | zero | final | 0.066802 | 0.012325 | 0.651236 | 78.465 | 71.831 |
| ON | GLIM only | final | 0.074305 | 0.012204 | 0.634732 | 78.453 | 71.822 |
| ON | Kimera only | final | 0.041311 | 0.012660 | 0.652449 | 78.515 | 71.866 |
| ON | both | final | 0.048404 | 0.012437 | 0.636920 | 78.504 | 71.858 |

The same prism reference path is 71.778 m in all rows. These are **fused
gnssFGO receiver trajectories**, not native source trajectories. Native source
ATEs from the original live runs remain: OFF GLIM 0.007046 m, OFF Kimera
0.655087 m, ON GLIM 0.015811 m, ON Kimera 0.170926 m. This experiment did
not reevaluate or change those native outputs.

## Source-specific effect and limits of attribution

Compared with the zero-addition **ON** final graph, Kimera-only addition
reduces ATE by **25.49 mm** (`0.066802 → 0.041311 m`); GLIM-only addition
increases it by **7.50 mm** (`0.066802 → 0.074305 m`). Adding both reduces ON
ATE by 18.40 mm (`→ 0.048404 m`), a weaker gain than Kimera-only. The ON
online ATE changes have the same directions: Kimera-only **−22.24 mm**,
GLIM-only **+6.74 mm**. For OFF final, Kimera-only lowers ATE by 3.76 mm,
whereas GLIM-only raises it by 5.24 mm. The 2×2 ATE interaction
`both − GLIM-only − Kimera-only + zero` is −0.41 mm for ON final and
−0.16 mm for OFF final: small on this scoring mask relative to the
single-source effects, but the graph is nonlinear and the effects should
not be assumed generally additive.

With Kimera-only addition, CBS-ON's fused ATE is below the matched CBS-OFF
case by **10.57 mm online** (`0.056297` versus `0.066867 m`) and **2.50 mm
final** (`0.041311` versus `0.043810 m`). With GLIM-only addition, ON remains
worse in ATE. Therefore, for these fixed values and this window, **reducing
the effective receiver information of the Kimera factors is the principal
source of the ATE reversal** in the prior both-source result; changing GLIM
works against it.

This is not a blanket improvement. Under Kimera-only addition, ON still has
worse 1 s t-RPE and r-RPE than OFF: online translation `0.048190` versus
`0.026403 m`, final translation `0.012660` versus `0.010865 m`, and final
rotation `0.652449` versus `0.378702 deg`. The ON final t-RPE even rises by
0.334 mm relative to ON zero, while its ATE falls. A lower ATE does not imply
better local motion. Nor does this establish that Kimera is intrinsically
“bad”: native ON Kimera improves markedly; the receiver may be responding to
source confidence, correlations, model discrepancy, or their interaction.
The controls do not distinguish those mechanisms or calibrate the new sigma
values statistically. They also do not justify adopting these values across
sequences or in the live receiver.

## Invariant and numerical checks

The [validation record](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_source_model_source_isolation_20260929/validation.json)
checks, for each new replay against its same-mode frozen zero control:

- identical support clocks, causal dispatch, unsupported-belief lists, and
  source accounting by byte hash;
- identical ordered revision identities and source-reported full 6×6 covariance
  entries; the unselected source's *effective* full matrix is also identical;
- only the chosen source's six diagonal effective-covariance entries gain the
  declared variance, with every off-diagonal/cross term unchanged;
- identical startup, 1,195 support states, factor-kind counts, accepted source
  IDs, and zero pending events; OFF has 1,190 GLIM and 591 Kimera final unique
  factors, ON has 1,190 and 593;
- exactly 954 prism matches and 1,181 IE-TC pairs for every new case, with
  matching evaluation bounds.

The largest effective-covariance construction error across the four controls
is `9.53e-22` in covariance units. New TUM SHA-256 hashes are retained in
`validation.json`. No production source code was changed for this follow-up.
The four controls used the previously built and focused-tested receiver; a
new clean build, full suite, sanitizer run, or repeated deterministic replay
was **not** performed here. The original worktree was already dirty and was
left that way; no commit was made. gnssFGO HEAD remains
`a842e7ef72e7a40f0cdfec826f089c66af364ae0`.

## Reproduction and decision

The [artifact directory](/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_spx3_source_model_source_isolation_20260929/)
contains the predeclared controls, four complete receiver outputs,
`scores.json`, `comparison.csv`, `contrasts.json`, evaluation JSON, exact
validation scripts/results, and the unchanged source journals are referenced
by hash above. Each new case uses the prior replay command and **one** of:

```text
--source-model-noise glim_hesai,0.0000561019904579,0.000244015026860
--source-model-noise kimera_front,0.0000450299900067,0.000601137810046
```

The resulting scientific conclusion is narrow: the ATE benefit of the tested
both-source additive setting came from the Kimera-side addition; GLIM-side
addition reduced that benefit. Neither fixes the CBS-ON local RPE deficit.
The saved production receiver configuration remains at **zero addition**.
