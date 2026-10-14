# FL4e — ETH1 CBS-ON fixed-lag `x71` failure, saved-journal diagnosis

Status: **isolated receiver-lifecycle candidate passes the saved-data gate; no new live source run or OFF/ON accuracy result** (2026-10-14).

## Scope and preserved inputs

This task follows the stopped [FL4d comparison](../fixed_lag_live_fl4d_20261014/REPORT.md). It uses only the existing ETH1 ON partial journal and completed FL4c OFF journal. It did not launch ROS, GLIM, Kimera, bag playback, a fresh receiver capture, or GT scoring. It did not change WNOA, source factors, measurement covariances, CBS, timing, source profiles, or installed GTSAM. The original receiver binary (SHA-256 `15a2ae458f5142c0b3341e9e444b25fc41f05a263cefbbb877c18c4a527718bc`) and 814 protected source/control artifacts remain unchanged.

## Reproduced failure and mechanism

Replay of the first 8,129 events from `pilot30_on_receiver/events.csv` with the original pinned binary reproduces exactly:

```
FL4A_STREAM_FAILURE Requested non-existent variable from VariableIndex: 120 71
```

The independent FL3h stock-slot receiver also fails at event 8,129 with the same error. Thus the failure predates FL3i's factor-slot reuse. At event 8,128, support 142 retires `x71/v71/w71`. The isolated [probe](Probe.cpp) found that *after* retirement `x71` was absent from the nonlinear factor graph, theta, node map, and VariableIndex, yet a surviving Bayes-tree clique still named it as a frontal variable (nine node-map entries shared that clique). The incoming Kimera factor at event 8,129 involved only retained support keys `x97`–`x100` and their velocity/angular-rate companions. Its ordinary iSAM2 update touched the inconsistent tree and exposed the stale `x71` reference. This is a retirement/re-elimination bookkeeping failure, not direct use of an expired Kimera endpoint and not a transport backlog.

The private fixed-lag wrapper had put only descendants of retiring keys into `extraReelimKeys`. At this event the constrained re-elimination left the retiring variables inside a surviving clique's frontal ordering. `marginalizeLeaves` removed them from the index but not that clique. This mechanism is supported by the before/after tree probe and by the one-line candidate below; it does not claim a general proof for every graph topology.

## Isolated change

The [private candidate](RecordedFixedLagSmoother.cpp) is a copy of FL3i's wrapper with one behavioral addition: include every `marginalizableKey` itself in `additionalKeys` passed to iSAM2's constrained re-elimination, along with the existing descendant keys. The process model, factors, Jacobians, noise, lag, and marginalization operation are unchanged. The frozen FL3i source and FL4a binary were not overwritten. Candidate source SHA-256: `2afbfcb8f51dfa9b4faea33639e503045463bbf1cc22dd1ad9b25950a427be69`; isolated candidate receiver binary SHA-256: `53c96a2b7a762800cf2291166dc78520269098684411de641b52bee6a54dc5c9`.

With the candidate, the same event 8,128 leaves **zero** clique references to `x71`; event 8,129 completes. This is a direct saved-input A/B check of the failure boundary.

## Bounded validation

| Check | Result |
|---|---|
| Original FL4a first 8,129 ON events | Reproduces missing `x71` error |
| Independent stock-slot FL3h first 8,129 ON events | Same error at event 8,129 |
| Candidate first 8,129 ON events | Pass; no residual `x71` clique reference |
| Candidate all 8,148 retained ON events | Pass; diagnostic-only synthetic EOF added because live capture was aborted |
| Candidate full 18,871-event completed OFF journal | Pass; 293 online/finalized supports, 15,884 belief versions, 420 inserts, 15,439 replacements, 25 pre-init exclusions, zero pending |
| Existing focused synthetic lifecycle gate | 50,818 checks pass in Release and ASan/UBSan |
| Candidate saved-journal repeat | OFF and partial ON archives, state CSVs, summaries byte-identical on separate repeats |
| Frozen FL4c OFF compared with candidate OFF on identical physical rows | Online position RMS `1.0135e-7 m`, max `5.0915e-7 m`; finalized position RMS `1.5505e-6 m`, max `1.4025e-5 m` |
| Frozen partial FL4d ON compared with candidate ON through its last published support | Online position RMS `7.3590e-8 m`, max `3.8115e-7 m` |
| Original protected manifest | 814/814 unchanged after tests |

The candidate and frozen OFF outputs are **not byte-identical**, as re-elimination ordering changes floating-point paths. The differences above are state differences between receiver implementations, not GT errors. The synthetic EOF on the partial ON journal establishes only that all *retained* events can be processed deterministically; it is not a completed 30-second ON experiment and cannot satisfy FL4d's native-source coverage, conservation, or GT gates. No throughput/real-time claim follows from these offline replays. Earlier FL3i strict full-history equivalence and nonlinear-stationarity limitations remain unchanged.

## Decision

The saved-data failure is resolved by one isolated ordering change and passes focused validation. **Do not compare a new ON live capture from this candidate binary directly with the old FL4c OFF fusion as a formally matched receiver-binary experiment.** The next authorized experiment should use the same candidate receiver binary for both modes—either a fresh matched live OFF/ON pair, or a separately declared comparison treating candidate replay of the saved OFF journal as an offline control. Keep the original/native source trajectories distinct from receiver outputs, and do not score the aborted FL4d ON capture.

gnssFGO HEAD: `d3d152a28440f97ced3bd7a50ee072874beafe19`. FL4e candidate and report are uncommitted; the repository has pre-existing unrelated dirty files. No production/default build was modified.
