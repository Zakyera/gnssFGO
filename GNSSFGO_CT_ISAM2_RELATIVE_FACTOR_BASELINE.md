# gnssFGO continuous-time relative-factor baseline

Date: 2026-09-21

## Executive result

The maintained gnssFGO stack can represent a relative `Pose3` measurement whose two endpoints are at arbitrary interpolated times and can optimize it with the repository's `IncrementalFixedLagSmoother` without retaining the full trajectory.

The pre-existing `GPInterpolatedDoublePose3BetweenFactor` is not safe when both measurement endpoints fall inside the same continuous-time support interval: its 12-key interface repeats the same six keys. The initial regression reached the correct zero residual but segfaulted when iSAM2 linearized/updated this repeated-key factor. Its analytical velocity/angular-rate Jacobian chains also omit the `Between()` pose Jacobian, producing finite-difference errors as large as approximately `0.273` in the synthetic case.

An isolated factor was therefore added for the same-support case:

`GPInterpolatedSameIntervalPose3BetweenFactor`

It owns six unique support keys, interpolates both endpoints, preserves the full dense 6x6 measurement covariance, sums both endpoint contributions into each support-state Jacobian, runs through iSAM2, exposes posterior interpolation covariance through support-state marginals, and survives ordinary streaming fixed-lag marginalization.

No dataset, GNSS, LiDAR, camera, GrandTour, CBS, or EKF experiment was run.

## Repository provenance

- Working repository: `/home/yeranis/repos/gnssFGO`
- User fork (`origin`): `https://github.com/Zakyera/gnssFGO.git`
- Official maintained upstream (`upstream`): `https://github.com/hz658832/gnssFGO.git`
- Archived original (`archive`): `https://github.com/rwth-irt/gnssFGO.git`
- Starting fork commit: `012b95296ac20b800ecb3c36e3c95229df05ae51`
- Official upstream `ros2` tip inspected: `16030fb6a24263a4b927d2ec5d1da266340ddb9b`
- Adapted GTSAM in the official image: `b5a4359d5e6d40c71e26633c961a0fd99ed72f06`
- Build image: `haomingac/gnssfgo:latest` (`sha256:dff747...` locally)

The user fork and official repository diverge from `b1fb2f5799cc2765dbb169b2b8154094ffd5faee`:

- the user fork has three code commits adding offline datasets, visual-odometry utilities, and GP-interpolator fixes;
- the official branch has two later Docker/build commits.

The official repository is now configured as `upstream`; the user fork remains `origin`. The user fork's additional code is intentionally retained rather than discarded.

## Existing gnssFGO components

The relevant framework was already present:

- `GPWNOAInterpolator` evaluates a pose inside a support interval from the two endpoint pose/velocity/angular-rate states.
- `GPWNOAPrior` supplies the WNOA motion prior between adjacent support states.
- `GPInterpolatedDoublePose3BetweenFactor` interpolates both ends of a relative pose when the endpoints use distinct support-key tuples.
- `IncrementalFixedLagSmoother` wraps the adapted GTSAM iSAM2 implementation and marginalizes states older than the configured lag.
- `LIOIntegrator` already selects ordinary, single-interpolated, or double-interpolated relative-pose factors depending on endpoint synchronization.

The existing Aachen time-centric configuration selects `IncrementalFixedLag` with a 3 s lag. WNOA code exists even though some configurations choose other GP models.

## Synthetic problem

The regression uses only two WNOA support states:

```text
t0 = 0 s
t1 = 1 s
```

and a dense-covariance relative pose measurement:

```text
ta = 0.23 s
tb = 0.71 s
Z  = T(ta)^-1 T(tb)
```

Both `ta` and `tb` are strictly inside the same `[t0,t1]` support interval. Constant translational velocity supplies an exact synthetic reference. The covariance is generated as `L L^T` with nonzero rotational, translational, and rotation/translation off-diagonal terms.

## Factor formulation

Let the six unique support variables be:

```text
x = [T_i, v_i, w_i, T_j, v_j, w_j].
```

Two WNOA interpolators give:

```text
T_a(x) = interpolate(x, tau_a)
T_b(x) = interpolate(x, tau_b)
T_hat_ab = T_a(x)^-1 T_b(x)
r = Local(Z, T_hat_ab).
```

For support variable `x_k`, the implemented Jacobian is:

```text
dr/dx_k = H_Local [ H_Between,a dT_a/dx_k
                   + H_Between,b dT_b/dx_k ].
```

The summation is essential because each support variable influences both interpolated measurement endpoints. It also lets the nonlinear factor expose each support key exactly once, as required by the Gaussian linearization/iSAM2 path.

The implementation is in:

- `online_fgo/include/factor/odometry/GPInterpolatedSameIntervalPose3BetweenFactor.h`

It is deliberately not wired into `LIOIntegrator` or any runtime configuration yet.

## Residual and Jacobian validation

Results on the exact synthetic motion:

- both interpolated poses equal their analytic references within `1e-10`;
- neither endpoint collapses to a support state;
- relative residual norm is below `1e-10`;
- all six support keys are unique;
- the factor noise model recovers the input dense 6x6 covariance within `1e-14`;
- all six analytical support-state Jacobians agree with central finite differences within `1e-3`.

The observed worst entrywise discrepancy after the correct chain rule was approximately `6.25e-4`. This remaining level comes from the inherited analytical `GPWNOAInterpolator` Jacobians. The earlier legacy-factor experiment produced errors of approximately `0.273` and `0.081` because several chains omitted the `Between()` pose Jacobian.

## Measurement covariance versus interpolation uncertainty

The original measurement covariance remains the factor's full dense `R_Z`. It is not diagonalized or inflated.

Inside the graph, GP interpolation uncertainty is represented by uncertainty in the WNOA support variables and their correlations. It belongs in the innovation through the factor Jacobian and graph covariance, not by copying the same state uncertainty into `R_Z`.

Posterior interpolation covariance can be recovered explicitly. If `P_support` is the 24x24 joint marginal covariance of:

```text
[T_i, v_i, w_i, T_j, v_j, w_j],
```

and `J_interp` stacks the two 6x24 interpolation Jacobians, then:

```text
P_[Ta,Tb] = J_interp P_support J_interp^T.
```

The test verifies that this 12x12 covariance is finite, symmetric, PSD within numerical tolerance, and has a nonzero cross-covariance between `T_a` and `T_b`.

For exporting a predicted relative-pose covariance outside the graph, the first-order state contribution is:

```text
P_interp_relative = J_between P_[Ta,Tb] J_between^T.
```

If the external measurement noise is independent of the CT-state estimate, a predicted measurement-difference covariance can be formed as:

```text
S = R_Z + P_interp_relative.
```

This sum is diagnostic/export mathematics. Adding `P_interp_relative` to the in-graph factor noise would double-count the support-state uncertainty already represented by the variables and GP prior.

## Incremental fixed-lag result

The relative factor was inserted with a WNOA prior and perturbed initial values into the repository's `IncrementalFixedLagSmoother`:

- graph linearization succeeded;
- the iSAM2 update succeeded;
- optimized graph error was lower than initial graph error;
- the posterior pose covariance was finite, symmetric, and PSD;
- interpolation and relative-pose covariances were recovered from joint marginals;
- a GP-connected state at `t=2.5 s` was then added with a 2 s lag;
- the `t=0` support state was marginalized;
- the `t=1` and `t=2.5` states remained.

This establishes the required finite-memory behavior for an ordinary streaming graph. It does not claim that every artificial graph teardown is supported: during diagnosis, asking this wrapper to retire the entire old connected component while adding only a disconnected set of priors segfaulted. The connected streaming scenario used by gnssFGO passes.

## Tests and build

Added test target:

```text
test_ct_relative_factor
```

Contained checks:

1. both endpoints are interpolated and the dense covariance is preserved;
2. unique-key analytical Jacobians match finite differences;
3. iSAM2 optimization, posterior/interpolation covariance recovery, and fixed-lag retirement work.

Validation:

```text
Release build: PASS (online_fgo and its nodes)
Focused CTest: PASS (1/1 target; 3/3 GoogleTests)
Formatter: PASS (ament_uncrustify on both new C++ files)
git diff --check: PASS
```

The package's pre-existing aggregate lint suite is not a clean baseline; it reports unrelated historical formatting/copyright issues. No attempt was made to reformat or repair unrelated source.

## Scope and next engineering step

This proves the minimum CT/iSAM2 primitive, not a complete sensor integration. No runtime factory currently selects the new factor.

Before using real odometry, the next bounded step should be:

1. add deterministic dispatch between ordinary, single-interpolated, distinct-support double-interpolated, and same-support double-interpolated cases;
2. correct and regression-test the missing chain-rule terms in the legacy distinct-support double factor;
3. test endpoints spanning adjacent and non-adjacent support intervals;
4. only then connect one saved relative-belief stream to an offline gnssFGO graph.

## Version-control result

- Scientific implementation/test commit: `4e85e01` (`Add same-interval CT relative pose factor test`)
- Official-upstream integration commit: `f447a46` (`Merge remote-tracking branch 'upstream/ros2' into ct-isam2-relative-factor`)
- Branch: `ct-isam2-relative-factor`
- The exact final HEAD after the report-only finalization commit is given in the task handoff; a commit cannot embed its own resulting hash.
