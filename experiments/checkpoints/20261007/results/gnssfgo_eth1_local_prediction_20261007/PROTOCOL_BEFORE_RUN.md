# Correct local prediction inside bounded private numerical refinement

The previous bounded-private trial stopped at ON/WNOJ support1155 on pass4:
gradient1.01288469e-5; cumulative actual cost change1.44148e-9; predicted change
from the OLD anchor linearization+2.12647e-10. That last quantity is not the
predicted descent of the current private Gauss-Newton step.

For private pass k, compute actual objective change against the fixed retained
anchor, but compute predicted decrease using the fresh linearization at the
current private seed. The final consensus/gradient requirements and all numeric
tolerances remain unchanged. Cumulative cost remains bounded against the same
anchor; no accumulation or gradient relaxation is permitted. Preserve every
factor slot/pointer and commit the entire solver only after convergence.

No model, Qc, covariance, prior, measurement, event or support-grid change.
Run focused Release/ASan/UBSan first, then frozen OFF/ON WNOA/WNOJ and repeats.
This is an offline numerical validation, not a new source experiment or tuning.
