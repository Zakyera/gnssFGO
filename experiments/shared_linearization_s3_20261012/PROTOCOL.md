# Shared native Gaussian shadow integration

S3 parent: `030bc8cef71e23a9758b28c70b7a4d5e2e5db66e`. Preserve the S2
backend byte for byte. Build overlays in a new private directory only. No source
checkout, installed library, old artifact or estimator configuration is changed.

## Scope and acceptance

One hypothesis: the real MAIN fixed-lag lifecycle can safely supply S2's native
Gaussian history. A positive native admission occurs before the bridge appends
CBS factors. A thread-local update scope observes the actual inner solver after
update and before retirement, then stages after retirement and commits only after
the outer fallback wrapper succeeds. Extra empty updates are observed. Explicit
fallback invalidates the candidate permanently, including fallback outside update.
No observer exception may reach MAIN. Default mode is off. No class layout changes.

Native IDs are monotonically assigned within one registered solver lifetime;
ownership prevents pointer reuse while an ID is active. Slots are assigned from
actual insertion results and checked against retained nonlinear factor pointers.
Each captured update receives a new equation-snapshot version (not a claim that
MAIN relinearized every factor). Native factors are copied before retirement;
MAIN's marginal factors are never admitted. Old native priors keep their charts.

Keep independent LOCAL covariance authoritative. The private bridge may compute
and log a shadow covariance using the already-requested endpoint means, but may
not substitute it. No candidate production mode is implemented at this gate.

Before any data capture, require default-off and enabled actual-wrapper fixtures,
multiple retirements, same-class native/external factors, slot reuse/removal,
empty updates, fallback and observer failure containment. Compare MAIN states,
cached equations and native call counts on identical deterministic inputs, exactly
where practicable (state/cache tolerance 1e-12). For native Gaussian/dense Schur
comparison use relative Frobenius 1e-9; covariance 1e-8. Keep S2's no-loading rank
policy and derivative tests. Run Release twice and focused ASan/UBSan on new code.
Prebuilt dependencies are not sanitizer-instrumented.

The retained B0 captures contain source/receiver journals and summaries, not a
complete MAIN pre-retirement native Gaussian lifecycle. LOCAL checker snapshots
are not a substitute. Do not claim real GPU/lifecycle validation from synthetic
fixtures. A fresh bounded shadow capture is a subsequent explicit launch decision.
Do not begin S4, publish candidate covariance, tune anything, or run a full bag.
