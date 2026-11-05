"""Repair only the Log/Exp representation check; preserve the pinned audit."""
from pathlib import Path
import hashlib
import json
import numpy as np

Q = Path(__file__).resolve().parent
H = Q.parent
P = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_gravity_projection_cpv9_20261026/prepared')
S = P/'full_on_source'
ORIGINAL_SHA = '0dceae64ffa1dc7ad3275e3a30a7138916efc0078fe12927837f9be49a982eee'
ORTHOGONALITY_LIMIT = 1e-10

def sha(p):
    with p.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def same(x, y):
    return np.allclose(x, y, rtol=1e-9, atol=1e-12)

def mean_check(actual, expected):
    assert actual.shape == expected.shape == (4, 4)
    assert np.isfinite(actual).all() and np.isfinite(expected).all()
    assert np.array_equal(actual[3], expected[3])
    R = actual[:3, :3]
    U, singular, Vt = np.linalg.svd(R)
    orthogonality = float(np.linalg.norm(R.T@R-np.eye(3), ord='fro'))
    assert np.linalg.det(R) > 0 and np.linalg.det(U@Vt) > 0
    assert orthogonality <= ORTHOGONALITY_LIMIT, orthogonality
    normalized = actual.copy()
    normalized[:3, :3] = U@Vt
    assert same(normalized, expected), float(np.max(abs(normalized-expected)))
    return dict(raw_pass=bool(same(actual, expected)),
                raw_error=float(np.max(abs(actual-expected))),
                orthogonality_fro=orthogonality,
                normalized_error=float(np.max(abs(normalized-expected))),
                stretch_error=float(np.max(abs(singular-1))))

def negative_controls():
    from scipy.spatial.transform import Rotation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec([.1, -.2, .3]).as_matrix()
    T[:3, 3] = [.05, -.03, .2]
    assert mean_check(T, T)['raw_pass']
    bad = []
    wrong_rotation = T.copy()
    wrong_rotation[:3, :3] = T[:3, :3]@Rotation.from_rotvec([1e-6, 0, 0]).as_matrix()
    bad.append(wrong_rotation)
    wrong_translation = T.copy(); wrong_translation[0, 3] += 1e-6
    bad.append(wrong_translation)
    wrong_scale = T.copy(); wrong_scale[:3, :3] *= 1+1e-6
    bad.append(wrong_scale)
    reflection = T.copy(); reflection[:3, 0] *= -1
    bad.append(reflection)
    for value in bad:
        try:
            mean_check(value, T)
        except AssertionError:
            continue
        raise AssertionError('negative mean control accepted')
    C = np.eye(6)*1e-5
    changed = C.copy(); changed[0, 0] *= 1.01
    assert not same(C, changed)
    return dict(rotation_1e6rad_rejected=True, translation_1e6m_rejected=True,
                nonorthogonal_scale_rejected=True, reflection_rejected=True,
                covariance_one_percent_rejected=True)

def main():
    assert not (Q/'audit.json').exists()
    assert not (S/'projection_validation.json').exists()
    original = H/'validate_projection.py'
    assert sha(original) == ORIGINAL_SHA
    inputs = [original, H/'failed.json', H/'status.json', H/'closeout.json',
              S/'estimators.log', S/'run_metadata.txt',
              *S.glob('ros_logs/*/*stdout.log')]
    inputs = {str(p): sha(p) for p in inputs}
    negatives = negative_controls()
    rows = []
    jacobian_errors = []
    code = original.read_text()
    old = '  assert same(C,frozen[k][1]) and same(Z,a.g.exp(frozen[k][0]))'
    new = '''  assert same(C,frozen[k][1])
  mean_diagnostic=mean_check(Z,a.g.exp(frozen[k][0]))
  mean_diagnostic['edge']=k
  rows.append(mean_diagnostic)'''
    assert code.count(old) == 1
    code = code.replace(old, new)
    before = '  max_cov=max(max_cov,float(np.max(np.abs(Cy-M@C@M.T))))'
    assert code.count(before) == 1
    code = code.replace(before, '''  jacobian_errors.append(max(float(np.max(abs(Hi-numeric[0]))),float(np.max(abs(Hj-numeric[1])))))
'''+before)
    # Do not let the original serializer write before input conservation passes.
    cut = "with (S/'projection_validation.json').open('x') as f:json.dump(result,f,indent=2)"
    assert code.count(cut) == 1
    code = code.split(cut)[0]
    env = dict(__file__=str(original), mean_check=mean_check, rows=rows,
               jacobian_errors=jacobian_errors)
    exec(compile(code, str(original)+' [report-only representation repair]', 'exec'), env)
    assert all(sha(Path(p)) == want for p, want in inputs.items())
    assert len(rows) == 1908 == len(jacobian_errors)
    diagnostic = dict(passed=True, original_validator_sha256=ORIGINAL_SHA,
        method='polar rotation equality under original tolerance; raw residual/Jacobians unchanged',
        orthogonality_limit_fro=ORTHOGONALITY_LIMIT,
        original_raw_failures=[r for r in rows if not r['raw_pass']],
        maxima={k:max(r[k] for r in rows) for k in
                ('raw_error','orthogonality_fro','normalized_error','stretch_error')},
        runtime_jacobian_max_error=max(jacobian_errors), negative_controls=negatives,
        input_sha256=inputs, no_source_rerun=True, no_runtime_changes=True)
    result = env['result']
    result['report_only_repair'] = str(Q/'audit.json')
    result['mean_representation_method'] = diagnostic['method']
    result['original_raw_comparison_failed_edges'] = len(diagnostic['original_raw_failures'])
    for path, value in [(Q/'audit.json', diagnostic), (S/'projection_validation.json', result)]:
        with path.open('x') as f:
            json.dump(value, f, indent=2, allow_nan=False)
    print(json.dumps(dict(result=result, maxima=diagnostic['maxima'],
                         runtime_jacobian_max_error=max(jacobian_errors)), indent=2))

if __name__ == '__main__':
    main()
