"""Read only the bounded diagnostic; write new evidence only after validation."""
import csv
import hashlib
import json
import os
from pathlib import Path

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
import numpy as np

HERE = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
R = W / 'runs/fixed_lag_linear_parity_fl4n_20261015'
OLD = W / 'runs/fixed_lag_retirement_fl4m_20261015'
D = R / 'remove_capture'


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(8 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def native(path):
    rows = list(csv.DictReader(path.open()))
    ignored = [x for x in rows if x.get('field.header.seq') is None]
    assert all(x['%time'] in [
        'WARNING: no messages received and simulated time is active.',
        'Is /clock being published?'] for x in ignored), ignored
    return [x for x in rows if x.get('field.header.seq') is not None]


def capture(update):
    p = D / 'linear_parity' / ('update_' + str(update))
    s = {k: float(v) for k, v in (line.split('=') for line in
         (p / 'summary.txt').read_text().splitlines())}
    assert s['complete'] == 1
    factors = list(csv.DictReader((p / 'factors.csv').open()))
    families = {}
    for row in factors:
        f = families.setdefault(row['family'], dict(count=0, unequal_keys=0,
            max_H_relative=0., max_g_relative=0.))
        f['count'] += 1
        f['unequal_keys'] += int(row['keys_identical'] != '1')
        f['max_H_relative'] = max(f['max_H_relative'], float(row['H_relative']))
        f['max_g_relative'] = max(f['max_g_relative'], float(row['g_relative']))
    arrays = {n: np.loadtxt(p / (n + '.txt'), delimiter=',') for n in
              ['cache_H', 'tree_H', 'fresh_H', 'cache_g', 'tree_g', 'fresh_g', 'delta']}
    assert all(np.isfinite(v).all() for v in arrays.values())
    H = arrays['cache_H']
    inv = 1 / np.sqrt(np.maximum(np.abs(np.diag(H)), 1e-24))
    scaled = inv[:, None] * H * inv[None, :]
    eig = np.linalg.eigvalsh((scaled + scaled.T) / 2)
    s['scaled_H_min_eigenvalue'] = float(eig[0])
    s['scaled_H_max_eigenvalue'] = float(eig[-1])
    s['scaled_H_condition_estimate'] = float(eig[-1] / eig[0]) if eig[0] > 0 else None
    keys = [line.split() for line in (p / 'keys.txt').read_text().splitlines()]
    s['keys_by_type'] = {}
    offset = 0
    for name, dim, fixed in keys:
        dim, fixed = int(dim), int(fixed)
        f = s['keys_by_type'].setdefault(name[0], dict(count=0, fixed=0, max_delta_norm=0.))
        f['count'] += 1
        f['fixed'] += fixed
        f['max_delta_norm'] = max(f['max_delta_norm'], float(np.linalg.norm(arrays['delta'][offset:offset+dim])))
        if name[0] == 'x':
            assert dim == 6
            f['max_phi_deg'] = max(f.get('max_phi_deg', 0.),
                float(np.linalg.norm(arrays['delta'][offset:offset+3]) * 180 / np.pi))
            f['max_rho_m'] = max(f.get('max_rho_m', 0.),
                float(np.linalg.norm(arrays['delta'][offset+3:offset+6])))
        offset += dim
    assert offset == len(arrays['delta'])
    s['factor_families'] = families
    s['fresh_H_bitwise_numeric_equal'] = bool(np.array_equal(H, arrays['fresh_H']))
    s['fresh_g_bitwise_numeric_equal'] = bool(np.array_equal(arrays['cache_g'], arrays['fresh_g']))
    return s


def main():
    validation = json.loads((D / 'validation.json').read_text())
    assert validation['valid'] and not (D / 'failed.txt').exists()
    a, b = native(D / 'native.log'), native(OLD / 'remove_capture/native.log')
    assert len(a) == len(b) == 454
    fields = [k for k in a[0] if k != '%time']
    differing = [int(x['field.header.seq']) for x, y in zip(a, b)
                 if any(x[k] != y[k] for k in fields)]
    assert all(x['field.header.stamp'] == y['field.header.stamp'] for x, y in zip(a, b))
    numeric = [k for k in fields if k not in ['field.header.frame_id', 'field.child_frame_id']]
    # Preserve text identity separately; large timestamp integers are not used
    # for numerical state-error estimates.
    state_fields = [k for k in numeric if k.startswith(('field.pose.', 'field.twist.'))]
    max_state_field_difference = max(abs(float(x[k]) - float(y[k]))
                                   for x, y in zip(a, b) for k in state_fields)
    objective = np.loadtxt(D / 'objective.csv', delimiter=',')
    old_objective = np.loadtxt(OLD / 'remove_capture/objective.csv', delimiter=',')
    assert objective.shape == old_objective.shape
    protected = json.loads((R / 'protected_before.json').read_text())
    changed = [p for p, h in protected.items() if not Path(p).exists() or sha(Path(p)) != h]
    assert not changed, changed
    flags = json.loads((D / 'preflight.json').read_text())['flags']
    old_flags = json.loads((OLD / 'remove_capture/preflight.json').read_text())['flags']
    assert all(flags[k] == old_flags[k] for k in flags if k.startswith('cbs_'))
    old_profile = OLD / 'kimera_profile'
    for p in (R / 'kimera_profile').rglob('*'):
        if p.is_file(): assert sha(p) == sha(old_profile / p.relative_to(R / 'kimera_profile'))
    assert sha(R / 'aligned_inputs.bag') == sha(OLD / 'aligned_inputs.bag')
    snapshots = {str(u): capture(u) for u in [28, 89, 438]}
    for u, s in snapshots.items():
        row = objective[objective[:, 0] == int(u)]
        assert len(row) == 1
        s['objective_row'] = row[0].tolist()
    result = dict(validation=validation, completion=json.loads((D / 'complete.json').read_text()),
        protected_files_unchanged=len(protected), same_CBS_flags=True, same_profile=True,
        same_frozen_input_bag=True, native_messages_differing_excluding_receipt=differing,
        max_native_state_field_difference=max_state_field_difference,
        exact_objective_repeat=bool(np.array_equal(objective, old_objective)),
        max_objective_absolute_difference=float(np.max(np.abs(objective - old_objective))),
        captures=snapshots,
        private_manifest={str(p.relative_to(R)): sha(p) for p in [
            R / 'private_build/libkimera_vio.so', R / 'private_build/libcbs.so',
            R / 'test', R / 'LinearParityAudit.h', R / 'aligned_inputs.bag',
            R / 'include/cbs/bpsam/bpsam.h',
            R / 'include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h']})
    for target in [R / 'analysis.json', HERE / 'receipt.json']:
        with target.open('x') as f: json.dump(result, f, indent=2, allow_nan=False)
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
