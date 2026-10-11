"""Read-only numerical comparison of the retained failed repeat; no reruns."""
from pathlib import Path
import csv
import hashlib
import json
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parent
live = ROOT/'gate_on_receiver/receiver'
repeat = ROOT/'gate_on_repeat'
names = ['trajectory.tum', 'states.csv', 'online_trajectory.tum',
         'online_states.csv', 'support_clock.csv', 'source_model_covariance.csv',
         'model_covariance.csv', 'causal_dispatch.csv', 'measurement_events.csv']
result = {'status': 'EXACT_REPEAT_FAILED', 'new_experiment_or_replay_run': False,
          'full40_started': False, 'files': {}}
for name in names:
    a, b = (live/name).read_bytes(), (repeat/name).read_bytes()
    info = {'byte_identical': a == b,
            'live_sha256': hashlib.sha256(a).hexdigest(),
            'repeat_sha256': hashlib.sha256(b).hexdigest()}
    if name.endswith('.tum'):
        x, y = np.loadtxt(live/name), np.loadtxt(repeat/name)
        assert x.shape == y.shape and np.array_equal(x[:, 0], y[:, 0])
        dp = np.linalg.norm(x[:, 1:4]-y[:, 1:4], axis=1)
        dq = (Rotation.from_quat(x[:, 4:8]).inv()*Rotation.from_quat(y[:, 4:8])).magnitude()
        info.update(rows=len(x), timestamps_identical=True,
                    position_difference_max_m=float(dp.max()),
                    position_difference_RMS_m=float(np.sqrt(np.mean(dp**2))),
                    orientation_difference_max_rad=float(dq.max()))
    else:
        with (live/name).open() as f: x = list(csv.DictReader(f))
        with (repeat/name).open() as f: y = list(csv.DictReader(f))
        info.update(live_rows=len(x), repeat_rows=len(y))
        if x and len(x) == len(y):
            differences = {}
            for col in x[0]:
                ax, by = [r[col] for r in x], [r[col] for r in y]
                if ax == by: continue
                try:
                    ax, by = np.asarray(ax, dtype=float), np.asarray(by, dtype=float)
                    finite = np.isfinite(ax) & np.isfinite(by)
                    differences[col] = {'max_abs_numeric_difference': float(np.max(np.abs(ax[finite]-by[finite]))) if finite.any() else None,
                                        'nonfinite_mask_equal': bool(np.array_equal(np.isfinite(ax), np.isfinite(by)))}
                except (ValueError, TypeError):
                    differences[col] = {'different_text_rows': sum(a != b for a, b in zip(ax, by))}
            info['different_columns'] = differences
    result['files'][name] = info
sa = json.loads((live/'summary.json').read_text())
sb = json.loads((repeat/'summary.json').read_text())
result['summary_differences'] = {k: [sa.get(k), sb.get(k)] for k in set(sa)|set(sb) if sa.get(k) != sb.get(k)}
for tag, directory in [('live', live), ('repeat', repeat)]:
    with (directory/'publication_convergence.csv').open() as f: rows = list(csv.DictReader(f))
    result[tag+'_stationarity'] = {'rows': len(rows), 'all_stationary': all(r['status']=='stationary' for r in rows),
                                  'max_gradient_after': max(float(r['gradient_after']) for r in rows)}
with (ROOT/'repeat_failure_audit.json').open('x') as f: json.dump(result, f, indent=2)
print(json.dumps(result, indent=2))
