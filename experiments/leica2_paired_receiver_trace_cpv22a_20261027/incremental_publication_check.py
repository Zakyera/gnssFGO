"""Saved-snapshot test of the user's Kimera-style publication hypothesis.

No receiver/source replay, optimization, changed covariance or delayed sample.
Retain original absolute ONLINE and compute a distinct causal odometry product.
"""
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

H = Path(__file__).resolve().parent
KIMERA = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/fixed_lag_current_chart_fl4q_20261016/VioBackend.cpp')
TP = np.array([.308837802, -.038312997, -.234944116])
pins = {}
out = {'cases': {}, 'input_sha256': pins}


def pin(path):
    with path.open('rb') as f:
        pins[str(path)] = hashlib.file_digest(f, 'sha256').hexdigest()
    return path


def norm(x):
    return np.linalg.norm(x, axis=-1)


def pose(row):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(row[5:9]).as_matrix()
    result[:3, 3] = row[2:5]
    return result


def inverse(T):
    result = np.eye(4)
    result[:3, :3] = T[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ T[:3, 3]
    return result


def points(poses, Q, shift):
    return np.array([T[:3, 3] + T[:3, :3] @ TP for T in poses]) @ Q.T + shift


def metrics(x, gt):
    step = np.diff(x, axis=0) - np.diff(gt, axis=0)
    anchored_error = (x - x[0]) - (gt - gt[0])
    return {
        'path_m': float(norm(np.diff(x, axis=0)).sum()),
        'step_vector_rms_m': float(np.sqrt(np.mean(norm(step)**2))),
        'position_rmse_original_fixed_fit_m': float(np.sqrt(np.mean(norm(x - gt)**2))),
        'position_rmse_common_start_translation_m': float(np.sqrt(np.mean(norm(anchored_error)**2))),
        'endpoint_displacement_error_m': float(norm(anchored_error[-1])),
        'one_second_displacement_vector_rms_m': float(np.sqrt(np.mean(norm((x[10:] - x[:-10]) - (gt[10:] - gt[:-10]))**2))),
    }


assert not (H / 'incremental_publication_check.json').exists()
source = pin(KIMERA).read_text()
assert 'W_Pose_B_lkf.between(W_Pose_B_kf)' in source
assert 'W_Pose_B_lkf_from_increments_.compose(B_lkf_Pose_kf)' in source
assert '(FLAGS_no_incremental_pose ? W_Pose_B_lkf_from_state_' in source
pin(H.parent / 'receiver_finalization_cpk19_20261021/Receiver.h')
pin(H.parent / 'receiver_finalization_cpk19_20261021/Live.cpp')
cache = np.load(pin(H.parent / 'leica2_sensor_jitter_audit_cpv21_20261027/timeseries.npz'))
arrays = {}
fig, axes = plt.subplots(3, 2, figsize=(13, 11), constrained_layout=True)
for col, mode in enumerate(('hesai', 'velodyne')):
    prepared = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs') / (
        'gnssfgo_leica2_full_cpk29_20261023' if mode == 'hesai' else 'gnssfgo_leica2_velodyne_full_cpv4_20261024') / 'prepared'
    effective = json.loads(pin(prepared / 'full_off_source/effective_mode_check.json').read_text())
    assert '--no_incremental_pose=false' in effective['kimera_argv']
    manifest = json.loads(pin(prepared / 'protected_manifest.json').read_text())
    assert manifest[str(KIMERA)] == pins[str(KIMERA)]
    view = H.parent / ('leica2_hesai_rerun_cpv20_20261027' if mode == 'hesai' else 'leica2_velodyne_rerun_cpv11_20261026')
    view = json.loads(pin(view / 'data_checks.json').read_text())
    fit = view['products']['off_fusion_online']
    Q = np.array(fit['alignment_rotation'])
    shift = np.array(fit['alignment_translation_m']) - np.array(view['origin_reference_enu_m'])
    pub = np.load(pin(H / f'{mode}_publication_states.npz'))
    rows = {(int(k), int(i)): row for k, i, row in zip(pub['newest'], pub['index'], pub['raw_states'])}
    t = cache[mode + '_time_s']
    indices = np.flatnonzero((t > 0) & (t <= 60))
    assert len(indices) == 596 and np.array_equal(indices, np.arange(596))
    absolute = [pose(rows[(i, i)]) for i in indices]
    incremental = [absolute[0].copy()]
    control = [absolute[0].copy()]
    current_edge_lengths, previous_state_revisions = [], []
    for i in indices[1:]:
        prev = pose(rows[(i, i-1)])
        curr = absolute[i]
        delta = inverse(prev) @ curr
        incremental.append(incremental[-1] @ delta)
        control.append(control[-1] @ inverse(absolute[i-1]) @ curr)
        pair = points([prev, curr], Q, shift)
        current_edge_lengths.append(float(norm(pair[1] - pair[0])))
        old = points([absolute[i-1]], Q, shift)[0]
        previous_state_revisions.append(pair[0] - old)
    absolute_xyz = points(absolute, Q, shift)
    incremental_xyz = points(incremental, Q, shift)
    control_xyz = points(control, Q, shift)
    original = cache[mode + '_online'][indices]
    gt = cache[mode + '_reference'][indices]
    assert np.max(norm(absolute_xyz - original)) < 1e-10
    assert np.max(norm(control_xyz - original)) < 1e-9
    assert np.max(np.abs(norm(np.diff(incremental_xyz, axis=0)) - current_edge_lengths)) < 1e-9
    # Exact vector identity: published step = same-snapshot edge + previous-state revision.
    edge = np.array([points([pose(rows[(i, i-1)]), absolute[i]], Q, shift)[1] - points([pose(rows[(i, i-1)]), absolute[i]], Q, shift)[0] for i in indices[1:]])
    closure = np.diff(original, axis=0) - edge - previous_state_revisions
    assert np.max(norm(closure)) < 1e-9
    out['cases'][mode] = {
        'kimera_incremental_flag_enabled': True, 'rows': len(indices),
        'interval_s': [float(t[indices[0]]), float(t[indices[-1]])],
        'reference_path_m': float(norm(np.diff(gt, axis=0)).sum()),
        'absolute_online': metrics(original, gt),
        'kimera_style_chained': metrics(incremental_xyz, gt),
        'control_published_to_published_chain_max_difference_m': float(norm(control_xyz - original).max()),
        'same_snapshot_edge_length_preservation_max_m': float(np.max(np.abs(norm(np.diff(incremental_xyz, axis=0)) - current_edge_lengths))),
        'published_step_decomposition_max_m': float(norm(closure).max()),
        'previous_state_revision_norm_p95_m': float(np.percentile(norm(previous_state_revisions), 95)),
        'chained_vs_current_graph_final_position_m': float(norm(incremental_xyz[-1] - original[-1])),
        'chained_vs_current_graph_final_orientation_deg': float(np.rad2deg(Rotation.from_matrix(incremental[-1][:3, :3].T @ absolute[-1][:3, :3]).magnitude())),
    }
    for key, value in [('time_s', t[indices]), ('absolute', original), ('chained', incremental_xyz), ('reference', gt), ('chained_body_T', np.array(incremental))]:
        arrays[f'{mode}_{key}'] = value
    labels = [('Absolute graph ONLINE', original, '#d52f48'), ('Chained current-window increments', incremental_xyz, '#a16b00'), ('IE-TC reference', gt, '#2877c7')]
    for label, data, color in labels:
        axes[0, col].plot(data[:, 0], data[:, 1], color=color, label=label, lw=1.2)
        axes[1, col].plot(t[indices][1:], norm(np.diff(data, axis=0))*100, color=color, label=label, lw=.85)
        if label != 'IE-TC reference':
            error = norm((data-data[0]) - (gt-gt[0]))
            axes[2, col].plot(t[indices], error, color=color, label=label, lw=1.2)
    axes[0, col].set(title=mode.upper()+' CBS OFF — same graph snapshots', xlabel='Reference-aligned East (m)', ylabel='North (m)')
    axes[0, col].axis('equal')
    axes[1, col].set(xlabel='Capture-relative time (s)', ylabel='100 ms edge length (cm)', xlim=(0, 60), ylim=(0, 32))
    axes[2, col].set(xlabel='Capture-relative time (s)', ylabel='Position error after common start (m)', xlim=(0, 60))
    for ax in axes[:, col]:
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
fig.suptitle('Publication policy diagnostic: absolute graph snapshots vs chained relative motion\nFirst minute only; no new optimization, filtering, added state age or weight changes. Original rigid orientation fits retained.')
fig.savefig(H / 'incremental_publication_check.png', dpi=150)
plt.close(fig)
out['semantics'] = {
    'chained': 'O[k]=O[k-1] inverse(X[k-1|publication k]) X[k|publication k]. Uses only current publication information.',
    'control': 'Chaining consecutive previously published absolute poses telescopes to the original and cannot remove zigzag.',
    'limits': 'First-minute saved-snapshot output-only reconstruction. Chained odometry differs from optimized graph and discards historical pose corrections from persistent history. No full-run ATE, calibrated covariance or estimator fix claim.',
}
pin(Path(__file__))
for path, expected in pins.items():
    with open(path, 'rb') as f:
        assert hashlib.file_digest(f, 'sha256').hexdigest() == expected
np.savez_compressed(H / 'incremental_publication_states.npz', **arrays)
with (H / 'incremental_publication_check.json').open('x') as f:
    json.dump(out, f, indent=2)
    f.write('\n')
print(json.dumps(out['cases'], indent=2))
