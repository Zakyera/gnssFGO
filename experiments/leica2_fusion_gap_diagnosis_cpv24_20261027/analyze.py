"""Read-only journal/snapshot arithmetic; no optimizer or sensor execution."""
import csv
import hashlib
import heapq
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

H = Path(__file__).resolve().parent
E = H.parent
H23 = E / 'leica2_incremental_full_cpv23_20261027'
R23 = Path('/home/yeranis/results/gnssfgo_leica2_incremental_full_cpv23_20261027')
P = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_velodyne_full_cpv4_20261024/prepared')
PERIOD = 100_000_000
TP = np.array([.308837802, -.038312997, -.234944116])
SOURCES = ('glim_velodyne', 'kimera_front')
pins = {}


def pin(path):
    with path.open('rb') as f:
        pins[str(path)] = hashlib.file_digest(f, 'sha256').hexdigest()
    return path


def stats(x):
    x = np.asarray(x)
    if not x.size:
        return {'n': 0}
    assert np.isfinite(x).all()
    return dict(n=len(x), median=float(np.median(x)), p95=float(np.percentile(x, 95)),
                max=float(np.max(x)), rms=float(np.sqrt(np.mean(x*x))))


def norm(x):
    return np.linalg.norm(x, axis=-1)


def points(T):
    return T[:, :3, 3] + T[:, :3, :3] @ TP


def pose(row):
    T = np.eye(4)
    T[:3, 3] = [float(row[key]) for key in ('px', 'py', 'pz')]
    T[:3, :3] = Rotation.from_quat([float(row[key]) for key in ('qx', 'qy', 'qz', 'qw')]).as_matrix()
    return T


def ceil_support(stamp, origin):
    assert stamp >= origin
    return (stamp - origin + PERIOD - 1) // PERIOD


assert ceil_support(1000, 1000) == 0
assert ceil_support(1001, 1000) == 1
assert ceil_support(1000 + PERIOD, 1000) == 1
assert ceil_support(1001 + PERIOD, 1000) == 2
assert not (H / 'results.json').exists()
prod = np.load(pin(H23 / 'publication_products.npz'))
previous_results = json.loads(pin(H23 / 'results.json').read_text())
results = {'task': 'CP-V24', 'Qc_multiplier': 1.0, 'modes': {}}
arrays = {}

for mode in ('off', 'on'):
    with pin(R23 / f'{mode}_publications.csv').open() as f:
        rows = list(csv.DictReader(f))
    publications = {}
    prev_poses = []
    for r in rows:
        k = int(r['newest'])
        if int(r['index']) == k:
            publications[int(r['event'])] = r
        else:
            assert int(r['index']) == k-1
            prev_poses.append(pose(r))
    G = prod[mode + '_graph_body_T']
    O = prod[mode + '_chained_body_T']
    t = prod[mode + '_timestamp_ns']
    prior = np.asarray(prev_poses)
    assert len(publications) == len(G) == 4282 and len(prior) == len(G)-1
    gp, op, pp = points(G), points(O), points(prior)
    # Exact vector identity in the ORIGINAL receiver coordinate frame.
    local_step = gp[1:] - pp
    revision = pp - gp[:-1]
    chain_step = np.diff(op, axis=0)
    rotation_transfer = O[:-1, :3, :3] @ np.swapaxes(prior[:, :3, :3], 1, 2)
    rotate_step = np.einsum('nij,nj->ni', rotation_transfer, local_step)
    rotation_term = np.cumsum(chain_step-local_step, axis=0)
    revision_term = -np.cumsum(revision, axis=0)
    separation = op[1:] - gp[1:]
    checks = {
        'absolute_increment_identity_max_m': float(np.max(np.abs(np.diff(gp, axis=0)-local_step-revision))),
        'chain_rotated_increment_identity_max_m': float(np.max(np.abs(chain_step-rotate_step))),
        'accumulated_separation_identity_max_m': float(np.max(np.abs(separation-rotation_term-revision_term))),
        'saved_revision_parity_max_m': float(np.max(np.abs(revision-prod[mode+'_previous_state_correction']))),
    }
    assert max(checks.values()) < 1e-8
    mismatch = np.rad2deg(Rotation.from_matrix(rotation_transfer).magnitude())
    decomposition = {
        'checks': checks,
        'previous_point_revision_m': stats(norm(revision)),
        'local_increment_rotation_frame_difference_deg': stats(mismatch),
        'final_chain_minus_graph_xyz_m': separation[-1].tolist(),
        'final_rotation_accumulation_xyz_m': rotation_term[-1].tolist(),
        'final_omitted_revision_accumulation_xyz_m': revision_term[-1].tolist(),
        'chain_graph_separation_m': stats(norm(separation)),
        'final_chain_graph_separation_m': float(norm(separation[-1])),
        'warning': 'Vector identity, not an ablation. Terms can cancel; their norms are not causal percentages. Rotation differences are between products, not reference errors.',
    }

    origin = int(t[0])
    pending = []
    latest = {s: None for s in SOURCES}
    newest = -1
    age, right = [], []
    unseen = []
    total_beliefs = 0
    startup_excluded = 0
    publication_event_checks = 0
    right_clock_offsets_ns = []
    with pin(P / f'full_{mode}_receiver/events.csv').open() as f:
        for event, r in enumerate(csv.reader(f), 1):
            if r[0] == 'B':
                total_beliefs += 1
                start, end = int(r[8]), int(r[9])
                if start < origin:
                    startup_excluded += 1
                else:
                    required = ceil_support(end, origin)
                    src = r[2]
                    assert src in SOURCES
                    if required <= newest:
                        latest[src] = max(latest[src] or end, end)
                    else:
                        heapq.heappush(pending, (required, end, src))
            if event not in publications:
                continue
            pr = publications[event]
            newest = int(pr['newest'])
            assert r[0] == 'I'
            assert int(r[1]) == int(pr['availability_ns'])
            assert int(pr['physical_ns']) == origin + newest*PERIOD
            # Independent supports use a causal right-hand IMU clock sample;
            # its physical stamp need not equal the uniform support stamp.
            offset = int(r[2])-int(pr['physical_ns'])
            assert 0 <= offset < PERIOD
            right_clock_offsets_ns.append(offset)
            publication_event_checks += 1
            while pending and pending[0][0] <= newest:
                required, end, src = heapq.heappop(pending)
                latest[src] = max(latest[src] or end, end)
            age.append([(int(pr['physical_ns'])-latest[src])*1e-6 if latest[src] is not None else np.nan for src in SOURCES])
            right.append([ceil_support(latest[src], origin) if latest[src] is not None else -1 for src in SOURCES])
            unseen.append([latest[src] is None for src in SOURCES])
    assert publication_event_checks == len(t)
    age, right, unseen = map(np.asarray, (age, right, unseen))
    k = np.arange(len(t))
    direct = right == k[:, None]
    # Report only after both sources have valid post-origin evidence.
    both = ~np.any(unseen, axis=1)
    after10 = both & (k >= 100)
    timing = {
        'publication_event_identity_checks': publication_event_checks,
        'right_clock_sample_minus_support_ns': stats(right_clock_offsets_ns),
        'received_belief_versions': total_beliefs,
        'before_origin_versions': startup_excluded,
        'publications_with_both_sources_seen': int(both.sum()),
        'initial_publications_missing_at_least_one_source': int((~both).sum()),
        'direct_source_factor_on_newest_after_both_seen_count': int(np.any(direct[both], axis=1).sum()),
        'neither_source_directly_touches_newest_after_both_seen_percent': float(100*np.mean(~np.any(direct[both], axis=1))),
        'sources': {},
        'interpretation': 'Latest ADMISSIBLE received physical endpoints before publication in exact journal order. No direct factor means newest support depends on motion prior and propagated past evidence, not that sources are disconnected. Not wall-clock solver latency.',
    }
    for j, src in enumerate(SOURCES):
        timing['sources'][src] = {
            'endpoint_age_ms_after_both_seen': stats(age[both, j]),
            'endpoint_age_ms_after10s': stats(age[after10, j]),
            'directly_touches_newest_after_both_seen_percent': float(100*np.mean(direct[both, j])),
        }
    results['modes'][mode] = {'timing': timing, 'chain_decomposition': decomposition}
    for name, v in dict(timestamp_ns=t, frontier_age_ms=age, latest_endpoint_right=right,
                        chain_graph_separation=separation, omitted_revision_term=revision_term,
                        rotation_term=rotation_term, local_frame_angle_deg=mismatch).items():
        arrays[mode+'_'+name] = v

for name in ('full', 'common', 'unchanged_native_full', 'unchanged_native_common', 'common_coverage'):
    results['saved_CPV23_'+name] = previous_results[name]
for path in (Path(__file__), H/'PROTOCOL.txt', E/'receiver_finalization_cpk19_20261021/Receiver.h',
             E/'receiver_endpoint_reuse_e2b_20261013/SourceWnoaModel.h',
             E/'leica2_velodyne_belief_audit_cpv5_20261024/REPORT.txt',
             E/'leica2_paired_receiver_trace_cpv22a_20261027/REPORT.txt',
             E/'leica2_sensor_jitter_audit_cpv21_20261027/REPORT.txt'):
    pin(path)
for path, digest in pins.items():
    with open(path, 'rb') as f:
        assert hashlib.file_digest(f, 'sha256').hexdigest() == digest
results['checks'] = {'all_input_hashes_unchanged': True, 'pinned_files': len(pins),
                     'no_optimizer_or_container_run': True, 'ceil_bracket_boundary_tests': True}
with (H/'input_pins.json').open('x') as f:
    json.dump(pins, f, indent=2)
with (H/'results.json').open('x') as f:
    json.dump(results, f, indent=2, allow_nan=False)
assert not (H/'diagnostic_arrays.npz').exists()
np.savez_compressed(H/'diagnostic_arrays.npz', **arrays)
print(json.dumps(results['modes'], indent=2))
