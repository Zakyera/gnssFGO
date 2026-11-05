"""CP-V26: reconstruct GLIM method2 from immutable same-publication snapshots."""
import collections
import csv
import hashlib
import importlib.util
import json
import sys
from decimal import Decimal
from pathlib import Path

sys.dont_write_bytecode = True
import numpy as np
from scipy.spatial.transform import Rotation as R

H = Path(__file__).resolve().parent
E = H.parent
P = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_velodyne_full_cpv4_20261024/prepared')
F = Path('/home/yeranis/results/gnssfgo_leica2_incremental_full_cpv23_20261027')
H23 = E/'leica2_incremental_full_cpv23_20261027'
H10 = E/'first_published_comparison_cpv10_20261026'
spec = importlib.util.spec_from_file_location('cpv10', H10/'analyze.py')
c = importlib.util.module_from_spec(spec)
spec.loader.exec_module(c)
a = c.a
N = lambda x: np.linalg.norm(x, axis=-1)
PINS = {}


def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def pin(p):
    p = Path(p)
    PINS[str(p)] = sha(p)
    return p


def load(p):
    return json.loads(pin(p).read_text())


def matrix(p, q):
    t = np.eye(4)
    t[:3, 3] = p
    t[:3, :3] = R.from_quat(q).as_matrix()
    return t


def inverse(t):
    out = np.eye(4)
    out[:3, :3] = t[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ t[:3, 3]
    return out


def chain(current, previous):
    out = [current[0].copy()]
    for k in range(1, len(current)):
        out.append(out[-1] @ inverse(previous[k]) @ current[k])
    return np.asarray(out)


def synthetic_checks():
    g = np.repeat(np.eye(4)[None], 3, axis=0)
    g[:, 0, 3] = [10., 10.3, 10.6]
    previous = g.copy()
    previous[1:, 0, 3] = [10.2, 10.5]
    o = chain(g, previous)
    assert np.allclose(o[:, 0, 3], [10., 10.1, 10.2], atol=1e-12, rtol=0)
    rng = np.random.default_rng(26)
    g = np.array([matrix(rng.normal(size=3), R.random(random_state=rng).as_quat()) for _ in range(30)])
    previous = np.array([matrix(rng.normal(size=3), R.random(random_state=rng).as_quat()) for _ in range(30)])
    o = chain(g, previous)
    world = matrix([3., -2., 1.], R.from_rotvec([.3, -.2, .7]).as_quat())
    body = matrix([.3, -.1, -.2], R.from_rotvec([-.2, .1, .4]).as_quat())
    left = float(np.max(np.abs(chain(world @ g, world @ previous) - world @ o)))
    right = float(np.max(np.abs(chain(g @ body, previous @ body) - o @ body)))
    no_revision = float(np.max(np.abs(chain(g, np.concatenate([g[:1], g[:-1]])) - g)))
    assert max(left, right, no_revision) < 1e-10
    return dict(known_correction=True, global_frame_max=left, body_frame_max=right,
                no_revision_max=no_revision, existing_geometry=a.tests())


def glim(mode):
    source = P/f'full_{mode}_source'
    logs = list((source/'ros_logs').glob('**/glim_rosnode*-stdout.log'))
    assert len(logs) == 1
    selected = {}
    counts = collections.Counter()
    last_update = -1
    with pin(logs[0]).open() as f:
        for line in f:
            if 'GLIM_CBS_ACTIVE_POSE_ROW,' not in line:
                continue
            row = line.split('GLIM_CBS_ACTIVE_POSE_ROW,', 1)[1].split('\x1b')[0].strip().split(',')
            assert len(row) == 12 and row[0] == 'g'
            k, i = int(row[1]), int(row[3])
            assert k >= last_update
            last_update = k
            counts[k] += 1
            if i in (k-1, k):
                assert (k, i) not in selected
                selected[k, i] = row
    with pin(source/'glim_odometry.csv').open() as f:
        published = list(csv.DictReader(f))
    assert len(published) == 4280
    assert sorted(counts) == list(range(len(published)))
    assert [int(x['field.header.seq']) for x in published] == list(range(len(published)))
    timestamps = np.asarray([int(x['field.header.stamp']) for x in published], np.int64)
    assert np.all(np.diff(timestamps) > 0)
    current, previous, rounded_errors = [], [], []
    parity = 0.
    for k, pub in enumerate(published):
        row = selected[k, k]
        assert pub['field.header.frame_id'] == 'map' and pub['field.child_frame_id'] == 'cpt7_imu'
        values = np.asarray(row[5:], float)
        assert np.isfinite(values).all() and abs(N(values[3:])-1) < 1e-6
        pose = matrix(values[:3], values[3:])
        native = matrix([float(pub['field.pose.pose.position.'+x]) for x in 'xyz'],
                        [float(pub['field.pose.pose.orientation.'+x]) for x in 'xyzw'])
        parity = max(parity, float(np.max(np.abs(pose-native))))
        rounded_errors.append(abs(int(Decimal(row[4])*10**9)-int(timestamps[k])))
        assert Decimal(row[2]) >= Decimal(row[4])
        current.append(pose)
        if k == 0:
            previous.append(pose)
        else:
            before = selected[k, k-1]
            assert row[2] == before[2]
            rounded_errors.append(abs(int(Decimal(before[4])*10**9)-int(timestamps[k-1])))
            values = np.asarray(before[5:], float)
            assert np.isfinite(values).all() and abs(N(values[3:])-1) < 1e-6
            previous.append(matrix(values[:3], values[3:]))
    assert parity < 1e-12 and max(rounded_errors) < 1000
    g, prev = np.asarray(current), np.asarray(previous)
    o = chain(g, prev)
    # Independent normalized quaternion/vector recurrence.
    qr = R.from_matrix(g[0, :3, :3])
    point = g[0, :3, 3].copy()
    independent = [matrix(point, qr.as_quat())]
    for k in range(1, len(g)):
        rp = R.from_matrix(prev[k, :3, :3])
        rc = R.from_matrix(g[k, :3, :3])
        delta_position = rp.inv().apply(g[k, :3, 3]-prev[k, :3, 3])
        point = point + qr.apply(delta_position)
        qr = qr * rp.inv() * rc
        independent.append(matrix(point, qr.as_quat()))
    independent_error = float(np.max(np.abs(o-np.asarray(independent))))
    control = chain(g, np.concatenate([g[:1], g[:-1]]))
    telescoping = float(np.max(np.abs(control-g)))
    correction = np.asarray([g[k] @ inverse(o[k]) for k in range(len(g))])
    reconstruction = float(np.max(np.abs(correction @ o-g)))
    po, _ = a.prism(o[:, :3, 3], o[:, :3, :3])
    pg, _ = a.prism(g[:, :3, 3], g[:, :3, :3])
    pp, _ = a.prism(prev[:, :3, 3], prev[:, :3, :3])
    edge_error = float(np.max(np.abs(N(np.diff(po, axis=0))-N(pg[1:]-pp[1:]))))
    assert max(independent_error, telescoping, reconstruction, edge_error) < 1e-8
    checks = dict(publications=len(g), active_pose_rows=sum(counts.values()),
                  missing_pairs=0, duplicate_pairs=0, publication_pose_max=parity,
                  rounded_log_timestamp_max_error_ns=max(rounded_errors),
                  independent_chain_max=independent_error, telescoping_max=telescoping,
                  graph_reconstruction_max=reconstruction, prism_edge_length_max_m=edge_error,
                  final_graph_chain_prism_separation_m=float(N(po[-1]-pg[-1])),
                  final_graph_chain_rotation_separation_deg=float(np.rad2deg(R.from_matrix(o[-1,:3,:3].T@g[-1,:3,:3]).magnitude())))
    arrays = dict(timestamp_ns=timestamps, graph_body_T=g, previous_same_update_body_T=prev,
                  chained_body_T=o, graph_T_odometry=correction)
    return (timestamps, o[:, :3, 3], o[:, :3, :3]), arrays, checks


def main():
    targets = ['results.json', 'inputs.json', 'publication_products.npz',
               'off_glim_chained.tum', 'on_glim_chained.tum']
    assert not any((H/x).exists() for x in targets), 'Refuse to overwrite existing products'
    tests = synthetic_checks()
    old = load(H10/'outputs/velodyne_leica2.json')
    old23 = load(H23/'results.json')
    cp23check = load(H23/'reconstruction_checks.json')
    load(H23/'closeout.json')
    common = np.load(pin(H10/'outputs/velodyne_leica2.npz'))
    grid, valid = common['timestamp_ns'], common['common_valid']
    assert int(valid.sum()) == 3850
    protected = load(P/'protected_manifest.json')
    for file in (
        Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/local_revision_covariance_cpi1_20261019/cbs_bridge.cpp'),
        Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/fixed_lag_current_chart_fl4q_20261016/VioBackend.cpp')):
        pin(file)
        assert PINS[str(file)] == protected[str(file)]
    ref = a.g.Reference(pin(a.GT))
    reference = np.asarray(ref.times, np.int64), np.asarray(ref.poses)[:, :3, 3], np.asarray(ref.poses)[:, :3, :3]
    full, common_scores, reconstruction, checks, arrays, paths = {}, {}, {}, {}, {}, {}
    curves = {}
    for mode in ('off', 'on'):
        source = P/f'full_{mode}_source'
        settings = load(source/'effective_mode_check.json')
        assert settings['passed'] and '--no_incremental_pose=false' in json.dumps(settings)
        assert settings['glim']['glim_cbs_export_endpoint_audit_enable'] is True
        cg, aa, reconstruction[mode] = glim(mode)
        arrays.update({mode+'_'+k:v for k,v in aa.items()})
        curves[mode+'_glim_method2'] = cg
        curves[mode+'_glim_method1'] = a.load_curve(pin(source/'glim_odometry.csv'))
        curves[mode+'_kimera_method2'] = a.load_curve(pin(source/'kimera_odometry.csv'))
        fpath = pin(F/mode/'chained_trajectory.tum')
        assert PINS[str(fpath)] == cp23check['cases'][mode]['chained_sha256']
        curves[mode+'_fusion_method2'] = a.load_curve(fpath)
        paths[mode+'_fusion_method2'] = str(fpath)
        paths[mode+'_kimera_method2'] = str(source/'kimera_odometry.csv')
        paths[mode+'_glim_method2'] = str(H/f'{mode}_glim_chained.tum')
    for key, curve in curves.items():
        t, p, r = curve
        gp, gr, good = a.sample(reference, t, 50_000_000)
        assert good.all()
        ep, er = a.prism(p, r)
        gp, gr = a.prism(gp, gr)
        ate, _, delta = c.ate(ep, gp)
        full[key] = dict(ATE_RMSE_m=ate, path_m=float(N(np.diff(ep,axis=0)).sum()),
                         reference_path_m=float(N(np.diff(gp,axis=0)).sum()),
                         rows=len(t), reference_supported_rows=int(good.sum()),
                         first_s=float((t[0]-a.START)*1e-9), last_s=float((t[-1]-a.START)*1e-9),
                         span_s=float((t[-1]-t[0])*1e-9),
                         tail_gap_s=float(428.75-(t[-1]-a.START)*1e-9),
                         gap_chords_over_500ms=int(np.sum(np.diff(t)>500_000_000)),
                         evo_alignment_difference_m=delta)
        p, r, good = a.sample(curve, grid, 500_000_000)
        assert good[valid].all()
        ep, er = a.prism(p, r)
        common_scores[key], _ = c.metrics(ep, er, common['reference_position'], common['reference_rotation'], valid, grid)
        mode, role = key.split('_', 1)
        historical = {'glim_method1':'glim_initial', 'kimera_method2':'kimera'}.get(role)
        if historical:
            previous_full = old['full_saved'][mode+'_'+historical]
            previous_common = old['common'][mode+'_'+historical]
        elif role == 'fusion_method2':
            previous_full = old23['full'][mode+'_chained']
            previous_common = old23['common'][mode+'_chained']
        else:
            continue
        checks[key] = dict(full_ATE_delta_m=abs(ate-previous_full['ATE_RMSE_m']),
                          common_ATE_delta_m=abs(common_scores[key]['ATE_RMSE_m']-previous_common['ATE_RMSE_m']))
        assert max(checks[key].values()) < 1e-8
    for module in (c, c.b, a, a.g):
        pin(module.__file__)
    pin(__file__)
    pin(H/'PROTOCOL.txt')
    for path, digest in a.HASHES.items():
        assert sha(path) == digest
        PINS[path] = digest
    historical_pins = old['input_sha256']
    verified_historical = 0
    for path, digest in PINS.items():
        if path in historical_pins:
            assert digest == historical_pins[path], path
            verified_historical += 1
        assert sha(path) == digest, path
    # Write only new analysis products after all reconstruction/scoring checks pass.
    for mode in ('off', 'on'):
        t, p, r = curves[mode+'_glim_method2']
        with (H/f'{mode}_glim_chained.tum').open('x') as f:
            for stamp, pos, quat in zip(t, p, R.from_matrix(r).as_quat()):
                f.write(f'{stamp//10**9}.{stamp%10**9:09d} '+' '.join(format(x,'.17g') for x in [*pos,*quat])+'\n')
    np.savez_compressed(H/'publication_products.npz', **arrays)
    result = dict(task='CP-V26', scope='LEICA-2 Velodyne OFF/ON, original Qc1.0; saved only',
                  primary='method2 for all three; native cadence retained', full_saved=full,
                  common=common_scores, common_coverage=old['common_coverage'],
                  reconstruction=reconstruction, historical_score_checks=checks,
                  historical_pins_verified=verified_historical, synthetic_checks=tests,
                  trajectories=paths, input_hashes_unchanged=True,
                  limitations=['Same publication rule does not equalize cadence/evidence age/latency.',
                               'IE-TC shares CPT7; not independent prism GT.',
                               'Full native polyline lengths include reported sparse-sampling gap chords.',
                               'No output covariance/velocity assigned; no production promotion.',
                               'No other sequence converted in this bounded task.'])
    a.save(H/'results.json', result)
    a.save(H/'inputs.json', PINS)
    print(json.dumps({key: {'ATE_m':v['ATE_RMSE_m'], 'path_m':v['path_length_m'],
                            'RPE1':v['horizons']['1']} for key,v in common_scores.items()}, indent=2))


if __name__ == '__main__':
    main()
