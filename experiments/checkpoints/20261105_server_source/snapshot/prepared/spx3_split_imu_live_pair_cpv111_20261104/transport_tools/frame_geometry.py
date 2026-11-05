"""Read retained source logs; never run or modify an estimator."""
import csv
import hashlib
import json
import re
import sys
from pathlib import Path

import numpy as np
from scipy.linalg import expm
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
WORK = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
CASES = {
    'eig2': ('gnssfgo_eig2_live60_off_on_fl4x_20261017', 'SPX-3', '2024-11-02-17-43-10'),
    'spx3': ('gnssfgo_spx3_live60_off_on_fl4w_20261017', 'SPX-3', '2024-11-02-17-43-10'),
}


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for data in iter(lambda: f.read(1024 * 1024), b''):
            h.update(data)
    return h.hexdigest()


def hat(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def jac(phi):
    w = hat(phi)
    th = np.linalg.norm(phi)
    if th < 1e-5:
        a = .5 - th**2 / 24 + th**4 / 720
        b = 1 / 6 - th**2 / 120 + th**4 / 5040
    else:
        a = (1 - np.cos(th)) / th**2
        b = (th - np.sin(th)) / th**3
    return np.eye(3) + a * w + b * w @ w


def exp(mu):
    mu = np.asarray(mu)
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(mu[:3]).as_matrix()
    T[:3, 3] = jac(mu[:3]) @ mu[3:]
    return T


def log(T):
    phi = Rotation.from_matrix(T[:3, :3]).as_rotvec()
    return np.r_[phi, np.linalg.solve(jac(phi), T[:3, 3])]


def inv(T):
    result = np.eye(4)
    result[:3, :3] = T[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ T[:3, 3]
    return result


def pose(pq):
    # Logged native pose: tx,ty,tz,qw,qx,qy,qz.
    pq = np.asarray(pq, float)
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(pq[[4, 5, 6, 3]]).as_matrix()
    T[:3, 3] = pq[:3]
    return T


def matrix(token):
    return np.asarray([[float(x) for x in row.split(':')]
                       for row in token.split(';')])


def rows(path, tags):
    with path.open() as f:
        for line in f:
            for tag in tags:
                marker = tag + ','
                if marker in line:
                    yield tag, re.sub(r'\x1b\[[0-9;]*m', '',
                                     line.split(marker, 1)[1]).strip().split(',')
                    break


def snapshots(path):
    result = {}
    max_duplicate = 0.
    for _, f in rows(path, ['KIMERA_CBS_REVISION_EDGE_ROW']):
        cur, a, b = int(f[0]), int(f[2]), int(f[3])
        for key, values in [(a, f[8:15]), (b, f[15:22])]:
            T = pose(values)
            if (cur, key) in result:
                max_duplicate = max(max_duplicate, float(np.max(np.abs(T-result[cur, key]))))
            result[cur, key] = T
    assert max_duplicate < 1e-10, max_duplicate
    return result


def parse(root):
    source = root/'live60_on_source'
    main = source/'estimators.log'
    stdout = next((source/'ros_logs').glob('*/kimera_vio_ros-kimera_vio_ros_node-4-stdout.log'))
    on = snapshots(main)
    off = snapshots(root/'live60_off_source/estimators.log')
    align = []
    frozen = {}
    for tag, f in rows(main, ['KIMERA_CBS_MEAN_ALIGNMENT_ROW', 'FL4M_FROZEN_FACTOR_ROW']):
        key = tuple(map(int, f[:5]))
        if tag == 'FL4M_FROZEN_FACTOR_ROW':
            assert key not in frozen
            frozen[key] = (exp(matrix(f[5]).ravel()), matrix(f[6]))
        else:
            align.append(dict(key=key, stamps=list(map(int, f[5:10])),
                              raw=exp(matrix(f[10]).ravel()),
                              aligned=exp(matrix(f[11]).ravel()), cov=matrix(f[12])))
    pre, sample, factor = {}, {}, {}
    tags = ['CBS_ODOM_PREINJECTION_RESIDUAL_ROW', 'CBS_COVARIANCE_EXCHANGE_MATRIX_ROW',
            'CBS_ODOM_FACTOR_MATRIX_ROW']
    for tag, f in rows(stdout, tags):
        if f[0] != 'G2K':
            continue
        if tag == tags[0]:
            key = (int(f[3][1:]), int(f[4][1:]))
            assert key not in pre
            pre[key] = np.asarray(f[7:], float)
        elif tag == tags[2]:
            key = (int(f[1][1:]), int(f[2][1:]))
            assert key not in factor
            factor[key] = matrix(f[4])
        elif f[8:10] == ['receiver_preinjection_state', 'relative_pose_matrix']:
            key = (int(f[5][1:]), int(f[6][1:]))
            assert key not in sample
            sample[key] = matrix(f[12])
    maxima = dict(pre_norm_abs_error=0., pre_whiten_relative_error=0.,
                  explicit_pre_matrix_error=0., frozen_mean_error=0., covariance_error=0.)
    missing = []
    for x in align:
        cur, sf, st, a, b = x['key']
        sa, sb, ra, rb, limit = x['stamps']
        assert sa < sb <= limit and ra < rb <= limit
        frozen_mean, frozen_cov = frozen[x['key']]
        maxima['frozen_mean_error'] = max(maxima['frozen_mean_error'], float(np.max(np.abs(frozen_mean-x['aligned']))))
        maxima['covariance_error'] = max(maxima['covariance_error'], float(np.max(np.abs(frozen_cov-x['cov']))), float(np.max(np.abs(factor[a, b]-x['cov']))))
        if (cur-1, a) not in on or (cur-1, b) not in on:
            missing.append(x['key'])
            continue
        x['pre'] = inv(on[cur-1, a]) @ on[cur-1, b]
        x['post'] = inv(on[cur, a]) @ on[cur, b]
        x['off_pre'] = inv(off[cur-1, a]) @ off[cur-1, b]
        residual = log(inv(x['aligned']) @ x['pre'])
        expected = pre[a, b]
        values = [np.linalg.norm(residual), np.linalg.norm(residual[:3]), np.linalg.norm(residual[3:])]
        maxima['pre_norm_abs_error'] = max(maxima['pre_norm_abs_error'], float(np.max(np.abs(values-expected[:3]))))
        white = np.sqrt(residual @ np.linalg.solve(x['cov'], residual))
        maxima['pre_whiten_relative_error'] = max(maxima['pre_whiten_relative_error'], abs(white-expected[6])/(1+expected[6]))
        if (a, b) in sample:
            maxima['explicit_pre_matrix_error'] = max(maxima['explicit_pre_matrix_error'], float(np.max(np.abs(x['pre']-sample[a, b]))))
    audit = dict(aligned=len(align), frozen=len(frozen), recorded_pre=len(pre), explicit_pre=len(sample),
                 missing=missing, maxima=maxima)
    print(json.dumps({'snapshot_validation': audit}, indent=2), flush=True)
    assert not missing
    assert len(align) == len(pre) == len(frozen) == len(factor)
    assert max(maxima.values()) < 1e-8, maxima
    return align, audit, [main, stdout, root/'live60_off_source/estimators.log']


def self_test():
    rng = np.random.default_rng(1729)
    for scale in [0., 1e-8, 1e-4, .2, .7]:
        for _ in range(10):
            mu = rng.normal(size=6)*scale
            A = np.zeros((4, 4)); A[:3, :3] = hat(mu[:3]); A[:3, 3] = mu[3:]
            assert np.max(np.abs(exp(mu)-expm(A))) < 1e-12
            assert np.max(np.abs(log(exp(mu))-mu)) < 1e-10
    return True


class Reference:
    def __init__(self, path):
        from rosbags.highlevel import AnyReader
        self.times, self.poses = [], []
        self.max_bracket_ns = 0
        with AnyReader([path]) as reader:
            connections = [c for c in reader.connections
                           if c.topic == '/boxi/inertial_explorer/tc/odometry']
            assert connections
            for connection, _, data in reader.messages(connections=connections):
                m = reader.deserialize(data, connection.msgtype)
                assert m.child_frame_id == 'cpt7_imu'
                self.times.append(m.header.stamp.sec*10**9+m.header.stamp.nanosec)
                p, q = m.pose.pose.position, m.pose.pose.orientation
                self.poses.append(pose([p.x, p.y, p.z, q.w, q.x, q.y, q.z]))
        self.times = np.asarray(self.times, dtype=np.int64)
        assert np.all(np.diff(self.times) > 0)
        self.cache = {}

    def at(self, t):
        if t in self.cache:
            return self.cache[t]
        i = int(np.searchsorted(self.times, t))
        assert 0 < i < len(self.times), ('reference out of coverage', t)
        a, b = self.poses[i-1], self.poses[i]
        gap = int(self.times[i]-self.times[i-1])
        self.max_bracket_ns = max(self.max_bracket_ns, gap)
        assert gap <= 50_000_000
        alpha = (t-int(self.times[i-1]))/gap
        T = np.eye(4)
        T[:3, 3] = (1-alpha)*a[:3, 3]+alpha*b[:3, 3]
        T[:3, :3] = a[:3, :3] @ Rotation.from_rotvec(
            alpha*Rotation.from_matrix(a[:3, :3].T@b[:3, :3]).as_rotvec()).as_matrix()
        self.cache[t] = T
        return T


def stats(values):
    a = np.asarray(values, float)
    assert len(a) and np.isfinite(a).all()
    return dict(n=len(a), mean=float(a.mean()), rms=float(np.sqrt(np.mean(a*a))),
                median=float(np.median(a)), p95=float(np.percentile(a, 95)),
                max=float(a.max()))


def error(Z, target):
    e = log(inv(Z)@target)
    return dict(translation_m=float(np.linalg.norm(Z[:3, 3]-target[:3, 3])),
                rotation_deg=float(np.linalg.norm(e[:3])*180/np.pi),
                rho_m=float(np.linalg.norm(e[3:])))


def summary(records):
    names = records[0]['errors'].keys()
    result = {name: {metric: stats([r['errors'][name][metric] for r in records])
                     for metric in records[0]['errors'][name]} for name in names}
    def win(first, second, metric):
        return float(np.mean([r['errors'][first][metric] < r['errors'][second][metric]
                              for r in records]))
    fractions = {}
    for a, b in [('aligned', 'pre'), ('post', 'pre'), ('aligned', 'oracle'),
                 ('aligned', 'raw_unshifted'), ('aligned', 'off_pre')]:
        fractions[a+'_better_than_'+b] = {m: win(a, b, m) for m in ['translation_m', 'rotation_deg']}
    return dict(count=len(records), errors=result, fractions=fractions)


def analyze(name):
    root = WORK/'runs'/CASES[name][0]/'prepared'
    out = HERE/(name+'_results.json')
    assert not out.exists(), 'No overwrite of completed audit'
    align, audit, inputs = parse(root)
    gt_path = WORK/'src/datasets/GrandTour'/CASES[name][1]/(CASES[name][2]+'_cpt7_ie_tc.bag')
    ref = Reference(gt_path)
    native_path = root/'live60_on_source/kimera_odometry.csv'
    native = list(csv.DictReader(native_path.open()))
    times = {int(r['field.header.seq']): int(r['field.header.stamp']) for r in native}
    off_native_path = root/'live60_off_source/kimera_odometry.csv'
    off_times = {int(r['field.header.seq']): int(r['field.header.stamp'])
                 for r in csv.DictReader(off_native_path.open())}
    assert times == off_times
    acceptance_path = root/'live60_on_source/acceptance.csv'
    acceptance = np.loadtxt(acceptance_path, delimiter=',')
    records = []
    for x in align:
        cur, sf, st, a, b = x['key']
        sa, sb, ra, rb, limit = x['stamps']
        assert times[a] == ra and times[b] == rb and times[cur] == limit
        ga, gb, ka, kb = [ref.at(t) for t in [sa, sb, ra, rb]]
        sender_gt = inv(ga)@gb
        receiver_gt = inv(ka)@kb
        L, R = inv(ka)@ga, inv(gb)@kb
        assert np.max(np.abs(L@sender_gt@R-receiver_gt)) < 1e-12
        oracle = L@x['raw']@R
        transforms = {k: x[k] for k in ['raw', 'aligned', 'pre', 'post', 'off_pre']}
        transforms.update(oracle=oracle, raw_unshifted=x['raw'])
        errors = {k:error(T, sender_gt if k == 'raw' else receiver_gt)
                  for k,T in transforms.items()}
        errors['bridge_difference'] = error(x['aligned'], oracle)
        errors['correction'] = error(x['pre'], x['post'])
        errors['alignment_change'] = error(x['raw'], x['aligned'])
        e = log(inv(x['aligned'])@receiver_gt)
        fact = acceptance[cur]
        records.append(dict(update=cur,sender_from=sf,sender_to=st,receiver_from=a,receiver_to=b,
             sender_from_ns=sa,sender_to_ns=sb,receiver_from_ns=ra,receiver_to_ns=rb,
             relative_end_s=(rb-times[0])*1e-9, update_s=(limit-times[0])*1e-9,
             from_shift_ms=(ra-sa)*1e-6,to_shift_ms=(rb-sb)*1e-6,
             sender_duration_s=(sb-sa)*1e-9,receiver_duration_s=(rb-ra)*1e-9,
             fallback=bool(fact[4]),fallback_iterations=int(fact[5]),errors=errors,
             measurement_whitened_reference_error=float(np.sqrt(e@np.linalg.solve(x['cov'],e))),
             transforms={k:T.tolist() for k,T in transforms.items()},
             sender_gt=sender_gt.tolist(),receiver_gt=receiver_gt.tolist()))
    bins = {}
    for start, end in [(0,20),(20,40),(40,60)]:
        subset = [r for r in records if start <= r['relative_end_s'] < end]
        bins[f'{start}-{end}s'] = summary(subset)
    inputs += [gt_path,native_path,off_native_path,acceptance_path]
    report = dict(sequence=name, self_tests=True, snapshot_validation=audit,
       reference=dict(path=str(gt_path),topic='/boxi/inertial_explorer/tc/odometry',
          body='cpt7_imu',caveat='IE-TC shares CPT7 IMU; not independent prism GT',
          coverage_fraction=1.,max_used_bracket_ns=ref.max_bracket_ns,
          convention='linear translation + SO(3) shortest-path interpolation; no global alignment'),
       interval=dict(count=len(records),first_receiver_from_ns=min(r['receiver_from_ns'] for r in records),
          last_receiver_to_ns=max(r['receiver_to_ns'] for r in records),
          first_kimera_ns=times[0],
          receiver_duration_s=stats([r['receiver_duration_s'] for r in records]),
          from_shift_abs_ms=stats([abs(r['from_shift_ms']) for r in records]),
          to_shift_abs_ms=stats([abs(r['to_shift_ms']) for r in records])),
       overall=summary(records),bins=bins,
       input_hashes={str(p):sha(p) for p in inputs},records=records)
    with out.open('x') as f:
        json.dump(report,f,indent=2,allow_nan=False)
    compact = {k:{m:round(v[m]['rms'],9) for m in ['translation_m','rotation_deg']}
               for k,v in report['overall']['errors'].items()}
    print(json.dumps({'sequence':name,'reference':report['reference'],
        'interval':report['interval'],'overall_rms':compact,'fractions':report['overall']['fractions']},indent=2))


if __name__ == '__main__':
    self_test()
    analyze(sys.argv[1])
