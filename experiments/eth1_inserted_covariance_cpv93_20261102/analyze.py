"""Saved-data CP-V93: actual persistent external-factor covariance, no runs."""
import hashlib
import importlib.util
import json
import os
import re
import sys
from collections import defaultdict
from decimal import Decimal
from pathlib import Path

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')
sys.dont_write_bytecode = True
sys.path.append('/home/yeranis/.local/share/pipx/venvs/rosbags/lib/python3.12/site-packages')
import numpy as np
from rosbags.highlevel import AnyReader

H = Path(__file__).resolve().parent
E = H.parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
NAMES = {'OFF': 'eth1_alpha_bias_rw_full_off_cpv89_20261102',
         'ON': 'eth1_split_imu_persistent_on_cpv91_20261102_attempt1'}
S = {m: W/'runs'/n/'prepared'/(m.lower()+'_source') for m, n in NAMES.items()}
P = S['ON'].parent
PINS = {}
ANSI = re.compile(r'\x1b\[[0-9;]*m')


def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def pin(p):
    p = Path(p)
    PINS[str(p)] = sha(p)
    return p


def ns(t):
    return int((Decimal(str(t))*Decimal(10**9)).to_integral_value())


def decode(line, tag):
    return ANSI.sub('', line.split(tag+',', 1)[1]).strip().split(',')


def mat(t):
    return np.array([list(map(float, r.split(':'))) for r in t.split(';')])


def keypair(t):
    return tuple(int(re.search(r'\d+$', x).group()) for x in t.split('->'))


def stats(x):
    x = np.asarray(x, float)
    assert len(x) and np.isfinite(x).all()
    return dict(n=len(x), minimum=float(x.min()), median=float(np.median(x)),
                mean=float(x.mean()), p05=float(np.percentile(x, 5)),
                p95=float(np.percentile(x, 95)), maximum=float(x.max()))


def traces(C):
    return np.stack([np.trace(C[:, :3, :3], axis1=1, axis2=2),
                     np.trace(C[:, 3:, 3:], axis1=1, axis2=2)], axis=1)


def describe(C):
    sym = (C+C.transpose(0, 2, 1))/2
    ev = np.linalg.eigvalsh(sym)
    assert np.isfinite(C).all() and np.all(ev > 0)
    tr = traces(C)
    rt, tt = np.linalg.eigvalsh(sym[:, :3, :3]), np.linalg.eigvalsh(sym[:, 3:, 3:])
    D = np.sqrt(np.diagonal(C, axis1=1, axis2=2))
    corr = C / (D[:, :, None]*D[:, None, :])
    return dict(n=len(C), symmetry_max=float(np.max(abs(C-C.transpose(0, 2, 1)))),
                minimum_full_eigenvalue_raw_rad_m_coordinates=float(ev.min()),
                rotation_trace_rad2=stats(tr[:, 0]), translation_trace_m2=stats(tr[:, 1]),
                rotation_sqrt_trace_deg=stats(np.sqrt(tr[:, 0])*180/np.pi),
                translation_sqrt_trace_mm=stats(np.sqrt(tr[:, 1])*1000),
                rotation_lambda_min_rad2=stats(rt[:, 0]), rotation_lambda_max_rad2=stats(rt[:, -1]),
                translation_lambda_min_m2=stats(tt[:, 0]), translation_lambda_max_m2=stats(tt[:, -1]),
                rotation_condition=stats(rt[:, -1]/rt[:, 0]), translation_condition=stats(tt[:, -1]/tt[:, 0]),
                max_abs_rotation_translation_correlation=stats(abs(corr[:, :3, 3:]).max(axis=(1, 2))))


def write(name, obj):
    with (H/name).open('x') as f:
        json.dump(obj, f, indent=2, allow_nan=False)


def main():
    assert not (H/'analysis.json').exists() and not (H/'matrices.npz').exists()
    pin(__file__); pin(H/'PROTOCOL.txt')
    spec = importlib.util.spec_from_file_location('cp93_geo', pin(E/'fixed_lag_eig2_g2k_reference_fl4y_20261017/analyze.py'))
    g = importlib.util.module_from_spec(spec); spec.loader.exec_module(g)
    frame = json.loads(pin(E/NAMES['ON']/'frame_validation.json').read_text())
    assert frame['passed']
    X = np.array(frame['T_alphasense_cpt7']); Y = g.inv(X)
    A = np.zeros((6, 6)); A[:3, :3] = A[3:, 3:] = X[:3, :3]
    A[3:, :3] = g.hat(X[:3, 3]) @ X[:3, :3]
    epoch = json.loads(pin(P/'input_audit.json').read_text())['epoch_ns']
    manifest = json.loads(pin(P/'protected_manifest.json').read_text())
    code = [W/'src/cbs/src/bpsam/bpsam.cpp',
            W/'runs/retention_readiness_cpv73_20261031/VioBackend.cpp',
            W/'runs/glim_persistent_compat_cpv82_20261101/release_4/inputs/cbs_bridge.cpp',
            W/'src/Kimera-VIO-ROS/src/CbsRelativeFrameConversion.cpp']
    for p in code:
        assert sha(pin(p)) == manifest[str(p)], p
    effective = json.loads(pin(S['ON']/'effective_mode_check.json').read_text())
    assert effective['passed']
    coverage = json.loads(pin(S['ON']/'source_coverage.json').read_text())
    assert coverage['passed'] and coverage['external_injections'] == {
        'G2K_queued':1473, 'G2K_persistent':1473, 'K2G_persistent':1473}
    logs = {d: next(p for p in (S['ON']/'ros_logs').glob(glob) if p.parent.name != 'latest')
            for d, glob in [('G2K', '*/kimera_vio_ros-*-stdout.log'), ('K2G', '*/glim_rosnode-1-stdout.log')]}
    factor = {d:{} for d in logs}; align = {d:{} for d in logs}
    receipt = {d:set() for d in logs}; exchange = defaultdict(dict); frozen = {}; transport = {}
    def unique(dst, k, v):
        assert k not in dst, ('duplicate key', k)
        dst[k] = v
    for direction, path in logs.items():
        for line in pin(path).open():
            if 'CBS_ODOM_FACTOR_MATRIX_ROW,' in line:
                f = decode(line, 'CBS_ODOM_FACTOR_MATRIX_ROW')
                assert f[0] == direction and f[3] == 'persistent_odom_applied'
                unique(factor[direction], (int(f[1][1:]), int(f[2][1:])), mat(f[4]))
            if direction == 'G2K' and 'CBS_BELIEF_ODOM_ROW,' in line:
                f = decode(line, 'CBS_BELIEF_ODOM_ROW')
                if f[5] == 'sender_odom_queued':
                    k = (int(f[3][1:]), int(f[4][1:])); assert k not in receipt[direction]; receipt[direction].add(k)
            if 'GLIM_CBS_ODOM_INJECT_ROW,' in line:
                f = decode(line, 'GLIM_CBS_ODOM_INJECT_ROW'); assert f[-1] == 'injected_persistent'
                k = keypair(f[1]); assert k not in receipt[direction]; receipt[direction].add(k)
            if 'GLIM_CBS_SOURCE_TRANSPORT_ROW,' in line:
                f = decode(line, 'GLIM_CBS_SOURCE_TRANSPORT_ROW')
                unique(transport, tuple(map(int, f[1:3])), (mat(f[4]), mat(f[5])))
            if 'GLIM_CBS_MEAN_ALIGNMENT_ROW,' in line:
                f = decode(line, 'GLIM_CBS_MEAN_ALIGNMENT_ROW')
                k = tuple(map(int, f[2:4]))
                unique(align['K2G'], k, dict(source_ids=tuple(map(int, f[:2])),
                    source_ns=list(map(ns, f[4:6])), receiver_ns=list(map(ns, f[6:8])),
                    causal_ns=ns(f[8]), raw_mu=list(map(float, f[9].split(';'))),
                    aligned_mu=list(map(float, f[10].split(';'))), C=mat(f[11])))
            if 'CBS_COVARIANCE_EXCHANGE_MATRIX_ROW,' in line:
                f = decode(line, 'CBS_COVARIANCE_EXCHANGE_MATRIX_ROW')
                if f[8] == 'receiver_inserted_factor' and f[9] in ('covariance', 'information_omega'):
                    unique(exchange[(direction, int(f[5][1:]), int(f[6][1:]))], f[9], mat(f[12]))
    for line in pin(S['ON']/'estimators.log').open():
        if 'KIMERA_CBS_MEAN_ALIGNMENT_ROW,' in line:
            f = decode(line, 'KIMERA_CBS_MEAN_ALIGNMENT_ROW'); k = tuple(map(int, f[3:5]))
            unique(align['G2K'], k, dict(source_ids=tuple(map(int, f[1:3])),
                source_ns=list(map(int, f[5:7])), receiver_ns=list(map(int, f[7:9])), causal_ns=int(f[9]),
                raw_mu=list(map(float, f[10].split(';'))), aligned_mu=list(map(float, f[11].split(';'))), C=mat(f[12])))
        if 'FL4M_FROZEN_FACTOR_ROW,' in line:
            f = decode(line, 'FL4M_FROZEN_FACTOR_ROW')
            unique(frozen, tuple(map(int, f[3:5])), (np.array(f[5].split(';'), float), mat(f[6])))
    wire = {m:{d:defaultdict(list) for d in logs} for m in NAMES}
    bytime = {m:{d:defaultdict(list) for d in logs} for m in NAMES}
    for m in NAMES:
        topics = {'/kimera/cbs/odom_belief_in':'G2K', '/kimera/cbs/odom_belief_out':'K2G'}
        with AnyReader([pin(S[m]/'recordings/cbs.bag')]) as reader:
            for conn, t, raw in reader.messages(connections=[c for c in reader.connections if c.topic in topics]):
                msg = reader.deserialize(raw, conn.msgtype); d = topics[conn.topic]
                for b in msg.beliefs:
                    assert b.source_agent == ord('g' if d == 'G2K' else 'k') and b.revision == 0
                    C = np.array(b.covariance, copy=True).reshape(6, 6)
                    M = g.exp(np.array(b.relative_mu, copy=True))
                    if d == 'G2K': C = A@C@A.T; M = X@M@Y
                    r = dict(C=C, M=M, times=(ns(b.from_stamp_sec),ns(b.to_stamp_sec)), bag_ns=t)
                    wire[m][d][(int(b.from_pose_index),int(b.to_pose_index))].append(r)
                    bytime[m][d][r['times']].append(r)
    result = {}; arrays = {}
    for d in logs:
        keys = sorted(factor[d]); assert len(keys) == 1473
        assert set(keys) == set(align[d]) == receipt[d]
        if d == 'G2K': assert set(keys) == set(frozen)
        CC = []; OO = []; matched = []; records = []; diffs = []; meanerrs = []; tdiffs = []
        ambiguous_on = 0; ambiguous_off = 0; unmatched_off = 0; info_errors = []
        for index, k in enumerate(keys):
            r = align[d][k]; C = factor[d][k]; assert np.array_equal(C, r['C'])
            if d == 'G2K':
                assert np.array_equal(frozen[k][1], C) and np.array_equal(frozen[k][0], r['aligned_mu'])
            else:
                assert all(np.array_equal(v, C) for v in transport[r['source_ids']])
            options = wire['ON'][d][r['source_ids']]
            matches = [v for v in options if np.max(abs(v['C']-C))/np.max(abs(C)) < 1e-12
                       and np.max(abs(v['M']-g.exp(np.array(r['raw_mu'])))) < 1e-9
                       and max(abs(a-b) for a,b in zip(v['times'],r['source_ns'])) <= 512]
            assert matches, (d,k)
            # Identical duplicates are still reported; don't silently treat versions as unique.
            ambiguous_on += len(matches) > 1
            v = matches[0]; diffs.append(float(np.max(abs(v['C']-C))/np.max(abs(C))))
            meanerrs.append(float(np.max(abs(v['M']-g.exp(np.array(r['raw_mu']))))))
            tdiffs.append(max(abs(a-b) for a,b in zip(v['times'],r['source_ns'])))
            assert max(r['source_ns'][1], r['receiver_ns'][1]) <= r['causal_ns']+512
            candidates = bytime['OFF'][d][v['times']]
            # Pair exact physical intervals. Exclude ambiguous OFF versions rather than choose by outcome.
            if len(candidates) == 1:
                OO.append(candidates[0]['C']); matched.append(index)
            elif not candidates: unmatched_off += 1
            else: ambiguous_off += 1
            CC.append(C)
            records.append(dict(receiver_ids=k, source_ids=r['source_ids'], source_ns=v['times'],
                receiver_ns=r['receiver_ns'], causal_ns=r['causal_ns'], on_wire_matches=len(matches),
                off_wire_matches=len(candidates)))
            ex = exchange.get((d,*k))
            if ex:
                assert np.array_equal(ex['covariance'],C)
                info_errors.append(float(np.max(abs(C@ex['information_omega']-np.eye(6)))))
        CC = np.array(CC); OO = np.array(OO); matched = np.array(matched)
        time = np.array([(r['receiver_ns'][1]-epoch)/1e9 for r in records])
        tr = traces(CC); ot = traces(OO); ratio = tr[matched]/ot
        bins = []
        for lo in range(0, 360, 30):
            ii = (time >= lo)&(time < lo+30); mi = (time[matched] >= lo)&(time[matched] < lo+30)
            if not ii.any(): continue
            bins.append(dict(start_s=lo,end_s=lo+30,n=int(ii.sum()),matched_n=int(mi.sum()),
                rotation_trace_median_rad2=float(np.median(tr[ii,0])),translation_trace_median_m2=float(np.median(tr[ii,1])),
                paired_ON_OFF_rotation_median=float(np.median(ratio[mi,0])) if mi.any() else None,
                paired_ON_OFF_translation_median=float(np.median(ratio[mi,1])) if mi.any() else None))
        result[d] = dict(receiver_body='Alpha IMU' if d=='G2K' else 'CPT7 IMU',
            applied=describe(CC), matched_OFF_export=describe(OO), matched_ON_inserted=describe(CC[matched]),
            paired_ON_OFF_rotation_trace_ratio=stats(ratio[:,0]), paired_ON_OFF_translation_trace_ratio=stats(ratio[:,1]),
            rotation_fraction_ratio_below_half=float(np.mean(ratio[:,0]<.5)),
            translation_fraction_ratio_below_half=float(np.mean(ratio[:,1]<.5)),
            rotation_fraction_ratio_below_tenth=float(np.mean(ratio[:,0]<.1)),
            translation_fraction_ratio_below_tenth=float(np.mean(ratio[:,1]<.1)),
            bins_30s=bins,coverage_receiver_end_s=[float(time.min()),float(time.max())],
            receiver_interval_seconds=stats([(r['receiver_ns'][1]-r['receiver_ns'][0])/1e9 for r in records]),
            source_interval_seconds=stats([(r['source_ns'][1]-r['source_ns'][0])/1e9 for r in records]),
            checks=dict(receipts=1473,full_matrix_exact_alignment_parity=True,
                wire_covariance_relative_error_max=max(diffs),wire_mean_error_max=max(meanerrs),
                source_timestamp_rounding_max_ns=max(tdiffs),ambiguous_ON_matches=ambiguous_on,
                matched_OFF=len(matched),ambiguous_OFF_excluded=ambiguous_off,unmatched_OFF=unmatched_off,
                capped_information_checks=len(info_errors),covariance_information_identity_error_max=max(info_errors) if info_errors else None),
            records=records)
        assert max(diffs)<1e-12 and (not info_errors or max(info_errors)<1e-8)
        arrays.update({d+'_C_ON':CC,d+'_C_OFF':OO,d+'_matched':matched,d+'_time_s':time})
        print(d, json.dumps({k:v for k,v in result[d].items() if k not in ('records','bins_30s')},indent=2),flush=True)
    terminal = {}
    for m,n in NAMES.items():
        data = json.loads(pin(E/n/'complete.json').read_text())
        for p,digest in data['output_sha256'].items(): assert sha(p)==digest,p
        terminal[m] = len(data['output_sha256'])
    assert all(sha(p)==v for p,v in PINS.items())
    np.savez_compressed(H/'matrices.npz', **arrays)
    write('analysis.json',dict(passed=True,epoch_ns=epoch,scope='actual persistent insertion covariance; not later native revisions',
        directions=result,T_alphasense_cpt7=X.tolist(),input_sha256=PINS,terminal_hash_counts=terminal,
        limitations=['OFF has no external insertions; matched exported intervals are controls in the receiver body.',
        'Time alignment changes means but leaves covariance unchanged; bridge uncertainty is not propagated.',
        'Trace/eigenvalues do not establish calibration, independence or causal feedback mechanism.',
        'Square-root trace is root-sum marginal variance, not a per-axis sigma or a 95% radius.',
        'No post-optimization covariance mutation audit; these are matrices at factor construction/insertion.',
        'Persistent mixed-prior feedback remains; no temporary-mode control or fresh source run.']))


if __name__ == '__main__':
    main()
