"""CP-V92: saved covariance telemetry only; no estimator/optimizer execution."""
import hashlib
import importlib.metadata
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
from scipy.linalg import expm
from scipy.stats import chi2, spearmanr
from rosbags.highlevel import AnyReader
from evo.core import metrics, trajectory, units

H = Path(__file__).resolve().parent
E = H.parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
NAMES = dict(OFF='eth1_alpha_bias_rw_full_off_cpv89_20261102',
             ON='eth1_split_imu_persistent_on_cpv91_20261102_attempt1')
ROOTS = {m: W/'runs'/n/'prepared'/(m.lower()+'_source') for m, n in NAMES.items()}
PINS = {}


def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def pin(p):
    p = Path(p)
    PINS[str(p)] = sha(p)
    return p


def loadmod(name, path):
    s = importlib.util.spec_from_file_location(name, pin(path))
    mod = importlib.util.module_from_spec(s)
    s.loader.exec_module(mod)
    return mod


g = loadmod('cp92_geometry', E/'fixed_lag_eig2_g2k_reference_fl4y_20261017/analyze.py')
ANSI = re.compile(r'\x1b\[[0-9;]*m')
AUDIT = json.loads(pin(W/'runs'/NAMES['ON']/'prepared/input_audit.json').read_text())
EPOCH = AUDIT['epoch_ns']
X = np.asarray(AUDIT['T_alphasense_cpt7'])  # Alpha <- CPT7
Y = g.inv(X)


def adj(T):
    A = np.zeros((6, 6))
    A[:3, :3] = A[3:, 3:] = T[:3, :3]
    A[3:, :3] = g.hat(T[:3, 3]) @ T[:3, :3]
    return A


AY = adj(Y)


def ns(s):
    # Source logs use decimal seconds from doubles; preserve their decimal
    # representation. Pairing requires exact equality, with no snapping.
    return int((Decimal(str(s))*Decimal(10**9)).to_integral_value())


def decode(line, tag):
    return ANSI.sub('', line.split(tag+',', 1)[1]).strip().split(',')


def write(name, data):
    with (H/name).open('x') as f:
        json.dump(data, f, indent=2, allow_nan=False)


def stats(a):
    a = np.asarray(a, float)
    assert len(a) and np.isfinite(a).all()
    return dict(n=len(a), min=float(a.min()), median=float(np.median(a)),
                mean=float(a.mean()), p05=float(np.percentile(a, 5)),
                p95=float(np.percentile(a, 95)), max=float(a.max()))


def record(a, b, mu, C, update=None, ids=None):
    assert a < b
    return dict(a=a, b=b, mu=np.asarray(mu, float), C=np.asarray(C, float),
                update=update, ids=ids)


def load_source(mode):
    S = ROOTS[mode]
    wire = {'GLIM': {}, 'Kimera': {}}
    versions = {'GLIM': defaultdict(dict), 'Kimera': defaultdict(dict)}
    times = {'GLIM': {}, 'Kimera': {}}
    covariance_checks = {}
    counts = defaultdict(int)
    tag = 'KIMERA_CBS_REVISION_EDGE_ROW'
    for line in pin(S/'estimators.log').open():
        if tag+',' not in line:
            continue
        f = decode(line, tag)
        assert len(f) == 64
        cur, i, j = int(f[0]), int(f[2]), int(f[3])
        a, b = ns(f[4]), ns(f[5])
        for idx, t in [(i, a), (j, b)]:
            assert idx not in times['Kimera'] or times['Kimera'][idx] == t
            times['Kimera'][idx] = t
        M = Y @ g.exp(np.array(f[22:28], float)) @ X
        C = AY @ np.array(f[28:], float).reshape(6, 6) @ AY.T
        key = a, b
        assert cur not in versions['Kimera'][key], (mode, 'duplicate', key, cur)
        versions['Kimera'][key][cur] = record(a, b, g.log(M), C, cur, (i, j))
        counts['Kimera_versions'] += 1
    logs = [p for p in (S/'ros_logs').glob('*/glim_rosnode-1-stdout.log')
            if p.parent.name != 'latest']
    assert len(logs) == 1
    for line in pin(logs[0]).open():
        tag = next((tag for tag in ['GLIM_CBS_EXPORT_BELIEF_ROW', 'GLIM_CBS_REVISION_BELIEF_ROW']
                    if tag+',' in line), None)
        if tag is None:
            continue
        f = decode(line, tag)
        assert len(f) == 10
        i, j = int(f[1]), int(f[2])
        a, b = ns(f[6]), ns(f[7])
        for idx, t in [(i, a), (j, b)]:
            assert idx not in times['GLIM'] or times['GLIM'][idx] == t
            times['GLIM'][idx] = t
        cur = j if tag == 'GLIM_CBS_EXPORT_BELIEF_ROW' else int(f[5])
        key = a, b
        z = record(a, b, list(map(float, f[8].split(';'))), g.matrix(f[9]), cur, (i, j))
        if cur in versions['GLIM'][key]:
            old = versions['GLIM'][key][cur]
            assert np.array_equal(z['mu'], old['mu']) and np.array_equal(z['C'], old['C'])
        versions['GLIM'][key][cur] = z
        counts['GLIM_versions'] += 1
    # Keep every native version for payload parity, including sparse updates
    # whose exact timestamp is absent from relative-edge endpoint logging.
    wire_candidates = {r:{k:list(vv.values()) for k,vv in vs.items()}
                       for r,vs in versions.items()}
    for role in wire_candidates:
        for vv in wire_candidates[role].values():
            for z in vv:
                z['update_ns'] = times[role].get(z['update'], -1)
    # Convert source update indices to actual physical update timestamps. A
    # missing keyframe changes indices, so OFF/ON are never joined by index.
    for role in versions:
        for key, vv in list(versions[role].items()):
            remap = {}
            for cur, z in vv.items():
                if cur not in times[role]:
                    # Sparse keyframes can update the graph without exporting a
                    # fresh short-horizon edge. Do not invent their timestamps.
                    counts[role+'_versions_missing_update_timestamp'] += 1
                    continue
                t = times[role][cur]
                assert t >= z['b'] and t not in remap
                z['update_ns'] = t
                remap[t] = z
            if remap:
                versions[role][key] = remap
            else:
                del versions[role][key]
        cs = np.asarray([z['C'] for vv in versions[role].values() for z in vv.values()])
        sym = float(np.max(abs(cs-cs.transpose(0, 2, 1))))
        eig = float(np.linalg.eigvalsh(cs).min())
        assert np.isfinite(cs).all() and sym < 1e-12 and eig > 0
        covariance_checks[role] = dict(versions=len(cs), symmetry_max=sym, eigenvalue_min=eig)
    topics = {'/kimera/cbs/odom_belief_in': 'GLIM', '/kimera/cbs/odom_belief_out': 'Kimera'}
    joins = defaultdict(list)
    with AnyReader([pin(S/'recordings/cbs.bag')]) as reader:
        for c, t, raw in reader.messages(connections=[c for c in reader.connections if c.topic in topics]):
            msg = reader.deserialize(raw, c.msgtype)
            role = topics[c.topic]
            for belief in msg.beliefs:
                a, b = ns(belief.from_stamp_sec), ns(belief.to_stamp_sec)
                ids = (int(belief.from_pose_index), int(belief.to_pose_index))
                key = a, b
                C = np.asarray(belief.covariance).reshape(6, 6).copy()
                z = record(a, b, belief.relative_mu.copy(), C, ids=ids)
                assert int(belief.revision) == 0
                candidates = wire_candidates[role].get(key, [])
                # Native double timestamps can be printed with more digits than
                # ROS float repr. If so resolve by native IDs and require <1us.
                if not candidates:
                    keys = [k for k, vv in wire_candidates[role].items()
                            if vv[0]['ids'] == ids]
                    assert len(keys) == 1
                    nkey = keys[0]
                    assert max(abs(a-nkey[0]), abs(b-nkey[1])) <= 1000
                    candidates = wire_candidates[role][nkey]
                diffs = [(max(float(np.max(abs(g.exp(z['mu'])-g.exp(v['mu']))))/1e-9,
                              float(np.max(abs(C-v['C'])))/max(1e-30,float(np.max(abs(C))))/1e-6), u)
                         for u, v in enumerate(candidates)]
                best, u = min(diffs)
                assert best < 1, (mode, role, 'wire/native parity', best, ids, key,
                                  len(candidates), z['mu'].tolist(),
                                  candidates[u]['mu'].tolist())
                z['update_ns'] = candidates[u]['update_ns']
                z['update'] = candidates[u]['update']
                joins[role].append(best)
                counts[role+'_wire_messages'] += 1
                if key in wire[role]:
                    counts[role+'_repeated_wire_edge'] += 1
                    assert z['update'] >= wire[role][key]['update']
                else:
                    wire[role][key] = z
    return wire, versions, dict(counts=counts, covariance=covariance_checks,
                                wire_counts={r:len(v) for r,v in wire.items()},
                                native_wire_scaled_max={r:max(v) for r,v in joins.items()})


def errors(records, reference):
    C = np.array([z['C'] for z in records])
    est = np.array([g.exp(z['mu']) for z in records])
    ref = np.array([g.inv(reference.at(z['a'])) @ reference.at(z['b']) for z in records])
    err = np.array([g.log(g.inv(z)@r) for z,r in zip(est,ref)])
    q = np.einsum('ni,ni->n', err, np.linalg.solve(C, err[...,None])[...,0])
    qw = np.sum(np.linalg.solve(np.linalg.cholesky(C),err[...,None])[...,0]**2,axis=1)
    assert np.max(abs(q-qw)/(1+q)) < 1e-8
    # EVO computes adjacent pair errors; alternating I,Z constructs independent
    # motion pairs without pretending these are an absolute trajectory.
    n = len(records)
    poses0 = np.repeat(np.eye(4)[None],2*n,axis=0)
    poses1 = poses0.copy()
    poses0[1::2] = ref
    poses1[1::2] = est
    t = np.arange(2*n, dtype=float)
    tr0 = trajectory.PoseTrajectory3D(poses_se3=poses0,timestamps=t)
    tr1 = trajectory.PoseTrajectory3D(poses_se3=poses1,timestamps=t)
    evo_stats = {}
    evo_arrays = {}
    for name, relation in [('translation_m',metrics.PoseRelation.translation_part),
                           ('rotation_deg',metrics.PoseRelation.rotation_angle_deg)]:
        m = metrics.RPE(pose_relation=relation,delta=1,delta_unit=units.Unit.frames,all_pairs=True)
        m.process_data((tr0,tr1))
        m.error = m.error[::2]
        assert len(m.error) == n
        evo_arrays[name] = m.error.copy()
        evo_stats[name] = {k:float(v) for k,v in m.get_all_statistics().items()}
    assert np.max(abs(evo_arrays['rotation_deg']-np.linalg.norm(err[:,:3],axis=1)*180/np.pi)) < 1e-6
    assert np.max(abs(evo_arrays['translation_m']-np.linalg.norm(est[:,:3,3]-ref[:,:3,3],axis=1))) < 1e-8
    traces = np.stack([np.trace(C[:,:3,:3],axis1=1,axis2=2),np.trace(C[:,3:,3:],axis1=1,axis2=2)],axis=1)
    blocks = {}
    for idx,(name,sl,scale) in enumerate([('rotation',slice(0,3),180/np.pi),('translation',slice(3,6),1000)]):
        e = err[:,sl];c = C[:,sl,sl];tr = traces[:,idx]
        qs = np.einsum('ni,ni->n',e,np.linalg.solve(c,e[...,None])[...,0])
        norm = np.linalg.norm(e,axis=1)
        whitened = np.linalg.solve(np.linalg.cholesky(c),e[...,None])[...,0]
        blocks[name] = dict(trace=stats(tr),uncertainty_norm=stats(np.sqrt(tr)*scale),
                           error_norm=stats(norm*scale),
                           error_rms=float(np.sqrt(np.mean(norm**2))*scale),
                           reported_norm_rms=float(np.sqrt(np.mean(tr))*scale),
                           squared_error_over_variance=float(np.mean(norm**2)/np.mean(tr)),
                           normalized_squared=stats(qs),nominal_95_fraction=float(np.mean(qs<=chi2.ppf(.95,3))),
                           variance_error_spearman=float(spearmanr(tr,norm).statistic),
                           whitened_signed_mean=whitened.mean(axis=0).tolist(),
                           units='deg' if idx==0 else 'mm')
    summary=dict(n=n,rotation=blocks['rotation'],translation=blocks['translation'],
                 joint_q=stats(q),joint_95_fraction=float(np.mean(q<=chi2.ppf(.95,6))),
                 evo=evo_stats,relative_duration_s=stats([(z['b']-z['a'])*1e-9 for z in records]))
    arrays=dict(error=err,covariance=C,mean=np.array([z['mu'] for z in records]),
                endpoint_ns=np.array([(z['a'],z['b']) for z in records],np.int64),
                update_ns=np.array([z['update_ns'] for z in records],np.int64),q=q,
                translation_m=evo_arrays['translation_m'],rotation_deg=evo_arrays['rotation_deg'])
    return summary,arrays


def paired(left,right):
    result={}
    for name,sl in [('rotation',slice(0,3)),('translation',slice(3,6))]:
        v0=np.trace(left['covariance'][:,sl,sl],axis1=1,axis2=2)
        v1=np.trace(right['covariance'][:,sl,sl],axis1=1,axis2=2)
        e0=np.sum(left['error'][:,sl]**2,axis=1)
        e1=np.sum(right['error'][:,sl]**2,axis=1)
        ratio=v1/v0
        result[name]=dict(paired_variance_ratio=stats(ratio),
                          aggregate_squared_error_ratio=float(np.mean(e1)/np.mean(e0)),
                          aggregate_variance_ratio=float(np.mean(v1)/np.mean(v0)),
                          normalization_change=float((np.mean(e1)/np.mean(v1))/(np.mean(e0)/np.mean(v0))),
                          fraction_cov_down_error_up=float(np.mean((ratio<1)&(e1>e0))),
                          fraction_cov_below_half=float(np.mean(ratio<.5)),
                          fraction_cov_below_tenth=float(np.mean(ratio<.1)))
    return result


def self_test():
    rng=np.random.default_rng(92)
    for _ in range(50):
        v=rng.normal(size=6)*.2
        assert np.max(abs(g.log(g.exp(v))-v))<1e-12
        M=np.zeros((4,4));M[:3,:3]=g.hat(v[:3]);M[:3,3]=v[3:]
        assert np.max(abs(g.exp(v)-expm(M)))<1e-12
        A=rng.normal(size=(6,6));C=A@A.T+np.eye(6)*.1
        e=rng.normal(size=6)
        q=e@np.linalg.solve(C,e)
        assert abs(q-(AY@e)@np.linalg.solve(AY@C@AY.T,AY@e))<1e-10
    C=np.diag(np.arange(1,7))
    errors=rng.multivariate_normal(np.zeros(6),C,size=20000)
    q=np.einsum('ni,ni->n',errors,np.linalg.solve(C,errors.T).T)
    fraction=float(np.mean(q<=chi2.ppf(.95,6)))
    assert .94<fraction<.96
    return dict(SE3_matrix_exponential=True,dense_frame_invariance=True,
                gaussian_95_synthetic_fraction=fraction)


def main():
    assert not (H/'analysis.json').exists()
    assert importlib.metadata.version('evo')=='1.36.3'
    pin(__file__);pin(H/'PROTOCOL.txt')
    terminal={}
    for mode,name in NAMES.items():
        complete=json.loads(pin(E/name/'complete.json').read_text())
        assert complete['passed']
        terminal[mode]=complete['output_sha256']
        assert all(sha(p)==digest for p,digest in terminal[mode].items())
    test=self_test()
    wire={};versions={};checks={}
    for mode in NAMES:
        print('Reading',mode,flush=True)
        wire[mode],versions[mode],checks[mode]=load_source(mode)
        print(mode,checks[mode],flush=True)
    bag=pin(W/'src/datasets/GrandTour/2024-10-01-11-29-55_cpt7_ie_tc.bag')
    reference=g.Reference(bag)
    summaries={};arrays={};cohorts={};timebins={}
    for role in ['GLIM','Kimera']:
        print('Scoring',role,flush=True)
        common=sorted(set(wire['OFF'][role])&set(wire['ON'][role]))
        cohorts[role]=dict(wire_matched=len(common),
                           wire_unmatched={m:len(set(wire[m][role])-set(common)) for m in NAMES},
                           wire_nonmatching_first_update=sum(wire['OFF'][role][k]['update_ns']!=wire['ON'][role][k]['update_ns'] for k in common))
        selected={'wire_first':{m:[wire[m][role][k] for k in common] for m in NAMES}}
        # Native log endpoint prints are common to both modes; matching here
        # never uses source indices across modes. Keep physical ~0.2s intervals.
        native_keys=sorted(set(versions['OFF'][role])&set(versions['ON'][role]))
        eligible=[];targets={};excluded=defaultdict(int)
        for key in native_keys:
            if (key[1]-key[0])*1e-9>.5:
                excluded['long_interval_gt_0.5s']+=1;continue
            off=versions['OFF'][role][key];on=versions['ON'][role][key]
            if min(off)!=min(on) or min(off)!=key[1]:
                excluded['first_update_not_identical_endpoint']+=1;continue
            shared=sorted(set(off)&set(on))
            tt=[key[1]]
            for age in [1,2,4]:
                candidates=[u for u in shared if key[1]+age*10**9<=u<=key[1]+age*10**9+250_000_000]
                if not candidates:break
                tt.append(min(candidates))
            if len(tt)!=4:
                excluded['missing_shared_update_at_1_2_4s']+=1;continue
            eligible.append(key);targets[key]=tt
        cohorts[role].update(native_common_edges=len(native_keys),fixed_age_edges=len(eligible),fixed_age_excluded=dict(excluded))
        for pos,age in enumerate([0,1,2,4]):
            selected['age'+str(age)]={m:[versions[m][role][k][targets[k][pos]] for k in eligible] for m in NAMES}
        summaries[role]={}
        for stage,pair in selected.items():
            summaries[role][stage]={}
            for mode,rr in pair.items():
                su,ar=errors(rr,reference)
                su['age_s']=stats((ar['update_ns']-ar['endpoint_ns'][:,1])*1e-9)
                summaries[role][stage][mode]=su
                arrays[role+'_'+stage+'_'+mode]=ar
            summaries[role][stage]['ON_over_OFF']=paired(arrays[role+'_'+stage+'_OFF'],arrays[role+'_'+stage+'_ON'])
        summaries[role]['maturation']={m:paired(arrays[role+'_age0_'+m],arrays[role+'_age4_'+m]) for m in NAMES}
        timebins[role]={}
        for stage in ['wire_first','age4']:
            rows=[]
            base=arrays[role+'_'+stage+'_OFF']
            ts=(base['endpoint_ns'][:,1]-EPOCH)*1e-9
            for lo in range(0,360,30):
                mask=(ts>=lo)&(ts<lo+30)
                if not mask.any():continue
                row=dict(start_s=lo,end_s=lo+30,n=int(mask.sum()))
                for mode in NAMES:
                    ar=arrays[role+'_'+stage+'_'+mode]
                    row[mode]={}
                    for name,sl,scale in [('rotation',slice(0,3),180/np.pi),('translation',slice(3,6),1000)]:
                        C=ar['covariance'][mask,sl,sl];er=ar['error'][mask,sl]
                        tr=np.trace(C,axis1=1,axis2=2);norm=np.linalg.norm(er,axis=1)
                        row[mode][name]=dict(uncertainty_median=float(np.median(np.sqrt(tr))*scale),
                            error_median=float(np.median(norm)*scale),
                            squared_error_over_variance=float(np.mean(norm**2)/np.mean(tr)))
                row['ON_over_OFF']=paired({k:v[mask] for k,v in base.items()},
                    {k:v[mask] for k,v in arrays[role+'_'+stage+'_ON'].items()})
                rows.append(row)
            timebins[role][stage]=rows
    np.savez_compressed(H/'selected_records.npz',**{key+'__'+field:value for key,ar in arrays.items() for field,value in ar.items()})
    assert all(sha(p)==h for p,h in PINS.items())
    assert all(sha(p)==h for mm in terminal.values() for p,h in mm.items())
    result=dict(task='CP-V92',scope='saved CP89 OFF vs CP91 ON split-IMU full ETH1',
                evo_version=importlib.metadata.version('evo'),checks=checks,cohorts=cohorts,
                sources=summaries,time_bins=timebins,reference_max_bracket_ns=reference.max_bracket_ns,
                verification=test,input_hashes=PINS,original_terminals_unchanged={m:len(v) for m,v in terminal.items()},
                limitations=['Normalized reference discrepancy, not true NEES/NIS; IE-TC covariance/cross-correlation unknown.',
                  'IE-TC shares CPT7 with GLIM; ON Kimera can inherit correlated peer information.',
                  'Wire first publications and fixed-age revision cohorts are different populations.',
                  'Source relative belief covariance, not absolute pose covariance or post-time-aligned factor certification.',
                  'Overlapping edges and revisions are dependent; no confidence p-value or fitted scale.',
                  'EVO errors are at the actual belief interval, not the existing trajectory0.5mRPE.'])
    write('analysis.json',result)
    print(json.dumps(dict(cohorts=cohorts,reference_max_bracket_ns=reference.max_bracket_ns,
                          terminal_counts=result['original_terminals_unchanged']),indent=2),flush=True)


if __name__=='__main__':
    main()
