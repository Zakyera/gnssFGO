"""CP-V19: read saved failed CP-V18 only; never invoke an estimator or container."""
from pathlib import Path
import csv, hashlib, importlib.util, json, re, sys
sys.dont_write_bytecode = True
import numpy as np

H = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P = W/'runs/gnssfgo_arc3_velodyne_on_cpv18_20261027/prepared'
S = P/'full_on_source'
START = 1731934117040035611
ANSI = re.compile(r'\x1b\[[0-9;]*m')
pins = {}
def pin(p):
    p = Path(p)
    with p.open('rb') as f: digest = hashlib.file_digest(f, 'sha256').hexdigest()
    pins[str(p)] = digest
    return p
def load(p): return json.loads(pin(p).read_text())
def stat(x):
    a = np.asarray(x, float)
    if not a.size: return {'n': 0}
    assert np.isfinite(a).all()
    return dict(n=len(a), median=float(np.median(a)), p95=float(np.percentile(a,95)), max=float(a.max()))
def fields(line, marker): return line.split(marker+',',1)[1].strip().split(',')
def cov_stats(token):
    c = np.array([[float(v) for v in r.split(':')] for r in token.split(';')])
    assert c.shape == (6,6) and np.isfinite(c).all()
    ev = np.linalg.eigvalsh((c+c.T)/2)
    return dict(rotation_trace_rad2=float(np.trace(c[:3,:3])), translation_trace_m2=float(np.trace(c[3:,3:])),
                min_eigenvalue=float(ev.min()), max_eigenvalue=float(ev.max()),
                mixed_unit_condition=float(ev.max()/ev.min()))

def main():
    out = H/'diagnosis.json'
    assert not out.exists(), 'Do not overwrite diagnostic evidence'
    audit = load(P/'input_audit.json')
    node = next(S.glob('ros_logs/*/glim_rosnode-*-stdout.log'))
    selected, edge, decomp, scans, active = [], [], [], [], []
    counts = {'provider_successes':0, 'revision_successes':0}
    for no, raw in enumerate(pin(node).open(),1):
        line = ANSI.sub('',raw).strip()
        if 'SHARED_NATIVE_PROVIDER_QUERY ' in line:
            counts['provider_successes'] += 1
            if 'epoch=10047 ' in line: selected.append(dict(line=no,text=line))
        if 'GLIM_CBS_REVISION_BELIEF_ROW,' in line or 'GLIM_CBS_EXPORT_BELIEF_ROW,' in line:
            revision = 'GLIM_CBS_REVISION_BELIEF_ROW,' in line
            f = fields(line,'GLIM_CBS_REVISION_BELIEF_ROW' if revision else 'GLIM_CBS_EXPORT_BELIEF_ROW')
            if revision: counts['revision_successes'] += 1
            if (f[1:3] == ['5018','5020']) or (revision and f[5]=='5023'):
                edge.append(dict(line=no,kind='revision' if revision else 'first',from_id=int(f[1]),to_id=int(f[2]),
                    version=int(f[3]),publication_relative_s=float(f[4])-START*1e-9,
                    update=int(f[5]) if revision else None,from_stamp_s=float(f[6]),to_stamp_s=float(f[7]),
                    mean_log6=[float(x) for x in f[8].split(';')],covariance=cov_stats(f[9])))
        if 'GLIM_CBS_ACTIVE_POSE_ROW,g,5023,' in line:
            f=fields(line,'GLIM_CBS_ACTIVE_POSE_ROW');active.append(dict(line=no,id=int(f[3]),stamp=float(f[4]),pose=list(map(float,f[5:]))))
        if 'CBS_ODOM_RELATIVE_COVARIANCE_DECOMP_ROW,' in line:
            f=fields(line,'CBS_ODOM_RELATIVE_COVARIANCE_DECOMP_ROW')
            t=float(re.search(r'\[\d+\.\d+, (\d+\.\d+)\]',line).group(1))-START*1e-9
            vals=list(map(float,f[5:]))
            decomp.append(dict(t=t,line=no,from_id=f[3],to_id=f[4],endpoint_trace_ff=vals[0],endpoint_trace_tt=vals[1],
                projected_from=vals[2],projected_to=vals[3],projected_cross=vals[4],projected_sum=vals[5],
                cancellation_ratio=(abs(vals[2])+abs(vals[3])+abs(vals[4]))/abs(vals[5])))
        if 'GLIM_ODOM_IMU_TIMING_ROW,' in line:
            f=fields(line,'GLIM_ODOM_IMU_TIMING_ROW');assert len(f)==25
            scans.append(list(map(float,f[:24])))
    errors=[]
    needles=['SHARED_NATIVE_SHADOW_FAILURE','GLIM_CBS_REVISION_ERROR_ROW','SHARED_NATIVE_PROVIDER_FAILURE',
             'IndeterminantLinearSystemException','indeterminant linear','CUDA error','out of memory','CPK9_PRIOR_FAILURE']
    for no,raw in enumerate(pin(S/'estimators.log').open(),1):
        line=ANSI.sub('',raw).strip()
        if any(s in line for s in needles): errors.append(dict(line=no,text=line))
    failures=[e for e in errors if 'extraction/chart mismatch' in e['text']]
    assert failures and 'epoch=10047' in failures[0]['text']
    timing={}
    for lo,hi in [(480,500),(500,512.1),(512.1,513.6)]:
        ss=[r for r in scans if lo<=r[0]-START*1e-9<hi]
        dd=[r for r in decomp if lo<=r['t']<hi]
        timing[f'{lo}-{hi}']={'scan_ms':stat([r[23] for r in ss]),'main_smoother_ms':stat([r[17] for r in ss]),
            'active_frames':stat([r[5] for r in ss]),'endpoint_ff_mixed_trace':stat([r['endpoint_trace_ff'] for r in dd]),
            'covariance_cancellation_ratio':stat([r['cancellation_ratio'] for r in dd])}
    # Existing read-only reference parser; no source replay, new alignment or full-result scoring.
    refmod=H.parent/'fixed_lag_eig2_g2k_reference_fl4y_20261017/analyze.py'
    spec=importlib.util.spec_from_file_location('reference',pin(refmod));g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
    gt=pin(Path(audit['bag_files']['cpt7_ie_tc']['path']));assert pins[str(gt)]==audit['bag_files']['cpt7_ie_tc']['sha256']
    reference=g.Reference(gt)
    motion={}
    paths={'glim':S/'glim_odometry.csv','kimera':S/'kimera_odometry.csv',
           'fusion_online':P/'full_on_receiver/receiver/online_trajectory.tum'}
    for name,p in paths.items():
        if p.suffix=='.csv':
            rows=list(csv.DictReader(pin(p).open()))
            ts=np.array([int(r['field.header.stamp']) for r in rows],np.int64)
            xyz=np.array([[float(r['field.pose.pose.position.'+k]) for k in 'xyz'] for r in rows])
        else:
            from decimal import Decimal
            rows=[r.split() for r in pin(p).read_text().splitlines() if r.strip() and not r.startswith('#')]
            ts=np.array([int(Decimal(r[0])*10**9) for r in rows],np.int64);xyz=np.array([list(map(float,r[1:4])) for r in rows])
        assert np.all(np.diff(ts)>0)
        groups={}
        for lo,hi in [(300,310),(480,500),(500,513.55),(512.1,513.55)]:
            ii=np.flatnonzero((ts[:-1]>=START+int(lo*1e9))&(ts[1:]<=START+int(hi*1e9)))
            steps=np.linalg.norm(xyz[ii+1]-xyz[ii],axis=1)
            gtsteps=np.array([np.linalg.norm(reference.at(int(ts[i+1]))[:3,3]-reference.at(int(ts[i]))[:3,3]) for i in ii])
            top=sorted(zip(steps,ii,gtsteps),reverse=True)[:6]
            groups[f'{lo}-{hi}']=dict(step_m=stat(steps),reference_step_m=stat(gtsteps),
              summed_chords_m=float(steps.sum()),reference_summed_chords_m=float(gtsteps.sum()),
              largest=[dict(start_s=float((ts[i]-START)*1e-9),end_s=float((ts[i+1]-START)*1e-9),
                            step_m=float(d),reference_step_m=float(gd)) for d,i,gd in top])
        motion[name]=dict(rows=len(ts),start_s=float((ts[0]-START)*1e-9),end_s=float((ts[-1]-START)*1e-9),groups=groups)
    profiles=[json.loads(l) for l in pin(S/'e1b_profile.jsonl').open()]
    code_files=[W/'runs/shared_linearization_s4_provider_20261012/release_2/source/Provider.h',
      W/'runs/shared_linearization_s4_provider_20261012/release_2/source/ShadowObserver.cpp',
      W/'runs/native_covariance_cache_cpi3_20261019/source/NativeGaussianHistory.cpp',
      W/'runs/local_revision_covariance_cpi1_20261019/cbs_bridge.cpp',
      W/'runs/main_gpu_batching_cpk16_20261020/release_1/libgtsam_points.so.1.2.1',
      W/'runs/local_revision_covariance_cpi1_20261019/libglim_ros.so']
    manifest=load(P/'protected_manifest.json')
    checked={}
    for p in code_files:
        pin(p);expected=manifest.get(str(p));checked[str(p)]=dict(sha256=pins[str(p)],manifest_sha256=expected)
        if expected is not None: assert pins[str(p)]==expected
    eff=load(S/'effective_mode_check.json')
    assert eff['cached_provider_sha256']==pins[str(code_files[-2])]
    result=dict(task='CP-V19',scope='saved-data diagnosis only; CP-V18 remains failed prefix',
      errors=errors,same_update_success=selected,selected_beliefs=edge,active_poses_at_failure=active,
      successful_covariance_decomposition=decomp[-35:],window_statistics=timing,native_motion=motion,
      reference_note='IE-TC CPT7 IMU origin, shared CPT7; not independent prism GT; no alignment needed for chord lengths',
      raw_gaps=[dict(start_s=(r['before_ns']-START)*1e-9,end_s=(r['after_ns']-START)*1e-9,gap_s=r['gap_s']) for r in audit['known_raw_gaps'] if r['gap_s']>1],
      log_counts=counts,profile_rows=len(profiles),profile_aborted=sum(x['aborted'] for x in profiles),
      failing_scan=next(r for r in scans if int(r[1])==5023),
      transport=load(P/'full_on_receiver/transport_status.json'),code_identity=checked,
      closeout=load(H.parent/'arc3_velodyne_on_cpv18_20261027/closeout.json'),input_sha256=pins)
    with out.open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
    print(json.dumps({k:result[k] for k in ['window_statistics','native_motion','profile_aborted']},indent=2))

if __name__=='__main__': main()
