"""CP-V8: read-only scientific analysis of seven immutable saved OFF/ON pairs."""
import gzip
import importlib.util
import json
import sys
from pathlib import Path

sys.dont_write_bytecode = True
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.stats import spearmanr

H = Path(__file__).resolve().parent
E = H.parent
OUT = H / 'outputs'
spec = importlib.util.spec_from_file_location('cpv7_geometry', E/'leica2_drift_attribution_cpv7_20261025/analyze.py')
a = importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)
ROWS = [
    ('hesai_eth1', 'Hesai', 'ETH1', 'eth1_awake_full_cpk23_20261021', 'results.json', 346.32, 1727775007855396309),
    ('hesai_eig2', 'Hesai', 'EIG-2', 'eig2_full_cpk24_20261021', 'results_eof_review.json', 355.02, 1730638816563546034),
    ('hesai_spx3', 'Hesai', 'SPX-3', 'spx3_stationary_full_cpk28_20261023', 'results.json', 197.02, 1730565899872888651),
    ('hesai_leica2', 'Hesai', 'LEICA-2', 'leica2_full_cpk29_20261023', 'results.json', 428.75, 1732549034915071665),
    ('velodyne_spx3', 'Velodyne', 'SPX-3', 'velodyne_full_campaign_cpv4_20261024/spx3', 'results.json', 197.02, 1730565899872888651),
    ('velodyne_eig2', 'Velodyne', 'EIG-2', 'velodyne_full_campaign_cpv4_20261024/eig2', 'results.json', 355.02, 1730638816563546034),
    ('velodyne_leica2', 'Velodyne', 'LEICA-2', 'velodyne_full_campaign_cpv4_20261024/leica2', 'results.json', 428.75, 1732549034915071665),
]
ROLES = ('native_glim', 'native_kimera', 'fused_online')
MODES = ('off', 'on')


def local(p):
    return Path(p.replace('/workspace/cbs_gtsam4.3', '/home/yeranis/repos/V4RL/cbs_gtsam4.3'))


def begin():
    a.HASHES.clear()
    for p in (Path(__file__), H/'PROTOCOL.txt', Path(a.__file__), Path(a.g.__file__)):
        a.pin(p)
    OUT.mkdir(exist_ok=True)


def finish(path, data):
    assert all(a.g.sha(Path(p)) == sha for p, sha in a.HASHES.items())
    data['input_sha256'] = dict(a.HASHES)
    data['input_hashes_unchanged'] = True
    a.save(path, data)


def load(row):
    case, sensor, seq, directory, filename, duration, epoch = row
    old = json.loads(a.pin(E/directory/filename).read_text())
    ev = old['evaluation']
    gt = local(ev['reference']['path'])
    assert a.g.sha(a.pin(gt)) == ev['reference']['sha256']
    ref = a.g.Reference(gt)
    return old, ev, ref


def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


def trajectory_case(row):
    case, sensor, seq, directory, filename, duration, epoch = row
    dest = OUT/(case+'_trajectories.json')
    assert not dest.exists(), dest
    begin()
    old, ev, ref = load(row)
    curves = {}
    for mode in MODES:
        for role in ROLES:
            name = mode+'_'+role
            record = ev['results'][name]
            path = local(record['path'])
            assert a.g.sha(a.pin(path)) == record['sha256']
            curves[name] = a.load_curve(path)
    reference = ref.times, np.array(ref.poses)[:, :3, 3], np.array(ref.poses)[:, :3, :3]
    start = max(c[0][0] for c in curves.values())
    end = min(c[0][-1] for c in curves.values())
    grid = np.arange(((start+a.STEP-1)//a.STEP)*a.STEP, (end//a.STEP)*a.STEP+1, a.STEP, dtype=np.int64)
    gp, gr, valid = a.sample(reference, grid, 50_000_000)
    samples = {}
    for name, c in curves.items():
        p, r, good = a.sample(c, grid, 500_000_000)
        valid &= good
        samples[name] = p, r
    gp_prism, gr_prism = a.prism(gp, gr)
    vi = np.flatnonzero(valid)
    pieces = np.split(vi, np.flatnonzero(np.diff(vi)>1)+1)
    time = (grid-epoch)*1e-9
    common20 = a.intervals(valid, 200)
    assert len(common20)>100
    coverage = dict(grid_queries=len(grid), valid_queries=len(vi), excluded_queries=int((~valid).sum()),
                    supported_s=sum((len(p)-1)*.1 for p in pieces), envelope_s=float(time[vi[-1]]-time[vi[0]]),
                    first_s=float(time[vi[0]]), last_s=float(time[vi[-1]]), capture_s=duration,
                    segments=[dict(start_s=float(time[p[0]]),end_s=float(time[p[-1]]),n=len(p)) for p in pieces],
                    same_start_cohort_n=len(common20), reference_max_abs_pitch_deg=float(np.max(np.abs(R.from_matrix(gr[valid]).as_euler('xyz',degrees=True)[:,1]))))
    metrics = {}
    arrays = dict(timestamp_ns=grid, time_s=time, valid=valid)
    for name, (p, r) in samples.items():
        pp, rr = a.prism(p, r)
        full = curves[name]
        fp, fr, ok = a.sample(reference, full[0], 50_000_000)
        assert ok.all(), name
        fep, _ = a.prism(full[1], full[2])
        fgp, _ = a.prism(fp, fr)
        ate = rms(np.linalg.norm(a.best_fit(fep, fgp),axis=1))
        delta = abs(ate-ev['results'][name]['full_saved']['ATE_RMSE_m'])
        assert delta<1e-5, (case,name,delta)
        dp, _, _ = a.fixed_start(pp, rr, gp_prism, gr_prism, int(vi[0]))
        _, _, angles = a.fixed_start(p, r, gp, gr, int(vi[0]))
        d = dict(common_ATE_m=rms(np.linalg.norm(a.best_fit(pp[valid],gp_prism[valid]),axis=1)),
                 full_saved_ATE_reproduction_error_m=delta,
                 fixed_start_terminal_position_m=float(np.linalg.norm(dp[vi[-1]])),
                 fixed_start_terminal_heading_deg=float(angles[vi[-1],2]),
                 fixed_start_rpy_rms_deg=np.sqrt(np.mean(angles[valid]**2,axis=0)),
                 common_supported_path_m=float(sum(np.linalg.norm(np.diff(pp[piece],axis=0),axis=1).sum() for piece in pieces)),
                 common_supported_reference_path_m=float(sum(np.linalg.norm(np.diff(gp_prism[piece],axis=0),axis=1).sum() for piece in pieces)),
                 horizons={},same_start_horizons={})
        for h in (1,10,20):
            for label, ii in [('horizons',a.intervals(valid,h*10)),('same_start_horizons',common20)]:
                jj=ii+h*10
                dt, dr, _ = a.rpe(pp,rr,gp_prism,gr_prism,ii,jj)
                assert np.all(grid[jj]-grid[ii] == h*1_000_000_000)
                d[label][str(h)] = dict(n=len(ii),translation_RMS_m=rms(np.linalg.norm(dt,axis=1)),rotation_RMS_deg=rms(np.linalg.norm(dr,axis=1)))
        metrics[name]=d
        arrays[name+'_position_error']=dp
        arrays[name+'_rpy_error_deg']=angles
    # Keep historical masks/scores separate; integer-ns grid fixes floating arange drift.
    data=dict(case=case,sensor=sensor,sequence=seq,capture_s=duration,start_ns=epoch,
              original_report=str(E/directory/filename),original_evaluation=ev,
              new_common_coverage=coverage,new_common_metrics=metrics,
              terminal_omissions=old.get('terminal_omissions'),passed_original=old.get('passed'),
              synthetic_tests=a.tests())
    np.savez_compressed(OUT/(case+'_series.npz'),**arrays)
    finish(dest,data)
    print(case,'trajectory analysis complete',len(vi),'common queries',flush=True)


def directional_rows(rows, ref):
    result=[]
    for x in rows:
        ra,rb=ref.at(x['a']),ref.at(x['b'])
        v=dict(a=x['a'],b=x['b'],time_s=x['time_s'],interval_s=x['interval_s'])
        for name,mufield,cf in [('incoming','incoming_mu','applied_cov'),('pre','pre_mu','pre_cov'),('post','post_mu',None)]:
            T=a.g.exp(np.asarray(x[mufield]))
            pred=ra[:3,:3]@T[:3,:3]
            diff=R.from_matrix(pred@rb[:3,:3].T).as_rotvec()*180/np.pi
            heading=(np.arctan2(pred[1,0],pred[0,0])-np.arctan2(rb[1,0],rb[0,0])+np.pi)%(2*np.pi)-np.pi
            d=dict(heading_deg=float(heading*180/np.pi),vertical_deg=float(diff[2]),tilt_deg=float(np.linalg.norm(diff[:2])),translation_mm=x[name]['translation_mm'])
            if cf:
                C=np.asarray(x[cf]);W=pred@C[:3,:3]@pred.T
                assert np.isclose(np.trace(W),np.trace(C[:3,:3]),rtol=1e-12)
                d.update(vertical_std_deg=float(np.sqrt(W[2,2])*180/np.pi),tilt_norm_deg=float(np.sqrt(np.trace(W[:2,:2]))*180/np.pi),
                         translation_norm_mm=float(np.sqrt(np.trace(C[3:,3:]))*1000))
            v[name]=d
        result.append(v)
    return result


def direction_summary(rows):
    if not rows:return dict(n=0)
    d=dict(n=len(rows),first_s=min(x['time_s'] for x in rows),last_s=max(x['time_s'] for x in rows),
           interval_s=a.stat([x['interval_s'] for x in rows]))
    for name in ('incoming','pre','post'):
        d[name]={k+'_rms':rms([x[name][k] for x in rows]) for k in ('heading_deg','vertical_deg','tilt_deg','translation_mm')}
        if name!='post':
            for field in ('vertical_std_deg','tilt_norm_deg','translation_norm_mm'):
                d[name][field+'_rms']=rms([x[name][field] for x in rows])
            for err,cov in [('vertical_deg','vertical_std_deg'),('tilt_deg','tilt_norm_deg'),('translation_mm','translation_norm_mm')]:
                rho=float(spearmanr([abs(x[name][err]) for x in rows],[x[name][cov] for x in rows]).statistic)
                d[name][err+'_covariance_rank']=rho if np.isfinite(rho) else None
    d['incoming_better_fraction']={k:float(np.mean([abs(x['incoming'][k])<abs(x['pre'][k]) for x in rows])) for k in ('heading_deg','tilt_deg','translation_mm')}
    # Hindsight selector diagnostics, NOT a new fused estimate or executable online policy.
    d['lower_marginal_selector']={}
    for err,cov in [('vertical_deg','vertical_std_deg'),('tilt_deg','tilt_norm_deg'),('translation_mm','translation_norm_mm')]:
        chosen=[x['incoming'][err] if x['incoming'][cov]<x['pre'][cov] else x['pre'][err] for x in rows]
        d['lower_marginal_selector'][err]=dict(rms=rms(chosen),choose_incoming_fraction=float(np.mean([x['incoming'][cov]<x['pre'][cov] for x in rows])))
    return d


def peer_case(row):
    case,sensor,seq,directory,filename,duration,epoch=row
    dest=OUT/(case+'_peers.json')
    assert not dest.exists(),dest
    begin()
    old,ev,ref=load(row)
    src=local(ev['results']['on_native_kimera']['path']).parent
    a.START=epoch
    rows,checks=a.peer_records(src,'on',ref)
    dr=directional_rows(rows,ref)
    bins=[]
    for t in range(0,int(np.ceil(duration/30))*30,30):
        bins.append(dict(start_s=t,end_s=t+30,summary=direction_summary([x for x in dr if t<=x['time_s']<t+30])))
    with gzip.open(OUT/(case+'_peer_records.jsonl.gz'),'wt') as f:
        for x in rows:f.write(json.dumps(x,default=a.serial,allow_nan=False)+'\n')
    with gzip.open(OUT/(case+'_directions.jsonl.gz'),'wt') as f:
        for x in dr:f.write(json.dumps(x,allow_nan=False)+'\n')
    finish(dest,dict(case=case,checks=checks,all=direction_summary(dr),bins=bins,
                     max_reference_bracket_ns=ref.max_bracket_ns))
    print(case,'actual peer analysis complete',len(rows),'verified factors',flush=True)


def assemble():
    begin()
    dest=OUT/'summary.json';assert not dest.exists()
    data={}
    for row in ROWS:
        case=row[0]
        data[case]={}
        for kind in ('trajectories','peers'):
            obj=json.loads(a.pin(OUT/(case+'_'+kind+'.json')).read_text())
            for p,sha in obj['input_sha256'].items():
                assert a.g.sha(Path(p))==sha
                a.HASHES[p]=sha
            data[case][kind]=obj
    # Check OFF Kimera identity across sensor runs; do not count as independent baselines.
    identity={}
    for seq in ('spx3','eig2','leica2'):
        x=data['hesai_'+seq]['trajectories']['original_evaluation']['results']['off_native_kimera']['sha256']
        y=data['velodyne_'+seq]['trajectories']['original_evaluation']['results']['off_native_kimera']['sha256']
        identity[seq]=dict(byte_identical=x==y,hesai_sha256=x,velodyne_sha256=y)
    # Matching actual receiver endpoint pairs is a separate sensor-cohort sensitivity check.
    matched={}
    for seq in ('spx3','eig2','leica2'):
        look={}
        for sensor in ('hesai','velodyne'):
            p=a.pin(OUT/(sensor+'_'+seq+'_directions.jsonl.gz'))
            with gzip.open(p,'rt') as f: rr=[json.loads(line) for line in f]
            look[sensor]={(x['a'],x['b']):x for x in rr}
            assert len(look[sensor])==len(rr)
        keys=set(look['hesai'])&set(look['velodyne'])
        matched[seq]=dict(n=len(keys),unmatched={s:len(v)-len(keys) for s,v in look.items()},
                         summaries={s:direction_summary([v[k] for k in sorted(keys)]) for s,v in look.items()},
                         bins=[dict(start_s=t,n=sum(t<=look['hesai'][k]['time_s']<t+30 for k in keys)) for t in range(0,450,30)])
    finish(dest,dict(cases=data,kimera_off_cross_sensor_identity=identity,matched_cross_sensor_peers=matched,
                     limits='Descriptive saved-data study; no independent repeated trials, calibrated NEES/NIS or isolated causal attribution.'))
    print('Summary complete; all pinned inputs unchanged',flush=True)


if __name__=='__main__':
    stage=sys.argv[1]
    selected=[x for x in ROWS if len(sys.argv)<3 or x[0]==sys.argv[2]]
    if stage=='trajectories':
        for row in selected:trajectory_case(row)
    elif stage=='peers':
        for row in selected:peer_case(row)
    elif stage=='assemble':assemble()
    else:raise ValueError(stage)
