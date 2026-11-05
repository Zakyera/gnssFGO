"""CP-V17 saved-only ARC-3 scoring; CP-V10 integer-time/gap-aware geometry."""
import collections,csv,hashlib,importlib.util,json,sys
from pathlib import Path
sys.dont_write_bytecode=True
import numpy as np
ROOT=Path(__file__).resolve().parent
OUT=ROOT/'evaluation_full'
E=Path('/home/yeranis/repos/gnssFGO/experiments')
spec=importlib.util.spec_from_file_location('cpv10',E/'first_published_comparison_cpv10_20261026/analyze.py')
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b);a=b.a
PRIMARY=('native_glim','native_kimera','fused_online')

def sha(p):return a.g.sha(p)
def source_root(mode):return ROOT
def stat(v):return a.stat(v)

def on_grid(curve,reference,grid,valid=None):
    p,r,ok=a.sample(curve,grid,500_000_000);p,r=a.prism(p,r)
    gp,gr,gok=a.sample(reference,grid,50_000_000);gp,gr=a.prism(gp,gr)
    mask=ok & gok if valid is None else ok & gok & valid
    return p,r,gp,gr,mask

def score(curve,reference,grid,valid):
    p,r,gp,gr,mask=on_grid(curve,reference,grid,valid)
    m,_=b.metrics(p,r,gp,gr,mask,grid)
    ii=a.intervals(mask,1);jj=ii+1
    # Same no-scale global alignment; step-vector errors remove fixed origin.
    u,_,_=b.umeyama_alignment(p[mask].T,gp[mask].T,with_scale=False)
    d=(p[jj]-p[ii])@u.T-(gp[jj]-gp[ii])
    return dict(rows=int(mask.sum()),start_s=float(grid[mask][0]*1e-9),
        end_s=float(grid[mask][-1]*1e-9),ATE_RMSE_m=m['ATE_RMSE_m'],
        translation_RPE_1s_RMSE_m=m['horizons']['1']['translation_RMSE_m'],
        rotation_RPE_1s_RMSE_deg=m['horizons']['1']['rotation_RMSE_deg'],
        path_length_prism_m=m['path_length_m'],reference_path_length_prism_m=m['reference_path_length_m'],
        rpe_pairs=m['horizons']['1']['pairs'],horizons=m['horizons'],
        displacement_100ms_vector_error_m=stat(np.linalg.norm(d,axis=1)),
        alignment_crosscheck_m=m['evo_alignment_difference_m'],
        direct_RPE_check_max=m['direct_RPE_check_max'])

def make_grid(first,last):
    return np.arange(((first+a.STEP-1)//a.STEP)*a.STEP,(last//a.STEP)*a.STEP+1,a.STEP,dtype=np.int64)

def score_saved(curve,reference):
    t,p,r=curve;gp,gr,ok=a.sample(reference,t,50_000_000)
    assert ok.all(),'Saved output without reference bracket'
    p,r=a.prism(p,r);gp,gr=a.prism(gp,gr)
    ate,_,delta=b.ate(p,gp)
    grid=make_grid(t[0],t[-1]);s=score(curve,reference,grid,np.ones(len(grid),bool))
    # Full native path deliberately retains every saved point and gap chords.
    s.update(rows=len(t),start_s=float(t[0]*1e-9),end_s=float(t[-1]*1e-9),
        span_s=float((t[-1]-t[0])*1e-9),ATE_RMSE_m=ate,
        path_length_prism_m=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum()),
        reference_path_length_prism_m=float(np.linalg.norm(np.diff(gp,axis=0),axis=1).sum()),
        gap_chords_over_half_second=int(np.sum(np.diff(t)>500_000_000)),
        native_ATE_alignment_check_m=delta,
        RPE_note='Per-product supported 100ms grid; reject all 1s intervals touching a gap')
    return s

def main():
    assert not OUT.exists(),'Never overwrite scoring'
    pre=json.loads((ROOT/'input_audit.json').read_text());win=pre['window']
    epoch=win['start_ns'];duration=win['duration_s']
    paths={};curves={};checks={}
    for mode in ('off','on'):
        s=ROOT/f'full_{mode}_source';r=ROOT/f'full_{mode}_receiver/receiver'
        for f in (ROOT/f'full_{mode}_control/complete.json',s/'complete.marker',r.parent/'complete.marker'):assert f.exists(),f
        assert not (ROOT/f'full_{mode}_control/failed.txt').exists()
        for role,name in [('native_glim',s/'glim_odometry.csv'),('native_kimera',s/'kimera_odometry.csv'),
                          ('fused_online',r/'online_trajectory.tum'),('glim_refined',s/'odom_imu.txt'),
                          ('fused_finalized',r/'finalized_trajectory.tum')]:
            key=mode+'_'+role;paths[key]=name;curves[key]=a.load_curve(name)
            if role=='native_glim':checks[key]=b.check_csv(name)
    gt=Path(pre['bag_files']['cpt7_ie_tc']['path']);assert sha(gt)==pre['bag_files']['cpt7_ie_tc']['sha256']
    ref=a.g.Reference(gt);reference=(ref.times,np.asarray(ref.poses)[:,:3,3],np.asarray(ref.poses)[:,:3,:3])
    primary=[m+'_'+r for m in ('off','on') for r in PRIMARY]
    grid=make_grid(max(curves[k][0][0] for k in primary),min(curves[k][0][-1] for k in primary))
    _,_,valid=a.sample(reference,grid,50_000_000)
    for k in primary:valid &= a.sample(curves[k],grid,500_000_000)[2]
    results={};arrays=dict(timestamp_ns=grid,common_valid=valid)
    for name,c in curves.items():
        _,_,_,_,mask=on_grid(c,reference,grid,valid)
        results[name]=dict(path=str(paths[name]),sha256=sha(paths[name]),
            product='first-published' if name.endswith('native_glim') else
                ('DELAYED fixed-lag-finalized' if name.endswith('fused_finalized') else
                 'refined GLIM; secondary only' if name.endswith('glim_refined') else 'native first-published/ONLINE'),
            full_saved_rows=len(c[0]),full_saved_span_s=float((c[0][-1]-c[0][0])*1e-9),
            full_saved_path_length_CPT7_m=float(np.linalg.norm(np.diff(c[1],axis=0),axis=1).sum()),
            full_saved=score_saved(c,reference),common=score(c,reference,grid,valid),
            common_coverage=b.coverage(grid,mask,epoch,duration))
        p,_,gp,_,_=on_grid(c,reference,grid,valid)
        arrays[name+'_position_prism']=p
        # Whole difficult input interval, predeclared from raw gaps, no metric-based selection.
        lo=min(g['before_ns'] for g in pre['known_raw_gaps'] if g['gap_s']>1)-2_000_000_000
        hi=max(g['after_ns'] for g in pre['known_raw_gaps'] if g['gap_s']>1)+2_000_000_000
        ownmask=on_grid(c,reference,grid)[-1] & (grid>=lo)&(grid<=hi)
        indices=a.intervals(ownmask,1)
        # Short-window diagnostic may be sparse; no minimum-duration gate or fabricated interpolation.
        results[name]['input_gap_cluster']=dict(start_ns=lo,end_ns=hi,supported_queries=int(ownmask.sum()),
            supported_100ms_intervals=len(indices),note='Sparse support explicitly counted; primary is full journey')
    coverage=b.coverage(grid,valid,epoch,duration)
    comparison={r:dict(ATE_improvement_percent=b.improve(results['off_'+r]['common']['ATE_RMSE_m'],results['on_'+r]['common']['ATE_RMSE_m']),
        translation_RPE1_improvement_percent=b.improve(results['off_'+r]['common']['translation_RPE_1s_RMSE_m'],results['on_'+r]['common']['translation_RPE_1s_RMSE_m'])) for r in PRIMARY}
    report=dict(passed=True,reference=dict(path=str(gt),sha256=sha(gt),topic='/boxi/inertial_explorer/tc/odometry',
        caveat='IE-TC shares CPT7 IMU; not independent prism ground truth'),
        common_mask=dict(rows=int(valid.sum()),coverage=coverage,primary_products=primary,
            timestamps_sha256=hashlib.sha256(grid[valid].astype('<i8').tobytes()).hexdigest(),
            interpolation_gap_estimate_s=.5,interpolation_gap_reference_s=.05),
        results=results,off_on=comparison,first_published_checks=checks,
        covariance={m:covariances(m) for m in ('off','on')},synthetic_tests=a.tests(),
        path_length_caveat='Full native polylines include gap chords. Common lengths sum contiguous segments only.',
        convention='No-scale rigid SE3 ATE/prism lever arm; gap-aware integer-ns100ms grid; 1s RPE excludes every gap-crossing interval',
        qc_multiplier_original=.25,not_a_Qc_ablation=True,
        limitations=['Mean-only covariance alignment approximation unchanged','No nonlinear stationarity certification',
                    'Finalized fusion is delayed, not ONLINE or full history','Original source covariances Gaussian scale1'])
    assert all(sha(Path(p))==digest for p,digest in a.HASHES.items())
    report['input_sha256']=dict(a.HASHES)
    OUT.mkdir();a.save(OUT/'summary.json',report)
    np.savez_compressed(OUT/'common_series.npz',**arrays)
    print(json.dumps(dict(passed=True,common_coverage=coverage,off_on=comparison),indent=2))
def covariances(mode):
    path = source_root(mode) / f'full_{mode}_receiver/events.csv'
    groups = collections.defaultdict(list)
    versions = collections.defaultdict(list)
    with path.open(newline='') as f:
        for row in csv.reader(f):
            if row[0] != 'B':
                continue
            assert len(row) == 52
            source = row[2]
            C = np.asarray([float(x) for x in row[16:]], float).reshape(6, 6)
            assert np.isfinite(C).all()
            sym = float(np.max(np.abs(C-C.T)))
            eig = np.linalg.eigvalsh((C+C.T)/2)
            groups[source].append((float(np.trace(C[:3,:3])),
                                   float(np.trace(C[3:,3:])), float(eig.min()),
                                   float(eig.max()), sym))
            versions[(source,row[3],row[6],row[7])].append(C)
    result = {}
    for source, values in groups.items():
        a = np.asarray(values)
        first_last = [(float(np.trace(v[0][:3,:3])), float(np.trace(v[-1][:3,:3])),
                       float(np.trace(v[0][3:,3:])), float(np.trace(v[-1][3:,3:])))
                      for key,v in versions.items() if key[0] == source and len(v)>1]
        result[source] = dict(versions=len(a), stable_ids=sum(k[0]==source for k in versions),
            revised_ids=len(first_last), rotation_trace_rad2=stat(a[:,0]),
            translation_trace_m2=stat(a[:,1]), min_eigenvalue=float(a[:,2].min()),
            max_eigenvalue=float(a[:,3].max()), max_symmetry_error=float(a[:,4].max()),
            first_last_rotation_trace_ratio=stat([x[1]/x[0] for x in first_last if x[0]>0]) if first_last else None,
            first_last_translation_trace_ratio=stat([x[3]/x[2] for x in first_last if x[2]>0]) if first_last else None)
    return result

if __name__=='__main__':main()

