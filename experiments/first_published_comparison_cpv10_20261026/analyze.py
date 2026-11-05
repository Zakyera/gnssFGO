"""CP-V10: immutable saved first-publication trajectory rescoring."""
import csv
import importlib.util
import json
import sys
from pathlib import Path

sys.dont_write_bytecode = True
import numpy as np
from scipy.spatial.transform import Rotation as R
from evo.core.geometry import umeyama_alignment
import evo.core.geometry as evo_geometry

H = Path(__file__).resolve().parent
E = H.parent
OUT = H/'outputs'
spec = importlib.util.spec_from_file_location('cpv8', E/'cross_sequence_decision_cpv8_20261026/analyze.py')
b = importlib.util.module_from_spec(spec)
spec.loader.exec_module(b)
a = b.a
ROLES = ('glim_initial', 'kimera', 'fusion_online')
OLD_ROLES = {'glim_refined': 'native_glim', 'kimera': 'native_kimera', 'fusion_online': 'fused_online'}
SOURCES = [
    '/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/glim_ros1/src/glim_ros/cbs_bridge.cpp',
    '/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/glim/src/glim/odometry/odometry_estimation_imu.cpp',
    '/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/glim/src/glim/mapping/global_mapping.cpp',
    '/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO/src/backend/VioBackend.cpp',
    '/home/yeranis/repos/gnssFGO/experiments/receiver_finalization_cpk19_20261021/Receiver.h',
]


def rms(x):
    return float(np.sqrt(np.mean(np.square(x))))


def improve(off, on):
    return 100*(off-on)/off


def start_hashes():
    a.HASHES.clear()
    for p in (Path(__file__), H/'PROTOCOL.txt', Path(b.__file__), Path(a.__file__),
              Path(a.g.__file__), Path(evo_geometry.__file__), *map(Path, SOURCES)):
        a.pin(p)


def ate(ep, gp):
    error = a.best_fit(ep, gp)
    value = rms(np.linalg.norm(error, axis=1))
    rotation, translation, scale = umeyama_alignment(ep.T, gp.T, with_scale=False)
    assert scale == 1.0
    independent = rms(np.linalg.norm(ep@rotation.T+translation-gp, axis=1))
    delta = abs(value-independent)
    assert delta < 1e-9, delta
    return value, error, delta


def pieces(valid):
    ii = np.flatnonzero(valid)
    assert len(ii)>100
    return np.split(ii, np.flatnonzero(np.diff(ii)>1)+1)


def length(p, valid):
    return float(sum(np.linalg.norm(np.diff(p[ii], axis=0), axis=1).sum() for ii in pieces(valid)))


def coverage(grid, valid, epoch, duration):
    pp = pieces(valid)
    first, last = int(pp[0][0]), int(pp[-1][-1])
    support = sum((len(x)-1)*.1 for x in pp)
    return dict(grid_queries=len(grid), valid_queries=int(valid.sum()),
                excluded_queries=int((~valid).sum()), supported_s=support,
                capture_s=duration, supported_capture_percent=100*support/duration,
                first_s=float((grid[first]-epoch)*1e-9), last_s=float((grid[last]-epoch)*1e-9),
                envelope_s=float((grid[last]-grid[first])*1e-9),
                start_gap_s=float((grid[first]-epoch)*1e-9),
                tail_gap_s=float(duration-(grid[last]-epoch)*1e-9),
                segments=[dict(first_s=float((grid[x[0]]-epoch)*1e-9),
                               last_s=float((grid[x[-1]]-epoch)*1e-9), rows=len(x)) for x in pp])


def homogeneous(p, r):
    T = np.eye(4); T[:3,:3] = r; T[:3,3] = p
    return T


def metrics(ep, er, gp, gr, valid, grid):
    value, residual, align_error = ate(ep[valid], gp[valid])
    horizons = {}
    direct_error = 0.
    for h in (1,10,20):
        ii = a.intervals(valid, h*10); jj = ii+h*10
        assert len(ii)>10 and np.all(grid[jj]-grid[ii] == h*1_000_000_000)
        dt, dr, _ = a.rpe(ep, er, gp, gr, ii, jj)
        # Independent matrix definition: error = inverse(G_i^-1 G_j) (E_i^-1 E_j).
        for k in np.linspace(0, len(ii)-1, 21, dtype=int):
            i,j = ii[k],jj[k]
            dE = np.linalg.inv(homogeneous(ep[i],er[i]))@homogeneous(ep[j],er[j])
            dG = np.linalg.inv(homogeneous(gp[i],gr[i]))@homogeneous(gp[j],gr[j])
            error = np.linalg.inv(dG)@dE
            direct_error = max(direct_error,
                abs(np.linalg.norm(error[:3,3])-np.linalg.norm(dt[k])),
                abs(np.linalg.norm(R.from_matrix(error[:3,:3]).as_rotvec())*180/np.pi-np.linalg.norm(dr[k])))
        horizons[str(h)] = dict(pairs=len(ii), translation_RMSE_m=rms(np.linalg.norm(dt,axis=1)),
                                rotation_RMSE_deg=rms(np.linalg.norm(dr,axis=1)))
    assert direct_error < 1e-7, direct_error
    return dict(ATE_RMSE_m=value, path_length_m=length(ep,valid),
                reference_path_length_m=length(gp,valid), horizons=horizons,
                evo_alignment_difference_m=align_error, direct_RPE_check_max=direct_error), residual


def check_csv(path):
    with path.open() as f:
        rows = list(csv.DictReader(f))
    ts = np.array([int(r['field.header.stamp']) for r in rows], np.int64)
    qs = np.array([[float(r['field.pose.pose.orientation.'+k]) for k in 'xyzw'] for r in rows])
    frames = sorted({r['field.header.frame_id'] for r in rows})
    children = sorted({r['field.child_frame_id'] for r in rows})
    assert frames == ['map'] and children == ['cpt7_imu'], (path,frames,children)
    assert np.all(np.diff(ts)>0) and np.isfinite(qs).all()
    norm_error = float(np.max(np.abs(np.linalg.norm(qs,axis=1)-1)))
    assert norm_error < 1e-6
    return dict(rows=len(rows), duplicate_or_nonmonotonic_stamps=0,
                frames=frames, children=children, quaternion_norm_max_error=norm_error,
                max_stamp_gap_s=float(np.diff(ts).max()*1e-9))


def analyze_case(row):
    case,sensor,seq,directory,filename,duration,epoch = row
    dest = OUT/(case+'.json')
    assert not dest.exists() and not (OUT/(case+'.npz')).exists(), 'Refuse case overwrite'
    start_hashes()
    old,ev,ref = b.load(row)
    reference = ref.times, np.array(ref.poses)[:,:3,3], np.array(ref.poses)[:,:3,:3]
    curves = {}; paths = {}; inputs = {}; full = {}; original_checks = {}
    for mode in ('off','on'):
        src = b.local(ev['results'][mode+'_native_glim']['path']).parent
        runner = a.pin(src/'experiment_runner_used.sh')
        assert 'rostopic echo -p /glim/cbs/odometry' in runner.read_text()
        modecheck = a.pin(src/'effective_mode_check.json')
        assert '--no_incremental_pose=false' in modecheck.read_text()
        a.pin(src.parent/'protected_manifest.json')
        a.pin(src.parent/'evaluate_fl4f.py')
        for role in (*ROLES,'glim_refined'):
            key = mode+'_'+role
            if role=='glim_initial':
                path = src/'glim_odometry.csv'
            else:
                record = ev['results'][mode+'_'+OLD_ROLES[role]]
                path = b.local(record['path'])
                assert a.g.sha(a.pin(path)) == record['sha256']
            c = a.load_curve(path); curves[key]=c; paths[key]=str(path)
            inputs[key]=dict(path=str(path),sha256=a.g.sha(path),rows=len(c[0]),
                first_s=float((c[0][0]-epoch)*1e-9),last_s=float((c[0][-1]-epoch)*1e-9),
                span_s=float((c[0][-1]-c[0][0])*1e-9))
            if role=='glim_initial': inputs[key]['csv_gate']=check_csv(path)
            gp,gr,ok = a.sample(reference,c[0],50_000_000)
            assert ok.all(), (key,'saved pose without reference bracket')
            ep,er = a.prism(c[1],c[2]); gp,gr = a.prism(gp,gr)
            av,_,check = ate(ep,gp)
            full[key]=dict(ATE_RMSE_m=av,rows=len(c[0]),reference_supported_rows=int(ok.sum()),
                span_s=inputs[key]['span_s'],first_s=inputs[key]['first_s'],last_s=inputs[key]['last_s'],
                native_polyline_length_m=float(np.linalg.norm(np.diff(ep,axis=0),axis=1).sum()),
                reference_native_polyline_length_m=float(np.linalg.norm(np.diff(gp,axis=0),axis=1).sum()),
                gap_chord_count=int(np.sum(np.diff(c[0])>500_000_000)),evo_alignment_difference_m=check)
            if role in OLD_ROLES:
                oldscore=ev['results'][mode+'_'+OLD_ROLES[role]]['full_saved']
                delta=abs(av-oldscore['ATE_RMSE_m'])
                assert delta < 1e-5, (case,key,delta)
                original_checks[key]=dict(ATE_difference_m=delta, historical_ATE_m=oldscore['ATE_RMSE_m'])
    primary = [m+'_'+r for m in ('off','on') for r in ROLES]
    start=max(curves[k][0][0] for k in primary); end=min(curves[k][0][-1] for k in primary)
    grid=np.arange(((start+a.STEP-1)//a.STEP)*a.STEP,(end//a.STEP)*a.STEP+1,a.STEP,dtype=np.int64)
    gp,gr,gtok=a.sample(reference,grid,50_000_000);gp,gr=a.prism(gp,gr)
    valid=gtok.copy(); sampled={}; supported={}
    for key,c in curves.items():
        p,r,ok=a.sample(c,grid,500_000_000);sampled[key]=a.prism(p,r);supported[key]=ok
        if key in primary:valid &= ok
    sensitivity=valid & supported['off_glim_refined'] & supported['on_glim_refined']
    result={}; maturity={}; arrays=dict(timestamp_ns=grid,time_s=(grid-epoch)*1e-9,
        common_valid=valid,maturity_valid=sensitivity,reference_position=gp,reference_rotation=gr)
    for key,(ep,er) in sampled.items():
        if key in primary:
            result[key], residual=metrics(ep,er,gp,gr,valid,grid)
            error=np.full(len(grid),np.nan);error[valid]=np.linalg.norm(residual,axis=1)
            arrays[key+'_aligned_error_m']=error
        maturity[key],_=metrics(ep,er,gp,gr,sensitivity,grid)
        arrays[key+'_position']=ep;arrays[key+'_rotation']=er
    maturity_effect={}
    for mode in ('off','on'):
        initial=maturity[mode+'_glim_initial'];refined=maturity[mode+'_glim_refined']
        maturity_effect[mode]=dict(initial_ATE_m=initial['ATE_RMSE_m'],refined_ATE_m=refined['ATE_RMSE_m'],
            refinement_ATE_improvement_percent=improve(initial['ATE_RMSE_m'],refined['ATE_RMSE_m']),
            initial_minus_refined_ATE_m=initial['ATE_RMSE_m']-refined['ATE_RMSE_m'],
            initial_path_m=initial['path_length_m'],refined_path_m=refined['path_length_m'],
            initial_RPE1=initial['horizons']['1'],refined_RPE1=refined['horizons']['1'])
    off_on={r:dict(ATE_improvement_percent=improve(result['off_'+r]['ATE_RMSE_m'],result['on_'+r]['ATE_RMSE_m']),
                  translation_RPE1_improvement_percent=improve(result['off_'+r]['horizons']['1']['translation_RMSE_m'],result['on_'+r]['horizons']['1']['translation_RMSE_m']),
                  rotation_RPE1_improvement_percent=improve(result['off_'+r]['horizons']['1']['rotation_RMSE_deg'],result['on_'+r]['horizons']['1']['rotation_RMSE_deg'])) for r in ROLES}
    data=dict(case=case,sensor=sensor,sequence=seq,epoch_ns=epoch,capture_s=duration,
        reference=ev['reference'],input_products=inputs,full_saved=full,
        common_coverage=coverage(grid,valid,epoch,duration),common=result,
        maturity_coverage=coverage(grid,sensitivity,epoch,duration),maturity=maturity,
        maturity_effect=maturity_effect,off_on=off_on,original_ATE_reproduction=original_checks,
        original_report=str(E/directory/filename),original_terminal_omissions=old.get('terminal_omissions'),
        synthetic_tests=a.tests(),mask_is_identical_with_refined=bool(np.array_equal(valid,sensitivity)))
    assert all(a.g.sha(Path(p))==sha for p,sha in a.HASHES.items())
    data['input_sha256']=dict(a.HASHES);data['input_hashes_unchanged']=True
    OUT.mkdir(exist_ok=True)
    np.savez_compressed(OUT/(case+'.npz'),**arrays)
    a.save(dest,data)
    print(json.dumps(dict(case=case,common={k:round(v['ATE_RMSE_m'],6) for k,v in result.items()},
                         maturity_effect=maturity_effect,coverage=data['common_coverage']['supported_s'])),flush=True)


def assemble():
    dest=OUT/'summary.json';assert not dest.exists()
    start_hashes();cases={}
    for row in b.ROWS:
        p=a.pin(OUT/(row[0]+'.json'));data=json.loads(p.read_text())
        for path,sha in data['input_sha256'].items():
            assert a.g.sha(Path(path))==sha, path
            a.HASHES[path]=sha
        a.pin(OUT/(row[0]+'.npz'))
        cases[row[0]]=data
    identity={s:cases['hesai_'+s]['input_products']['off_kimera']['sha256']==
                cases['velodyne_'+s]['input_products']['off_kimera']['sha256'] for s in ('spx3','eig2','leica2')}
    assert all(identity.values())
    changes=[v['refinement_ATE_improvement_percent'] for d in cases.values() for v in d['maturity_effect'].values()]
    checks=dict(historical_ATE_values_reproduced=sum(len(d['original_ATE_reproduction']) for d in cases.values()),
        historical_ATE_max_difference_m=max(v['ATE_difference_m'] for d in cases.values() for v in d['original_ATE_reproduction'].values()),
        evo_ATE_max_difference_m=max(v['evo_alignment_difference_m'] for d in cases.values() for block in ('full_saved','common','maturity') for v in d[block].values()),
        direct_RPE_max_difference=max(v['direct_RPE_check_max'] for d in cases.values() for block in ('common','maturity') for v in d[block].values()),
        live_glim_files=14,kimera_off_identical_across_sensors=identity,
        all_primary_masks_unchanged_by_refined=all(d['mask_is_identical_with_refined'] for d in cases.values()),
        refinement_improves_ATE_cases=sum(x>0 for x in changes),refinement_worsens_ATE_cases=sum(x<0 for x in changes),
        refinement_ATE_improvement_range_percent=[min(changes),max(changes)])
    a.save(dest,dict(cases=cases,verification=checks,input_sha256=dict(a.HASHES),input_hashes_unchanged=True))
    print(json.dumps(checks,indent=2),flush=True)


if __name__=='__main__':
    if sys.argv[1]=='assemble':assemble()
    else:
        selected=[row for row in b.ROWS if sys.argv[1] in ('all',row[0])]
        assert selected, sys.argv[1]
        for row in selected:analyze_case(row)
