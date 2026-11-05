"""Saved-only EVO Method1 scoring; conditional remove/retain, no estimator calls."""
import csv,importlib.metadata,importlib.util,json,sys
from pathlib import Path
sys.dont_write_bytecode=True
import numpy as np
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent;E=H.parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name/'replay'
spec=importlib.util.spec_from_file_location('cp69_evo_helpers',E/'eig2_hesai_refresh_full_cpv49_20261030/evaluate.py')
u=importlib.util.module_from_spec(spec);spec.loader.exec_module(u)
m=u.m;a=u.a;epoch=u.epoch;O=H/'evaluation'
def save(p,v):u.save(p,v)
def stats(x):return u.d39.stats(x)
assert json.loads((H/'replay_attempt/passed.json').read_text())['passed']
assert importlib.metadata.version('evo')=='1.36.3' and not O.exists()
O.mkdir();a.HASHES.clear();u.pin(__file__);u.pin(H/'PROTOCOL.txt')
for mod in (u,m,m.c,m.c.b,a,a.g,u.u,u.d39,u.d39.v):u.pin(mod.__file__)
import evo
for f in Path(evo.__file__).parent.rglob('*.py'):u.pin(f)
for kind in ('ape','rpe'):u.pin(Path('/home/yeranis/.local/bin')/('evo_'+kind))
curves={};diagnostics={};exact_checks={}
record=next(r for r in m.c.b.ROWS if r[0]=='hesai_eig2')
_,_,ref=m.c.b.load(record)
reference=(ref.times,np.asarray(ref.poses)[:,:3,3],np.asarray(ref.poses)[:,:3,:3])
for mode in ('remove','retain'):
    src=P/(mode+'_capture');validation=u.read(src/'validation.json');assert validation['valid']
    absolute,_,checks=m.kimera(src);curves[mode]=absolute;exact_checks[mode]=checks
    rows=list(csv.DictReader((src/'kimera_odometry.csv').open()))
    native=(absolute[0],np.array([[float(r['field.pose.pose.position.'+k]) for k in 'xyz'] for r in rows]),
        Rotation.from_quat([[float(r['field.pose.pose.orientation.'+k]) for k in 'xyzw'] for r in rows]).as_matrix())
    summary,events=u.d39.window_summary(src/'estimators.log',absolute[0],native,absolute)
    save(O/(mode+'_revision_events.json'),events)
    late=[e for e in events if 40<=e['time_s']<=60]
    final=[e for e in events if 295.02<=e['time_s']<=355.02]
    cost=np.loadtxt(u.pin(src/'objective.csv'),delimiter=',');assert np.isfinite(cost).all()
    absorptions=[list(map(float,line.split('CPV69_ABSORPTION_ROW,',1)[1].split(',')))
        for line in (src/'estimators.log').read_text().splitlines() if 'CPV69_ABSORPTION_ROW,' in line]
    if mode=='retain':
        assert len(absorptions)==int(np.sum(cost[:,2]>0))
        assert sum(x[1] for x in absorptions)==int(cost[:,2].sum())
        expected=cost[cost[:,2]>0][:,[0,2]]
        assert np.array_equal(np.asarray(absorptions)[:,:2],expected)
        assert all(x[2]>=x[1] for x in absorptions)
    else:assert not absorptions
    timing=[]
    for line in (src/'estimators.log').read_text().splitlines():
        if 'KIMERA_OPTIMIZE_TIMING_ROW,' in line:
            timing.append([float(x) for x in m.ANSI.sub('',line.split('KIMERA_OPTIMIZE_TIMING_ROW,',1)[1]).strip().split(',')])
    tt=np.asarray(timing);assert np.isfinite(tt).all()
    diagnostics[mode]=dict(validation=validation,whole_revisions=summary,
        late_40_60s={k:stats([e[k] for e in late]) for k in ('previous_revision_m','same_update_motion_m','absolute_step_m','revision_rotation_deg')},
        final_60s={k:stats([e[k] for e in final]) for k in ('previous_revision_m','same_update_motion_m','absolute_step_m','revision_rotation_deg')},
        cost=dict(rows=len(cost),total_after=stats(cost[:,5]),imu_after=stats(cost[:,7]),retirements=int(cost[:,2].sum())),
        absorption_boundaries=len(absorptions),
        timing=dict(optimize_ms=stats(tt[:,1]),smoother_ms=stats(tt[:,5])))
start=max(c[0][0] for c in curves.values());end=min(c[0][-1] for c in curves.values())
grid=np.arange(((start+a.STEP-1)//a.STEP)*a.STEP,(end//a.STEP)*a.STEP+1,a.STEP,dtype=np.int64)
gp,gr,valid=a.sample(reference,grid,50_000_000);gp,gr=a.prism(gp,gr);sampled={}
for name,curve in curves.items():
    ep,er,good=a.sample(curve,grid,500_000_000);sampled[name]=a.prism(ep,er);valid &= good
ids=np.flatnonzero(valid);segments=np.split(ids,np.flatnonzero(np.diff(ids)>1)+1)
common=O/'common';common.mkdir();rp=common/'reference.tum';rt=u.export(rp,u.curve(grid[ids],gp[ids],gr[ids]))
refs,pairs,info=u.reference_segments(common,grid,gp,gr,segments)
results={};full={}
for name,(ep,er) in sampled.items():
    results[name]=u.score_series(common,name,grid,ep,er,gp,gr,segments,refs,rp,rt)
    ts,p,r=curves[name];fp,fr=a.prism(p,r);fg,fgr,good=a.sample(reference,ts,50_000_000)
    assert good.all();fg,fgr=a.prism(fg,fgr);folder=O/('full_'+name);folder.mkdir()
    cuts=np.split(np.arange(len(ts)),np.flatnonzero(np.diff(ts)>500_000_000)+1)
    rpf=folder/'reference.tum';rtf=u.export(rpf,u.curve(ts,fg,fgr));rfs,prs,inf=u.reference_segments(folder,ts,fg,fgr,cuts)
    full[name]=u.score_series(folder,name,ts,fp,fr,fg,fgr,cuts,rfs,rpf,rtf)
    full[name].update(first_s=float((ts[0]-epoch)*1e-9),last_s=float((ts[-1]-epoch)*1e-9),span_s=float((ts[-1]-ts[0])*1e-9),gaps=len(cuts)-1,
        coverage_supported_s=sum(float((ts[x[-1]]-ts[x[0]])*1e-9) for x in cuts))
for p,v in a.HASHES.items():u.PINS[p]=v
assert all(u.sha(p)==v for p,v in u.PINS.items())
result=dict(passed=True,task='CP-V71',scope='conditional Kimera-only355.02s remove/retain; saved CP49 ON history held fixed; recorded-update delivery',
    method='Method1 first absolute backend publication, not ROS incremental or delayed finalized',evo_version=importlib.metadata.version('evo'),
    reference='IE-TC shares CPT7; not independent prism GT',common=results,full_saved=full,
    common_coverage=m.c.coverage(grid,valid,epoch,355.02),reference_common_path_m=sum(x[0].path_length for x in refs),
    RPE_pairs=len(pairs),segments=info,diagnostics=diagnostics,kimera_absolute_checks=exact_checks,verification=u.checks,
    ATE_improvement_percent=100*(results['remove']['ATE_RMSE_m']-results['retain']['ATE_RMSE_m'])/results['remove']['ATE_RMSE_m'],
    live_bidirectional_provenance_gate_passed=False)
save(H/'results.json',result);save(O/'commands.json',u.commands);save(O/'input_sha256.json',u.PINS)
print(json.dumps({k:result[k] for k in ('common','reference_common_path_m','ATE_improvement_percent')},indent=2))
