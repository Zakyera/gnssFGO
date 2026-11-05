"""Read actual saved data contracts and smoke-test EVO on synthetic data only."""
import json,importlib.metadata
import numpy as np
from pathlib import Path
from paths import H,E,W,P,BASE_H,BASE_P
import evaluate as q
d=q.d;a=q.a;b=q.b
assert importlib.metadata.version('evo')=='1.36.3'
assert json.loads((BASE_H/'complete.json').read_text())['passed']
record=next(r for r in d.m.c.b.ROWS if r[0]=='hesai_eth1')
_,_,ref=d.m.c.b.load(record)
reference=q.physical((ref.times,np.asarray(ref.poses)[:,:3,3],np.asarray(ref.poses)[:,:3,:3]),'cpt7')
curves={};details={}
saved_on=W/'runs/eth1_split_imu_persistent_on_cpv91_20261102_attempt1/prepared/on_source'
for name,S in [('saved_original_off',BASE_P/'full_off_source'),('historical_on',saved_on)]:
    native=b.kimera(S,'alphasense')
    curves[name+'_kimera']=q.physical(native,'alphasense')
    curves[name+'_glim']=q.physical(a.load_curve(S/'glim_odometry.csv'),'cpt7')
    revisions,events=d.d39.window_summary(S/'estimators.log',native[0],b.native_csv(S/'kimera_odometry.csv'),native)
    times=[];cov=[]
    for line in (S/'estimators.log').open():
        for marker,out in [('KIMERA_OPTIMIZE_TIMING_ROW,',times),('KIMERA_CBS_REVISION_EDGE_ROW,',cov)]:
            if marker in line:out.append(d.m.ANSI.sub('',line.split(marker,1)[1]).strip().split(','))
    tm=np.asarray(times,float);C=np.asarray([x[28:] for x in cov],float).reshape(-1,6,6)
    assert tm.ndim==2 and len(tm)>100 and np.isfinite(tm).all()
    assert np.isfinite(C).all() and np.linalg.eigvalsh(C).min()>0
    details[name]=dict(native_rows=len(native[0]),revisions=revisions,covariance_rows=len(C))
for role,file in [('online','online_trajectory.tum'),('delayed','finalized_trajectory.tum')]:
    curves['saved_original_off_fusion_'+role]=q.physical(a.load_curve(BASE_P/'full_off_receiver/receiver'/file),'cpt7')
for name,(ts,pp,rr) in curves.items():
    _,_,good=a.sample(reference,ts,50_000_000)
    assert good.all() and len(ts)>100,name
    assert np.isfinite(pp).all() and np.isfinite(rr).all()
primary=[key for key in curves if not key.endswith('delayed')]
first=max(curves[k][0][0] for k in primary);last=min(curves[k][0][-1] for k in primary)
grid=np.arange(((int(first)+99_999_999)//100_000_000)*100_000_000,int(last)+1,100_000_000,dtype=np.int64)
_,_,valid=a.sample(reference,grid,50_000_000)
for k in primary:valid &= a.sample(curves[k],grid,500_000_000)[2]
assert valid.sum()>100
cov=d.covariance_summary(BASE_P/'full_off_receiver/events.csv')

# Exercise the exact evaluator's file export, CLI/API equality and pooled
# archive paths with a non-degenerate synthetic curve, not another run score.
O=H/'report_contract_test';O.mkdir()
ts=q.epoch+np.arange(601,dtype=np.int64)*100_000_000
t=np.linspace(0,1,len(ts));xyz=np.column_stack([210*t,3*np.sin(6*t),2*np.cos(5*t)])
rot=np.tile(np.eye(3),(len(ts),1,1));cuts=[np.arange(0,300),np.arange(305,601)]
rp,rt,refs,pairs,info=q.reference_files(O/'fixture',ts,xyz,rot,cuts)
row=d.score_series(O/'fixture','synthetic_contract_only',ts,xyz,rot,xyz,rot,cuts,refs,rp,rt)
assert row['ATE_RMSE_m']<1e-8 and len(pairs)>100
for kind in ('translation_m','rotation_deg'):
    result=d.file_interface.load_res_file(row['RPE'][kind]['archive'])
    assert result.stats['rmse']<1e-8 and len(result.np_arrays['error_array'])==len(pairs)
assert all(c['exit']==0 for c in d.commands)
d.save(O/'commands.json',d.commands)
d.PINS.update(a.HASHES)
d.save(O/'input_sha256.json',d.PINS)
d.save(H/'report_preflight.json',dict(passed=True,actual_saved_contracts=details,
    curves_checked=list(curves),reference_rows=len(reference[0]),contract_grid_valid=int(valid.sum()),
    transported_covariance=cov,synthetic_EVO_CLI_calls=len(d.commands),checks=d.checks,
    actual_new_ON_not_yet_available=True,new_scientific_results=False,source_runs=0,receiver_replays=0))
print('PASS: actual saved parser/reference/diagnostic contracts and synthetic EVO end-to-end')
