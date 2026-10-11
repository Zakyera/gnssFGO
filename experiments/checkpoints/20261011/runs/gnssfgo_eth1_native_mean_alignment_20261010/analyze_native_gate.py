"""Diagnose isolated native Kimera controls; no estimator/evaluator changes."""
from pathlib import Path
import json
import sys
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path(__file__).resolve().parent
AUDIT = ROOT.parent/'gnssfgo_eth1_active_window40_off_20261008/insertion_retirement_audit_20261008'
sys.path.insert(0,str(AUDIT))
from analyze import parse_snapshots, parse_lifecycle, pose_error, summary
from eig2_single_source_preservation import exp_se3, log_se3, inverse

valid = {c:json.loads((ROOT/f'native8_{c}/validation.json').read_text())
         for c in ['no_cbs','raw','aligned']}
assert valid['raw']['input_hashes']==valid['aligned']['input_hashes']==valid['no_cbs']['input_hashes']
assert valid['raw']['matched_edges']==valid['aligned']['matched_edges']
ref=np.load(AUDIT/'reference.npz')
rt=ref['times']; rp=ref['positions']; rq=ref['quaternions']; t0=rt[0]
slerp=Slerp(rt-t0,Rotation.from_quat(rq))
def truth(t):
    j=np.searchsorted(rt,t)
    assert 0<j<len(rt) and rt[j]-rt[j-1]<.006
    alpha=(t-rt[j-1])/(rt[j]-rt[j-1]); T=np.eye(4)
    T[:3,3]=(1-alpha)*rp[j-1]+alpha*rp[j]
    T[:3,:3]=slerp([t-t0]).as_matrix()[0]
    return T
events=[]
for row in valid['aligned']['alignment_rows']:
    gt=inverse(truth(row['c']*1e-9))@truth(row['d']*1e-9)
    raw=pose_error(exp_se3(np.array(row['raw'])),gt)
    aligned=pose_error(exp_se3(np.array(row['mean'])),gt)
    events.append(dict(update=row['update'],raw=raw,aligned=aligned))
curves={}; snapshots={}; lifecycle={}; native={}; closure={}
for c,v in valid.items():
    curve={}
    for row in v['native_poses']:
        T=np.eye(4); T[:3,3]=row[1:4]; T[:3,:3]=Rotation.from_quat(row[4:8]).as_matrix()
        curve[row[8]]=(row[0]*1e-9,T)
    curves[c]=curve
    snapshots[c],_,_,_=parse_snapshots(ROOT/f'native8_{c}/estimators.log')
    lifecycle[c]=parse_lifecycle(ROOT/f'native8_{c}/estimators.log')
    errors=[]; clos=[]
    for u,(t,T) in sorted(curve.items()):
        if u-1 not in curve:continue
        tp,Tp=curve[u-1]
        if t-tp>.5:continue  # never interpolate across native startup gap
        estimate=inverse(Tp)@T
        errors.append(pose_error(estimate,inverse(truth(tp))@truth(t)))
        snap=snapshots[c].get(u,{})
        if u in snap and u-1 in snap:
            clos.append(float(np.abs(log_se3(inverse(inverse(snap[u-1])@snap[u])@estimate)).max()))
    native[c]={k:summary([e[k] for e in errors]) for k in ['t_m','r_deg']}
    closure[c]=dict(count=len(clos),max_log_difference=max(clos))
for c in ['raw','aligned']:
    for u in [0,1,2]:
        assert curves[c][u][0]==curves['no_cbs'][u][0]
        assert np.array_equal(curves[c][u][1],curves['no_cbs'][u][1])
checks=[]
for row in valid['aligned']['alignment_rows']:
    u,i,j=row['update'],row['ri'],row['rj']
    before=inverse(snapshots['aligned'][u-1][i])@snapshots['aligned'][u-1][j]
    residual=log_se3(inverse(exp_se3(np.array(row['mean'])))@before)
    pre=lifecycle['aligned'][3][(u,i,j)]
    checks.append(max(abs(np.linalg.norm(residual)-pre['norm']),
                      abs(np.linalg.norm(residual[:3])-pre['rotation']),
                      abs(np.linalg.norm(residual[3:])-pre['rho'])))
assert max(checks)<1e-7
result=dict(passed=True,identical_sensor_and_wire_inputs=True,same_20_matches=True,
    first_three_native_poses_exact=True,first_external_update=valid['aligned']['alignment_rows'][0]['update'],
    aligned_pre_update_residual_max_error=max(checks),
    native_increment_snapshot_closure=closure,
    measurement_error_on_recipient_interval={mode:{k:summary([e[mode][k] for e in events]) for k in ['t_m','r_deg']}
        for mode in ['raw','aligned']},native_consecutive_interval_errors=native,
    events=events,reference='Continuous IE-TC (shared CPT7; not independent ground truth)',
    scope='8-second isolation gate; no 40-second accuracy claim. Startup gap excluded from relative-interval checks.')
with (ROOT/'native_gate_analysis.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k!='events'},indent=2))
