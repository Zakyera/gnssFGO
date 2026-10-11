#!/usr/bin/env python3
"""Read-only comparisons; write a new immutable result, including failures."""
import csv,hashlib,json,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parent
CAPTURE=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_e1_e2_live160_20261011/prepared')
sys.path.insert(0,str(CAPTURE))
from numerical_equivalence import compare
def rows(p):
    with p.open() as f:return list(csv.DictReader(f))
def stats(x):
    x=np.asarray(x,dtype=float)
    return dict(n=len(x),mean=float(x.mean()),median=float(np.median(x)),p95=float(np.quantile(x,.95)),max=float(x.max())) if len(x) else {}
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1<<20),b''):h.update(b)
    return h.hexdigest()
def compare_all(a,b):
    result={}
    try:result['frozen_validator']=compare(a,b)
    except Exception as e:result['frozen_validator']=dict(passed=False,error=repr(e))
    result['state_differences']={}
    for name in ['online_states.csv','states.csv']:
        x,y=rows(a/name),rows(b/name)
        if len(x)!=len(y):result['state_differences'][name]={'lengths':[len(x),len(y)]};continue
        record={}
        for key,columns in [('position_m',['px','py','pz']),('velocity_m_s',['vx','vy','vz']),('omega_rad_s',['wx','wy','wz'])]:
            diff=np.array([[float(r[c])-float(s[c]) for c in columns] for r,s in zip(x,y)])
            norms=np.linalg.norm(diff,axis=1);record[key]=stats(norms)
            if (norms>1e-7).any():record[key]['first_failure']=int(np.flatnonzero(norms>1e-7)[0])
        q=lambda rr:np.array([[float(r[c]) for c in ['qx','qy','qz','qw']] for r in rr])
        rot=(Rotation.from_quat(q(x)).inv()*Rotation.from_quat(q(y))).magnitude()
        record['orientation_rad']=stats(rot)
        record['byte_identical']=(a/name).read_bytes()==(b/name).read_bytes()
        result['state_differences'][name]=record
    x,y=rows(a/'publication_convergence.csv'),rows(b/'publication_convergence.csv')
    assert [(r['phase'],r['index'],r['physical_ns']) for r in x]==[(r['phase'],r['index'],r['physical_ns']) for r in y]
    cost=np.array([abs(float(r['error_after'])-float(s['error_after']))/max(1,abs(float(r['error_after']))) for r,s in zip(x,y)])
    result['objective_relative_difference']=stats(cost)
    result['passed']=result['frozen_validator']['passed'] and bool(np.isfinite(cost).all() and max(cost)<=1e-8)
    return result
def timing(folder):
    rr=rows(folder/'publication_convergence.csv');dd=rows(folder/'e4_refinement.csv')
    begin=int(rr[0]['physical_ns'])-800000000 # Only labels below use actual event start instead.
    with (CAPTURE/'live160_on_receiver/events.csv').open() as f:
        for line in f:
            if line.startswith('I,'):begin=int(line.split(',')[2]);break
    result={}
    for lo,hi in [(0,40),(40,80),(80,120),(120,160)]:
        group=[r for r in rr if lo<=(int(r['physical_ns'])-begin)*1e-9<hi]
        diag=[r for r in dd if lo<=(int(r['physical_ns'])-begin)*1e-9<hi]
        from collections import Counter
        entry=dict(publication_s=stats([r['seconds'] for r in group]),branches=dict(Counter(r['branch'] for r in diag)))
        for col in ['verification_s','selection_s','copy_update_s','fallback_s']:
            entry[col]=stats([r[col] for r in diag])
            entry[col]['sum']=sum(float(r[col]) for r in diag)
        entry['selected_fraction']=stats([int(r['selected'])/int(r['variables']) for r in diag if int(r['variables'])])
        result[f'{lo}-{hi}']=entry
    from collections import Counter
    result['branches']=dict(Counter(r['branch'] for r in dd))
    nonstationary=[r for r in dd if r['branch']!='stationary']
    result['fallback_fraction_nonstationary']=sum(r['branch']!='selective_accepted' for r in nonstationary)/max(1,len(nonstationary))
    result['total_publication_s']=sum(float(r['seconds']) for r in rr)
    result['max_gradient']=max(float(r['gradient_after']) for r in rr)
    result['attempts_max']=max(int(r['attempted']) for r in rr)
    return result
if __name__=='__main__':
    action=sys.argv[1];target=ROOT/(action+'.json');assert not target.exists()
    if action=='reference_check':result=compare_all(CAPTURE/'live160_on_receiver/receiver',ROOT/'reference')
    elif action=='candidate_check':
        result=compare_all(ROOT/'reference',ROOT/'candidate')
        result['timing']={tag:timing(ROOT/tag) for tag in ['reference','candidate']}
        wall={t:json.loads((ROOT/(t+'.json')).read_text())['seconds'] for t in ['reference','candidate']}
        result['wall_seconds']=wall
        result['wall_gain']=1-wall['candidate']/wall['reference']
        result['late_p95_gain']=1-result['timing']['candidate']['120-160']['publication_s']['p95']/result['timing']['reference']['120-160']['publication_s']['p95']
        result['performance_pass']=result['wall_gain']>=.2 and result['late_p95_gain']>=.2 and result['timing']['candidate']['fallback_fraction_nonstationary']<.5
    elif action=='repeat_check':result=compare_all(ROOT/'candidate',ROOT/'candidate_repeat')
    elif action=='preservation_check':
        old=json.loads((ROOT/'protected.json').read_text());wrong=[p for p,h in old.items() if sha(Path(p))!=h]
        result=dict(passed=not wrong,files=len(old),mismatches=wrong)
    else:raise ValueError(action)
    with target.open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result,indent=2))
    sys.exit(0 if result['passed'] else 1)
