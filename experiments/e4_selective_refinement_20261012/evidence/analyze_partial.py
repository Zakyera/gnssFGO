#!/usr/bin/env python3
"""Post-stop evidence; never complete or repair the intentionally stopped run."""
import collections,csv,io,json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from analyze import ROOT,CAPTURE,stats
incomplete={}
def read(p):
    raw=p.read_text();lines=raw.splitlines(keepends=True)
    incomplete[str(p)]=bool(lines and not lines[-1].endswith('\n'))
    if incomplete[str(p)]:lines.pop()
    return list(csv.DictReader(io.StringIO(''.join(lines))))
a,b=ROOT/'reference',ROOT/'candidate'
x,y=read(a/'online_states.csv'),read(b/'online_states.csv')
assert [(r['state_index'],r['timestamp_ns']) for r in y]==[(r['state_index'],r['timestamp_ns']) for r in x[:len(y)]]
with (CAPTURE/'live160_on_receiver/events.csv').open() as f:
    begin=next(int(line.split(',')[2]) for line in f if line.startswith('I,'))
result=dict(status='STOPPED_NUMERICAL_EQUIVALENCE_FAILURE',full160_candidate_completed=False,
            complete_online_rows=len(y),differences={})
for label,columns,limit in [('position_m',['px','py','pz'],1e-7),('velocity_m_s',['vx','vy','vz'],1e-7),('omega_rad_s',['wx','wy','wz'],1e-7)]:
    d=np.array([[float(r[k])-float(s[k]) for k in columns] for r,s in zip(x,y)])
    n=np.linalg.norm(d,axis=1);entry=stats(n);entry['limit']=limit
    failures=np.flatnonzero(n>limit)
    if len(failures):
        i=int(failures[0]);entry['first_failure']=dict(row=i,state_index=y[i]['state_index'],timestamp_ns=y[i]['timestamp_ns'],bag_seconds=(int(y[i]['timestamp_ns'])-begin)*1e-9,difference=float(n[i]))
    result['differences'][label]=entry
qx=np.array([[float(r[k]) for k in ['qx','qy','qz','qw']] for r in x[:len(y)]])
qy=np.array([[float(r[k]) for k in ['qx','qy','qz','qw']] for r in y])
n=(Rotation.from_quat(qx).inv()*Rotation.from_quat(qy)).magnitude()
result['differences']['orientation_rad']=stats(n)|dict(limit=1e-8)
result['online_rows_byte_identical']=x[:len(y)]==y
cx,cy=read(a/'publication_convergence.csv'),read(b/'publication_convergence.csv')
keys=lambda rows:[(r['phase'],r['index'],r['physical_ns']) for r in rows]
assert keys(cx[:len(cy)])==keys(cy)
matched=cx[:len(cy)];end=(int(cy[-1]['physical_ns'])-begin)*1e-9
result['last_completed_publication_bag_seconds']=end
result['completed_publications']=len(cy)
cost=np.array([abs(float(r['error_after'])-float(s['error_after']))/max(1,abs(float(r['error_after']))) for r,s in zip(matched,cy)])
result['objective_relative_difference']=stats(cost)|dict(limit=1e-8)
result['stationarity_all_pass']=all(r['status']=='stationary' and float(r['gradient_after'])<=1e-5 for r in cy)
result['max_gradient']=max(float(r['gradient_after']) for r in cy)
result['max_attempts']=max(int(r['attempted']) for r in cy)
dd=read(b/'e4_refinement.csv')[:len(cy)]
result['branches']=dict(collections.Counter(r['branch'] for r in dd))
nst=[r for r in dd if r['branch']!='stationary']
result['fallback_fraction_nonstationary']=sum(r['branch']!='selective_accepted' for r in nst)/len(nst)
result['timing']={}
for lo,hi in [(0,40),(40,80),(80,120),(120,end+1e-8),(0,end+1e-8)]:
    indices=[i for i,r in enumerate(cy) if lo<=(int(r['physical_ns'])-begin)*1e-9<hi]
    record={}
    for tag,rr in [('reference',matched),('candidate',cy)]:
        entry=stats([rr[i]['seconds'] for i in indices]);entry['sum']=sum(float(rr[i]['seconds']) for i in indices)
        record[tag]=entry
    for col in ['verification_s','selection_s','copy_update_s','fallback_s']:
        record[col]=stats([dd[i][col] for i in indices]);record[col]['sum']=sum(float(dd[i][col]) for i in indices)
    subset=[dd[i] for i in indices if int(dd[i]['selected'])]
    record['selected_fraction']=stats([int(r['selected'])/int(r['variables']) for r in subset])
    attempted=[r for r in subset if r['branch'].startswith('selective_')]
    record['reported_relinearized_fraction']=stats([int(r['relinearized'])/int(r['variables']) for r in attempted])
    result['timing'][f'{lo}-{min(hi,end):.6f}']=record
result['exact_prefix_checks']={}
for name in ['causal_dispatch.csv','support_clock.csv']:
    r,s=read(a/name),read(b/name)
    result['exact_prefix_checks'][name]=dict(rows=len(s),equal=s==r[:len(s)])
r,s=read(a/'revision_events.csv'),read(b/'revision_events.csv')
columns=['ordinal','from_index','to_index','revision','record_timestamp_ns','outcome','old_factor_index','new_factor_index','active_factors']
result['exact_prefix_checks']['revision_events.csv']=dict(rows=len(s),equal=all([row[k] for row in s]==[row[k] for row in r[:len(s)]] for k in columns),outcomes=dict(collections.Counter(row['outcome'] for row in s)))
result['incomplete_trailing_lines_excluded']=incomplete
result['not_done']=['candidate final retrospective trajectory','full160 candidate timing','real posterior covariance comparison','deterministic candidate replay','live capacity test']
with (ROOT/'partial_comparison.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
