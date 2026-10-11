"""Bounded read-only evidence extraction; no estimator/scoring experiment."""
import collections
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import numpy as np

ROOT=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_e1_e2_live160_20261011/prepared')
OUT=ROOT.parent.parent/'gnssfgo_e1_e2_live160_validation_20261011'
SOURCE=ROOT/'live160_on_source';REC=ROOT/'live160_on_receiver/receiver'
def rows(p):
    with p.open() as f:return list(csv.DictReader(f))
def stats(a):
    a=np.asarray(a,dtype=float);assert np.isfinite(a).all() and len(a)
    return dict(mean=float(a.mean()),median=float(np.median(a)),p95=float(np.quantile(a,.95)),max=float(a.max()))
conv=rows(REC/'publication_convergence.csv');steps=rows(REC/'publication_steps.csv')
recovery=[r for r in steps if r['recovery']=='1']
rejected=[r for r in steps if r['accepted']=='0']
clock={int(r['index']):int(r['physical_ns']) for r in conv}
t0=1727775008.003475
def event(r):
    return dict(r,bag_seconds=clock[int(r['index'])]*1e-9-t0)
receiver=dict(publications=len(conv),statuses=dict(collections.Counter(r['status'] for r in conv)),
    max_gradient=max(float(r['gradient_after']) for r in conv),steps=len(steps),
    rejected_steps=len(rejected),recovery_steps=len(recovery),
    recovery_publications=len(set(r['index'] for r in recovery)),
    accepted_recoveries=sum(r['accepted']=='1' for r in recovery),
    consensus_recoveries=sum(r['consensus']=='1' for r in recovery),
    local_prediction_rows=sum(r['local_prediction']=='1' for r in steps),
    recovery_cost_change_abs=stats([abs(float(r['stable_cost_delta'])) for r in recovery]),
    recovery_state_step_mixed_units=stats([float(r['state_step_norm']) for r in recovery]),
    first_recovery=event(recovery[0]),
    largest_recovery_step=event(max(recovery,key=lambda r:float(r['state_step_norm']))),
    all_accepted_recoveries_stationary=all(float(r['gradient_after'])<=1e-5 for r in recovery if r['accepted']=='1'),
    all_normal_steps_relinearize_all_values=all(int(r['relinearized'])==3*(int(r['index'])+1) for r in steps if r['recovery']=='0'))
logs=[p for p in (SOURCE/'ros_logs').glob('*/*stdout.log') if p.parent.name!='latest']
klog,=[p for p in logs if 'kimera_vio' in p.name]
glog,=[p for p in logs if 'glim_rosnode' in p.name]
update_times={};aligned={}
for ln,line in enumerate((SOURCE/'estimators.log').open(),1):
    if 'KIMERA_CBS_MEAN_ALIGNMENT_ROW,' in line:
        v=line.split('KIMERA_CBS_MEAN_ALIGNMENT_ROW,',1)[1].split(',')
        update_times[int(v[0])]=int(v[9])*1e-9-t0
        aligned[(int(v[0]),int(v[3]),int(v[4]))]=ln
guards=[];backend=None;forbidden=collections.Counter();pre={}
tokens=['Caught ValuesKeyDoesNotExist','Caught IndeterminantLinearSystemException',
        'Missing validated iSAM2 factor slot','slot did not match; scanning graph',
        'no receiver pose estimate','MEAN_ALIGNMENT_UNAVAILABLE','terminate called']
for path in [klog,glog]:
    for ln,line in enumerate(path.open(),1):
        for token in tokens:
            if token in line:forbidden[token]+=1
        if path!=klog:continue
        if 'CBS_BPSAM_ACTIVE_WINDOW_LIFECYCLE_ROW,k,' in line:
            backend=int(float(line.split('CBS_BPSAM_ACTIVE_WINDOW_LIFECYCLE_ROW,',1)[1].split(',')[1]))
        if 'CBS_ODOM_PREINJECTION_RESIDUAL_ROW,G2K,' in line:
            v=line.split('CBS_ODOM_PREINJECTION_RESIDUAL_ROW,',1)[1].strip().split(',')
            pre[(backend,int(v[3][1:]),int(v[4][1:]))]=dict(line=ln,columns=v)
        if any('action='+x in line for x in ['drop_before_stage','rollback_and_retry_without_staged_cbs','rollback_remove_active_and_retry']):
            m=re.search(r'k,g,x(\d+)->x(\d+)',line);assert m
            key=(backend,*map(int,m.groups()))
            guards.append(dict(update=backend,edge=list(key[1:]),bag_seconds=update_times.get(backend),
                line=ln,text=line.strip(),pre=pre.get(key),alignment_line=aligned.get(key)))
life=json.loads((OUT/'live160_on_lifecycle_validation.json').read_text())
rollbacks=life['kimera']['rollbacks']
source=dict(guard_events=len(guards),first_guard=guards[0],guard_rows=guards,
    rollback_transactions=dict(collections.Counter(r['reason'] for r in rollbacks)),
    undone_constructions=sum(r['undone_insertions'] for r in rollbacks),
    invalid_identity_or_exception_log_counts=dict(forbidden),
    committed_lifecycle={k:{n:v for n,v in vals.items() if n not in ['rollbacks','removals']}
                         for k,vals in life.items() if isinstance(vals,dict)})
cache=rows(REC/'e2_memo.csv')
assert all(int(r['failures'])==0 for r in cache)
report=dict(receiver=receiver,source=source,cache_failure_count=0,
    limits='This checks recorded decisions/slot identities and code paths, not a new healthy-reference source solve or proof of zero indirect marginal information. Mixed-unit state step is not position error.')
with (OUT/'instability_audit.json').open('x') as f:json.dump(report,f,indent=2)
print(json.dumps(dict(receiver=receiver,source={k:v for k,v in source.items() if k!='guard_rows'}),indent=2))
