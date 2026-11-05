"""Bounded timing/conservation summary, no new estimator or accuracy scoring."""
from pathlib import Path
import collections,csv,hashlib,json,math,re,statistics
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs/gnssfgo_eig2_local_cov30_cpr2_20261019/prepared'
S=P/'live30_off_source';R=P/'live30_off_receiver'
RESULT=Path('/home/yeranis/results/gnssfgo_eig2_local_cov30_cpr2_20261019/off')
OLD=W/'runs/gnssfgo_eig2_local_cov60_cpr1_20261019/prepared/live60_off_source'
FROZEN=W/'runs/gnssfgo_eig2_live60_off_on_fl4x_20261017/prepared/live60_off_source'
def stat(values):
 a=sorted(values);assert a
 return dict(n=len(a),mean=statistics.fmean(a),median=statistics.median(a),
             p95=a[math.ceil(.95*len(a))-1],max=a[-1],min=a[0])
def load(path):return json.loads(path.read_text())
def records(path):
 with path.open() as f:return list(csv.reader(f))
checks={name:load(path) for name,path in {
 'receiver':RESULT/'validation.json','repeat':RESULT/'repeat_validation.json',
 'provenance':S/'local_provenance.json','source':S/'candidate_validation.json'}.items()}
assert all(x['passed'] for x in checks.values())
profiles={};finish={};tail_counts={}
for label,source in [('cached_local',S),('uncached_local',OLD),('old_main_revision',FROZEN)]:
 profile=collections.defaultdict(dict)
 for line in (source/'e1b_profile.jsonl').open():
  r=json.loads(line);assert not r['aborted']
  t=r['timestamp'];context=r['context'];assert t not in profile[context]
  profile[context][t]=r['owner_wall']['root'][1]*1e-9
 profiles[label]=profile
 logs=list((source/'ros_logs').glob('*/glim_rosnode-*-stdout.log'));assert len(logs)==1
 by_index={}
 for line in logs[0].open():
  if 'GLIM_CBS_TIMING_ROW,on_smoother_update_finish,' not in line:continue
  fields=re.sub(r'\x1b\[[0-9;]*m','',line).strip().split('GLIM_CBS_TIMING_ROW,on_smoother_update_finish,',1)[1].split(',')
  assert len(fields)==16
  index=int(fields[0]);assert index not in by_index
  by_index[index]=float(fields[-1])*1e-3
 # Shutdown can continue the profiler after ROS stops recording finish rows.
 # Retain only the complete contiguous finish-row prefix, never invent rows.
 times=sorted(profile[4]);assert 0<len(by_index)<=len(times)
 assert sorted(by_index)==list(range(len(by_index)))
 finish[label]={times[i]:v for i,v in by_index.items()}
 tail_counts[label]=dict(revision_profiles=len(times),finish_rows=len(by_index),excluded_unpaired_tail=len(times)-len(by_index))
common=sorted(set.intersection(*(set(p) for p in finish.values())))
steady=[t for t in common if t>=common[0]+5]
comparison={label:stat([p[4][t] for t in steady]) for label,p in profiles.items()}
ratios=stat([profiles['uncached_local'][4][t]/profiles['cached_local'][4][t] for t in steady])
new=profiles['cached_local'];start=min(new[4])
valid_new=set(finish['cached_local'])
contexts={str(c):stat([v for t,v in rows.items() if t>=start+5 and t in valid_new]) for c,rows in new.items() if any(t>=start+5 and t in valid_new for t in rows)}
matched_main={label:stat([p[1][t] for t in steady]) for label,p in profiles.items()}
matched_finish={label:stat([p[t] for t in steady]) for label,p in finish.items()}
events=records(R/'events.csv')
receipts=[json.loads(line) for line in (R/'receipts.jsonl').open()]
assert len(events)==len(receipts)
update_endpoints={}
for e,r in zip(events,receipts):
 if e[0]=='B' and e[2]=='glim_hesai' and r['metadata']['source_revision']==0:
  # First-publication logger uses update timestamp, revisions use update index.
  # The new belief's target index is the producing index (same CP-R1 convention).
  key=int(e[7]);timestamp=int(e[9])
  assert key not in update_endpoints or update_endpoints[key]==timestamp
  update_endpoints[key]=timestamp
ages=[];lags=[];history=[];first=[];missing=0;worst=None
for e,r in zip(events,receipts):
 if e[0]!='B' or e[2]!='glim_hesai':continue
 arrival=int(e[1]);a=int(e[8]);b=int(e[9])
 if r['metadata']['source_revision']==0:first.append((arrival-b)*1e-9);continue
 u=int(r['metadata']['source_update'])
 age=(arrival-a)*1e-9;ages.append(age)
 if worst is None or age>worst['first_endpoint_age_s']:
  worst=dict(edge=[int(e[6]),int(e[7])],source_update=u,revision=int(r['metadata']['source_revision']),first_endpoint_age_s=age,arrival_ns=arrival)
 if u in update_endpoints:
  lags.append((arrival-update_endpoints[u])*1e-9)
  history.append((update_endpoints[u]-a)*1e-9)
 else:missing+=1
assert missing==0
result=dict(passed=True,checks=checks,matched_revision_root_seconds=comparison,
 matched_timestamps=len(steady),matched_uncached_over_cached_ratio=ratios,
 matched_MAIN_optimization_root_seconds=matched_main,
 matched_complete_source_finish_callback_seconds=matched_finish,
 timing_tail_accounting=tail_counts,
 cached_post5s_profile_root_seconds=contexts,
 revision_first_endpoint_age_s=stat(ages),revision_source_update_lag_s=stat(lags),
 revision_within_source_history_s=stat(history),first_publication_endpoint_lag_s=stat(first),
 worst_revision=worst,minimum_margin_to_7s_lag_s=7-max(ages),
 limits='Context1=MAIN optimizer,4=revision routine; timings exclude uninstrumented work. Not totalGLIM or1x/longer/ON certification. LOCAL provenance is not independence.',
 capture=load(P/'live30_off_control/complete.json'))
with (H/'results.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k!='checks'},indent=2))
