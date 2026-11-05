"""Read completed ON evidence; no estimator rerun or cross-policy comparison."""
from pathlib import Path
import collections,csv,json,math,re,shutil,statistics
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs/gnssfgo_eig2_local_cov60_on_cpr3_20261019/prepared'
S=P/'live60_on_source';R=P/'live60_on_receiver'
V=Path('/home/yeranis/results/gnssfgo_eig2_local_cov60_on_cpr3_20261019/on')
def load(p):return json.loads(p.read_text())
def stat(values):
 a=sorted(values);assert a
 return dict(n=len(a),mean=statistics.fmean(a),median=statistics.median(a),p95=a[math.ceil(.95*len(a))-1],max=a[-1],min=a[0])
checks={name:load(path) for name,path in {'receiver':V/'validation.json',
 'repeat':V/'repeat_validation.json','source':S/'candidate_validation.json',
 'local_provenance':S/'local_provenance.json'}.items()}
assert all(c['passed'] for c in checks.values())
assert (P/'live60_on_control/complete.json').exists()
assert not (P/'live60_on_control/failed.txt').exists()
with (R/'events.csv').open() as f:events=list(csv.reader(f))
receipts=[json.loads(line) for line in (R/'receipts.jsonl').open()]
assert len(events)==len(receipts)
ends={int(e[7]):int(e[9]) for e,r in zip(events,receipts)
      if e[0]=='B' and e[2]=='glim_hesai' and r['metadata']['source_revision']==0}
ages=[];lags=[];history=[];worst=None
for e,r in zip(events,receipts):
 if e[0]!='B' or e[2]!='glim_hesai' or not r['metadata']['source_revision']:continue
 u=int(r['metadata']['source_update']);assert u in ends
 a=(int(e[1])-int(e[8]))*1e-9
 ages.append(a);lags.append((int(e[1])-ends[u])*1e-9);history.append((ends[u]-int(e[8]))*1e-9)
 if worst is None or a>worst['age_s']:worst=dict(edge=list(map(int,e[6:8])),update=u,revision=r['metadata']['source_revision'],age_s=a)
profile=collections.defaultdict(dict)
for line in (S/'e1b_profile.jsonl').open():
 r=json.loads(line);assert not r['aborted']
 assert r['timestamp'] not in profile[r['context']]
 profile[r['context']][r['timestamp']]=r['owner_wall']['root'][1]*1e-9
logs=list((S/'ros_logs').glob('*/glim_rosnode-*-stdout.log'));assert len(logs)==1
finish={}
for line in logs[0].open():
 marker='GLIM_CBS_TIMING_ROW,on_smoother_update_finish,'
 if marker not in line:continue
 fields=re.sub(r'\x1b\[[0-9;]*m','',line).strip().split(marker,1)[1].split(',')
 assert len(fields)==16
 index=int(fields[0]);assert index not in finish;finish[index]=float(fields[-1])*1e-3
times=sorted(profile[4]);assert sorted(finish)==list(range(len(finish))) and len(finish)<=len(times)
valid=set(times[:len(finish)]);start=min(times)
timings={str(c):stat(v for t,v in values.items() if t in valid and t>=start+5)
         for c,values in profile.items() if any(t in valid and t>=start+5 for t in values)}
timings['source_finish_callback']=stat(v for i,v in finish.items() if times[i]>=start+5)
evaluation=load(P/'evaluation_full/summary.json')
assert set(evaluation['results'])=={'on_native_glim','on_native_kimera','on_fused_online','on_fused_finalized'}
for v in evaluation['results'].values():assert v['full_saved']['span_s']>=58
result=dict(passed=True,mode='standalone CBS-ON60s',checks=checks,
 capture=load(P/'live60_on_control/complete.json'),evaluation=evaluation,
 revision_first_endpoint_age_s=stat(ages),revision_producing_update_lag_s=stat(lags),
 revision_within_source_history_s=stat(history),worst_revision=worst,
 lag_margin_s=7-max(ages),post5s_timing_seconds=timings,
 unpaired_profile_shutdown_tail=len(times)-len(finish),
 limits='No matched60s cachedLOCAL OFF; no universal real-time/stationarity/independence claim. Full saved output vs IE-TC shares CPT7; finalized is delayed, not ONLINE.')
with (H/'results.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k not in ('checks','evaluation')},indent=2))
print(json.dumps({k:v['full_saved'] for k,v in evaluation['results'].items()},indent=2))
