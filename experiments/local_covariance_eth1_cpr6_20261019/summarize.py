"""Summarize the matched capture; no estimator launches or tuning."""
from pathlib import Path
import collections,csv,hashlib,json,math,re,statistics,shutil,subprocess
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs/gnssfgo_eth1_local_cov60_cpr6_20261019/prepared'
V=Path('/home/yeranis/results/gnssfgo_eth1_local_cov60_cpr6_20261019')
def load(p):return json.loads(p.read_text())
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def stat(values):
 a=sorted(values);assert a
 return dict(n=len(a),mean=statistics.fmean(a),median=statistics.median(a),p95=a[math.ceil(.95*len(a))-1],max=a[-1],min=a[0])
manifest=load(P/'protected_manifest.json')
for p,h in manifest.items():assert sha(p)==h,p
cases={}
for mode in ('off','on'):
 S=P/f'live60_{mode}_source';R=P/f'live60_{mode}_receiver';C=P/f'live60_{mode}_control'
 assert (C/'complete.json').exists() and not (C/'failed.txt').exists()
 checks={name:load(path) for name,path in {'receiver':V/mode/'validation.json',
  'repeat':V/mode/'repeat_validation.json','source':S/'candidate_validation.json',
  'local_provenance':S/'local_provenance.json'}.items()}
 assert all(c['passed'] for c in checks.values())
 with (R/'events.csv').open() as f:events=list(csv.reader(f))
 receipts=[json.loads(line) for line in (R/'receipts.jsonl').open()];assert len(events)==len(receipts)
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
  r=json.loads(line);assert not r['aborted'] and r['timestamp'] not in profile[r['context']]
  profile[r['context']][r['timestamp']]=r['owner_wall']['root'][1]*1e-9
 logs=list((S/'ros_logs').glob('*/glim_rosnode-*-stdout.log'));assert len(logs)==1
 finish={}
 for line in logs[0].open():
  marker='GLIM_CBS_TIMING_ROW,on_smoother_update_finish,'
  if marker not in line:continue
  f=re.sub(r'\x1b\[[0-9;]*m','',line).strip().split(marker,1)[1].split(',');assert len(f)==16
  index=int(f[0]);assert index not in finish;finish[index]=float(f[-1])*1e-3
 times=sorted(profile[4]);assert sorted(finish)==list(range(len(finish))) and len(finish)<=len(times)
 valid=set(times[:len(finish)]);start=min(times)
 timings={str(c):stat(v for t,v in values.items() if t in valid and t>=start+5)
          for c,values in profile.items() if any(t in valid and t>=start+5 for t in values)}
 timings['source_finish_callback']=stat(v for i,v in finish.items() if times[i]>=start+5)
 cases[mode]=dict(checks=checks,capture=load(C/'complete.json'),
  revision_first_endpoint_age_s=stat(ages),revision_producing_update_lag_s=stat(lags),
  revision_within_source_history_s=stat(history),worst_revision=worst,
  lag_margin_s=7-max(ages),post5s_timing_seconds=timings,
  unpaired_profile_shutdown_tail=len(times)-len(finish))
 assert max(ages)<7
off_effective=load(P/'live60_off_source/effective_mode_check.json')
on_effective=load(P/'live60_on_source/effective_mode_check.json')
for field in ('glim_revision_covariance','cached_provider_sha256','kimera'):
 assert off_effective[field]==on_effective[field],field
g_off=off_effective['glim'].copy();g_on=on_effective['glim'].copy()
assert g_off.pop('glim_cbs_receive_external_beliefs') is False
assert g_on.pop('glim_cbs_receive_external_beliefs') is True
assert g_off==g_on
for folder in ('glim_config','kimera_profile'):
 trees=[]
 for mode in ('off','on'):
  base=P/f'live60_{mode}_source'/folder
  trees.append({str(p.relative_to(base)):sha(p) for p in base.rglob('*') if p.is_file()})
 assert trees[0] and trees[0]==trees[1],folder
evaluation=load(P/'evaluation_full/summary.json')
assert set(evaluation['results'])=={m+'_'+s for m in ('off','on') for s in ('native_glim','native_kimera','fused_online','fused_finalized')}
comparison={}
coverage={}
for role in ('native_glim','native_kimera','fused_online','fused_finalized'):
 off=evaluation['results']['off_'+role]['full_saved'];on=evaluation['results']['on_'+role]['full_saved']
 assert min(off['span_s'],on['span_s'])>=58
 coverage[role]={k:dict(off=off[k],on=on[k]) for k in ('rows','start_s','end_s','span_s','rpe_pairs')}
 comparison[role]={k:dict(off=off[k],on=on[k],on_minus_off=on[k]-off[k],percent=100*(on[k]/off[k]-1))
  for k in ('ATE_RMSE_m','translation_RPE_1s_RMSE_m','rotation_RPE_1s_RMSE_deg','path_length_prism_m')}
states={}
for name in ('glim_ros1','Kimera-VIO','cbs'):
 path=W/'src'/name
 status=subprocess.check_output(['git','-C',str(path),'status','--porcelain'],text=True)
 assert not status,(name,status)
 states[name]=dict(clean=True,head=subprocess.check_output(['git','-C',str(path),'rev-parse','HEAD'],text=True).strip())
result=dict(passed=True,cases=cases,evaluation=evaluation,comparison_full=comparison,coverage=coverage,
 matching=dict(runtime_covariance_provider=True,source_flags_except_receiving=True,copied_profiles=True),
 protected_unchanged=len(manifest),source_repos=states,free_bytes=shutil.disk_usage(W).free,
 gnssFGO_head=subprocess.check_output(['git','-C','/home/yeranis/repos/gnssFGO','rev-parse','HEAD'],text=True).strip(),
 limits='One matched60s ETH1 pair at0.10x; no stationarity/independence/1x claim. Full shared-CPT7 IE-TC reference. Finalized is delayed, not ONLINE.')
with (H/'results.json').open('x') as f:json.dump(result,f,indent=2)
for mode in ('off','on'):
 out=H/'evidence'/mode;out.mkdir(parents=True,exist_ok=False)
 S=P/f'live60_{mode}_source';R=P/f'live60_{mode}_receiver'
 paths=[P/f'live60_{mode}_control/complete.json',S/'local_provenance.json',S/'candidate_validation.json',
  S/'source_coverage.json',S/'effective_mode_check.json',S/'run_metadata.txt',R/'transport_complete.json',
  R/'resolved_config.json',V/mode/'validation.json',V/mode/'repeat_validation.json']
 for p in paths:shutil.copy2(p,out/p.name)
shutil.copy2(P/'preflight/validation.json',H/'evidence/preflight_validation.json')
shutil.copy2(P/'evaluation_full/summary.json',H/'evidence/evaluation_summary.json')
print(json.dumps(comparison,indent=2))
