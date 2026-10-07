from pathlib import Path
import sys,csv,json,hashlib,collections
import numpy as np
ROOT=Path(__file__).resolve().parent;OLD=ROOT.parent/'gnssfgo_eth1_roundoff_40s_20261006';LONG=ROOT.parent/'gnssfgo_eth1_wnoa_wnoj_120s_20261006'
def rows(p):
 with p.open()as f:return list(csv.DictReader(f))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def diffs(a,b):
 assert [r['timestamp_ns']for r in a]==[r['timestamp_ns']for r in b]
 def arr(r,f):return np.array([[float(x[k])for k in f]for x in r])
 d={}
 for k,f in [('p',['px','py','pz']),('v',['vx','vy','vz']),('w',['wx','wy','wz'])]:d[k]=float(np.max(np.linalg.norm(arr(a,f)-arr(b,f),axis=1)))
 f=['qx','qy','qz','qw'];qa=arr(a,f);qb=arr(b,f);qa/=np.linalg.norm(qa,axis=1)[:,None];qb/=np.linalg.norm(qb,axis=1)[:,None];qb*=np.where((qa*qb).sum(axis=1)>=0,1.,-1.)[:,None]
 d['orientation_rad']=float(np.max(4*np.arctan2(np.linalg.norm(qa-qb,axis=1),np.linalg.norm(qa+qb,axis=1))))
 if 'a_x'in a[0]:d['acceleration']=float(np.max(np.linalg.norm(arr(a,['alpha_x','alpha_y','alpha_z','a_x','a_y','a_z'])-arr(b,['alpha_x','alpha_y','alpha_z','a_x','a_y','a_z']),axis=1)))
 return d
label=sys.argv[1];case=ROOT/label;mode,model=label.split('_')[:2];short=label.endswith('_short');repeat=label.endswith('_repeat')
out=dict(case=label,complete=(case/'summary.json').exists(),prefix={},checkpoints=[])
conv=rows(case/'publication_convergence.csv');out['publications']=len(conv);out['failures']=[r for r in conv if r['status']!='stationary'or not float(r['gradient_after'])<=1e-5]
out['max_gradient']=max(float(r['gradient_after'])for r in conv)
steps=rows(case/'publication_steps.csv');special=[r for r in steps if r['roundoff']=='1'];out['stable_steps']=special
assert all(float(r['stable_cost_delta']) < -float(r['arithmetic_bound'])and float(r['predicted_delta']) < -float(r['predicted_bound'])and float(r['gradient_after'])<=1e-5 for r in special)
out['consensus_steps']=[r for r in steps if r['consensus']=='1']
for r in out['consensus_steps']:
 assert r['accepted']=='1'and r['recovery']=='1'and r['roundoff']=='0'
 assert float(r['gradient_after'])<=1e-5 and float(r['qr_disagreement'])<=np.sqrt(np.finfo(float).eps)
 assert abs(float(r['stable_cost_delta']))<=1e-11*max(1.,float(r['error_before']))
 assert -1e-11*max(1.,float(r['error_before']))<=float(r['predicted_delta'])<-float(r['predicted_bound'])
 previous=steps[steps.index(r)-1]
 assert previous['index']==r['index']and previous['accepted']=='0'and float(previous['gradient_after'])<=1e-5
 assert abs(float(previous['stable_cost_delta']))<=1e-11*max(1.,float(r['error_before']))
for r in steps:
 if r.get('local_prediction')=='1':
  assert float(r['predicted_delta']) < -float(r['predicted_bound'])
  assert abs(float(r['stable_cost_delta']))<=1e-11*max(1.,float(r['error_before']))
  assert int(r['iteration'])<=20
out['local_prediction_steps']=[r for r in steps if r.get('local_prediction')=='1']
if repeat:
 files=['trajectory.tum','states.csv','online_trajectory.tum','online_states.csv','causal_dispatch.csv','model_covariance.csv','source_model_covariance.csv','support_clock.csv','publication_steps.csv','measurement_events.csv']
 out['repeat']={f:sha(case/f)==sha(ROOT/f'{mode}_{model}'/f)for f in files}
 out['repeat_bitwise']=all(out['repeat'].values())
 out['repeat_state_differences']={f:diffs(rows(case/f),rows(ROOT/f'{mode}_{model}'/f))for f in ['states.csv','online_states.csv']}
 limits=dict(p=1e-4,orientation_rad=1e-4,v=1e-3,w=1e-3,acceleration=1e-3)
 unchanged=all(out['repeat'][f]for f in ['causal_dispatch.csv','model_covariance.csv','source_model_covariance.csv','support_clock.csv','measurement_events.csv'])
 out['repeat_numerical']=unchanged and all(v<=limits[k]for d in out['repeat_state_differences'].values()for k,v in d.items())
else:
 for f in ['online_trajectory.tum','online_states.csv','support_clock.csv']:
  a=(OLD/f'{mode}_{model}'/f).read_bytes().splitlines(keepends=True);b=(case/f).read_bytes().splitlines(keepends=True);out['prefix'][f]=b[:len(a)]==a
 out['prefix']['all_394_state_and_ledger_snapshots']=all(all(sha(case/f'snapshot_{i}'/f)==sha(OLD/f'{mode}_{model}'/f'snapshot_{i}'/f)for f in ['incremental_states.csv','ledger.csv'])for i in range(1,395))if len(conv)>=394 else False
 if not all(out['prefix'].values())and len(conv)>=395:
  comparisons=[];ledger_equal=True
  for i in range(1,395):
   a=OLD/f'{mode}_{model}/snapshot_{i}';b=case/f'snapshot_{i}'
   ledger_equal &= sha(a/'ledger.csv')==sha(b/'ledger.csv')
   comparisons.append(diffs(rows(a/'incremental_states.csv'),rows(b/'incremental_states.csv')))
  maximum={k:max(d[k]for d in comparisons)for k in comparisons[0]}
  # Existing physical-state comparison tolerances from copied-graph validation.
  limits=dict(p=1e-4,orientation_rad=1e-4,v=1e-3,w=1e-3,acceleration=1e-3)
  out['numerical_prefix']=dict(maximum=maximum,ledger_equal=ledger_equal,valid=ledger_equal and all(v<=limits[k]for k,v in maximum.items()))
 if short and out['complete']:
  out['short_scientific_equality']={f:sha(case/f)==sha(OLD/f'{mode}_{model}'/f)for f in ['trajectory.tum','states.csv','model_covariance.csv','source_model_covariance.csv','measurement_events.csv','causal_dispatch.csv']}
 for snap in sorted(case.glob('snapshot_*')):
  if not (snap/'polished_states.csv').exists():continue
  d=diffs(rows(snap/'incremental_states.csv'),rows(snap/'polished_states.csv'));restart=diffs(rows(snap/'batch_states.csv'),rows(snap/'polished_states.csv'));s=rows(snap/'summary.csv')[0]
  g=float(rows(snap/'incremental_linear_health.csv')[0]['column_normalized_gradient_inf']);rg=float(rows(snap/'polished_linear_health.csv')[0]['column_normalized_gradient_inf']);cert=rows(snap/'batch_stationarity.csv')[0]
  valid=g<=1e-5 and rg<=1e-5 and cert['polished_stationary']=='1'and(float(s['initial_error'])-float(s['polished_error']))/max(1.,float(s['polished_error']))<=1e-6 and d['p']<=1e-4 and d['orientation_rad']<=1e-4 and d['v']<=1e-3 and d['w']<=1e-3 and restart['p']<1e-6 and restart['v']<1e-4 and int(s['batch_iterations'])<500 and int(s['polish_iterations'])<100
  out['checkpoints'].append(dict(snapshot=snap.name,valid=valid,difference=d,gradient=g,reference_gradient=rg,batch_iterations=s['batch_iterations'],polish_iterations=s['polish_iterations']))
if out['complete']:
 s=json.loads((case/'summary.json').read_text());out['summary']=s;expected_states=395 if short else 1193
 for f in ['states.csv','online_states.csv']:
  values=rows(case/f)
  assert all(np.isfinite(float(v))for r in values for k,v in r.items()if k!='timestamp_ns')
  assert all(int(a['timestamp_ns'])<int(b['timestamp_ns'])for a,b in zip(values,values[1:]))
 assert s['support_states']==expected_states and len(conv)==expected_states
 assert s['imu_factor_count']==s['bias_variable_count']==s['startup_imu_samples']==0
 assert s['source_model_noise']=={}and s['pending_at_end']==s['revision_duplicates']==s['revision_stale']==0
 m=json.loads((LONG/'manifest.json').read_text())['modes'][mode]
 assert s['beliefs_accepted']==({'off':573,'on':572}[mode]if short else m['unique_ids'])and s['beliefs_skipped_no_support']==25
 clock={int(r['state_index']):int(r['available_timestamp_ns'])for r in rows(case/'support_clock.csv')};ids={};replacements=0
 for r in rows(case/'causal_dispatch.csv'):
  assert int(r['arrival_ns'])<=int(r['insertion_ns'])and int(r['required_support'])<=int(r['created_support'])and clock[int(r['required_support'])]<=int(r['insertion_ns'])
  k=tuple(r[x]for x in ['source','session','from_index','to_index']);revision=int(r['revision'])
  if r['outcome']=='insert':assert k not in ids
  elif r['outcome']=='replace':assert k in ids and revision>ids[k];replacements+=1
  else:raise AssertionError(r)
  ids[k]=revision
 assert len(ids)==s['beliefs_accepted']and replacements==s['revision_removals']
 if not short:assert replacements==m['replacements']
 out['accounting']=dict(ids=len(ids),replacements=replacements)
prefix_valid=all(out['prefix'].values())or out.get('numerical_prefix',{}).get('valid',False)
scientific=out.get('short_scientific_equality',{})
short_valid=all(scientific.values())or(out.get('numerical_prefix',{}).get('valid',False)and all(scientific.get(f,False)for f in ['model_covariance.csv','source_model_covariance.csv','measurement_events.csv','causal_dispatch.csv']))
out['pass_all']=out['complete']and not out['failures']and prefix_valid and all(c['valid']for c in out['checkpoints'])and short_valid and out.get('repeat_numerical',True)
(ROOT/f'validation_{label}.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps({k:v for k,v in out.items()if k not in ['summary','checkpoints','stable_steps']},indent=2))
print('batch checks',len(out['checkpoints']),'stable steps',len(special))
if not out['pass_all']:raise SystemExit(2)
