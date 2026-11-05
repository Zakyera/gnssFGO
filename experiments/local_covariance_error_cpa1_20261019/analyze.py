"""Read frozen all-LOCAL journals; no estimator/optimizer execution."""
from pathlib import Path
import importlib.util,json,sys
import numpy as np
sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('fl4z',H.parent/'fixed_lag_covariance_fl4z_20261017/analyze.py')
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
W=a.W
CASES={
 'eig2':dict(off='gnssfgo_eig2_local_cov60_off_cpr4_20261019',on='gnssfgo_eig2_local_cov60_on_cpr3_20261019',bag='EIG-2/2024-11-03-13-59-54_cpt7_ie_tc.bag'),
 'spx3':dict(off='gnssfgo_spx3_local_cov60_cpr5_20261019',on='gnssfgo_spx3_local_cov60_cpr5_20261019',bag='SPX-3/2024-11-02-17-43-10_cpt7_ie_tc.bag'),
 'eth1':dict(off='gnssfgo_eth1_local_cov60_cpr6_20261019',on='gnssfgo_eth1_local_cov60_cpr6_20261019',bag='2024-10-01-11-29-55_cpt7_ie_tc.bag')}
STAGES=('first','last_common_update','age4s_first','age4s_later')

def extra(rr,mode):
 e=np.asarray([r[mode]['phi']+r[mode]['rho'] for r in rr])
 out={}
 for label,sl in [('rotation',slice(0,3)),('translation',slice(3,6))]:
  v=e[:,sl];mean=v.mean(axis=0);center=v-mean
  white=np.asarray([np.linalg.solve(np.linalg.cholesky(np.asarray(r[mode+'_cov'])[sl,sl]),v[i]) for i,r in enumerate(rr)])
  out[label]=dict(mean_error_vector=mean.tolist(),mean_vector_norm=float(np.linalg.norm(mean)),
   rms_centered_error_norm=float(np.sqrt(np.mean(np.sum(center*center,axis=1)))),
   mean_whitened_error_vector=white.mean(axis=0).tolist(),
   reference_discrepancy_mean_per_dimension=float(np.mean(np.sum(white*white,axis=1))/3),
   units='radians' if label=='rotation' else 'metres')
 return out

def main(seq):
 assert seq in CASES
 out=H/(seq+'_results.json');assert not out.exists(),out
 cfg=dict(CASES[seq],prefix='live60',lower=0,upper=2**63-1)
 roots={m:W/'runs'/cfg[m]/'prepared' for m in ('off','on')}
 inputs={str(p):a.geo.sha(p) for p in [Path(__file__),H/'PROTOCOL.txt',Path(a.__file__),Path(a.geo.__file__)]}
 versions={};first={};checks={};keytimes={}
 for m,root in roots.items():
  paths=[root/f'live60_{m}_control/complete.json',root/f'live60_{m}_source/local_provenance.json',
   root/f'live60_{m}_source/candidate_validation.json',root/f'live60_{m}_source/effective_mode_check.json',
   Path('/home/yeranis/results')/cfg[m]/m/'validation.json',
   Path('/home/yeranis/results')/cfg[m]/m/'repeat_validation.json']
  for p in paths:
   d=json.loads(p.read_text());inputs[str(p)]=a.geo.sha(p)
   if p.name!='complete.json':assert d['passed'],p
  eff=json.loads(paths[3].read_text())
  assert eff['glim_revision_covariance']=='native_local'
  assert eff['cached_provider_sha256']=='3f23e6115e34acc5a2f2b965b15bd4c651897c889a6c38a0a52cdb3855c0ceb7'
  versions[m],first[m],checks[m],keytimes[m]=a.read_events(root,'live60',m,cfg,inputs)
 common=set(first['off'])&set(first['on']);assert common
 bag=W/'src/datasets/GrandTour'/cfg['bag'];inputs[str(bag)]=a.geo.sha(bag)
 reference=a.geo.Reference(bag)
 records=[];cohorts={s:dict(total=0,eligible=0,excluded=0) for s in a.SOURCES}
 def pair(key,stage,xs):
  p=dict(source=key[0],a=key[1],b=key[2],stage=stage)
  for m,x in xs.items():
   p[m]=a.measure(x,reference)
   p[m+'_metadata']={k:v for k,v in x.items() if k not in ('mu','cov')}
   p[m+'_mu']=x['mu'].tolist();p[m+'_cov']=x['cov'].tolist()
  assert p['off_metadata']['update']==p['on_metadata']['update']
  assert p['off_metadata']['source_i']==p['on_metadata']['source_i']
  assert p['off_metadata']['source_j']==p['on_metadata']['source_j']
  records.append(p)
 for key in sorted(common):
  updates=set(versions['off'][key])&set(versions['on'][key]);assert updates,key
  pair(key,'first',{m:first[m][key] for m in roots})
  pair(key,'last_common_update',{m:versions[m][key][max(updates)] for m in roots})
  offset=40 if key[0]=='glim_hesai' else 20
  target=first['off'][key]['update']+offset
  cohort=cohorts[key[0]];cohort['total']+=1
  if target not in updates:cohort['excluded']+=1;continue
  cohort['eligible']+=1
  pair(key,'age4s_first',{m:first[m][key] for m in roots})
  pair(key,'age4s_later',{m:versions[m][key][target] for m in roots})
 summaries={}
 for s in a.SOURCES:
  summaries[s]={}
  for stage in STAGES:
   rr=[r for r in records if r['source']==s and r['stage']==stage];assert rr
   summary={m:a.summarize([r[m] for r in rr]) for m in roots}
   summary['matched']=a.paired_summary(rr)
   summary['signed_error']={m:extra(rr,m) for m in roots}
   summary['coverage']=dict(edges=len(rr),from_ns=min(r['a'] for r in rr),to_ns=max(r['b'] for r in rr),
    interval_duration_s=a.stat([(r['b']-r['a'])/1e9 for r in rr]),
    source_update_age=a.stat([r['off_metadata']['update']-r['off_metadata']['source_j'] for r in rr]))
   summaries[s][stage]=summary
 assert all(a.geo.sha(Path(p))==h for p,h in inputs.items())
 result=dict(sequence=seq,config=cfg,checks=checks,common_edges=len(common),
  unmatched_edges={m:len(set(first[m])-common) for m in roots},age4s_cohorts=cohorts,
  reference=dict(path=str(bag),max_used_bracket_ns=reference.max_bracket_ns,
    caveat='Shared-CPT7 IE-TC with unknown covariance/correlation, not independent noiseless truth'),
  sources=summaries,records=records,input_hashes=inputs,inputs_unchanged=True)
 a.write(out,result)
 compact={k:v for k,v in result.items() if k!='records'}
 a.write(H/(seq+'_summary.json'),compact)
 print(json.dumps(dict(sequence=seq,paired_edges=len(common),cohorts=cohorts,unmatched=result['unmatched_edges']),indent=2))

if __name__=='__main__':
 a.geo.self_test();main(sys.argv[1])
