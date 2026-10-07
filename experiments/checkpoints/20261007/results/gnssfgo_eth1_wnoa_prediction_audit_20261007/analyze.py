from pathlib import Path
import json,hashlib,csv
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

R=Path(__file__).resolve().parent;P=R.parent/'gnssfgo_eth1_local_prediction_20261007'
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(2**20),b''):h.update(b)
 return h.hexdigest()
def stat(x):
 x=np.asarray(x,dtype=float);x=x[np.isfinite(x)]
 return dict(n=len(x),mean=float(np.mean(x)),median=float(np.median(x)),rms=float(np.sqrt(np.mean(x*x))),p95=float(np.quantile(x,.95)),maximum=float(np.max(x))) if len(x) else dict(n=0)
def load(p):return pd.read_csv(p,dtype={'snapshot':str})
pub=load(R/'publications.csv');chg=load(R/'changes.csv');m=load(R/'primary/marginal_summary.csv');g=load(R/'primary/graph_checks.csv');de=load(R/'primary/prediction_decomposition.csv');dense=load(R/'means/intervals.csv');front=load(R/'means/frontier_changes.csv')
zero=1727775008003475000
out={};sels=json.loads((R/'selection.json').read_text())
for mode in ['off','on']:
 p=pub[pub['mode']==mode];a=chg[chg['mode']==mode];s=m[m['mode']==mode];ds=dense[dense['mode']==mode];d={}
 d['publications']=len(p);d['source_free_startup_publications']=int((p.frontier_ns==0).sum());d['source_informed']=int((p.frontier_ns>0).sum())
 d['first_source_s']=(int(p[p.frontier_ns>0].iloc[0].now_ns)-zero)*1e-9
 d['endpoint_delay_s']=stat(p.endpoint_delay_s);d['pure_prediction_s']=stat(p.pure_prediction_s)
 d['pure_tail_publications']=int((p.pure_prediction_s>0).sum());d['speed_mps']=stat(p.speed_mps);d['omega_radps']=stat(p.omega_radps)
 d['changes']={z:{k:stat(b[k]) for k in ['position_m','orientation_rad','velocity_mps','omega_radps']} for z,b in a.groupby('zone')}
 f=front[front['mode']==mode];d['exact_frontier_changes']={k:stat(f[k]) for k in ['next_position_change_m','next_orientation_change_rad']}
 d['covariance_sample_snapshots']=int(g[g['mode']==mode].shape[0]);d['uncertainty']={}
 regular=set(k for k,v in sels[mode].items() if 'regular_20' in v)
 for subset,ss in [('stress_and_regular',s),('regular_only',s[s.snapshot.isin(regular)])]:
  d['uncertainty'][subset]={}
  for (zone,component),b in ss.groupby(['zone','component']):
   d['uncertainty'][subset][zone+'_'+component]={k:stat(b[k]) for k in ['sqrt_trace','next_change_norm','next_change_mahalanobis','next_change_direction_sigma']}
 dd=de[de['mode']==mode].merge(g[['mode','snapshot','source_free_duration_s']],on=['mode','snapshot'])
 d['new_process_trace_fraction']={component:stat(b.process_trace/b.current_trace) for component,b in dd[dd.source_free_duration_s>0].groupby('component')}
 d['dense_interpolation']={}
 for label,b in [('online_last5_intervals',ds[ds.snapshot!='final']),('final_history',ds[ds.snapshot=='final'])]:
  d['dense_interpolation'][label]={k:stat(b[k]) for k in ['excess_m','max_perpendicular_m','overshoot_m','max_fd_speed_mps','max_fd_omega_radps']}
  d['dense_interpolation'][label]['arc_m']=float(b.arc_m.sum());d['dense_interpolation'][label]['chord_m']=float(b.chord_m.sum())
  d['dense_interpolation'][label]['note']='overlapping local intervals, not a trajectory path' if label.startswith('online') else 'full final retrospective history'
  d['dense_interpolation'][label]['worst_excess']=b.loc[b.excess_m.idxmax()].to_dict()
 worst=a[a.zone=='current'].sort_values('position_m').iloc[-1]
 d['largest_next_position_revision']=worst.to_dict();snap=worst.snapshot
 d['largest_event_publication']=p[p.snapshot==snap].iloc[0].to_dict();d['largest_event_uncertainty']=s[s.snapshot==snap].to_dict('records')
 out[mode]=d

verification={}
verification['repeat_covariance_byte_identical']={p.name:sha(p)==sha(R/'repeat'/p.name) for p in (R/'primary').glob('*.csv')}
verification['repeat_means_byte_identical']={p.name:sha(p)==sha(R/'means_repeat'/p.name) for p in (R/'means').glob('*.csv')}
verification['sanitizer_byte_subset']={}
for path in (R/'sanitized').glob('*.csv'):
 with path.open() as f:sr=list(csv.reader(f))
 keys=set((r[0],r[1]) for r in sr[1:])
 with (R/'primary'/path.name).open() as f:pr=list(csv.reader(f))
 wanted=[pr[0]]+[r for r in pr[1:] if (r[0],r[1]) in keys]
 verification['sanitizer_byte_subset'][path.name]=(sr==wanted)
verification['reconstructed_graphs']=len(g)
verification['payload_probed_graphs']=int((g.probe_cov_error>=0).sum())
verification['max_factor_cost_relative_error']=float(g.cost_relative_error.max())
verification['max_probe_covariance_difference']=float(g.probe_cov_error.max());verification['max_probe_residual_difference']=float(g.probe_residual_error.max())
verification['max_propagation_relative_error']=float(g.propagation_relative_error.max());verification['min_state_joint_eigenvalue']=float(g.min_eigenvalue.min())
blocks=load(R/'primary/marginal_blocks.csv');mineig=1e100;sym=0.;jointchecked=0
for key,b in blocks.groupby(['mode','snapshot','zone']):
 n=int(b.row.max())+1;v=np.zeros((n,n));v[b.row,b.col]=b.value
 assert np.isfinite(v).all();sym=max(sym,float(np.max(abs(v-v.T))))
 if key[2] not in ['process']:
  eig=float(np.linalg.eigvalsh((v+v.T)/2).min());mineig=min(mineig,eig);assert eig>0; jointchecked+=1
 else:assert np.linalg.eigvalsh((v+v.T)/2).min()>-1e-12
verification['all_exported_covariances_positive']=True;verification['exported_spd_matrices']=jointchecked;verification['max_exported_symmetry_error']=sym;verification['min_exported_eigenvalue']=mineig
one=blocks[(blocks['mode']=='on')&(blocks.snapshot=='201')&(blocks.zone=='current')]
p=np.zeros((12,12));p[one.row,one.col]=one.value
ar=pd.read_csv(P/'on_wnoa/snapshot_201/incremental_states.csv');br=pd.read_csv(P/'on_wnoa/snapshot_202/incremental_states.csv')
dv=br.iloc[201][['vx','vy','vz']].to_numpy(float)-ar.iloc[201][['vx','vy','vz']].to_numpy(float)
ind=float(np.sqrt(dv@np.linalg.solve(p[6:9,6:9],dv)));expected=m[(m['mode']=='on')&(m.snapshot=='201')&(m.zone=='current')&(m.component=='velocity')].iloc[0].next_change_mahalanobis
assert abs(ind-expected)<1e-9;verification['independent_on201_velocity_revision_distance']=ind

# Exact causal availability check uses saved publication availability, not wall time.
maxahead=0;causal_count=0
for mode in ['off','on']:
 for row in pub[pub['mode']==mode].itertuples():
  folder=P/f'{mode}_wnoa/snapshot_{row.snapshot}'
  with (folder/'summary.csv').open() as f:summary=next(csv.DictReader(f))
  with (folder/'ledger.csv').open() as f:
   for e in csv.DictReader(f):
    assert int(e['arrival_ns'])<=int(summary['available_ns']),(mode,row.snapshot,e['arrival_ns'],summary['available_ns'])
    maxahead=max(maxahead,int(e['arrival_ns'])-int(row.now_ns));causal_count+=1
verification['ledger_availability_checks']=causal_count;verification['max_arrival_after_physical_support_ns']=maxahead

vel=load(R/'velocity_check.csv')
verification['endpoint_derivative_checks']={str(eps):{k:stat(b[k]) for k in ['velocity_error_mps','omega_error_radps']} for eps,b in vel.groupby('epsilon_s')}
fine=vel[np.isclose(vel.epsilon_s,1e-6,rtol=1e-10,atol=0)]
assert len(fine)==1930
assert fine.velocity_error_mps.max()<1e-3
assert fine.omega_error_radps.max()<1e-3

# Conditional GP bridge chart covariance, not added to source factors.
def Q(t,q):return q*np.array([[t**3/3,t*t/2],[t*t/2,t]])
def phi(t):return np.array([[1,t],[0,1]])
bridge=[]
for q,part in [(.00375,'rotation'),(.015,'translation')]:
 for t in np.linspace(0,.1,101):
  qt=Q(t,q);cross=qt@phi(.1-t).T;b=qt-cross@np.linalg.solve(Q(.1,q),cross.T)
  assert np.linalg.eigvalsh(b).min()>-1e-15
  bridge.append(dict(component=part,tau_s=t,pose_variance=b[0,0]))
verification['conditional_bridge_max_per_axis_std']={part:float(np.sqrt(max(x['pose_variance'] for x in bridge if x['component']==part))) for part in ['rotation','translation']}
pd.DataFrame(bridge).to_csv(R/'conditional_bridge.csv',index=False)
manifest=json.loads((R/'manifest.json').read_text());bad=[p for p,h in manifest.items() if not Path(p).is_file() or sha(p)!=h]
verification['protected_files']=len(manifest);verification['protected_mismatches']=bad
assert not bad
assert all(verification['repeat_covariance_byte_identical'].values()) and all(verification['repeat_means_byte_identical'].values())
assert all(verification['sanitizer_byte_subset'].values())
out['validation']=verification
(R/'summary.json').write_text(json.dumps(out,indent=2,default=lambda x:x.item() if hasattr(x,'item') else str(x))+'\n')

fig,axes=plt.subplots(2,2,figsize=(13,7),sharex=True,sharey='row')
for col,mode in enumerate(['off','on']):
 a=chg[(chg['mode']==mode)&(chg.zone=='current')].merge(pub[['mode','snapshot','now_ns']],on=['mode','snapshot'])
 ss=m[(m['mode']==mode)&(m.zone=='current')].merge(pub[['mode','snapshot','now_ns']],on=['mode','snapshot'])
 regular=set(k for k,v in sels[mode].items() if 'regular_20' in v)
 for row,comp,field,scale,unit in [(0,'position','position_m',100,'cm'),(1,'velocity','velocity_mps',1,'m/s')]:
  ax=axes[row,col];b=ss[(ss.component==comp)&ss.snapshot.isin(regular)]
  ax.plot((a.now_ns-zero)*1e-9,a[field]*scale,color='#bf4c45',lw=.8,label='Next-publication revision at SAME time')
  ax.scatter((b.now_ns-zero)*1e-9,b.sqrt_trace*scale,color='#26699a',s=14,label='Earlier sqrt(trace P), regular samples')
  ax.set_ylabel(f'{comp.capitalize()} ({unit})');ax.grid(alpha=.2)
 axes[0,col].set_title('CBS-'+mode.upper());axes[1,col].set_xlabel('Seconds after retained journal start')
axes[0,0].legend(fontsize=8,loc='upper left');fig.suptitle('ETH1 WNOA: revisions versus nominal uncertainty\nNeither revisions nor uncertainty envelopes are ground-truth accuracy scores')
fig.tight_layout();fig.savefig(R/'prediction_and_uncertainty.png',dpi=170);plt.close(fig)

fig,axes=plt.subplots(2,2,figsize=(12,6),sharey='row',sharex='col')
for col,(mode,snap) in enumerate([('off',817),('on',201)]):
 b=chg[(chg['mode']==mode)&(chg.snapshot==str(snap))&chg.zone.isin(['covered','boundary','current'])].set_index('zone').loc[['covered','boundary','current']]
 x=np.arange(3)
 for row,vals,color,unit in [(0,b.position_m*100,'#267a9b','Position revision (cm)'),(1,b.velocity_mps,'#c98023','Velocity revision (m/s)')]:
  ax=axes[row,col];ax.bar(x,vals,width=.6,color=color);ax.set_xticks(x,['Covered support','Connected boundary','Current prediction']);ax.tick_params(axis='x',labelsize=8);ax.set_ylabel(unit);ax.grid(axis='y',alpha=.2)
 axes[0,col].set_title(f'CBS-{mode.upper()}, publication {snap} → {snap+1}')
fig.suptitle('Largest position-revision event in each mode: same-time changes across the prediction boundary')
fig.tight_layout();fig.savefig(R/'largest_revision_events.png',dpi=170);plt.close(fig)
print(json.dumps(verification,indent=2))
