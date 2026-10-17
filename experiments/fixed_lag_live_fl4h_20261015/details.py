"""Read-only factor payload, covariance, solver/recovery and GT diagnostics."""
import runpy, re, json, sys
from collections import Counter, defaultdict
from pathlib import Path
import numpy as np
import rosbag

A=runpy.run_path(str(Path(__file__).with_name('analyze.py')))
ROOT,OUT,BASE=A['ROOT'],A['OUT'],A['BASE'];st=A['st'];between=A['between'];gt=A['gt'];err=A['err']
native,snapshots,versions,align=A['all_data']['on']
src=ROOT/'pilot120_on_source'
log,=[p for p in (src/'ros_logs').glob('*/*stdout.log') if 'kimera_vio_ros_node' in p.name and p.parent.name!='latest']
lines=log.read_text().splitlines()
sys.path.insert(0,'/workspace/cbs_gtsam4.3/runs/gnssfgo_e1_e2_live160_validation_20261011')
from admission_contract import verify_candidates,kimera_outcomes
wire={}
with rosbag.Bag(str(src/'recordings/cbs.bag')) as bag:
 for topic,msg,stamp in bag.read_messages(topics=['/kimera/cbs/odom_belief_in']):
  for b in msg.beliefs:wire.setdefault((b.from_pose_index,b.to_pose_index),[]).append(dict(mu=tuple(b.relative_mu),cov=tuple(b.covariance),ti=b.from_stamp_sec,tj=b.to_stamp_sec))
candidates=[]
for line in (src/'estimators.log').open():
 if 'KIMERA_CBS_MEAN_ALIGNMENT_ROW,' in line:
  v=line.split('KIMERA_CBS_MEAN_ALIGNMENT_ROW,')[1].strip().split(',')
  candidates.append(dict(update=int(v[0]),sender=tuple(map(int,v[1:3])),receiver=tuple(map(int,v[3:5])),times=tuple(float(x)*1e-9 for x in v[5:10]),raw=tuple(A['vec'](v[10])),aligned=tuple(A['vec'](v[11])),cov=tuple(A['vec'](v[12]))))
verified=verify_candidates(candidates,kimera_outcomes(lines),wire)

def logpose(z):
 p,R=z;a=np.arccos(np.clip((np.trace(R)-1)/2,-1,1));assert a<1., 'diagnostic restricted to small relative errors'
 w=np.array([R[2,1]-R[1,2],R[0,2]-R[2,0],R[1,0]-R[0,1]])*(.5 if a<1e-8 else a/(2*np.sin(a)));W=A['skew'](w)
 V=np.eye(3)+.5*W+W@W/6 if a<1e-8 else np.eye(3)+(1-np.cos(a))/a**2*W+(a-np.sin(a))/a**3*W@W
 return np.r_[w,np.linalg.solve(V,p)]
rows=[]
for u,a in align.items():
 sf,stt,rf,rt,now=a['ts'];Z=a['aligned'];truth=between(gt(rf),gt(rt));e=logpose(between(Z,truth));C=a['cov']
 r=dict(update=u,time=native[u][0]-BASE,nis_reference=float(e@np.linalg.solve(C,e)),translation_reference_norm=float(np.linalg.norm(e[3:])),rotation_reference_deg=float(np.degrees(np.linalg.norm(e[:3]))),sigma_translation_rms_axis_m=float(np.sqrt(np.trace(C[3:,3:])/3)),sigma_rotation_rms_axis_deg=float(np.degrees(np.sqrt(np.trace(C[:3,:3])/3))))
 rows.append(r)
byu={r['update']:r for r in rows};groups=defaultdict(dict);summaries={};guards=[];post={};pre={};u=None
for ln,line in enumerate(lines,1):
 if 'CBS_BPSAM_ACTIVE_WINDOW_LIFECYCLE_ROW,k,' in line:u=int(float(line.split('CBS_BPSAM_ACTIVE_WINDOW_LIFECYCLE_ROW,')[1].split(',')[1]))
 if 'CBS_COVARIANCE_EXCHANGE_GLOBAL_FACTOR_GROUP_ROW,G2K,' in line:
  v=line.split('CBS_COVARIANCE_EXCHANGE_GLOBAL_FACTOR_GROUP_ROW,')[1].split(',');key=v[8]
  g=groups[u].setdefault(key,dict(family=key,count=0,cost=0.,typed_groups=[]));g['count']+=int(v[10]);g['cost']+=float(v[11]);g['typed_groups'].append(dict(type=v[9],count=int(v[10]),cost=float(v[11]),hessian_norm=float(v[14]),gradient_norm=float(v[16])))
 if 'CBS_COVARIANCE_EXCHANGE_SUMMARY_ROW,G2K,' in line:
  v=line.split('CBS_COVARIANCE_EXCHANGE_SUMMARY_ROW,')[1].split(',');summaries[u]=dict(nis=float(v[28]),cost=float(v[29]),hessian_norm=float(v[31]),gradient_norm=float(v[32]))
 if ',active_window_rollback,' in line or ',active_window_staged_factor_dropped,' in line or ',active_window_active_residual_observed,' in line or ',active_window_staged_residual_observed,' in line or 'action=drop_before_stage' in line:
  guards.append(dict(update=u,time=native[u][0]-BASE,line=ln,text=line))
 if 'CBS_ODOM_TEMPORARY_POSTSOLVE_RESIDUAL_ROW,G2K,' in line:
  v=line.split('CBS_ODOM_TEMPORARY_POSTSOLVE_RESIDUAL_ROW,')[1].split(',');post[u]=dict(raw=v,line=ln)
 if 'CBS_ODOM_PREINJECTION_RESIDUAL_ROW,G2K,' in line:
  v=line.split('CBS_ODOM_PREINJECTION_RESIDUAL_ROW,')[1].split(',');pre[u]=dict(raw=v,line=ln)

drop_updates={g['update'] for g in guards if 'staged_factor_dropped' in g['text']}
allrows=A['result']['on']['rows'];trace={r['update']:r for r in allrows}
examples=[]
for u in [368,438,479,482,497,498,534,549,551]:
 r=dict(trace[u]);r.update(byu[u]);r['candidate_dropped_by_recovery']=u in drop_updates;r['guards']=[g for g in guards if g['update']==u];r['pre_solve_log']=pre.get(u);r['first_trial_post_solve_log']=post.get(u);examples.append(r)

report=dict(wire_verification=dict(candidates=len(verified),constructed=sum(r['outcome']=='constructed' for r in verified),pre_stage_drops=sum(r['outcome']=='drop_before_stage' for r in verified),exact_full_covariance_and_raw_mean=True),covariance_reference_diagnostic={k:st([r[k] for r in rows]) for k in rows[0] if k not in ['update','time']},fraction_reference_mahalanobis_above_12_592=float(np.mean([r['nis_reference']>12.592 for r in rows])),guards=guards,dropped_candidate_count=len(drop_updates),candidate_never_rollback_count=len(verified)-len(drop_updates),sampled_existing_family_costs={str(u):dict(time=native[u][0]-BASE,new_external=summaries.get(u),existing=gg) for u,gg in groups.items()},examples=examples,all_reference_covariance_rows=rows)
target=OUT/'factor_details_v2.json';data=json.dumps(report,indent=2).encode()
if target.exists():assert target.read_bytes()==data, 'Existing diagnostic differs'
else:
 with target.open('xb') as f:f.write(data)
print('PAYLOAD',report['wire_verification'],'DROPPED',len(drop_updates),'ROLLBACKS',sum('active_window_rollback,' in g['text'] for g in guards))
print('COVARIANCE_VS_REFERENCE',report['covariance_reference_diagnostic'])
for u,g in groups.items():
 print('EXISTING_GRAPH',u,round(native[u][0]-BASE,2),{k:(v['count'],round(v['cost'],2)) for k,v in g.items()})
print('EXAMPLES',[(r['update'],round(r['time'],3),round(r['raw_error_m'],5),round(r['aligned_error_m'],5),round(r['prior_injected_edge_error_m'],5),round(r['after_injected_edge_error_m'],5),r['candidate_dropped_by_recovery']) for r in examples])

checks=[]
for u,a in align.items():
 r=dict(update=u,dropped=u in drop_updates)
 for tag,sn,logged in [('pre',snapshots[u-1],pre[u]),('post',snapshots[u],post[u])]:
  z=between(sn[a['ri']],sn[a['rj']]);res=logpose(between(a['aligned'],z));v=logged['raw']
  r[tag+'_rotation_norm_difference']=abs(float(np.linalg.norm(res[:3]))-float(v[8]));r[tag+'_translation_norm_difference']=abs(float(np.linalg.norm(res[3:]))-float(v[9]))
 checks.append(r)
summary=dict(max_pre_translation_difference=max(r['pre_translation_norm_difference'] for r in checks),max_pre_rotation_difference=max(r['pre_rotation_norm_difference'] for r in checks),max_retained_post_translation_difference=max(r['post_translation_norm_difference'] for r in checks if not r['dropped']),max_retained_post_rotation_difference=max(r['post_rotation_norm_difference'] for r in checks if not r['dropped']),rows=checks)
target=OUT/'stage_checks.json';data=json.dumps(summary,indent=2).encode()
if target.exists():assert target.read_bytes()==data
else:
 with target.open('xb') as f:f.write(data)
print('STAGE_CHECKS',{k:v for k,v in summary.items() if k!='rows'})
