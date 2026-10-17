"""Read-only FL4g G->K mechanism audit. Run with ROS Noetic Python."""
import csv, json, math, re, sys, os
from pathlib import Path
from collections import Counter
import numpy as np

ROOT=Path(os.environ.get('FL4H_CAPTURE','/workspace/cbs_gtsam4.3/runs/gnssfgo_eth1_fixed_lag_live120_fl4g_20261015/prepared'))
OUT=Path(os.environ.get('FL4H_OUT','/workspace/cbs_gtsam4.3/runs/gnssfgo_eth1_fixed_lag_live120_fl4g_20261015/audit_fl4h'))
sys.path.insert(0,'/workspace/cbs_gtsam4.3/src/cbsms/tools')
import score_grandtour_relative_pose as rel
import score_grandtour_prism as prism
BASE=1727775007.
GT=Path('/workspace/cbs_gtsam4.3/src/datasets/GrandTour/2024-10-01-11-29-55_cpt7_ie_tc.bag')
reference=rel.sort_unique(*rel.load_reference_bag(GT,'/boxi/inertial_explorer/tc/odometry')[:3])

def skew(v):
 x,y,z=v;return np.array([[0,-z,y],[z,0,-x],[-y,x,0.]])
def exp(x):
 x=np.asarray(x);w=x[:3];rho=x[3:];a=np.linalg.norm(w);W=skew(w)
 if a<1e-8: R=np.eye(3)+W+.5*W@W;V=np.eye(3)+.5*W+W@W/6
 else: R=np.eye(3)+math.sin(a)/a*W+(1-math.cos(a))/a**2*W@W;V=np.eye(3)+(1-math.cos(a))/a**2*W+(a-math.sin(a))/a**3*W@W
 return V@rho,R
def pose(v):
 v=list(map(float,v));return np.array(v[:3]),rel.quaternion_to_matrix(v[4:7]+[v[3]])
def between(a,b):return a[1].T@(b[0]-a[0]),a[1].T@b[1]
def compose(a,b):return a[0]+a[1]@b[0],a[1]@b[1]
def err(a,b):
 return float(np.linalg.norm(a[0]-b[0])),math.degrees(math.acos(float(np.clip((np.trace(a[1].T@b[1])-1)/2,-1,1))))
def gt(t):return rel.interpolate_pose(*reference,t,.05)
def st(x):
 a=np.asarray(x,float);return dict(n=len(a),median=float(np.median(a)),rms=float(np.sqrt(np.mean(a*a))),p95=float(np.percentile(a,95)),max=float(np.max(a))) if len(a) else None
def vec(s):return np.array([float(x) for x in re.split('[:;]',s)])
def ate(poses,times):
 E=np.array([x[0] for x in poses]);G=np.array([gt(t)[0] for t in times]);A,b=prism.rigid_kabsch(E,G);e=np.linalg.norm(E@A.T+b-G,axis=1)
 return dict(ATE_CPT7_m=float(np.sqrt(np.mean(e*e))),max_error_m=float(e.max()))
def raw_trajectory(p):
 with p.open() as f:rows=list(csv.DictReader(f))
 ids=[int(r['field.header.seq']) for r in rows]
 t,pos,q=rel.load_odometry_csv(p)
 return {i:(float(t[j]),(pos[j],rel.quaternion_to_matrix(q[j]))) for j,i in enumerate(ids)}

result={};all_data={}
for mode in ('off','on'):
 src=ROOT/f'pilot120_{mode}_source';native=raw_trajectory(src/'kimera_odometry.csv')
 snapshots={};edge_versions={};align={};merge={};warnings=[];outputgap={};pending_warnings=[]
 for lineno,line in enumerate((src/'estimators.log').open(),1):
  if 'Stereo tracking failure' in line:pending_warnings.append(('stereo_failure',lineno))
  if 'KIMERA_CBS_REVISION_EDGE_ROW,' in line:
   v=line.split('KIMERA_CBS_REVISION_EDGE_ROW,')[1].strip().split(',');u,i,j=int(v[0]),int(v[2]),int(v[3]);ti,tj=float(v[4]),float(v[5])
   a,b=pose(v[8:15]),pose(v[15:22]);snapshots.setdefault(u,{})[i]=a;snapshots[u][j]=b
   Z=exp(list(map(float,v[22:28])));edge_versions.setdefault((i,j),[]).append((u,Z,int(v[6])))
  elif 'KIMERA_CBS_MEAN_ALIGNMENT_ROW,' in line:
   v=line.split('KIMERA_CBS_MEAN_ALIGNMENT_ROW,')[1].strip().split(',');u,si,sj,ri,rj=map(int,v[:5]);ts=np.array(list(map(int,v[5:10])),dtype=np.int64)*1e-9
   align[u]=dict(si=si,sj=sj,ri=ri,rj=rj,ts=ts,raw=exp(vec(v[10])),aligned=exp(vec(v[11])),cov=vec(v[12]).reshape(6,6),line=lineno)
  elif 'KIMERA_CBS_POSE_MERGE_ROW,' in line:
   v=line.split('KIMERA_CBS_POSE_MERGE_ROW,')[1].strip().split(',');merge[int(v[0])]=list(map(float,v[2:]))
  elif 'KIMERA_BACKEND_POSE_PATH_ROW,' in line:
   v=line.split('KIMERA_BACKEND_POSE_PATH_ROW,')[1].strip().split(',');u=int(v[1]);outputgap[u]=float(v[19])
   for w,ln in pending_warnings:warnings.append(dict(kind=w,update=u,t=native[u][0]-BASE,line=ln))
   pending_warnings=[]
 latest=[];latest_t=[];rows=[];chain_first=(np.zeros(3),np.eye(3));chain_errors=[]
 for u in sorted(native):
  t,N=native[u];snap=snapshots.get(u,{})
  r=dict(update=u,time=t-BASE,reported_state_output_gap_m=outputgap.get(u))
  if u in snap:latest.append(snap[u]);latest_t.append(t)
  if u and (u-1) in native and u in snap and u-1 in snap:
   prev= native[u-1];truth=between(gt(prev[0]),gt(t));step=between(prev[1],N);birth=between(snap[u-1],snap[u]);te,rot_error=err(step,truth)
   r.update(native_step_translation_error_m=te,native_step_rotation_error_deg=rot_error,birth_output_difference=err(step,birth)[0],step_translation_m=float(np.linalg.norm(step[0])))
   revs=edge_versions.get((u-1,u),[])
   if revs:
    r.update(first_edge_error_m=err(revs[0][1],truth)[0],last_edge_error_m=err(revs[-1][1],truth)[0],last_edge_error_deg=err(revs[-1][1],truth)[1],first_last_change_m=err(revs[0][1],revs[-1][1])[0])
    if len(revs)>1:r['second_edge_error_m']=err(revs[1][1],truth)[0]
  if u in align:
   a=align[u];sf,stt,rf,rt,now=a['ts'];raw_truth=between(gt(sf),gt(stt));target_truth=between(gt(rf),gt(rt))
   r.update(sender_from=a['si'],sender_to=a['sj'],receiver_from=a['ri'],receiver_to=a['rj'],endpoint_shift_ms=(rf-sf)*1e3,applied_age_s=now-stt,raw_error_m=err(a['raw'],raw_truth)[0],raw_error_deg=err(a['raw'],raw_truth)[1],aligned_error_m=err(a['aligned'],target_truth)[0],aligned_error_deg=err(a['aligned'],target_truth)[1],mean_alignment_translation_change_m=err(a['raw'],a['aligned'])[0],cov_translation_trace=float(np.trace(a['cov'][3:,3:])))
   for name,sn in [('prior',snapshots.get(u-1,{})),('after',snap)]:
    if a['ri'] in sn and a['rj'] in sn:
     z=between(sn[a['ri']],sn[a['rj']]);r[f'{name}_injected_edge_error_m']=err(z,target_truth)[0];r[f'{name}_injected_edge_residual_m']=err(z,a['aligned'])[0]
   if u in merge:r.update(dict(zip(('merge_pre_rot','merge_pre_trans','merge_post_rot','merge_post_trans'),merge[u])))
  rows.append(r)
 bins=[]
 for lo,hi in [(0,30),(30,45),(45,60),(60,75),(75,90),(90,105),(105,120)]:
  rr=[r for r in rows if lo<=r['time']<hi]
  keys=['native_step_translation_error_m','native_step_rotation_error_deg','reported_state_output_gap_m','raw_error_m','aligned_error_m','prior_injected_edge_error_m','after_injected_edge_error_m','merge_pre_trans','merge_post_trans','first_edge_error_m','second_edge_error_m','last_edge_error_m','first_last_change_m']
  bins.append(dict(window=[lo,hi],stats={k:st([r[k] for r in rr if r.get(k) is not None]) for k in keys},stereo_failures=sum(lo<=w['t']<hi for w in warnings)))
 result[mode]=dict(native_ate=ate([v[1] for v in native.values()],[v[0] for v in native.values()]),latest_graph_ate=ate(latest,latest_t),stereo_failures=warnings,bins=bins,largest_native_steps=sorted([r for r in rows if 'native_step_translation_error_m' in r],key=lambda r:r['native_step_translation_error_m'],reverse=True)[:15],largest_alignment_error=sorted([r for r in rows if 'aligned_error_m' in r],key=lambda r:r['aligned_error_m'],reverse=True)[:10],rows=rows)
 all_data[mode]=(native,snapshots,edge_versions,align)
 import io
 f=io.StringIO(newline='');keys=sorted(set().union(*(set(r) for r in rows)));w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(rows)
 target=OUT/f'{mode}_update_trace.csv';data=f.getvalue().encode()
 if target.exists():assert target.read_bytes()==data, 'Existing diagnostic differs'
 else:
  with target.open('xb') as dest:dest.write(data)

target=OUT/'mechanism_summary.json';data=json.dumps(result,indent=2).encode()
if target.exists():assert target.read_bytes()==data, 'Existing diagnostic differs'
else:
 with target.open('xb') as f:f.write(data)
for mode,d in result.items():
 print(mode,'NATIVE',d['native_ate'],'LATEST_GRAPH',d['latest_graph_ate'],'stereo warnings',len(d['stereo_failures']))
 for b in d['bins']:
  print(b['window'],'warnings',b['stereo_failures'],{k:None if v is None else round(v['rms'],5) for k,v in b['stats'].items()})
 print('worst native steps',[(r['update'],r['time'],r['native_step_translation_error_m']) for r in d['largest_native_steps'][:5]])
