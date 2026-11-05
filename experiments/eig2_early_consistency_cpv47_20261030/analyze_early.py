"""Saved early pose/cost/admission chronology; no covariance or scoring."""
import csv,hashlib,json,re
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');pins={}
sources={'off42':W/'runs/eig2_refresh_full_cpv42_20261029/prepared/full_off_source',
         'on42':W/'runs/eig2_refresh_full_cpv42_20261029/prepared/full_on_source',
         'on44':W/'runs/eig2_on_checkpoint_cpv44_20261029/prepared/full_on_source'}
def pin(p):
 with p.open('rb') as f:pins[str(p)]=hashlib.file_digest(f,'sha256').hexdigest()
 return p
def rot(v):
 w,x,y,z=v[3:];assert abs(w*w+x*x+y*y+z*z-1)<1e-10
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
  [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
  [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def angle(r):return float(np.degrees(np.arctan2(np.linalg.norm([r[2,1]-r[1,2],r[0,2]-r[2,0],r[1,0]-r[0,1]])*.5,(np.trace(r)-1)*.5)))
def movement(a,b):
 keys=sorted(set(a)&set(b));mats=[rot(b[k])@rot(a[k]).T for k in keys];u,_,vt=np.linalg.svd(sum(mats));f=np.eye(3);f[2,2]=np.linalg.det(u@vt);r=u@f@vt
 return dict(overlap=len(keys),common_rotation_deg=angle(r),common_tilt_deg=float(np.degrees(np.arccos(np.clip(r[2,2],-1,1)))),
  max_position_change_m=max(float(np.linalg.norm(b[k][:3]-a[k][:3])) for k in keys),
  previous_latest_position_change_m=float(np.linalg.norm(b[max(keys)][:3]-a[max(keys)][:3])),
  previous_latest_rotation_deg=angle(rot(b[max(keys)])@rot(a[max(keys)]).T))
def parse(root):
 windows={};edges={};lines={}
 for n,line in enumerate(pin(root/'estimators.log').open(),1):
  if 'KIMERA_CBS_REVISION_EDGE_ROW,' in line:
   f=line.split('KIMERA_CBS_REVISION_EDGE_ROW,',1)[1].strip().split(',');k=int(f[0])
   if k>35:break
   d=windows.setdefault(k,{})
   for i,raw in [(int(f[2]),f[8:15]),(int(f[3]),f[15:22])]:
    v=np.array(raw,float)
    if i in d:assert np.array_equal(d[i],v)
    d[i]=v
   lines.setdefault(k,n)
  if 'KIMERA_CBS_MEAN_ALIGNMENT_ROW,' in line:
   f=line.split('KIMERA_CBS_MEAN_ALIGNMENT_ROW,',1)[1].strip().split(',');k=int(f[0])
   if k<=35:edges.setdefault(k,[]).append(dict(sender=list(map(int,f[1:3])),receiver=list(map(int,f[3:5])),line=n,raw=f[10],aligned=f[11]))
 with pin(root/'kimera_odometry.csv').open() as f:
  stamps=[int(r['field.header.stamp']) for r in csv.DictReader(f) if r.get('field.header.seq','').isdigit()]
 with pin(root/'objective.csv').open() as f:cost=np.array(list(csv.reader(f)),float)
 assert np.array_equal(cost[:,0],np.arange(len(cost))) and len(stamps)==len(cost)
 events=[]
 for k,w in windows.items():
  event=dict(update=k,time_from_first_native_s=(stamps[k]-stamps[0])*1e-9,stamp_ns=stamps[k],line=lines[k],
   total_before=float(cost[k,4]),total_after=float(cost[k,5]),imu_before=float(cost[k,6]),imu_after=float(cost[k,7]),admissions=edges.get(k,[]))
  if k-1 in windows:event['old_window_change']=movement(windows[k-1],w)
  events.append(event)
 return windows,stamps,events
data={name:parse(root) for name,root in sources.items()};comparisons={}
for left,right in [('off42','on42'),('on42','on44')]:
 a,at,ae=data[left];b,bt,be=data[right];rr=[]
 for k in sorted(set(a)&set(b)):
  common=sorted(set(a[k])&set(b[k]));assert at[k]==bt[k]
  rr.append(dict(update=k,same_pose_values=all(np.array_equal(a[k][i],b[k][i]) for i in common),
   max_position_difference_m=max(float(np.linalg.norm(a[k][i][:3]-b[k][i][:3])) for i in common),
   max_rotation_difference_deg=max(angle(rot(a[k][i])@rot(b[k][i]).T) for i in common)))
 comparisons[left+'_vs_'+right]=dict(first_difference=next((r for r in rr if not r['same_pose_values']),None),rows=rr)
code=[W/'runs/kimera_on_incremental_refresh_cpv40_20261029/VioBackend.cpp',W/'runs/eig2_on_checkpoint_cpv44_20261029/VioBackend.cpp',
 W/'src/Kimera-VIO/include/kimera-vio/backend/VioBackendParams.h',W/'src/Kimera-VIO/src/imu-frontend/ImuFrontend.cpp',
 W/'src/Kimera-VIO/include/kimera-vio/imu-frontend/ImuFrontend.h',W/'src/Kimera-VIO/src/frontend/StereoVisionImuFrontend.cpp',
 W/'src/Kimera-VIO/src/pipeline/StereoImuPipeline.cpp',W/'src/gtsam/gtsam/navigation/PreintegrationBase.cpp',
 W/'src/gtsam/gtsam/navigation/ManifoldPreintegration.cpp',W/'src/gtsam/gtsam/navigation/ImuFactor.cpp']
for p in code:pin(p)
for root in sources.values():pin(root/'kimera_profile/BackendParams.yaml')
out=dict(sources={n:str(p) for n,p in sources.items()},events={n:v[2] for n,v in data.items()},comparisons=comparisons,input_sha256=pins,
 limits=['Full X/V/b/PIM checkpoints start at21, so early pose/cost chronology is not a complete first-admission causal replay.',
 'Current frontend/GTSAM source is separately hashed code-inspection evidence, not assumed to be an archived exact compiler input.'])
with (H/'early_analysis.json').open('x') as f:json.dump(out,f,indent=2,allow_nan=False)
for pair,c in comparisons.items():print(pair,c['first_difference'])
for name,(_,_,events) in data.items():
 print(name,'first admission',next((e for e in events if e['admissions']),None))
 for e in events:
  if e['update'] in [5,6,7,10,15,20,21,22,24,25]:print(name,e['update'],e.get('old_window_change',{}).get('common_tilt_deg'),e['imu_after'])
