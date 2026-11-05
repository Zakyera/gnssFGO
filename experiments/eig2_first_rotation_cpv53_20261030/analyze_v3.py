"""Single-edge raw gyro/transport audit. No optimizer or covariance calculation."""
import csv,hashlib,importlib.util,json,re,sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as SO3
from rosbags.highlevel import AnyReader
sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');K=W/'runs'/H.name
P=W/'runs/eig2_hesai_insertion_cpv51_20261030/prepared';S=P/'full_on_source'
P52=W/'runs/eig2_first_factor_cpv52_20261030/attempt_compile1/results'
pins={}
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def pin(p):
 p=Path(p).resolve();pins[str(p)]=sha(p);return p
def save(n,d):
 with (H/n).open('x') as f:json.dump(d,f,indent=2,allow_nan=False)
def readcsv(p):
 with pin(p).open() as f:return list(csv.DictReader(f))
spec=importlib.util.spec_from_file_location('geometry',pin(H.parent/'eig2_hesai_onset_cpv50_20261030/analyze.py'))
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
def deg(R):return float(np.degrees(SO3.from_matrix(R).magnitude()))
def describe(R):return dict(matrix=R.tolist(),rotvec_deg=np.degrees(SO3.from_matrix(R).as_rotvec()).tolist(),angle_deg=deg(R))
def err(A,B):return describe(A.T@B)
def stamp(msg):return int(msg.header.stamp.sec)*10**9+int(msg.header.stamp.nanosec)

assert json.loads(pin(H/'extraction_passed.json').read_text())['passed']
assert not (H/'analysis.json').exists()
source=a.parse_source(S/'estimators.log');pins.update(a.PINS);edge=source['aligned'][6,4,5]
sf,st,rf,rt,causal=edge['times'];assert edge['ids']==[5,7,4,5]
assert max(sf,st,rf,rt)<causal and rf<sf<rt<st
raw=edge['raw'];aligned=edge['mean'];frozen=source['frozen'][6,4,5]['mean']
actual=np.loadtxt(pin(K/'extracted/factor_matrix.txt'))
assert np.max(abs(actual-aligned))<1e-12 and np.array_equal(frozen,aligned)

def exports(mode):
 logs=[p for p in (P/f'full_{mode}_source/ros_logs').glob('*/glim_rosnode-1-stdout.log') if p.parent.name!='latest']
 assert len(logs)==1
 matches=[]
 for n,line in enumerate(pin(logs[0]).open(),1):
  if 'GLIM_CBS_EXPORT_ENDPOINT_ROW,' not in line:continue
  f=a.ANSI.sub('',line.split('GLIM_CBS_EXPORT_ENDPOINT_ROW,',1)[1]).strip().split(',')
  if list(map(int,f[3:5]))!=[5,7]:continue
  poses=[]
  for offset in (7,14):
   T=np.eye(4);T[:3,3]=np.array(f[offset:offset+3],float);T[:3,:3]=SO3.from_quat(np.array(f[offset+3:offset+7],float)).as_matrix();poses.append(T)
  mean=a.exp(f[21].split(';'));computed=a.inv(poses[0])@poses[1]
  assert np.max(abs(mean-computed))<1e-12
  matches.append(dict(line=n,current=int(f[1]),times=[round(float(x)*1e9) for x in f[5:7]],
                      endpoint_mean=mean.tolist(),reconstructed_mean=computed.tolist(),poses=[x.tolist() for x in poses]))
 assert len(matches)==1
 return matches[0]
ex={mode:exports(mode) for mode in ('off','on')}
assert ex['on']['times']==[sf,st]
assert np.max(abs(np.array(ex['on']['endpoint_mean'])-raw))<1e-12
incoming=[]
with AnyReader([pin(S/'recordings/cbs.bag')]) as reader:
 for c,t,b in reader.messages(connections=[c for c in reader.connections if c.topic=='/kimera/cbs/odom_belief_in']):
  m=reader.deserialize(b,c.msgtype)
  for z in m.beliefs:
   if (int(z.from_pose_index),int(z.to_pose_index))!=(5,7):continue
   mean=a.exp(np.array(z.relative_mu,dtype=float,copy=True))
   assert np.max(abs(mean-raw))<1e-12
   assert [round(z.from_stamp_sec*1e9),round(z.to_stamp_sec*1e9)]==[sf,st]
   incoming.append(dict(frame=z.header.frame_id,source=int(z.source_agent),record_ns=t,mean=mean.tolist()))
# Header is the configured world/map label, not the body convention of a
# relative transform. Check the actual effective contract explicitly.
effective=pin(S/'estimators.log').read_text()
assert len(incoming)==1 and incoming[0]['frame']=='map' and incoming[0]['source']==ord('g')
for required in ('/glim_rosnode/glim_cbs_frame_id: map',
                 '/glim_rosnode/glim_cbs_child_frame_id: cpt7_imu',
                 '/kimera_vio_ros/kimera_vio_ros_node/base_link_frame_id: cpt7_imu',
                 '/kimera_vio_ros/kimera_vio_ros_node/cbs_external_pose_frame_id: cpt7_imu'):
 assert required in effective,required

audit=json.loads(pin(P/'input_audit.json').read_text());epoch=audit['window']['start_ns']
ib=Path(audit['bag_files']['cpt7_imu']['path']);cb=Path(audit['bag_files']['alphasense']['path'])
assert sha(pin(ib))==audit['bag_files']['cpt7_imu']['sha256']
imu=[];frames=set()
with AnyReader([ib]) as reader:
 for c,t,b in reader.messages(connections=[c for c in reader.connections if c.topic=='/boxi/cpt7/imu']):
  m=reader.deserialize(b,c.msgtype);ns=stamp(m)
  if epoch-500_000_000<=ns<=causal+500_000_000:
   frames.add(m.header.frame_id);imu.append([ns,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z])
assert frames=={'cpt7_imu'}
tt=np.array([r[0] for r in imu],dtype=np.int64);ww=np.array([r[1:] for r in imu]);assert (np.diff(tt)>0).all()
assert sha(pin(cb))==audit['bag_files']['alphasense']['sha256']
camera=[]
with AnyReader([cb]) as reader:
 connections=[c for c in reader.connections if c.topic=='/boxi/alphasense/front_left/image_raw/compressed'];assert len(connections)==1
 for c,t,b in reader.messages(connections=connections,start=epoch-500_000_000,end=causal+500_000_000):
  m=reader.deserialize(b,c.msgtype);camera.append(stamp(m))
assert all(t in camera for t in (rf,rt,causal)),(rf,rt,causal,camera)
assert camera==sorted(set(camera))

def interpolation(ns):
 j=int(np.searchsorted(tt,ns));assert 0<j<len(tt)
 if int(tt[j])==ns:return ww[j].copy()
 ratio=(ns-int(tt[j-1]))/(int(tt[j])-int(tt[j-1]))
 return ww[j-1]+ratio*(ww[j]-ww[j-1])

# Exact native frontend packet convention: interpolated image endpoints,
# strict interior raw samples, then hold each rate on its outgoing interval.
segments=[]
for left,right in zip(camera,camera[1:]):
 if left<tt[0] or right>tt[-1]:continue
 bounds=[left,*[int(t) for t in tt if left<t<right],right]
 rates=[interpolation(left),*[ww[i] for i,t in enumerate(tt) if left<t<right]]
 for b,e,w in zip(bounds,bounds[1:],rates):segments.append((b,e,w))
assert all(a0[1]==b0[0] for a0,b0 in zip(segments,segments[1:]))

def integrate(begin,end,bias,method='native'):
 if begin>end:return integrate(end,begin,bias,method).T
 R=np.eye(3);cursor=begin
 if method=='native':
  for lo,hi,w in segments:
   if hi<=begin or lo>=end:continue
   left=max(begin,lo);right=min(end,hi);assert left==cursor
   R=R@SO3.from_rotvec((w-bias)*((right-left)*1e-9)).as_matrix();cursor=right
 else:
  bounds=[begin,*[int(t) for t in tt if begin<t<end],end]
  for lo,hi in zip(bounds,bounds[1:]):
   w=(interpolation(lo)+interpolation(hi))/2
   R=R@SO3.from_rotvec((w-bias)*((hi-lo)*1e-9)).as_matrix();cursor=hi
 assert cursor==end
 return R

def values(p):
 with pin(p).open() as f:return {r[0]:np.array(r[1:],float) for r in csv.reader(f)}
initial=values(P52/'initial_values.csv');b4=initial['b4'][3:];b5=initial['b5'][3:]
kimera_R=a.inv(a.pose(initial['x4']))@a.pose(initial['x5'])
left=integrate(rf,sf,b4);right=integrate(st,rt,b5)
predicted=left@raw[:3,:3]@right
bridge_error=deg(predicted.T@aligned[:3,:3]);assert bridge_error<1e-7,bridge_error
rows=readcsv(K/'extracted/pim.csv');r45=[r for r in rows if r['from']=='4' and r['to']=='5'];assert len(r45)==1;r45=r45[0]
hat=np.array([r45[k] for k in ('bgx','bgy','bgz')],float)
current=np.array([r45[k] for k in ('current_bgx','current_bgy','current_bgz')],float);assert np.array_equal(current,b4)
def matrix(row,prefix):return np.array([row[prefix+str(i)] for i in range(9)],float).reshape(3,3)
pim_R=matrix(r45,'rawR');pim_corrected_R=matrix(r45,'correctedR')
reintegrated=integrate(rf,rt,hat);pim_error=deg(pim_R.T@reintegrated)
assert pim_error<1e-7,pim_error
assert abs(float(r45['dt'])-(rt-rf)*1e-9)<1e-12
assert np.max(abs(matrix(r45,'bodyR')-np.eye(3)))<1e-12

# Reporting geometry tests: SO3 inverse, world-frame invariance, interval split.
random=np.random.default_rng(53)
for _ in range(100):
 x,y,z=[SO3.from_rotvec(random.normal(size=3)*.2).as_matrix() for i in range(3)]
 assert deg((x.T@y).T@((z@x).T@(z@y)))<1e-10
 assert deg(x@x.T)<1e-10
assert deg(integrate(rf,rt,b4).T@(integrate(rf,sf,b4)@integrate(sf,rt,b4)))<1e-10

rotations=dict(GLIM_native_sender=raw[:3,:3],GLIM_aligned_receiver=aligned[:3,:3],
              GLIM_OFF_same_sender=np.array(ex['off']['endpoint_mean'])[:3,:3],
              gyro_sender_zero=integrate(sf,st,np.zeros(3)),gyro_sender_b4=integrate(sf,st,b4),
              gyro_receiver_zero=integrate(rf,rt,np.zeros(3)),gyro_receiver_b4=integrate(rf,rt,b4),
              gyro_sender_trapezoidal_b4=integrate(sf,st,b4,'trapezoidal'),
              gyro_receiver_trapezoidal_b4=integrate(rf,rt,b4,'trapezoidal'),
              saved_PIM_biasHat=pim_R,saved_PIM_current_bias=pim_corrected_R,
              Kimera_before_insertion=kimera_R[:3,:3],bridge_left=left,bridge_right=right)
pairs=[('GLIM_native_sender','gyro_sender_b4'),('GLIM_native_sender','gyro_sender_zero'),
       ('GLIM_OFF_same_sender','gyro_sender_b4'),('GLIM_native_sender','GLIM_OFF_same_sender'),
       ('GLIM_aligned_receiver','gyro_receiver_b4'),('GLIM_aligned_receiver','Kimera_before_insertion'),
       ('Kimera_before_insertion','gyro_receiver_b4'),('saved_PIM_current_bias','gyro_receiver_b4'),
       ('gyro_sender_b4','gyro_sender_trapezoidal_b4'),('gyro_receiver_b4','gyro_receiver_trapezoidal_b4'),
       ('GLIM_native_sender','GLIM_aligned_receiver')]
comparisons={x+'__vs__'+y:err(rotations[x],rotations[y]) for x,y in pairs}
support=dict(imu_count=len(tt),imu_first_ns=int(tt[0]),imu_last_ns=int(tt[-1]),imu_min_dt_ns=int(np.min(np.diff(tt))),
             imu_max_dt_ns=int(np.max(np.diff(tt))),imu_frames=list(frames),camera_count=len(camera),
             camera_stamps=camera,segments=len(segments),gyro_bias_preupdate_b4=b4.tolist(),gyro_bias_preupdate_b5=b5.tolist(),
             saved_PIM_biasHat=hat.tolist(),PIM_deltaT=float(r45['dt']))
out=dict(edge=[5,7,4,5],times=dict(sender_from=sf,sender_to=st,receiver_from=rf,receiver_to=rt,causal_limit=causal),
         sender_duration_s=(st-sf)*1e-9,receiver_duration_s=(rt-rf)*1e-9,endpoint_offsets_ms=[(sf-rf)*1e-6,(st-rt)*1e-6],
         support=support,exports=ex,incoming=incoming,rotations={k:describe(r) for k,r in rotations.items()},comparisons=comparisons,
         gates=dict(native_export_endpoint_match=True,transport_match=True,aligned_actual_factor_match=True,
                    reconstructed_bridge_error_deg=bridge_error,raw_reintegration_PIM_error_deg=pim_error,
                    raw_reintegration_current_bias_vs_PIM_deg=deg(pim_corrected_R.T@integrate(rf,rt,b4)),
                    rotation_fixtures=100),input_sha256=pins,
         scope='First edge only; no source replay/optimizer/covariance analysis.',
         limitations=['Raw gyro is not independent truth; pre-update Kimera bias is estimated, not true.',
                     'Zero-bias and pre-update-bias versions reported without fitting.',
                     'GLIM endpoint poses are MAIN estimates at export, not necessarily first publications.',
                     'No contemporaneous GLIM full bias/PIM snapshot loaded. Final map bias is not substituted.'])
save('analysis.json',out)
save('raw_samples.json',dict(imu=imu,camera_stamps=camera,source_bags=[str(ib),str(cb)]))
print(json.dumps({k:v for k,v in out.items() if k not in ('input_sha256','exports','rotations','incoming')},indent=2))
for k,r in rotations.items():print(k,np.degrees(SO3.from_matrix(r).as_rotvec()))
