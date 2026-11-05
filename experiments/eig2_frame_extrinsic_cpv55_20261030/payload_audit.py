"""Audit saved runtime means in both directions; never run an estimator."""
import hashlib,importlib.util,json,re,sys
from pathlib import Path
from collections import defaultdict
import numpy as np
from scipy.spatial.transform import Rotation
from rosbags.highlevel import AnyReader
sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');pins={}
def pin(p):
 p=Path(p)
 with p.open('rb') as f:pins[str(p)]=hashlib.file_digest(f,'sha256').hexdigest()
 return p
s=importlib.util.spec_from_file_location('geometry',pin(H.parent/'eig2_hesai_onset_cpv50_20261030/analyze.py'));a=importlib.util.module_from_spec(s);s.loader.exec_module(a)
def put(n,v):
 with (H/n).open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
def decode(line,tag):return a.ANSI.sub('',line.split(tag+',',1)[1]).strip().split(',')
def near(A,candidates):return min(float(np.max(abs(A-B))) for B in candidates)
def stats(x):return dict(n=len(x),max_matrix_error=max(x,default=0.))
results={}
for case,run in [('Hesai_CP49','eig2_hesai_refresh_full_cpv49_20261030'),('Hesai_CP54','eig2_glim_first_edge_cpv54_20261030'),('Velodyne_CPV4','gnssfgo_eig2_velodyne_full_cpv4_20261024')]:
 for mode in ['off','on']:
  S=W/'runs'/run/'prepared'/f'full_{mode}_source';exports={'g':defaultdict(list),'k':defaultdict(list)};parity={'g':[],'k':[]};aligned={'g':[],'k':[]};frozen={};native_observer=[]
  glogs=[p for p in (S/'ros_logs').glob('*/glim_rosnode-1-stdout.log') if p.parent.name!='latest'];assert len(glogs)==1
  for line in pin(S/'estimators.log').open():
   if 'KIMERA_CBS_REVISION_EDGE_ROW,' in line:
    f=decode(line,'KIMERA_CBS_REVISION_EDGE_ROW');assert len(f)==64
    i,j=int(f[2]),int(f[3]);T1=a.pose(f[8:15]);T2=a.pose(f[15:22]);M=a.exp(f[22:28]);e=float(np.max(abs(a.inv(T1)@T2-M)));parity['k'].append(e);exports['k'][i,j].append(M)
   if 'KIMERA_CBS_MEAN_ALIGNMENT_ROW,' in line:
    f=decode(line,'KIMERA_CBS_MEAN_ALIGNMENT_ROW');aligned['k'].append((tuple(map(int,f[1:3])),a.exp(f[10].split(';')),a.exp(f[11].split(';')),tuple(map(int,[f[0],f[3],f[4]]))))
   if 'FL4M_FROZEN_FACTOR_ROW,' in line:
    f=decode(line,'FL4M_FROZEN_FACTOR_ROW');frozen[tuple(map(int,[f[0],f[3],f[4]]))]=a.exp(f[5].split(';'))
  for line in pin(glogs[0]).open():
   if 'GLIM_CBS_EXPORT_ENDPOINT_ROW,' in line:
    f=decode(line,'GLIM_CBS_EXPORT_ENDPOINT_ROW');i,j=int(f[3]),int(f[4]);pp=[]
    for off in [7,14]:
     T=np.eye(4);T[:3,3]=np.array(f[off:off+3],float);T[:3,:3]=Rotation.from_quat(np.array(f[off+3:off+7],float)).as_matrix();pp.append(T)
    M=a.exp(f[21].split(';'));parity['g'].append(float(np.max(abs(a.inv(pp[0])@pp[1]-M))));exports['g'][i,j].append(M)
    obs=S/f'glim_observer/{f[1]}_after.json'
    if obs.exists():
     d=json.loads(pin(obs).read_text());ss={v['id']:np.array(v['T_world_imu']).reshape(4,4) for v in d['states']}
     native_observer.append(float(np.max(abs(a.inv(ss[i])@ss[j]-M))))
   if 'GLIM_CBS_MEAN_ALIGNMENT_ROW,' in line:
    f=decode(line,'GLIM_CBS_MEAN_ALIGNMENT_ROW');aligned['g'].append((tuple(map(int,f[:2])),a.exp(f[9].split(';')),a.exp(f[10].split(';')),tuple(map(int,f[2:4]))))
  wire={'g':defaultdict(list),'k':defaultdict(list)};wire_errors={'g':[],'k':[]};missing={'g':0,'k':0};headers={'g':set(),'k':set()}
  with AnyReader([pin(S/'recordings/cbs.bag')]) as r:
   topics={'/kimera/cbs/odom_belief_in':'g','/kimera/cbs/odom_belief_out':'k'}
   for c,t,b in r.messages(connections=[c for c in r.connections if c.topic in topics]):
    m=r.deserialize(b,c.msgtype);agent=topics[c.topic]
    for z in m.beliefs:
     assert int(z.source_agent)==ord(agent)
     key=(int(z.from_pose_index),int(z.to_pose_index));M=a.exp(np.array(z.relative_mu,copy=True));wire[agent][key].append(M);headers[agent].add(z.header.frame_id)
     if key not in exports[agent]:missing[agent]+=1;continue
     wire_errors[agent].append(near(M,exports[agent][key]))
  receive_errors={};freeze=[]
  for receiver,sender in [('g','k'),('k','g')]:
   es=[];miss=0
   for key,raw,mean,k in aligned[receiver]:
    if key not in wire[sender]:miss+=1;continue
    es.append(near(raw,wire[sender][key]))
    if receiver=='k':freeze.append(float(np.max(abs(mean-frozen[k]))))
   receive_errors[receiver]=dict(**stats(es),unmatched=miss)
  data=dict(native_endpoint_reconstruction={x:stats(parity[x]) for x in parity},wire_native_match={x:dict(**stats(wire_errors[x]),unmatched=missing[x],header_frames=sorted(headers[x])) for x in wire},receiver_raw_match=receive_errors,Kimera_aligned_frozen_match=stats(freeze),GLIM_native_observer_export_match=stats(native_observer))
  # Absence of logs is a coverage limitation, not a pass on that stage.
  for es in [*parity.values(),*wire_errors.values(),freeze,native_observer]:assert max(es,default=0)<1e-9,(case,mode,'mean mismatch',max(es))
  for es in receive_errors.values():assert es['max_matrix_error']<1e-9,(case,mode,es)
  results[case+'_'+mode]=data;print(case,mode,json.dumps(data),flush=True)
put('payloads.json',dict(passed=True,cases=results,input_sha256=pins,limits=['Matching repeated keys searches retained versions for an equal mean; not an independent arrival-order audit.','Classic active revision audit rows are not all published; counts are not claimed to be all wire payloads.','Kimera frozen logs are the insertion measurement, with prior CP51 native factor parity separately retained.','No native GLIM BetweenFactor matrices in CP54 observer; K2G last stage traced in source and exercised in standalone native test, not historical native snapshot verification.']))
