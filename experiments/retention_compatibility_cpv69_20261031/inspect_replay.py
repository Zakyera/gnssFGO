"""Saved-only diagnosis of the failed admission-timing gate; no scoring/reruns."""
import hashlib,json,re
from pathlib import Path
import numpy as np
from rosbags.rosbag1 import Reader
H=Path(__file__).resolve().parent;K=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')/H.name
P=K/'replay_pathcheck';D=P/'remove_capture';frozen=json.loads((P/'frozen_inputs.json').read_text())['rows']
ansi=re.compile(r'\x1b\[[0-9;]*m');bound=[];adds=[];header={};poses=[]
for line in (D/'kimera.log').read_text().splitlines():
 if 'FL4M_FROZEN_FACTOR_ROW,' in line:
  v=ansi.sub('',line.split('FL4M_FROZEN_FACTOR_ROW,',1)[1]).split(',')
  bound.append(dict(ids=list(map(int,v[:5])),mu=list(map(float,v[5].split(';'))),cov=list(map(float,re.split('[;:]',v[6])))))
 if 'CBS_BPSAM_ODOM_ADD_ROW_G2K,' in line:
  v=line.split('CBS_BPSAM_ODOM_ADD_ROW_G2K,',1)[1].split(',');assert v[2]=='0'
  adds.append((list(map(int,re.findall(r'\d+',v[0]))),list(map(int,re.findall(r'\d+',v[1])))))
 if 'KIMERA_CBS_REVISION_EDGE_ROW,' in line:
  v=ansi.sub('',line.split('KIMERA_CBS_REVISION_EDGE_ROW,',1)[1]).split(',')
  if int(v[0])==int(v[3]):header[int(v[0])]=float(v[5])
 if 'KIMERA_BACKEND_POSE_PATH_ROW,' in line:poses.append(line)
assert len(bound)==len(adds)==len(frozen)==292
assert adds==[(r['sender'],r['receiver']) for r in frozen]
mismatches=[];meanmax=0.
for got,r in zip(bound,frozen):
 assert got['ids'][1:]==r['sender']+r['receiver']
 assert got['cov']==r['cov'];meanmax=max(meanmax,float(np.max(np.abs(np.array(got['mu'])-r['aligned']))))
 if got['ids'][0]!=r['update']:
  mismatches.append(dict(sender=r['sender'],receiver=r['receiver'],original_update=r['update'],replay_update=got['ids'][0],
   original_update_time_s=(r['times'][4]-1730638816563546034)*1e-9,
   replay_update_time_s=header[got['ids'][0]]-1730638816.563546034,
   original_record_arrival_ns=r['arrival_ns']))
assert meanmax<1e-12
def messages(path):
 with Reader(path) as bag:
  return [(t,bytes(raw)) for c,t,raw in bag.messages(connections=[c for c in bag.connections if c.topic=='/kimera/cbs/odom_belief_in'])]
inputs=messages(P/'aligned_inputs.bag');received=messages(D/'outputs.bag')
assert [x[1] for x in inputs]==[x[1] for x in received] and len(inputs)==292
arrival=np.array([r[0]-i[0] for i,r in zip(inputs,received)])
cost=np.loadtxt(D/'objective.csv',delimiter=',');assert np.isfinite(cost).all()
result=dict(compatibility_gate_passed=True,replay_admission_gate_passed=False,retain_launched=(P/'retain_capture').exists(),
 comparison_available=False,source_capture_completed=True,method1_pose_rows=len(poses),received_factors=len(inputs),
 exact_wire_payload_sequence=True,exact_receiver_edges=True,exact_covariance=True,maximum_mean_roundtrip_difference=meanmax,
 exact_update_matches=292-len(mismatches),update_mismatches=mismatches,
 receipt_minus_input_record_ns=dict(minimum=int(arrival.min()),maximum=int(arrival.max()),median=float(np.median(arrival))),
 objective_updates=len(cost),retirements=int(cost[:,2].sum()),no_source_retry_after_timing_gate=True,
 conclusion='Recorded ROS arrival times do not fully reproduce backend admission order. No conclusion about sustained retain accuracy; original timing gate preserved.',
 proposed_next='Explicit recorded-update delivery gate in a fresh diagnostic harness, with causal availability and exact update/endpoint/payload checks; not a blind rosbag retry.')
with (H/'replay_diagnosis.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
