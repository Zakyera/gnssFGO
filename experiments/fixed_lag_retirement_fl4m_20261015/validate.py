"""Fail-closed frozen-input/binding/coverage gate, not an accuracy claim."""
from pathlib import Path
import io,json,re,sys
import numpy as np
import rosbag
R=Path(__file__).resolve().parent;mode=sys.argv[1];D=R/(mode+'_capture')
assert (D/'complete.json').exists() and not (D/'failed.txt').exists()
frozen=json.loads((R/'frozen_inputs.json').read_text())['rows']
lines=(D/'kimera.log').read_text().splitlines()
adds=[];transport=[];bound=[]
for line in lines:
    if 'FL4M_FROZEN_FACTOR_ROW,' in line:
        v=line.split('FL4M_FROZEN_FACTOR_ROW,',1)[1].split(',')
        bound.append((list(map(int,v[:5])),[float(x) for x in v[5].split(';')],[float(x) for x in re.split('[;:]',v[6])]))
    if 'CBS_BPSAM_ODOM_ADD_ROW_G2K,' in line:
        v=line.split('CBS_BPSAM_ODOM_ADD_ROW_G2K,',1)[1].split(',')
        if v[2]=='0':adds.append((tuple(map(int,re.findall(r'\d+',v[0]))),tuple(map(int,re.findall(r'\d+',v[1])))))
    if 'KIMERA_CBS_SOURCE_TRANSPORT_ROW,' in line:
        v=line.split('KIMERA_CBS_SOURCE_TRANSPORT_ROW,',1)[1].split(',')
        transport.append((tuple(map(int,v[1:3])),[float(x) for x in re.split('[;:]',v[5])]))
expected=[(tuple(r['sender']),tuple(r['receiver'])) for r in frozen]
assert adds==expected,('factor identities/bindings mismatch',len(adds),len(expected),next(((a,b) for a,b in zip(adds,expected) if a!=b),None))
assert len(transport)==len(frozen)
for (key,cov),r in zip(transport,frozen):assert key==tuple(r['sender']) and cov==r['cov']
assert len(bound)==len(frozen)
for (ids,mu,cov),r in zip(bound,frozen):
    assert ids==[r['update']]+r['sender']+r['receiver'],('update/binding mismatch',ids,r['update'])
    assert np.max(np.abs(np.array(mu)-r['aligned']))<1e-12 and cov==r['cov']
def serialized(msg):
    s=io.BytesIO();msg.serialize(s);return s.getvalue()
with rosbag.Bag(str(R/'aligned_inputs.bag')) as bag:
    input_payloads=[serialized(m) for _,m,_ in bag.read_messages(topics=['/kimera/cbs/odom_belief_in'])]
poses=[];received=[]
with rosbag.Bag(str(D/'outputs.bag')) as bag:
    for topic,msg,_ in bag.read_messages():
        if topic=='/kimera/cbs/odom_belief_in':received.append(serialized(msg))
        if topic=='/kimera_vio_ros/odometry':
            poses.append(msg.header.stamp.to_nsec())
            p=msg.pose.pose
            assert np.isfinite([p.position.x,p.position.y,p.position.z,p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z]).all()
assert received==input_payloads,('wire payload mismatch',len(received),len(input_payloads))
assert len(poses)==454 and (poses[-1]-poses[0])*1e-9>93.5,('coverage',len(poses))
assert all(a<b for a,b in zip(poses,poses[1:])), 'nonmonotonic native output'
objective=np.loadtxt(D/'objective.csv',delimiter=',')
assert len(objective)>=453 and np.isfinite(objective).all()
assert set(objective[:,1])=={0 if mode=='remove' else 1}
report=dict(valid=True,mode=mode,pose_count=len(poses),pose_span_s=(poses[-1]-poses[0])*1e-9,
    incoming_factor_count=len(adds),exact_covariance=True,exact_endpoint_bindings=True,
    exact_recorded_wire_payloads=True,exact_backend_update_ids=True,
    cost_updates=len(objective),cost_increases=int(np.sum(objective[:,5]>objective[:,4]*1.000001+1e-6)),
    planned_retirements=int(objective[:,2].sum()))
(D/'validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
