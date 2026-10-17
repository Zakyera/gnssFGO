"""Validate the stopped prefix, never present it as a completed experiment."""
import csv,io,json,re
from pathlib import Path
import numpy as np
import rosbag
R=Path('/workspace/cbs_gtsam4.3/runs/fixed_lag_step_acceptance_fl4o_20261016')
OLD=R.parent/'fixed_lag_linear_parity_fl4n_20261015'
D=R/'remove_capture'
assert (D/'failed.txt').exists() and not (D/'complete.json').exists()
frozen=json.loads((R/'frozen_inputs.json').read_text())['rows']
lines=(D/'kimera.log').read_text().splitlines()
bound=[];trans=[];adds=[]
for line in lines:
    if 'FL4M_FROZEN_FACTOR_ROW,' in line:
        v=line.split('FL4M_FROZEN_FACTOR_ROW,',1)[1].split(',')
        bound.append((list(map(int,v[:5])),list(map(float,v[5].split(';'))),list(map(float,re.split('[;:]',v[6])))))
    if 'KIMERA_CBS_SOURCE_TRANSPORT_ROW,' in line:
        v=line.split('KIMERA_CBS_SOURCE_TRANSPORT_ROW,',1)[1].split(',')
        trans.append((list(map(int,v[1:3])),list(map(float,re.split('[;:]',v[5])))))
    if 'CBS_BPSAM_ODOM_ADD_ROW_G2K,' in line:
        v=line.split('CBS_BPSAM_ODOM_ADD_ROW_G2K,',1)[1].split(',')
        if v[2]=='0':adds.append((list(map(int,re.findall(r'\d+',v[0]))),list(map(int,re.findall(r'\d+',v[1])))))
assert len(bound)==len(trans)==len(adds)
for i,((ids,mean,cov),(sender,transport),(a,b)) in enumerate(zip(bound,trans,adds)):
    r=frozen[i]
    assert ids==[r['update']]+r['sender']+r['receiver']
    assert np.max(np.abs(np.array(mean)-r['aligned']))<1e-12
    assert cov==transport==r['cov'] and sender==a==r['sender'] and b==r['receiver']
def serial(m):
    out=io.BytesIO();m.serialize(out);return out.getvalue()
with rosbag.Bag(str(R/'aligned_inputs.bag')) as bag:
    expected=[serial(m) for _,m,_ in bag.read_messages(topics=['/kimera/cbs/odom_belief_in'])]
native=[];received=[]
with rosbag.Bag(str(D/'outputs.bag')) as bag:
    for topic,m,_ in bag.read_messages():
        if topic=='/kimera/cbs/odom_belief_in':received.append(serial(m))
        if topic=='/kimera_vio_ros/odometry':native.append([m.header.seq,m.header.stamp.to_nsec()])
assert received==expected[:len(received)] and len(received)==len(bound)
acc=np.loadtxt(D/'acceptance.csv',delimiter=',',ndmin=2)
obj=np.loadtxt(D/'objective.csv',delimiter=',',ndmin=2)
assert np.isfinite(acc).all() and len(acc)==len(obj)==len(native)
assert np.array_equal(acc[:,0],obj[:,0])
assert np.all(acc[:,3]<=acc[:,1]+1e-10*(1+np.abs(acc[:,1])))
assert all(a[0]+1==b[0] and a[1]<b[1] for a,b in zip(native,native[1:]))
markers=['active_window_rollback','active_window_retry_local_update_','active_window_staged_factor_dropped','action=drop_before_stage','Smoother recovery','Persistent BPSAM recovery']
assert not any(any(m in line for m in markers) for line in lines)
failures=[line for line in lines if 'FL4O no decreasing step' in line]
assert len(failures)==1
old=list(csv.reader((OLD/'remove_capture/native.log').open()))
new=list(csv.reader((D/'native.log').open()))
different=[i-1 for i in range(1,min(len(old),len(new))) if old[i][1:]!=new[i][1:]]
failed_update=bound[-1][0][0]
assert native[-1][0]==failed_update-1 and failed_update==108
summary=dict(completed=False,candidate_validated=False,stopped_by_safeguard=True,
    failed_update=failed_update,last_published_update=native[-1][0],native_poses=len(native),
    native_span_s=(native[-1][1]-native[0][1])*1e-9,
    last_native_bag_relative_s=native[-1][1]*1e-9-1727775007,
    exact_received_frozen_prefix=len(received),planned_full_frozen_inputs=len(expected),
    exact_prefix_covariances_means_bindings_updates=True,
    accepted_updates=len(acc),retirements_committed=int(obj[:,2].sum()),
    first_changed_native_update=different[0] if different else None,
    shortened=[dict(update=int(r[0]),before=float(r[1]),full=float(r[2]),accepted=float(r[3]),alpha=float(r[4]),evaluations=int(r[5])) for r in acc if r[4]<1],
    accepted_cost_increases=0,other_recovery_or_drop=0,error=failures[0],
    no_full_window_accuracy_claim=True)
with (R/'failure_analysis.json').open('x') as f:json.dump(summary,f,indent=2)
print(json.dumps(summary,indent=2))
