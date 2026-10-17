"""Freeze existing aligned measurements; standard rosbag handles sensor replay."""
from pathlib import Path
import copy, hashlib, io, json, re
import numpy as np
import rosbag
R=Path(__file__).resolve().parent
W=Path('/workspace/cbs_gtsam4.3')
S=W/'runs/fixed_lag_same_graph_fl4l_20261015/on_source_capture'
rows=[]
for line in (S/'estimators.log').read_text().splitlines():
    if 'KIMERA_CBS_MEAN_ALIGNMENT_ROW,' not in line: continue
    v=line.split('KIMERA_CBS_MEAN_ALIGNMENT_ROW,',1)[1].split(',')
    assert len(v)==13
    rows.append(dict(update=int(v[0]),sender=list(map(int,v[1:3])),receiver=list(map(int,v[3:5])),
        times=list(map(int,v[5:10])),raw=[float(x) for x in v[10].split(';')],
        aligned=[float(x) for x in v[11].split(';')],cov=[float(x) for x in re.split('[;:]',v[12])]))
assert len(rows)==450 and len({tuple(r['sender']) for r in rows})==450
assert len({tuple(r['receiver']) for r in rows})==450
by_id={tuple(r['sender']):r for r in rows}; messages={}
with rosbag.Bag(str(S/'recordings/cbs.bag')) as bag:
    for topic,msg,t in bag.read_messages(topics=['/kimera/cbs/odom_belief_in']):
        for b in msg.beliefs:
            key=(b.from_pose_index,b.to_pose_index)
            if key not in by_id: continue
            r=by_id[key]
            if np.max(np.abs(np.array(b.relative_mu)-r['raw']))>1e-12: continue
            assert list(b.covariance)==r['cov']
            assert key not in messages, 'ambiguous duplicate wire ID'
            m=copy.deepcopy(msg); n=copy.deepcopy(b)
            n.relative_mu=r['aligned']; n.from_stamp_sec=r['times'][2]*1e-9; n.to_stamp_sec=r['times'][3]*1e-9
            m.beliefs=[n]
            r['arrival_ns']=t.to_nsec()
            assert r['arrival_ns']>=r['times'][1], 'noncausal incoming record'
            messages[key]=(m,t)
assert len(messages)==450
out=R/'aligned_inputs.bag'; assert not out.exists()
digest=hashlib.sha256()
with rosbag.Bag(str(out),'w') as bag:
    for r in sorted(rows,key=lambda r:r['arrival_ns']):
        msg,t=messages[tuple(r['sender'])]
        bag.write('/kimera/cbs/odom_belief_in',msg,t)
        stream=io.BytesIO(); msg.serialize(stream)
        digest.update(t.to_nsec().to_bytes(8,'little')); digest.update(stream.getvalue())
(R/'frozen_inputs.json').write_text(json.dumps(dict(rows=rows,count=len(rows),serialized_sha256=digest.hexdigest(),
    source=str(S),alignment='saved post-alignment; no recomputation',duration=94.0),indent=2))
print('FROZEN',len(rows),'aligned factors',digest.hexdigest())
