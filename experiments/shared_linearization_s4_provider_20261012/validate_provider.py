"""Read-only full ordinary-provider -> ROS-wire covariance audit (Noetic)."""
from pathlib import Path
import argparse,json,re
import numpy as np
import rosbag
p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('out',type=Path);a=p.parse_args()
source=a.run/'prepared/profile40_on_source'
log,=[p for p in (source/'ros_logs').glob('*/glim_rosnode-*-stdout.log') if p.parent.name!='latest']
text=re.sub(r'\x1b\[[0-9;]*m','',log.read_text())
assert 'SHARED_NATIVE_PROVIDER_SETUP enabled=1 independent_local=0 revision_covariance=unchanged_main' in text
assert not any(t in text for t in ['SHARED_NATIVE_SHADOW_FAILURE','SHARED_NATIVE_SHADOW_SETUP_FAILURE','SHARED_NATIVE_PROVIDER_FAILURE'])
records=[];matrices={}
for line in text.splitlines():
    if 'SHARED_NATIVE_PROVIDER_QUERY ' not in line:continue
    fields=dict(x.split('=',1) for x in line.split('SHARED_NATIVE_PROVIDER_QUERY ',1)[1].split())
    cov=np.array([float(x) for x in re.split('[:;]',fields.pop('matrix'))]).reshape(6,6)
    record={k:float(v) if k=='query_ms' else int(v) for k,v in fields.items()}
    edge=(record['from']&((1<<56)-1),record['to']&((1<<56)-1))
    assert edge not in matrices,'unexpected duplicate ordinary query'
    assert np.isfinite(cov).all() and np.linalg.eigvalsh(cov).min()>0
    assert np.max(np.abs(cov-cov.T))<1e-12
    assert record['admitted']==record['active']+record['absorbed']+record['removed']
    assert record['resets']==0
    record.update(edge=list(edge),minimum_eigenvalue=float(np.linalg.eigvalsh(cov).min()),
                  symmetry_max=float(np.max(np.abs(cov-cov.T))))
    matrices[edge]=tuple(cov.ravel());records.append(record)
assert len(records)>=350
assert [r['epoch'] for r in records]==sorted(r['epoch'] for r in records)
wire={}
with rosbag.Bag(str(source/'recordings/cbs.bag')) as bag:
    for topic,msg,stamp in bag.read_messages(topics=['/kimera/cbs/odom_belief_in']):
        for b in msg.beliefs:
            edge=(b.from_pose_index,b.to_pose_index)
            assert edge not in wire,'unexpected duplicate ordinary belief on wire'
            wire[edge]=tuple(b.covariance)
assert wire.keys()==matrices.keys(),(len(wire),len(matrices),set(wire)^set(matrices))
assert all(wire[k]==matrices[k] for k in wire),'provider covariance changed before wire'
profile=[json.loads(l) for l in (source/'e1b_profile.jsonl').open()]
assert not any(r['context']==2 for r in profile),'independent LOCAL still running'
assert all(not r['aborted'] for r in profile)
covariance=[r for r in profile if r['context']==3]
assert covariance
assert not any(r['owner_wall'].get('gpu_linearize',[0])[0] or r['owner_wall'].get('gpu_sync_linearize',[0])[0] or r['metrics'][7] for r in covariance),'provider reevaluated GPU factors'
result=dict(passed=True,ordinary_queries=len(records),wire_beliefs=len(wire),
    all_full_6x6_covariances_byte_value_equal=True,local_update_rows=0,
    provider_gpu_calls=0,minimum_eigenvalue=min(r['minimum_eigenvalue'] for r in records),
    max_symmetry_error=max(r['symmetry_max'] for r in records),rows=records,
    final_counts={k:records[-1][k] for k in ['epoch','admitted','active','absorbed','removed','resets']})
with a.out.open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))
