"""Exact frozen prefix and native baseline comparison, no accuracy scoring."""
from pathlib import Path
import io,json,re
import numpy as np
import rosbag
R=Path(__file__).resolve().parent;D=R/'remove_capture'
BASE=R.parent/'fixed_lag_linear_parity_fl4n_20261015/remove_capture'
assert (D/'complete.json').exists() and not (D/'failed.txt').exists()
assert 'complete=1' in (D/'direction/update_89/summary.txt').read_text()
def serialized(msg):
    s=io.BytesIO();msg.serialize(s);return s.getvalue()
def messages(p,topic):
    with rosbag.Bag(str(p)) as bag:return [(m.header.stamp.to_nsec() if hasattr(m,'header') else None,serialized(m)) for _,m,_ in bag.read_messages(topics=[topic])]
topic='/kimera_vio_ros/odometry'
poses=messages(D/'outputs.bag',topic);baseline=messages(BASE/'outputs.bag',topic)
assert len(poses)>=90 and poses==baseline[:len(poses)],('native prefix',len(poses))
topic='/kimera/cbs/odom_belief_in'
received=messages(D/'outputs.bag',topic);frozenwire=messages(R/'aligned_inputs.bag',topic)
assert received==frozenwire[:len(received)] and len(received)>=86
frozen=json.loads((R/'frozen_inputs.json').read_text())['rows'];bound=[]
lines=(D/'kimera.log').read_text().splitlines()
for l in lines:
    if 'FL4M_FROZEN_FACTOR_ROW,' in l:
        v=l.split('FL4M_FROZEN_FACTOR_ROW,',1)[1].split(',')
        bound.append((list(map(int,v[:5])),[float(x) for x in v[5].split(';')],[float(x) for x in re.split('[;:]',v[6])]))
for (ids,mu,cov),r in zip(bound,frozen):
    assert ids==[r['update']]+r['sender']+r['receiver']
    assert np.max(np.abs(np.array(mu)-r['aligned']))<1e-12 and cov==r['cov']
assert len(bound)==len(received)
a=np.loadtxt(D/'objective.csv',delimiter=',');b=np.loadtxt(BASE/'objective.csv',delimiter=',')
assert np.array_equal(a,b[:len(a)]) and len(a)>=90
assert not any('FL4O_' in l for l in lines)
report=dict(valid=True,native_pose_count=len(poses),native_span_s=(poses[-1][0]-poses[0][0])*1e-9,
    baseline_native_prefix_byte_identical=True,objective_prefix_exact=True,incoming_count=len(received),
    exact_wire_payloads=True,exact_binding_update_covariance=True,max_aligned_mean_error= max(np.max(np.abs(np.array(x[1])-r['aligned'])) for x,r in zip(bound,frozen)),
    observer_update=89,solver_change=False,planned_retirements=int(a[:,2].sum()))
(D/'validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
