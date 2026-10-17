"""Frozen source payload/lifecycle and numerical acceptance validation, no ATE."""
from pathlib import Path
import io,json,re
import numpy as np
import rosbag
R=Path(__file__).resolve().parent;D=R/'remove_capture'
B=R.parent/'fixed_lag_linear_parity_fl4n_20261015/remove_capture'
assert (D/'complete.json').exists() and not (D/'failed.txt').exists()
def serialize(m):
    s=io.BytesIO();m.serialize(s);return s.getvalue()
def messages(path,topic):
    with rosbag.Bag(str(path)) as bag:
        return [(m.header.stamp.to_nsec() if hasattr(m,'header') else None,serialize(m)) for _,m,_ in bag.read_messages(topics=[topic])]
poses=messages(D/'outputs.bag','/kimera_vio_ros/odometry')
baseline=messages(B/'outputs.bag','/kimera_vio_ros/odometry')
assert len(poses)==134,('missing native output',len(poses))
incoming=messages(D/'outputs.bag','/kimera/cbs/odom_belief_in')
frozen=messages(R/'aligned_inputs.bag','/kimera/cbs/odom_belief_in')
assert len(incoming)==130 and incoming==frozen[:len(incoming)]
rows=json.loads((R/'frozen_inputs.json').read_text())['rows']
bound=[];lines=(D/'kimera.log').read_text().splitlines()
for line in lines:
    if 'FL4M_FROZEN_FACTOR_ROW,' in line:
        v=line.split('FL4M_FROZEN_FACTOR_ROW,',1)[1].split(',')
        bound.append((list(map(int,v[:5])),[float(x) for x in v[5].split(';')],[float(x) for x in re.split('[;:]',v[6])]))
assert len(bound)==len(incoming)
for (ids,mu,cov),r in zip(bound,rows):
    assert ids==[r['update']]+r['sender']+r['receiver']
    assert np.max(np.abs(np.array(mu)-r['aligned']))<1e-12 and cov==r['cov']
assert not any(marker in line for line in lines for marker in ['FL4Q','active_window_rollback','Smoother recovery','active_window_retry_local_update_'])
a=np.loadtxt(D/'acceptance.csv',delimiter=',')
obj=np.loadtxt(D/'objective.csv',delimiter=',')
base_obj=np.loadtxt(B/'objective.csv',delimiter=',')
assert len(a)==len(obj)==134 and np.all(np.isfinite(a))
fallback=a[a[:,4]==1];assert len(fallback)>0 and fallback[0,0]==89
assert np.all(a[:,3]<=a[:,1]+1e-10*(1+np.abs(a[:,1])))
assert np.all(fallback[:,3]<fallback[:,1]) and np.max(a[:,7:9])<=1e-8
assert np.all(a[:,5]<=10)
assert np.array_equal(obj[:89],base_obj[:89]) and poses[:89]==baseline[:89]
assert poses[89]!=baseline[89]
def summary(x):return dict(median=float(np.median(x)),p95=float(np.percentile(x,95)),maximum=float(np.max(x)),total=float(np.sum(x)))
report=dict(valid=True,native_poses=len(poses),native_span_s=(poses[-1][0]-poses[0][0])*1e-9,
    incoming=len(incoming),wire_payloads_exact=True,update_bindings_covariances_exact=True,
    native_prefix_0_through_88_byte_identical=True,first_changed_native_update=89,
    all_objective_steps_nonincreasing=True,accepted_fallback_count=len(fallback),
    fallback_updates=fallback[:,0].astype(int).tolist(),fallback_iterations=fallback[:,5].astype(int).tolist(),
    fallback_rows=fallback.tolist(),decision_seconds=summary(a[:,6]),fallback_seconds=summary(fallback[:,6]),
    max_cache_fresh_error=float(np.max(a[:,7])),max_cache_tree_error=float(np.max(a[:,8])),
    planned_retirements=int(obj[:,2].sum()),complete=json.loads((D/'complete.json').read_text()),
    accuracy_evaluated=False,stationarity_certified=False)
(D/'validation.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
