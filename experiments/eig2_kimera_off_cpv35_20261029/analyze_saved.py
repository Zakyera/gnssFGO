"""Read original OFF audits. No estimator, metric rescore, or source edits."""
import ast,hashlib,json,re
from pathlib import Path
import numpy as np
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
V=W/'runs/gnssfgo_eig2_velodyne_full_cpv4_20261024/prepared/full_off_source'
S=W/'runs/gnssfgo_eig2_full_cpk24_20261021/prepared/full_off_source'
K=W/'runs/kimera_incremental_prior_cpk9_20261019'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
events=json.loads((H.parent/'eig2_kimera_jitter_cpv34_20261029/velodyne_eig2_off_events.json').read_text())
e={int(d['k']):d for d in events};a=np.loadtxt(V/'objective.csv',delimiter=',');b=np.loadtxt(S/'objective.csv',delimiter=',')
assert np.array_equal(a[:,0],np.arange(len(a)))
assert np.array_equal(a,b),'OFF cost histories differ'
assert np.all(a[:,1:3]==0),'OFF retain/retirement is nonzero'
launch=list((V/'ros_logs').glob('*/roslaunch-*.log'));assert len(launch)==1
envline=next(l for l in launch[0].read_text().splitlines() if 'process[kimera_vio_ros/kimera_vio_ros_node-' in l and ': env[' in l)
env=ast.literal_eval(envline.split(': env[',1)[1][:-1])
assert env['FL4M_AUDIT'].endswith('/full_off_source/ros_logs/../objective.csv')
assert env['FL4Q_ENABLE']=='0' and env['CPK9_PRIOR_REFRESH']=='1'
text=(V/'estimators.log').read_text()
pose_ids=[int(l.split('BACKEND_POSE_PATH_ROW,',1)[1].split(',')[1]) for l in text.splitlines() if 'BACKEND_POSE_PATH_ROW,' in l]
assert pose_ids==list(range(len(a))),(len(pose_ids),len(a),pose_ids[:3])
def record(k):
    r=a[k];out=dict(update=k,pre_total=r[4],post_total=r[5],pre_imu=r[6],post_imu=r[7],pre_prior=r[8],post_prior=r[9],post_other=r[5]-r[7]-r[9],total_ratio=r[5]/r[4] if r[4] else None)
    for key in ('time_s','previous_revision_m','same_update_motion_m','absolute_step_m','shape_residual_max_m','window_rigid_explained_fraction'):
        if key in e.get(k,{}):out[key]=e[k][key]
    return out
crossings={}
for threshold in (1e3,1e5,1e6,1e8,1e9):
    ids=np.flatnonzero(a[:,7]>threshold)
    crossings[str(threshold)]=dict(count=len(ids),first=record(int(ids[0])) if len(ids) else None,last=record(int(ids[-1])) if len(ids) else None)
inc=np.flatnonzero(a[:,5]>a[:,4]+1e-6)
indices=[int(x) for x in np.argsort(a[:,7])[-10:][::-1]]
large=[d for d in events if d['previous_revision_m']>.5]
result=dict(passed=True,rows=len(a),off_cost_arrays_identical=True,off_retirements=0,
 columns=['update','retain','retirements','native_marginal_priors','total_before','total_after','native_imu_before','native_imu_after','prior_before','prior_after'],
 provenance=dict(writer=str(K/'RetirementAudit.h'),actual_launch_env=dict(FL4M_AUDIT=env['FL4M_AUDIT'],FL4Q_ENABLE=env['FL4Q_ENABLE'],CPK9_PRIOR_REFRESH=env['CPK9_PRIOR_REFRESH']),same_graph_before_after=True,before_marginalization=True,not_a_stationarity_certificate=True),
 total_cost_increases=len(inc),total_cost_increase_ids=inc.tolist(),post_imu_quantiles=np.quantile(a[:,7],[0,.5,.9,.95,.99,1]).tolist(),
 crossings=crossings,largest_post_imu=[record(k) for k in indices],target_rows=[record(k) for k in [30,50,100,500,1000,1334,1335,1336,1337,1338,1339,1340,1410,1467,1760]],
 large_revision_count=len(large),large_revision_post_imu_quantiles=np.quantile([a[d['k'],7] for d in large],[.5,.95,1]).tolist(),
 source_runs=0,input_sha256={str(p):sha(p) for p in [V/'objective.csv',S/'objective.csv',V/'estimators.log',launch[0],K/'RetirementAudit.h',K/'include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h',K/'include/cbs/bpsam/fixed_lag_objective_audit.h',H.parent/'eig2_kimera_jitter_cpv34_20261029/velodyne_eig2_off_events.json']})
with (H/'saved_cost_results.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k in ('rows','off_cost_arrays_identical','total_cost_increases','post_imu_quantiles','largest_post_imu','target_rows')},indent=2))
