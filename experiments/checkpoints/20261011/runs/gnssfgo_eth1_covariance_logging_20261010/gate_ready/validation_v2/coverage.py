"""Read-only native/live/final trajectory coverage, without any accuracy scoring."""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent.parent
source=ROOT/'gate_on_source'; receiver=ROOT/'gate_on_receiver/receiver'
assert (ROOT/'gate_on_receiver/complete.marker').exists()
assert 'Done.' in (source/'rosbag.log').read_text()
assert 'Insufficient estimator output: GLIM=91, Kimera=34.' in (ROOT/'gate_on_control/source.log').read_text()
assert json.loads((source/'effective_mode_check.json').read_text())['passed']
imus=[]
with (ROOT/'gate_on_receiver/events.csv').open() as f:
    for line in f:
        v=line.split(',')
        if v[0]=='I': imus.append(int(v[2])*1e-9)
paths={'native_glim':source/'odom_imu.txt','native_kimera':source/'kimera_odometry.csv',
       'fused_online':receiver/'online_trajectory.tum','fused_final_retrospective':receiver/'trajectory.tum'}
result={}
for role,path in paths.items():
    if role=='native_kimera':
        with path.open() as f: rows=list(csv.DictReader(f))
        t=np.array([int(r['field.header.stamp'])*1e-9 for r in rows])
        p=np.array([[float(r['field.pose.pose.position.'+c]) for c in 'xyz'] for r in rows])
    else:
        a=np.loadtxt(path);t=a[:,0];p=a[:,1:4]
    assert len(t)>10 and np.isfinite(p).all() and (np.diff(t)>0).all(),role
    span=float(t[-1]-t[0]);tail=float(imus[-1]-t[-1])
    assert span>=8.5 and tail<.5,(role,span,tail)
    result[role]=dict(rows=len(t),start_s=float(t[0]),end_s=float(t[-1]),span_s=span,
         maximum_pose_gap_s=float(np.diff(t).max()),tail_behind_final_imu_s=tail,
         path_length_m=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum()),
         sha256=hashlib.sha256(path.read_bytes()).hexdigest())
out=dict(passed=True,requested_bag_duration_s=10,clock_imu_samples=len(imus),
         clock_imu_span_s=imus[-1]-imus[0],trajectories=result,
         caveat='Native startup gaps are reported, not filled. No GT/ATE comparison is made by this logging gate.')
with (ROOT/'coverage_validation.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps(out,indent=2))
