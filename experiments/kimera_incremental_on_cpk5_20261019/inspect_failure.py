"""Read only the stopped capture; no scoring/replay or full-window acceptance."""
import hashlib,io,json
from pathlib import Path
import numpy as np
import rosbag
W=Path('/workspace/cbs_gtsam4.3');R=W/'runs/kimera_incremental_on_cpk5_20261019';D=R/'on_capture'
O=W/'runs/kimera_incremental_control_cpk4b_20261019/off_capture'
def raw(m):
    out=io.BytesIO();m.serialize(out);return out.getvalue()
assert (D/'failed.txt').exists() and not (D/'capture_complete.json').exists()
assert json.loads((D/'cleanup.json').read_text())['kimera_process_gone']
assert not (D/'failure_evidence.json').exists()
pre=json.loads((D/'preflight.json').read_text());oldpre=json.loads((O/'preflight.json').read_text())
assert pre['selected_env']['FL4Q_ENABLE']=='0' and pre['selected_env']['FL4M_RETIREMENT']=='remove'
for k,v in oldpre['flags'].items():
    if k not in ('flagfile','output_path'):assert pre['flags'][k]==v,(k,v,pre['flags'][k])
expected=[];source={};scheduled={};received=[];arrival_errors=[];native=[]
with rosbag.Bag(str(R/'raw_glim_inputs.bag')) as bag:
    for _,m,t in bag.read_messages():
        if t.to_sec()>=1727775118.955396:continue
        expected.append(raw(m));b=m.beliefs[0];source[(int(b.from_pose_index),int(b.to_pose_index))]=b
        scheduled[raw(m)]=t.to_nsec()
with rosbag.Bag(str(D/'outputs.bag')) as bag:
    for topic,m,t in bag.read_messages():
        if topic=='/kimera/cbs/odom_belief_in':
            received.append(raw(m));arrival_errors.append((t.to_nsec()-scheduled[raw(m)])*1e-9)
        elif topic=='/kimera_vio_ros/odometry':
            p,q=m.pose.pose.position,m.pose.pose.orientation
            assert np.isfinite([p.x,p.y,p.z,q.x,q.y,q.z,q.w]).all()
            native.append((m.header.stamp.to_nsec(),raw(m)))
assert received==expected[:len(received)] and len(received)<len(expected)
assert all(y[0]>x[0] for x,y in zip(native,native[1:]))
log=(D/'kimera.log').read_text();covdiff=[];mudiff=[];bindings=[];trigger=[]
for ln,line in enumerate(log.splitlines(),1):
    if 'KIMERA_CBS_MEAN_ALIGNMENT_ROW,' in line:
        f=line.split('KIMERA_CBS_MEAN_ALIGNMENT_ROW,',1)[1].split(',');assert len(f)==13
        key=(int(f[1]),int(f[2]));b=source[key]
        c=np.array([float(x) for row in f[12].split(';') for x in row.split(':')])
        covdiff.append(float(np.max(np.abs(c-np.array(b.covariance)))))
        mudiff.append(float(np.max(np.abs(np.array([float(x) for x in f[10].split(';')])-np.array(b.relative_mu)))))
        bindings.append((int(f[0]),key))
    if any(s in line for s in ('active_window_staged_residual_observed','active_window_rollback',
                               'active_window_staged_factor_dropped','active_window_retry_local_update_without_staged_cbs')):
        trigger.append(dict(line=ln,text=line))
assert max(covdiff)==0 and max(mudiff)<1e-12
a=np.loadtxt(D/'objective.csv',delimiter=',');off=np.loadtxt(O/'objective.csv',delimiter=',')
assert np.isfinite(a).all() and np.all(a[:,1]==0)
assert np.array_equal(a[:,0],np.arange(len(a)))
old=[]
with rosbag.Bag(str(O/'outputs.bag')) as bag:
    for _,m,_ in bag.read_messages(topics=['/kimera_vio_ros/odometry']):old.append(raw(m))
firstdiff=next((i for i,(x,y) in enumerate(zip(native,old)) if x[1]!=y),None)
checkpoints={str(i):dict(before=a[i,4],after=a[i,5],imu_before=a[i,6],imu_after=a[i,7],prior_after=a[i,9],retired=int(a[i,2])) for i in (4,27,28,44,438,463,494,495) if i<len(a)}
out=dict(capture_passed=False,evidence_checks_passed=True,stop_reason=(D/'failed.txt').read_text().strip(),
         native_count=len(native),last_native_time_s=native[-1][0]*1e-9,last_native_bag_offset_s=native[-1][0]*1e-9-1727775007.955396,
         native_span_s=(native[-1][0]-native[0][0])*1e-9,first_native_difference_from_off=firstdiff,
         objective_rows=len(a),last_objective_update=int(a[-1,0]),fl4q_calls=0,
         temporary_retirements=int(a[:,2].sum()),first_retirement_update=int(a[np.flatnonzero(a[:,2])[0],0]),
         covariance_alignment_rows=len(bindings),covariance_max_abs_diff=max(covdiff),raw_mean_max_abs_diff=max(mudiff),
         expected_full_wire=len(expected),received_prefix_wire=len(received),exact_received_payload_prefix=True,
         arrival_record_replay_error_s=dict(min=min(arrival_errors),max=max(arrival_errors),median=float(np.median(arrival_errors))),
         cost_increases=int(np.count_nonzero(a[:,5]>a[:,4]+1e-10)),max_imu_after=float(a[:,7].max()),
         max_off_imu_after=float(off[:,7].max()),checkpoints=checkpoints,stop_log=trigger,
         native_outputs_finite_monotonic=True,complete_111s=False,ATE_scored=False,
         scope='One-way conditional saved GLIM input, not bidirectional CBS; no retries or fusion')
with (D/'failure_evidence.json').open('x') as f:json.dump(out,f,indent=2,allow_nan=False)
print(json.dumps(out,indent=2))
