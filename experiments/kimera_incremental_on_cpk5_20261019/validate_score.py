"""Validate completed conditional ON; compare original native odometry with CP-K4b."""
import hashlib,importlib.util,io,json,math
from pathlib import Path
import numpy as np
import rosbag,yaml
W=Path('/workspace/cbs_gtsam4.3')
R=W/'runs/kimera_incremental_on_cpk5_20261019';D=R/'on_capture'
O=W/'runs/kimera_incremental_control_cpk4b_20261019/off_capture'
TOPIC='/kimera/cbs/odom_belief_in'
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def raw(m):
    b=io.BytesIO();m.serialize(b);return b.getvalue()
def native(path):
    rows=[]
    with rosbag.Bag(str(path)) as bag:
        for _,m,_ in bag.read_messages(topics=['/kimera_vio_ros/odometry']):
            p,q,v,w=m.pose.pose.position,m.pose.pose.orientation,m.twist.twist.linear,m.twist.twist.angular
            assert np.isfinite([p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z,*m.pose.covariance,*m.twist.covariance]).all()
            assert m.child_frame_id=='cpt7_imu'
            rows.append((m.header.stamp.to_nsec(),[p.x,p.y,p.z],[q.x,q.y,q.z,q.w],raw(m)))
    assert all(b[0]>a[0] for a,b in zip(rows,rows[1:]))
    return rows
assert not (D/'validation.json').exists() and not (D/'evaluation.json').exists(),'Refuse overwrite'
assert json.loads((D/'capture_complete.json').read_text())['passed']
assert json.loads((D/'cleanup.json').read_text())['kimera_process_gone']
assert not (D/'failed.txt').exists() and not (D/'acceptance.csv').exists()
assert not (D/'outputs.bag.active').exists()
pre=json.loads((D/'preflight.json').read_text());offpre=json.loads((O/'preflight.json').read_text())
assert pre['selected_env']['FL4Q_ENABLE']=='0' and pre['selected_env']['FL4M_RETIREMENT']=='remove'
for k,v in offpre['flags'].items():
    if k not in ('flagfile','output_path'):assert pre['flags'][k]==v,(k,v,pre['flags'][k])
assert yaml.safe_load((D/'resolved_params.yaml').read_text())['cbs_belief_receive_enable'] is True
for p,h in json.loads((R/'pre_capture_receipt.json').read_text())['manifest'].items():
    assert sha(p.replace('/home/yeranis/repos/V4RL','/workspace'))==h,p
expected=[];source={};scheduled={};received=[];arrival_errors=[]
with rosbag.Bag(str(R/'raw_glim_inputs.bag')) as bag:
    for _,m,t in bag.read_messages():
        if t.to_sec()>=1727775118.955396:continue
        expected.append(raw(m));b=m.beliefs[0];key=(int(b.from_pose_index),int(b.to_pose_index))
        assert key not in source
        source[key]=b;scheduled[raw(m)]=t.to_nsec()
with rosbag.Bag(str(D/'outputs.bag')) as bag:
    for _,m,t in bag.read_messages(topics=[TOPIC]):
        data=raw(m);received.append(data)
        arrival_errors.append((t.to_nsec()-scheduled[data])*1e-9)
assert len(expected)==1100 and received==expected,'Wire payload loss/reorder/mutation'
log=(D/'kimera.log').read_text();bindings=[];cov_diff=[];mu_diff=[]
for line in log.splitlines():
    if 'KIMERA_CBS_MEAN_ALIGNMENT_ROW,' not in line:continue
    f=line.split('KIMERA_CBS_MEAN_ALIGNMENT_ROW,',1)[1].split(',')
    assert len(f)==13
    key=(int(f[1]),int(f[2]));b=source[key]
    cov=np.array([float(x) for row in f[12].split(';') for x in row.split(':')])
    mu=np.array([float(x) for x in f[10].split(';')])
    cov_diff.append(float(np.max(np.abs(cov-np.array(b.covariance)))))
    mu_diff.append(float(np.max(np.abs(mu-np.array(b.relative_mu)))))
    bindings.append(dict(update=int(f[0]),source=list(key),receiver=[int(f[3]),int(f[4])],
                         source_t_ns=[int(f[5]),int(f[6])],receiver_t_ns=[int(f[7]),int(f[8])]))
assert bindings and max(cov_diff)==0 and max(mu_diff)<1e-12
a=np.loadtxt(D/'objective.csv',delimiter=',');assert np.isfinite(a).all() and a.shape[1]==10
assert np.array_equal(a[:,0],np.arange(len(a))) and np.all(a[:,1]==0)
assert a[:,2].sum()>0,'Temporary lifecycle was not exercised'
on,off=native(D/'outputs.bag'),native(O/'outputs.bag')
assert len(on)==len(a)>531 and on[-1][0]*1e-9>1727775118.455396,'Did not complete111s'
assert (on[-1][0]-on[0][0])*1e-9>110.5
bad=[s for s in ('FL4Q no decreasing','active_window_rollback','Smoother recovery','Persistent BPSAM recovery',
                  'VioBackend::optimize failed','terminate called','Segmentation fault','FL4m prior chart changed') if s in log]
assert not bad,bad
qnorm=max(abs(np.linalg.norm(x[2])-1) for x in on);assert qnorm<1e-10
firstdiff=next((i for i,(x,y) in enumerate(zip(on,off)) if x[3]!=y[3]),None)
v=dict(passed=True,source_playbacks=1,mode='conditional saved-input ON; not live bidirectional',
       native_count=len(on),native_off_count=len(off),fl4q_calls=0,temporary_retirements=int(a[:,2].sum()),
       retirement_updates=int(np.count_nonzero(a[:,2])),first_retirement_update=int(a[np.flatnonzero(a[:,2])[0],0]),
       objective_cost_increase_count=int(np.count_nonzero(a[:,5]>a[:,4]+1e-10)),
       objective_is_observational_not_stationarity=True,first_native_difference_from_off=firstdiff,
       expected_wire=1100,received_wire=len(received),exact_ordered_payloads=True,
       arrival_record_replay_error_s=dict(min=min(arrival_errors),max=max(arrival_errors),median=float(np.median(arrival_errors))),
       applied_alignment_rows=len(bindings),covariance_max_abs_difference=max(cov_diff),raw_mean_max_abs_difference=max(mu_diff),
       bindings=bindings,revision_log_rows=log.count('KIMERA_CBS_REVISION_EDGE_ROW,'),
       max_quaternion_norm_error=float(qnorm),max_native_gap_s=float(np.diff([x[0] for x in on]).max()*1e-9),
       nominal_playback_start_s=1727775007.955396,nominal_playback_end_s=1727775118.955396,
       native_start_gap_s=on[0][0]*1e-9-1727775007.955396,native_end_gap_s=1727775118.955396-on[-1][0]*1e-9,
       capture=json.loads((D/'capture_complete.json').read_text()),output_sha256=sha(D/'outputs.bag'))
with (D/'validation.json').open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
evaluator=W/'runs/gnssfgo_eth1_local_cov_full_cpr7_20261019/prepared/evaluate_fl4f.py'
spec=importlib.util.spec_from_file_location('existing_evaluation',evaluator);ev=importlib.util.module_from_spec(spec);spec.loader.exec_module(ev)
t,p,q,frames,children=ev.rel.load_reference_bag(ev.GT_BAG,ev.GT_TOPIC);assert children==['cpt7_imu']
ref=ev.rel.sort_unique(t,p,q)
curves={name:(np.array([x[0]*1e-9 for x in rows]),np.array([x[1] for x in rows]),np.array([x[2] for x in rows])) for name,rows in [('OFF_ordinary',off),('ON_saved_inputs_ordinary',on)]}
start=max(c[0][0] for c in [*curves.values(),ref]);end=min(c[0][-1] for c in [*curves.values(),ref])
grid=np.arange(math.ceil(start*10)/10,math.floor(end*10)/10+1e-8,.1)
times=np.array([s for s in grid if ev.rel.interpolate_pose(*ref,s,.05) is not None and all(ev.rel.interpolate_pose(*c,s,.5) is not None for c in curves.values())])
metrics={name:dict(full_saved=ev.score_saved(c,ref),common=ev.score(c,ref,times),CPT7_path_m=ev.path_length(c[1])) for name,c in curves.items()}
out=dict(metrics=metrics,reference=dict(path=str(ev.GT_BAG),topic=ev.GT_TOPIC,sha256=sha(ev.GT_BAG),note='Shared-CPT7 IE-TC, not independent prism GT'),
         evaluator_sha256=sha(evaluator),common_mask=dict(rows=len(times),start_s=float(times[0]),end_s=float(times[-1]),span_s=float(times[-1]-times[0]),
         excluded_grid_rows=len(grid)-len(times),max_gap_s=float(np.diff(times).max()),sha256=hashlib.sha256(times.astype('<f8').tobytes()).hexdigest()),
         convention='Native ROS odometry; CPT7-to-prism; rigid SE3/no-scale ATE; 1sRPE; all saved poses vs GT',
         scope='Kimera-only conditional saved-input control; no new GLIM/fusion/live-bidirectional result')
with (D/'evaluation.json').open('x') as f:json.dump(out,f,indent=2,allow_nan=False)
print(json.dumps({k:x for k,x in v.items() if k!='bindings'},indent=2));print(json.dumps(out,indent=2))
