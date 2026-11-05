"""Supplemental scoring of verified actual ROS receipts; retain shutdown extras."""
import csv,importlib.util,json
from pathlib import Path
import numpy as np
from rosbags.rosbag1 import Reader
H=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('cpv29_original_analysis',H/'analyze.py')
v=importlib.util.module_from_spec(spec);spec.loader.exec_module(v)
m=v.m

def live_glim(src):
    curve,arrays,checks=m.glim(src)
    allrows=v.csvrows(src/'glim_incremental_pairs.csv');n=len(curve[0])
    rr=v.csvrows(src/'glim_incremental_ros.csv')
    assert n>=3430 and len(rr)==n and len(allrows)==n+2
    assert [int(x['index']) for x in allrows]==list(range(n+2))
    assert [int(x['field.header.seq']) for x in rr]==list(range(n))
    ts=np.array([int(x['stamp_ns']) for x in allrows],np.int64)
    assert np.all(np.diff(ts)>0) and np.array_equal(ts[:n],curve[0])
    assert [int(x['field.header.stamp']) for x in rr]==ts[:n].tolist()
    assert {x['field.header.frame_id'] for x in rr}=={'glim_incremental_odom_cpv29'}
    av=np.array([int(x['availability_ros_ns']) for x in allrows],np.int64)
    wall=np.array([int(x['wall_ns']) for x in allrows],np.int64)
    assert np.all(av>=ts) and np.all(np.diff(av)>=0) and np.all(np.diff(wall)>0)
    assert (wall[n]-wall[n-1])*1e-9>25,'extras must be after frozen25s drain'
    bagids={}
    bag=m.pin(src/'recordings/cbs.bag')
    with Reader(bag) as reader:
        conns=[c for c in reader.connections if c.topic in ('/glim/cbs/odometry','/glim/incremental_pose_cpv29')]
        for conn,stamp,data in reader.messages(connections=conns):bagids.setdefault(conn.topic,[]).append(int.from_bytes(data[:4],'little'))
    assert len(bagids)==2 and all(ids==list(range(n)) for ids in bagids.values())
    pair={k:v.poses(allrows,k) for k in ('current','previous','incremental','graph_T_odom')}
    errors={k:float(np.max(abs(pair[k][:n]-arrays[name]))) for k,name in
            [('current','graph_body_T'),('previous','previous_same_update_body_T'),
             ('incremental','chained_body_T'),('graph_T_odom','graph_T_odometry')]}
    assert max(errors.values())<1e-8,errors
    _,aa,extra_checks=m.verify_chain(ts,pair['current'],pair['previous'])
    entire_error=float(np.max(abs(pair['incremental']-aa['chained_body_T'])))
    assert entire_error<1e-8
    ros=np.array([m.base.matrix([float(x['field.pose.position.'+k]) for k in 'xyz'],
                 [float(x['field.pose.orientation.'+k]) for k in 'xyzw']) for x in rr])
    errors['ros_vs_live_csv']=float(np.max(abs(ros-pair['incremental'][:n])))
    assert errors['ros_vs_live_csv']<1e-12
    checks.update(passed=True,live_vs_independent_max_matrix_errors=errors,
        live_ros_rows=n,observer_csv_rows=len(allrows),bag_topic_rows={k:len(ids) for k,ids in bagids.items()},
        no_interior_receipt_omission=True,excluded_unrecorded_shutdown_observer_rows=allrows[n:],
        unrecorded_tail_span_s=float((ts[-1]-ts[n-1])*1e-9),
        all_observer_rows_chain_max=entire_error,all_observer_rows_checks=extra_checks,
        latency_ros_ms=np.percentile((av[:n]-ts[:n])*1e-6,[50,95,100]).tolist(),
        physical_stamps_exact=True,incremental_frame='glim_incremental_odom_cpv29',newest_pose_body='cpt7_imu')
    arrays['availability_ros_ns']=av[:n];arrays['wall_ns']=wall[:n]
    for k,val in pair.items():arrays['all_observer_'+k]=val
    arrays['all_observer_timestamp_ns']=ts
    for p in (Path(__file__),H/'TERMINAL_RECEIPT_AUDIT.txt',src/'experiment_runner_used.sh'):m.pin(p)
    return (ts[:n],pair['incremental'][:n,:3,3],pair['incremental'][:n,:3,:3]),arrays,checks

if __name__=='__main__':
    assert (H/'reporting_repair_failure.json').exists(),'preserve original strict scoring failure first'
    assert json.loads((H/'fusion_parity.json').read_text())['pass_']
    assert not (H/'results.json').exists()
    v.live_glim=live_glim
    v.main()
