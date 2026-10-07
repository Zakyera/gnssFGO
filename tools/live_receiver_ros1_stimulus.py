#!/usr/bin/env python3
"""Transport smoke fixture: retained short IMU/beliefs, no estimator execution."""
import argparse
import json
import time
from decimal import Decimal
from pathlib import Path


def main():
    import rospy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Imu
    from geometry_msgs.msg import PoseStamped
    from std_srvs.srv import Empty
    p = argparse.ArgumentParser()
    p.add_argument('--journal', type=Path, required=True)
    p.add_argument('--source-run', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--rate', type=float, default=0.1)
    args = p.parse_args()
    rospy.init_node('gnssfgo_transport_test_stimulus', disable_signals=True)
    clock = rospy.Publisher('/clock', Clock, queue_size=100, latch=True)
    imu = rospy.Publisher('/boxi/cpt7/imu', Imu, queue_size=10000)
    poses = []
    rospy.Subscriber('/gnssfgo/pose', PoseStamped, lambda m: poses.append(m.header.stamp.to_nsec()))
    logdir = args.source_run/'ros_logs/test'
    logdir.mkdir(parents=True, exist_ok=False)
    g = (logdir/'glim_rosnode-1-stdout.log').open('x', buffering=1)
    k = (logdir/'kimera_vio_ros-kimera_vio_ros_node-4-stdout.log').open('x', buffering=1)
    rospy.wait_for_service('/gnssfgo/finish', timeout=30)
    deadline = time.monotonic()+10
    while imu.get_num_connections() == 0:
        if time.monotonic() > deadline:
            raise RuntimeError('no IMU subscriber')
        time.sleep(0.05)
    lines = [line.split(',') for line in args.journal.read_text().splitlines() if not line.startswith('E,')]
    first = int(lines[0][1])
    start = time.monotonic()
    imus = 0
    sec = lambda ns: str(Decimal(ns)/Decimal(1000000000))
    for f in lines:
        arrival = int(f[1])
        due = start+(arrival-first)/1e9/args.rate
        time.sleep(max(0, due-time.monotonic()))
        # ROS clock distribution is itself asynchronous. Give it an explicitly
        # recorded 1 ms transport margin; never assign a false source timestamp.
        current = arrival+1000000
        clock.publish(Clock(rospy.Time(current//1000000000, current%1000000000)))
        time.sleep(0.003)
        if f[0] == 'I':
            msg = Imu()
            stamp = int(f[2])
            msg.header.stamp = rospy.Time(stamp//1000000000, stamp%1000000000)
            msg.header.frame_id = 'cpt7_imu'
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = map(float, f[3:6])
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = map(float, f[6:9])
            imu.publish(msg)
            imus += 1
        elif f[0] == 'B':
            _, _, source, session, ordinal, revision, i, j, ti, tj, *payload = f
            if source in ('g', 'glim'):
                marker = 'GLIM_CBS_EXPORT_BELIEF_ROW' if revision == '0' else 'GLIM_CBS_REVISION_BELIEF_ROW'
                mu = ';'.join(payload[:6])
                cov = ';'.join(':'.join(payload[6+r*6:12+r*6]) for r in range(6))
                g.write(','.join([marker, 'g', i, j, revision, sec(arrival), ordinal, sec(ti), sec(tj), mu, cov])+'\n')
            else:
                k.write(','.join(['KIMERA_CBS_REVISION_EDGE_ROW', ordinal, 'k', i, j,
                                  sec(ti), sec(tj), revision, 'first' if revision == '0' else 'active',
                                  *(['0']*14), *payload])+'\n')
    g.close(); k.close()
    time.sleep(2)
    before_finish = len(poses)
    if before_finish == 0:
        raise RuntimeError('no fused pose published while stream was live')
    rospy.ServiceProxy('/gnssfgo/finish', Empty)()
    deadline = time.monotonic()+180
    while not (args.out/'transport_complete.json').exists():
        if (args.out/'transport_failed.txt').exists() or time.monotonic() > deadline:
            raise RuntimeError('adapter failed or did not finish')
        time.sleep(0.1)
    summary = json.loads((args.out/'transport_complete.json').read_text())
    assert summary['imu'] == imus, (summary['imu'], imus)
    result = dict(passed=True, poses_received_before_finish=before_finish,
                  input_imu=imus, adapter_summary=summary,
                  note='Retained-data ROS transport fixture, not co-running GLIM/Kimera')
    (args.out/'ros_smoke_validation.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
