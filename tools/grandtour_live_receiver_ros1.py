#!/usr/bin/env python3
"""ROS1 input/output adapter for the existing gnssFGO streaming executable.

Classic CBS topics are untouched. Revision logs are an interim, observable
transport, not a new estimator. Availability is observation time, never a
backdated endpoint/source-log timestamp. No reweighting or covariance edits.
"""
import argparse
import csv
import glob
import json
import os
from pathlib import Path
import queue
import threading
import time
import traceback

from live_receiver_protocol import LineTail, Versions, parse_belief, parse_online_pose, validate_config


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as RosPath
    from sensor_msgs.msg import Imu
    from std_srvs.srv import Empty, EmptyResponse
    p = argparse.ArgumentParser()
    p.add_argument('--config', required=True, type=Path)
    p.add_argument('--out', required=True, type=Path)
    p.add_argument('--fifo', required=True, type=Path)
    args = p.parse_args()
    config = json.loads(args.config.read_text())
    validate_config(config)
    os.environ['ROS_MASTER_URI'] = config['ros_master_uri']
    rospy.init_node('gnssfgo_live_receiver_transport', anonymous=False, disable_signals=True)
    if not rospy.get_param('/use_sim_time', False):
        raise RuntimeError('GrandTour live preset requires /use_sim_time=true')
    lock = threading.RLock()
    queue_limit = int(config.get('max_queued_events', 200000))
    pending = queue.Queue(maxsize=queue_limit)
    error = []
    finishing = threading.Event()
    stop_capture = threading.Event()
    counts = dict(imu=0, duplicate_imu=0, imu_clock_lag_events=0,
                  max_imu_clock_lag_ns=0, belief_clock_lag_events=0,
                  max_belief_clock_lag_ns=0, beliefs=0, duplicates=0,
                  published_poses=0, max_queue=0)
    source_counts = {s['id']: 0 for s in config['sources']}
    last_clock, last_raw_clock, last_imu, first_clock = 0, 0, 0, 0
    last_imu_values = None
    versions = Versions()
    tails = {}
    pose_tail = None
    pose_header = None
    journal = (args.out/'events.csv').open('x', buffering=1)
    receipts = (args.out/'receipts.jsonl').open('x', buffering=1)
    state_path = args.out/'receiver/online_states.csv'
    pose_pub = rospy.Publisher('/gnssfgo/pose', PoseStamped, queue_size=100)
    path_pub = rospy.Publisher('/gnssfgo/path', RosPath, queue_size=1, latch=True)
    path_msg = RosPath()
    path_msg.header.frame_id = config.get('world_frame', 'gnssfgo_world')

    def fail(exc):
        with lock:
            if not error:
                error.append(str(exc))
                (args.out/'transport_failed.txt').write_text(str(exc)+'\n')
        finishing.set()

    def observe_clock(physical_floor=0, kind=None):
        nonlocal last_clock, last_raw_clock, first_clock
        raw = rospy.Time.now().to_nsec()
        if raw <= 0 or raw < last_raw_clock:
            raise RuntimeError('ROS clock unavailable or moved backwards; start a new session')
        last_raw_clock = raw
        if raw < physical_floor:
            if kind == 'imu':
                counts['imu_clock_lag_events'] += 1
                counts['max_imu_clock_lag_ns'] = max(counts['max_imu_clock_lag_ns'], physical_floor-raw)
            elif kind == 'belief':
                counts['belief_clock_lag_events'] += 1
                counts['max_belief_clock_lag_ns'] = max(counts['max_belief_clock_lag_ns'], physical_floor-raw)
        # ROS /clock and sensor topics are delivered on different connections.
        # A received sample proves that physical time has reached its stamp,
        # even when the latest /clock callback is still in transit.
        now = max(raw, physical_floor, last_clock)
        last_clock = now
        if not first_clock:
            first_clock = now
        return now

    def enqueue(fields, metadata=None):
        # Caller holds lock: this establishes deterministic observation order.
        line = ','.join(format(x, '.17g') if isinstance(x, float) else str(x) for x in fields)+'\n'
        if pending.full():
            raise RuntimeError('live transport backlog overflow; no silent event dropping')
        journal.write(line)
        receipts.write(json.dumps(dict(event=sum((counts['imu'], counts['beliefs'])),
                                       receipt_ros_ns=fields[1], wall_ns=time.time_ns(),
                                       kind=fields[0], metadata=metadata))+'\n')
        pending.put_nowait(line)
        counts['max_queue'] = max(counts['max_queue'], pending.qsize())

    def imu_callback(msg):
        nonlocal last_imu, last_imu_values
        try:
            with lock:
                if stop_capture.is_set() or error:
                    return
                stamp = msg.header.stamp.to_nsec()
                now = observe_clock(stamp, 'imu')
                if msg.header.frame_id != config['imu_frame']:
                    raise RuntimeError('unexpected IMU frame '+msg.header.frame_id)
                a, w = msg.linear_acceleration, msg.angular_velocity
                values = (a.x, a.y, a.z, w.x, w.y, w.z)
                if stamp == last_imu and values == last_imu_values:
                    # A repeated ROS delivery of the identical stamped sample
                    # contributes no new IMU information. Keep it in status,
                    # but do not give the ALL_RIGHT integrator two copies.
                    counts['duplicate_imu'] += 1
                    return
                if stamp <= last_imu:
                    raise RuntimeError(f'out-of-order IMU timestamp: stamp={stamp} last={last_imu}')
                if last_imu and stamp-last_imu > int(config.get('max_imu_gap_ns', 30000000)):
                    raise RuntimeError('IMU gap exceeds explicit 30 ms preset; inspect dropped input')
                last_imu = stamp
                last_imu_values = values
                enqueue(['I', now, stamp, a.x, a.y, a.z, w.x, w.y, w.z])
                counts['imu'] += 1
        except Exception as exc:
            fail(exc)

    def writer():
        try:
            with args.fifo.open('w', buffering=1) as stream:
                while True:
                    line = pending.get()
                    if line is None:
                        break
                    stream.write(line)
        except Exception as exc:
            fail(exc)

    def finish(_request):
        finishing.set()
        return EmptyResponse()

    def poll_sources():
        for source in config['sources']:
            sid = source['id']
            if sid not in tails:
                # ROS creates a `latest` symlink beside the real run UUID.
                # Both glob paths can name the same live inode.
                matches = sorted({str(Path(p).resolve()) for p in glob.glob(source['log_glob'])})
                if len(matches) > 1:
                    raise RuntimeError('ambiguous log_glob for '+sid)
                if not matches:
                    continue
                # A late attachment reads old rows only NOW, never backdates them.
                tails[sid] = LineTail(matches[0])
            for line in tails[sid].read():
                belief = parse_belief(source['kind'], line)
                if belief is None:
                    continue
                with lock:
                    now = observe_clock(belief['tj'], 'belief')
                    fields = versions.accept(sid, source['session'], belief, now)
                    if fields is None:
                        counts['duplicates'] += 1
                    else:
                        enqueue(fields, dict(source=sid, source_revision=belief['source_revision'],
                                             logger_source=belief['logger_source'],
                                             source_update=belief['source_update'],
                                             source_record=belief['source_record']))
                        counts['beliefs'] += 1
                        source_counts[sid] += 1

    def publish_states():
        nonlocal pose_tail, pose_header
        if pose_tail is None and state_path.exists():
            pose_tail = LineTail(state_path)
        if pose_tail is None:
            return
        changed = False
        for line in pose_tail.read():
            if not line:
                continue
            if line.startswith('state_index,'):
                pose_header = line.split(',')
                continue
            stamp, position, orientation = parse_online_pose(pose_header, line)
            pose = PoseStamped()
            pose.header.stamp = rospy.Time(stamp//1000000000, stamp%1000000000)
            pose.header.frame_id = path_msg.header.frame_id
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
            (pose.pose.orientation.x, pose.pose.orientation.y,
             pose.pose.orientation.z, pose.pose.orientation.w) = orientation
            pose_pub.publish(pose)
            path_msg.header.stamp = pose.header.stamp
            path_msg.poses.append(pose)
            counts['published_poses'] += 1
            changed = True
        if changed:
            path_pub.publish(path_msg)

    rospy.Subscriber(config['imu_topic'], Imu, imu_callback, queue_size=10000, tcp_nodelay=True)
    rospy.Service('/gnssfgo/finish', Empty, finish)
    worker = threading.Thread(target=writer, daemon=True)
    worker.start()
    (args.out/'transport_ready.marker').write_text('subscribed; waiting for data\n')
    last_status = 0.0
    try:
        while not finishing.is_set() and not rospy.is_shutdown():
            poll_sources()
            publish_states()
            if (args.out/'STOP').exists():
                finishing.set()
            if time.monotonic()-last_status > 1.0:
                with lock:
                    status = dict(counts, sources=source_counts, queued=pending.qsize(),
                                  last_ros_ns=last_clock, last_imu_ns=last_imu,
                                  last_published_ns=path_msg.header.stamp.to_nsec())
                    status['output_age_seconds'] = ((last_clock-path_msg.header.stamp.to_nsec())/1e9
                                                    if path_msg.poses else None)
                    (args.out/'transport_status.json').write_text(json.dumps(status, indent=2)+'\n')
                last_status = time.monotonic()
            time.sleep(0.01)
        # Source processes must have drained before requesting finish. No timer
        # guesses can certify that all source computations have completed.
        with lock:
            stop_capture.set()
        poll_sources()
        if error:
            raise RuntimeError(error[0])
        if any(s not in tails or source_counts[s] == 0 for s in source_counts):
            raise RuntimeError('one or more configured sources produced no revision rows')
        with lock:
            enqueue(['E', observe_clock()])
        pending.put(None, timeout=10)
        worker.join(timeout=float(config.get('drain_timeout_wall_sec', 180)))
        if worker.is_alive() or error:
            raise RuntimeError(error[0] if error else 'receiver did not drain before timeout')
        deadline = time.monotonic()+float(config.get('drain_timeout_wall_sec', 180))
        while not (args.out/'receiver/summary.json').exists():
            if time.monotonic() > deadline:
                raise RuntimeError('receiver finalization timed out')
            publish_states()
            time.sleep(0.1)
        publish_states()
        (args.out/'transport_complete.json').write_text(json.dumps(dict(counts, sources=source_counts), indent=2)+'\n')
    except BaseException as exc:
        fail(exc)
        raise
    finally:
        journal.close()
        receipts.close()


if __name__ == '__main__':
    main()
