"""Small matched passive observer. No measurement publication or estimator work."""
import csv
import sys
import threading
import time
from pathlib import Path
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
out = Path(sys.argv[1])
rospy.init_node("cpk16d_observer", anonymous=False)
lock = threading.Lock()
f = (out / "receipts.csv").open("x", buffering=1)
w = csv.writer(f)
w.writerow(["kind", "seq", "stamp_ns", "receipt_ros_ns", "wall_ns",
            "px", "py", "pz", "qx", "qy", "qz", "qw"])
def callback(msg, kind):
    row = [kind, msg.header.seq, msg.header.stamp.to_nsec(),
           rospy.Time.now().to_nsec(), time.time_ns()]
    if kind == "odom":
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        row += [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    with lock:
        w.writerow(row)
subs = [rospy.Subscriber("/boxi/cpt7/imu", Imu, callback, "imu", queue_size=10000),
        rospy.Subscriber("/boxi/hesai/points", PointCloud2, callback, "lidar",
                         queue_size=100, buff_size=64 * 1024 * 1024),
        rospy.Subscriber("/glim/cbs/odometry", Odometry, callback, "odom", queue_size=10000)]
(out / "observer_ready").touch(exist_ok=False)
rospy.spin()
f.close()
