#!/usr/bin/env python3
"""Export retained IE-TC and GLIM ROS poses for the bounded source audit."""
import argparse
from pathlib import Path
import sys
import numpy as np
import rosbag

sys.path.insert(0, "/workspace/cbs_gtsam4.3/src/cbsms/tools")
from score_grandtour_relative_pose import load_reference_bag, sort_unique

p = argparse.ArgumentParser()
p.add_argument("--reference-bag", required=True, type=Path)
p.add_argument("--glim-bag", required=True, type=Path)
p.add_argument("--out", required=True, type=Path)
args = p.parse_args()
args.out.mkdir(parents=True, exist_ok=True)
t, x, q, frames, children = load_reference_bag(
    args.reference_bag, "/boxi/inertial_explorer/tc/odometry")
t, x, q = sort_unique(t, x, q)
np.savez(args.out/"ie_tc.npz", times=t, positions=x, quaternions=q)
rows = []
with rosbag.Bag(str(args.glim_bag)) as bag:
    for _, m, _ in bag.read_messages(topics=["/glim/cbs/odometry"]):
        rows.append([m.header.stamp.to_sec(), m.pose.pose.position.x,
                     m.pose.pose.position.y, m.pose.pose.position.z,
                     m.pose.pose.orientation.x, m.pose.pose.orientation.y,
                     m.pose.pose.orientation.z, m.pose.pose.orientation.w])
np.savez(args.out/"glim_online.npz", poses=np.asarray(rows))
print({"reference_frames": frames, "reference_children": children,
       "reference_rows": len(t), "glim_online_rows": len(rows)})
