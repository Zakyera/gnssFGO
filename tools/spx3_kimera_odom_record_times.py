#!/usr/bin/env python3
"""Export actual ROS-bag receipt times for Kimera backend update IDs."""

import argparse
import csv

import rosbag


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    seen = set()
    with rosbag.Bag(args.bag, "r") as bag, open(args.out, "w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("update_frame_id", "physical_timestamp_ns", "record_timestamp_ns"))
        for _, msg, receipt in bag.read_messages(topics=["/kimera_vio_ros/odometry"]):
            update = int(msg.header.seq)
            if update in seen:
                raise ValueError(f"duplicate Kimera backend update {update}")
            seen.add(update)
            writer.writerow((update, msg.header.stamp.to_nsec(), receipt.to_nsec()))
    if not seen:
        raise ValueError("no recorded Kimera odometry updates")


if __name__ == "__main__":
    main()
