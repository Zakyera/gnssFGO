#!/usr/bin/env python3
"""Export ROS bag record times for Kimera smoother output update IDs.

Run with ROS Noetic Python in the source container. This distinguishes the
physical header stamp from when the same-update output was actually recorded.
"""

import argparse
import csv
import rosbag


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("out")
    args = parser.parse_args()
    with rosbag.Bag(args.bag) as bag, open(args.out, "w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(["update_frame_id", "physical_timestamp_ns",
                         "record_timestamp_ns"])
        seen = set()
        for _, message, stamp in bag.read_messages(
                topics=["/kimera_vio_ros/odometry"]):
            update = int(message.header.seq)
            if update in seen:
                raise ValueError(f"duplicate Kimera smoother update {update}")
            seen.add(update)
            writer.writerow([update, message.header.stamp.to_nsec(), stamp.to_nsec()])


if __name__ == "__main__":
    main()
