#!/usr/bin/env python3
"""Certify a completed diagnostic source capture after ROS log redirection.

The legacy runner checks only estimators.log. With node output=log, GLIM's
injection records and Kimera's applied-factor records are in per-node files.
This validator checks the same completed capture using the actual log sinks;
it never changes a source estimator or a recorded measurement.
"""

import argparse
import csv
import json
from pathlib import Path

import rosbag


def selected_file(run, pattern):
    files = [path for path in (run / "ros_logs").glob(pattern)
             if path.parent.name != "latest"]
    if len(files) != 1:
        raise ValueError(f"expected one {pattern}, found {len(files)}")
    return files[0]


def load_rows(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def count_token(path, token, width=None, required=None):
    count = 0
    malformed = 0
    with path.open(errors="replace") as stream:
        for line in stream:
            if token not in line or (required is not None and required not in line):
                continue
            count += 1
            if width is not None:
                body = line.split(token, 1)[1].split("\x1b", 1)[0].strip()
                if len(body.split(",")) != width:
                    malformed += 1
    return count, malformed


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--mode", choices=("off", "on"), required=True)
    args = parser.parse_args()
    run = args.run
    glim_log = selected_file(run, "*/glim_rosnode-1-stdout.log")
    kimera_node_log = selected_file(
        run, "*/kimera_vio_ros-kimera_vio_ros_node-4-stdout.log")
    kimera_audit_log = run / "estimators.log"
    glim = load_rows(run / "glim_odometry.csv")
    kimera = load_rows(run / "kimera_odometry.csv")
    if (len(glim), len(kimera)) != (1332, 667):
        raise ValueError(f"incomplete pose streams: {len(glim)}/{len(kimera)}")
    kimera_duration = (int(kimera[-1]["%time"]) - int(kimera[0]["%time"])) / 1e9
    if kimera_duration < 134.0 * 0.90:
        raise ValueError(f"Kimera coverage below runner threshold: {kimera_duration}")
    bag_path = run / "recordings" / "cbs.bag"
    with rosbag.Bag(str(bag_path)) as bag:
        topics = bag.get_type_and_topic_info().topics
        glim_messages = topics["/kimera/cbs/odom_belief_in"].message_count
        kimera_messages = topics["/kimera/cbs/odom_belief_out"].message_count
    if glim_messages != 1330 or kimera_messages < 1:
        raise ValueError(f"belief recorder incomplete: {glim_messages}/{kimera_messages}")
    glim_revisions, glim_malformed = count_token(
        glim_log, "GLIM_CBS_REVISION_BELIEF_ROW,", 10)
    kimera_revisions, kimera_malformed = count_token(
        kimera_audit_log, "KIMERA_CBS_REVISION_EDGE_ROW,", 64)
    if glim_revisions < 1330 or kimera_revisions < kimera_messages:
        raise ValueError("missing revision audit rows")
    if glim_malformed or kimera_malformed:
        raise ValueError(f"malformed revision rows: {glim_malformed}/{kimera_malformed}")
    errors = sum(count_token(path, token)[0] for path, token in (
        (glim_log, "GLIM_CBS_REVISION_ERROR_ROW,"),
        (kimera_audit_log, "KIMERA_CBS_REVISION_EDGE_ERROR_ROW,")))
    if errors:
        raise ValueError(f"revision-source errors: {errors}")
    g2k_queued, _ = count_token(
        kimera_audit_log, "CBS_BPSAM_ODOM_ADD_ROW_G2K,",
        required="sender_odom_queued")
    g2k_applied, _ = count_token(
        kimera_node_log, "CBS_ODOM_FACTOR_COVARIANCE_ROW,G2K,",
        required="persistent_odom_applied")
    k2g_applied, _ = count_token(
        glim_log, "GLIM_CBS_ODOM_INJECT_ROW,",
        required="injected_persistent")
    if args.mode == "off":
        if g2k_queued or g2k_applied or k2g_applied:
            raise ValueError("CBS-OFF unexpectedly has injected factors")
    elif min(g2k_queued, g2k_applied, k2g_applied) < 1:
        raise ValueError("classic CBS-ON lacks bidirectional factor evidence")
    result = {
        "mode": args.mode, "glim_poses": len(glim), "kimera_poses": len(kimera),
        "kimera_duration_s": kimera_duration,
        "glim_belief_messages": glim_messages,
        "kimera_belief_messages": kimera_messages,
        "glim_revision_rows": glim_revisions,
        "kimera_revision_rows": kimera_revisions,
        "g2k_queued": g2k_queued, "g2k_applied": g2k_applied,
        "k2g_injected": k2g_applied,
        "revision_errors": errors, "validated": True,
        "note": "Post-run validation of the recorded bags and separate node logs; the source estimator is unchanged.",
    }
    (run / "diagnostic_validation.json").write_text(json.dumps(result, indent=2) + "\n")
    (run / "validated_complete.marker").touch()
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
