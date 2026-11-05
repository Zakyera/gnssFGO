"""Container post-check: saved ROS messages and passive observer agree."""
import collections
import csv
import hashlib
import io
import json
from pathlib import Path
import rosbag
P = Path("/workspace/cbs_gtsam4.3/runs/main_gpu_batching_cpk16_20261020/k16d_source_1")
reports = {}
for mode in ("reference", "batched"):
    out = P / mode
    assert json.loads((out / "capture_complete.json").read_text())["passed"]
    expected = [r for r in csv.DictReader((out / "receipts.csv").open()) if r["kind"] == "odom"]
    pose_i = 0
    counts = collections.Counter()
    digests = collections.defaultdict(hashlib.sha256)
    with rosbag.Bag(str(out / "outputs.bag")) as bag:
        for topic, msg, record in bag.read_messages():
            counts[topic] += 1
            data = io.BytesIO()
            msg.serialize(data)
            digests[topic].update(data.getvalue())
            if topic == "/glim/cbs/odometry":
                row = expected[pose_i]
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                assert (msg.header.seq, msg.header.stamp.to_nsec()) == (int(row["seq"]), int(row["stamp_ns"]))
                assert [p.x, p.y, p.z, q.x, q.y, q.z, q.w] == [
                    float(row[k]) for k in ("px", "py", "pz", "qx", "qy", "qz", "qw")]
                pose_i += 1
    assert pose_i == len(expected)
    val = json.loads((out / "validation.json").read_text())
    assert counts["/kimera/cbs/odom_belief_in"] == val["first"]
    reports[mode] = dict(counts=dict(counts), observer_matches_wire_exactly=True,
        message_bytes_sha256={k: v.hexdigest() for k, v in digests.items()})
reports["same_topic_counts"] = reports["reference"]["counts"] == reports["batched"]["counts"]
assert reports["same_topic_counts"]
with (P / "wire_check.json").open("x") as f:
    json.dump(reports, f, indent=2)
print(json.dumps(reports, indent=2))
