"""Container preflight: exact selected input identities without a source run."""
import hashlib
import io
import json
from pathlib import Path
import rosbag
import rospy
P = Path(__file__).resolve().parent
D = Path("/workspace/cbs_gtsam4.3/src/datasets/GrandTour")
start = 1727775007855396309
end = start + 60_000_000_000
manifest = dict(window_start_ns=start, window_end_ns=end, topics={})
earliest = []
for suffix, topic in (("hesai", "/boxi/hesai/points"), ("cpt7_imu", "/boxi/cpt7/imu")):
    p = D / ("2024-10-01-11-29-55_" + suffix + ".bag")
    digest = hashlib.sha256()
    rows = []
    with rosbag.Bag(str(p)) as bag:
        for _, msg, t in bag.read_messages(topics=[topic],
                start_time=rospy.Time(start // 10**9, start % 10**9),
                end_time=rospy.Time(end // 10**9, end % 10**9)):
            assert start <= t.to_nsec() < end
            stream = io.BytesIO()
            msg.serialize(stream)
            digest.update(t.to_nsec().to_bytes(8, "little"))
            digest.update(stream.getvalue())
            rows.append([msg.header.seq, msg.header.stamp.to_nsec(), t.to_nsec()])
    assert rows
    earliest.append(rows[0][2])
    manifest["topics"][topic] = dict(bag=str(p), bytes=p.stat().st_size,
        mtime_ns=p.stat().st_mtime_ns, messages=rows, sha256=digest.hexdigest())
# rosbag starts at the first selected sensor record (camera is not played).
# Shorten its duration by the 148ms empty prefix to keep the exact old end epoch.
manifest["playback_start_ns"] = min(earliest)
manifest["playback_duration_s"] = (end - min(earliest)) / 1e9
with (P / "input_manifest.json").open("x") as f:
    json.dump(manifest, f, indent=2)
print(json.dumps({**{k: v for k, v in manifest.items() if k != "topics"},
                  "counts": {k: len(v["messages"]) for k, v in manifest["topics"].items()}}, indent=2))
