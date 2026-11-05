"""Offline source/covariance/timing checks. Does not run an estimator."""
import collections
import csv
import json
from pathlib import Path
import re
import sys
import numpy as np
W = Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3")
P = W / "runs/main_gpu_batching_cpk16_20261020/k16d_source_1"
K = W / "runs/gnssfgo_eth1_full_rate020_cpk14_20261020/prepared"
sys.path.insert(0, str(K / "transport_tools"))
from live_receiver_protocol import parse_belief

def stats(values):
    x = np.asarray(list(values), dtype=float)
    assert x.size and np.isfinite(x).all()
    return dict(n=len(x), median=float(np.median(x)), p95=float(np.percentile(x, 95)),
                rms=float(np.sqrt(np.mean(x*x))), max=float(np.max(x)))
def load(mode):
    out = P / mode
    assert json.loads((out / "capture_complete.json").read_text())["passed"]
    assert json.loads((out / "cleanup.json").read_text())["all_exited"]
    assert not (out / "failed.txt").exists()
    inputs = json.loads((P / "input_manifest.json").read_text())
    receipts = list(csv.DictReader((out / "receipts.csv").open()))
    for kind, topic in (("imu", "/boxi/cpt7/imu"), ("lidar", "/boxi/hesai/points")):
        actual = [[int(r["seq"]), int(r["stamp_ns"])] for r in receipts if r["kind"] == kind]
        assert actual == [r[:2] for r in inputs["topics"][topic]["messages"]], (kind, "input loss/order")
    odom = [r for r in receipts if r["kind"] == "odom"]
    times = np.array([int(r["stamp_ns"]) for r in odom], dtype=np.int64)
    poses = np.array([[float(r[k]) for k in ("px", "py", "pz", "qx", "qy", "qz", "qw")] for r in odom])
    assert len(odom) >= 590 and np.isfinite(poses).all()
    assert np.all(np.diff(times) > 0) and (times[-1] - times[0]) / 1e9 > 58
    assert np.max(np.abs(np.linalg.norm(poses[:, 3:], axis=1) - 1)) < 1e-6
    lidar = np.array([r[1] for r in inputs["topics"]["/boxi/hesai/points"]["messages"]], dtype=np.int64)
    assert all(np.min(np.abs(lidar - t)) <= 250 for t in times), "unexpected native timestamp"
    # Native pipeline initialization and last-scan look-ahead are explicit.
    assert (inputs["window_end_ns"] - times[-1]) / 1e9 < .3
    provider = pending = None
    first = revised = queries = 0
    seen = set()
    beliefs = {}
    scans, finishes, queues = [], [], []
    first_covariance_error = 0.
    for raw in (out / "node.log").open(errors="strict"):
        line = re.sub(r"\x1b\[[0-9;]*m", "", raw).strip()
        for bad in ("SHARED_NATIVE_PROVIDER_FAILURE", "E1B_PROFILE_SINK_ERROR",
                    "terminate called", "Segmentation fault", "IndeterminantLinearSystemException"):
            assert bad not in line, line
        if "SHARED_NATIVE_PROVIDER_QUERY " in line:
            fields = dict(x.split("=", 1) for x in line.split("SHARED_NATIVE_PROVIDER_QUERY ", 1)[1].split())
            assert int(fields["resets"]) == 0
            assert int(fields["admitted"]) == sum(int(fields[k]) for k in ("active", "absorbed", "removed"))
            provider = (int(fields["from"]) & ((1 << 56)-1), int(fields["to"]) & ((1 << 56)-1),
                np.array([[float(v) for v in r.split(":")] for r in fields["matrix"].split(";")]).reshape(-1))
            queries += 1
        if "GLIM_CBS_REVISION_COVARIANCE_PROVENANCE_ROW," in line:
            f = line.split("GLIM_CBS_REVISION_COVARIANCE_PROVENANCE_ROW,", 1)[1].split(",")
            assert len(f) == 5 and f[-1] == "native_local"
            pending = tuple(map(int, f[:4]))
            assert pending not in seen
            seen.add(pending)
        b = parse_belief("glim", line)
        if b is not None:
            key = (b["i"], b["j"], b["source_revision"])
            assert key not in beliefs
            beliefs[key] = b
            if b["source_revision"]:
                assert pending == (*key, int(b["source_update"]))
                pending = None
                revised += 1
            else:
                assert provider is not None and provider[:2] == key[:2]
                difference = float(np.max(np.abs(provider[2] - b["covariance"])))
                first_covariance_error = max(first_covariance_error, difference)
                assert difference == 0
                first += 1
        if "GLIM_ODOM_IMU_TIMING_ROW," in line:
            f = line.split("GLIM_ODOM_IMU_TIMING_ROW,", 1)[1].split(",")
            assert len(f) == 25 and f[-1] == "processed"
            scans.append(list(map(float, f[:24])))
        if "GLIM_CBS_TIMING_ROW,on_smoother_update_finish," in line:
            finishes.append(list(map(float, line.split("GLIM_CBS_TIMING_ROW,on_smoother_update_finish,", 1)[1].split(","))))
        if "GLIM_ASYNC_ODOM_TIMING_ROW," in line:
            f = line.split("GLIM_ASYNC_ODOM_TIMING_ROW,", 1)[1].split(",")
            if f[-1] == "processed":
                queues.append(int(f[9]))
    assert first >= len(odom) - 3 and revised > 0 and len(seen) == revised and pending is None
    # Frame0 initializes without a scan-timing row. The final buffered scan is
    # processed during shutdown after the bridge's output callback is removed.
    # Keep that tail separate from the online timing/output cohort.
    online_ids = {int(r["seq"]) for r in odom}
    timed_ids = {int(r[1]) for r in scans}
    assert online_ids == set(range(len(odom)))
    assert timed_ids == set(range(1, len(odom) + 1)), (timed_ids, online_ids)
    shutdown_scans = [r for r in scans if int(r[1]) not in online_ids]
    assert len(shutdown_scans) == 1
    scans = [r for r in scans if int(r[1]) in online_ids]
    profile = [json.loads(line) for line in (out / "e1b_profile.jsonl").open()]
    assert all(not r["aborted"] for r in profile)
    main = [r for r in profile if r["context"] == 1]
    assert len(main) == len(odom) + 1
    main = [r for r in main if r["timestamp"] <= times[-1] / 1e9 + 1e-7]
    assert len(main) == len(odom)
    work = collections.Counter()
    for row in main:
        for k, v in row["owner_wall"].items():
            work[k] += v[0]
    assert work["gpu_linearize"] > 0
    cache_hits = sum(r["metrics"][7] for r in main)
    if mode == "batched":
        assert work["gpu_sync_linearize"] == 0, work
        assert cache_hits == work["gpu_linearize"], (cache_hits, work)
    else:
        assert work["gpu_sync_linearize"] > 0
        assert cache_hits == 0
    bins = {}
    start_s = inputs["window_start_ns"] / 1e9
    by_id = {int(r["seq"]): int(r["stamp_ns"]) / 1e9 for r in odom}
    for name, begin, end in (("whole", 0, 60), ("filled_10_60", 10, 60), ("last20", 40, 60)):
        selected = [r for r in scans if begin <= r[0] - start_s < end]
        selected_finish = [r for r in finishes if begin <= by_id.get(int(r[0]), 0) - start_s < end]
        bins[name] = {label: stats(r[col] for r in selected) for label, col in
            (("scan_ms", 23), ("main_ms", 17), ("post_callback_ms", 18), ("marginalization_ms", 19))}
        bins[name]["export_ms"] = stats(r[12] for r in selected_finish)
        bins[name]["scan_fraction_over_500ms"] = sum(r[23] > 500 for r in selected) / len(selected)
    report = dict(passed=True, mode=mode, input_counts={k: len(v["messages"]) for k, v in inputs["topics"].items()},
        native_count=len(odom), native_first_ns=int(times[0]), native_last_ns=int(times[-1]),
        output_span_s=float((times[-1]-times[0])/1e9),
        startup_gap_s=float((times[0]-inputs["window_start_ns"])/1e9),
        tail_gap_s=float((inputs["window_end_ns"]-times[-1])/1e9),
        path_m=float(np.sum(np.linalg.norm(np.diff(poses[:, :3], axis=0), axis=1))),
        first=first, revised=revised, provider_queries=queries, first_covariance_error=first_covariance_error,
        shutdown_only_scan_ids=[int(r[1]) for r in shutdown_scans],
        per_revision_native_local_provenance=True, profile_main_work=dict(work),
        prepared_gpu_consumptions=cache_hits,
        timing=bins, queue_count=stats(queues),
        output_receipt_ros_age_ms=stats((int(r["receipt_ros_ns"])-int(r["stamp_ns"]))/1e6 for r in odom))
    return report, times, poses, beliefs

if __name__ == "__main__":
    mode = sys.argv[1]
    assert mode in ("reference", "batched")
    report, _, _, _ = load(mode)
    with (P / mode / "validation.json").open("x") as f:
        json.dump(report, f, indent=2)
    print(json.dumps(report, indent=2))
