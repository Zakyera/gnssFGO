"""Read retained logs only; no estimator, replay, or configuration changes."""
import hashlib
import json
import pathlib
import re

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
ANSI = re.compile(r"\x1b\[[0-9;]*m")
ROS_TIME = re.compile(r"\[([0-9.]+), ([0-9.]+)\]")
T0 = 1727775008.003475
COLS = "stamp frame points nimu nfactors active marginalized lookup inter_imu imufactor intra_imu deskew pointcov cpuframe createframe createfactors pre_callback smoother post_callback marginalize updateframes imuvalidate outputcallbacks total".split()


def stats(values):
    a = np.asarray(values, dtype=float)
    if not a.size:
        return {"n": 0}
    return dict(n=int(a.size), mean=float(a.mean()), median=float(np.median(a)),
                p95=float(np.quantile(a, .95)), max=float(a.max()), sum=float(a.sum()))


def read(mode):
    source = ROOT / "prepared" / f"full160_{mode}_source"
    paths = list((source / "ros_logs").glob("*/glim_rosnode-*-stdout.log"))
    assert len(paths) == 1
    path = paths[0]
    odom, cbs, queues, competition = {}, {}, [], []
    active_diagnostics = 0
    diagnostic_interval_calls = []
    native_input = []
    for line in path.open():
        line = ANSI.sub("", line.strip())
        if "GLIM_ODOM_IMU_TIMING_ROW," in line:
            v = line.split("GLIM_ODOM_IMU_TIMING_ROW,", 1)[1].split(",")
            assert len(v) == 25 and v[-1] == "processed"
            row = dict(zip(COLS, map(float, v[:-1])))
            assert int(row["frame"]) not in odom
            odom[int(row["frame"])] = row
        elif "GLIM_CBS_TIMING_ROW," in line:
            v = line.split("GLIM_CBS_TIMING_ROW,", 1)[1].split(",")
            kind, a = v[0], list(map(float, v[1:]))
            ts = ROS_TIME.search(line)
            if ts:
                cbs.setdefault(kind, {})[int(a[0])] = (a, float(ts[2]))
        elif "GLIM_ASYNC_ODOM_TIMING_ROW," in line:
            v = line.split("GLIM_ASYNC_ODOM_TIMING_ROW,", 1)[1].split(",")
            if v[-1] == "processed":
                queues.append([float(v[0]), float(v[3]), float(v[9])])
        elif "CBS_RECEIVER_LOCAL_COMPETITION_ROW,K2G," in line:
            competition.append(int(line.split("CBS_RECEIVER_LOCAL_COMPETITION_ROW,", 1)[1].split(",")[7]))
        elif "GLIM_CBS_ACTIVE_FACTOR_DIAGNOSTIC_ROW," in line:
            active_diagnostics += 1
            v = line.split("GLIM_CBS_ACTIVE_FACTOR_DIAGNOSTIC_ROW,", 1)[1].split(",")
            # Recorded Hessian invocation counts for same-edge, unary, other-local,
            # and other-external neighbors, excluding the tracked factor itself.
            diagnostic_interval_calls.append(sum(int(v[i]) for i in [24, 29, 34, 39]))
        elif "GLIM_ROS_INPUT_TIMING_ROW," in line:
            v = line.split("GLIM_ROS_INPUT_TIMING_ROW,", 1)[1].split(",")
            native_input.append(list(map(float, v)))
    result = {"log": str(path), "log_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
              "odom_timer_rows": len(odom), "active_edge_diagnostic_rows": active_diagnostics,
              "successful_neighbor_hessian_queries": int(sum(diagnostic_interval_calls)),
              "capped_incoming_diagnostic_samples": len(set(competition)),
              "input_callback_ms": {name: stats([r[i] for r in native_input])
                                    for name,i in [("preprocess",4),("backpressure_wait",5),("enqueue",6),("total",7)]},
              "input_workload_max": max(r[8] for r in native_input), "windows": []}
    for lo, hi in [(0,115),(0,20),(20,40),(40,60),(60,80),(80,100),(100,115)]:
        ids = [i for i, r in odom.items() if lo <= r["stamp"]-T0 < hi]
        rows = [odom[i] for i in ids]
        window = {"physical_seconds": [lo,hi], "n": len(ids),
                  "timestamp_hash": hashlib.sha256(json.dumps([r["stamp"] for r in rows]).encode()).hexdigest(),
                  "scan_ms": {c: stats([r[c] for r in rows]) for c in COLS[7:]}}
        details = {}
        for kind, indexes in {
            "on_smoother_update": {"sidecar":10, "injection_including_capped_audit":9, "remove":11},
            "publish_outgoing": {"local_covariance_graph_rebuild":17, "get_beliefs":18, "ros_publish":20, "total":15},
            "on_smoother_update_finish": {"odom_publish":11, "outgoing_plus_revisions":12, "total":15},
        }.items():
            data = [cbs[kind][i][0] for i in ids if i in cbs.get(kind,{})]
            details[kind] = {k: stats([a[j] for a in data]) for k,j in indexes.items()}
            if kind == "on_smoother_update_finish":
                # Includes the uninstrumented active-window diagnostics, book-
                # keeping and pose-merge finalization; not a dedicated timer.
                details[kind]["unbracketed_post_work"] = stats([
                    a[15] - a[10] - a[11] - a[12] - a[13] for a in data])
        window["nested_ms"] = details
        window["publication_age_s"] = stats([
            cbs["publish_odometry"][i][1] - odom[i]["stamp"]
            for i in ids if i in cbs.get("publish_odometry",{})])
        window["async_local_queue_before_scan"] = stats([
            r[1] for r in queues if lo <= r[0]-T0 < hi])
        result["windows"].append(window)
    return result


if __name__ == "__main__":
    result = {m: read(m) for m in ["off","on"]}
    assert result["off"]["windows"][0]["timestamp_hash"] == result["on"]["windows"][0]["timestamp_hash"]
    protected = json.loads((ROOT / "prepared/protected_manifest.json").read_text())
    result["protected_current_bridge_sources_match_run"] = {}
    for p, sha in protected.items():
        if p.endswith(("/glim_ros/cbs_bridge.cpp", "/glim_ros/cbs_bridge.hpp")):
            result["protected_current_bridge_sources_match_run"][p] = hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest() == sha
    assert all(result["protected_current_bridge_sources_match_run"].values())
    print(json.dumps(result, indent=2))
