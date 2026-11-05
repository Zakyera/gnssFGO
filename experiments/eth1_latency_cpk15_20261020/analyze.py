"""CP-K15: read saved logs only. No solver, ROS, Docker or launch invocation."""
import collections
import csv
import datetime
import hashlib
import json
import re
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parent
WS = Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3")
START = 1727775007855396309
ANSI = re.compile(r"\x1b\[[0-9;]*m")
ROS = re.compile(r"\[INFO\] \[(\d+\.\d+), (\d+\.\d+)\]")
GLOG = re.compile(r"[IWEF]1020 (\d\d:\d\d:\d\d\.\d+) ")
inputs = {}


def tracked(path):
    path = Path(path)
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    inputs[str(path)] = {"size": path.stat().st_size, "sha256": h.hexdigest()}
    return path


def stats(a):
    a = np.asarray(list(a), dtype=float)
    if len(a) == 0:
        return {"n": 0}
    return dict(n=len(a), mean=float(a.mean()), median=float(np.median(a)),
                p95=float(np.percentile(a, 95)), max=float(a.max()), sum=float(a.sum()))


def fields(line, marker):
    return ANSI.sub("", line.split(marker, 1)[1]).strip().split(",")


def wall(line):
    m = GLOG.search(line)
    if m:
        return datetime.datetime.fromisoformat("2026-10-20T" + m[1] + "+00:00").timestamp()
    m = ROS.search(ANSI.sub("", line))
    return float(m[1]) if m else None


def analyze(label, rate):
    p = WS / f"runs/gnssfgo_eth1_full_rate{int(rate*100):03d}_{label}_20261020/prepared"
    s, r = p / "full_off_source", p / "full_off_receiver"
    glog = next((s / "ros_logs").glob("*/glim_rosnode-1-stdout.log"))
    stamps = {}
    for name in ("glim", "kimera"):
        with tracked(s / f"{name}_odometry.csv").open() as f:
            rows = list(csv.DictReader(f))
        stamps[name] = {int(x["field.header.seq"]): int(x["field.header.stamp"]) for x in rows}
    receipts = [json.loads(x) for x in tracked(r / "receipts.jsonl").open()]
    # Match source wall export to the wire, without conflating wire and source revision IDs.
    wanted = {}
    ages = {"glim": [], "kimera": []}
    with tracked(r / "events.csv").open() as f:
        for row, receipt in zip(csv.reader(f), receipts):
            if row[0] != "B":
                continue
            md = receipt["metadata"]
            name = "glim" if md["logger_source"] == "g" else "kimera"
            # GLIM revision0's legacy source_update column is a seconds stamp,
            # not a pose index; this audit's GLIM age cohort is revisions > 0.
            if name == "glim" and int(md["source_revision"]) == 0:
                continue
            up = int(md["source_update"])
            ti, tj = int(row[8]), int(row[9])
            produced = stamps[name].get(up)
            key = (name, up, int(row[6]), int(row[7]), int(md["source_revision"]))
            wanted[key] = (receipt["wall_ns"], receipt["receipt_ros_ns"])
            if produced is not None:
                ages[name].append({
                    "update": up, "event": receipt["event"], "from": int(row[6]), "to": int(row[7]),
                    "revision": int(md["source_revision"]),
                    "history_s": (produced - ti) / 1e9,
                    "freshness_s": (receipt["receipt_ros_ns"] - produced) / 1e9,
                    "total_s": (receipt["receipt_ros_ns"] - ti) / 1e9,
                })
    scans, finishes, queues, async_rows = [], [], [], []
    export_delivery = {"glim": [], "kimera": []}
    source_export = {}
    warning_count = 0
    for line in tracked(glog).open(errors="replace"):
        if "GLIM_ODOM_IMU_TIMING_ROW," in line:
            f = fields(line, "GLIM_ODOM_IMU_TIMING_ROW,")
            assert len(f) == 25
            scans.append([float(x) for x in f[:24]])
        if "GLIM_ASYNC_ODOM_TIMING_ROW," in line:
            f = fields(line, "GLIM_ASYNC_ODOM_TIMING_ROW,")
            if f[-1] == "processed":
                queues.append(int(f[9]))
                async_rows.append([float(x) for x in f[:-1]])
        if "GLIM_CBS_TIMING_ROW,on_smoother_update_finish," in line:
            f = fields(line, "GLIM_CBS_TIMING_ROW,on_smoother_update_finish,")
            finishes.append([float(x) for x in f])
        if "GLIM_CBS_REVISION_BELIEF_ROW," in line:
            f = fields(line, "GLIM_CBS_REVISION_BELIEF_ROW,")
            k = ("glim", int(f[5]), int(f[1]), int(f[2]), int(f[3]))
            if k in wanted:
                w = wall(line)
                if w is not None:
                    export_delivery["glim"].append((wanted[k][0] / 1e9 - w) * 1000)
                    source_export[k] = {"wall": w, "record_ros": float(f[4])}
    kt = {}
    kexport = {}
    for line in tracked(s / "estimators.log").open(errors="replace"):
        if "performing linearization in sync mode" in line:
            warning_count += 1
        if "KIMERA_BACKEND_SPINONCE_TIMING_ROW," in line:
            f = fields(line, "KIMERA_BACKEND_SPINONCE_TIMING_ROW,")
            kt.setdefault(int(f[0]), {}).update(backend=float(f[1]), wall=wall(line))
        if "KIMERA_OPTIMIZE_TIMING_ROW," in line:
            f = fields(line, "KIMERA_OPTIMIZE_TIMING_ROW,")
            kt.setdefault(int(f[0]), {}).update(optimize=float(f[1]), preparation=float(f[2]),
                smoother=float(f[5]), outgoing=float(f[9]), marginal_graph=float(f[10]),
                first_beliefs=float(f[11]), graph_factors=int(f[15]))
        if "KIMERA_CBS_REVISION_EDGE_ROW," in line:
            f = fields(line, "KIMERA_CBS_REVISION_EDGE_ROW,")
            k = ("kimera", int(f[0]), int(f[2]), int(f[3]), int(f[6]))
            if k in wanted:
                w = wall(line)
                if w is not None:
                    export_delivery["kimera"].append((wanted[k][0] / 1e9 - w) * 1000)
                    kexport[k] = w
    contexts = collections.defaultdict(list)
    for line in tracked(s / "e1b_profile.jsonl").open():
        x = json.loads(line)
        contexts[x["context"]].append(x)
    profile = {}
    for ctx, rows in contexts.items():
        profile[str(ctx)] = {"n": len(rows)}
        for group in ("owner_wall", "worker_sum"):
            stages = sorted({k for x in rows for k in x[group]})
            profile[str(ctx)][group] = {
                k: {"inclusive_ms": stats(x[group].get(k, [0, 0, 0, 0])[1] / 1e6 for x in rows),
                    "exclusive_ms": stats(x[group].get(k, [0, 0, 0, 0])[2] / 1e6 for x in rows),
                    "calls": stats(x[group].get(k, [0, 0, 0, 0])[0] for x in rows)}
                for k in stages}
    with tracked(r / "receiver/stream_progress.csv").open() as f:
        progress = list(csv.DictReader(f))
    services, waits, support_cycles, all_cycle_work = [], [], [], []
    last_pub = 0
    cycle_work = 0
    for x in progress:
        event = int(x["event"]) - 1
        service = float(x["service_seconds"]) * 1000
        services.append(service)
        cycle_work += service
        waits.append((int(x["processed_wall_ns"]) - receipts[event]["wall_ns"]) / 1e6 - service)
        if int(x["publications"]) > last_pub:
            support_cycles.append(service)
            all_cycle_work.append(cycle_work)
            cycle_work = 0
        last_pub = int(x["publications"])
    bins = {}
    for begin in range(0, 330, 30):
        select = [x for x in scans if max(5, begin) <= x[0] - START / 1e9 < begin + 30]
        if not select:
            continue
        finish = [x for x in finishes if
                  max(5, begin) <= (stamps["glim"].get(int(x[0]), 0) - START) / 1e9 < begin + 30]
        kr = [x for i, x in kt.items() if
              max(5, begin) <= (stamps["kimera"].get(i, 0) - START) / 1e9 < begin + 30]
        bins[str(begin)] = {
            "glim_ms": {k: stats(x[j] for x in select) for k, j in
                       [("total", 23), ("smoother", 17), ("post_callback", 18),
                        ("pre_callback", 16), ("marginalization", 19)]},
            "glim_outgoing_and_revisions_ms": stats(x[12] for x in finish),
            "glim_fraction_over_input_budget": sum(x[23] > 100/rate for x in select) / len(select),
            "kimera_ms": {k: stats(x[k] for x in kr if k in x) for k in
                          ("backend", "smoother", "outgoing", "preparation")},
        }
    # The transport can journal a few additional queued events after the receiver
    # stops. The first uncommitted event, not the last journal row, is the failure.
    failed_index = int(progress[-1]["event"])
    late = next(v for a in ages.values() for v in a if v["event"] == failed_index)
    name = "kimera" if label == "cpk14" else "glim"
    key = (name, late["update"], late["from"], late["to"], late["revision"])
    fail = {"event": late, "backend_timers": kt.get(late["update"]) if name == "kimera" else None,
            "source_export": kexport.get(key) if name == "kimera" else source_export.get(key),
            "receipt_wall": wanted[key][0] / 1e9,
            "prior_backend": kt.get(late["update"] - 1) if name == "kimera" else None}
    if name == "glim":
        fail["producing_scan"] = next((x for x in scans if int(x[1]) == late["update"]), None)
        fail["producing_finish"] = next((x for x in finishes if int(x[0]) == late["update"]), None)
        if fail["producing_finish"]:
            fail["producing_finish"] = [x if np.isfinite(x) else None for x in fail["producing_finish"]]
    gaps = [{"update": i, "gap_s": (t-stamps["kimera"][i-1])/1e9,
             "since_start_s": (t-START)/1e9} for i, t in stamps["kimera"].items() if i-1 in stamps["kimera"]]
    result = {
        "root": str(p), "rate": rate, "bins_30s": bins, "failed_event_detail": fail,
        "source_export_to_adapter_wall_ms": {k: stats(v) for k, v in export_delivery.items()},
        "revision_age": {name: {k: stats(x[k] for x in a) for k in
                              ("history_s", "freshness_s", "total_s")} for name, a in ages.items()},
        "revision_examples": {name: sorted(a, key=lambda x:x["total_s"], reverse=True)[:5]
                              for name, a in ages.items()},
        "kimera_gaps": stats(x["gap_s"] for x in gaps),
        "kimera_largest_gaps": sorted(gaps, key=lambda x:x["gap_s"], reverse=True)[:10],
        "kimera_last_updates": {str(i): kt[i] for i in sorted(kt)[-8:]},
        "glim_processed_queue": stats(queues),
        "glim_active_frames": stats(x[5] for x in scans),
        "glim_sync_warning_lines_estimators": warning_count,
        "profile": profile,
        "receiver": {"all_event_service_ms": stats(services), "support_cycle_service_ms": stats(support_cycles),
                     "all_event_work_between_publications_ms": stats(all_cycle_work),
                     "adapter_to_service_start_ms": stats(waits),
                     "last_progress": progress[-1],
                     "transport_status": json.loads(tracked(r/"transport_status.json").read_text())},
    }
    return result


if __name__ == "__main__":
    target = OUT / "evidence_v2.json"
    if target.exists():
        raise SystemExit("Refusing to overwrite evidence_v2.json")
    result = {case: analyze(case, rate) for case, rate in [("cpk13", .3), ("cpk14", .2)]}
    result["inputs"] = inputs
    result["scope"] = "Read-only saved log analysis; no replay, estimator, solver or ROS execution."
    with target.open("x") as f:
        json.dump(result, f, indent=2, allow_nan=False)
    print(target)
