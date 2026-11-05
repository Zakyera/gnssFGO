"""Read-only retained-data speed analysis; writes only this experiment's summary."""
from pathlib import Path
import csv
import hashlib
import json
import math
import re
import statistics

H = Path(__file__).resolve().parent
W = Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3")
P = W / "runs/gnssfgo_eth1_speed60_cpk12_20261020/prepared"
T = W / "runs/gnssfgo_eth1_prior_live120_cpk11_20261020/prepared"
V = Path("/home/yeranis/results/gnssfgo_eth1_speed60_cpk12_20261020/on")
S = P / "speed60_on_source"
R = P / "speed60_on_receiver"

def load(p):
    return json.loads(p.read_text())

def sha(p):
    with Path(p).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()

def stat(values):
    a = sorted(values)
    assert a
    return dict(n=len(a), mean=statistics.fmean(a), median=statistics.median(a),
                p95=a[math.ceil(.95 * len(a)) - 1], maximum=a[-1])

def fields(line, marker):
    return re.sub(r"\x1b\[[0-9;]*m", "", line).split(marker, 1)[1].strip().split(",")

checks = dict(receiver=load(V / "validation.json"), repeat=load(V / "repeat_validation.json"),
              local=load(S / "local_provenance.json"), candidate=load(S / "candidate_validation.json"))
assert all(x["passed"] for x in checks.values())
assert not (P / "speed60_on_control/failed.txt").exists()
capture = load(P / "speed60_on_control/complete.json")
preflight = load(P / "preflight/validation.json")
end = preflight["bags"]["window"]["query_end_ns"] * 1e-9
start = preflight["bags"]["window"]["query_start_ns"] * 1e-9
glim_last = max(float(line.split()[0]) for line in (S / "odom_imu.txt").open() if line.strip())
manifest = load(P / "protected_manifest.json")
for p, expected in manifest.items():
    assert sha(p) == expected, p

def timing(source, prefix_only):
    logs = list((source / "ros_logs").glob("*/glim_rosnode-*-stdout.log"))
    assert len(logs) == 1
    scans, queues, all_finish = [], [], []
    for line in logs[0].open():
        marker = "GLIM_ODOM_IMU_TIMING_ROW,"
        if marker in line:
            f = fields(line, marker)
            assert len(f) == 25
            if start + 5 <= float(f[0]) <= glim_last:
                scans.append([float(f[i]) for i in (0, 23, 17, 18)])
        marker = "GLIM_ASYNC_ODOM_TIMING_ROW,"
        if marker in line:
            f = fields(line, marker)
            if f[-1] == "processed" and float(f[2]) < end:
                queues.append(int(f[9]))
        marker = "GLIM_CBS_TIMING_ROW,on_smoother_update_finish,"
        if marker in line:
            f = fields(line, marker)
            all_finish.append(float(f[-1]))
    # Native keyframe timestamps identify the identical physical section.
    with (source / "kimera_odometry.csv").open() as f:
        poses = list(csv.DictReader(f))
    ids = {i for i, r in enumerate(poses)
           if start + 5 <= int(r["field.header.stamp"]) * 1e-9 < end}
    kimera = dict(backend=[], optimize_total=[], smoother_update=[], outgoing=[])
    for line in (source / "estimators.log").open():
        if "KIMERA_BACKEND_SPINONCE_TIMING_ROW," in line:
            f = fields(line, "KIMERA_BACKEND_SPINONCE_TIMING_ROW,")
            if int(f[0]) in ids:
                kimera["backend"].append(float(f[1]))
        if "KIMERA_OPTIMIZE_TIMING_ROW," in line:
            f = fields(line, "KIMERA_OPTIMIZE_TIMING_ROW,")
            if int(f[0]) in ids:
                for k, i in (("optimize_total", 1), ("smoother_update", 5), ("outgoing", 9)):
                    kimera[k].append(float(f[i]))
    return dict(
        glim_scan_ms=stat(r[1] for r in scans),
        glim_smoother_ms=stat(r[2] for r in scans),
        glim_post_callback_ms=stat(r[3] for r in scans),
        glim_scan_by15s_ms={str(b):stat(r[1] for r in scans if start+b <= r[0] < start+b+15)
                           for b in (0, 15, 30, 45)},
        glim_internal_queue_max=max(queues),
        glim_fraction_exceeding_333ms=sum(r[1] > 100/.30 for r in scans)/len(scans),
        kimera_ms={k:stat(v) for k,v in kimera.items()},
        section="same first60s, after5s initialization; source wall timers",
        prefix_of120s_reference=prefix_only)

timings = dict(rate030=timing(S, False),
               rate010=timing(T / "live120_on_source", True))
with (R / "events.csv").open() as f:
    events = list(csv.reader(f))
receipts = [json.loads(line) for line in (R / "receipts.jsonl").open()]
assert len(events) == len(receipts)
ends = {int(e[7]):int(e[9]) for e,r in zip(events, receipts)
        if e[0] == "B" and e[2] == "glim_hesai" and r["metadata"]["source_revision"] == 0}
ages, producing_lags = [], []
for e, r in zip(events, receipts):
    if e[0] != "B" or e[2] != "glim_hesai" or not r["metadata"]["source_revision"]:
        continue
    u = int(r["metadata"]["source_update"])
    assert u in ends
    ages.append((int(e[1]) - int(e[8])) * 1e-9)
    producing_lags.append((int(e[1]) - ends[u]) * 1e-9)
assert max(ages) < 7

timestamp_checks = {}
for role, filename in (("glim", "odom_imu.txt"), ("kimera", "kimera_odometry.csv")):
    def stamps(source):
        if role == "glim":
            return [line.split()[0] for line in (source / filename).open()
                    if line.strip() and float(line.split()[0]) < end]
        with (source / filename).open() as f:
            return [r["field.header.stamp"] for r in csv.DictReader(f)
                    if int(r["field.header.stamp"]) * 1e-9 < end]
    actual, ref = stamps(S), stamps(T / "live120_on_source")
    # A scan stamped just before the cutoff can require IMU support AFTER it.
    # The longer baseline can finish that scan; the bounded capture cannot.
    # Compare all actual stamps against the exact saved reference prefix,
    # and disclose any baseline-only tail rather than invent a missing pose.
    timestamp_checks[role] = dict(actual_count=len(actual), reference_count_before_cutoff=len(ref),
                                  identical_timestamp_prefix=actual == ref[:len(actual)],
                                  baseline_only_tail_stamps=ref[len(actual):])
    assert actual == ref[:len(actual)], (role, "changed source timestamp coverage")
    assert len(ref) - len(actual) <= (1 if role == "glim" else 0)

result = dict(passed=True, capture=capture, playback_rate=.30, requested_s=60,
              protected_unchanged=len(manifest), checks=checks, timings=timings,
              source_timestamps_vs_010_prefix=timestamp_checks,
              glim_revision_first_endpoint_age_s=stat(ages),
              glim_revision_producing_update_lag_s=stat(producing_lags),
              minimum_glim_revision_lag_margin_s=7-max(ages),
              limits="One60sON test only. No full-sequence/1x/stationarity certification;"
                     " faster playback changes causal availability, not bitwise source-run equivalence.")
with (H / "results.json").open("x") as f:
    json.dump(result, f, indent=2)
print(json.dumps({k:v for k,v in result.items() if k != "checks"}, indent=2))
