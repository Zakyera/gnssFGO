"""Read-only analysis of stopped capture; writes one distinct failure receipt."""
from pathlib import Path
import csv
import json
import statistics

H = Path(__file__).resolve().parent
P = Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/main_gpu_batching_cpk16_20261020/k16g_source_1")
D = P / "paired"
assert (D / "failed.txt").exists() and not (D / "capture_complete.json").exists()
cleanup = json.loads((D / "cleanup.json").read_text())
assert cleanup["all_exited"]
rows = [json.loads(s) for s in (D / "probe/updates.jsonl").read_text().splitlines()]
assert [r["update"] for r in rows] == list(range(1, len(rows) + 1))
assert all(r["passed"] for r in rows)
receipts = list(csv.DictReader((D / "receipts.csv").open()))
inputs = json.loads((P / "input_manifest.json").read_text())
counts = {}
for kind, topic in (("lidar", "/boxi/hesai/points"), ("imu", "/boxi/cpt7/imu")):
    received = [(int(r["seq"]), int(r["stamp_ns"])) for r in receipts if r["kind"] == kind]
    expected = [tuple(r[:2]) for r in inputs["topics"][topic]["messages"]]
    counts[kind] = dict(expected=len(expected), received=len(received),
                        observer_identical_order=(received == expected))
native = [r for r in receipts if r["kind"] == "odom"]
times = [int(r["stamp_ns"]) for r in native]
profile = [json.loads(s) for s in (D / "e1b_profile.jsonl").read_text().splitlines()]
main = [r["owner_wall"]["root"][1] / 1e6 for r in profile
        if r["context"] == 1 and "root" in r["owner_wall"]]
input_rows = []
for line in (D / "node.log").open():
    if "GLIM_ROS_INPUT_TIMING_ROW," in line:
        input_rows.append(line.split("GLIM_ROS_INPUT_TIMING_ROW,", 1)[1].strip().split(","))
result = dict(
    passed=False, reason="instrumented source required TERM after45s graceful shutdown timeout",
    requested_window_s=60, playback_rate=.20, cleanup=cleanup,
    no_numerical_mismatch_observed=not (D / "probe/failure.txt").exists(),
    completed_comparisons=rows[-1],
    compared_physical_span_s=rows[-1]["stamp"]-rows[0]["stamp"],
    maximum_active_poses=max(r["states"] for r in rows),
    sensor_observer=counts,
    native_count=len(native), native_span_s=(times[-1]-times[0])/1e9,
    native_first_stamp_ns=times[0], native_last_stamp_ns=times[-1],
    native_startup_gap_s=(times[0]-inputs["window_start_ns"])/1e9,
    native_tail_gap_s=(inputs["window_end_ns"]-times[-1])/1e9,
    native_gaps_over_150ms=sum(b-a > 150_000_000 for a,b in zip(times,times[1:])),
    instrumented_main_root_ms=dict(
        count=len(main), first20_median=statistics.median(main[:20]),
        last20_median=statistics.median(main[-20:]), maximum=max(main)),
    input_timing_rows=len(input_rows),
    limitations=[
        "Sensor receipt conservation is in the separate observer, not proof all scans reached GLIM.",
        "Instrumented MAIN timing includes duplicate solving and exhaustive comparisons.",
        "Nested in-memory profiler sessions do not retain a complete diagnostic substage breakdown.",
        "Missing frames and unfinished drain invalidate full60s acceptance.",
        "No estimator performance regression or arithmetic failure is established by this overloaded diagnostic."])
if input_rows:
    result["input_callback_wait_ms"] = dict(
        maximum=max(float(r[5]) for r in input_rows),
        last20_median=statistics.median(float(r[5]) for r in input_rows[-20:]),
        maximum_workload=max(int(r[8]) for r in input_rows))
with (H / "RESULT_K16G_FAILURE.json").open("x") as f:
    json.dump(result, f, indent=2)
print(json.dumps(result, indent=2))
