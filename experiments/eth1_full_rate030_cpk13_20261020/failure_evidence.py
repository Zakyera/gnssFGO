"""Read-only localization of the stopped CP-K13 capture; no estimator replay."""
from pathlib import Path
import csv
import hashlib
import json
from decimal import Decimal

H = Path(__file__).resolve().parent
P = Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_full_rate030_cpk13_20261020/prepared")
R = P / "full_off_receiver"
S = P / "full_off_source"
C = P / "full_off_control"

def sha(p):
    with Path(p).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()

def rows(p):
    with p.open() as f:
        return list(csv.reader(f))

assert not (H / "failure_evidence.json").exists()
assert not (C / "complete.json").exists()
assert (C / "failed.txt").read_text().strip() == "receiver stopped during playback"
assert (R / "receiver.log").read_text().strip() == "FL4A_STREAM_FAILURE revision after absorption"
with (R / "receiver/stream_progress.csv").open() as f:
    progress = list(csv.DictReader(f))
events = rows(R / "events.csv")
receipts = [json.loads(line) for line in (R / "receipts.jsonl").open()]
last = progress[-1]
index = int(last["event"])
b = events[index]
receipt = receipts[index]
assert b[0] == "B" and receipt["event"] == index
assert b[2] == "glim_hesai" and b[6:8] == ["1286", "1288"] and b[5] == "47"
marker = R / "receiver/archive/ids" / b[2] / b[3] / (b[6] + "_" + b[7])
commit = int(marker.read_text())
record = next(r for r in rows(R / f"receiver/archive/commits/{commit}/records.txt")
              if r[0] == "absorbed" and r[1:5] == b[2:4] + b[6:8])
producing_update = receipt["metadata"]["source_update"]
newest = next(e for e,r in zip(events, receipts)
              if e[0] == "B" and e[2] == b[2] and e[7] == producing_update
              and r["metadata"]["source_revision"] == 0)
source_record_ns = int(Decimal(receipt["metadata"]["source_record"]) * 1000000000)
with (R / "receiver/online_states.csv").open() as f:
    online = list(csv.DictReader(f))
with (S / "kimera_odometry.csv").open() as f:
    kimera = list(csv.DictReader(f))
glim = rows(S / "odom_imu.txt")
glim_times = [float(r[0].split()[0]) for r in glim if r]
kimera_times = [int(r["field.header.stamp"]) * 1e-9 for r in kimera]
origin = int(online[0]["timestamp_ns"])
oldest_ns = origin + int(last["oldest"]) * 100000000
query = json.loads((P / "preflight/validation.json").read_text())["bags"]["window"]
manifest = json.loads((P / "protected_manifest.json").read_text())
for p, expected in manifest.items():
    assert sha(p) == expected, p
assert not any((P / f"full_on_{part}").exists() for part in ("source", "receiver", "control"))
evidence = dict(
    capture_passed=False,
    controller_failure=(C / "failed.txt").read_text().strip(),
    receiver_failure=(R / "receiver.log").read_text().strip(),
    last_committed_event=index,
    failing_event_1based=index+1,
    failing_receipt_index=index,
    source=b[2], session=b[3], belief_edge=[int(x) for x in b[6:8]],
    revision=int(b[5]), source_update=int(producing_update),
    endpoints_ns=[int(x) for x in b[8:10]], arrival_ns=int(b[1]),
    elapsed_from_selected_start_s=(int(b[1])-query["query_start_ns"])*1e-9,
    first_endpoint_age_s=(int(b[1])-int(b[8]))*1e-9,
    producing_update_endpoint_ns=int(newest[9]),
    revision_history_age_s=(int(newest[9])-int(b[8]))*1e-9,
    producing_endpoint_to_arrival_s=(int(b[1])-int(newest[9]))*1e-9,
    producing_endpoint_to_source_record_s=(source_record_ns-int(newest[9]))*1e-9,
    source_record_to_adapter_receipt_s=(int(b[1])-source_record_ns)*1e-9,
    oldest_retained_support_index=int(last["oldest"]),
    oldest_retained_support_ns=oldest_ns,
    endpoint_before_oldest_support_s=(oldest_ns-int(b[8]))*1e-9,
    absorption_commit=commit, absorption_record=record,
    last_committed_progress=last,
    last_sampled_transport=json.loads((R / "transport_status.json").read_text()),
    partial_outputs=dict(
        native_glim=dict(rows=len(glim_times),span_s=glim_times[-1]-glim_times[0]),
        native_kimera=dict(rows=len(kimera_times),span_s=kimera_times[-1]-kimera_times[0]),
        fusion_online=dict(rows=len(online),span_s=(int(online[-1]["timestamp_ns"])-origin)*1e-9)),
    protected_hashes_unchanged=len(manifest),
    on_started=False,
    limitation="Failure localization only; no complete346.32s/accuracy comparison, no source retry or replay.",
)
with (H / "failure_evidence.json").open("x") as f:
    json.dump(evidence, f, indent=2)
print(json.dumps(evidence, indent=2))
