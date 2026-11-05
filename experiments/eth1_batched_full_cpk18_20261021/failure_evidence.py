"""Read-only CP-K18 late-revision localization; no estimator or journal replay."""
from pathlib import Path
import csv
import hashlib
import json
import collections, math, re, statistics
from decimal import Decimal

H = Path(__file__).resolve().parent
P = Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_batched_full_cpk18_20261021/prepared")
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
marker = R / "receiver/archive/ids" / b[2] / b[3] / (b[6] + "_" + b[7])
commit = int(marker.read_text())
record = next(r for r in rows(R / f"receiver/archive/commits/{commit}/records.txt")
              if r[0] == "absorbed" and r[1:5] == b[2:4] + b[6:8])
producing_update = receipt["metadata"]["source_update"]
newest = next((e for e, r in zip(events, receipts)
              if e[0] == "B" and e[2] == b[2] and e[7] == producing_update
              and r["metadata"]["source_revision"] == 0), None)
source_record = receipt["metadata"]["source_record"]
source_record_ns = int(Decimal(source_record) * 1000000000) if source_record is not None else None
with (R / "receiver/online_states.csv").open() as f:
    online = list(csv.DictReader(f))
with (S / "kimera_odometry.csv").open() as f:
    kimera = list(csv.DictReader(f))
glim_times = [float(r[0].split()[0]) for r in rows(S / "odom_imu.txt") if r]
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
    last_committed_event=index, failing_event_1based=index+1, failing_receipt_index=index,
    source=b[2], session=b[3], belief_edge=[int(x) for x in b[6:8]],
    revision=int(b[5]), source_update=int(producing_update),
    endpoints_ns=[int(x) for x in b[8:10]], arrival_ns=int(b[1]),
    elapsed_from_selected_start_s=(int(b[1])-query["query_start_ns"])*1e-9,
    first_endpoint_age_s=(int(b[1])-int(b[8]))*1e-9,
    producing_update_endpoint_ns=int(newest[9]) if newest is not None else None,
    revision_history_age_s=(int(newest[9])-int(b[8]))*1e-9 if newest is not None else None,
    producing_endpoint_to_arrival_s=(int(b[1])-int(newest[9]))*1e-9 if newest is not None else None,
    producing_endpoint_to_source_record_s=(source_record_ns-int(newest[9]))*1e-9
        if source_record_ns is not None and newest is not None else None,
    source_record_to_adapter_receipt_s=(int(b[1])-source_record_ns)*1e-9
        if source_record_ns is not None else None,
    timing_decomposition_limit="This Kimera receipt has no source_record timestamp; producing-update revision0 was not captured. Do not attribute elapsed time to CPU or transport from this receipt.",
    oldest_retained_support_index=int(last["oldest"]),
    oldest_retained_support_ns=oldest_ns,
    endpoint_before_oldest_support_s=(oldest_ns-int(b[8]))*1e-9,
    absorption_commit=commit, absorption_record=record,
    last_committed_progress=last,
    last_sampled_transport=json.loads((R / "transport_status.json").read_text()),
    partial_outputs=dict(
        native_glim=dict(rows=len(glim_times), span_s=glim_times[-1]-glim_times[0]),
        native_kimera=dict(rows=len(kimera_times), span_s=kimera_times[-1]-kimera_times[0]),
        fusion_online=dict(rows=len(online), span_s=(int(online[-1]["timestamp_ns"])-origin)*1e-9)),
    protected_hashes_unchanged=len(manifest), on_started=False,
    limitation="Failure localization only; not a complete346.32s result, accuracy comparison or source retry.",
)
stamps={int(r['field.header.seq']):int(r['field.header.stamp']) for r in kimera}
produced=stamps[int(producing_update)]
evidence.update(producing_update_endpoint_ns=produced,
    revision_history_age_s=(produced-int(b[8]))/1e9,
    producing_endpoint_to_arrival_s=(int(b[1])-produced)/1e9,
    preceding_keyframe_gap_s=(produced-stamps[int(producing_update)-1])/1e9,
    timing_decomposition_limit='Producing epoch matched by source update to native Kimera header.seq; source_record unavailable. Physical history age is distinct from arrival freshness, which combines processing/transport.')
def fields(line,marker):
    return re.sub(r'\x1b\[[0-9;]*m','',line).split(marker,1)[1].strip().split(',')
def stat(a):
    a=sorted(a)
    return dict(n=len(a),median=statistics.median(a),p95=a[math.ceil(.95*len(a))-1],max=a[-1])
logs=list(S.glob('ros_logs/*/glim_rosnode-*-stdout.log'));assert len(logs)==1
scans=[];queues=[]
for line in logs[0].open():
    if 'GLIM_ODOM_IMU_TIMING_ROW,' in line:
        f=fields(line,'GLIM_ODOM_IMU_TIMING_ROW,');assert len(f)==25
        r=list(map(float,f[:24]))
        if r[0]*1e9<=int(b[1]):scans.append(r)
    if 'GLIM_ASYNC_ODOM_TIMING_ROW,' in line:
        f=fields(line,'GLIM_ASYNC_ODOM_TIMING_ROW,')
        if f[-1]=='processed':queues.append(int(f[9]))
cohorts={}
for label,begin in [('prefix_after5s',query['query_start_ns']/1e9+5),
                    ('last20s_before_failure',int(b[1])/1e9-20)]:
    selected=[r for r in scans if r[0]>=begin]
    cohorts[label]={k:stat(r[i] for r in selected) for k,i in [('main_ms',17),('scan_ms',23)]}
    cohorts[label]['scans_over500ms']=sum(r[23]>500 for r in selected)
work=collections.Counter();hits=0;aborted=0
for line in (S/'e1b_profile.jsonl').open():
    r=json.loads(line)
    if r['context']!=1 or r['timestamp']*1e9>int(b[1]):continue
    hits+=r['metrics'][7];aborted+=bool(r['aborted'])
    for k,v in r['owner_wall'].items():work[k]+=v[0]
evidence['glim_prefix_timing']=cohorts
evidence['glim_prefix_main_work']=dict(prepared_hits=hits,sync_calls=work['gpu_sync_linearize'],
    linearize_calls=work['gpu_linearize'],aborted=aborted,internal_queue_max=max(queues))
for line in (S/'estimators.log').open():
    if f'KIMERA_BACKEND_SPINONCE_TIMING_ROW,{producing_update},' in line:
        evidence['failing_kimera_update_backend_ms']=float(fields(line,'KIMERA_BACKEND_SPINONCE_TIMING_ROW,')[1])
with (H / "failure_evidence.json").open("x") as f:
    json.dump(evidence, f, indent=2)
print(json.dumps(evidence, indent=2))
