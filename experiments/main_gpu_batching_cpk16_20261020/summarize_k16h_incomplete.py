"""Preserve the deliberately stopped diagnostic; never report a full pass."""
from pathlib import Path
import csv,json,statistics
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/main_gpu_batching_cpk16_20261020/k16h_source_1')
D=P/'paired'
assert (D/'failed.txt').exists() and not (D/'capture_complete.json').exists()
cleanup=json.loads((D/'cleanup.json').read_text());assert cleanup['all_exited']
assert not (D/'probe/failure.txt').exists()
updates=[json.loads(l) for l in (D/'probe/updates.jsonl').read_text().splitlines()]
assert all(r['passed'] for r in updates)
assert [r['update'] for r in updates]==list(range(1,len(updates)+1))
pace=list(csv.DictReader((D/'pace.csv').open()))
expected=list(csv.DictReader((P/'coverage_3/coverage.csv').open()))
assert [int(r['entered']) for r in pace]==list(range(1,len(pace)+1))
fields=('scan','seq','record_ns','header_ns','normalized_stamp','scan_end','covering_imu','native_eos')
assert [[r[k] for k in fields] for r in pace]==[[r[k] for k in fields] for r in expected[:len(pace)]]
dispatch=list(csv.DictReader((D/'dispatch.csv').open()))
plan=list(csv.DictReader((P/'coverage_3/expected_dispatch.csv').open()))
assert dispatch==plan[:len(dispatch)]
receipts=list(csv.DictReader((D/'receipts.csv').open()))
native=[r for r in receipts if r['kind']=='odom']
assert [int(r['seq']) for r in native]==list(range(len(native)))
t=[int(r['stamp_ns']) for r in native]
dt=[(b-a)/1e9 for a,b in zip(t,t[1:])]
recent=[float(r['wall_wait_s']) for r in pace[-20:]]
inputs=json.loads((P/'input_manifest.json').read_text())
result=dict(status='INCOMPLETE_STOPPED_DIAGNOSTIC_COST',full_60s_pass=False,
    stop_reason='Agent explicitly interrupted the owned controller after measured ~30s/scan implied nearly three more hours, beyond the frozen one-hour watchdog; no numeric failure.',
    completed_scan_acknowledgements=len(pace),requested_scans=598,
    input_dispatch_prefix_events=len(dispatch),exact_dispatch_prefix=True,
    native_outputs=len(native),native_span_s=(t[-1]-t[0])/1e9,
    native_first_ns=t[0],native_last_ns=t[-1],native_max_interval_s=max(dt),
    native_gaps_above_150ms=sum(x>.15 for x in dt),
    startup_gap_s=(t[0]-inputs['window_start_ns'])/1e9,
    requested_tail_missing_s=(inputs['window_end_ns']-t[-1])/1e9,
    recent20_scan_wait_mean_s=statistics.mean(recent),
    recent20_scan_wait_median_s=statistics.median(recent),
    recent20_scan_wait_max_s=max(recent),
    projected_remaining_minutes_at_recent_mean=(598-len(pace))*statistics.mean(recent)/60,
    paired_summary=updates[-1],max_active_poses=max(r['states'] for r in updates),
    cleanup=cleanup,driver_exit_explanation='92 is the diagnostic ROS-stopped exit after intentional controller interruption, not a numerical comparison failure.',
    limitation='No full validation, throughput claim, default promotion or automatic retry. Exact expensive observer substage is not established by these aggregate timings.')
with (H/'RESULT_K16H_INCOMPLETE.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
