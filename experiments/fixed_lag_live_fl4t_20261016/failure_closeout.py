"""Read-only failed-capture accounting; not a passed experiment validator."""
import csv
import hashlib
import json
import math
import subprocess
from pathlib import Path

H = Path(__file__).resolve().parent
R = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_fixed_lag_live120_off_on_fl4t_20261016/prepared')
S = R / 'live120_off_source'
C = R / 'live120_off_control'
T = R / 'live120_off_receiver'
O = T / 'receiver'
PREVIOUS = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_fixed_lag_live30_off_on_fl4s2_20261016/prepared/pilot30_off_source')

def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()

def tree(path):
    return {str(p.relative_to(path)): sha(p) for p in path.rglob('*') if p.is_file()}

def rows(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))

protected = json.loads((R / 'protected_manifest.json').read_text())
for path, digest in protected.items():
    assert sha(path) == digest, path
assert (C / 'failed.txt').exists() and not (C / 'complete.json').exists()
assert not (R / 'live120_on_control').exists()
assert not (R / 'live120_on_source').exists()
assert not (R / 'evaluation_full').exists()
for name in ('glim_config', 'kimera_profile'):
    assert tree(S / name) == tree(PREVIOUS / name), name
effective = json.loads((S / 'effective_mode_check.json').read_text())
assert effective['passed'] and effective['checked_before_playback']
assert effective['glim']['glim_cbs_receive_external_beliefs'] is False
assert '--cbs_live_revision_enable=false' in effective['kimera_argv']

with (S / 'acceptance.csv').open() as stream:
    accepted = [[float(x) for x in r] for r in csv.reader(stream)]
assert [int(r[0]) for r in accepted] == list(range(len(accepted)))
assert all(all(math.isfinite(v) for v in r) for r in accepted)
assert all(r[3] <= r[1] + 1e-10 * (1 + abs(r[1])) for r in accepted)
fallback = [r for r in accepted if r[4] == 1]
native = rows(S / 'kimera_odometry.csv')
assert len(native) == len(accepted)

first_imu = None
last_imu = None
event_counts = {}
with (T / 'events.csv').open() as stream:
    for event in csv.reader(stream):
        event_counts[event[0]] = event_counts.get(event[0], 0) + 1
        if event[0] == 'I':
            # Event time is column1 (nanoseconds) for IMU events.
            stamp = int(event[1])
            first_imu = stamp if first_imu is None else first_imu
            last_imu = stamp

def coverage(stamps):
    return dict(rows=len(stamps), first_ns=stamps[0], last_ns=stamps[-1],
                span_s=(stamps[-1]-stamps[0])*1e-9,
                start_after_first_imu_s=(stamps[0]-first_imu)*1e-9,
                end_after_first_imu_s=(stamps[-1]-first_imu)*1e-9)

with (S / 'odom_imu.txt').open() as stream:
    glim = [round(float(line.split()[0])*1e9) for line in stream
            if line.strip() and not line.startswith('#')]
kimera = [int(r['field.header.stamp']) for r in native]
online = rows(O / 'online_states.csv')
finalized = rows(O / 'finalized_states.csv')
monitor = [json.loads(line) for line in (C / 'monitor.jsonl').read_text().splitlines()]
summary = json.loads((O / 'summary.json').read_text())
transport = json.loads((T / 'transport_complete.json').read_text())
assert summary['events'] == sum(event_counts.values())
assert summary['received'] == transport['beliefs'] == event_counts['B']
assert summary['publications'] == len(online) == transport['published_poses']
assert summary['finalized'] == len(finalized)
assert summary['pending_at_end'] == summary['unsupported_end'] == 0
assert summary['received'] == summary['inserted'] + summary['replaced'] + summary['excluded']
assert summary['inserted'] == summary['active'] + summary['absorbed']
errors = []
with (S / 'estimators.log').open() as stream:
    for number, line in enumerate(stream, 1):
        if 'FL4Q' in line or 'KIMERA_OPTIMIZE_TIMING_ROW,531,' in line or 'Backend did not return an output' in line:
            errors.append(dict(line=number, text=line.rstrip()))
assert any('no decreasing current-chart LM step' in r['text'] for r in errors)
containers = subprocess.check_output(['docker', 'inspect', '--format', '{{.Name}} {{.State.Status}}',
    'cbsms_ws_gpu', 'ubuntu-jammy-humble-nvidia'], text=True).strip().splitlines()
assert containers == ['/cbsms_ws_gpu exited', '/ubuntu-jammy-humble-nvidia exited']
head = subprocess.check_output(['git', '-C', str(H.parents[1]), 'rev-parse', 'HEAD'], text=True).strip()
assert head == '67c00392cd640a432bda9b67afcb57be4face381'
result = dict(
    experiment_passed=False, closeout_verified=True, requested_seconds=120,
    off='FAILED_SOURCE_KIMERA_UPDATE_531', on='NOT_STARTED',
    controller_failure=(C / 'failed.txt').read_text().strip(), failure_log=errors,
    protected_unchanged=len(protected), source_profiles_identical_to_fl4s=True,
    effective_settings_passed_before_playback=True,
    covariance_or_model_tuning=False, automatic_retry=False,
    first_imu_ns=first_imu, last_imu_ns=last_imu,
    captured_imu_span_s=(last_imu-first_imu)*1e-9,
    coverage=dict(native_glim=coverage(glim), native_kimera=coverage(kimera),
        fused_online=coverage([int(r['timestamp_ns']) for r in online]),
        fused_fixed_lag_finalized=coverage([int(r['timestamp_ns']) for r in finalized])),
    accepted_before_failure=len(accepted), all_logged_accepted_nonincreasing=True,
    failed_update_has_acceptance_row=False,
    fallbacks_before_failure=len(fallback), fallback_updates=[int(r[0]) for r in fallback],
    cap_hits_before_failure=sum(r[5] == 10 for r in fallback),
    max_cache_fresh_error=max(r[7] for r in accepted),
    max_tree_error=max(r[8] for r in accepted), last_accepted=accepted[-1],
    receiver_partial_journal_summary=summary, transport=transport,
    last_monitor=monitor[-1],
    full_window_validation_run=False, deterministic_repeat_run=False, accuracy_scoring_run=False,
    containers_restored=containers, head=head, commit_created=False,
    note='Receiver complete means its truncated journal drained, not the requested120s experiment passed.')
with (H / 'failure_closeout.json').open('x') as stream:
    json.dump(result, stream, indent=2)
print(json.dumps(result, indent=2))
