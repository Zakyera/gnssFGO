"""Seal CP102 build/synthetic qualification only; no capture or deployment."""
import datetime,hashlib,json,re,subprocess
from pathlib import Path

H=Path(__file__).resolve().parent;E=H.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');K=W/'runs'/H.name
C=W/'runs/persistent_compatibility_cpv76_20261101'
H101=E/'prior_anchor_candidate_cpv101_20261103'
K101=W/'runs'/H101.name
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def identity(pid):
    p=Path(f'/proc/{pid}/stat')
    return p.read_text().rsplit(')',1)[1].split()[19] if p.exists() else None
assert not (H/'complete.json').exists()
assert (H/'REPORT.txt').is_file()
pins={};counts={};jobs=[]
for stage in ('baseline','candidate','tests_v2'):
    d=H/stage
    assert read(d/'passed.json')['passed'] and read(d/'cleanup.json')['owned_jobs_exited']
    entries=read(d/'inputs.json');counts[stage]=len(entries)
    for p,v in entries.items():
        assert p not in pins or pins[p]==v,('conflicting pin',p)
        pins[p]=v
    for p in [d/'start.json',*sorted(d.glob('*_process.json'))]:
        r=read(p);assert identity(r['pid'])!=str(r['proc_start']),p
        jobs.append(dict(receipt=str(p),pid=r['pid'],proc_start=r['proc_start'],args=r.get('args'),exited=True))
assert not (H/'tests/start.json').exists()
assert (H/'tests/PRELAUNCH_REJECTION.txt').exists()
assert sha(K/'baseline_build/libkimera_vio.so')==sha(C/'private_build/libkimera_vio.so')
assert sha(K/'private_build/libcbs.so')==sha(C/'private_build/libcbs.so')
assert sha(K/'private_build/libkimera_vio.so')!=sha(C/'private_build/libkimera_vio.so')
for name in ('OriginalChartPrior.h','NativePriorRefresh.h'):
    assert sha(K/'candidate'/name)==sha(K101/'candidate'/name)
old={str(p.relative_to(K/'baseline')):sha(p) for p in (K/'baseline').rglob('*') if p.is_file()}
new={str(p.relative_to(K/'candidate')):sha(p) for p in (K/'candidate').rglob('*') if p.is_file()}
changed=sorted(p for p in old if old[p]!=new[p])
added=sorted(set(new)-set(old))
assert changed==['NativePriorRefresh.h','OriginalChartPrior.h','include/cbs/bpsam/bpsam.h']
assert added==['PriorFailureArchive.cpp','PriorFailureArchive.h']
for name in ('candidate/backend_compile','candidate/archive_compile','tests_v2/compatibility_compile','tests_v2/archive_integration_compile'):
    assert read(H/(name+'_receipt.json'))['exit']==0
for name in ('compatibility','baseline_compatibility'):
    text=(H/'tests_v2'/(name+'.log')).read_text()
    assert 'PASS CP76 persistent native compatibility.' in text
    assert 'persistent_injected_rollback_cases=9' in text
    assert 'persistent_replacement_cases=6' in text
    assert 'persistent_absorption_boundaries=9' in text
archive=(H/'tests_v2/archive_integration.log').read_text()
assert 'PASS CP102: actual backend DSO;' in archive
assert archive.count('CPV102 prior failure archived at ')==3
assert archive.count('CPV102 archive failed; original guard still fatal:')==2
assert 'CPV102 prior archive disabled;' in archive
archives=sorted((K/'archive_integration_results/archives').iterdir())
assert len(archives)==3 and all((p/'complete.marker').is_file() for p in archives)
maps=(K/'archive_integration_results/process_maps.txt').read_text()
assert str(K/'private_build/libkimera_vio.so').replace(str(W),'/workspace/cbs_gtsam4.3') in maps
def metrics(path):
    rows=[]
    for line in path.read_text().splitlines():
        m=re.fullmatch(r'(\w+)=([-+0-9.eE]+)',line)
        if m:rows.append((m[1],float(m[2])))
    return rows
baseline=metrics(H/'tests_v2/baseline_compatibility.log')
candidate=metrics(H/'tests_v2/compatibility.log')
assert [r[0] for r in baseline]==[r[0] for r in candidate]
metric_delta=max(abs(a[1]-b[1]) for a,b in zip(baseline,candidate))
historic={}
for name in ('prior_anchor_candidate_cpv101_20261103','prior_conversion_offline_cpv100_20261103',
             'eth1_early_native_capture_cpv98_20261103','eth1_split_imu_live_pair_cpv95_20261103_continuation1',
             'eth1_split_imu_persistent_on_cpv91_20261102_attempt1'):
    entries=read(E/name/'complete.json')['output_sha256'];historic[name]=len(entries)
    for p,v in entries.items():assert pins.get(p)==v,(name,p)
original=read(W/'runs/eth1_split_imu_repeat_cpv99_20261103/prepared/protected_manifest.json')
assert len(original)==6425
for p,v in original.items():assert pins.get(p)==v,p
for p,v in pins.items():assert sha(p)==v,p
containers=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}',
    'cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert containers=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
hesai=subprocess.check_output(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
assert 'PASS: 4 cases, 667 saved files; originals=True' in hesai
out=dict(passed=True,finished=datetime.datetime.now().astimezone().isoformat(),
         finding='Exact-anchor correction integrated in private backend; compatibility and failure recording qualified.',
         live_source_runs=0,deployed=False,actual_cp99_cause_proven=False,
         baseline_reconstructed_byte_identical=True,cbs_library_reused_byte_identical=True,
         exact_cp101_headers=True,changed_source_files=changed,added_source_files=added,
         compatibility_metric_rows=len(candidate),maximum_baseline_candidate_metric_delta=metric_delta,
         failure_archives=3,exact_matrix_anchor_readbacks=3,guard_and_rollback_preserved=True,
         prelaunch_test_rejection_preserved=True,unique_inputs_verified=len(pins),input_counts=counts,
         original_protected_files=6425,historic_terminal_files=historic,
         owned_jobs=jobs,all_owned_jobs_exited=True,containers=containers,hesai=hesai,
         library_sha256={p.name:sha(p) for p in (K/'private_build').glob('*.so')},
         output_sha256={str(p):sha(p) for root in (H,K) for p in root.rglob('*') if p.is_file()})
with (H/'complete.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps({k:v for k,v in out.items() if k not in ('owned_jobs','output_sha256')},indent=2))
