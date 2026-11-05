"""Read-only preservation/exit audit, seals only this new CP101 qualification."""
import datetime
import hashlib
import json
from pathlib import Path
import subprocess

H=Path(__file__).resolve().parent
E=H.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
K=W/'runs'/H.name
P99=W/'runs/eth1_split_imu_repeat_cpv99_20261103/prepared'
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def identity(pid):
    p=Path(f'/proc/{pid}/stat')
    return p.read_text().rsplit(')',1)[1].split()[19] if p.exists() else None
assert not (H/'complete.json').exists()
assert read(H/'analysis.json')['passed']
assert read(H/'qualification_v3/passed.json')['passed']
frozen=W/'runs/persistent_compatibility_cpv76_20261101/release_fixture1/source_snapshot'
original=(frozen/'OriginalChartPrior.h').read_text()
candidate=(K/'candidate/OriginalChartPrior.h').read_text()
marker='    const size_t n = H_.rows();'
assert original[original.index(marker):]==candidate[candidate.index(marker):]
refresh=(K/'candidate/NativePriorRefresh.h').read_text()
assert 'if (err>1e-8)' in refresh
assert 'const double err=std::max(original_error,converted_error);' in refresh
assert 'const double original_error=relative(original,old);' in refresh
stages=[H,*sorted(p.parent for p in H.glob('*/start.json'))]
pins={}
counts={}
jobs=[]
for d in stages:
    assert read(d/'cleanup.json')['owned_jobs_exited'],d
    entries=read(d/'inputs.json');counts[str(d)]=len(entries)
    for p,v in entries.items():
        assert p not in pins or pins[p]==v,('conflicting pin',p)
        pins[p]=v
    for p in [d/'start.json',*sorted(d.glob('*_process.json'))]:
        r=read(p)
        assert identity(r['pid'])!=str(r['proc_start']),p
        jobs.append(dict(receipt=str(p),pid=r['pid'],recorded_start=r['proc_start'],
                         recorded_args=r.get('args'),exited=True))
assert len(stages)==7
historic={}
for name in ('prior_conversion_offline_cpv100_20261103',
             'eth1_early_native_capture_cpv98_20261103',
             'eth1_split_imu_live_pair_cpv95_20261103_continuation1',
             'eth1_split_imu_persistent_on_cpv91_20261102_attempt1'):
    entries=read(E/name/'complete.json')['output_sha256']
    historic[name]=len(entries)
    for p,v in entries.items():assert pins.get(p)==v,(name,p)
original=read(P99/'protected_manifest.json');assert len(original)==6425
for p,v in original.items():assert pins.get(p)==v,p
for p,v in pins.items():assert sha(p)==v,p
containers=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}',
                                   'cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert containers=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
hesai=subprocess.check_output(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),
                               '--originals'],text=True)
assert 'PASS: 4 cases, 667 saved files; originals=True' in hesai
output={str(p):sha(p) for root in (H,K) for p in root.rglob('*') if p.is_file()}
out=dict(passed=True,finished=datetime.datetime.now().astimezone().isoformat(),
         authoritative_qualification=str(H/'qualification_v3'),
         finding='Private exact-anchor candidate removes reproduced false rejections with unchanged guard and moved-state behavior.',
         actual_cp99_cause_proven=False,production_fix_applied=False,
         live_source_runs=0,real_data_optimizer_replays=0,estimator_builds=0,
         standalone_diagnostic_compiles=len(stages),synthetic_optimizer_tests=True,
         initial_and_debug_failed_stages_preserved=6,
         input_pin_counts=counts,unique_inputs_verified=len(pins),original_protected_files=len(original),
         historic_terminal_files=historic,all_owned_jobs_exited=True,owned_jobs=jobs,
         static_checks=dict(moved_branch_byte_identical=True,guard_threshold=1e-8,original_cache_check_retained=True),
         containers=containers,hesai=hesai,output_sha256=output)
with (H/'complete.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps({k:v for k,v in out.items() if k not in ('output_sha256','owned_jobs')},indent=2))
