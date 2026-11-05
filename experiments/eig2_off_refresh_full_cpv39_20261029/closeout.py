"""Close out completed source + saved analysis; no source/solver launch."""
import datetime,hashlib,json,shutil,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent;R=H.parents[1]
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');P=W/'runs/eig2_off_refresh_full_cpv39_20261029'
K=W/'runs/kimera_off_incremental_refresh_cpv38_20261029';C=P/'capture'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(p.read_text())
assert read(H/'results.json')['passed'] and (H/'REPORT.txt').exists()
assert read(H/'capture_finished.json')['returncode']==0
assert not (C/'failed.txt').exists() and not (H/'controller_failed.json').exists()
pins=read(H/'protected_before.json')
for p,s in pins.items():assert sha(p)==s,p
manifest=read(P/'prepared_manifest.json')
for p,s in manifest.items():assert sha(P/p)==s,p
runtime=read(P/'runtime_manifest.json')
for p,s in runtime.items():assert sha(K/p)==s,p
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).splitlines()
assert all(x.endswith(' exited') for x in states),states
jobs=[read(H/'start.json')['pid'],read(H/'capture_process.json')['host_pid']]
for pid in jobs:assert not Path('/proc',str(pid)).exists(),pid
cleanup=read(C/'cleanup.json');assert not cleanup['errors'] and cleanup['kimera_process_gone']
assert all(x is not None for x in cleanup['returncodes'].values())
hesai=subprocess.check_output(['python3',str(R/'experiments/checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True).strip()
assert shutil.disk_usage(H).free>5*1024**3
for repo in (R,W/'src/cbsms',W/'src/cbs',W/'src/Kimera-VIO'):
    subprocess.run(['git','-C',str(repo),'diff','--check'],check=True)
assert not (H/'closeout.json').exists()
result=dict(passed=True,completed_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),source_runs=1,source_retries=0,source_duration_s=355.02,source_rate=.20,CBS_ON_runs=0,GLIM_or_fusion_runs=0,production_changes=False,protected_files_unchanged=len(pins),prepared_files_unchanged=len(manifest),runtime_manifest_unchanged=runtime,container_states=states,owned_host_jobs_exited=jobs,cleanup=cleanup,hesai_checkpoint=hesai,free_bytes=shutil.disk_usage(H).free,evidence_sha256={str(p):sha(p) for root in (H,C) for p in root.rglob('*') if p.is_file()})
with (H/'closeout.json').open('x') as f:json.dump(result,f,indent=2)
print('PASS CP-V39 closeout:',len(pins),'protected files;',hesai)
