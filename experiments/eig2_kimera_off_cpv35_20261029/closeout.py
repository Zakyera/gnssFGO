"""Close out one finite diagnostic; never launches a source or optimizer."""
import ast,hashlib,json,subprocess
from datetime import datetime,timezone
from pathlib import Path
H=Path(__file__).resolve().parent;E=H.parent;R=E.parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eig2_kimera_off_cpv35_20261029')
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
v=json.loads((H/'verification.json').read_text());assert v['passed']
for p,s in json.loads((H/'protected_before.json').read_text()).items():assert sha(p)==s,p
for p,s in json.loads((P/'prepared_manifest.json').read_text()).items():assert sha(P/p)==s,p
for p in H.glob('*.py'):ast.parse(p.read_text(),filename=str(p))
owned=[json.loads((H/'start.json').read_text())['pid'],json.loads((H/'capture_process.json').read_text())['host_pid']]
for pid in owned:assert not Path('/proc/'+str(pid)).exists(),('owned PID still present',pid)
states=subprocess.check_output(['docker','inspect','--format','{{.Name}} {{.State.Running}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert states.splitlines()==['/cbsms_ws_gpu false','/ubuntu-jammy-humble-nvidia false'],states
cleanup=json.loads((P/'capture/cleanup.json').read_text())
assert cleanup['kimera_process_gone'] and not cleanup['errors'] and all(x==0 for x in cleanup['returncodes'].values())
outputs={}
for old in ['all_method1_rerun_cpv33_20261029','all_method2_rerun_cpv30_20261029']:
    c=json.loads((E/old/'closeout.json').read_text());hashes=c.get('output_sha256',c.get('outputs'))
    assert hashes
    for p,s in hashes.items():assert sha(p)==s,p
    outputs[old]=len(hashes)
subprocess.run(['git','diff','--check','--','AGENTS.md','GNSSFGO_RUNTIME_ENGINEERING_PLAN_20261011.md'],cwd=R,check=True)
receipt=dict(passed=True,completed_utc=datetime.now(timezone.utc).isoformat(),
    status='Finite OFF reproducer and mechanism diagnosis complete; exact cause/fix open; no further run launched.',
    source_runs=1,source_retries=0,source_duration_s=300,source_rate=.2,
    original_pose_and_cost_rows_exact=v['parity']['native_rows'],main_revision_rows_exact=v['main_revision_rows_exact'],
    diagnostic_snapshot_updates=len(v['snapshots']),original_protected_files_unchanged=v['protected_files_unchanged'],
    private_files_unchanged=v['private_files_unchanged'],hesai_checkpoint=v['hesai_checkpoint'].strip(),
    checkpoint_verified_by='verification.json',original_rerun_outputs_unchanged=outputs,
    host_owned_pids_exited=owned,container_states=states.splitlines(),cleanup=cleanup,
    production_or_model_changes=False,cbs_on_runs=0,new_accuracy_metrics=False,
    visual_check='off_diagnosis_v2.png inspected; original first figure retained.',
    private_library_identity=json.loads((H/'ready.json').read_text()),
    evidence_sha256={str(p):sha(p) for p in sorted(H.iterdir()) if p.is_file()},
    capture_sha256={str(p.relative_to(P)):sha(p) for p in sorted((P/'capture').rglob('*')) if p.is_file()},
    documentation_sha256={str(p):sha(p) for p in [R/'AGENTS.md',R/'GNSSFGO_RUNTIME_ENGINEERING_PLAN_20261011.md']})
with (H/'closeout.json').open('x') as f:json.dump(receipt,f,indent=2)
print(json.dumps({k:v for k,v in receipt.items() if not k.endswith('sha256')},indent=2))
