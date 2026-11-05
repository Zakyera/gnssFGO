"""Read-only integrity audit followed by one new closeout receipt."""
import ast,hashlib,json,subprocess
from datetime import datetime,timezone
from pathlib import Path
H=Path(__file__).resolve().parent;E=H.parent;R=E.parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eig2_counterfactual_cpv36_20261029')
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
v=json.loads((H/'capture_verification.json').read_text());assert v['passed']
results=json.loads((H/'diagnostic_results_v2.json').read_text())
assert len(results['checkpoints'])==7
finished=json.loads((H/'offline_finished.json').read_text());assert len(finished['results'])==7
for p,s in json.loads((H/'protected_before.json').read_text()).items():assert sha(p)==s,p
for p,s in json.loads((P/'prepared_manifest.json').read_text()).items():assert sha(P/p)==s,p
for p in H.glob('*.py'):ast.parse(p.read_text(),filename=str(p))
owned=[json.loads((H/'start.json').read_text())['pid'],json.loads((H/'capture_process.json').read_text())['host_pid'],json.loads((H/'offline_start.json').read_text())['pid']]
owned.extend(json.loads(p.read_text())['pid'] for p in H.glob('offline_process_*.json'))
owned.append(json.loads((H/'inspection_start.json').read_text())['pid'])
assert json.loads((H/'inspection_finished.json').read_text())['passed']
assert len(json.loads((H/'equation_audit.json').read_text()))==7
for pid in owned:assert not Path('/proc/'+str(pid)).exists(),('owned PID still present',pid)
states=subprocess.check_output(['docker','inspect','--format','{{.Name}} {{.State.Running}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert states.splitlines()==['/cbsms_ws_gpu false','/ubuntu-jammy-humble-nvidia false'],states
cleanup=json.loads((P/'capture/cleanup.json').read_text())
assert cleanup['kimera_process_gone'] and not cleanup['errors'] and all(x==0 for x in cleanup['returncodes'].values())
outputs={}
for old in ('all_method1_rerun_cpv33_20261029','all_method2_rerun_cpv30_20261029'):
    c=json.loads((E/old/'closeout.json').read_text());hashes=c.get('output_sha256',c.get('outputs'));assert hashes
    for p,s in hashes.items():assert sha(p)==s,p
    outputs[old]=len(hashes)
checkpoint=subprocess.check_output(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
subprocess.run(['git','diff','--check','--','AGENTS.md','GNSSFGO_RUNTIME_ENGINEERING_PLAN_20261011.md'],cwd=R,check=True)
receipt=dict(passed=True,completed_utc=datetime.now(timezone.utc).isoformat(),
    status='Finite OFF checkpoint comparison finished; inspect report for scientific qualifications. No production fix.',
    source_runs=1,source_retries=0,source_duration_s=300,source_rate=.2,
    original_pose_and_cost_rows_exact=v['parity']['native_rows'],main_revision_rows_exact=v['main_revision_rows_exact'],
    diagnostic_checkpoints=7,offline_receipts=finished,
    original_protected_files_unchanged=v['protected_files_unchanged'],private_files_unchanged=v['private_files_unchanged'],
    hesai_checkpoint=checkpoint.strip(),original_rerun_outputs_unchanged=outputs,
    host_owned_pids_exited=owned,container_states=states.splitlines(),cleanup=cleanup,
    production_changes=False,cbs_on_runs=0,new_accuracy_metrics=False,
    private_library_identity=json.loads((H/'ready.json').read_text()),
    evidence_sha256={str(p):sha(p) for p in sorted(H.iterdir()) if p.is_file()},
    capture_sha256={str(p.relative_to(P)):sha(p) for p in sorted((P/'capture').rglob('*')) if p.is_file()},
    offline_sha256={str(p.relative_to(P)):sha(p) for p in sorted((P/'offline').rglob('*')) if p.is_file()},
    inspection_sha256={str(p.relative_to(P)):sha(p) for p in sorted((P/'inspection').rglob('*')) if p.is_file()},
    documentation_sha256={str(p):sha(p) for p in [R/'AGENTS.md',R/'GNSSFGO_RUNTIME_ENGINEERING_PLAN_20261011.md']})
with (H/'closeout.json').open('x') as f:json.dump(receipt,f,indent=2)
print(json.dumps({k:v for k,v in receipt.items() if not k.endswith('sha256')},indent=2))
