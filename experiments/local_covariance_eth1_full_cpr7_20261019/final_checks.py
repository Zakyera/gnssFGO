"""Read-only final integrity checks, saving only the new audit result."""
from pathlib import Path
import ast,hashlib,json,subprocess
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_local_cov_full_cpr7_20261019/prepared')
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
assert json.loads((H/'results.json').read_text())['passed']
manifest=json.loads((P/'protected_manifest.json').read_text())
for p,want in manifest.items():assert sha(p)==want,p
for mode in ('off','on'):
 assert (P/f'full_{mode}_control/complete.json').exists()
 assert not (P/f'full_{mode}_control/failed.txt').exists()
for p in H.glob('*.py'):ast.parse(p.read_text())
subprocess.run(['git','-C','/home/yeranis/repos/gnssFGO','diff','--check'],check=True)
states={name:subprocess.check_output(['docker','inspect','--format','{{.State.Status}}',name],text=True).strip()
        for name in ('cbsms_ws_gpu','ubuntu-jammy-humble-nvidia')}
assert all(v=='exited' for v in states.values()),states
result=dict(passed=True,protected_unchanged=len(manifest),container_states=states,
 report_sha256=sha(H/'REPORT.txt'),results_sha256=sha(H/'results.json'),
 evidence_sha256={str(p.relative_to(H)):sha(p) for p in (H/'evidence').rglob('*') if p.is_file()},
 source_repos=json.loads((H/'results.json').read_text())['source_repos'],
 note='No estimator/model edits, default promotion, commit/push or further run.')
with (H/'final_checks.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k!='evidence_sha256'},indent=2))
