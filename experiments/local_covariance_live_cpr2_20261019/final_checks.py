"""Preserve compact evidence and verify the completed gate without rerunning it."""
from pathlib import Path
import ast,hashlib,json,shutil,subprocess
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs/gnssfgo_eig2_local_cov30_cpr2_20261019/prepared'
S=P/'live30_off_source';R=P/'live30_off_receiver'
RESULT=Path('/home/yeranis/results/gnssfgo_eig2_local_cov30_cpr2_20261019/off')
def sha(path):
 with Path(path).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
manifest=json.loads((P/'protected_manifest.json').read_text())
for path,h in manifest.items():assert sha(path)==h,path
evidence=H/'evidence';evidence.mkdir(exist_ok=False)
paths=[P/'protected_manifest.json',P/'live30_off_control/complete.json',
 S/'local_provenance.json',S/'candidate_validation.json',S/'source_coverage.json',
 S/'effective_mode_check.json',S/'run_metadata.txt',R/'transport_complete.json',
 R/'resolved_config.json',RESULT/'validation.json',RESULT/'repeat_validation.json']
for path in paths:shutil.copy2(path,evidence/path.name)
repo_state={}
for name,path in [('glim_ros1',W/'src/glim_ros1'),('Kimera-VIO',W/'src/Kimera-VIO'),('cbs',W/'src/cbs')]:
 status=subprocess.check_output(['git','-C',str(path),'status','--porcelain'],text=True)
 assert not status,(name,status)
 repo_state[name]=dict(head=subprocess.check_output(['git','-C',str(path),'rev-parse','HEAD'],text=True).strip(),clean=True)
assert not (P/'live30_off_control/failed.txt').exists()
assert not list(P.glob('*on_control'))
for path in H.glob('*.py'):ast.parse(path.read_text())
subprocess.run(['git','-C','/home/yeranis/repos/gnssFGO','diff','--check'],check=True)
out=dict(passed=True,protected_unchanged=len(manifest),source_repos=repo_state,
 no_on_capture=True,free_bytes=shutil.disk_usage(W).free,
 gnssFGO_head=subprocess.check_output(['git','-C','/home/yeranis/repos/gnssFGO','rev-parse','HEAD'],text=True).strip(),
 evidence_sha256={str(p.relative_to(H)):sha(p) for p in evidence.iterdir()},
 report_sha256=sha(H/'REPORT.txt'),results_sha256=sha(H/'results.json'))
with (H/'final_checks.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps(out,indent=2))
