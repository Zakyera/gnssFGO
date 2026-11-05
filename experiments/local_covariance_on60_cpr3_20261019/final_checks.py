"""Preserve compact validated ON evidence; no experiments or source modifications."""
from pathlib import Path
import ast,hashlib,json,shutil,subprocess
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs/gnssfgo_eig2_local_cov60_on_cpr3_20261019/prepared'
S=P/'live60_on_source';R=P/'live60_on_receiver'
V=Path('/home/yeranis/results/gnssfgo_eig2_local_cov60_on_cpr3_20261019/on')
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
assert json.loads((H/'results.json').read_text())['passed']
manifest=json.loads((P/'protected_manifest.json').read_text())
for p,h in manifest.items():assert sha(p)==h,p
out=H/'evidence';out.mkdir(exist_ok=False)
paths=[P/'protected_manifest.json',P/'preflight_on.json',P/'live60_on_control/complete.json',
 S/'local_provenance.json',S/'candidate_validation.json',S/'source_coverage.json',
 S/'effective_mode_check.json',S/'run_metadata.txt',R/'transport_complete.json',
 R/'resolved_config.json',V/'validation.json',V/'repeat_validation.json',P/'evaluation_full/summary.json']
for p in paths:shutil.copy2(p,out/p.name)
states={}
for name in ('glim_ros1','Kimera-VIO','cbs'):
 p=W/'src'/name
 status=subprocess.check_output(['git','-C',str(p),'status','--porcelain'],text=True)
 assert not status,(name,status)
 states[name]=dict(clean=True,head=subprocess.check_output(['git','-C',str(p),'rev-parse','HEAD'],text=True).strip())
assert not (P/'live60_on_control/failed.txt').exists()
assert not list(P.glob('*off_control'))
for p in H.glob('*.py'):ast.parse(p.read_text())
subprocess.run(['git','-C','/home/yeranis/repos/gnssFGO','diff','--check'],check=True)
result=dict(passed=True,protected_unchanged=len(manifest),source_repos=states,
 no_off_capture=True,free_bytes=shutil.disk_usage(W).free,
 gnssFGO_head=subprocess.check_output(['git','-C','/home/yeranis/repos/gnssFGO','rev-parse','HEAD'],text=True).strip(),
 evidence_sha256={str(p.relative_to(H)):sha(p) for p in out.iterdir()},
 report_sha256=sha(H/'REPORT.txt'),results_sha256=sha(H/'results.json'))
with (H/'final_checks.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
