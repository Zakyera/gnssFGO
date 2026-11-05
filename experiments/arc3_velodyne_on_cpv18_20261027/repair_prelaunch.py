"""Correct caught prelaunch identity-substitution error; no runtime exists."""
from pathlib import Path
import difflib,hashlib,json,shutil
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')
P=W/'gnssfgo_arc3_velodyne_on_cpv18_20261027/prepared'
S=W/'gnssfgo_arc3_velodyne_qcquarter_cpv17_20261026/prepared'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def main():
 assert not (H/'launch.json').exists() and not (H/'status.json').exists()
 assert not any(P.glob('full_*_source')) and not any(P.glob('full_*_control'))
 old=(P/'input_audit.json').read_text();correct=(S/'input_audit.json').read_text()
 assert old==correct.replace('11488','11489')
 assert len(list(difflib.unified_diff(correct.splitlines(),old.splitlines())))==11
 evidence=H/'prelaunch_port_rebase_evidence';evidence.mkdir()
 for src in [P/'input_audit.json',P/'protected_manifest.json']:
  shutil.copy2(src,evidence/src.name)
 shutil.copy2(S/'input_audit.json',P/'input_audit.json')
 assert sha(P/'input_audit.json')==sha(S/'input_audit.json')
 manifest=json.loads((P/'protected_manifest.json').read_text())
 changed=[str(P/'input_audit.json'),str(H/'prepare.py')]
 for name,want in manifest.items():
  if name not in changed:assert sha(name)==want,name
 for name in changed:manifest[name]=sha(name)
 # Archive the initial preparation manifest; pin the finalized launch inputs.
 with (P/'protected_manifest.json').open('w') as f:json.dump(manifest,f,indent=2)
 report=dict(passed=True,no_capture_or_container_start=True,
  cause='Broad port-name substitution also changed one raw scan point count11488 to11489 in copied audit',
  repair='Restore exact immutable CP-V17 input audit; exclude raw audit from identity rebase in generator',
  prior_prelaunch_evidence=str(evidence),source_or_model_change=False,changed_manifest_entries=changed,
  raw_input_audit_sha256=sha(P/'input_audit.json'),protected_files=len(manifest))
 with (H/'prelaunch_review.json').open('x') as f:json.dump(report,f,indent=2)
 print(json.dumps(report,indent=2))
if __name__=='__main__':main()
