"""Preserve small run evidence and verify immutable prior inputs after S5."""
from pathlib import Path
import json
import shutil
import subprocess
from prepare import HERE,ROOT,RUN,OLD,BUILD,PRIOR,sha

assert json.loads((RUN/'validation_logs/complete.json').read_text())['passed']
assert json.loads((HERE/'summary.json').read_text())['functional_validation_passed']
manifest=json.loads((ROOT/'protected_manifest.json').read_text())
for p,h in manifest.items():assert sha(p)==h,p
repos=json.loads((PRIOR/'before_integrity.json').read_text())['source_repositories']
for p,v in repos.items():
    assert subprocess.check_output(['git','-C',p,'rev-parse','HEAD'],text=True).strip()==v['head']
    assert subprocess.check_output(['git','-C',p,'status','--porcelain'],text=True).strip()==v['status']
evidence=HERE/'evidence';evidence.mkdir(exist_ok=False)
def copy(p,dest):
    dst=evidence/dest;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,dst)
for p in ROOT.iterdir():
    if p.is_file() and p.suffix in ['.json','.py','.sh','.env','.launch']:
        copy(p,'harness_and_validation/'+p.name)
for sub in ['preflight','preflight_sandbox_denied','preflight_reference_path_hold','capacity160_on_control']:
    for p in (ROOT/sub).iterdir():
        if p.is_file() and p.stat().st_size<5_000_000:copy(p,sub+'/'+p.name)
for name in ['source_coverage.json','effective_mode_check.json','resolved_glim_params.yaml',
             'resolved_kimera_params.yaml','run_metadata.txt','complete.marker']:
    copy(ROOT/'capacity160_on_source'/name,'source/'+name)
for name in ['transport_status.json','manifest.json','complete.marker']:
    p=ROOT/'capacity160_on_receiver'/name
    if p.exists():copy(p,'receiver/'+name)
for name in ['summary.json','source_accounting.csv','unsupported_beliefs.csv']:
    copy(ROOT/'capacity160_on_receiver/receiver'/name,'receiver/'+name)
for p in (RUN/'validation_logs').iterdir():
    if p.is_file():copy(p,'validation_logs/'+p.name)
copy(RUN/'memory_observations.jsonl','memory_observations.jsonl')
for label,p in [('release',BUILD/'tests/result.json'),('sanitizer_retained',BUILD.parent/'asan_nopie/tests/result.json')]:
    copy(p,'retained_tests/'+label+'.json')
with (HERE/'integrity.json').open('x') as f:
    json.dump(dict(passed=True,protected_files=len(manifest),source_repositories=repos,
        private_bridge_sha256=sha(BUILD/'libglim_ros.so'),
        private_cpu_sha256=sha(BUILD/'libgtsam_points.so.1.2.1'),
        evidence_files={str(p.relative_to(evidence)):sha(p) for p in sorted(evidence.rglob('*')) if p.is_file()}),f,indent=2)
print('ARCHIVED_AND_INTEGRITY_PASS',len(manifest))
