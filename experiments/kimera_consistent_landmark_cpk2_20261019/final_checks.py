"""Preserve historical artifacts; report the failed gate, never promote it."""
import hashlib,json,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent
R=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/kimera_consistent_landmark_cpk2_20261019')
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
manifest=json.loads((R/'protected_before.json').read_text())
changed=[p for p,h in manifest.items() if sha(p)!=h]
assert not changed,changed
release=json.loads((R/'release_all_status.json').read_text())
san=json.loads((R/'san_fixtures_status.json').read_text())
assert release['returncode']==1 and not release['passed']
assert san['passed']
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Running}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).strip().splitlines()
assert all(s.endswith(' false') for s in states),states
sources={str(p):sha(p) for p in H.iterdir() if p.is_file() and p.name!='final_checks.json'}
out={'protected_files':len(manifest),'protected_changes':changed,
     'saved_factor_gate_passed':False,'synthetic_sanitizer_fixtures_passed':True,
     'linked_libraries_instrumented':False,'leaks_checked':False,
     'estimator_integrated':False,'source_replays':0,'production_changed':False,
     'containers':states,'HEAD':subprocess.check_output(['git','-C','/home/yeranis/repos/gnssFGO','rev-parse','HEAD'],text=True).strip(),
     'candidate_sources':sources}
with (H/'final_checks.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps({k:v for k,v in out.items() if k!='candidate_sources'},indent=2))
