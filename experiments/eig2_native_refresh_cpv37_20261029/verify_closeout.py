"""Independent final accounting/integrity audit; does not solve any graph."""
import ast
import csv
import hashlib
import json
from pathlib import Path
import subprocess
from datetime import datetime,timezone

H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/eig2_native_refresh_cpv37_20261029')


def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()


pins=json.loads((H/'protected_before.json').read_text())
for path,digest in pins.items():assert sha(path)==digest,path
# The test implementation and original controller remain exact. Protocol wording
# was clarified post-test to call the single solve/retraction one GN step, without
# changing any setting, threshold, input, result or executed code.
implementation=json.loads((H/'implementation_before.json').read_text())
for path,digest in implementation.items():
    if path.endswith('/PROTOCOL.txt'):continue
    assert sha(path)==digest,path
summary=json.loads((H/'results.json').read_text())
assert summary['successful_refresh_steps']==7 and summary['prior_checks_total']==42
assert summary['source_runs']==summary['cbs_on_runs']==0
assert json.loads((H/'finished.json').read_text())['passed']
assert not (H/'continuation_failed.json').exists()
for v in ['release','sanitizer']:
    assert json.loads((P/('build_'+v)/'passed.json').read_text())['passed']
    for suffix in ['','_displaced']:
        d=P/('build_'+v)/('fixture'+suffix)
        assert sha(d/'input_before.bin')==sha(d/'input_after.bin')
        assert all(x['pass']=='1' for x in csv.DictReader((d/'prior_checks.csv').open()))
for x in summary['checkpoints']:
    d=P/'results'/('update_'+str(x['update']))
    assert sha(d/'input_before.bin')==sha(d/'input_after.bin')
    assert x['comparison']['stored_equation_relative']==0
    for branch in ['stored_control','refreshed_once']:
        assert (d/(branch+'_complete.txt')).exists()
        assert not (d/(branch+'_failed.txt')).exists()
        assert x['branches'][branch]['equations']['backward_relative']<1e-14
states=subprocess.check_output(['docker','inspect','--format','{{.Name}} {{.State.Running}}',
    'cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).splitlines()
assert states==['/cbsms_ws_gpu false','/ubuntu-jammy-humble-nvidia false']
owned=[]
for f in [H/'start.json',H/'continuation_start.json']:
    owned.append(json.loads(f.read_text())['pid'])
for f in list(H.glob('*.process.json'))+list(P.glob('build_*/*.process.json')):
    owned.append(json.loads(f.read_text())['pid'])
for pid in owned:assert not Path('/proc',str(pid)).exists(),('PID still exists',pid)
checkpoint=subprocess.check_output(['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
for f in H.glob('*.py'):ast.parse(f.read_text(),filename=str(f))
subprocess.run(['git','diff','--check','--','AGENTS.md','GNSSFGO_RUNTIME_ENGINEERING_PLAN_20261011.md'],cwd=H.parent.parent,check=True)
receipt=dict(passed=True,completed_utc=datetime.now(timezone.utc).isoformat(),
    interpretation='Seven single-step saved-checkpoint comparisons; no trajectory or live-fix claim',
    unchanged_protected_files=len(pins),hesai_checkpoint=checkpoint.strip(),
    source_runs=0,cbs_on_runs=0,production_changes=False,
    synthetic_fixtures=4,real_prior_checks=42,successful_native_refresh_steps=7,
    same_equation_controls=7,container_states=states,owned_pids_exited=owned,
    initial_preflight_failure_preserved=True,no_repeated_dataset_step=True,
    executable_sha256=sha(P/'build_release/test'),
    implementation_sha256={str(P/'NativeRefresh.cpp'):sha(P/'NativeRefresh.cpp')},
    evidence_sha256={str(f):sha(f) for f in H.rglob('*') if f.is_file() and '__pycache__' not in f.parts},
    results_sha256={str(f.relative_to(P)):sha(f) for f in (P/'results').rglob('*') if f.is_file()},
    fixture_sha256={str(f.relative_to(P)):sha(f) for f in P.glob('build_*/fixture*/*') if f.is_file()},
    documentation_sha256={str(f):sha(f) for f in [H.parent.parent/'AGENTS.md',H.parent.parent/'GNSSFGO_RUNTIME_ENGINEERING_PLAN_20261011.md']})
with (H/'closeout.json').open('x') as f:json.dump(receipt,f,indent=2)
print(json.dumps({k:v for k,v in receipt.items() if not k.endswith('sha256')},indent=2))
