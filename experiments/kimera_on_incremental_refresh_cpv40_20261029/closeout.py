"""Read-only gate evidence/ownership verification and exclusive receipt."""
import hashlib,json,os,subprocess,time
from pathlib import Path
H=Path(__file__).resolve().parent;R=H.parents[1];W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');P=W/'runs'/H.name
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
pins=json.loads((H/'protected_before.json').read_text())
for p,want in pins.items():assert sha(p)==want,p
for stage in ('release_exact_edge','san_initial','backend_initial'):
    assert json.loads((P/stage/'passed.json').read_text())['passed']
    receipt=json.loads((H/(stage+'_receipt.json')).read_text());assert receipt['exit']==0
    assert all(s.endswith(' exited') for s in receipt['after'])
    for inside,want in json.loads((P/stage/'sources.json').read_text()).items():
        assert sha(inside.replace('/workspace/cbs_gtsam4.3',str(W)))==want,inside
parity=json.loads((P/'release_exact_edge/parity.json').read_text());assert all(parity.values())
assert sha(P/'private_build/libcbs.so')==sha(W/'runs/kimera_off_incremental_refresh_cpv38_20261029/private_build/libcbs.so')
deps=(P/'backend_initial/dependencies.log').read_text();assert 'not found' not in deps and 'undefined symbol' not in deps
containers=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).splitlines()
assert all(x.endswith(' exited') for x in containers)
owned=[]
for p in H.glob('*_process.json'):
    r=json.loads(p.read_text());pid=r['host_pid'];cmd=Path(f'/proc/{pid}/cmdline')
    if cmd.exists():assert 'kimera_on_incremental_refresh_cpv40_20261029' not in cmd.read_bytes().decode(errors='replace'),pid
    owned.append(pid)
hesai=subprocess.check_output(['python3',str(R/'experiments/checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
metrics={}
for line in (P/'release_exact_edge/ctest0.log').read_text().splitlines():
    if line.startswith('1: ') and '=' in line:
        key,value=line[3:].split('=',1)
        try:metrics.setdefault(key,[]).append(float(value))
        except ValueError:pass
result=dict(passed=True,finished_ns=time.time_ns(),protected_files_unchanged=len(pins),hesai_checkpoint=hesai,
    containers=containers,owned_gate_processes_exited=owned,parity=parity,metrics=metrics,
    backend_sha256=sha(P/'private_build/libkimera_vio.so'),libcbs_unchanged=True,
    failures_preserved=['release_initial','release_fault_stage'],source_captures=0,defaults_changed=False,
    limitations='Synthetic engineering admission only; no nonlinear stationarity, runtime CBS-ON or accuracy claim; vendor DSOs not sanitizer-instrumented.')
with (H/'closeout.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
