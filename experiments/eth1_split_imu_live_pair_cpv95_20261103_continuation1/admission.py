"""Freeze ON-only continuation after all saved OFF gates. No capture/startup."""
import ast,datetime,json,os,shutil,socket,subprocess,sys
from pathlib import Path
from paths import H,E,W,P,ORIGINAL
from integrity import sha,tree_snapshot
def read(p):return json.loads(Path(p).read_text())
def save(name,v):
    if (H/name).exists():
        assert read(H/name)==v,('Do not overwrite prior admission evidence',name)
        return
    with (H/name).open('x') as f:json.dump(v,f,indent=2)
def call(args):return subprocess.check_output(args,text=True)
assert not (H/'prepared.json').exists() and not (H/'start.json').exists()
assert not any(P.glob('full_on_*'))
for name in ('off_validated.json','validator_test.json','report_preflight.json'):
    assert read(H/name)['passed']
assert read(ORIGINAL/'closeout.json')['passed']
assert read(ORIGINAL/'failed.json')['stage']=='validate_off_source'
assert read(P/'full_off_control/owned_job_cleanup.json')=={'master':0,'source':0,'receiver':0}
assert call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
for port in (11601,11602):
    with socket.socket() as s:s.bind(('127.0.0.1',port))
jobs=[]
for receipt in [ORIGINAL/'start.json',*ORIGINAL.glob('*_process.json'),*(P/'full_off_control').glob('*_process.json'),
                H/'saved_off_gates_start.json',*H.glob('*_process.json')]:
    z=read(receipt);f=Path(f"/proc/{z['pid']}/stat")
    alive=f.exists() and f.read_text().split()[21]==z['proc_start']
    assert not alive,receipt;jobs.append(dict(receipt=str(receipt),pid=z['pid'],original_identity_alive=False))
z=read(ORIGINAL/'launch.json');f=Path(f"/proc/{z['owned_supervisor_pid']}/stat")
assert not f.exists() or f.read_text().split()[21]!=z['supervisor_proc_start']
save('previous_jobs_exited.json',dict(passed=True,jobs=jobs,prior_supervisor=z['owned_supervisor_pid']))
pins=read(P/'protected_manifest.json');original_count=len(pins)
for path,digest in pins.items():assert sha(path)==digest,path
print('PASS original protected files:',original_count,flush=True)
parity={}
for mode,name in [('off','eth1_alpha_bias_rw_full_off_cpv89_20261102'),('on','eth1_split_imu_persistent_on_cpv91_20261102_attempt1')]:
    orig=W/'runs'/name/'prepared';n=0
    for directory in ('input_profiles','private_build'):
        for f in (orig/directory).rglob('*'):
            if f.is_file():assert sha(f)==sha(P/mode/f.relative_to(orig));n+=1
    assert n==32
    parity[mode]=n
    assert read(E/name/'complete.json')['passed']
prior_counts={}
for name in ('eth1_alpha_bias_rw_full_off_cpv89_20261102','eth1_split_imu_persistent_on_cpv91_20261102_attempt1','eig2_symmetric_persistent_full_cpv84_20261101_attempt1'):
    z=read(E/name/'complete.json')
    for path,digest in z['output_sha256'].items():assert sha(path)==digest,path
    prior_counts[name]=len(z['output_sha256'])
checks=[]
for command in (['python3','-B',str(P/'transport_tools/qualify_runtime.py')],
                ['python3','-B',str(P/'transport_tools/launch_grandtour_live_receiver.py'),'--help'],
                ['python3','-B',str(P/'run_case_on.py'),'--help']):
    result=subprocess.run(command,text=True,capture_output=True)
    assert result.returncode==0,(command,result.stderr)
    checks.append(dict(command=command,exit=result.returncode,stdout=result.stdout,stderr=result.stderr))
save('host_runtime_preflight.json',dict(passed=True,checks=checks))
for p in H.glob('*.py'):ast.parse(p.read_text(),str(p))
sys.path.insert(0,str(P));from eof_boundary import test_boundaries
assert test_boundaries()=={'valid':1,'rejected':4}
snapshot={}
for name in ('full_off_source','full_off_receiver','full_off_control'):
    snapshot[name]=tree_snapshot(P/name);print('Pinned saved OFF:',name,snapshot[name]['files'],flush=True)
save('preserved_off.json',snapshot)
hesai=call(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'])
storage=read(ORIGINAL/'storage_preflight.json');free=shutil.disk_usage(P).free
assert free>storage['required_free_bytes']
storage.update(free_bytes=free,continuation='Conservatively retaining original whole-pair reserve for one remaining ON')
save('storage_preflight.json',storage)
pins[str(P/'protected_manifest.json')]=sha(P/'protected_manifest.json')
# Original failure artifacts and all completed supplements become immutable.
# REPORT remains mutable only until complete.json seals it.
for root in (ORIGINAL,H):
    for p in root.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts and not (root==H and p.name=='REPORT.txt'):
            pins[str(p.resolve())]=sha(p)
save('protected_manifest.json',pins)
save('prepared.json',dict(passed=True,prepared_path=str(P),original_host_root=str(ORIGINAL),
    original_protected_files=original_count,protected_files=len(pins),prior_terminal_counts=prior_counts,
    unchanged_profile_binary_files=parity,hesai=hesai,OFF_validated_and_preserved=True,ON_not_started=True,
    source_captures_to_start=1,receiver_captures_to_start=1,OFF_rerun=False,created=datetime.datetime.now().astimezone().isoformat()))
print('PASS ON-only admission; original OFF and failed attempt unchanged',flush=True)
