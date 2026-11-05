"""Read-only evidence verification; writes a new exclusive closeout receipt."""
from pathlib import Path
import datetime
import hashlib
import json
import re
import shutil
import subprocess

H=Path(__file__).resolve().parent
R=H.parents[1]
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(p.read_text())
def write(p,data):
    with p.open('x') as f:json.dump(data,f,indent=2)
pins=read(H/'protected_before.json')
for path,digest in pins.items():assert sha(path)==digest,path
initial=read(H/'implementation_before.json')
changes=[]
for path,digest in initial.items():
    if sha(path)!=digest:
        assert Path(path).name in ('test.cpp','build.py'),path
        changes.append(path)
assert (P/'baseline.cpp').read_bytes()==(H/'baseline.cpp').read_bytes()
for name in ('BpsamRefresh.inc','OffRefresh.h','test.cpp','build.py'):
    assert (P/name).read_bytes()==(H/name).read_bytes(),name
release=P/'release_lifecyclefix'
san=P/'san_initial'
backend=P/'backend_initial'
for d in (release,san,backend):assert read(d/'passed.json')['passed']
base=P/'release_initial'
assert (base/'baseline.log').read_bytes()==(base/'disabled.log').read_bytes()
logs=[(release/'ctest0.log').read_text(),(release/'ctest1.log').read_text(),
      (san/'ctest0.log').read_text()]
for log in logs:
    assert '100% tests passed' in log and 'PASS CP-V38 OFF incremental refresh' in log
    assert 'FAIL ' not in log
def metrics(text):
    return [{'name':m[0],'value':float(m[1])} for m in
            re.findall(r'^1: ([A-Za-z_]+)=([-+0-9.eE]+)$',text,re.M)]
assert metrics(logs[0])==metrics(logs[1])
dependencies=(backend/'dependencies.log').read_text()
assert 'not found' not in dependencies and 'undefined symbol' not in dependencies
assert str(P).replace(str(W),'/workspace/cbs_gtsam4.3')+'/private_build/libcbs.so' in dependencies
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}',
    'cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).splitlines()
assert all(x.endswith(' exited') for x in states),states
for receipt in H.glob('*_receipt.json'):
    assert all(x.endswith(' exited') for x in read(receipt)['after']),receipt
hesai=subprocess.check_output(['python3',str(R/'experiments/checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),
                             '--originals'],text=True).strip()
jobs=[]
for p in H.glob('*_process.json'):
    pid=read(p)['host_pid']
    assert not Path('/proc',str(pid)).exists(),pid
    jobs.append(pid)
assert shutil.disk_usage(H).free>5*1024**3
for repo in (R,W/'src/cbsms',W/'src/cbs',W/'src/Kimera-VIO'):
    subprocess.run(['git','-C',str(repo),'diff','--check'],check=True)
write(H/'closeout.json',{
    'passed':True,'completed_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
    'scope':'Private OFF incremental synthetic gates and private DSO only; no trajectory result',
    'protected_files_unchanged':len(pins),'hesai_checkpoint':hesai,
    'candidate_implementation_unchanged_since_first_compile':True,
    'test_build_helper_repairs':changes,'source_runs':0,'cbs_on_runs':0,
    'production_changes':False,'container_states':states,'owned_host_jobs_exited':jobs,
    'disabled_output_sha256':sha(base/'baseline.log'),
    'release_metrics':metrics(logs[0]),'sanitizer_metrics':metrics(logs[2]),
    'release_executable_sha256':sha(release/'test'),
    'private_backend_sha256':sha(P/'private_build/libkimera_vio.so'),
    'candidate_files':{str(p):sha(p) for p in (P/'BpsamRefresh.inc',P/'OffRefresh.h',
        P/'include/cbs/bpsam/bpsam.h',P/'include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h')},
    'evidence_sha256':{str(p):sha(p) for root in (H,release,san,backend)
        for p in root.rglob('*') if p.is_file() and p.name not in ('closeout.json',)},
    'free_bytes':shutil.disk_usage(H).free})
print('PASS CP-V38 closeout; protected files',len(pins),';',hesai)
