"""Read-only preflight plus fresh CP103 receipts; no source or container launch."""
import ast,datetime,hashlib,json,os,shutil,socket,subprocess,sys
from pathlib import Path
from paths import H,E,W,P,V,PY,BASE_H,BASE_P,FIX_H,FIX_K
from injection_schema import validate_injections
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,z):
    with p.open('x') as f:json.dump(z,f,indent=2)
def call(a):return subprocess.check_output(a,text=True)
def identity(pid):
    p=Path(f'/proc/{pid}/stat');return p.read_text().rsplit(')',1)[1].split()[19] if p.exists() else None
assert not (H/'prepared.json').exists() and not (H/'start.json').exists()
assert not V.exists() and not any(P.glob('full_*_source'))
assert read(FIX_H/'complete.json')['passed']
assert call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
with socket.socket() as s:s.bind(('127.0.0.1',11622))
assert all(not os.environ.get(k) for k in ('CPV66_AUDIT_DIR','CPV66_TEST_ALL','KIMERA_IMU_SAMPLE_SUPPORT','CPV102_PRIOR_FAILURE_DIR'))
assert os.access(P/'receiver_cpk19.sh',os.X_OK)
for p in H.glob('*.py'):ast.parse(p.read_text(),str(p))
host=[]
for args in (['python3','-B',str(P/'transport_tools/qualify_runtime.py')],
             ['python3','-B',str(P/'transport_tools/launch_grandtour_live_receiver.py'),'--help'],
             ['python3','-B',str(P/'run_case_on.py'),'--help'],
             [PY,'-B',str(H/'validate_source.py'),'--help']):
    r=subprocess.run(args,text=True,capture_output=True);assert r.returncode==0,(args,r.stderr)
    host.append(dict(command=args,exit=r.returncode,stdout=r.stdout,stderr=r.stderr))
save(H/'host_runtime_preflight.json',dict(passed=True,checks=host))
sys.path.insert(0,str(P));from eof_boundary import test_boundaries
assert test_boundaries()=={'valid':1,'rejected':4}
bad=[('off',{}),('off',{'total':0}),('off',False),('off',0.),('off','0'),('off',1),
 ('on',0),('on',{}),('on',{'G2K_queued':1,'G2K_persistent':1}),
 ('on',{'G2K_queued':1,'G2K_persistent':1,'K2G_persistent':0}),
 ('on',{'G2K_queued':1,'G2K_persistent':1,'K2G_persistent':True}),
 ('on',{'G2K_queued':1,'G2K_persistent':1,'K2G_persistent':'1'}),
 ('on',{'G2K_queued':2,'G2K_persistent':1,'K2G_persistent':1}),
 ('on',{'G2K_queued':1,'G2K_persistent':1,'K2G_persistent':1,'extra':1})]
for mode,value in bad:
    try:validate_injections(mode,value)
    except AssertionError:pass
    else:raise AssertionError((mode,value))
for mode,name in [('off','eth1_alpha_bias_rw_full_off_cpv89_20261102'),('on','eth1_split_imu_persistent_on_cpv91_20261102_attempt1')]:
    validate_injections(mode,read(W/'runs'/name/'prepared'/f'{mode}_source/source_coverage.json')['external_injections'])
assert sha(H/'injection_schema.py')==sha(BASE_H/'injection_schema.py')
assert read(BASE_H/'validator_test.json')['passed']
save(H/'validator_test.json',dict(passed=True,negative_schemas=14,real_saved_injection_schemas=2,
    inherited_full_validator_tests=str(BASE_H/'validator_test.json'),new_validator_additions='qualified DSO hash and empty failure archive',eof_controls=5))
pins=read(FIX_H/'tests_v2/inputs.json')
pins.update(read(FIX_H/'complete.json')['output_sha256']);pins[str(FIX_H/'complete.json')]=sha(FIX_H/'complete.json')
for p in BASE_H.rglob('*'):
    if p.is_file() and '__pycache__' not in p.parts:pins[str(p)]=sha(p)
parity=read(H/'copy_parity.json');assert parity['passed']
for ent in parity['files']:
    assert sha(BASE_P/ent['path'])==ent['old_sha256'] and sha(P/ent['path'])==ent['new_sha256']
    pins[str(BASE_P/ent['path'])]=ent['old_sha256']
for name in ('validate_local.py','validate_case.py','frame_on.py','injection_schema.py'):
    assert sha(H/name)==sha(BASE_H/name),name
audit=read(P/'input_audit.json')
for ent in audit['bag_files'].values():
    p=Path(ent['path']);st=p.stat();assert st.st_size==ent['bytes'] and st.st_mtime_ns==ent['mtime_ns']
    pins[str(p)]=ent['sha256']
pins[audit['tf']['path']]=audit['tf']['sha256']
for p,v in pins.items():assert sha(p)==v,p
jobs=[]
for root in (FIX_H,BASE_H):
    for p in root.rglob('*.json'):
        if p.name!='start.json' and not p.name.endswith('_process.json'):continue
        r=read(p)
        if 'pid' not in r or 'proc_start' not in r:continue
        assert identity(r['pid'])!=str(r['proc_start']),p
        jobs.append(dict(receipt=str(p),pid=r['pid'],exited=True))
save(H/'prior_jobs_exited.json',dict(passed=True,jobs=jobs))
hesai=call(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'])
assert 'PASS: 4 cases, 667 saved files; originals=True' in hesai
storage=read(BASE_H/'storage_preflight.json')
storage.update(free_bytes=shutil.disk_usage(P).free,basis='Conservative CP99 whole-pair reserve retained for one ON arm')
assert storage['free_bytes']>storage['required_free_bytes']
save(H/'storage_preflight.json',storage)
for root in (H,P):
    for p in root.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts and p.name not in ('REPORT.txt','prepared.json','protected_manifest.json','preflight.json'):
            pins[str(p)]=sha(p)
save(P/'protected_manifest.json',pins)
out=dict(passed=True,created=datetime.datetime.now().astimezone().isoformat(),prepared_path=str(P),
         protected_files=len(pins),hesai=hesai,source_captures_authorized=1,receiver_captures_authorized=1,
         mode='on',port=11622,corrected_backend_qualification=True,scientific_settings_unchanged=True,
         identical_binary_repeat=False,old_OFF_reused_or_repeated=False,private_kimera_sha256=sha(P/'on/private_build/libkimera_vio.so'))
save(P/'preflight.json',out);save(H/'prepared.json',out)
print(json.dumps(out,indent=2))
