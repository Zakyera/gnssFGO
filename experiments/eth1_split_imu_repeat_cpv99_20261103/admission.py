"""Read-only input/identity checks and new admission receipts; no container launch."""
import ast,datetime,hashlib,json,os,shutil,socket,subprocess,sys,xml.etree.ElementTree as ET
from pathlib import Path
from paths import H,E,W,P,V,PY,BASE_H,BASE_P
def read(p):return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,z):
    with Path(p).open('x') as f:json.dump(z,f,indent=2,allow_nan=False)
def call(args):return subprocess.check_output(args,text=True)
assert not (H/'prepared.json').exists() and not (H/'start.json').exists()
assert not V.exists() and not any(P.glob('full_*_source')) and not any(P.glob('full_*_receiver'))
assert read(H/'validator_test.json')['passed'] and read(H/'report_preflight.json')['passed']
assert call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
for port in (11611,11612):
    with socket.socket() as s:s.bind(('127.0.0.1',port))
assert all(not os.environ.get(x) for x in ('CPV66_AUDIT_DIR','CPV66_TEST_ALL','KIMERA_IMU_SAMPLE_SUPPORT'))
parity=[]
for f in P.rglob('*'):
    if not f.is_file() or '__pycache__' in f.parts:continue
    rel=f.relative_to(P);old=BASE_P/rel
    assert old.is_file(),rel
    assert f.stat().st_mode&0o777==old.stat().st_mode&0o777,rel
    if f.suffix in ('.py','.sh','.env','.json','.launch','.txt'):
        want=old.read_text().replace(BASE_P.parent.name,H.name)
        if f.name in ('source.env','run_case_on.py','full_off.json','full_on.json'):
            want=want.replace('11601','11611').replace('11602','11612')
        if f.name in ('full_off.json','full_on.json'):
            want=want.replace('CP-V95','CP-V99').replace('eth1_cpv95_','eth1_cpv99_')
        assert f.read_text()==want,rel
    else:assert sha(f)==sha(old),rel
    if f.suffix=='.py':ast.parse(f.read_text(),str(f))
    if f.suffix in ('.env','.sh'):subprocess.run(['bash','-n',str(f)],check=True)
    if f.suffix=='.launch':ET.parse(f)
    parity.append(str(rel))
for f in H.glob('*.py'):ast.parse(f.read_text(),str(f))
save(H/'profile_parity.json',dict(passed=True,files=len(parity),relative_files=parity,
    allowed_changes=['fresh root paths','ports11611/11612','new receiver session/sequence labels'],
    profiles_private_binaries_adapter_unchanged=True,CP98_observer_enabled=False))
host=[]
for args in (['python3','-B',str(P/'transport_tools/qualify_runtime.py')],
             ['python3','-B',str(P/'transport_tools/launch_grandtour_live_receiver.py'),'--help'],
             ['python3','-B',str(P/'run_case_on.py'),'--help']):
    z=subprocess.run(args,text=True,capture_output=True);assert z.returncode==0,(args,z.stderr)
    host.append(dict(command=args,exit=z.returncode,stdout=z.stdout,stderr=z.stderr))
save(H/'host_runtime_preflight.json',dict(passed=True,checks=host))
assert os.access(P/'receiver_cpk19.sh',os.X_OK)
sys.path.insert(0,str(P));from eof_boundary import test_boundaries
assert test_boundaries()=={'valid':1,'rejected':4}
import compare_repeat as q
import numpy as np
ts=np.arange(601,dtype=np.int64)*100_000_000
old=dict(timestamp_ns=ts,common_valid=np.ones(601,bool),reference_position=np.zeros((601,3)),reference_rotation=np.tile(np.eye(3),(601,1,1)))
new={k:v.copy() for k,v in old.items()};old['common_valid'][100:110]=False;new['common_valid'][300:320]=False
_,_,_,valid,cuts=q.common_data(old,new)
assert valid.sum()==571 and len(cuts)==3
save(H/'repeat_mask_test.json',dict(passed=True,synthetic_rows=601,intersection_rows=571,segments=3))
pins=read(BASE_H/'protected_manifest.json')
for root in (BASE_H,E/'eth1_early_native_capture_cpv98_20261103'):
    z=read(root/'complete.json');assert z['passed']
    pins.update(z['output_sha256']);pins[str(root/'complete.json')]=sha(root/'complete.json')
    if (root/'post_closeout.json').exists():pins[str(root/'post_closeout.json')]=sha(root/'post_closeout.json')
pins.update(read(BASE_H/'evaluation/input_sha256.json'))
fixture=read(E/BASE_P.parent.name/'live_adapter_test.json')
assert fixture['passed']
pins.update(fixture['input_sha256'])
save(H/'live_adapter_test.json',dict(passed=True,qualification_reused_from=str(E/BASE_P.parent.name/'live_adapter_test.json'),
     adapter_bytes_identical_to_qualified=True,exact_host_fixture_passed=True,actual_container_fixture_mandatory_before_playback=True))
audit=read(P/'input_audit.json')
for ent in audit['bag_files'].values():
    f=Path(ent['path']);st=f.stat();assert st.st_size==ent['bytes'] and st.st_mtime_ns==ent['mtime_ns'];pins[str(f)]=ent['sha256']
pins[audit['tf']['path']]=audit['tf']['sha256']
for path,digest in pins.items():assert sha(path)==digest,path
print('Original protected/terminal/input hashes passed:',len(pins),flush=True)
jobs=[]
for root in (BASE_H,E/'eth1_early_native_capture_cpv98_20261103'):
    for f in [root/'start.json',*root.glob('*_process.json')]:
        z=read(f)
        if 'pid' not in z or 'proc_start' not in z:continue
        stat=Path('/proc')/str(z['pid'])/'stat'
        alive=stat.exists() and stat.read_text().split()[21]==str(z['proc_start'])
        assert not alive,f;jobs.append(dict(receipt=str(f),pid=z['pid'],original_identity_alive=False))
save(H/'prior_jobs_exited.json',dict(passed=True,jobs=jobs))
hesai=call(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'])
storage=read(E/BASE_P.parent.name/'storage_preflight.json')
storage.update(free_bytes=shutil.disk_usage(P).free,basis='Unchanged original CP95 whole-pair conservative forecast and5GiB reserve')
assert storage['free_bytes']>storage['required_free_bytes']
save(H/'storage_preflight.json',storage)
for root in (P,H):
    for f in root.rglob('*'):
        if f.is_file() and '__pycache__' not in f.parts and f.name not in ('REPORT.txt','protected_manifest.json','status.json'):
            pins[str(f.resolve())]=sha(f)
save(P/'protected_manifest.json',pins)
data=dict(passed=True,prepared_path=str(P),protected_files=len(pins),hesai=hesai,parity_files=len(parity),
          created=datetime.datetime.now().astimezone().isoformat(),source_captures_to_start=2,receiver_captures_to_start=2,
          original_pair=str(BASE_H),fresh_repeat_pair=True,scientific_changes=False,CP98_observer_enabled=False,
          source_retries_authorized=0,ports=[11611,11612])
save(P/'preflight.json',data);save(H/'prepared.json',data)
print(json.dumps(data,indent=2),flush=True)
