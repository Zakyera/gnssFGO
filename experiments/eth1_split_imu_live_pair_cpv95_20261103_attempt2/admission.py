"""Read-only admission plus fresh receipts. Does not start containers or estimators."""
import ast,hashlib,importlib.util,json,os,re,shutil,socket,subprocess,sys,xml.etree.ElementTree as ET
from pathlib import Path
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name/'prepared';sys.dont_write_bytecode=True
old={'off':'eth1_alpha_bias_rw_full_off_cpv89_20261102','on':'eth1_split_imu_persistent_on_cpv91_20261102_attempt1'}
def read(p):return json.loads(p.read_text())
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def call(args):return subprocess.check_output(args,text=True)
assert not (H/'prepared.json').exists() and not (H/'start.json').exists()
assert not any(P.glob('full_*_source')) and not any(P.glob('full_*_receiver'))
assert read(H/'live_adapter_test.json')['passed']
assert call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
for port in (11599,11600):
    with socket.socket() as s:s.bind(('127.0.0.1',port))
syntax=[];parity={}
for root in (H,P):
    for f in root.rglob('*'):
        if not f.is_file():continue
        if f.suffix=='.py':ast.parse(f.read_text(),str(f));syntax.append(str(f))
        elif f.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(f)],check=True);syntax.append(str(f))
        elif f.suffix=='.launch':ET.parse(f);syntax.append(str(f))
for mode,name in old.items():
    orig=W/'runs'/name/'prepared';count=0
    for folder in ('input_profiles','private_build'):
        for f in (orig/folder).rglob('*'):
            if f.is_file():assert sha(f)==sha(P/mode/f.relative_to(orig)),f;count+=1
    assert read(P/mode/'input_audit.json')==read(orig/'input_audit.json')
    assert (P/mode/'expected_headers.json').read_bytes()==(orig/'expected_headers.json').read_bytes()
    # All copied frozen source code/launch differences are paths, explicit live-master integration,
    # and relocating the unchanged private library lookup to the per-mode harness directory.
    for f in orig.iterdir():
        if not f.is_file() or f.suffix not in ('.py','.sh','.launch','.env'):continue
        text=f.read_text().replace(name+'/prepared/'+mode+'_source',H.name+'/prepared/full_'+mode+'_source').replace(name+'/prepared',H.name+'/prepared/'+mode).replace(name,H.name)
        if f.name=='source.env':
            text=text.replace('CBSMS_RUN_ROOT=/workspace/cbs_gtsam4.3/runs/'+H.name+'/prepared/'+mode,'CBSMS_RUN_ROOT=/workspace/cbs_gtsam4.3/runs/'+H.name+'/prepared')
            text=text.replace('CBSMS_USE_EXISTING_ROS_MASTER=false','CBSMS_USE_EXISTING_ROS_MASTER=true')
            text=re.sub(r'CBSMS_ROS_MASTER_PORT=\d+','CBSMS_ROS_MASTER_PORT='+str(11599+(mode=='on')),text)
        if f.name=='source_runner_on.sh':text=text.replace('${run_root}/private_build','${experiment_dir}/private_build')
        assert text==(P/mode/f.name).read_text(),f
    for f in (P/mode).glob('*.launch'):
        text=f.read_text();assert name not in text
        for value in re.findall(r'/workspace/cbs_gtsam4\.3/runs/[^"\s<]+',text):
            if H.name+'/prepared/' in value:
                assert '/'+mode+'/' in value or '/full_'+mode+'_source/' in value,(f,value)
    parity[mode]=dict(passed=True,unchanged_profile_and_binary_files=count,only_static_rebase_and_live_master_changes=True)
assert (P/'off/input_profiles/kimera_profile/ImuParams.yaml').read_bytes()==(P/'on/input_profiles/kimera_profile/ImuParams.yaml').read_bytes()
profile_result=dict(passed=True,modes=parity)
if (H/'profile_parity.json').exists():assert read(H/'profile_parity.json')==profile_result
else:save(H/'profile_parity.json',profile_result)
sys.path.insert(0,str(P/'transport_tools'));sys.path.insert(0,str(P))
from live_receiver_protocol import validate_config
from eof_boundary import test_boundaries
for mode in ('off','on'):validate_config(read(P/f'full_{mode}.json'))
assert os.access(P/'receiver_cpk19.sh',os.X_OK) and (P/'receiver_cpk19.sh').stat().st_mode&0o777==0o755
tests=test_boundaries();assert tests=={'valid':1,'rejected':4}
spec=importlib.util.spec_from_file_location('cp95_eval_admission',H/'evaluate.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
row=next(r for r in m.d.m.c.b.ROWS if r[0]=='hesai_eth1');_,_,ref=m.d.m.c.b.load(row)
assert len(ref.times)==69601 and 0<=ref.times[0]-m.epoch<200_000_000
assert ref.times[-1]>m.epoch+346_320_000_000
pins=read(W/'runs'/old['on']/'prepared/protected_manifest.json')
dependency_attempt=E/'eth1_split_imu_live_pair_cpv95_20261103_attempt1'
assert read(dependency_attempt/'closeout.json')['passed'] and read(dependency_attempt/'failed.json')['stage']=='container_adapter_import'
assert not (dependency_attempt/'capture_off_process.json').exists()
for f in dependency_attempt.rglob('*'):
    if f.is_file() and '__pycache__' not in f.parts:pins[str(f)]=sha(f)
for p,digest in read(W/'runs'/dependency_attempt.name/'prepared/protected_manifest.json').items():assert sha(p)==digest;pins[p]=digest
failed=E/'eth1_split_imu_live_pair_cpv95_20261103'
assert read(failed/'closeout.json')['passed'] and read(failed/'failed.json')['stage']=='start_containers'
assert not (failed/'capture_off_process.json').exists()
for f in failed.rglob('*'):
    if f.is_file() and '__pycache__' not in f.parts:pins[str(f)]=sha(f)
prior_prepared=W/'runs'/failed.name/'prepared'
assert not any(prior_prepared.glob('full_*_source')) and not any(prior_prepared.glob('full_*_receiver'))
for p,digest in read(prior_prepared/'protected_manifest.json').items():assert sha(p)==digest;pins[p]=digest
terminal={}
for name in (*old.values(),'eig2_symmetric_persistent_full_cpv84_20261101_attempt1'):
    done=read(E/name/'complete.json');assert done['passed'];terminal[name]=len(done['output_sha256'])
    pins.update(done['output_sha256']);pins[str(E/name/'complete.json')]=sha(E/name/'complete.json')
for p,digest in pins.items():assert sha(p)==digest,p
audit=read(P/'input_audit.json')
for entry in audit['bag_files'].values():
    path=Path(entry['path']);st=path.stat()
    assert st.st_size==entry['bytes'] and st.st_mtime_ns==entry['mtime_ns'],entry
    assert sha(path)==entry['sha256'],str(path);pins[str(path)]=entry['sha256']
assert sha(audit['tf']['path'])==audit['tf']['sha256']
pins[audit['tf']['path']]=audit['tf']['sha256']
for p,digest in m.a.HASHES.items():pins[p]=digest
for mod in (m,m.b,m.d,m.d.m,m.d.m.c,m.d.m.c.b,m.a,m.a.g,m.d.u,m.d.d39,m.d.d39.v):pins[str(Path(mod.__file__).resolve())]=sha(mod.__file__)
for p,digest in read(H/'live_adapter_test.json')['input_sha256'].items():assert sha(p)==digest;pins[p]=digest
for root in (P,H):
    for f in root.rglob('*'):
        if f.is_file() and '__pycache__' not in f.parts and f.name not in ('REPORT.txt','protected_manifest.json','status.json'):
            pins[str(f.resolve())]=sha(f)
save(P/'protected_manifest.json',pins)
hesai=call(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'])
allocated={}
for mode,name in old.items():allocated[mode]=int(call(['du','-s','-B1',str(W/'runs'/name/'prepared'/(mode+'_source'))]).split()[0])
receiver=int(call(['du','-s','-B1',str(W/'runs/eig2_symmetric_persistent_full_cpv84_20261101_attempt1/prepared/full_on_receiver')]).split()[0])
forecast=int(1.5*(sum(allocated.values())+2*receiver*346.32/355.02))+512*2**20
required=max(10*2**30,forecast+5*2**30);free=shutil.disk_usage(P).free;assert free>required
storage=dict(passed=True,source_allocated_bytes=allocated,receiver_history_bytes=receiver,forecast_bytes=forecast,
    required_free_bytes=required,free_bytes=free,runtime_floor_bytes=5*2**30,basis='1.5x full source pair plus scaled two CP84 receivers,512MiB reports,5GiB reserve; initial>=10GiB')
save(H/'storage_preflight.json',storage)
data=dict(passed=True,prepared_path=str(P),protected_files=len(pins),syntax_files=len(syntax),profile_parity=parity,
    terminal_hash_counts=terminal,hesai=hesai,eof_boundary_tests=tests,reference_rows=len(ref.times),
    live_frame_test=str(H/'live_adapter_test.json'),fresh_pair=True,source_captures_started=0,receiver_captures_started=0,
    prior_infrastructure_failures_preserved=[str(failed),str(dependency_attempt)],
    repair='bounded startup wait; dependency-free equivalent adjoint adapter; fresh paths and ports only')
save(P/'preflight.json',data);save(H/'prepared.json',data)
print(json.dumps(dict(data,storage=storage),indent=2))
