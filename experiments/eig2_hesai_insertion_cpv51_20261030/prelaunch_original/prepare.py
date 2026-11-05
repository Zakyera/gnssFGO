"""Fresh short CP-V49-derived harness; passive CP-V51 observer only."""
import ast,copy,csv,difflib,hashlib,importlib.util,json,shutil,socket,subprocess,sys
from pathlib import Path
import xml.etree.ElementTree as ET
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores,get_typestore
sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent;R=H.parents[1];W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
S=W/'runs/eig2_hesai_refresh_full_cpv49_20261030/prepared';K=W/'runs'/H.name
P=K/'prepared';V=Path('/home/yeranis/results')/H.name
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
assert not P.exists() and not V.exists() and not (H/'prepared.json').exists()
gate=json.loads((H/'build_passed.json').read_text());assert gate['passed']
assert json.loads((R/'experiments/eig2_hesai_refresh_full_cpv49_20261030/complete.json').read_text())['passed']
with socket.socket() as sock:sock.bind(('127.0.0.1',11551))
required=8*1024**3;free=shutil.disk_usage(W).free
forecast=dict(passed=free>required,current_free_bytes=free,required_free_bytes=required,
 launch_reserve_bytes=required,runtime_reserve_bytes=5*1024**3,
 reason='10s pair plus 120 native snapshots; CP-V44 15 snapshots occupy 51MiB')
save(H/'storage_preflight.json',forecast);assert forecast['passed']
pins=json.loads((S/'protected_manifest.json').read_text())
for p,want in pins.items():assert sha(p)==want,p
audit=json.loads((S/'input_audit.json').read_text());audit['parent_audit']=str(S/'input_audit.json')
start=audit['window']['start_ns'];end=start+10_000_000_000
store=get_typestore(Stores.ROS1_NOETIC);stereo={}
for kind,item in audit['bag_files'].items():
    bag=Path(item['path']);st=bag.stat()
    assert (st.st_size,st.st_mtime_ns)==(item['bytes'],item['mtime_ns']) and sha(bag)==item['sha256']
    pins[str(bag)]=item['sha256']
    with Reader(bag) as reader:
        for topic,meta in item['topics'].items():
            stamps=[];headers=[]
            for c,t,raw in reader.messages(connections=[c for c in reader.connections if c.topic==topic],start=start,stop=end):
                stamps.append(t)
                if kind=='alphasense':
                    m=store.deserialize_ros1(raw,c.msgtype);headers.append(m.header.stamp.sec*10**9+m.header.stamp.nanosec)
            assert stamps and all(b>a for a,b in zip(stamps,stamps[1:]))
            meta['selected']=dict(count=len(stamps),first_record_ns=stamps[0],last_record_ns=stamps[-1],
                                 maximum_record_gap_ns=max(b-a for a,b in zip(stamps,stamps[1:])))
            if kind=='alphasense':stereo[topic]=headers
assert len(stereo)==2 and len(next(iter(stereo.values())))==100 and list(stereo.values())[0]==list(stereo.values())[1]
assert audit['bag_files']['hesai']['topics']['/boxi/hesai/points']['selected']['count']==100
audit['window'].update(end_ns=end,duration_s=10.0,last_hesai_record_ns=audit['bag_files']['hesai']['topics']['/boxi/hesai/points']['selected']['last_record_ns'])
audit.update(stereo_pairs=100,frontend_min=97,frontend_max=100,stereo_headers_identical=True,
 diagnostic_prefix=True,source_parameters_changed=False)
P.mkdir(parents=True)
shutil.copytree(S/'input_profiles',P/'input_profiles')
shutil.copytree(S/'transport_tools',P/'transport_tools',ignore=shutil.ignore_patterns('__pycache__'))
shutil.copytree(K/'private_build',P/'private_build')
assert sha(P/'private_build/libkimera_vio.so')==gate['backend_sha256']
diff=[]
def rebase(s):
    for a,b in [(S.parent.name,H.name),('11549','11551'),('cpv49','cpv51'),('CP-V49','CP-V51'),
                ('355.02','10.0'),('353.02','8.0'),('3547','97'),('3550','100'),('3517','90'),
                ('750b8c8ffed61d186ec8f071d4f58f068c9b63baccefcc9fc37803d77a118f06',gate['backend_sha256'])]:s=s.replace(a,b)
    s=s.replace('full-usable10.0-second','diagnostic-prefix10.0-second')
    return s
files=['coverage_checks.py','experiment.py','expiry_audit.py','eof_boundary.py','full_off.json','full_on.json',
 'active_window.launch','active_window_on.launch','glim_profile.launch','glim_profile_on.launch',
 'grandtour_eth1_kimera_stereo.launch','kimera_vio_ros.launch','kimera_vio_ros_newer_college_2021_stereo.launch',
 'map_guard.py','receiver_cpk19.sh','run_case.py','run_case_on.py','source.env','source_runner.sh',
 'source_runner_on.sh','validate_candidate.py','validate_case.py','validate_local.py',
 'verify_effective.py','verify_effective_on.py','preflight_on.py','verify_hesai_startup.py','validate_hesai.py']
for name in files:
    old=(S/name).read_text();s=rebase(old)
    if name=='kimera_vio_ros.launch':
        marker='    <env name="CPV40_REFRESH" value="1"/>'
        assert s.count(marker)==1
        s=s.replace(marker,marker+'\n    <env name="CPV51_AUDIT_DIR" value="$(env ROS_LOG_DIR)/../checkpoints"/>')
    if name.startswith('verify_effective'):
        marker="assert env(pid)['CPV40_REFRESH']=='1' and env(pid)['CPV38_OFF_REFRESH']=='0'"
        assert s.count(marker)==1
        s=s.replace(marker,marker+"\nassert Path(env(pid)['CPV51_AUDIT_DIR']).resolve()==source/'checkpoints'\nassert 'CPV51_TEST_ALL' not in env(pid)\nassert 'CPV51_AUDIT_DIR' not in env(glim_pid)")
    with (P/name).open('x') as f:f.write(s)
    shutil.copymode(S/name,P/name)
    diff.extend(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=str(S/name),tofile=str(P/name)))
for dst in (P/'transport_tools').rglob('*.py'):
    old=dst.read_text();new=rebase(old)
    if old!=new:
        dst.write_text(new)
        diff.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(S/dst.relative_to(P)),tofile=str(dst)))
save(P/'input_audit.json',audit);shutil.copy2(H/'PROTOCOL.txt',P/'PROTOCOL.txt')
for p in (S/'input_profiles').rglob('*'):
    if p.is_file():assert sha(p)==sha(P/p.relative_to(S)),p
for mode in ('off','on'):
    a=json.loads((S/f'full_{mode}.json').read_text());b=json.loads((P/f'full_{mode}.json').read_text())
    for c in (a,b):
        for key in ('sequence','ros_master_uri','receiver_binary'):c.pop(key)
        for source in c['sources']:source.pop('session')
    assert a==b,'receiver model changed'
for root in (P,H):
    for p in root.rglob('*'):
        if not p.is_file():continue
        if p.suffix=='.py':ast.parse(p.read_text())
        elif p.suffix=='.launch':ET.parse(p)
        elif p.suffix in ('.env','.sh'):subprocess.run(['bash','-n',str(p)],check=True)
spec=importlib.util.spec_from_file_location('boundary',P/'eof_boundary.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
assert m.test_boundaries()=={'valid':1,'rejected':4}
with (H/'harness.diff').open('x') as f:f.write(''.join(diff))
for root in (P,):
    for p in root.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts:pins[str(p)]=sha(p)
for p in H.iterdir():
    if p.suffix=='.py' or p.name=='PROTOCOL.txt':pins[str(p)]=sha(p)
for p in K.iterdir():
    if p.is_file():pins[str(p)]=sha(p)
for p in (K/'include').rglob('*.h'):pins[str(p)]=sha(p)
save(P/'protected_manifest.json',pins);(P/'preflight').mkdir()
receipt=dict(passed=True,prepared=str(P),validation=str(V),window=audit['window'],
 kimera_backend_sha256=gate['backend_sha256'],profiles_byte_identical_to=str(S/'input_profiles'),
 observer_updates=[6,25],expected_snapshots_per_mode=60,receiver_model_unchanged=True,
 protected_files=len(pins),storage=forecast)
save(P/'preflight/validation.json',receipt);save(H/'prepared.json',receipt)
print(json.dumps(receipt,indent=2))
