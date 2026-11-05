"""Fresh bounded ON harness; identical sensors/model, tested private backend."""
import ast,difflib,hashlib,importlib.util,json,re,shutil,socket,struct,subprocess,sys
from pathlib import Path
import numpy as np
from rosbags.rosbag1 import Reader
import xml.etree.ElementTree as ET
sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent;R=H.parents[1];W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
S=W/'runs/gnssfgo_eig2_velodyne_full_cpv4_20261024/prepared';P=W/'runs'/H.name/'prepared'
K=W/'runs/eig2_on_checkpoint_cpv44_20261029';G=R/'experiments'/K.name
V=Path('/home/yeranis/results')/H.name
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
assert not P.exists() and not V.exists() and not (H/'prepared.json').exists()
assert shutil.disk_usage(W).free>8*1024**3
assert json.loads((G/'build_passed.json').read_text())['passed']
want=json.loads((G/'build_passed.json').read_text())['backend_sha256']
assert sha(K/'private_build/libkimera_vio.so')==want
assert json.loads((R/'experiments/eig2_off_refresh_full_cpv39_20261029/closeout.json').read_text())['passed']
oldH=R/'experiments/velodyne_full_campaign_cpv4_20261024/eig2'
assert json.loads((oldH/'off_validated.json').read_text())['passed']
assert json.loads((oldH/'closeout.json').read_text())['passed']
with socket.socket() as sock:sock.bind(('127.0.0.1',11544))
audit=json.loads((S/'input_audit.json').read_text());start=audit['window']['start_ns'];end=start+10_000_000_000
pins=json.loads((G/'protected_before.json').read_text())
for p,want_pin in pins.items():assert sha(p)==want_pin,p
selected={};headers={}
topics={'anymal_velodyne':['/anymal/velodyne/points'],'cpt7_imu':['/boxi/cpt7/imu'],
 'alphasense':['/boxi/alphasense/front_left/image_raw/compressed','/boxi/alphasense/front_right/image_raw/compressed'],
 'cpt7_ie_tc':['/boxi/inertial_explorer/tc/odometry']}
for kind,item in audit['bag_files'].items():
    bag=Path(item['path']);st=bag.stat();assert (st.st_size,st.st_mtime_ns)==(item['bytes'],item['mtime_ns'])
    assert sha(bag)==item['sha256'],bag;pins[str(bag)]=item['sha256']
    if kind not in topics:continue
    with Reader(bag) as reader:
        cs=[c for c in reader.connections if c.topic in topics[kind]]
        assert sorted(c.topic for c in cs)==sorted(topics[kind])
        for c in cs:
            ts=[x.time for x in reader.indexes[c.id] if start<=x.time<end]
            assert ts and np.all(np.diff(ts)>0)
            selected[c.topic]=dict(count=len(ts),first_ns=ts[0],last_ns=ts[-1],max_gap_ns=int(max(np.diff(ts))))
            headers[c.topic]=[]
        for c,t,raw in reader.messages(connections=cs,start=start,stop=end):
            _,sec,ns=struct.unpack_from('<III',raw);stamp=sec*10**9+ns
            assert abs(t-stamp)<50_000_000;headers[c.topic].append(stamp)
left,right=(headers[t] for t in topics['alphasense']);assert left==right and len(left)==100
imu=headers[topics['cpt7_imu'][0]];assert np.all(np.diff(imu)>0) and max(np.diff(imu))<30_000_000
audit['clouds']=[c for c in audit['clouds'] if start<=c['record_ns']<end]
assert len(audit['clouds'])==100
for c in audit['clouds']:c['captured_imu_bracketed']=imu[0]/1e9<=c['normalized_stamp_s'] and imu[-1]/1e9>=c['header_ns']/1e9+c['max_offset_s']
assert all(c['captured_imu_bracketed'] for c in audit['clouds'][1:])
audit.update(selected=selected,stereo_pairs=100,frontend_min=97,frontend_max=100,glim_min=80,
 imu=dict(first_header_ns=imu[0],last_header_ns=imu[-1],count=len(imu),max_gap_ns=int(max(np.diff(imu)))),
 bounded_from_hashed_full_audit=True)
audit['window'].update(end_ns=end,duration_s=10.0,last_included_scan_ns=audit['clouds'][-1]['header_ns'])
P.mkdir(parents=True)
for folder in ('input_profiles','transport_tools'):
    shutil.copytree(S/folder,P/folder,ignore=shutil.ignore_patterns('__pycache__'))
shutil.copytree(K/'private_build',P/'private_build')
diff=[]
def rebase(s):
    return s.replace(S.parent.name,H.name).replace('11482','11544').replace('eig2_cpv4_velodyne','eig2_cpv44_velodyne')
def bounds(s):
    for a,b in [('355.02','10.0'),('353.02','8.0'),('3547','97'),('3550','100'),('3530','80')]:
        s=re.sub(r'(?<![\w.])'+re.escape(a)+r'(?![\w.])',b,s)
    return s.replace('full-usable355.02','bounded10.0').replace('frozen full EIG-2 window','bounded EIG-2 window')
def generate(src,dst,s):
    with dst.open('x') as f:f.write(s)
    shutil.copymode(src,dst)
    diff.extend(difflib.unified_diff(src.read_text().splitlines(True),s.splitlines(True),fromfile=str(src),tofile=str(dst)))
files=['coverage_checks.py','experiment.py','expiry_audit.py','eof_boundary.py','full_on.json',
 'active_window_on.launch','glim_profile_on.launch','grandtour_eth1_kimera_stereo.launch',
 'kimera_vio_ros.launch','kimera_vio_ros_newer_college_2021_stereo.launch','map_guard.py',
 'receiver_cpk19.sh','run_case_on.py','source.env','source_runner_on.sh','validate_candidate.py',
 'validate_case.py','validate_local.py','verify_effective_on.py','validate_velodyne.py','verify_velodyne_startup.py']
for name in files:
    s=bounds(rebase((S/name).read_text()))
    if name=='full_on.json':
        value=json.loads(s);value['sequence']=f'EIG-2 CP-V44 Velodyne bounded ON integration {start}..{end}ns; not a matched accuracy pair'
        s=json.dumps(value,indent=2)+'\n'
    if name=='kimera_vio_ros.launch':
        needle='<env name="CPK9_PRIOR_REFRESH" value="1"/>';assert s.count(needle)==1
        s=s.replace(needle,needle+'\n    <env name="CPV40_REFRESH" value="1"/>\n    <env name="CPV38_OFF_REFRESH" value="0"/>')
        s=s.replace(needle,needle+'\n    <env name="CPV44_AUDIT_DIR" value="/workspace/cbs_gtsam4.3/runs/eig2_on_checkpoint_cpv44_20261029/checkpoints"/>')
    if name in ('verify_effective_on.py','validate_candidate.py'):
        assert 'be637ad745f73bf12596d3bf06754f56ed7bcadfdb8b4792b94c4fcfee62aad9' in s
        s=s.replace('be637ad745f73bf12596d3bf06754f56ed7bcadfdb8b4792b94c4fcfee62aad9',want)
    if name=='verify_effective_on.py':
        needle="assert env(pid)['CPK9_PRIOR_REFRESH']=='1'";assert s.count(needle)==1
        s=s.replace(needle,needle+"\nassert env(pid)['CPV40_REFRESH']=='1' and env(pid)['CPV38_OFF_REFRESH']=='0'\nassert 'CPV40_REFRESH' not in env(glim_pid)")
        s=s.replace(needle,needle+"\nassert env(pid)['CPV44_AUDIT_DIR']=='/workspace/cbs_gtsam4.3/runs/eig2_on_checkpoint_cpv44_20261029/checkpoints'\nassert 'CPV44_AUDIT_DIR' not in env(glim_pid)")
        s=s.replace("('CPK9_PRIOR_REFRESH','FL4Q_ENABLE','FL4M_RETIREMENT')","('CPK9_PRIOR_REFRESH','FL4Q_ENABLE','FL4M_RETIREMENT','CPV40_REFRESH','CPV38_OFF_REFRESH')")
    if name=='validate_candidate.py':
        needle='"CPK9_PRIOR_REFRESH": "1", "FL4Q_ENABLE": "0", "FL4M_RETIREMENT": "remove"}'
        assert needle in s;s=s.replace(needle,needle[:-1]+', "CPV40_REFRESH": "1", "CPV38_OFF_REFRESH": "0"}')
        s=s.replace('("FL4Q", "CPK9 ",','("FL4Q", "CPK9 ", "CPV40 ",')
    if name=='run_case_on.py':
        s=s.replace("'CPK9 ','FL4Q'","'CPK9 ','CPV40 ','FL4Q'")
    generate(S/name,P/name,s)
for dst in (P/'transport_tools').rglob('*.py'):
    old=dst.read_text();new=rebase(old)
    if old!=new:
        dst.write_text(new)
        diff.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(S/dst.relative_to(P)),tofile=str(dst)))
save(P/'input_audit.json',audit);shutil.copy2(H/'PROTOCOL.txt',P/'PROTOCOL.txt')
for folder in ('input_profiles',):
    for p in (S/folder).rglob('*'):
        if p.is_file():assert sha(p)==sha(P/p.relative_to(S)),p
a=json.loads((S/'full_on.json').read_text());b=json.loads((P/'full_on.json').read_text())
for c in (a,b):
    for key in ('sequence','ros_master_uri','receiver_binary'):c.pop(key)
    for source in c['sources']:source.pop('session')
assert a==b,'receiver numerical settings changed'
for p in P.rglob('*'):
    if not p.is_file():continue
    if p.suffix=='.py':ast.parse(p.read_text())
    elif p.suffix=='.launch':ET.parse(p)
    elif p.suffix in ('.env','.sh'):subprocess.run(['bash','-n',str(p)],check=True)
spec=importlib.util.spec_from_file_location('boundary',P/'eof_boundary.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
assert m.test_boundaries()=={'valid':1,'rejected':4}
with (H/'harness.diff').open('x') as f:f.write(''.join(diff))
for root in (K,P):
    for p in root.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts:pins[str(p)]=sha(p)
save(P/'protected_manifest.json',pins);(P/'preflight').mkdir()
receipt=dict(passed=True,scope='ONE bounded10s ON engineering gate, not full/matched accuracy',prepared=str(P),
    window=audit['window'],stereo_pairs=100,kimera_backend=want,profiles_unchanged=True,receiver_model_unchanged=True,
    guarded_by=[str(G/'build_passed.json'),str(oldH/'off_validated.json'),str(R/'experiments/eig2_off_refresh_full_cpv39_20261029/closeout.json')],
    protected_files=len(pins),free_bytes=shutil.disk_usage(W).free)
save(P/'preflight/validation.json',receipt);save(P/'preflight_on.json',receipt);save(H/'prepared.json',receipt)
print(json.dumps(receipt,indent=2))
