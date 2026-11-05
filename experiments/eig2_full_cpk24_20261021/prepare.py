"""Mechanically reuse CP-K23 with EIG-2 identities, input window and count gates."""
from pathlib import Path
import ast, difflib, hashlib, json, shutil, socket, subprocess
import xml.etree.ElementTree as ET

H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
S=W/'runs/gnssfgo_eth1_awake_full_cpk23_20261021/prepared'
P=W/'runs/gnssfgo_eig2_full_cpk24_20261021/prepared'
V=Path('/home/yeranis/results/gnssfgo_eig2_full_cpk24_20261021')
H23=H.parent/'eth1_awake_full_cpk23_20261021'
E=W/'runs/gnssfgo_eig2_local_cov60_off_cpr4_20261019/prepared/input_profiles'

def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()

def common(s):
    replacements=[(S.parent.name,P.parent.name),('eth1_awake_full_cpk23_20261021',H.name),
        ('eth1_cpk23_full','eig2_cpk24_full'),('11473','11474'),('CP-K23','CP-K24'),
        ('ETH1-CPK23','EIG2-CPK24'),('ETH1','EIG-2'),('2024-10-01-11-29-55','2024-11-03-13-59-54'),
        ('1727775007.855396309','1730638816.563546034'),('1727775007','1730638815'),
        ('346.32','355.02'),('344.32','353.02'),('326.32','335.02'),('3430','3517'),
        ('bag0-355.02s','bag5.7-360.72s')]
    for old,new in replacements:s=s.replace(old,new)
    return s

assert not P.parent.exists() and not V.exists()
assert shutil.disk_usage(W).free>15*1024**3
with socket.socket() as sock:sock.bind(('127.0.0.1',11474))
manifest=json.loads((S/'protected_manifest.json').read_text())
for name,want in manifest.items():assert sha(name)==want,name
preflight=json.loads((H/'bag_preflight.json').read_text());assert preflight['passed']
for item in preflight['bag_files'].values():
    st=Path(item['path']).stat()
    assert (st.st_size,st.st_mtime_ns)==(item['bytes'],item['mtime_ns']),item['path']
P.mkdir(parents=True)
for folder in ('private_build','transport_tools'):
    shutil.copytree(S/folder,P/folder,symlinks=True,ignore=shutil.ignore_patterns('__pycache__'))
shutil.copytree(E,P/'input_profiles',symlinks=True)
files=['PROTOCOL.txt','coverage_checks.py','evaluate_fl4f.py','experiment.py','expiry_audit.py',
 'full_off.json','full_on.json','active_window.launch','active_window_on.launch',
 'glim_profile.launch','glim_profile_on.launch','grandtour_eth1_kimera_stereo.launch',
 'kimera_vio_ros.launch','kimera_vio_ros_newer_college_2021_stereo.launch','map_guard.py',
 'preflight_on.py','receiver_cpk19.sh','run_case.py','run_case_on.py','source.env',
 'source_runner.sh','source_runner_on.sh','validate_candidate.py','validate_case.py',
 'validate_local.py','verify_effective.py','verify_effective_on.py']
diff=[]
for name in files:
    old=(S/name).read_text();new=common(old)
    if name=='PROTOCOL.txt':new=(H/name).read_text()
    if name=='source.env':
        new=new.replace('CBSMS_BAG_START_SEC=0.0','CBSMS_BAG_START_SEC=5.7')
        new=new.replace('CBSMS_DATASET_DIR=/workspace/cbs_gtsam4.3/src/datasets/GrandTour\n',
                        'CBSMS_DATASET_DIR=/workspace/cbs_gtsam4.3/src/datasets/GrandTour/EIG-2\n')
    if name.startswith('source_runner'):
        new=new.replace('CBSMS_BAG_START_SEC:-0.0','CBSMS_BAG_START_SEC:-5.7')
        new=new.replace('"${bag_start_sec}" == 0.0','"${bag_start_sec}" == 5.7')
        new=new.replace('CBSMS_DATASET_DIR:-${project_root}/src/datasets/GrandTour}',
                        'CBSMS_DATASET_DIR:-${project_root}/src/datasets/GrandTour/EIG-2}')
    if name.startswith('verify_effective'):
        new=new.replace("['CBSMS_BAG_START_SEC']=='0.0'","['CBSMS_BAG_START_SEC']=='5.7'")
    if name=='coverage_checks.py':new=new.replace('3460','3547').replace('3463','3550')
    if name=='evaluate_fl4f.py':new=new.replace('GrandTour/2024-11-03','GrandTour/EIG-2/2024-11-03')
    (P/name).write_text(new);shutil.copymode(S/name,P/name)
    if old!=new:diff.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(S/name),tofile=str(P/name)))
for path in (P/'transport_tools').rglob('*.py'):path.write_text(common(path.read_text()))
for name in ('run_pair.py','summarize.py'):
    assert not (H/name).exists()
    old=(H23/name).read_text();new=common(old)
    if name=='run_pair.py':
        new=new.replace("'-eo','comm,args'","'-eo','pid,comm,args'")
        new=new.replace("line.split()[0] not in allowed","line.split()[1] not in allowed")
    (H/name).write_text(new)
    diff.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(H23/name),tofile=str(H/name)))
(H/'harness.diff').write_text(''.join(diff))
for folder in ('private_build','input_profiles'):
    for path in (S/folder).rglob('*'):
        if path.is_file():assert sha(path)==sha(P/path.relative_to(S)),path
for name in ('expiry_audit.py','map_guard.py'):assert sha(P/name)==sha(S/name),name
for name in ('full_off.json','full_on.json'):
    a=json.loads((S/name).read_text());b=json.loads((P/name).read_text())
    for c in (a,b):
        for k in ('sequence','ros_master_uri','receiver_binary'):c.pop(k)
        for source in c['sources']:source.pop('session')
    assert a==b,name
for path in [*P.rglob('*'),*H.glob('*.py')]:
    if path.suffix=='.py':ast.parse(path.read_text())
    elif path.suffix=='.launch':ET.parse(path)
    elif path.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(path)],check=True)
for base in (P,H):
    for path in base.rglob('*'):
        if path.is_file() and '__pycache__' not in path.parts and path.name!='REPORT.txt':manifest[str(path)]=sha(path)
(P/'protected_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
preflight.update(template=str(S),protected_files=len(manifest),model_identical=True,
    source_profiles_byte_identical=True,window_specific_count_gates=True,
    coverage_semantics_unchanged=True,sleep_prevention='required scoped systemd-inhibit',
    disk_free_bytes=shutil.disk_usage(W).free)
(P/'preflight').mkdir()
(P/'preflight/validation.json').write_text(json.dumps(preflight,indent=2)+'\n')
print(json.dumps(dict(passed=True,prepared=str(P),protected_files=len(manifest),window=preflight['window']),indent=2))
