"""Admission only. Pin the new harness and prior immutable evidence."""
import ast,hashlib,json,shutil,socket,subprocess
from pathlib import Path
import xml.etree.ElementTree as ET
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name/'prepared';OLD=E/'eth1_split_imu_off60_cpv85_20261101_attempt1';Q=W/'runs'/OLD.name/'prepared'
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2)
assert not (H/'prepared.json').exists() and not (P/'off_source').exists()
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert states=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
with socket.socket() as s:s.bind(('127.0.0.1',11588))
free=shutil.disk_usage(P).free;assert free>10*1024**3,free
pins=read(Q/'protected_manifest.json')
for p,d in pins.items():assert sha(p)==d,p
terminal=read(OLD/'report_completion_1/complete.json');assert terminal['report_complete'] and terminal['endpoint_audit_passed']
for p,d in terminal['output_sha256'].items():assert sha(p)==d,p
for root in (OLD,Q):
    for p in root.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts:pins[str(p)]=sha(p)
for root in (H,P):
    for p in root.rglob('*'):
        if not p.is_file() or '__pycache__' in p.parts:continue
        if p.suffix=='.py':ast.parse(p.read_text())
        if p.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(p)],check=True)
        if p.suffix=='.launch':ET.parse(p)
        if p.name!='REPORT.txt':pins[str(p)]=sha(p)
assert sha(P/'private_build/libkimera_vio.so')==sha(Q/'private_build/libkimera_vio.so')=='b7d6829da6c2c5a0edbe3d7092ba109fa29d24a60b2bb5395b05dd629fa1cee8'
hesai=subprocess.check_output(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
save(P/'protected_manifest.json',pins)
save(H/'prepared.json',dict(passed=True,prepared=str(P),containers=states,protected_files=len(pins),free_bytes=free,
     required_free_bytes=10*1024**3,runtime_floor_bytes=5*1024**3,hesai=hesai,scope='single matched ETH1 CPT7 CBS-OFF60 control'))
print('ADMITTED',len(pins),'protected files;',hesai.strip(),'free',free)
