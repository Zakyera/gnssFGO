"""Fail-closed host admission; no containers or estimators are started."""
import ast,difflib,hashlib,json,shutil,socket,subprocess
from pathlib import Path
import xml.etree.ElementTree as ET
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name/'prepared';OLD=E/'eig2_symmetric_persistent_full_cpv84_20261101_attempt1'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(p.read_text())
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
assert not (H/'prepared.json').exists() and not (P/'off_source').exists()
assert read(P/'input_audit.json')['passed']
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert states=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
with socket.socket() as s:s.bind(('127.0.0.1',11586))
required=10*1024**3;free=shutil.disk_usage(P).free
assert free>required,('storage guard',free,required)
pins=read(W/'runs'/OLD.name/'prepared/protected_manifest.json')
pins.update(read(OLD/'complete.json')['output_sha256'])
pins[str(OLD/'complete.json')]=sha(OLD/'complete.json')
pins[str(OLD/'post_closeout.json')]=sha(OLD/'post_closeout.json')
assert read(OLD/'complete.json')['passed'] and read(OLD/'post_closeout.json')['passed']
for name,v in read(P/'input_audit.json')['bag_files'].items():pins[v['path']]=v['sha256']
for p,d in pins.items():assert sha(p)==d,p
assert sha(P/'private_build/libkimera_vio.so')=='b7d6829da6c2c5a0edbe3d7092ba109fa29d24a60b2bb5395b05dd629fa1cee8'
diff=[];base=W/'runs'/OLD.name/'prepared'
for root in (P,H):
    for p in root.rglob('*'):
        if not p.is_file() or '__pycache__' in p.parts:continue
        if p.suffix=='.py':ast.parse(p.read_text())
        if p.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(p)],check=True)
        if p.suffix=='.launch':ET.parse(p)
        if root==P and p.suffix in ('.sh','.env','.launch','.py') and (base/p.relative_to(P)).is_file():
            old=base/p.relative_to(P)
            diff.extend(difflib.unified_diff(old.read_text().splitlines(True),p.read_text().splitlines(True),fromfile=str(old),tofile=str(p)))
        if p.name!='REPORT.txt':pins[str(p)]=sha(p)
with (H/'harness.diff').open('x') as f:f.write(''.join(diff))
hesai=subprocess.check_output(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
save(P/'protected_manifest.json',pins)
save(H/'prepared.json',dict(passed=True,prepared=str(P),protected_files=len(pins),containers=states,
    free_bytes=free,required_free_bytes=required,runtime_floor_bytes=5*1024**3,hesai=hesai,
    scope='single source-only ETH1 split-IMU CBS-OFF60',receiver=False,CBS_ON=False))
print('PREPARED',len(pins),'pins;',hesai.strip(),'free',free)
