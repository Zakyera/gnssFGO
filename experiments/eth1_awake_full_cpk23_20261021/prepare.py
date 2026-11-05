"""Reuse frozen CP-K22 harness, only fresh identities and awake orchestration."""
from pathlib import Path
import ast
import difflib
import hashlib
import json
import shutil
import socket
import subprocess
import xml.etree.ElementTree as ET

H = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
S = W/'runs/gnssfgo_eth1_keyframe_finalization_full_cpk22_20261021/prepared'
P = W/'runs/gnssfgo_eth1_awake_full_cpk23_20261021/prepared'
V = Path('/home/yeranis/results/gnssfgo_eth1_awake_full_cpk23_20261021')
H22 = H.parent/'eth1_keyframe_finalization_full_cpk22_20261021'

def sha(path):
    with Path(path).open('rb') as f: return hashlib.file_digest(f,'sha256').hexdigest()

def common(s):
    return s.replace(S.parent.name,P.parent.name).replace('eth1_cpk22_full','eth1_cpk23_full').replace('11472','11473').replace('CP-K22','CP-K23')

assert not P.parent.exists() and not V.exists()
assert shutil.disk_usage(W).free > 15*1024**3
with socket.socket() as sock: sock.bind(('127.0.0.1',11473))
manifest = json.loads((S/'protected_manifest.json').read_text())
for path,want in manifest.items(): assert sha(path)==want,path
preflight = json.loads((S/'preflight/validation.json').read_text())
assert preflight['passed']
for item in preflight['bag_files'].values():
    stat = Path(item['path']).stat()
    assert stat.st_size == item['bytes'] and stat.st_mtime_ns == item['mtime_ns'], item['path']
P.mkdir(parents=True)
for folder in ('private_build','input_profiles','transport_tools'):
    shutil.copytree(S/folder,P/folder,symlinks=True,ignore=shutil.ignore_patterns('__pycache__'))
diff = []
for path in S.iterdir():
    if not path.is_file() or path.name in ('protected_manifest.json','controller.lock'): continue
    assert path.suffix in ('.py','.sh','.env','.launch','.json','.txt'), path
    if path.name in ('coverage_checks.py','expiry_audit.py'):
        shutil.copy2(path,P/path.name); continue
    old = path.read_text(); new = common(old)
    if path.name == 'PROTOCOL.txt': new = (H/'PROTOCOL.txt').read_text()
    (P/path.name).write_text(new); shutil.copymode(path,P/path.name)
    if new != old: diff.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(path),tofile=str(P/path.name)))
for path in (P/'transport_tools').rglob('*.py'): path.write_text(common(path.read_text()))
(H/'summarize.py').write_text(common((H22/'summarize.py').read_text()))
(H/'harness.diff').write_text(''.join(diff))
for folder in ('private_build','input_profiles'):
    for path in (S/folder).rglob('*'):
        if path.is_file(): assert sha(path)==sha(P/path.relative_to(S)),path
assert sha(P/'coverage_checks.py')==sha(S/'coverage_checks.py')
assert sha(P/'expiry_audit.py')==sha(S/'expiry_audit.py')
for name in ('full_off.json','full_on.json'):
    original=json.loads((S/name).read_text()); candidate=json.loads((P/name).read_text())
    for k in ('sequence','ros_master_uri','receiver_binary'): original.pop(k);candidate.pop(k)
    for c in (original,candidate):
        for source in c['sources']: source.pop('session')
    assert original==candidate,name
for path in [*P.rglob('*'),*H.glob('*.py')]:
    if path.suffix=='.py': ast.parse(path.read_text())
    elif path.suffix=='.launch': ET.parse(path)
    elif path.suffix in ('.sh','.env'): subprocess.run(['bash','-n',str(path)],check=True)
for base in (P,H):
    for path in base.rglob('*'):
        if path.is_file() and '__pycache__' not in path.parts and path.name!='REPORT.txt': manifest[str(path)]=sha(path)
(P/'protected_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
preflight.update(template=str(S),protected_files=len(manifest),coverage_checks_identical=True,
    saved_data_tests_reused=True,sleep_prevention='required scoped systemd-inhibit',
    bag_identity_metadata_unchanged=True)
(P/'preflight').mkdir()
(P/'preflight/validation.json').write_text(json.dumps(preflight,indent=2)+'\n')
print(json.dumps(dict(passed=True,prepared=str(P),protected_files=len(manifest),
                     unchanged_model_profiles_coverage=True),indent=2))
