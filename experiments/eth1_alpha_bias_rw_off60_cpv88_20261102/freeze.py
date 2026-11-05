"""One-time admission and freeze; never starts estimators."""
import ast,difflib,hashlib,json,re,shutil,socket,subprocess
from pathlib import Path
import xml.etree.ElementTree as ET
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name/'prepared';H85=E/'eth1_split_imu_off60_cpv85_20261101_attempt1';Q85=W/'runs'/H85.name/'prepared'
H86=E/'eth1_cpt7_control_off60_cpv86_20261102';Q86=W/'runs'/H86.name/'prepared'
H87=E/'split_imu_diagnostic_cpv87_20261102'
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
assert not (H/'prepared.json').exists() and not (P/'off_source').exists()
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert states=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
with socket.socket() as s:s.bind(('127.0.0.1',11589))
free=shutil.disk_usage(P).free;assert free>10*1024**3,free
pins=read(Q86/'protected_manifest.json')
for p,d in pins.items():assert sha(p)==d,p
for root,file in [(H85,'report_completion_1/complete.json'),(H86,'complete.json')]:
    terminal=read(root/file)
    for p,d in terminal['output_sha256'].items():assert sha(p)==d,p
    assert read(root/('report_completion_1/post_closeout.json' if root==H85 else 'post_closeout.json'))['passed']
changes={'accelerometer_random_walk':0.0043,'gyroscope_random_walk':0.000266}
profile_files=[];diffs=[]
for folder in ('input_profiles','private_build'):
    source=Q85/folder;dest=P/folder
    oldfiles={str(p.relative_to(source)) for p in source.rglob('*') if p.is_file()}
    newfiles={str(p.relative_to(dest)) for p in dest.rglob('*') if p.is_file()}
    assert oldfiles==newfiles
    for name in sorted(oldfiles):
        op=source/name;np=dest/name
        if name=='kimera_profile/ImuParams.yaml' and folder=='input_profiles':
            old=op.read_text();new=np.read_text();expected=old
            for field,value in changes.items():
                expected,n=re.subn(r'^'+field+r': .*$',field+': '+str(value),expected,flags=re.M);assert n==1
            assert new==expected
            diffs.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(op),tofile=str(np)))
        else:assert op.read_bytes()==np.read_bytes(),name
        profile_files.append(dict(path=str(np),sha256=sha(np),baseline=str(op),baseline_sha256=sha(op)))
save(H/'profile_parity.json',dict(passed=True,exactly_two_changed_scalars=changes,all_other_profile_and_binary_bytes_identical=True,files=profile_files))
with (H/'profile_only.diff').open('x') as f:f.writelines(diffs)
audit=read(Q85/'input_audit.json');audit['baseline_input_audit']=str(Q85/'input_audit.json')
audit['noise'].update(changes)
audit['bias_random_walk_override']=dict(scope='CP-V88 hypothesis test only; NOT calibrated Alpha noise',values=changes)
save(P/'input_audit.json',audit)
assert (P/'expected_headers.json').read_bytes()==(Q86/'expected_headers.json').read_bytes()
assert not (P/'coverage_control.py').exists();shutil.copy2(H/'coverage_control.py',P/'coverage_control.py')
for root in (H85,Q85,H86,Q86,H87):
    for p in root.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts:pins[str(p)]=sha(p)
for root in (H,P):
    for p in root.rglob('*'):
        if not p.is_file() or '__pycache__' in p.parts:continue
        if p.suffix=='.py':ast.parse(p.read_text())
        if p.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(p)],check=True)
        if p.suffix=='.launch':ET.parse(p)
        if p.name!='REPORT.txt':pins[str(p)]=sha(p)
assert sha(P/'private_build/libkimera_vio.so')=='b7d6829da6c2c5a0edbe3d7092ba109fa29d24a60b2bb5395b05dd629fa1cee8'
hesai=subprocess.check_output(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
save(P/'protected_manifest.json',pins)
save(H/'prepared.json',dict(passed=True,prepared=str(P),containers=states,protected_files=len(pins),free_bytes=free,
    required_free_bytes=10*1024**3,runtime_floor_bytes=5*1024**3,hesai=hesai,scope='single ETH1 Alpha CBS-OFF60 bias-RW-only ablation'))
print('ADMITTED',len(pins),'protected files;',hesai.strip(),'free',free)
