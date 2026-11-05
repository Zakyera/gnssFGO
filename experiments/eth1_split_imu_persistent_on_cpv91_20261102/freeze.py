"""Read-only admission checks and fresh manifests; never launches sources."""
import ast,difflib,hashlib,json,shutil,socket,subprocess
from pathlib import Path
import xml.etree.ElementTree as ET
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name/'prepared';OLD=E/'eth1_alpha_bias_rw_full_off_cpv89_20261102';Q=W/'runs'/OLD.name/'prepared'
CP90=E/'eth1_ap20_saved_comparison_cpv90_20261102'
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
assert not (H/'prepared.json').exists() and not (P/'on_source').exists()
assert read(OLD/'post_closeout.json')['passed'] and read(H/'frame_preflight.json')['passed']
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert states=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
with socket.socket() as s:s.bind(('127.0.0.1',11591))
past_bytes=sum(p.stat().st_size for p in (Q/'off_source').rglob('*') if p.is_file())
other=W/'runs/eig2_symmetric_persistent_full_cpv84_20261101_attempt1/prepared/full_on_source'
on_bytes=sum(p.stat().st_size for p in other.rglob('*') if p.is_file())
forecast=3*max(past_bytes,on_bytes)+512*1024**2
free=shutil.disk_usage(P).free;required=max(10*1024**3,forecast+5*1024**3);assert free>required,(free,required)
pins=read(Q/'protected_manifest.json')
for p,d in pins.items():assert sha(p)==d,p
for root in (OLD,CP90):
    terminal=read(root/'complete.json');assert terminal['passed']
    for p,d in terminal['output_sha256'].items():assert sha(p)==d,p
for p,d in read(H/'frame_preflight.json')['input_sha256'].items():assert sha(p)==d,p;pins[p]=d
parity=[]
for name in ('input_profiles','private_build'):
    a=Q/name;b=P/name
    af={str(p.relative_to(a)) for p in a.rglob('*') if p.is_file()};bf={str(p.relative_to(b)) for p in b.rglob('*') if p.is_file()};assert af==bf
    for rel in sorted(af):
        assert sha(a/rel)==sha(b/rel),rel
        parity.append(dict(path=str(b/rel),baseline=str(a/rel),sha256=sha(b/rel)))
save(H/'profile_parity.json',dict(passed=True,baseline=str(Q),all_profiles_and_private_binaries_identical=True,files=parity))
audit=read(P/'input_audit.json');assert audit['passed'] and audit['duration_s']==346.32 and audit['prefix_CP88_exact']
assert audit['noise']['accelerometer_random_walk']==.0043 and audit['noise']['gyroscope_random_walk']==.000266
for name in ('input_audit.json','expected_headers.json'):assert sha(P/name)==sha(Q/name)
for v in audit['bag_files'].values():assert sha(v['path'])==v['sha256'];pins[v['path']]=v['sha256']
for v in (audit['tf'],audit['historical_pair_count']):assert sha(v['path'])==v['sha256'];pins[v['path']]=v['sha256']
assert not (P/'coverage_control.py').exists();shutil.copy2(H/'coverage_control.py',P/'coverage_control.py')
assert (P/'source_runner_on.sh').stat().st_mode&0o777==0o755
changes=[]
for p in sorted(P.iterdir()):
    if p.suffix not in ('.py','.sh','.env','.launch') or not (Q/p.name).exists():continue
    changes.extend(difflib.unified_diff((Q/p.name).read_text().splitlines(True),p.read_text().splitlines(True),fromfile=str(Q/p.name),tofile=str(p)))
with (H/'harness_final.diff').open('x') as f:f.writelines(changes)
for root in (OLD,Q,CP90):
    for p in root.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts:pins[str(p)]=sha(p)
for root in (H,P):
    for p in root.rglob('*'):
        if not p.is_file() or '__pycache__' in p.parts:continue
        if p.suffix=='.py':ast.parse(p.read_text())
        if p.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(p)],check=True)
        if p.suffix=='.launch':ET.parse(p)
        if p.name!='REPORT.txt':pins[str(p)]=sha(p)
hesai=subprocess.check_output(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
save(P/'protected_manifest.json',pins)
save(H/'prepared.json',dict(passed=True,prepared=str(P),containers=states,protected_files=len(pins),free_bytes=free,
    required_free_bytes=required,runtime_floor_bytes=5*1024**3,storage_forecast_bytes=forecast,
    storage_forecast_basis=dict(CP89_OFF_source_bytes=past_bytes,CP84_ON_source_bytes=on_bytes,headroom_multiplier=3,extra_bytes=512*1024**2),
    hesai=hesai,scope='single full ETH1 split-IMU BOTH-persistent CBS-ON; source-only'))
print('ADMITTED',len(pins),'protected files;',hesai.strip(),'free',free,'forecast',forecast,'required',required)
