"""Admission/freeze only; no source launch or scoring."""
import ast,hashlib,json,shutil,socket,subprocess
from pathlib import Path
import xml.etree.ElementTree as ET
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name/'prepared';OLD=E/'eth1_alpha_bias_rw_off60_cpv88_20261102';Q=W/'runs'/OLD.name/'prepared'
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
assert not (H/'prepared.json').exists() and not (P/'off_source').exists()
assert read(OLD/'post_closeout.json')['passed']
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert states=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
with socket.socket() as s:s.bind(('127.0.0.1',11590))
past_bytes=sum(p.stat().st_size for p in (Q/'off_source').rglob('*') if p.is_file())
forecast=int(past_bytes*(346.32/60)*3)+512*1024**2
free=shutil.disk_usage(P).free;required=max(10*1024**3,forecast+5*1024**3);assert free>required,(free,required)
pins=read(Q/'protected_manifest.json')
for p,d in pins.items():assert sha(p)==d,p
for p,d in read(OLD/'complete.json')['output_sha256'].items():assert sha(p)==d,p
parity=[]
for name in ('input_profiles','private_build'):
    a=Q/name;b=P/name
    af={str(p.relative_to(a)) for p in a.rglob('*') if p.is_file()}
    bf={str(p.relative_to(b)) for p in b.rglob('*') if p.is_file()};assert af==bf
    for rel in sorted(af):
        assert sha(a/rel)==sha(b/rel),rel
        parity.append(dict(path=str(b/rel),baseline=str(a/rel),sha256=sha(b/rel)))
save(H/'profile_parity.json',dict(passed=True,baseline=str(Q),all_profiles_and_private_binaries_identical=True,files=parity))
audit=read(P/'input_audit.json');assert audit['passed'] and audit['duration_s']==346.32 and audit['prefix_CP88_exact']
assert audit['noise']['accelerometer_random_walk']==.0043 and audit['noise']['gyroscope_random_walk']==.000266
for v in audit['bag_files'].values():assert sha(v['path'])==v['sha256'];pins[v['path']]=v['sha256']
for v in (audit['tf'],audit['historical_pair_count']):assert sha(v['path'])==v['sha256'];pins[v['path']]=v['sha256']
assert not (P/'coverage_control.py').exists();shutil.copy2(H/'coverage_control.py',P/'coverage_control.py')
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
hesai=subprocess.check_output(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
save(P/'protected_manifest.json',pins)
save(H/'prepared.json',dict(passed=True,prepared=str(P),containers=states,protected_files=len(pins),free_bytes=free,
    required_free_bytes=required,runtime_floor_bytes=5*1024**3,storage_forecast_bytes=forecast,
    storage_forecast_basis=dict(CP88_source_bytes=past_bytes,duration_ratio=346.32/60,headroom_multiplier=3,extra_bytes=512*1024**2),
    hesai=hesai,scope='single full ETH1 Alpha CBS-OFF346.32 validation'))
print('ADMITTED',len(pins),'protected files;',hesai.strip(),'free',free,'forecast',forecast,'required',required)
