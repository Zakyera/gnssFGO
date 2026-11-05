"""Read-only preparation gates and exclusive manifests; never starts containers."""
import ast,datetime,difflib,hashlib,importlib.util,json,shutil,socket,subprocess,sys
from pathlib import Path
import xml.etree.ElementTree as ET
sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name/'prepared';S=W/'runs/eig2_hesai_refresh_full_cpv49_20261030/prepared'
V=Path('/home/yeranis/results')/H.name;OLD=E/'persistent_native_full_cpv78_20261101'
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def read(p):return json.loads(p.read_text())
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
def call(args):return subprocess.check_output(args,text=True)
assert not (H/'prepared.json').exists() and not V.exists()
assert not any(P.glob('full_*_source')) and not any(P.glob('full_*_control'))
assert read(OLD/'complete.json')['passed'] and read(OLD/'post_closeout.json')['passed']
assert read(E/'eig2_hesai_refresh_full_cpv49_20261030/complete.json')['passed']
with socket.socket() as sock:sock.bind(('127.0.0.1',11583))
containers=call(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'])
assert containers=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
source_bytes=sum(int(call(['du','-sB1',str(S/f'full_on_{part}')]).split()[0]) for part in ('source','receiver','control'))
required=int(source_bytes*(60.0/355.02)*1.5)+8*1024**3+512*1024**2
forecast=dict(passed=shutil.disk_usage(W).free>required,current_free_bytes=shutil.disk_usage(W).free,
    required_free_bytes=required,original_live_on_bytes=source_bytes,forecast_margin=1.5, duration_ratio=60.0/355.02,
    launch_reserve_bytes=8*1024**3,runtime_reserve_bytes=5*1024**3)
assert forecast['passed'],forecast
pins=read(OLD/'protected_before.json')
CP82=E/'glim_persistent_compat_cpv82_20261101'
assert read(CP82/'complete.json')['passed']
pins.update(read(CP82/'complete.json')['output_sha256'])
pins[str(CP82/'complete.json')]=sha(CP82/'complete.json')
CP80=E/'eig2_persistent_live_cpv80_20261101'
pins.update(read(CP80/'complete.json')['output_sha256'])
pins[str(CP80/'complete.json')]=sha(CP80/'complete.json')
pins.update(read(OLD/'complete.json')['output_sha256'])
pins[str(OLD/'complete.json')]=sha(OLD/'complete.json')
pins[str(OLD/'post_closeout.json')]=sha(OLD/'post_closeout.json')
base=E/'eig2_hesai_refresh_full_cpv49_20261030'
pins.update(read(base/'complete.json')['output_sha256'])
pins[str(base/'complete.json')]=sha(base/'complete.json')
# Retain all pinned runtime DSOs/executables used by the verified live baseline,
# without inheriting thousands of unrelated historical result files.
for p,d in read(S/'protected_manifest.json').items():
    q=Path(p)
    if '.so' in q.name or '/build/' in p or '/devel/' in p or p.endswith('/live'):
        pins[p]=d
for p,d in pins.items():assert sha(p)==d,p
audit=read(P/'input_audit.json')
parent_audit=Path(audit['reused_verified_60s_audit'])
assert read(parent_audit)['window']==audit['window']
pins[str(parent_audit)]=sha(parent_audit)
assert audit['stereo_pairs']==600 and audit['frontend_min']==597 and audit['frontend_max']==600
assert audit['stereo_headers_identical'] and audit['reference_covers_complete_window']
assert abs(audit['window']['duration_s']-60.0)<1e-9 and audit['window']['bag_offset_s']==5.7
for b in audit['bag_files'].values():
    p=Path(b['path']);st=p.stat()
    assert (st.st_size,st.st_mtime_ns)==(b['bytes'],b['mtime_ns'])
    assert sha(p)==b['sha256'],p;pins[str(p)]=b['sha256']
for p in (P/'input_profiles').rglob('*'):
    if p.is_file():assert sha(p)==sha(S/p.relative_to(P)),p
cfgs=[read(p/'full_on.json') for p in (S,P)]
for cfg in cfgs:
    for key in ('sequence','ros_master_uri','receiver_binary'):cfg.pop(key)
    for source in cfg['sources']:source.pop('session')
assert cfgs[0]==cfgs[1],'receiver model/environment changed'
binaries={'libkimera_vio.so':'b7d6829da6c2c5a0edbe3d7092ba109fa29d24a60b2bb5395b05dd629fa1cee8',
          'libcbs.so':'82f85c2a307a9d9a849b2b8f09bb5c1e026ad80a4ef0a633c903d30cca712476'}
for name,d in binaries.items():assert sha(P/'private_build'/name)==d
assert sha(P/'receiver_cpk19.sh')==sha(S/'receiver_cpk19.sh')
assert sha('/home/yeranis/results/gnssfgo_receiver_finalization_cpk19_20261021/live')=='4b08a7f5ba97f5e4ec22959df23367df79eebf5d644dc19d939747dcd4566d6e'
for root in (P,H):
    for p in root.rglob('*'):
        if not p.is_file() or '__pycache__' in p.parts:continue
        if p.suffix=='.py':ast.parse(p.read_text())
        elif p.suffix=='.launch':ET.parse(p)
        elif p.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(p)],check=True)
spec=importlib.util.spec_from_file_location('boundary',P/'eof_boundary.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
assert m.test_boundaries()=={'valid':1,'rejected':4}
spec=importlib.util.spec_from_file_location('wire',P/'transport_tools/live_receiver_protocol.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);m.validate_config(read(P/'full_on.json'))
launch=(P/'active_window_on.launch').read_text();node=(P/'kimera_vio_ros.launch').read_text()
assert '<arg name="cbs_odom_factor_mode" value="persistent"/>' in launch
assert '<env name="CPV76_PERSISTENT_COMPAT" value="1"/>' in node
assert "glim_cbs_mode=inject_persistent" in (P/'source_runner_on.sh').read_text()
assert 'persistent_odom_applied' in (P/'source_runner_on.sh').read_text()
assert 'CBSMS_PRESENTATION_ENABLE=false' in (P/'source.env').read_text()
diff=[]
for p in P.rglob('*'):
    old=S/p.relative_to(P)
    if p.suffix in ('.py','.launch','.sh','.env','.json') and old.is_file() and 'input_profiles' not in p.parts:
        diff.extend(difflib.unified_diff(old.read_text().splitlines(True),p.read_text().splitlines(True),fromfile=str(old),tofile=str(p)))
with (H/'harness.diff').open('x') as f:f.write(''.join(diff))
hesai=call(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'])
save(H/'storage_preflight.json',forecast)
for root in (P,H):
    for p in root.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts and p.name not in ('REPORT.txt','prepare_host.log'):
            pins[str(p)]=sha(p)
save(P/'protected_manifest.json',pins)
receipt=dict(passed=True,prepared=str(P),prepared_at=datetime.datetime.now().astimezone().isoformat(),
    window=audit['window'],protected_files=len(pins),binaries=binaries,receiver_Qc_multiplier=1.,
    profiles_byte_identical_to=str(S/'input_profiles'),receiver_model_unchanged=True,
    kimera_native_only_outgoing_information=False,mixed_prior_feedback_user_accepted=True,
    scope='Single 60s live bidirectional ON with BOTH persistent / concurrent fusion',
    containers=containers,hesai=hesai,storage=forecast)
(P/'preflight').mkdir();save(P/'preflight/validation.json',receipt);save(H/'prepared.json',receipt)
print(json.dumps(receipt,indent=2),flush=True)
