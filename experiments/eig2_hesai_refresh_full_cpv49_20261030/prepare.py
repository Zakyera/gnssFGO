"""CP-V49: fresh corrected Hesai pair, frozen CP-K24 profile + CP-V42 harness."""
import ast,difflib,hashlib,importlib.util,json,shutil,socket,subprocess,sys
from pathlib import Path
import xml.etree.ElementTree as ET
sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent;R=H.parents[1];W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
S=W/'runs/eig2_refresh_full_cpv42_20261029/prepared'
B=W/'runs/gnssfgo_eig2_full_cpk24_20261021/prepared'
P=W/'runs'/H.name/'prepared';V=Path('/home/yeranis/results')/H.name
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
assert not P.exists() and not V.exists() and not (H/'prepared.json').exists()
assert json.loads((R/'experiments/eig2_refresh_full_cpv42_20261029/complete.json').read_text())['passed']
with socket.socket() as sock:sock.bind(('127.0.0.1',11549))
totals={}
for mode in ('off','on'):
    paths=[B/f'full_{mode}_{part}' for part in ('source','receiver','control')]
    totals[mode]=sum(int(subprocess.check_output(['du','-sB1',str(p)],text=True).split()[0]) for p in paths)
budgets={k:int(v*1.25) for k,v in totals.items()}
required=max(budgets['off']+8*1024**3,sum(budgets.values())+5*1024**3)+512*1024**2
free=shutil.disk_usage(W).free
forecast=dict(passed=free>required,current_free_bytes=free,required_free_bytes=required,
 original_mode_totals=totals,forecast_mode_budgets_25_percent_margin=budgets,
 launch_reserve_bytes=8*1024**3,runtime_reserve_bytes=5*1024**3,
 preparation_and_report_reserve_bytes=512*1024**2,source=str(B))
save(H/'storage_preflight.json',forecast);assert forecast['passed']
pins=json.loads((R/'experiments/eig2_rotation_seed_cpv48_20261030/protected_manifest.json').read_text())
pins.update(json.loads((S/'protected_manifest.json').read_text()))
for p,want in pins.items():assert sha(p)==want,p
audit=json.loads((R/'experiments/eig2_full_cpk24_20261021/bag_preflight.json').read_text())
assert abs(audit['window']['duration_s']-355.02)<1e-10 and audit['window']['bag_offset_s']==5.7
for item in audit['bag_files'].values():
    bag=Path(item['path']);st=bag.stat()
    assert (st.st_size,st.st_mtime_ns)==(item['bytes'],item['mtime_ns'])
    assert sha(bag)==item['sha256'],bag;pins[str(bag)]=item['sha256']
P.mkdir(parents=True)
shutil.copytree(B/'input_profiles',P/'input_profiles')
for folder in ('private_build','transport_tools'):
    shutil.copytree(S/folder,P/folder,ignore=shutil.ignore_patterns('__pycache__'))
diff=[]
def rebase(s):
    for old,new in [(S.parent.name,H.name),('11542','11549'),('/anymal/velodyne/points','/boxi/hesai/points'),
                    ('_anymal_velodyne.bag','_hesai.bag'),('velodyne','hesai'),('Velodyne','Hesai'),
                    ('cpv42','cpv49'),('CP-V42','CP-V49'),('20261024','20261030')]:
        s=s.replace(old,new)
    return s
files=['coverage_checks.py','experiment.py','expiry_audit.py','eof_boundary.py','full_off.json','full_on.json',
 'active_window.launch','active_window_on.launch','glim_profile.launch','glim_profile_on.launch',
 'grandtour_eth1_kimera_stereo.launch','kimera_vio_ros.launch','kimera_vio_ros_newer_college_2021_stereo.launch',
 'map_guard.py','receiver_cpk19.sh','run_case.py','run_case_on.py','source.env','source_runner.sh',
 'source_runner_on.sh','validate_candidate.py','validate_case.py','validate_local.py',
 'verify_effective.py','verify_effective_on.py','preflight_on.py']
for name in files:
    old=(S/name).read_text();s=rebase(old)
    if name.startswith('source_runner'):s=s.replace('min_glim_rows=3530','min_glim_rows=3517')
    if name=='validate_case.py':s=s.replace("['rows']>=3530","['rows']>=3517")
    with (P/name).open('x') as f:f.write(s)
    shutil.copymode(S/name,P/name)
    diff.extend(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=str(S/name),tofile=str(P/name)))
for dst in (P/'transport_tools').rglob('*.py'):
    old=dst.read_text();new=rebase(old)
    if old!=new:
        dst.write_text(new)
        diff.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(S/dst.relative_to(P)),tofile=str(dst)))
for name in ('verify_hesai_startup.py','validate_hesai.py'):
    shutil.copy2(H/name,P/name)
save(P/'input_audit.json',audit);shutil.copy2(H/'PROTOCOL.txt',P/'PROTOCOL.txt')
for p in (B/'input_profiles').rglob('*'):
    if p.is_file():assert sha(p)==sha(P/p.relative_to(B)),p
for p in (B/'input_profiles/kimera_profile').rglob('*'):
    if p.is_file():assert sha(p)==sha(S/p.relative_to(B)),p
configs=[]
for mode in ('off','on'):
    a=json.loads((B/f'full_{mode}.json').read_text());b=json.loads((P/f'full_{mode}.json').read_text())
    for c in (a,b):
        for key in ('sequence','ros_master_uri','receiver_binary'):c.pop(key)
        for source in c['sources']:source.pop('session')
    assert a==b,'receiver model changed';configs.append(b)
assert configs[0]==configs[1]
assert sha(P/'private_build/libkimera_vio.so')=='750b8c8ffed61d186ec8f071d4f58f068c9b63baccefcc9fc37803d77a118f06'
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
save(P/'protected_manifest.json',pins);(P/'preflight').mkdir()
receipt=dict(passed=True,prepared=str(P),window=audit['window'],kimera_backend_sha256=sha(P/'private_build/libkimera_vio.so'),
 profiles_byte_identical_to=str(B/'input_profiles'),camera_imu_profiles_identical_to_CPV42=True,
 receiver_model_unchanged=True,protected_files=len(pins),storage=forecast)
save(P/'preflight/validation.json',receipt);save(H/'prepared.json',receipt)
print(json.dumps(receipt,indent=2))
