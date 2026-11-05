"""Fresh full matched harness; mechanical CP-V41 backend changes, no tuning."""
import ast,difflib,hashlib,importlib.util,json,shutil,socket,subprocess,sys
from pathlib import Path
import xml.etree.ElementTree as ET
sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent;R=H.parents[1];W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
S=W/'runs/gnssfgo_eig2_velodyne_full_cpv4_20261024/prepared'
P=W/'runs'/H.name/'prepared';V=Path('/home/yeranis/results')/H.name
H41=R/'experiments/eig2_on_refresh_smoke_cpv41_20261029'
P41=W/'runs'/H41.name/'prepared'
K=W/'runs/kimera_on_incremental_refresh_cpv40_20261029';G=R/'experiments'/K.name
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
assert not P.exists() and not V.exists() and not (H/'prepared.json').exists()
for marker in (H41/'complete.json',H41/'closeout.json',G/'closeout.json',
               R/'experiments/eig2_off_refresh_full_cpv39_20261029/closeout.json'):
    assert json.loads(marker.read_text())['passed'],marker
want=json.loads((G/'closeout.json').read_text())['backend_sha256']
assert want=='750b8c8ffed61d186ec8f071d4f58f068c9b63baccefcc9fc37803d77a118f06'
assert sha(K/'private_build/libkimera_vio.so')==want
with socket.socket() as sock:sock.bind(('127.0.0.1',11542))
forecast=json.loads((H41/'full_pair_storage_forecast.json').read_text())
free=shutil.disk_usage(W).free
forecast.update(current_free_bytes=free,passed=free>forecast['required_free_bytes'],
 additional_bytes_required=max(0,forecast['required_free_bytes']-free),
 predicted_free_before_ON_without_growth_margin=free-forecast['original_mode_totals']['off'],
 decision='Fresh full pair admission only; previous CP-V41 forecast remains untouched.')
save(H/'storage_preflight.json',forecast);assert forecast['passed']
pins=json.loads((P41/'protected_manifest.json').read_text())
for p,want_pin in pins.items():assert sha(p)==want_pin,p
audit=json.loads((S/'input_audit.json').read_text())
assert audit['window']['duration_s']==355.02 and audit['window']['bag_offset_s']==5.7
for item in audit['bag_files'].values():
    bag=Path(item['path']);st=bag.stat()
    assert (st.st_size,st.st_mtime_ns)==(item['bytes'],item['mtime_ns'])
    assert sha(bag)==item['sha256'],bag;pins[str(bag)]=item['sha256']
P.mkdir(parents=True)
for folder in ('input_profiles','transport_tools'):
    shutil.copytree(S/folder,P/folder,ignore=shutil.ignore_patterns('__pycache__'))
shutil.copytree(K/'private_build',P/'private_build')
diff=[]
def rebase(s):
    return s.replace(S.parent.name,H.name).replace('11482','11542').replace('eig2_cpv4_velodyne','eig2_cpv42_velodyne')
files=['coverage_checks.py','experiment.py','expiry_audit.py','eof_boundary.py',
 'full_off.json','full_on.json','active_window.launch','active_window_on.launch',
 'glim_profile.launch','glim_profile_on.launch','grandtour_eth1_kimera_stereo.launch',
 'kimera_vio_ros.launch','kimera_vio_ros_newer_college_2021_stereo.launch','map_guard.py',
 'receiver_cpk19.sh','run_case.py','run_case_on.py','source.env','source_runner.sh',
 'source_runner_on.sh','validate_candidate.py','validate_case.py','validate_local.py',
 'verify_effective.py','verify_effective_on.py','validate_velodyne.py','verify_velodyne_startup.py','preflight_on.py']
for name in files:
    old=(S/name).read_text();s=rebase(old)
    if name in ('full_off.json','full_on.json'):
        value=json.loads(s);value['sequence']=f'EIG-2 CP-V42 Velodyne full355.02s matched corrected Kimera {name[5:-5]}; original offset5.7s'
        s=json.dumps(value,indent=2)+'\n'
    if name=='kimera_vio_ros.launch':
        needle='<env name="CPK9_PRIOR_REFRESH" value="1"/>';assert s.count(needle)==1
        s=s.replace(needle,needle+'\n    <env name="CPV40_REFRESH" value="1"/>\n    <env name="CPV38_OFF_REFRESH" value="0"/>')
    if name in ('verify_effective.py','verify_effective_on.py','validate_candidate.py'):
        oldsha='be637ad745f73bf12596d3bf06754f56ed7bcadfdb8b4792b94c4fcfee62aad9'
        assert s.count(oldsha)==1;s=s.replace(oldsha,want)
    if name in ('verify_effective.py','verify_effective_on.py'):
        needle="assert env(pid)['CPK9_PRIOR_REFRESH']=='1'";assert s.count(needle)==1
        s=s.replace(needle,needle+"\nassert env(pid)['CPV40_REFRESH']=='1' and env(pid)['CPV38_OFF_REFRESH']=='0'\nassert 'CPV40_REFRESH' not in env(glim_pid)")
        s=s.replace("('CPK9_PRIOR_REFRESH','FL4Q_ENABLE','FL4M_RETIREMENT')","('CPK9_PRIOR_REFRESH','FL4Q_ENABLE','FL4M_RETIREMENT','CPV40_REFRESH','CPV38_OFF_REFRESH')")
    if name=='validate_candidate.py':
        needle='"CPK9_PRIOR_REFRESH": "1", "FL4Q_ENABLE": "0", "FL4M_RETIREMENT": "remove"}'
        assert needle in s;s=s.replace(needle,needle[:-1]+', "CPV40_REFRESH": "1", "CPV38_OFF_REFRESH": "0"}')
        s=s.replace('("FL4Q", "CPK9 ",','("FL4Q", "CPK9 ", "CPV40 ",')
    if name in ('run_case.py','run_case_on.py'):
        assert "'CPK9 ','FL4Q'" in s;s=s.replace("'CPK9 ','FL4Q'","'CPK9 ','CPV40 ','FL4Q'")
    if name=='preflight_on.py':
        s=s.replace('same30.0s OFF passes','same355.02s OFF passes')
    with (P/name).open('x') as f:f.write(s)
    shutil.copymode(S/name,P/name)
    diff.extend(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=str(S/name),tofile=str(P/name)))
for dst in (P/'transport_tools').rglob('*.py'):
    old=dst.read_text();new=rebase(old)
    if old!=new:
        dst.write_text(new)
        diff.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(S/dst.relative_to(P)),tofile=str(dst)))
shutil.copy2(S/'input_audit.json',P/'input_audit.json')
shutil.copy2(H/'PROTOCOL.txt',P/'PROTOCOL.txt')
for p in (S/'input_profiles').rglob('*'):
    if p.is_file():assert sha(p)==sha(P/p.relative_to(S)),p
configs=[]
for mode in ('off','on'):
    a=json.loads((S/f'full_{mode}.json').read_text());b=json.loads((P/f'full_{mode}.json').read_text())
    for c in (a,b):
        for key in ('sequence','ros_master_uri','receiver_binary'):c.pop(key)
        for source in c['sources']:source.pop('session')
    assert a==b,'receiver numerical settings changed';configs.append(b)
assert configs[0]==configs[1]
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
for name in ('prepare.py','controller.py','launch.py','PROTOCOL.txt'):
    pins[str(H/name)]=sha(H/name)
save(P/'protected_manifest.json',pins);(P/'preflight').mkdir()
receipt=dict(passed=True,scope='ONE full355.02s matched corrected OFF then ON; no retries',prepared=str(P),
    window=audit['window'],kimera_backend=want,profiles_unchanged=True,receiver_model_unchanged=True,
    protected_files=len(pins),storage=forecast,free_bytes=shutil.disk_usage(W).free,
    note='ON preflight deliberately absent until this fresh OFF passes every gate.')
save(P/'preflight/validation.json',receipt);save(H/'prepared.json',receipt)
print(json.dumps(receipt,indent=2))
