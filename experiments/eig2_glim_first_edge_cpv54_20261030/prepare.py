"""Fresh unchanged-settings CP-V51 copy, only bounded GLIM observer override."""
import ast,difflib,hashlib,importlib.util,json,shutil,socket,subprocess,sys
from pathlib import Path
import xml.etree.ElementTree as ET
sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');K=W/'runs'/H.name;P=K/'prepared'
S=W/'runs/eig2_hesai_insertion_cpv51_20261030/prepared';V=Path('/home/yeranis/results')/H.name
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
 with p.open('x') as f:json.dump(v,f,indent=2)
assert not P.exists() and not V.exists()
gate=json.loads((H/'build_passed.json').read_text());assert gate['passed']
with socket.socket() as sock:sock.bind(('127.0.0.1',11554))
pins=json.loads((H/'protected_before.json').read_text())
for p,w in pins.items():assert sha(p)==w,p
forecast=dict(passed=shutil.disk_usage(W).free>8*1024**3,current_free_bytes=shutil.disk_usage(W).free,required_free_bytes=8*1024**3,runtime_reserve_bytes=5*1024**3)
save(H/'storage_preflight.json',forecast);assert forecast['passed']
P.mkdir();diff=[]
for sub in ('input_profiles','transport_tools','private_build'):
 shutil.copytree(S/sub,P/sub,ignore=shutil.ignore_patterns('__pycache__'))
def rebase(s):return s.replace(S.parent.name,H.name).replace('11551','11554').replace('cpv51','cpv54')
oldbridge='/workspace/cbs_gtsam4.3/runs/local_revision_covariance_cpi1_20261019'
newbridge='/workspace/cbs_gtsam4.3/runs/'+H.name
files=['coverage_checks.py','experiment.py','expiry_audit.py','eof_boundary.py','full_off.json','full_on.json',
 'active_window.launch','active_window_on.launch','glim_profile.launch','glim_profile_on.launch',
 'grandtour_eth1_kimera_stereo.launch','kimera_vio_ros.launch','kimera_vio_ros_newer_college_2021_stereo.launch',
 'map_guard.py','receiver_cpk19.sh','run_case.py','run_case_on.py','source.env','source_runner.sh',
 'source_runner_on.sh','validate_candidate.py','validate_case.py','validate_local.py',
 'verify_effective.py','verify_effective_on.py','preflight_on.py','verify_hesai_startup.py','validate_hesai.py']
for name in files:
 old=(S/name).read_text();s=rebase(old)
 if name in ('glim_profile.launch','glim_profile_on.launch','map_guard.py','verify_effective.py','verify_effective_on.py'):
  assert oldbridge in s;s=s.replace(oldbridge,newbridge)
 if name in ('glim_profile.launch','glim_profile_on.launch'):
  marker='<env name="LD_LIBRARY_PATH"';assert s.count(marker)==1
  s=s.replace(marker,'<env name="CPV54_GLIM_AUDIT_DIR" value="$(env ROS_LOG_DIR)/../glim_observer"/>\n\t\t'+marker)
 if name.startswith('verify_effective'):
  s=s.replace('42df73c172232a612da965bf43ead8ec454caf08434b84e2da5de74d26cdb79f',gate['bridge_sha256'])
  marker="assert 'CPV51_AUDIT_DIR' not in env(glim_pid)"
  assert marker in s
  s=s.replace(marker,marker+"\nassert Path(env(glim_pid)['CPV54_GLIM_AUDIT_DIR']).resolve()==source/'glim_observer'\nassert 'CPV54_GLIM_AUDIT_DIR' not in env(pid)")
 with (P/name).open('x') as f:f.write(s)
 shutil.copymode(S/name,P/name)
 diff.extend(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=str(S/name),tofile=str(P/name)))
for p in (P/'transport_tools').rglob('*.py'):
 old=p.read_text();new=rebase(old)
 if old!=new:p.write_text(new)
audit=json.loads((S/'input_audit.json').read_text());audit['parent_audit']=str(S/'input_audit.json')
for item in audit['bag_files'].values():
 p=Path(item['path']);st=p.stat();assert (st.st_size,st.st_mtime_ns)==(item['bytes'],item['mtime_ns']) and sha(p)==item['sha256']
save(P/'input_audit.json',audit);shutil.copy2(H/'PROTOCOL.txt',P/'PROTOCOL.txt')
for p in (S/'input_profiles').rglob('*'):
 if p.is_file():assert sha(p)==sha(P/p.relative_to(S))
for mode in ('off','on'):
 a=json.loads((S/f'full_{mode}.json').read_text());b=json.loads((P/f'full_{mode}.json').read_text())
 for c in (a,b):
  for key in ('sequence','ros_master_uri','receiver_binary'):c.pop(key)
  for source in c['sources']:source.pop('session')
 assert a==b,'receiver model changed'
assert sha(P/'private_build/libkimera_vio.so')==sha(S/'private_build/libkimera_vio.so')
for root in (P,H):
 for p in root.rglob('*'):
  if not p.is_file():continue
  if p.suffix=='.py':ast.parse(p.read_text())
  elif p.suffix=='.launch':ET.parse(p)
  elif p.suffix in ('.env','.sh'):subprocess.run(['bash','-n',str(p)],check=True)
spec=importlib.util.spec_from_file_location('boundary',P/'eof_boundary.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
assert m.test_boundaries()=={'valid':1,'rejected':4}
with (H/'harness.diff').open('x') as f:f.write(''.join(diff))
for p in K.rglob('*'):
 if p.is_file() and '__pycache__' not in p.parts:pins[str(p)]=sha(p)
for p in H.iterdir():
 if p.suffix=='.py' or p.name=='PROTOCOL.txt':pins[str(p)]=sha(p)
save(P/'protected_manifest.json',pins);(P/'preflight').mkdir()
receipt=dict(passed=True,prepared=str(P),validation=str(V),window=audit['window'],bridge_sha256=gate['bridge_sha256'],profiles_byte_identical_to=str(S/'input_profiles'),receiver_model_unchanged=True,kimera_backend_unchanged=True,protected_files=len(pins),glim_observer_updates=[0,10])
save(P/'preflight/validation.json',receipt);save(H/'prepared.json',receipt);print(json.dumps(receipt,indent=2))
