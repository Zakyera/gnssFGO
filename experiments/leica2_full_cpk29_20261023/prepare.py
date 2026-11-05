"""Mechanically rebase CP-K28 only after independent LEICA-2 mission audit."""
from pathlib import Path
import ast,difflib,hashlib,importlib.util,json,re,shutil,socket,subprocess,sys
import xml.etree.ElementTree as ET
sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent
OLD=H.parent/'spx3_stationary_full_cpk28_20261023'
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
S=W/'runs/gnssfgo_spx3_stationary_full_cpk28_20261023/prepared'
P=W/'runs/gnssfgo_leica2_full_cpk29_20261023/prepared'
V=Path('/home/yeranis/results/gnssfgo_leica2_full_cpk29_20261023')
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def rebase(s):
 for a,b in [(S.parent.name,P.parent.name),(OLD.name,H.name),
  ('spx3_cpk28_full','leica2_cpk29_full'),('SPX3-CPK28','LEICA2-CPK29'),
  ('CP-K28','CP-K29'),('SPX-3','LEICA-2'),('2024-11-02-17-43-10','2024-11-25-16-36-19'),
  ('11478','11479'),('197.02','428.75'),('195.02','426.75'),
  ('1730565899.872888651','1732549034.915071665'),('1730565898','1732549033'),
  ('bag29-226.02s','bag6-434.75s')]:s=s.replace(a,b)
 for a,b in [('1950','4267'),('1967','4284'),('1970','4287')]:
  s=re.sub(r'(?<![\w.])'+a+r'(?![\w.])',b,s)
 # Only known crop declarations, never arbitrary profile numbers.
 s=s.replace('CBSMS_BAG_START_SEC=29.0','CBSMS_BAG_START_SEC=6.0')
 s=s.replace('CBSMS_BAG_START_SEC:-29.0','CBSMS_BAG_START_SEC:-6.0')
 s=s.replace('"${bag_start_sec}" == 29.0','"${bag_start_sec}" == 6.0')
 s=s.replace("['CBSMS_BAG_START_SEC']=='29.0'","['CBSMS_BAG_START_SEC']=='6.0'")
 return s
diff=[]
def generate(src,dest,s):
 old=src.read_text()
 with dest.open('x') as f:f.write(s)
 shutil.copymode(src,dest)
 diff.extend(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=str(src),tofile=str(dest)))
assert not P.parent.exists() and not V.exists() and not (H/'status.json').exists()
assert json.loads((OLD/'closeout.json').read_text())['passed']
assert json.loads((OLD/'status.json').read_text())['state']=='complete'
assert shutil.disk_usage(W).free>15*1024**3
with socket.socket() as s:s.bind(('127.0.0.1',11479))
manifest=json.loads((S/'protected_manifest.json').read_text())
for p,want in manifest.items():assert sha(p)==want,p
pre=json.loads((H/'bag_preflight.json').read_text())
assert pre['passed'] and pre['calibration_matches_frozen_profiles']
assert pre['stereo_pairs']==4287 and pre['window']['duration_s']==428.75
assert pre['window']['start_ns']==1732549034915071665
assert pre['window']['end_ns']==1732549463665071665
for b in pre['bag_files'].values():
 st=Path(b['path']).stat();assert (st.st_size,st.st_mtime_ns)==(b['bytes'],b['mtime_ns']),b['path']
P.mkdir(parents=True)
for folder in ('private_build','input_profiles','transport_tools'):
 shutil.copytree(S/folder,P/folder,symlinks=True,ignore=shutil.ignore_patterns('__pycache__'))
files=['coverage_checks.py','evaluate_fl4f.py','experiment.py','expiry_audit.py','eof_boundary.py',
 'full_off.json','full_on.json','active_window.launch','active_window_on.launch',
 'glim_profile.launch','glim_profile_on.launch','grandtour_eth1_kimera_stereo.launch',
 'kimera_vio_ros.launch','kimera_vio_ros_newer_college_2021_stereo.launch','map_guard.py',
 'preflight_on.py','receiver_cpk19.sh','run_case.py','run_case_on.py','source.env',
 'source_runner.sh','source_runner_on.sh','validate_candidate.py','validate_case.py',
 'validate_local.py','verify_effective.py','verify_effective_on.py']
for name in files:generate(S/name,P/name,rebase((S/name).read_text()))
shutil.copy2(H/'PROTOCOL.txt',P/'PROTOCOL.txt')
for path in (P/'transport_tools').rglob('*.py'):
 old=path.read_text();new=rebase(old)
 if old!=new:
  path.write_text(new)
  diff.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=str(S/path.relative_to(P)),tofile=str(path)))
for name in ('run_pair.py','summarize.py'):
 s=rebase((OLD/name).read_text())
 if name=='summarize.py':
  assert "('last20s',177.02)" in s
  s=s.replace("('last20s',177.02)","('last20s',408.75)")
 generate(OLD/name,H/name,s)
with (H/'harness.diff').open('x') as f:f.write(''.join(diff))
for folder in ('private_build','input_profiles'):
 for p in (S/folder).rglob('*'):
  if p.is_file():assert sha(p)==sha(P/p.relative_to(S)),p
for name in ('expiry_audit.py','eof_boundary.py','map_guard.py'):assert sha(S/name)==sha(P/name)
for mode in ('off','on'):
 old=json.loads((S/f'full_{mode}.json').read_text());new=json.loads((P/f'full_{mode}.json').read_text())
 for c in (old,new):
  for k in ('sequence','ros_master_uri','receiver_binary'):c.pop(k)
  for source in c['sources']:source.pop('session')
 assert old==new,'Receiver model settings changed'
for p in [*P.rglob('*'),*H.glob('*.py')]:
 if p.suffix=='.py':ast.parse(p.read_text())
 elif p.suffix=='.launch':ET.parse(p)
 elif p.suffix in ('.env','.sh'):subprocess.run(['bash','-n',str(p)],check=True)
spec=importlib.util.spec_from_file_location('boundary',P/'eof_boundary.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
assert m.test_boundaries()=={'valid':1,'rejected':4}
for base in (P,H):
 for p in base.rglob('*'):
  if p.is_file() and '__pycache__' not in p.parts and p.name!='REPORT.txt':manifest[str(p)]=sha(p)
with (P/'protected_manifest.json').open('x') as f:json.dump(manifest,f,indent=2)
pre.update(template=str(S),protected_files=len(manifest),model_identical=True,
 source_profiles_byte_identical=True,independently_validated_mission_calibration=True,
 change='New mission/window only; frozen method; stationary bag6s start through lastHesai',
 eof_contract='finite_recording_eof_cpk24a',disk_free_bytes=shutil.disk_usage(W).free)
(P/'preflight').mkdir()
with (P/'preflight/validation.json').open('x') as f:json.dump(pre,f,indent=2)
print(json.dumps(dict(passed=True,prepared=str(P),protected_files=len(manifest),window=pre['window'],disk_free_gib=shutil.disk_usage(W).free/1024**3),indent=2))
