"""Copy frozen60s ON harness with CP-R2's tested cache/validator. No source edits."""
from pathlib import Path
import ast,difflib,hashlib,json,shutil,subprocess
import xml.etree.ElementTree as ET
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
T=W/'runs/gnssfgo_eig2_local_cov60_cpr1_20261019/prepared'
Q=W/'runs/gnssfgo_eig2_local_cov30_cpr2_20261019/prepared'
R=W/'runs/gnssfgo_eig2_local_cov60_on_cpr3_20261019/prepared'
CS='/workspace/cbs_gtsam4.3/runs/native_covariance_cache_cpi3_20261019/release_v2'
BRIDGE='/workspace/cbs_gtsam4.3/runs/local_revision_covariance_cpi1_20261019'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def once(s,a,b):
 assert s.count(a)==1,(a,s.count(a))
 return s.replace(a,b)
assert not R.parent.exists(),'refuse duplicate preparation'
assert shutil.disk_usage(W).free>8*1024**3
manifest=json.loads((Q/'protected_manifest.json').read_text())
for p,h in manifest.items():assert sha(p)==h,p
prior=H.parent/'local_covariance_live_cpr2_20261019'
assert json.loads((prior/'final_checks.json').read_text())['passed']
assert json.loads((prior/'results.json').read_text())['passed']
assert json.loads((Q/'live30_off_source/local_provenance.json').read_text())['passed']
R.mkdir(parents=True)
shutil.copytree(Q/'private_build',R/'private_build',symlinks=True)
shutil.copytree(Q/'input_profiles',R/'input_profiles')
def common(s):
 return s.replace(T.parent.name,R.parent.name).replace(Q.parent.name,R.parent.name).replace('eig2_cpr1_live60_on_20261017','eig2_cpr3_live60_on_20261019').replace('11447','11449').replace('11448','11449')
files=['experiment.py','coverage_checks.py','validate_case.py','validate_candidate.py',
 'source.env','receiver_fl4e.sh','run_case_on.py','verify_effective_on.py',
 'source_runner_on.sh','active_window_on.launch','glim_profile_on.launch',
 'grandtour_eth1_kimera_stereo.launch','kimera_vio_ros_newer_college_2021_stereo.launch',
 'kimera_vio_ros.launch','live60_on.json','evaluate_fl4f.py','map_guard.py','validate_local.py']
diff=[]
for name in files:
 template=Q if name in ('map_guard.py','validate_local.py') else T
 old=(template/name).read_text();s=common(old)
 if name=='glim_profile_on.launch':s=once(s,'value="'+BRIDGE+':','value="'+CS+':'+BRIDGE+':')
 if name=='verify_effective_on.py':
  s=once(s,"assert 'GLIM_SHARED_NATIVE_JOURNAL' not in env(glim_pid)","assert hashlib.sha256(Path('"+CS+"/libgtsam_points.so.1.2.1').read_bytes()).hexdigest()=='3f23e6115e34acc5a2f2b965b15bd4c651897c889a6c38a0a52cdb3855c0ceb7'\nassert 'GLIM_SHARED_NATIVE_JOURNAL' not in env(glim_pid)")
  s=once(s,"glim_revision_covariance='native_local'","glim_revision_covariance='native_local',cached_provider_sha256='3f23e6115e34acc5a2f2b965b15bd4c651897c889a6c38a0a52cdb3855c0ceb7'")
 if name=='validate_case.py':s=once(s,"assert MODE in ('off','on')","assert MODE == 'on'")
 if name=='validate_local.py':
  s=s.replace('live30','live60');s=once(s,"assert mode == 'off'","assert mode == 'on'");s=once(s,'first>=280','first>=570')
 if name=='evaluate_fl4f.py':
  s=s.replace("for mode in ('off','on')","for mode in ('on',)")
  s=once(s,'Run after both live cases validate','Run after the standalone ON case validates')
 if name.endswith('.json'):
  obj=json.loads(s);obj['sequence']='EIG-2 LOCAL CP-R3 bag5.7-65.7s standalone ON';s=json.dumps(obj,indent=2)+'\n'
 dest=R/name;dest.write_text(s);shutil.copymode(template/name,dest)
 diff.extend(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=str(template/name),tofile=name+' CP-R3'))
shutil.copytree(Q/'transport_tools',R/'transport_tools',ignore=shutil.ignore_patterns('__pycache__'))
for p in (R/'transport_tools').rglob('*.py'):p.write_text(common(p.read_text()))
for name in ('PROTOCOL.txt','preflight_on.py'):shutil.copy2(H/name,R/name)
assert sha(R/'receiver_fl4e.sh')==sha(Q/'receiver_fl4e.sh')
old=json.loads((Q/'live30_off.json').read_text());new=json.loads((R/'live60_on.json').read_text())
for obj in (old,new):
 for k in ('sequence','ros_master_uri','receiver_binary'):obj.pop(k)
 for source in obj['sources']:source.pop('session')
assert old==new
for p in R.rglob('*'):
 if p.suffix=='.py':ast.parse(p.read_text())
 if p.suffix=='.launch':ET.parse(p)
 if p.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(p)],check=True)
for p in [prior/'final_checks.json',prior/'results.json',Q/'protected_manifest.json',Q/'live30_off_source/local_provenance.json']:
 manifest[str(p)]=sha(p)
for p in R.rglob('*'):
 if p.is_file():manifest[str(p)]=sha(p)
for name in ('prepare.py','PROTOCOL.txt','preflight_on.py'):manifest[str(H/name)]=sha(H/name)
(R/'protected_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
(R/'preflight').mkdir()
result=json.loads((Q/'preflight/validation.json').read_text())
result.update(passed=True,template=str(T),requested_s=60,bag_start_s=5.7,
 protected_files=len(manifest),free_bytes=shutil.disk_usage(W).free,
 change='Explicit standalone60s ON; CP-R2 cache and unchanged source/receiver mathematics',
 input_validation='Frozen EIG-2 full60s bags/profiles hash-verified; CP-R2 short OFF gate passed')
(R/'preflight/validation.json').write_text(json.dumps(result,indent=2)+'\n')
(H/'harness.diff').write_text(''.join(diff))
print(json.dumps(dict(prepared=str(R),protected=len(manifest),free_bytes=result['free_bytes']),indent=2))
