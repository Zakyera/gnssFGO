"""Fresh60s OFF from validated CP-R2 harness; match frozen CP-R3 ON inputs."""
from pathlib import Path
import ast,difflib,hashlib,json,shutil,subprocess
import xml.etree.ElementTree as ET
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
T=W/'runs/gnssfgo_eig2_local_cov30_cpr2_20261019/prepared'
O=W/'runs/gnssfgo_eig2_local_cov60_on_cpr3_20261019/prepared'
R=W/'runs/gnssfgo_eig2_local_cov60_off_cpr4_20261019/prepared'
ON=H.parent/'local_covariance_on60_cpr3_20261019'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def once(s,a,b):
 assert s.count(a)==1,(a,s.count(a))
 return s.replace(a,b)
assert not R.parent.exists(),'refuse duplicate preparation'
assert shutil.disk_usage(W).free>8*1024**3
assert json.loads((ON/'final_checks.json').read_text())['passed']
manifest=json.loads((O/'protected_manifest.json').read_text())
for p,h in manifest.items():assert sha(p)==h,p
R.mkdir(parents=True)
shutil.copytree(O/'private_build',R/'private_build',symlinks=True)
shutil.copytree(O/'input_profiles',R/'input_profiles')
def common(s):
 return s.replace(T.parent.name,R.parent.name).replace('eig2_cpr2_live30_off_20261019','eig2_cpr4_live60_off_20261019').replace('live30','live60').replace('11448','11450')
files=['experiment.py','coverage_checks.py','validate_case.py','validate_candidate.py',
 'map_guard.py','source.env','receiver_fl4e.sh','run_case.py','verify_effective.py',
 'source_runner.sh','active_window.launch','glim_profile.launch',
 'grandtour_eth1_kimera_stereo.launch','kimera_vio_ros_newer_college_2021_stereo.launch',
 'kimera_vio_ros.launch','live30_off.json','validate_local.py']
diff=[]
for name in files:
 old=(T/name).read_text();s=common(old)
 if name=='source.env':s=once(s,'CBSMS_BAG_DURATION_SEC:-30.0','CBSMS_BAG_DURATION_SEC:-60.0')
 if name=='run_case.py':s=once(s,'duration=30','duration=60')
 if name=='source_runner.sh':
  s=once(s,'"${duration_sec}" == 30.0','"${duration_sec}" == 60.0').replace('30-second OFF','60-second OFF')
  s=once(s,'min_glim_rows=280','min_glim_rows=570');s=once(s,'min_kimera_rows=135','min_kimera_rows=270')
 if name=='verify_effective.py':s=once(s,"== '30.0'","== '60.0'")
 if name=='validate_case.py':
  s=once(s,"source_cov['requested_s']==30","source_cov['requested_s']==60")
  s=once(s,"['rows']>=280","['rows']>=570");s=once(s,"['rows']>=135","['rows']>=270")
  s=once(s,'*1e-9>=28.','*1e-9>=58.')
 if name=='validate_candidate.py':s=once(s,'len(a)>=135','len(a)>=270')
 if name=='validate_local.py':s=once(s,'first>=280','first>=570')
 if name.endswith('.json'):
  obj=json.loads(s);obj['sequence']='EIG-2 LOCAL CP-R4 bag5.7-65.7s matched OFF';s=json.dumps(obj,indent=2)+'\n'
 dest=R/name.replace('live30','live60');dest.write_text(s);shutil.copymode(T/name,dest)
 diff.extend(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile=name+' CP-R2',tofile=dest.name+' CP-R4'))
shutil.copytree(T/'transport_tools',R/'transport_tools',ignore=shutil.ignore_patterns('__pycache__'))
for p in (R/'transport_tools').rglob('*.py'):p.write_text(common(p.read_text()))
shutil.copy2(H/'PROTOCOL.txt',R/'PROTOCOL.txt')
# Evaluation reads old ON paths without copying or changing any ON artifact.
old=(O/'evaluate_fl4f.py').read_text();s=old.replace("for mode in ('on',)","for mode in ('off','on')")
s=once(s,"OUT = ROOT / 'evaluation_full'","OUT = ROOT / 'evaluation_full'\nON_ROOT = Path('/workspace/cbs_gtsam4.3/runs/gnssfgo_eig2_local_cov60_on_cpr3_20261019/prepared')\ndef source_root(mode):\n    return ON_ROOT if mode == 'on' else ROOT")
s=s.replace("ROOT / f'live60_{mode}","source_root(mode) / f'live60_{mode}").replace("ROOT/f'live60_{mode}","source_root(mode)/f'live60_{mode}")
s=s.replace('Run after the standalone ON case validates','Run after matching OFF validates; read already-validated ON')
(R/'evaluate_fl4f.py').write_text(s)
diff.extend(difflib.unified_diff(old.splitlines(True),s.splitlines(True),fromfile='evaluator CP-R3',tofile='evaluator CP-R4'))
# Mathematical scoring helpers are unchanged, only source selection is extended.
def functions(code):
 return {x.name:ast.dump(x,include_attributes=False) for x in ast.parse(code).body if isinstance(x,ast.FunctionDef)}
for name in ('sha','stat','prism_pose','path_length','score','score_saved'):
 assert functions(old)[name]==functions(s)[name],name
assert sha(R/'receiver_fl4e.sh')==sha(O/'receiver_fl4e.sh')
def normalize_config(p):
 obj=json.loads(p.read_text())
 for k in ('sequence','ros_master_uri','receiver_binary'):obj.pop(k)
 for source in obj['sources']:source.pop('session')
 return obj
assert normalize_config(R/'live60_off.json')==normalize_config(O/'live60_on.json')
def env_normalize(p):
 return p.read_text().replace(str(R.parent.name),'RUN').replace(O.parent.name,'RUN').replace('11450','PORT').replace('11449','PORT')
assert env_normalize(R/'source.env')==env_normalize(O/'source.env')
for name in ('input_profiles','private_build'):
 for p in (R/name).rglob('*'):
  if p.is_file():assert sha(p)==sha(O/p.relative_to(R)),p
for p in R.rglob('*'):
 if p.suffix=='.py':ast.parse(p.read_text())
 if p.suffix=='.launch':ET.parse(p)
 if p.suffix in ('.sh','.env'):subprocess.run(['bash','-n',str(p)],check=True)
for p in [ON/'final_checks.json',ON/'results.json',ON/'REPORT.txt',O/'protected_manifest.json',O/'evaluation_full/summary.json',
 O/'live60_on_control/complete.json',O/'live60_on_source/odom_imu.txt',O/'live60_on_source/kimera_odometry.csv',
 O/'live60_on_receiver/receiver/online_trajectory.tum',O/'live60_on_receiver/receiver/finalized_trajectory.tum',O/'live60_on_receiver/events.csv']:
 manifest[str(p)]=sha(p)
for p in R.rglob('*'):
 if p.is_file():manifest[str(p)]=sha(p)
for name in ('prepare.py','PROTOCOL.txt'):manifest[str(H/name)]=sha(H/name)
(R/'protected_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
(R/'preflight').mkdir()
result=json.loads((O/'preflight/validation.json').read_text())
result.update(passed=True,template=str(T),matched_on=str(O),requested_s=60,bag_start_s=5.7,
 protected_files=len(manifest),free_bytes=shutil.disk_usage(W).free,
 change='CP-R2 OFF expanded to same60s as CP-R3 ON; identical model, builds and profiles',
 matching=dict(receiver_config=True,source_environment=True,profiles_and_private_build=True,scoring_helpers=True))
(R/'preflight/validation.json').write_text(json.dumps(result,indent=2)+'\n')
(H/'harness.diff').write_text(''.join(diff))
print(json.dumps(result,indent=2))
